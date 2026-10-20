// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM8953/SDM632 SLIMbus NGD (Non-ported Generic Device) controller
 *
 * The application processor acts as a SLIMbus satellite; the ADSP owns the
 * bus as manager and framer. The satellite talks to the ADSP through the
 * NGD message queues (register-based RX, BAM or register-based TX) and a
 * QMI power/instance service, and translates core channel management
 * messages into the Qualcomm user-defined satellite protocol.
 *
 * Power-up sequence: QMI PM_ACTIVE, NGD setup, wait for MASTER_CAPABILITY,
 * send REPORT_SATELLITE.
 *
 * Copyright (c) 2011-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024, Mainline port for MSM8953
 */

#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/slimbus.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/notifier.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/soc/qcom/pdr.h>
#include <net/sock.h>
#include "slimbus.h"

/* ---- Register layout ---------------------------------------------------- */

/*
 * NGD base offset within the SLIMbus register block.
 * MSM8953 always uses Version 2 layout: NGD1=0x1000, NGD2=0x2000
 */
#define NGD_BASE(nr)		(((nr) % 2) ? 0x1000 : 0x2000)

/* PGD port registers (V2 layout, relative to SLIMbus base) */
#define PGD_PORT_CFGn(p)	(0x14000 + (p) * 0x1000)
#define PGD_PORT_STATn(p)	(0x14004 + (p) * 0x1000)
#define PGD_PORT_BLKn(p)	(0x1400C + (p) * 0x1000)
#define PGD_PORT_TRANn(p)	(0x14010 + (p) * 0x1000)
#define PGD_PORT_INT_EN_EEn	(0x5000) /* + EE * 0x1000 */
#define PGD_PORT_INT_CL_EEn	(0x5008)

/* BAM pipe registers (relative to BAM base, per-pipe stride = 0x1000) */
#define BAM_P_CTRL(p)		(0x1000 + (p) * 0x1000)
#define BAM_P_RST(p)		(0x1004 + (p) * 0x1000)
#define BAM_P_HALT(p)		(0x1008 + (p) * 0x1000)
#define BAM_P_IRQ_STTS(p)	(0x1010 + (p) * 0x1000)
#define BAM_P_IRQ_CLR(p)	(0x1014 + (p) * 0x1000)
#define BAM_P_IRQ_EN(p)		(0x1018 + (p) * 0x1000)
#define BAM_P_DESC_FIFO_ADDR(p)	(0x181C + (p) * 0x1000)
#define BAM_P_DESC_FIFO_SIZE(p)	(0x1820 + (p) * 0x1000)
#define BAM_P_EVNT_REG(p)	(0x1818 + (p) * 0x1000)
#define BAM_P_SW_OFSTS(p)	(0x1800 + (p) * 0x1000)
#define BAM_P_EVNT_DEST_ADDR(p)	(0x182C + (p) * 0x1000)

/* BAM descriptor flags */
#define BAM_DESC_INT		BIT(15)
#define BAM_DESC_EOT		BIT(14)

#define BAM_P_CTRL_EN		BIT(1)
#define BAM_P_CTRL_SYS_MODE	BIT(5)  /* 1=SYS mode, 0=BAM2BAM */
#define BAM_P_CTRL_DIRECTION	BIT(3)  /* 1=mem to periph, 0=periph to mem */

#define SLIM_BAM_DESC_NUM	32

#define PGD_PORT_CFG_WATERMARK(x)	((x) << 1)
#define PGD_PORT_CFG_ENABLE		BIT(0)
#define PGD_PORT_CFG_PACK		(0 << 6)
#define PGD_PORT_CFG_ALIGN_LSB		(0 << 9)

#define DEF_WATERMARK	PGD_PORT_CFG_WATERMARK(1)
#define DEF_BLKSZ	3  /* 4-byte blocks */
#define DEF_TRANSZ	0  /* default */

/* WCD9335 codec port numbering: RX ports start at 16 */
#define WCD9335_RX_START	16

/* NGD registers (relative to NGD base) */
#define QCOM_SLIM_NGD_DESC_NUM	32

#define NGD_CFG			0x0
#define NGD_CFG_ENABLE		BIT(0)
#define NGD_CFG_RX_MSGQ_EN	BIT(1)
#define NGD_CFG_TX_MSGQ_EN	BIT(2)

#define NGD_STATUS		0x4
#define NGD_LADDR		BIT(1)

#define NGD_RX_MSGQ_CFG		0x8
#define NGD_INT_EN		0x10
#define NGD_INT_STAT		0x14
#define NGD_INT_CLR		0x18
#define NGD_TX_MSG		0x30
#define NGD_RX_MSG		0x70

/* NGD interrupt bits */
#define NGD_INT_RECFG_DONE	BIT(24)
#define NGD_INT_TX_NACKED_2	BIT(25)
#define NGD_INT_MSG_BUF_CONTE	BIT(26)
#define NGD_INT_MSG_TX_INVAL	BIT(27)
#define NGD_INT_IE_VE_CHG	BIT(28)
#define NGD_INT_DEV_ERR		BIT(29)
#define NGD_INT_RX_MSG_RCVD	BIT(30)
#define NGD_INT_TX_MSG_SENT	BIT(31)

#define DEF_NGD_INT_MASK	(NGD_INT_TX_NACKED_2 | NGD_INT_MSG_BUF_CONTE | \
				 NGD_INT_MSG_TX_INVAL | NGD_INT_IE_VE_CHG | \
				 NGD_INT_DEV_ERR | NGD_INT_TX_MSG_SENT | \
				 NGD_INT_RX_MSG_RCVD)

#define SLIM_RX_MSGQ_TIMEOUT_VAL	0x10000

/* ---- QMI ---------------------------------------------------------------- */

#define SLIMBUS_QMI_SVC_ID		0x0301
#define SLIMBUS_QMI_SVC_V1		1
#define SLIMBUS_QMI_INS_ID		0

#define SLIMBUS_QMI_SELECT_INSTANCE_REQ_V01	0x0020
#define SLIMBUS_QMI_SELECT_INSTANCE_RESP_V01	0x0020
#define SLIMBUS_QMI_POWER_REQ_V01		0x0021
#define SLIMBUS_QMI_POWER_RESP_V01		0x0021

#define SLIMBUS_QMI_POWER_REQ_MAX_MSG_LEN		14
#define SLIMBUS_QMI_POWER_RESP_MAX_MSG_LEN		7
#define SLIMBUS_QMI_SELECT_INSTANCE_REQ_MAX_MSG_LEN	14
#define SLIMBUS_QMI_SELECT_INSTANCE_RESP_MAX_MSG_LEN	7

#define SLIMBUS_QMI_RESP_TOUT		msecs_to_jiffies(5000)
#define SLIMBUS_QMI_SELECT_RETRIES	10
#define SLIMBUS_QMI_SELECT_RETRY_MS	500

/* ---- Protocol constants ------------------------------------------------- */

#define SLIM_LA_MGR		0xFF
#define SLIM_ROOT_FREQ		24576000
#define LADDR_RETRY		60

#define SLIM_MSGQ_BUF_LEN	40

/* REPORT_SATELLITE magic bytes */
#define SAT_MAGIC_LSB	0xD9
#define SAT_MAGIC_MSB	0xC5
#define SAT_MSG_VER	0x1
#define SAT_MSG_PROT	0x1
#define MSM_SAT_SUCCSS	0x20

/* User-defined message codes */
#define SLIM_USR_MC_MASTER_CAPABILITY		0x0
#define SLIM_USR_MC_REPEAT_CHANGE_VALUE		0x0
#define SLIM_USR_MC_REPORT_SATELLITE		0x1
#define SLIM_USR_MC_ADDR_QUERY		0xD
#define SLIM_USR_MC_ADDR_REPLY		0xE
#define SLIM_USR_MC_DEFINE_CHAN		0x20
#define SLIM_USR_MC_DEF_ACT_CHAN	0x21
#define SLIM_USR_MC_CHAN_CTRL		0x23
#define SLIM_USR_MC_RECONFIG_NOW	0x24
#define SLIM_USR_MC_GENERIC_ACK		0x25
#define SLIM_USR_MC_REQ_BW		0x28
#define SLIM_USR_MC_CONNECT_SRC		0x2C
#define SLIM_USR_MC_CONNECT_SINK	0x2D
#define SLIM_USR_MC_DISCONNECT_PORT	0x2E

/* Message assembly helper */
#define SLIM_MSG_ASM_FIRST_WORD(l, mt, mc, dt, ad) \
	((l) | ((mt) << 5) | ((mc) << 8) | ((dt) << 15) | ((ad) << 16))

#define INIT_MX_RETRIES	5
#define DEF_RETRY_MS	10

/* EE number for Apps (satellite) */
#define MSM_SLIM_EE		1

/* ---- State machine ------------------------------------------------------- */

enum msm8953_slim_state {
	MSM8953_SLIM_AWAKE,
	MSM8953_SLIM_IDLE,
	MSM8953_SLIM_ASLEEP,
	MSM8953_SLIM_DOWN,
};

/* ---- QMI message types -------------------------------------------------- */

enum slimbus_mode_enum_type_v01 {
	SLIMBUS_MODE_ENUM_TYPE_MIN_ENUM_VAL_V01 = INT_MIN,
	SLIMBUS_MODE_SATELLITE_V01 = 1,
	SLIMBUS_MODE_MASTER_V01 = 2,
	SLIMBUS_MODE_ENUM_TYPE_MAX_ENUM_VAL_V01 = INT_MAX,
};

enum slimbus_pm_enum_type_v01 {
	SLIMBUS_PM_ENUM_TYPE_MIN_ENUM_VAL_V01 = INT_MIN,
	SLIMBUS_PM_INACTIVE_V01 = 1,
	SLIMBUS_PM_ACTIVE_V01 = 2,
	SLIMBUS_PM_ENUM_TYPE_MAX_ENUM_VAL_V01 = INT_MAX,
};

struct slimbus_select_inst_req_msg_v01 {
	uint32_t instance;
	uint8_t mode_valid;
	enum slimbus_mode_enum_type_v01 mode;
};

struct slimbus_select_inst_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};

struct slimbus_power_req_msg_v01 {
	enum slimbus_pm_enum_type_v01 pm_req;
	uint8_t resp_type_valid;
};

struct slimbus_power_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};

static const struct qmi_elem_info slimbus_select_inst_req_msg_v01_ei[] = {
	{
		.data_type = QMI_UNSIGNED_4_BYTE,
		.elem_len  = 1,
		.elem_size = sizeof(uint32_t),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x01,
		.offset    = offsetof(struct slimbus_select_inst_req_msg_v01,
				      instance),
	},
	{
		.data_type = QMI_OPT_FLAG,
		.elem_len  = 1,
		.elem_size = sizeof(uint8_t),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x10,
		.offset    = offsetof(struct slimbus_select_inst_req_msg_v01,
				      mode_valid),
	},
	{
		.data_type = QMI_UNSIGNED_4_BYTE,
		.elem_len  = 1,
		.elem_size = sizeof(enum slimbus_mode_enum_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x10,
		.offset    = offsetof(struct slimbus_select_inst_req_msg_v01,
				      mode),
	},
	{ .data_type = QMI_EOTI },
};

static const struct qmi_elem_info slimbus_select_inst_resp_msg_v01_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len  = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x02,
		.offset    = offsetof(struct slimbus_select_inst_resp_msg_v01,
				      resp),
		.ei_array  = qmi_response_type_v01_ei,
	},
	{ .data_type = QMI_EOTI },
};

static const struct qmi_elem_info slimbus_power_req_msg_v01_ei[] = {
	{
		.data_type = QMI_UNSIGNED_4_BYTE,
		.elem_len  = 1,
		.elem_size = sizeof(enum slimbus_pm_enum_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x01,
		.offset    = offsetof(struct slimbus_power_req_msg_v01, pm_req),
	},
	{
		.data_type = QMI_OPT_FLAG,
		.elem_len  = 1,
		.elem_size = sizeof(uint8_t),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x10,
		.offset    = offsetof(struct slimbus_power_req_msg_v01,
				      resp_type_valid),
	},
	{ .data_type = QMI_EOTI },
};

static const struct qmi_elem_info slimbus_power_resp_msg_v01_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len  = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type  = 0x02,
		.offset    = offsetof(struct slimbus_power_resp_msg_v01, resp),
		.ei_array  = qmi_response_type_v01_ei,
	},
	{ .data_type = QMI_EOTI },
};

/* ---- DMA descriptor ----------------------------------------------------- */

struct msm8953_slim_dma_desc {
	struct dma_async_tx_descriptor *desc;
	struct msm8953_slim_ctrl *dev;
	dma_addr_t phys;
	void *base;
};

/* ---- Driver data structures --------------------------------------------- */

struct msm8953_slim_ctrl {
	struct slim_controller ctrl;
	struct slim_framer framer;
	struct device *dev;
	void __iomem *base;
	void __iomem *bam_base;
	int irq;
	int bam_irq;
	int ee;
	u32 ctrl_nr;
	enum msm8953_slim_state state;
	struct completion reconf;
	struct completion ctrl_up;
	struct mutex tx_lock;
	spinlock_t rx_lock;
	/* Static TX buffer; protected by tx_lock */
	u32 tx_buf[10];
	/*
	 * Single TX-completion slot, pointing at the sender's on-stack
	 * completion. Senders hold tx_lock; wr_lock also orders them
	 * against the DMA callback and the IRQ handler, so a sender that
	 * timed out cannot return while one of those is completing it.
	 */
	spinlock_t wr_lock;
	struct completion *wr_comp;
	int err;
	atomic_t ssr_in_progress;

	/*
	 * SSR generation counter: incremented on every DOWN event and
	 * sampled by UP before scheduling recovery work. If the worker
	 * sees a different generation, a newer DOWN has arrived and this
	 * recovery attempt is stale.
	 */
	atomic_t ssr_gen;
	u32 ngd_up_gen;  /* generation sampled by the UP handler */
	u32 mcap_gen;    /* ssr_gen expected by current MCAP waiter */

	/* QMI */
	struct qmi_handle qmi;
	struct sockaddr_qrtr qmi_svc_info;
	struct completion qmi_up;

	/* Work items */
	struct delayed_work ngd_up_work;
	struct work_struct slave_notify_work;
	struct notifier_block nb;
	void *notifier;
	struct pdr_handle *pdr;

	/* DT-derived platform data */
	u32 apps_pipes;
	u32 eapc;
	struct device_node *ngd_node; /* slim@N child for device iteration */

	/* PGD logical address (resolved lazily) */
	u8 pgdla;

	/* Port-to-pipe mapping: SLIMbus port to BAM/PGD pipe */
	u8 pipe_map[32]; /* 0xFF = unallocated */
	u32 pipe_alloc;   /* bitmask of allocated pipes */

	/* BAM DMA descriptor FIFO for data pipes */
	void *desc_fifo_virt;
	dma_addr_t desc_fifo_phys;
	bool bam_pipe_setup;

	/* BAM DMA channels */
	struct dma_chan *dma_rx_channel;
	struct dma_chan *dma_tx_channel;
	void *rx_base;
	dma_addr_t rx_phys_base;
	void *tx_base;
	dma_addr_t tx_phys_base;
	struct msm8953_slim_dma_desc rx_desc[QCOM_SLIM_NGD_DESC_NUM];

	/* Direct BAM TX descriptor FIFO */
	void *tx_desc_fifo_virt;
	dma_addr_t tx_desc_fifo_phys;
	u32 tx_desc_write_offset;
	u32 tx_desc_fifo_size;
	int tx_bam_pipe;  /* BAM pipe index for TX msgq */
	bool use_bam_tx;  /* true when direct BAM TX is ready */
	/*
	 * Set when a CORE NEXT_REMOVE_CHANNEL has been translated to USR
	 * CHAN_CTRL; the next CORE RECONFIGURE_NOW is then translated to
	 * USR RECONFIG_NOW. teardown_laddr holds the codec laddr from that
	 * NEXT_REMOVE_CHANNEL, since the RECONFIGURE_NOW on the disable path
	 * is a broadcast and carries no laddr.
	 */
	bool teardown_reconfig_pending;
	u8 teardown_laddr;

	/* Data port DMA channels and buffers */
	struct dma_chan *data_chan[2];
	void *data_buf[2];
	dma_addr_t data_buf_phys[2];

	/*
	 * Manager-side (AP) data port allocation. Each set bit in
	 * apps_pipes (bits 7..31) is a data pipe owned by the AP, handed
	 * out one at a time via alloc_port.
	 *
	 * mgrport_alloc_mask: bit set if that apps_pipes bit is allocated.
	 *   port_b = bit - 7 is the manager-side SLIMbus port number
	 *   used in USR CONNECT messages.
	 * mgrport_count: number of currently allocated manager ports.
	 */
	u32 mgrport_alloc_mask;
	u8 mgrport_count;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
	/* Message trace ring buffer */
#define SLIM_MSG_LOG_SIZE 64
	struct {
		u64 timestamp;
		u8 dir;       /* 0=TX, 1=RX */
		u8 len;
		u8 data[16];
	} msg_log[SLIM_MSG_LOG_SIZE];
	unsigned int msg_log_head;
	spinlock_t msg_log_lock;
#endif
};

/* ---- Helpers ------------------------------------------------------------- */

#ifdef CONFIG_DEBUG_FS
static void msm8953_slim_log_msg(struct msm8953_slim_ctrl *dev,
				 u8 dir, u8 *buf, u8 len);
#endif

static inline void __iomem *ngd_base(struct msm8953_slim_ctrl *dev)
{
	return dev->base + NGD_BASE(dev->ctrl_nr);
}

static void msm8953_slim_set_wr_comp(struct msm8953_slim_ctrl *dev,
				     struct completion *comp)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->wr_lock, flags);
	dev->wr_comp = comp;
	spin_unlock_irqrestore(&dev->wr_lock, flags);
}

static bool msm8953_slim_complete_wr(struct msm8953_slim_ctrl *dev)
{
	unsigned long flags;
	bool completed = false;

	spin_lock_irqsave(&dev->wr_lock, flags);
	if (dev->wr_comp) {
		complete(dev->wr_comp);
		dev->wr_comp = NULL;
		completed = true;
	}
	spin_unlock_irqrestore(&dev->wr_lock, flags);

	return completed;
}


/* BAM v1.7.0 pipe register offsets: base 0x13000, stride 0x1000 per pipe */
#define BAM17_P_CTRL(p)		(0x13000 + (p) * 0x1000)
#define BAM17_P_DESC_FIFO_ADDR(p) (0x1381C + (p) * 0x1000)
#define BAM17_P_FIFO_SIZES(p)	(0x13820 + (p) * 0x1000)
#define BAM17_P_EVNT_REG(p)	(0x13818 + (p) * 0x1000)

/* Send message via BAM TX pipe */
static int msm8953_slim_bam_tx(struct msm8953_slim_ctrl *dev,
			       u32 *buf, u8 len)
{
	if (!dev->use_bam_tx)
		return -ENODEV;

	len = (len + 3) & 0xfc;
	memcpy(dev->tx_base, buf, len);

	if (dev->dma_tx_channel) {
		struct dma_async_tx_descriptor *desc;

		desc = dmaengine_prep_slave_single(dev->dma_tx_channel,
						   dev->tx_phys_base, len,
						   DMA_MEM_TO_DEV,
						   DMA_PREP_INTERRUPT);
		if (!desc) {
			dev_err(dev->dev, "DMA TX prep failed\n");
			return -ENOMEM;
		}

		desc->cookie = dmaengine_submit(desc);
		dma_async_issue_pending(dev->dma_tx_channel);

		return 0;
	}

	return -ENODEV;
}

/* AHB-based TX: write message words directly to NGD_TX_MSG register */
static int msm8953_slim_ahb_tx(struct msm8953_slim_ctrl *dev,
				u32 *buf, u8 len, u32 tx_reg_offset)
{
	int i;

	for (i = 0; i < ((len + 3) >> 2); i++)
		writel_relaxed(buf[i], dev->base + tx_reg_offset + (i * 4));

	/* Guarantee message is committed before returning */
	mb();
	return 0;
}

/* ---- RX message dispatch ------------------------------------------------ */

static void msm8953_slim_rx(struct msm8953_slim_ctrl *dev, u8 *buf)
{
	unsigned long flags;
	u8 mc, mt, len;

	len = buf[0] & 0x1F;
	mt  = (buf[0] >> 5) & 0x7;
	mc  = buf[1];

#ifdef CONFIG_DEBUG_FS
	msm8953_slim_log_msg(dev, 1, buf, min_t(u8, len, 16));
#endif

	if (mc != SLIM_MSG_MC_REPLY_VALUE)
		dev_dbg(dev->dev, "RX: mt=0x%x mc=0x%x len=%d %5ph\n",
			mt, mc, len, buf);
	else if (buf[4] != 0)
		dev_dbg(dev->dev, "RX REPLY_VALUE: tid=%d data=0x%02x\n",
			buf[3], buf[4]);

	/* MASTER_CAPABILITY: signal the power-up completion */
	if (mc == SLIM_USR_MC_MASTER_CAPABILITY &&
	    mt == SLIM_MSG_MT_SRC_REFERRED_USER) {
		if (dev->mcap_gen != (u32)atomic_read(&dev->ssr_gen)) {
			dev_dbg(dev->dev,
				 "MCAP ignored: stale gen %u (current %d)\n",
				 dev->mcap_gen, atomic_read(&dev->ssr_gen));
			return;
		}
		dev_dbg(dev->dev, "received MASTER_CAPABILITY\n");
		complete(&dev->reconf);
		return;
	}

	/* REPLY_INFORMATION / REPLY_VALUE: forward to framework */
	if (mc == SLIM_MSG_MC_REPLY_INFORMATION ||
	    mc == SLIM_MSG_MC_REPLY_VALUE) {
		u8 tid = buf[3];

		slim_msg_response(&dev->ctrl, &buf[4], tid, len - 4);
		pm_runtime_mark_last_busy(dev->dev);
		return;
	}

	/* ADDR_REPLY: logical address response */
	if (mc == SLIM_USR_MC_ADDR_REPLY &&
	    mt == SLIM_MSG_MT_SRC_REFERRED_USER) {
		struct slim_msg_txn *txn;
		u8 failed_ea[6] = { 0 };

		spin_lock_irqsave(&dev->ctrl.txn_lock, flags);
		txn = idr_find(&dev->ctrl.tid_idr, buf[3]);
		if (!txn) {
			spin_unlock_irqrestore(&dev->ctrl.txn_lock, flags);
			dev_warn(dev->dev,
				 "LADDR response after timeout, tid:0x%x\n",
				 buf[3]);
			return;
		}
		if (memcmp(&buf[4], failed_ea, 6))
			txn->la = buf[10];
		idr_remove(&dev->ctrl.tid_idr, txn->tid);
		spin_unlock_irqrestore(&dev->ctrl.txn_lock, flags);
		complete(txn->comp);
		return;
	}

	/* GENERIC_ACK */
	if (mc == SLIM_USR_MC_GENERIC_ACK &&
	    mt == SLIM_MSG_MT_SRC_REFERRED_USER) {
		struct slim_msg_txn *txn;
		bool nack = false;

		spin_lock_irqsave(&dev->ctrl.txn_lock, flags);
		txn = idr_find(&dev->ctrl.tid_idr, buf[3]);
		if (!txn) {
			spin_unlock_irqrestore(&dev->ctrl.txn_lock, flags);
			dev_warn(dev->dev,
				 "ACK after timeout, tid:0x%x\n", buf[3]);
			return;
		}
		if (!(buf[4] & MSM_SAT_SUCCSS)) {
			dev_warn(dev->dev, "TID:%d NACK:0x%x\n",
				 (int)buf[3], buf[4]);
			nack = true;
		}
		idr_remove(&dev->ctrl.tid_idr, txn->tid);
		spin_unlock_irqrestore(&dev->ctrl.txn_lock, flags);
		if (nack)
			txn->ec = (u16)(-EIO);
		complete(txn->comp);
		return;
	}
}

/* ---- Interrupt handler --------------------------------------------------- */

static irqreturn_t msm8953_slim_interrupt(int irq, void *d)
{
	struct msm8953_slim_ctrl *dev = d;
	void __iomem *ngd = ngd_base(dev);
	u32 stat = readl_relaxed(ngd + NGD_INT_STAT);

	dev_dbg(dev->dev, "IRQ: stat=0x%x\n", stat);

	/* TX error conditions */
	if ((stat & NGD_INT_MSG_BUF_CONTE) ||
	    (stat & NGD_INT_MSG_TX_INVAL) ||
	    (stat & NGD_INT_DEV_ERR) ||
	    (stat & NGD_INT_TX_NACKED_2)) {
		writel_relaxed(stat, ngd + NGD_INT_CLR);
		if (stat & NGD_INT_MSG_TX_INVAL)
			dev->err = -EINVAL;
		else
			dev->err = -EIO;
		dev_warn_ratelimited(dev->dev, "NGD interrupt error: 0x%x err:%d\n",
				     stat, dev->err);
		/* Publish dev->err before completing the waiter */
		mb();
		msm8953_slim_complete_wr(dev);
	}

	/* TX sent successfully (AHB path) */
	if (stat & NGD_INT_TX_MSG_SENT) {
		writel_relaxed(NGD_INT_TX_MSG_SENT, ngd + NGD_INT_CLR);
		/* Complete the IRQ clear before signalling the waiter */
		mb();
		if (!msm8953_slim_complete_wr(dev))
			dev_dbg(dev->dev, "TX_MSG_SENT: no wr_comp set!\n");
	}

	/* RX message received via register (no BAM needed for enumeration) */
	if (stat & NGD_INT_RX_MSG_RCVD) {
		u32 rx_buf[10];
		u8 len, i;

		rx_buf[0] = readl_relaxed(ngd + NGD_RX_MSG);
		len = rx_buf[0] & 0x1F;
		for (i = 1; i < ((len + 3) >> 2); i++)
			rx_buf[i] = readl_relaxed(ngd + NGD_RX_MSG + (4 * i));

		writel_relaxed(NGD_INT_RX_MSG_RCVD, ngd + NGD_INT_CLR);
		/* Complete the IRQ clear before dispatching the message */
		mb();
		msm8953_slim_rx(dev, (u8 *)rx_buf);
	}

	if (stat & NGD_INT_RECFG_DONE) {
		writel_relaxed(NGD_INT_RECFG_DONE, ngd + NGD_INT_CLR);
		/* Complete the IRQ clear */
		mb();
		dev_dbg(dev->dev, "reconfig done IRQ\n");
	}

	if (stat & NGD_INT_IE_VE_CHG) {
		writel_relaxed(NGD_INT_IE_VE_CHG, ngd + NGD_INT_CLR);
		/* Complete the IRQ clear */
		mb();
		dev_dbg(dev->dev, "NGD IE VE change\n");
	}

	return IRQ_HANDLED;
}

/* ---- QMI ---------------------------------------------------------------- */

static int msm8953_slim_qmi_power_request(struct msm8953_slim_ctrl *dev,
					  bool active)
{
	struct slimbus_power_req_msg_v01 req = {};
	struct slimbus_power_resp_msg_v01 resp = {};
	struct qmi_txn txn;
	int rc;

	req.pm_req = active ? SLIMBUS_PM_ACTIVE_V01 : SLIMBUS_PM_INACTIVE_V01;
	req.resp_type_valid = 0;

	rc = qmi_txn_init(&dev->qmi, &txn,
			  slimbus_power_resp_msg_v01_ei, &resp);
	if (rc < 0) {
		dev_err(dev->dev, "QMI power TXN init fail: %d\n", rc);
		return rc;
	}

	rc = qmi_send_request(&dev->qmi, &dev->qmi_svc_info, &txn,
			      SLIMBUS_QMI_POWER_REQ_V01,
			      SLIMBUS_QMI_POWER_REQ_MAX_MSG_LEN,
			      slimbus_power_req_msg_v01_ei, &req);
	if (rc < 0) {
		dev_err(dev->dev, "QMI power send fail: %d\n", rc);
		qmi_txn_cancel(&txn);
		return rc;
	}

	rc = qmi_txn_wait(&txn, SLIMBUS_QMI_RESP_TOUT);
	if (rc < 0) {
		dev_err(dev->dev, "QMI power TXN wait fail: %d\n", rc);
		return rc;
	}

	if (resp.resp.result != QMI_RESULT_SUCCESS_V01) {
		dev_err(dev->dev, "QMI power req failed: 0x%x\n",
			resp.resp.result);
		return -EREMOTEIO;
	}

	return 0;
}

static int msm8953_slim_qmi_select_instance(struct msm8953_slim_ctrl *dev)
{
	struct slimbus_select_inst_req_msg_v01 req = {};
	struct slimbus_select_inst_resp_msg_v01 resp = {};
	struct qmi_txn txn;
	int rc;

	/*
	 * The QMI instance is ctrl_nr >> 1 (cell-index is 1-based).
	 * Mode MASTER means the ADSP is the bus manager and framer.
	 */
	req.instance   = dev->ctrl_nr >> 1;
	req.mode_valid = 1;
	req.mode       = SLIMBUS_MODE_MASTER_V01;

	rc = qmi_txn_init(&dev->qmi, &txn,
			  slimbus_select_inst_resp_msg_v01_ei, &resp);
	if (rc < 0) {
		dev_err(dev->dev, "QMI select TXN init fail: %d\n", rc);
		return rc;
	}

	rc = qmi_send_request(&dev->qmi, &dev->qmi_svc_info, &txn,
			      SLIMBUS_QMI_SELECT_INSTANCE_REQ_V01,
			      SLIMBUS_QMI_SELECT_INSTANCE_REQ_MAX_MSG_LEN,
			      slimbus_select_inst_req_msg_v01_ei, &req);
	if (rc < 0) {
		dev_err(dev->dev, "QMI select send fail: %d\n", rc);
		qmi_txn_cancel(&txn);
		return rc;
	}

	rc = qmi_txn_wait(&txn, SLIMBUS_QMI_RESP_TOUT);
	if (rc < 0) {
		dev_err(dev->dev, "QMI select TXN wait fail: %d\n", rc);
		return rc;
	}

	if (resp.resp.result != QMI_RESULT_SUCCESS_V01) {
		dev_err(dev->dev, "QMI select instance failed: 0x%x\n",
			resp.resp.result);
		return -EREMOTEIO;
	}

	dev_dbg(dev->dev, "QMI: instance selected (ctrl %d)\n", dev->ctrl_nr);
	return 0;
}

/* QMI new-server callback: service is up */
static int msm8953_qmi_new_server(struct qmi_handle *qmi,
				  struct qmi_service *svc)
{
	struct msm8953_slim_ctrl *dev =
		container_of(qmi, struct msm8953_slim_ctrl, qmi);

	dev->qmi_svc_info.sq_family = AF_QIPCRTR;
	dev->qmi_svc_info.sq_node   = svc->node;
	dev->qmi_svc_info.sq_port   = svc->port;
	dev_dbg(dev->dev, "QMI slimbus service found (node %d port %d)\n",
		 svc->node, svc->port);
	complete(&dev->qmi_up);
	return 0;
}

static void msm8953_qmi_del_server(struct qmi_handle *qmi,
				   struct qmi_service *svc)
{
	struct msm8953_slim_ctrl *dev =
		container_of(qmi, struct msm8953_slim_ctrl, qmi);

	dev_dbg(dev->dev, "QMI slimbus service gone\n");
	reinit_completion(&dev->qmi_up);
}

static const struct qmi_ops msm8953_slim_qmi_ops = {
	.new_server = msm8953_qmi_new_server,
	.del_server = msm8953_qmi_del_server,
};

/* ---- NGD hardware setup ------------------------------------------------- */

static void msm8953_slim_rx_msgq_cb(void *args)
{
	struct msm8953_slim_dma_desc *d = args;
	struct msm8953_slim_ctrl *dev = d->dev;

	msm8953_slim_rx(dev, (u8 *)d->base);

	d->desc = dmaengine_prep_slave_single(dev->dma_rx_channel,
					      d->phys, SLIM_MSGQ_BUF_LEN,
					      DMA_DEV_TO_MEM,
					      DMA_PREP_INTERRUPT);
	if (!d->desc) {
		dev_err(dev->dev, "RX msgq: repost prep failed\n");
		return;
	}
	d->desc->callback       = msm8953_slim_rx_msgq_cb;
	d->desc->callback_param = d;
	d->desc->cookie         = dmaengine_submit(d->desc);
	dma_async_issue_pending(dev->dma_rx_channel);
}

static int msm8953_slim_ngd_setup(struct msm8953_slim_ctrl *dev);

static int msm8953_slim_init_dma(struct msm8953_slim_ctrl *dev)
{
	int ret, i, size;

	if (dev->dma_rx_channel)
		return 0;

	dev->dma_rx_channel = dma_request_chan(dev->dev, "rx");
	if (IS_ERR(dev->dma_rx_channel)) {
		ret = PTR_ERR(dev->dma_rx_channel);
		dev_err(dev->dev, "Failed to request RX DMA channel: %d\n", ret);
		dev->dma_rx_channel = NULL;
		return ret;
	}

	size = QCOM_SLIM_NGD_DESC_NUM * SLIM_MSGQ_BUF_LEN;
	dev->rx_base = dma_alloc_coherent(dev->dev, size,
					  &dev->rx_phys_base, GFP_KERNEL);
	if (!dev->rx_base) {
		ret = -ENOMEM;
		goto rel_rx;
	}

	for (i = 0; i < QCOM_SLIM_NGD_DESC_NUM; i++) {
		struct msm8953_slim_dma_desc *d = &dev->rx_desc[i];

		d->dev  = dev;
		d->phys = dev->rx_phys_base + i * SLIM_MSGQ_BUF_LEN;
		d->base = dev->rx_base + i * SLIM_MSGQ_BUF_LEN;

		d->desc = dmaengine_prep_slave_single(dev->dma_rx_channel,
						      d->phys, SLIM_MSGQ_BUF_LEN,
						      DMA_DEV_TO_MEM,
						      DMA_PREP_INTERRUPT);
		if (!d->desc) {
			dev_err(dev->dev, "RX msgq: prep failed at desc %d\n", i);
			ret = -EINVAL;
			goto free_rx;
		}
		d->desc->callback       = msm8953_slim_rx_msgq_cb;
		d->desc->callback_param = d;
		d->desc->cookie         = dmaengine_submit(d->desc);
	}
	dma_async_issue_pending(dev->dma_rx_channel);

	/*
	 * TX messaging via the DMA engine. The DMA engine resets the TX BAM
	 * pipe and reconfigures it with its own descriptor FIFO. Without a
	 * TX channel, messages are sent through the NGD_TX_MSG registers.
	 */
	dev->dma_tx_channel = dma_request_chan(dev->dev, "tx");
	if (IS_ERR(dev->dma_tx_channel)) {
		dev_warn(dev->dev, "TX DMA not available, using AHB TX\n");
		dev->dma_tx_channel = NULL;
		goto skip_bam_tx;
	}

	size = SLIM_MSGQ_BUF_LEN;
	dev->tx_base = dma_alloc_coherent(dev->dev, size,
					  &dev->tx_phys_base, GFP_KERNEL);
	if (!dev->tx_base) {
		dma_release_channel(dev->dma_tx_channel);
		dev->dma_tx_channel = NULL;
		goto skip_bam_tx;
	}

	dev->use_bam_tx = true;

skip_bam_tx:
	dev_dbg(dev->dev, "BAM DMA init done: rx_phys=0x%pad bam_tx=%s\n",
		&dev->rx_phys_base, dev->use_bam_tx ? "yes" : "no(AHB)");
	return 0;
free_rx:
	dma_free_coherent(dev->dev, QCOM_SLIM_NGD_DESC_NUM * SLIM_MSGQ_BUF_LEN,
			  dev->rx_base, dev->rx_phys_base);
	dev->rx_base = NULL;
rel_rx:
	dmaengine_terminate_sync(dev->dma_rx_channel);
	dma_release_channel(dev->dma_rx_channel);
	dev->dma_rx_channel = NULL;
	return ret;
}

static void msm8953_slim_teardown_dma(struct msm8953_slim_ctrl *dev)
{
	if (dev->dma_rx_channel) {
		dmaengine_terminate_sync(dev->dma_rx_channel);
		dma_release_channel(dev->dma_rx_channel);
		dev->dma_rx_channel = NULL;
	}
	dev->use_bam_tx = false;
	if (dev->dma_tx_channel) {
		dmaengine_terminate_sync(dev->dma_tx_channel);
		dma_release_channel(dev->dma_tx_channel);
		dev->dma_tx_channel = NULL;
	}
	if (dev->rx_base) {
		dma_free_coherent(dev->dev,
				  QCOM_SLIM_NGD_DESC_NUM * SLIM_MSGQ_BUF_LEN,
				  dev->rx_base, dev->rx_phys_base);
		dev->rx_base = NULL;
	}
	if (dev->tx_base) {
		dma_free_coherent(dev->dev, SLIM_MSGQ_BUF_LEN,
				  dev->tx_base, dev->tx_phys_base);
		dev->tx_base = NULL;
	}
}

#define DATA_BUF_SIZE 256

/* BAM v1.7.0 data pipe and EE register offsets */
#define BAM17_P_RST(p)		(0x13004 + (p) * 0x1000)
#define BAM17_P_IRQ_EN(p)	(0x13018 + (p) * 0x1000)
#define BAM17_IRQ_SRCS_MSK_EE(ee) (0x03004 + (ee) * 0x1000)

/*
 * msm8953_slim_alloc_port - controller ->alloc_port callback
 *
 * Hands out the next free manager-side data port from the apps_pipes
 * bitmap. Returns the SLIMbus port number ("port_b") in *port_out so
 * the caller can use it in USR CONNECT messages addressed to the
 * manager. apps_pipes bits 7..31 enumerate AP-owned data pipes; port_b
 * is (bit - 7).
 */
static int msm8953_slim_alloc_port(struct slim_controller *ctrl, u8 *port_out)
{
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);
	int bit;
	int port_idx;
	u8 port_b;

	if (!port_out)
		return -EINVAL;
	if (!dev->apps_pipes) {
		dev_err(dev->dev,
			"alloc_port: no apps_pipes (DT missing qcom,apps-ch-pipes?)\n");
		return -ENODEV;
	}

	mutex_lock(&dev->tx_lock);

	/* Find the lowest apps_pipes bit not yet handed out. */
	for (bit = 7; bit < 32; bit++) {
		if (!(dev->apps_pipes & (1u << bit)))
			continue;
		if (dev->mgrport_alloc_mask & (1u << bit))
			continue;
		break;
	}
	if (bit >= 32) {
		mutex_unlock(&dev->tx_lock);
		dev_err(dev->dev,
			"alloc_port: no free mgr port (apps_pipes=0x%x alloc=0x%x)\n",
			dev->apps_pipes, dev->mgrport_alloc_mask);
		return -EBUSY;
	}

	port_b   = (u8)(bit - 7);
	port_idx = dev->mgrport_count;

	/*
	 * Only track the allocation; PGD_PORT_CFGn is not touched. The
	 * ADSP configures the PGD ports and data pipes for audio in
	 * response to CONNECT and DEF_ACT_CHAN, and programming
	 * PGD_PORT_CFGn from the AP conflicts with it (the ADSP AFE then
	 * fails AFE_PORT_CMD_DEVICE_START).
	 */

	dev->mgrport_alloc_mask |= (1u << bit);
	dev->mgrport_count++;
	mutex_unlock(&dev->tx_lock);

	*port_out = port_b;
	dev_dbg(dev->dev,
		 "alloc_port: handed out port_b=%d (bit=%d port_idx=%d apps_pipes=0x%x)\n",
		 port_b, bit, port_idx, dev->apps_pipes);
	return 0;
}

/*
 * msm8953_slim_dealloc_port - controller ->dealloc_port callback
 *
 * Reverse of alloc_port: marks the port free.
 */
static int msm8953_slim_dealloc_port(struct slim_controller *ctrl, u8 port_b)
{
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);
	int bit = port_b + 7;

	if (bit < 7 || bit >= 32)
		return -EINVAL;

	mutex_lock(&dev->tx_lock);
	if (!(dev->mgrport_alloc_mask & (1u << bit))) {
		mutex_unlock(&dev->tx_lock);
		return -ENOENT;
	}
	dev->mgrport_alloc_mask &= ~(1u << bit);
	if (dev->mgrport_count)
		dev->mgrport_count--;
	mutex_unlock(&dev->tx_lock);

	dev_dbg(dev->dev, "dealloc_port: freed port_b=%d (bit=%d)\n",
		 port_b, bit);
	return 0;
}

/*
 * msm8953_slim_ngd_setup - program NGD registers and enable the controller
 *
 * When the ADSP restarts, it resets the SLIMbus hardware block, and writes
 * to NGD_CFG are dropped (read back as 0) until the reset is released.
 * NGD_CFG is therefore written and read back until the value sticks.
 *
 * Returns 0 on success, -EAGAIN if NGD_CFG never took effect; the caller
 * then retries the power-up sequence after a delay.
 */
static int msm8953_slim_ngd_setup(struct msm8953_slim_ctrl *dev)
{
	void __iomem *ngd = ngd_base(dev);
	u32 rx_msgq, cfg_readback;
	/*
	 * The ADSP firmware requires TX_MSGQ_EN to be set when the NGD is
	 * enabled before it completes the satellite handshake, so both
	 * message queues are enabled here. USR messages for the ADSP use
	 * the BAM TX queue.
	 */
	u32 cfg_val = NGD_CFG_ENABLE | NGD_CFG_RX_MSGQ_EN | NGD_CFG_TX_MSGQ_EN;
	int i;

	writel_relaxed(DEF_NGD_INT_MASK, ngd + NGD_INT_EN);

	rx_msgq = readl_relaxed(ngd + NGD_RX_MSGQ_CFG);
	writel_relaxed(rx_msgq | SLIM_RX_MSGQ_TIMEOUT_VAL,
		       ngd + NGD_RX_MSGQ_CFG);

	/* Allow up to ~50 ms for the ADSP to release the block from reset */
	for (i = 0; i < 10; i++) {
		writel_relaxed(cfg_val, ngd + NGD_CFG);
		/* Complete the write before reading it back */
		mb();

		cfg_readback = readl_relaxed(ngd + NGD_CFG);
		if (cfg_readback == cfg_val)
			break;

		dev_dbg(dev->dev,
			"NGD_CFG write not taken (attempt %d, read 0x%x)\n",
			i, cfg_readback);
		usleep_range(5000, 6000);
	}

	if (cfg_readback != cfg_val) {
		dev_dbg(dev->dev,
			"NGD_CFG stuck at 0x%x after %d attempts\n",
			cfg_readback, i);
		return -EAGAIN;
	}

	return 0;
}

static int msm8953_slim_xfer_msg(struct slim_controller *ctrl,
				 struct slim_msg_txn *txn);

/* ---- enable_stream: DEF_ACT_CHAN + RECONFIG_NOW via USR messages -------- */

static int msm8953_slim_calc_coef(struct slim_stream_runtime *rt, int *exp)
{
	struct slim_controller *ctrl = rt->dev->ctrl;
	int coef;

	if (rt->ratem * ctrl->a_framer->superfreq < rt->rate)
		rt->ratem++;

	coef = rt->ratem;
	*exp = 0;

	while (1) {
		while ((coef & 0x1) != 0x1) {
			coef >>= 1;
			(*exp)++;
		}
		if (coef <= 3)
			break;
		coef++;
	}
	if (coef == 1)
		coef = 0;

	return coef;
}

static int msm8953_slim_xfer_msg_sync(struct slim_controller *ctrl,
				       struct slim_msg_txn *txn)
{
	int ret;

	ret = msm8953_slim_xfer_msg(ctrl, txn);
	if (ret) {
		dev_warn(ctrl->dev, "USR msg send failed: mc=0x%x ret=%d\n",
			 txn->mc, ret);
		return ret;
	}

	/*
	 * Wait for the GENERIC_ACK from the ADSP. DEF_ACT_CHAN and
	 * RECONFIG_NOW are state transitions: the TID has to stay
	 * allocated until the ADSP has processed them, otherwise channels
	 * can be left defined but inactive.
	 */
	if (txn->comp) {
		unsigned long time_left;

		time_left = wait_for_completion_timeout(txn->comp,
							msecs_to_jiffies(500));
		if (!time_left) {
			dev_warn(ctrl->dev,
				 "USR msg ACK timeout: mc=0x%x tid=%d\n",
				 txn->mc, txn->tid);
			slim_free_txn_tid(ctrl, txn);
			return -ETIMEDOUT;
		}
		dev_dbg(ctrl->dev, "USR msg ACK OK: mc=0x%x tid=%d\n",
			 txn->mc, txn->tid);
	}

	return 0;
}

/*
 * Messaging slots requested with REQ_BW. The vendor codec driver reserves
 * messaging bandwidth for SLIMbus clock gear 9 (388 slots) so the ADSP
 * framer's superframe has room to schedule the codec data slot; with a
 * lower gear the ADSP does not schedule it.
 */
#define MSM8953_SLIM_REQBW_MSGSL 388

/*
 * The master side of an RX channel is not connected by the AP: the ADSP
 * AFE feeds the slot itself, and an extra master CONNECT_SOURCE makes
 * AFE_PORT_CMD_DEVICE_START fail.
 */

/*
 * disable_stream: drop the stream-duration runtime-PM hold taken in
 * enable_stream so the controller can autosuspend once no stream is active.
 */
static int msm8953_slim_disable_stream(struct slim_stream_runtime *rt)
{
	struct slim_device *sdev = rt->dev;
	struct slim_controller *ctrl = sdev->ctrl;
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
	return 0;
}

static int msm8953_slim_enable_stream(struct slim_stream_runtime *rt)
{
	struct slim_device *sdev = rt->dev;
	struct slim_controller *ctrl = sdev->ctrl;
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);
	struct slim_val_inf msg = { 0 };
	u8 wbuf[SLIM_MSGQ_BUF_LEN];
	u8 rbuf[SLIM_MSGQ_BUF_LEN];
	struct slim_msg_txn txn = { 0 };
	DECLARE_COMPLETION_ONSTACK(done);
	int i, ret, exp = 0, coef = 0;

	/*
	 * Hold the SLIM controller runtime-PM-active for the whole stream.
	 * Otherwise autosuspend sends QMI PM_INACTIVE mid-stream, the ADSP
	 * framer pauses the bus clock and the data slot starves. Balanced
	 * by the put in msm8953_slim_disable_stream.
	 */
	pm_runtime_get_sync(dev->dev);

	txn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
	txn.dt = SLIM_MSG_DEST_LOGICALADDR;
	txn.la = SLIM_LA_MGR;
	txn.ec = 0;
	txn.msg = &msg;
	txn.msg->num_bytes = 0;
	txn.msg->wbuf = wbuf;
	txn.msg->rbuf = rbuf;

	for (i = 0; i < rt->num_ports; i++) {
		struct slim_port *port = &rt->ports[i];

		if (txn.msg->num_bytes == 0) {
			/*
			 * DEF_ACT_CHAN USR message format:
			 * Byte 0: (dataf << 5) | (laddr & 0x1f)
			 * Byte 1: (sampleszbits >> 2) | (CL << 5) | (auxf << 6)
			 * Byte 2: (rootexp << 4) | protocol
			 * Byte 3: standard SLIMbus prrate | FL for ISO
			 * Byte 4: TID
			 * Byte 5+: channel IDs
			 *
			 * dataf=0 (NOT_DEFINED) matches the vendor codec
			 * driver. prrate is the standard SLIMbus presence
			 * rate code from stream_prepare.
			 */

			/*
			 * Byte 0: dataf=0 + laddr low 5 bits. The codec laddr
			 * (e.g. 0xc8) does not fit in 5 bits, and the unmasked
			 * value would set spurious dataf bits so the channel
			 * never activates. Matches the vendor NGD driver.
			 */
			wbuf[txn.msg->num_bytes++] = sdev->laddr & 0x1f;

			/* Byte 1: sampleszbits + coef + auxf */
			wbuf[txn.msg->num_bytes] = rt->bps >> 2 |
						   (port->ch.aux_fmt << 6);

			coef = msm8953_slim_calc_coef(rt, &exp);
			if (coef < 0) {
				dev_err(&sdev->dev,
					"error calculating coef %d\n", coef);
				return -EIO;
			}

			if (coef)
				wbuf[txn.msg->num_bytes] |= BIT(5);

			txn.msg->num_bytes++;

			/* Byte 2: rootexp + protocol */
			wbuf[txn.msg->num_bytes++] = exp << 4 | rt->prot;

			/* Byte 3: prrate | FL (Frequency-Locked) bit */
			wbuf[txn.msg->num_bytes++] = port->ch.prrate | 0x80;

			ret = slim_alloc_txn_tid(ctrl, &txn);
			if (ret) {
				dev_err(&sdev->dev, "Fail to allocate TID\n");
				return -ENXIO;
			}
			wbuf[txn.msg->num_bytes++] = txn.tid;

			dev_dbg(ctrl->dev,
				"DEF_ACT_CHAN: %4ph tid=%d coef=%d exp=%d prrate=%d\n",
				wbuf, txn.tid, coef, exp, port->ch.prrate);
		}
		wbuf[txn.msg->num_bytes++] = port->ch.id;
	}

	/*
	 * The ADSP satellite dispatcher ignores CORE channel definition
	 * messages; the USR DEF_ACT_CHAN below both defines and activates
	 * the channel on the ADSP.
	 *
	 * REQ_BW requests bus bandwidth from the ADSP manager. Without it
	 * the ADSP defines the channels but never allocates frame slots
	 * for them.
	 */
	{
		u8 bbuf[4];
		struct slim_val_inf bmsg = {0, 3, NULL, bbuf, NULL};
		struct slim_msg_txn btxn = {0};
		DECLARE_COMPLETION_ONSTACK(bdone);

		btxn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
		btxn.dt = SLIM_MSG_DEST_LOGICALADDR;
		btxn.la = SLIM_LA_MGR;
		btxn.mc = SLIM_USR_MC_REQ_BW;
		btxn.msg = &bmsg;

		/*
		 * REQ_BW format:
		 * byte 0: (laddr & 0x1f) | ((pending_msgsl & 0x7) << 5)
		 * byte 1: (pending_msgsl >> 3)
		 * byte 2: TID
		 */
		bbuf[0] = (sdev->laddr & 0x1f) | ((MSM8953_SLIM_REQBW_MSGSL & 0x7) << 5);
		bbuf[1] = (MSM8953_SLIM_REQBW_MSGSL >> 3);

		ret = slim_alloc_txn_tid(ctrl, &btxn);
		if (ret)
			return ret;
		bbuf[2] = btxn.tid;
		bmsg.num_bytes = 3;
		btxn.rl = bmsg.num_bytes + 4;
		btxn.comp = &bdone;

		dev_dbg(ctrl->dev, "REQ_BW: [%02x %02x tid=%d]\n",
			 bbuf[0], bbuf[1], btxn.tid);

		ret = msm8953_slim_xfer_msg_sync(ctrl, &btxn);
		if (ret) {
			slim_free_txn_tid(ctrl, &btxn);
			dev_warn(&sdev->dev, "REQ_BW failed: %d\n", ret);
		}
	}

	/*
	 * No RECONFIG_NOW is sent for the REQ_BW vote alone: it is
	 * committed together with the channel by the RECONFIG_NOW that
	 * follows DEF_ACT_CHAN, so the framer builds one non-empty
	 * configuration.
	 *
	 * Sink ports are connected here, after the REQ_BW vote and right
	 * before DEF_ACT_CHAN, matching the vendor message order. xfer_msg
	 * translates CORE CONNECT_SINK to USR CONNECT_SINK.
	 */
	{
		for (i = 0; i < rt->num_ports; i++) {
			struct slim_port *cport = &rt->ports[i];
			u8 cbuf[2] = { cport->id, cport->ch.id };
			struct slim_val_inf cmsg = { 0, 2, NULL, cbuf, NULL };
			struct slim_msg_txn ctxn = { 0 };
			DECLARE_COMPLETION_ONSTACK(cdone);

			/*
			 * The core defers only sink connects at prepare time.
			 * Source ports are already connected; connecting them
			 * again would make the ADSP tear down the existing
			 * connection.
			 */
			if (cport->direction != SLIM_PORT_SINK)
				continue;

			ctxn.mt = SLIM_MSG_MT_CORE;
			ctxn.dt = SLIM_MSG_DEST_LOGICALADDR;
			ctxn.la = sdev->laddr;
			ctxn.mc = (cport->direction == SLIM_PORT_SINK) ?
				  SLIM_MSG_MC_CONNECT_SINK :
				  SLIM_MSG_MC_CONNECT_SOURCE;
			ctxn.msg = &cmsg;
			ctxn.rl = 6;
			ctxn.comp = &cdone;

			dev_dbg(ctrl->dev, "CONNECT_%s: port=%d chan=%d\n",
				cport->direction == SLIM_PORT_SINK ? "SINK" : "SRC",
				cport->id, cport->ch.id);
			/*
			 * xfer_msg itself waits for the CONNECT ACK and consumes
			 * the completion, so xfer_msg_sync must not be used here:
			 * it would wait on the same completion again and time out.
			 */
			ret = msm8953_slim_xfer_msg(ctrl, &ctxn);
			if (ret)
				dev_warn(&sdev->dev,
					 "CONNECT_SINK failed: %d\n", ret);
		}
	}

	txn.mc = SLIM_USR_MC_DEF_ACT_CHAN;
	txn.rl = txn.msg->num_bytes + 4;
	txn.comp = &done;

	dev_dbg(ctrl->dev,
		 "enable_stream: DEF_ACT_CHAN laddr=0x%x nports=%d bps=%d prot=%d\n",
		 sdev->laddr, rt->num_ports, rt->bps, rt->prot);

	ret = msm8953_slim_xfer_msg_sync(ctrl, &txn);
	if (ret) {
		slim_free_txn_tid(ctrl, &txn);
		dev_err(&sdev->dev, "DEF_ACT_CHAN failed: %d\n", ret);
		return ret;
	}

	/* Send RECONFIG_NOW */
	reinit_completion(&done);
	txn.mc = SLIM_USR_MC_RECONFIG_NOW;
	txn.msg->num_bytes = 2;
	wbuf[1] = sdev->laddr; /* full laddr, unlike DEF_ACT_CHAN byte 0 */
	txn.rl = txn.msg->num_bytes + 4;

	ret = slim_alloc_txn_tid(ctrl, &txn);
	if (ret) {
		dev_err(ctrl->dev, "Fail to allocate TID\n");
		return ret;
	}

	wbuf[0] = txn.tid;
	txn.comp = &done;
	ret = msm8953_slim_xfer_msg_sync(ctrl, &txn);
	if (ret) {
		slim_free_txn_tid(ctrl, &txn);
		dev_err(&sdev->dev, "RECONFIG_NOW failed: %d\n", ret);
		return ret;
	}

	/*
	 * No CORE reconfiguration messages are sent: the ADSP, as sole
	 * manager, ignores them from the satellite and schedules the
	 * transport itself after DEF_ACT_CHAN and RECONFIG_NOW.
	 */
	return 0;
}

/* ---- REPORT_SATELLITE send ---------------------------------------------- */

static int msm8953_slim_report_satellite(struct msm8953_slim_ctrl *dev)
{
	DECLARE_COMPLETION_ONSTACK(tx_sent);
	u32 buf[3] = {};
	u8 *puc;
	int timeout, retries = 0;
	int ret;

retry:
	if (atomic_read(&dev->ssr_in_progress)) {
		dev_dbg(dev->dev, "REPORT_SAT: SSR in progress, aborting\n");
		return -ENODEV;
	}
	memset(buf, 0, sizeof(buf));

	/*
	 * REPORT_SATELLITE message:
	 *   Byte 0:    RL=7 | MT=6 (SRC_REFERRED_USER) << 5
	 *   Byte 1:    MC=0x01 (REPORT_SATELLITE) | DT=0 << 7
	 *   Byte 2:    LA = SLIM_LA_MGR (0xFF)
	 *   Bytes 3-6: magic_lsb, magic_msb, ver, prot
	 */
	buf[0] = SLIM_MSG_ASM_FIRST_WORD(7,
					 SLIM_MSG_MT_SRC_REFERRED_USER,
					 SLIM_USR_MC_REPORT_SATELLITE,
					 SLIM_MSG_DEST_LOGICALADDR,
					 SLIM_LA_MGR);
	puc = ((u8 *)buf) + 3;
	*puc++ = SAT_MAGIC_LSB;
	*puc++ = SAT_MAGIC_MSB;
	*puc++ = SAT_MSG_VER;
	*puc++ = SAT_MSG_PROT;

	msm8953_slim_set_wr_comp(dev, &tx_sent);
	dev->err = 0;

	/*
	 * BAM TX is the normal AP to ADSP control path. Without it, the
	 * message is written to NGD_TX_MSG; the NGD ignores those writes
	 * while TX_MSGQ_EN is set, so the bit is cleared around the write.
	 */
	if (dev->use_bam_tx) {
		ret = msm8953_slim_bam_tx(dev, buf, 7);
	} else {
		void __iomem *ngd = ngd_base(dev);
		u32 cfg = readl_relaxed(ngd + NGD_CFG);

		writel_relaxed(cfg & ~NGD_CFG_TX_MSGQ_EN, ngd + NGD_CFG);
		/* TX_MSGQ_EN must be clear before writing NGD_TX_MSG */
		mb();
		ret = msm8953_slim_ahb_tx(dev, buf, 7,
					  NGD_BASE(dev->ctrl_nr) + NGD_TX_MSG);
		writel_relaxed(cfg | NGD_CFG_TX_MSGQ_EN, ngd + NGD_CFG);
		/* Restore TX_MSGQ_EN before waiting for TX completion */
		mb();
	}
	if (ret)
		goto out;

	timeout = wait_for_completion_timeout(&tx_sent, HZ);
	if (!timeout) {
		dev_dbg(dev->dev, "REPORT_SATELLITE TX timeout, retry %d\n",
			retries);
		if (retries < INIT_MX_RETRIES) {
			reinit_completion(&tx_sent);
			retries++;
			goto retry;
		}
		ret = -ETIMEDOUT;
		goto out;
	}

	if (dev->err) {
		dev_dbg(dev->dev, "REPORT_SATELLITE TX NACKed: %d, retry %d\n",
			dev->err, retries);
		if (retries < INIT_MX_RETRIES) {
			msleep(DEF_RETRY_MS);
			reinit_completion(&tx_sent);
			retries++;
			goto retry;
		}
		ret = dev->err;
		goto out;
	}

	dev_dbg(dev->dev, "REPORT_SATELLITE sent\n");

out:
	msm8953_slim_set_wr_comp(dev, NULL);
	return ret;
}

/* ---- Power-up sequence -------------------------------------------------- */

/*
 * Satellite power-up, in the order the ADSP expects:
 *  1. Send QMI PM_ACTIVE
 *  2. Configure and enable the NGD
 *  3. Wait for MASTER_CAPABILITY from the ADSP
 *  4. Send REPORT_SATELLITE
 *
 * Called with tx_lock held.
 */
static int msm8953_slim_power_up(struct msm8953_slim_ctrl *dev)
{
	void __iomem *ngd;
	int timeout, ret = 0;
	u32 laddr;
	int mcap_retries = 0;

	ret = msm8953_slim_qmi_power_request(dev, true);
	if (ret) {
		dev_err(dev->dev, "SLIM QMI PM_ACTIVE failed: %d\n", ret);
		return ret;
	}
	dev_dbg(dev->dev, "QMI PM_ACTIVE sent\n");

	ngd = ngd_base(dev);
	laddr = readl_relaxed(ngd + NGD_STATUS);

	if (laddr & NGD_LADDR)
		dev_dbg(dev->dev, "NGD already has LADDR, stat=0x%x\n",
			laddr);

capability_retry:
	reinit_completion(&dev->reconf);
	dev->mcap_gen = atomic_read(&dev->ssr_gen);

	ret = msm8953_slim_init_dma(dev);
	if (ret)
		dev_warn(dev->dev, "DMA init failed: %d\n", ret);

	usleep_range(1000, 2000);

	ret = msm8953_slim_ngd_setup(dev);
	if (ret == -EAGAIN) {
		if (mcap_retries < INIT_MX_RETRIES &&
		    !atomic_read(&dev->ssr_in_progress)) {
			mcap_retries++;
			msm8953_slim_teardown_dma(dev);
			msleep(100);
			goto capability_retry;
		}
		return -ETIMEDOUT;
	}

	/*
	 * The audio PDR service can report UP before the ADSP SLIMbus
	 * manager is ready, notably on cold boot after an early ADSP
	 * restart, so the first MASTER_CAPABILITY wait is 3 s. Later
	 * attempts wait 1 s.
	 */
	{
		unsigned long initial_to = (mcap_retries == 0) ? 3 * HZ : HZ;

		timeout = wait_for_completion_timeout(&dev->reconf, initial_to);
	}
	if (atomic_read(&dev->ssr_in_progress))
		return -ENODEV;

	if (!timeout) {
		u32 cfg  = readl_relaxed(ngd + NGD_CFG);
		u32 stat = readl_relaxed(ngd + NGD_STATUS);

		dev_dbg(dev->dev,
			"MASTER_CAPABILITY timeout (attempt %d), stat=0x%x cfg=0x%x\n",
			mcap_retries, stat, cfg);
		if (mcap_retries < INIT_MX_RETRIES &&
		    !atomic_read(&dev->ssr_in_progress)) {
			mcap_retries++;
			if (cfg == 0) {
				/*
				 * The ADSP reset the SLIMbus block after NGD
				 * setup. Re-send PM_ACTIVE to wake the ADSP
				 * SLIMbus manager and give it 500 ms to recover
				 * before retrying, without DMA teardown. Matches
				 * the vendor NGD driver.
				 */
				dev_dbg(dev->dev,
					"NGD_CFG cleared by ADSP, re-sending PM_ACTIVE\n");
				msm8953_slim_qmi_power_request(dev, true);
				msleep(500);
			}
			goto capability_retry;
		}
		return -ETIMEDOUT;
	}

	ret = msm8953_slim_report_satellite(dev);
	if (ret) {
		dev_warn(dev->dev, "REPORT_SAT failed: %d\n", ret);
		return ret;
	}

	if (atomic_read(&dev->ssr_in_progress)) {
		dev_dbg(dev->dev, "SSR during power-up, aborting\n");
		return -ENODEV;
	}

	dev->ctrl.sched.clk_state = SLIM_CLK_ACTIVE;
	complete_all(&dev->ctrl_up);
	schedule_work(&dev->slave_notify_work);

	return 0;
}

/* ---- Mainline slimbus controller ops ------------------------------------ */

static int msm8953_slim_xfer_msg(struct slim_controller *ctrl,
				 struct slim_msg_txn *txn)
{
	DECLARE_COMPLETION_ONSTACK(done);
	DECLARE_COMPLETION_ONSTACK(tx_sent);
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);
	u32 *pbuf;
	u8 *puc;
	int ret = 0;
	u8 la = txn->la;
	u16 txn_mc = txn->mc;
	u8 txn_mt = txn->mt;
	u8 wbuf[SLIM_MSGQ_BUF_LEN];
	int timeout;

	memset(wbuf, 0, sizeof(wbuf));

	/*
	 * CORE reconfiguration messages.
	 *
	 * The ADSP manager does not accept CORE reconfiguration from the
	 * satellite, only the USR protocol. Channel setup is done with USR
	 * messages in enable_stream, so the CORE BEGIN/DEFINE/ACTIVATE
	 * messages are dropped.
	 *
	 * Channel teardown (BEGIN_RECONFIGURATION, NEXT_DEACTIVATE_CHANNEL,
	 * NEXT_REMOVE_CHANNEL, RECONFIGURE_NOW from slim_stream_disable) is
	 * translated to the USR messages the vendor driver sends on stop:
	 *   CHAN_CTRL(REMOVE) mc=0x23 [(SLIM_CH_REMOVE << 6) | laddr, tid, chan]
	 *   RECONFIG_NOW      mc=0x24 [tid, laddr]
	 * NEXT_DEACTIVATE_CHANNEL is covered by CHAN_CTRL(REMOVE). If the
	 * channel is not removed, the ADSP keeps it active and it fails to
	 * come up again on the next stream start.
	 */
	if (txn->mt == SLIM_MSG_MT_CORE &&
	    txn->mc >= SLIM_MSG_MC_BEGIN_RECONFIGURATION &&
	    txn->mc <= SLIM_MSG_MC_RECONFIGURE_NOW) {
		/*
		 * Nested transfers are safe here: tx_lock is not held yet,
		 * and xfer_msg_sync releases it before returning.
		 */
		if (txn->mc == SLIM_MSG_MC_NEXT_REMOVE_CHANNEL &&
		    txn->msg && txn->msg->wbuf) {
			u8 cbuf[3];
			struct slim_val_inf cmsg = { 0, 3, NULL, cbuf, NULL };
			struct slim_msg_txn ctxn = { 0 };
			DECLARE_COMPLETION_ONSTACK(cdone);

			ctxn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
			ctxn.dt = SLIM_MSG_DEST_LOGICALADDR;
			ctxn.la = SLIM_LA_MGR;
			ctxn.mc = SLIM_USR_MC_CHAN_CTRL;
			ctxn.msg = &cmsg;
			cbuf[0] = (2 << 6) | (txn->la & 0x1f); /* SLIM_CH_REMOVE action */
			cbuf[2] = txn->msg->wbuf[0];           /* ch.id */
			if (!slim_alloc_txn_tid(ctrl, &ctxn)) {
				cbuf[1] = ctxn.tid;
				ctxn.rl = cmsg.num_bytes + 4;
				ctxn.comp = &cdone;
				dev_dbg(dev->dev,
					 "chan_teardown: CHAN_CTRL remove chan=%d [%02x %02x %02x]\n",
					 cbuf[2], cbuf[0], cbuf[1], cbuf[2]);
				if (msm8953_slim_xfer_msg_sync(ctrl, &ctxn))
					slim_free_txn_tid(ctrl, &ctxn);
			}
			dev->teardown_reconfig_pending = true;
			dev->teardown_laddr = txn->la;
			return 0;
		}
		if (txn->mc == SLIM_MSG_MC_RECONFIGURE_NOW &&
		    dev->teardown_reconfig_pending) {
			u8 rbuf[2];
			struct slim_val_inf rmsg = { 0, 2, NULL, rbuf, NULL };
			struct slim_msg_txn rtxn = { 0 };
			DECLARE_COMPLETION_ONSTACK(rdone);

			rtxn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
			rtxn.dt = SLIM_MSG_DEST_LOGICALADDR;
			rtxn.la = SLIM_LA_MGR;
			rtxn.mc = SLIM_USR_MC_RECONFIG_NOW;
			rtxn.msg = &rmsg;
			rbuf[1] = dev->teardown_laddr;
			if (!slim_alloc_txn_tid(ctrl, &rtxn)) {
				rbuf[0] = rtxn.tid;
				rtxn.rl = rmsg.num_bytes + 4;
				rtxn.comp = &rdone;
				dev_dbg(dev->dev,
					 "chan_teardown: RECONFIG_NOW laddr=0x%x\n",
					 dev->teardown_laddr);
				if (msm8953_slim_xfer_msg_sync(ctrl, &rtxn))
					slim_free_txn_tid(ctrl, &rtxn);
			}
			dev->teardown_reconfig_pending = false;
			return 0;
		}
		/* Channel setup, BEGIN and non-teardown RECONFIGURE_NOW */
		return 0;
	}

	/*
	 * CORE CHANGE_VALUE and REQUEST_VALUE are sent unmodified, as the
	 * vendor NGD driver does: the NGD emits them as standard SLIMbus
	 * frames that the codec processes. The ADSP has no handler for the
	 * USR REPEAT_CHANGE_VALUE equivalent.
	 */

	/* Wake the bus; runtime resume sends QMI PM_ACTIVE to the ADSP */
	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0 && ret != -EACCES) {
		pm_runtime_put_noidle(dev->dev);
		return ret;
	}

	mutex_lock(&dev->tx_lock);

	/* Wait for bus to come up if it's down */
	if (dev->state == MSM8953_SLIM_DOWN) {
		u8 mc = (u8)txn->mc;

		mutex_unlock(&dev->tx_lock);

		/* Channel management messages can't wait */
		if (txn->mt == SLIM_MSG_MT_DEST_REFERRED_USER &&
		    (mc == SLIM_USR_MC_CHAN_CTRL ||
		     mc == SLIM_USR_MC_DISCONNECT_PORT ||
		     mc == SLIM_USR_MC_RECONFIG_NOW))
			return -EREMOTEIO;

		dev_dbg(dev->dev, "ADSP slimbus not up yet, waiting...\n");
		timeout = wait_for_completion_timeout(&dev->ctrl_up, HZ);
		if (!timeout)
			return -ETIMEDOUT;
		mutex_lock(&dev->tx_lock);
	}

	/* Translate CORE connect/disconnect to USR messages for satellite */
	if (txn->mt == SLIM_MSG_MT_CORE &&
	    (txn->mc == SLIM_MSG_MC_CONNECT_SOURCE ||
	     txn->mc == SLIM_MSG_MC_CONNECT_SINK ||
	     txn->mc == SLIM_MSG_MC_DISCONNECT_PORT)) {
		int i = 0;

		txn->mt = SLIM_MSG_MT_DEST_REFERRED_USER;
		switch (txn->mc) {
		case SLIM_MSG_MC_CONNECT_SOURCE:
			txn->mc = SLIM_USR_MC_CONNECT_SRC;
			break;
		case SLIM_MSG_MC_CONNECT_SINK:
			txn->mc = SLIM_USR_MC_CONNECT_SINK;
			break;
		case SLIM_MSG_MC_DISCONNECT_PORT:
			txn->mc = SLIM_USR_MC_DISCONNECT_PORT;
			break;
		default:
			ret = -EINVAL;
			goto xfer_err;
		}

		/*
		 * USR CONNECT payload: [codec_la, codec_port, chan, tid],
		 * using the codec device LA and port numbers. The message
		 * is addressed to the manager; addressing it to the codec
		 * LA gets no ACK.
		 */
		{
			u8 slim_port = txn->msg->wbuf[0];

			wbuf[i++] = txn->la;
			la = SLIM_LA_MGR;
			wbuf[i++] = slim_port;
		}
		if (txn->mc != SLIM_USR_MC_DISCONNECT_PORT)
			wbuf[i++] = txn->msg->wbuf[1]; /* channel number */

		/*
		 * Keep a caller-provided completion: the caller may wait on
		 * txn->comp after this function returns, when the on-stack
		 * `done` is gone.
		 */
		if (!txn->comp)
			txn->comp = &done;
		ret = slim_alloc_txn_tid(ctrl, txn);
		if (ret) {
			dev_err(dev->dev, "TID alloc failed: %d\n", ret);
			goto xfer_err;
		}

		wbuf[i++] = txn->tid;
		txn->msg->num_bytes = i;
		txn->msg->wbuf = wbuf;
		txn->rl = txn->msg->num_bytes + 4;

		dev_dbg(dev->dev,
			"CONNECT: mc=0x%x la=%d port=%d chan=%d tid=%d\n",
			txn->mc, wbuf[0], wbuf[1],
			(txn->mc != SLIM_USR_MC_DISCONNECT_PORT) ? wbuf[2] : -1,
			txn->tid);
	}

	txn->rl--;

	if (txn->msg->num_bytes > SLIM_MSGQ_BUF_LEN || txn->rl > SLIM_MSGQ_BUF_LEN) {
		dev_warn(dev->dev, "msg exceeds HW limit: num_bytes=%d rl=%d\n",
			 txn->msg->num_bytes, txn->rl);
		ret = -EDQUOT;
		goto xfer_err;
	}

	pbuf = dev->tx_buf;
	msm8953_slim_set_wr_comp(dev, &tx_sent);
	dev->err = 0;

	/* Assemble message header */
	if (txn->dt == SLIM_MSG_DEST_LOGICALADDR)
		*pbuf = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt,
						txn->mc, 0, la);
	else
		*pbuf = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt,
						txn->mc, 1, la);

	puc = (txn->dt == SLIM_MSG_DEST_LOGICALADDR) ?
		((u8 *)pbuf) + 3 : ((u8 *)pbuf) + 2;

	if (slim_tid_txn(txn->mt, txn->mc))
		*(puc++) = txn->tid;

	if (slim_ec_txn(txn->mt, txn->mc) ||
	    (txn->mt == SLIM_MSG_MT_DEST_REFERRED_USER &&
	     txn->mc == SLIM_USR_MC_REPEAT_CHANGE_VALUE)) {
		*(puc++) = txn->ec & 0xFF;
		*(puc++) = (txn->ec >> 8) & 0xFF;
		if (txn->mc == SLIM_MSG_MC_REQUEST_VALUE)
			dev_dbg(dev->dev, "REQ_VALUE: la=%d ec=0x%04x len=%d\n",
				 la, txn->ec, txn->msg ? txn->msg->num_bytes : 0);
		else if (txn->mc == SLIM_USR_MC_REPEAT_CHANGE_VALUE)
			dev_dbg(dev->dev, "REPEAT_CHG_VALUE: la=%d ec=0x%04x len=%d\n",
				 la, txn->ec, txn->msg ? txn->msg->num_bytes : 0);
	}

	if (txn->msg && txn->msg->wbuf)
		memcpy(puc, txn->msg->wbuf, txn->msg->num_bytes);

	txn_mc = txn->mc;
	txn_mt = txn->mt;

#ifdef CONFIG_DEBUG_FS
	{
		u8 *tb = (u8 *)pbuf;

		msm8953_slim_log_msg(dev, 0, tb, min_t(u8, txn->rl, 16));
	}
#endif

	/* BAM TX when available, otherwise the NGD_TX_MSG registers */
	if (dev->use_bam_tx) {
		ret = msm8953_slim_bam_tx(dev, pbuf, txn->rl);
	} else {
		ret = msm8953_slim_ahb_tx(dev, pbuf, txn->rl,
					  NGD_BASE(dev->ctrl_nr) + NGD_TX_MSG);
	}
	if (!ret) {
		timeout = wait_for_completion_timeout(&tx_sent, HZ);
		if (!timeout) {
			dev_warn(dev->dev, "TX timeout: mc=0x%x mt=0x%x\n",
				 txn_mc, txn_mt);
			ret = -ETIMEDOUT;
		} else {
			ret = dev->err;
		}
	}

	msm8953_slim_set_wr_comp(dev, NULL);

	if (ret) {
		/*
		 * The codec PGD NACKs REQUEST_VALUE reads while audio is
		 * streaming, so those are not warnings. Other failures are
		 * rate-limited since some boards NACK many control writes
		 * transiently during codec init.
		 */
		if (txn_mt == SLIM_MSG_MT_CORE &&
		    txn_mc == SLIM_MSG_MC_REQUEST_VALUE)
			dev_dbg_ratelimited(dev->dev,
				"REQUEST_VALUE failed: mc=0x%x mt=0x%x ret=%d\n",
				txn_mc, txn_mt, ret);
		else
			dev_warn_ratelimited(dev->dev,
				"TX failed: mc=0x%x mt=0x%x ret=%d\n",
				txn_mc, txn_mt, ret);
	}

	/* Wait for the ADSP ACK on CONNECT/DISCONNECT */
	if (!ret && txn->comp &&
	    txn_mt == SLIM_MSG_MT_DEST_REFERRED_USER &&
	    (txn_mc == SLIM_USR_MC_CONNECT_SRC ||
	     txn_mc == SLIM_USR_MC_CONNECT_SINK ||
	     txn_mc == SLIM_USR_MC_DISCONNECT_PORT)) {
		int ack_timeout = wait_for_completion_timeout(txn->comp,
							     msecs_to_jiffies(200));
		if (!ack_timeout) {
			dev_warn(dev->dev,
				 "CONNECT ACK timeout: mc=0x%x tid=%d la=%d\n",
				 txn_mc, txn->tid,
				 txn->msg->wbuf ? txn->msg->wbuf[0] : -1);
			slim_free_txn_tid(ctrl, txn);
			ret = 0; /* non-fatal */
		} else {
			dev_dbg(dev->dev, "CONNECT ACK: mc=0x%x tid=%d\n",
				txn_mc, txn->tid);
		}

		/*
		 * PGD ports and BAM pipes for audio data are configured by
		 * the ADSP in response to CONNECT and DEF_ACT_CHAN; writing
		 * PGD_PORT_CFGn from the AP for those ports conflicts with
		 * it and causes I/O errors.
		 */
	}

xfer_err:
	mutex_unlock(&dev->tx_lock);
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
	return ret ? ret : dev->err;
}

static int msm8953_slim_set_laddr(struct slim_controller *ctrl,
				  struct slim_eaddr *ea, u8 laddr)
{
	/* ADSP assigns logical addresses; AP just acknowledges */
	return 0;
}

static int msm8953_slim_get_laddr(struct slim_controller *ctrl,
				  struct slim_eaddr *ea, u8 *laddr)
{
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);
	DECLARE_COMPLETION_ONSTACK(done);
	DECLARE_COMPLETION_ONSTACK(tx_sent);
	struct slim_msg_txn txn = {};
	u32 buf[4] = {};
	u8 *puc;
	int ret, timeout;
	u8 ea_bytes[6];

	if (dev->state != MSM8953_SLIM_AWAKE) {
		dev_dbg(dev->dev, "get_laddr: bus not up yet, deferring\n");
		return -EBUSY;
	}

	/*
	 * Send ADDR_QUERY to the ADSP manager. The ADSP looks up the
	 * enumeration address and returns ADDR_REPLY with the logical
	 * address. The query also opens the device on the ADSP side, which
	 * is required before VALUE messages to it are accepted.
	 */
	txn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
	txn.mc = SLIM_USR_MC_ADDR_QUERY;
	txn.dt = SLIM_MSG_DEST_LOGICALADDR;
	txn.la = SLIM_LA_MGR;
	txn.comp = &done;

	ret = slim_alloc_txn_tid(ctrl, &txn);
	if (ret) {
		dev_err(dev->dev, "ADDR_QUERY: TID alloc failed: %d\n", ret);
		return ret;
	}

	/*
	 * Enumeration address bytes:
	 *   [instance, dev_index, prod_code_lo, prod_code_hi,
	 *    manf_id_lo, manf_id_hi]
	 * e.g. WCD9335 codec: [00 01 a0 01 17 02]
	 */
	ea_bytes[0] = ea->instance;
	ea_bytes[1] = ea->dev_index;
	ea_bytes[2] = ea->prod_code & 0xFF;
	ea_bytes[3] = (ea->prod_code >> 8) & 0xFF;
	ea_bytes[4] = ea->manf_id & 0xFF;
	ea_bytes[5] = (ea->manf_id >> 8) & 0xFF;

	/*
	 * ADDR_QUERY message format:
	 *   Byte 0:  RL=10 | MT=6<<5
	 *   Byte 1:  MC=0x0D | DT=0<<7
	 *   Byte 2:  LA=0xFF (manager)
	 *   Byte 3:  TID
	 *   Bytes 4-9: EA (6 bytes)
	 * Total: 10 bytes (rl=10 in first word, DMA len rounds to 12)
	 */
	buf[0] = SLIM_MSG_ASM_FIRST_WORD(10,
					  SLIM_MSG_MT_DEST_REFERRED_USER,
					  SLIM_USR_MC_ADDR_QUERY,
					  SLIM_MSG_DEST_LOGICALADDR,
					  SLIM_LA_MGR);
	puc = ((u8 *)buf) + 3;
	*puc++ = txn.tid;
	memcpy(puc, ea_bytes, 6);

	dev_dbg(dev->dev, "ADDR_QUERY: ea=%04x:%04x:%d:%d tid=%d %10ph\n",
		ea->manf_id, ea->prod_code, ea->dev_index, ea->instance,
		txn.tid, buf);

	msm8953_slim_set_wr_comp(dev, &tx_sent);
	dev->err = 0;

	if (dev->use_bam_tx) {
		ret = msm8953_slim_bam_tx(dev, buf, 10);
	} else {
		ret = msm8953_slim_ahb_tx(dev, buf, 10,
					  NGD_BASE(dev->ctrl_nr) + NGD_TX_MSG);
	}
	if (ret) {
		dev_err(dev->dev, "ADDR_QUERY TX failed: %d\n", ret);
		goto out_free_tid;
	}

	/* Wait for TX completion */
	timeout = wait_for_completion_timeout(&tx_sent, HZ);
	if (!timeout) {
		dev_warn(dev->dev, "ADDR_QUERY TX timeout\n");
		ret = -ETIMEDOUT;
		goto out_free_tid;
	}

	msm8953_slim_set_wr_comp(dev, NULL);

	/* Wait for ADDR_REPLY from ADSP */
	timeout = wait_for_completion_timeout(&done, 2 * HZ);
	if (!timeout) {
		dev_dbg(dev->dev, "ADDR_QUERY: no ADDR_REPLY for %04x:%04x:%d\n",
			ea->manf_id, ea->prod_code, ea->dev_index);
		ret = -ETIMEDOUT;
		goto out_free_tid;
	}

	if (txn.la == 0xFF) {
		dev_dbg(dev->dev, "ADDR_QUERY: device not enumerated yet\n");
		ret = -ENXIO;
		goto out_free_tid;
	}

	*laddr = txn.la;
	dev_dbg(dev->dev, "ADDR_QUERY: %04x:%04x:%d:%d laddr=%d\n",
		ea->manf_id, ea->prod_code, ea->dev_index, ea->instance,
		*laddr);
	return 0;

out_free_tid:
	spin_lock_irq(&ctrl->txn_lock);
	idr_remove(&ctrl->tid_idr, txn.tid);
	spin_unlock_irq(&ctrl->txn_lock);
	msm8953_slim_set_wr_comp(dev, NULL);

	/*
	 * Return -EAGAIN so the retry loop in slave_notify_worker keeps
	 * trying. The WCD9335 may not have enumerated on the bus yet
	 * (needs MCLK + reset release + a few hundred ms).
	 */
	return -EAGAIN;
}

/*
 * Persistent messaging bandwidth reservation, made once after the codec has
 * been enumerated: REQ_BW followed by RECONFIG_NOW against the codec PGD
 * laddr, never released. It raises the framer's clock gear, as the vendor
 * codec driver does at start-up. Without it, the framer commits channels
 * but their data is never clocked to the codec.
 */
static int msm8953_slim_reserve_msg_bw(struct slim_controller *ctrl, u8 laddr,
				       int msgsl)
{
	u8 bbuf[4];
	struct slim_val_inf bmsg = { 0, 3, NULL, bbuf, NULL };
	struct slim_msg_txn btxn = { 0 };
	DECLARE_COMPLETION_ONSTACK(bdone);
	u8 rbuf[2];
	struct slim_val_inf rmsg = { 0, 2, NULL, rbuf, NULL };
	struct slim_msg_txn rtxn = { 0 };
	DECLARE_COMPLETION_ONSTACK(rdone);
	int ret;

	/* REQ_BW, never released */
	btxn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
	btxn.dt = SLIM_MSG_DEST_LOGICALADDR;
	btxn.la = SLIM_LA_MGR;
	btxn.mc = SLIM_USR_MC_REQ_BW;
	btxn.msg = &bmsg;
	bbuf[0] = (laddr & 0x1f) | ((msgsl & 0x7) << 5);
	bbuf[1] = (msgsl >> 3);
	ret = slim_alloc_txn_tid(ctrl, &btxn);
	if (ret)
		return ret;
	bbuf[2] = btxn.tid;
	bmsg.num_bytes = 3;
	btxn.rl = bmsg.num_bytes + 4;
	btxn.comp = &bdone;
	dev_dbg(ctrl->dev, "reserve REQ_BW: %2ph tid=%d msgsl=%d laddr=%d\n",
		bbuf, btxn.tid, msgsl, laddr);
	ret = msm8953_slim_xfer_msg_sync(ctrl, &btxn);
	if (ret) {
		slim_free_txn_tid(ctrl, &btxn);
		dev_warn(ctrl->dev, "reserve REQ_BW failed: %d\n", ret);
		return ret;
	}

	/* RECONFIG_NOW to commit the reservation */
	rtxn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
	rtxn.dt = SLIM_MSG_DEST_LOGICALADDR;
	rtxn.la = SLIM_LA_MGR;
	rtxn.mc = SLIM_USR_MC_RECONFIG_NOW;
	rtxn.msg = &rmsg;
	rbuf[1] = laddr;
	ret = slim_alloc_txn_tid(ctrl, &rtxn);
	if (ret)
		return ret;
	rbuf[0] = rtxn.tid;
	rtxn.rl = rmsg.num_bytes + 4;
	rtxn.comp = &rdone;
	dev_dbg(ctrl->dev, "reserve RECONFIG_NOW: tid=%d laddr=%d\n",
		rtxn.tid, laddr);
	ret = msm8953_slim_xfer_msg_sync(ctrl, &rtxn);
	if (ret) {
		slim_free_txn_tid(ctrl, &rtxn);
		dev_warn(ctrl->dev, "reserve RECONFIG_NOW failed: %d\n", ret);
	}
	return ret;
}

/* ---- Slave notify worker ------------------------------------------------- */

static void msm8953_slim_slave_notify_worker(struct work_struct *work)
{
	struct msm8953_slim_ctrl *dev =
		container_of(work, struct msm8953_slim_ctrl, slave_notify_work);
	struct slim_controller *ctrl = &dev->ctrl;
	struct slim_device *sbdev;
	struct device_node *node;
	int ret, j;
	u8 codec_pgd_laddr = 0;

	struct device_node *parent = dev->ngd_node ?: dev->dev->of_node;

	/*
	 * The WCD9335 needs MCLK and reset release, done by its driver
	 * probe, before it enumerates on SLIMbus. The probe runs
	 * asynchronously, so wait 5 s, then retry ADDR_QUERY up to
	 * LADDR_RETRY times at 500 ms intervals (35 s window in total).
	 */
	msleep(5000);

	/*
	 * If SSR happened during our initial sleep, wait for recovery
	 * before attempting ADDR_QUERY.
	 */
	if (dev->state == MSM8953_SLIM_DOWN) {
		dev_dbg(dev->dev, "slave_notify: SSR during init, waiting for recovery\n");
		wait_for_completion_interruptible(&dev->ctrl_up);
		msleep(2000);
	}

	for_each_child_of_node(parent, node) {
		sbdev = of_slim_get_device(ctrl, node);
		if (!sbdev)
			continue;

		for (j = 0; j < LADDR_RETRY; j++) {
			/*
			 * If SSR happened during retries, the ADSP was
			 * restarted and doesn't know this device anymore.
			 * Wait for recovery and restart the retry loop.
			 */
			if (dev->state == MSM8953_SLIM_DOWN) {
				dev_dbg(dev->dev,
					 "%s: SSR detected at attempt %d, waiting for recovery\n",
					 dev_name(&sbdev->dev), j);
				sbdev->is_laddr_valid = false;
				wait_for_completion_interruptible(&dev->ctrl_up);
				msleep(2000);
				j = 0;
				dev_dbg(dev->dev,
					 "%s: SSR recovered, restarting ADDR_QUERY\n",
					 dev_name(&sbdev->dev));
			}

			ret = slim_get_logical_addr(sbdev);
			if (!ret) {
				dev_dbg(dev->dev, "%s: got laddr %d (attempt %d)\n",
					 dev_name(&sbdev->dev), sbdev->laddr, j);
				break;
			}
			msleep(500);
		}
		if (ret) {
			/*
			 * Fall back to the codec PGD LA so the codec driver
			 * can still probe.
			 */
			struct slim_eaddr *ea = &sbdev->e_addr;

			if (ea->manf_id == 0x0217 && ea->prod_code == 0x01a0) {
				sbdev->laddr = 192 + ea->dev_index;
				sbdev->is_laddr_valid = true;
				dev_warn(dev->dev,
					 "%s: ADDR_QUERY exhausted, fallback laddr=%d\n",
					 dev_name(&sbdev->dev), sbdev->laddr);
			} else {
				dev_warn(dev->dev,
					 "laddr assignment failed for %s: %d\n",
					 dev_name(&sbdev->dev), ret);
			}
		}

		/*
		 * Notify the codec driver that the device is UP.
		 * slim_get_logical_addr() does this internally when it
		 * succeeds, but the fallback path above bypasses it.
		 * The codec's device_status callback (wcd9335_slim_status)
		 * completes initialization: regmap, IRQ, ASoC registration.
		 */
		if (sbdev->is_laddr_valid &&
		    sbdev->status != SLIM_DEVICE_STATUS_UP) {
			struct slim_driver *sbdrv;

			sbdev->status = SLIM_DEVICE_STATUS_UP;
			if (sbdev->dev.driver) {
				sbdrv = to_slim_driver(sbdev->dev.driver);
				if (sbdrv->device_status)
					sbdrv->device_status(sbdev,
							     sbdev->status);
			}
		}
		/*
		 * The codec PGD has the highest codec laddr; it is used for
		 * the messaging bandwidth reservation below.
		 */
		if (sbdev->is_laddr_valid &&
		    sbdev->e_addr.manf_id == 0x0217 &&
		    sbdev->e_addr.prod_code == 0x01a0 &&
		    sbdev->laddr > codec_pgd_laddr)
			codec_pgd_laddr = sbdev->laddr;
		put_device(&sbdev->dev);
	}

	if (codec_pgd_laddr && dev->state != MSM8953_SLIM_DOWN)
		msm8953_slim_reserve_msg_bw(&dev->ctrl, codec_pgd_laddr,
					    MSM8953_SLIM_REQBW_MSGSL);
}

/* ---- SSR / power-up worker ----------------------------------------------- */

static int msm8953_slim_ngd_enable(struct msm8953_slim_ctrl *dev)
{
	int ret;
	unsigned long timeout;

	dev_dbg(dev->dev, "ngd_enable: waiting for QMI (qmi_done=%d)\n",
		 completion_done(&dev->qmi_up));
	/*
	 * On cold boot the ADSP has to load its firmware and start its QMI
	 * service, which can take more than 10 s.
	 */
	timeout = wait_for_completion_timeout(&dev->qmi_up,
					      msecs_to_jiffies(30000));
	if (!timeout) {
		dev_err(dev->dev, "QMI service not found after 30s (state=%d)\n",
			dev->state);
		return -ETIMEDOUT;
	}
	dev_dbg(dev->dev, "ngd_enable: QMI ready\n");

	{
		int sel_retries = 0;

		do {
			ret = msm8953_slim_qmi_select_instance(dev);
			if (!ret)
				break;
			if (sel_retries < SLIMBUS_QMI_SELECT_RETRIES) {
				sel_retries++;
				msleep(SLIMBUS_QMI_SELECT_RETRY_MS);
			} else {
				dev_err(dev->dev,
					"QMI select instance failed after %d retries: %d\n",
					sel_retries, ret);
				return ret;
			}
		} while (true);
	}

	mutex_lock(&dev->tx_lock);
	ret = msm8953_slim_power_up(dev);
	if (!ret)
		dev->state = MSM8953_SLIM_AWAKE;
	else if (dev->state != MSM8953_SLIM_DOWN)
		dev->state = MSM8953_SLIM_ASLEEP;
	mutex_unlock(&dev->tx_lock);

	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put(dev->dev);
	return ret;
}

static void msm8953_slim_ngd_up_worker(struct work_struct *work)
{
	struct msm8953_slim_ctrl *dev =
		container_of(to_delayed_work(work), struct msm8953_slim_ctrl,
			     ngd_up_work);
	u32 gen;
	int ret, retries = 0;

	gen = dev->ngd_up_gen;

	dev_dbg(dev->dev, "ngd_up_worker: start (state=%d qmi=%d gen=%u)\n",
		 dev->state, completion_done(&dev->qmi_up), gen);

retry:
	if (gen != atomic_read(&dev->ssr_gen)) {
		dev_dbg(dev->dev, "ngd_up_worker: stale (gen %u != %d), bailing\n",
			 gen, atomic_read(&dev->ssr_gen));
		return;
	}

	mutex_lock(&dev->tx_lock);
	/*
	 * Leave DOWN before power-up so that an ADSP crash during power-up
	 * is handled by the SSR DOWN handler rather than ignored as a
	 * duplicate DOWN.
	 */
	dev->state = MSM8953_SLIM_ASLEEP;
	pm_runtime_disable(dev->dev);
	pm_runtime_set_suspended(dev->dev);
	pm_runtime_enable(dev->dev);
	mutex_unlock(&dev->tx_lock);

	ret = msm8953_slim_ngd_enable(dev);

	/* Slow ADSP cold boots can need several attempts */
	if (ret && retries < 5 && gen == atomic_read(&dev->ssr_gen)) {
		retries++;
		dev_warn(dev->dev,
			 "ngd_up_worker: attempt %d failed (%d), retrying in 500ms\n",
			 retries, ret);
		mutex_lock(&dev->tx_lock);
		msm8953_slim_teardown_dma(dev);
		dev->state = MSM8953_SLIM_DOWN;
		mutex_unlock(&dev->tx_lock);
		msleep(500);
		goto retry;
	}

	dev_dbg(dev->dev, "ngd_up_worker: done ret=%d (state=%d qmi=%d)\n",
		 ret, dev->state, completion_done(&dev->qmi_up));
}

/* ---- SSR / PDR notifiers ------------------------------------------------- */

static void msm8953_slim_ssr_pdr_notify(struct msm8953_slim_ctrl *dev,
					unsigned long action)
{
	switch (action) {
	case QCOM_SSR_BEFORE_SHUTDOWN:
	case SERVREG_SERVICE_STATE_DOWN:
		/*
		 * Guard against late duplicate DOWN events.
		 *
		 * SSR and PDR both fire for the same ADSP crash, and PDR
		 * DOWN can arrive after SSR recovery has already completed.
		 * Processing it would tear down a working bus.
		 *
		 * Only process DOWN when transitioning out of a non-DOWN
		 * state.  The first DOWN (SSR or PDR) does the teardown;
		 * any duplicate is ignored.
		 */
		if (dev->state == MSM8953_SLIM_DOWN) {
			dev_dbg(dev->dev,
				"SSR/PDR: DOWN ignored, already DOWN (action=%lu)\n",
				action);
			break;
		}
		dev_dbg(dev->dev, "SSR/PDR: service DOWN (action=%lu, state=%d)\n",
			 action, dev->state);
		atomic_set(&dev->ssr_in_progress, 1);
		atomic_inc(&dev->ssr_gen);
		dev->state = MSM8953_SLIM_DOWN;
		cancel_delayed_work(&dev->ngd_up_work);
		complete_all(&dev->reconf);
		complete_all(&dev->ctrl_up);
		reinit_completion(&dev->ctrl_up);
		/*
		 * qmi_up is owned by the QMI del_server/new_server
		 * callbacks; reinitializing it here would race with a
		 * new_server that has already completed it.
		 */

		/*
		 * Disable the NGD and its interrupts so that stale
		 * MASTER_CAPABILITY or other IRQs from the crashed ADSP
		 * do not reach the recovery path.
		 */
		{
			void __iomem *ngd = ngd_base(dev);

			writel_relaxed(0, ngd + NGD_CFG);
			writel_relaxed(0, ngd + NGD_INT_EN);
			/* Quiesce the NGD before tearing down DMA */
			mb();
		}

		msm8953_slim_teardown_dma(dev);
		break;
	case QCOM_SSR_AFTER_POWERUP:
	case SERVREG_SERVICE_STATE_UP:
		if (dev->state != MSM8953_SLIM_DOWN) {
			dev_dbg(dev->dev,
				"SSR/PDR: UP ignored (action=%lu, state=%d)\n",
				action, dev->state);
			break;
		}
		dev_dbg(dev->dev, "SSR/PDR: service UP (action=%lu)\n",
			 action);
		atomic_set(&dev->ssr_in_progress, 0);
		dev->ngd_up_gen = atomic_read(&dev->ssr_gen);
		reinit_completion(&dev->ctrl_up);
		schedule_delayed_work(&dev->ngd_up_work, 0);
		break;
	default:
		break;
	}
}

static int msm8953_slim_ssr_notifier(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct msm8953_slim_ctrl *dev =
		container_of(nb, struct msm8953_slim_ctrl, nb);

	msm8953_slim_ssr_pdr_notify(dev, event);
	return NOTIFY_DONE;
}

static void msm8953_slim_pd_status(int state, char *svc_path, void *priv)
{
	struct msm8953_slim_ctrl *dev = priv;

	dev_dbg(dev->dev, "PDR status: state=%d path=%s\n", state,
		 svc_path ?: "null");
	msm8953_slim_ssr_pdr_notify(dev, state);
}

/* ---- Runtime PM ---------------------------------------------------------- */

static int msm8953_slim_runtime_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm8953_slim_ctrl *dev = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&dev->tx_lock);
	if (dev->state == MSM8953_SLIM_DOWN) {
		mutex_unlock(&dev->tx_lock);
		return 0;
	}

	if (dev->state >= MSM8953_SLIM_ASLEEP) {
		/*
		 * Only send QMI PM_ACTIVE to wake the ADSP SLIMbus
		 * manager. The ADSP keeps the satellite registration from
		 * the initial power-up and does not expect a new
		 * negotiation, so reprogramming NGD_CFG here would lead to
		 * MASTER_CAPABILITY timeouts. Matches the vendor NGD driver.
		 */
		ret = msm8953_slim_qmi_power_request(dev, true);
		if (ret)
			dev_warn(device, "QMI PM_ACTIVE on resume failed: %d\n",
				 ret);
	}

	if (!ret) {
		dev->state = MSM8953_SLIM_AWAKE;
		dev->ctrl.sched.clk_state = SLIM_CLK_ACTIVE;
	} else {
		if (dev->state != MSM8953_SLIM_DOWN)
			dev->state = MSM8953_SLIM_ASLEEP;
		else
			dev_err(device, "HW wakeup attempt during SSR\n");
	}
	mutex_unlock(&dev->tx_lock);
	dev_dbg(device, "slim runtime resume: %d\n", ret);
	return 0;
}

static int msm8953_slim_runtime_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm8953_slim_ctrl *dev = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&dev->tx_lock);
	ret = msm8953_slim_qmi_power_request(dev, false);
	if (!ret || ret == -ETIMEDOUT)
		dev->state = MSM8953_SLIM_ASLEEP;
	mutex_unlock(&dev->tx_lock);
	dev_dbg(device, "slim runtime suspend: %d\n", ret);
	return ret;
}

static int msm8953_slim_runtime_idle(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm8953_slim_ctrl *dev = platform_get_drvdata(pdev);

	mutex_lock(&dev->tx_lock);
	if (dev->state == MSM8953_SLIM_AWAKE)
		dev->state = MSM8953_SLIM_IDLE;
	mutex_unlock(&dev->tx_lock);
	pm_request_autosuspend(device);
	return -EAGAIN;
}

/* ---- debugfs ------------------------------------------------------------- */

#ifdef CONFIG_DEBUG_FS
static void msm8953_slim_log_msg(struct msm8953_slim_ctrl *dev,
				 u8 dir, u8 *buf, u8 len)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->msg_log_lock, flags);
	dev->msg_log[dev->msg_log_head].timestamp = ktime_get_ns();
	dev->msg_log[dev->msg_log_head].dir = dir;
	dev->msg_log[dev->msg_log_head].len = len;
	memcpy(dev->msg_log[dev->msg_log_head].data, buf, min_t(u8, len, 16));
	dev->msg_log_head = (dev->msg_log_head + 1) % SLIM_MSG_LOG_SIZE;
	spin_unlock_irqrestore(&dev->msg_log_lock, flags);
}

static const char *msm8953_slim_state_str(enum msm8953_slim_state state)
{
	switch (state) {
	case MSM8953_SLIM_AWAKE:  return "AWAKE";
	case MSM8953_SLIM_IDLE:   return "IDLE";
	case MSM8953_SLIM_ASLEEP: return "ASLEEP";
	case MSM8953_SLIM_DOWN:   return "DOWN";
	default:                  return "UNKNOWN";
	}
}

static int msm8953_slim_status_show(struct seq_file *s, void *unused)
{
	struct msm8953_slim_ctrl *dev = s->private;

	seq_printf(s, "state: %s\n", msm8953_slim_state_str(dev->state));
	seq_printf(s, "qmi: %s\n",
		   completion_done(&dev->qmi_up) ? "up" : "down");
	seq_printf(s, "ctrl: %s\n",
		   completion_done(&dev->ctrl_up) ? "up" : "down");
	seq_printf(s, "bam_tx: %s\n", dev->use_bam_tx ? "yes" : "no");
	seq_printf(s, "pgdla: 0x%02x\n", dev->pgdla);
	seq_printf(s, "apps_pipes: 0x%08x\n", dev->apps_pipes);
	seq_printf(s, "ssr: %s\n",
		   atomic_read(&dev->ssr_in_progress) ? "yes" : "no");

	/* PGD port registers (MMIO) */
	if (dev->state != MSM8953_SLIM_DOWN) {
		int p;

		seq_puts(s, "\nPGD ports:\n");
		for (p = 0; p < 6; p++) {
			u32 cfg = readl_relaxed(dev->base + 0x14000 + p * 0x1000);
			u32 stat = readl_relaxed(dev->base + 0x14004 + p * 0x1000);

			if (cfg || stat)
				seq_printf(s, "  port %d: CFG=0x%08x STAT=0x%08x\n",
					   p, cfg, stat);
		}
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(msm8953_slim_status);

static int msm8953_slim_ngd_regs_show(struct seq_file *s, void *unused)
{
	struct msm8953_slim_ctrl *dev = s->private;
	void __iomem *ngd;

	if (dev->state == MSM8953_SLIM_DOWN) {
		seq_puts(s, "NGD is DOWN\n");
		return 0;
	}

	ngd = ngd_base(dev);
	seq_printf(s, "NGD_CFG:        0x%08x\n", readl_relaxed(ngd + 0x0));
	seq_printf(s, "NGD_STATUS:     0x%08x\n", readl_relaxed(ngd + 0x4));
	seq_printf(s, "NGD_RX_MSGQ_CFG:0x%08x\n", readl_relaxed(ngd + 0x8));
	seq_printf(s, "NGD_INT_EN:     0x%08x\n", readl_relaxed(ngd + 0x10));
	seq_printf(s, "NGD_INT_STAT:   0x%08x\n", readl_relaxed(ngd + 0x14));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(msm8953_slim_ngd_regs);

static int msm8953_slim_msg_log_show(struct seq_file *s, void *unused)
{
	struct msm8953_slim_ctrl *dev = s->private;
	typeof(&dev->msg_log[0]) log_copy;
	unsigned long flags;
	unsigned int head, i, idx;

	log_copy = kmalloc(sizeof(dev->msg_log), GFP_KERNEL);
	if (!log_copy)
		return -ENOMEM;

	spin_lock_irqsave(&dev->msg_log_lock, flags);
	memcpy(log_copy, dev->msg_log, sizeof(dev->msg_log));
	head = dev->msg_log_head;
	spin_unlock_irqrestore(&dev->msg_log_lock, flags);

	for (i = 0; i < SLIM_MSG_LOG_SIZE; i++) {
		u64 ts;
		u8 plen;

		idx = (head + i) % SLIM_MSG_LOG_SIZE;
		ts = log_copy[idx].timestamp;
		if (!ts)
			continue;
		plen = min_t(u8, log_copy[idx].len, 16);
		seq_printf(s, "[%llu.%06llu] %s %*ph\n",
			   ts / 1000000000ULL,
			   (ts % 1000000000ULL) / 1000ULL,
			   log_copy[idx].dir ? "RX" : "TX",
			   plen, log_copy[idx].data);
	}
	kfree(log_copy);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(msm8953_slim_msg_log);
#endif /* CONFIG_DEBUG_FS */

/* ---- Probe / remove ------------------------------------------------------ */

static int msm8953_slim_probe(struct platform_device *pdev)
{
	struct msm8953_slim_ctrl *dev;
	struct resource *slim_mem, *bam_mem;
	int ret, irq, bam_irq;

	slim_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"slimbus_physical");
	if (!slim_mem) {
		dev_err(&pdev->dev, "no slimbus_physical memory resource\n");
		return -ENODEV;
	}

	bam_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					       "slimbus_bam_physical");
	if (!bam_mem) {
		dev_err(&pdev->dev, "no slimbus_bam_physical memory resource\n");
		return -ENODEV;
	}

	irq = platform_get_irq_byname(pdev, "slimbus_irq");
	if (irq < 0) {
		dev_err(&pdev->dev, "no slimbus_irq resource\n");
		return irq;
	}

	bam_irq = platform_get_irq_byname(pdev, "slimbus_bam_irq");
	if (bam_irq < 0) {
		dev_err(&pdev->dev, "no slimbus_bam_irq resource\n");
		return bam_irq;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, dev);

	dev->base = devm_ioremap(&pdev->dev, slim_mem->start,
				 resource_size(slim_mem));
	if (!dev->base) {
		dev_err(&pdev->dev, "ioremap slimbus_physical failed\n");
		return -ENOMEM;
	}

	dev->bam_base = devm_ioremap(&pdev->dev, bam_mem->start,
				     resource_size(bam_mem));
	if (!dev->bam_base) {
		dev_err(&pdev->dev, "ioremap slimbus_bam_physical failed\n");
		return -ENOMEM;
	}

	dev->irq     = irq;
	dev->bam_irq = bam_irq;
	dev->ee      = MSM_SLIM_EE;

	ret = of_property_read_u32(pdev->dev.of_node, "cell-index",
				   &dev->ctrl_nr);
	if (ret) {
		dev_err(&pdev->dev, "cell-index not specified: %d\n", ret);
		return ret;
	}

	of_property_read_u32(pdev->dev.of_node, "qcom,apps-ch-pipes",
			     &dev->apps_pipes);
	of_property_read_u32(pdev->dev.of_node, "qcom,ea-pc",
			     &dev->eapc);

	dev_dbg(&pdev->dev,
		 "MSM8953 SLIMbus: ctrl=%d ee=%d apps_pipes=0x%x eapc=0x%x\n",
		 dev->ctrl_nr, dev->ee, dev->apps_pipes, dev->eapc);

	init_completion(&dev->reconf);
	init_completion(&dev->ctrl_up);
	init_completion(&dev->qmi_up);
	mutex_init(&dev->tx_lock);
	spin_lock_init(&dev->rx_lock);
	spin_lock_init(&dev->wr_lock);
	memset(dev->pipe_map, 0xFF, sizeof(dev->pipe_map));
	dev->pipe_alloc = 0;
	atomic_set(&dev->ssr_in_progress, 0);
	atomic_set(&dev->ssr_gen, 0);

	dev->state  = MSM8953_SLIM_DOWN;
	dev->pgdla  = SLIM_LA_MGR;

	dev->ctrl.set_laddr      = msm8953_slim_set_laddr;
	dev->ctrl.get_laddr      = msm8953_slim_get_laddr;
	dev->ctrl.xfer_msg       = msm8953_slim_xfer_msg;
	dev->ctrl.enable_stream  = msm8953_slim_enable_stream;
	dev->ctrl.disable_stream = msm8953_slim_disable_stream;
	dev->ctrl.alloc_port     = msm8953_slim_alloc_port;
	dev->ctrl.dealloc_port   = msm8953_slim_dealloc_port;
	dev->ctrl.dev          = &pdev->dev;
	dev->ctrl.a_framer     = &dev->framer;

	dev->framer.rootfreq  = SLIM_ROOT_FREQ >> 3;
	dev->framer.superfreq = dev->framer.rootfreq /
				SLIM_CL_PER_SUPERFRAME_DIV8;

	ret = devm_request_irq(&pdev->dev, dev->irq,
			       msm8953_slim_interrupt,
			       IRQF_TRIGGER_HIGH,
			       "msm8953_slim_irq", dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		return ret;
	}

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 1000);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	INIT_DELAYED_WORK(&dev->ngd_up_work, msm8953_slim_ngd_up_worker);
	INIT_WORK(&dev->slave_notify_work, msm8953_slim_slave_notify_worker);

	ret = qmi_handle_init(&dev->qmi, 0, &msm8953_slim_qmi_ops, NULL);
	if (ret) {
		dev_err(&pdev->dev, "qmi_handle_init failed: %d\n", ret);
		goto err_pm;
	}

	ret = qmi_add_lookup(&dev->qmi, SLIMBUS_QMI_SVC_ID,
			     SLIMBUS_QMI_SVC_V1, SLIMBUS_QMI_INS_ID);
	if (ret) {
		dev_err(&pdev->dev, "qmi_add_lookup failed: %d\n", ret);
		goto err_qmi;
	}

	/*
	 * Point of_node at the "slim@N" NGD child while registering the
	 * controller so that of_register_slim_devices() finds the codec
	 * child nodes, then restore it so DMA lookups find the parent's
	 * dmas/dma-names properties.
	 */
	{
		struct device_node *parent_node = pdev->dev.of_node;
		struct device_node *ngd_node;
		u32 id;

		for_each_child_of_node(parent_node, ngd_node) {
			if (of_property_read_u32(ngd_node, "reg", &id))
				continue;
			dev_dbg(&pdev->dev,
				 "Using NGD child %s (reg=%d) for device discovery\n",
				 ngd_node->name, id);
			dev->ngd_node = ngd_node;
			pdev->dev.of_node = ngd_node;
			break;
		}

		ret = slim_register_controller(&dev->ctrl);

		/* Restore parent node for DMA and other platform lookups */
		pdev->dev.of_node = parent_node;
	}
	if (ret) {
		dev_err(&pdev->dev, "slim_register_controller failed: %d\n",
			ret);
		goto err_qmi;
	}

	/*
	 * PDR registration tells the ADSP that a client is interested in
	 * the audio service, which the ADSP needs before it starts the
	 * SLIMbus framer. The PDR callback schedules ngd_up_work when the
	 * audio protection domain comes up.
	 */
	dev->pdr = pdr_handle_alloc(msm8953_slim_pd_status, dev);
	if (IS_ERR(dev->pdr)) {
		dev_warn(&pdev->dev,
			 "PDR handle alloc failed: %ld (continuing)\n",
			 PTR_ERR(dev->pdr));
		dev->pdr = NULL;
	} else {
		struct pdr_service *pds;

		pds = pdr_add_lookup(dev->pdr, "avs/audio",
				     "msm/adsp/audio_pd");
		if (IS_ERR(pds) && PTR_ERR(pds) != -EALREADY)
			dev_warn(&pdev->dev,
				 "PDR add lookup failed: %ld\n",
				 PTR_ERR(pds));
	}

	dev->nb.notifier_call = msm8953_slim_ssr_notifier;
	dev->notifier = qcom_register_ssr_notifier("lpass", &dev->nb);
	if (IS_ERR(dev->notifier)) {
		dev_warn(&pdev->dev,
			 "SSR notifier registration failed: %ld (continuing)\n",
			 PTR_ERR(dev->notifier));
		dev->notifier = NULL;
	}

	/*
	 * Bus bring-up is not started from probe: the SSR/PDR UP
	 * notifications signal that the ADSP is ready, and an ADSP crash
	 * during bring-up cancels the work (DOWN) and reschedules it
	 * after recovery (UP).
	 */

	dev_dbg(&pdev->dev, "SLIMbus NGD controller registered\n");

#ifdef CONFIG_DEBUG_FS
	spin_lock_init(&dev->msg_log_lock);
	dev->debugfs_root = debugfs_create_dir("slimbus-msm8953", NULL);
	debugfs_create_file("status", 0444, dev->debugfs_root, dev,
			    &msm8953_slim_status_fops);
	debugfs_create_file("ngd_regs", 0444, dev->debugfs_root, dev,
			    &msm8953_slim_ngd_regs_fops);
	debugfs_create_file("msg_log", 0444, dev->debugfs_root, dev,
			    &msm8953_slim_msg_log_fops);
#endif

	return 0;

err_qmi:
	qmi_handle_release(&dev->qmi);
err_pm:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void msm8953_slim_remove(struct platform_device *pdev)
{
	struct msm8953_slim_ctrl *dev = platform_get_drvdata(pdev);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(dev->debugfs_root);
#endif

	if (dev->notifier)
		qcom_unregister_ssr_notifier(dev->notifier, &dev->nb);
	if (dev->pdr)
		pdr_handle_release(dev->pdr);

	cancel_delayed_work_sync(&dev->ngd_up_work);
	cancel_work_sync(&dev->slave_notify_work);
	msm8953_slim_qmi_power_request(dev, false);
	msm8953_slim_teardown_dma(dev);
	qmi_handle_release(&dev->qmi);
	slim_unregister_controller(&dev->ctrl);
	pm_runtime_disable(&pdev->dev);
}

/*
 * Disable the NGD on reboot/poweroff so the ADSP does not carry satellite
 * state over into the next boot.
 *
 * Only NGD_CFG is cleared; the ADSP firmware watches it to know whether the
 * AP-side controller is alive. The QMI power request is not sent because
 * qmi_txn_wait() can block for up to 5 s on a dead QMI socket, and the DMA
 * channels are not released because BAM teardown can block on hardware
 * state and hang the reboot.
 */
static void msm8953_slim_shutdown(struct platform_device *pdev)
{
	struct msm8953_slim_ctrl *dev = platform_get_drvdata(pdev);
	void __iomem *ngd;

	if (!dev)
		return;

	ngd = ngd_base(dev);
	writel_relaxed(0, ngd + NGD_CFG);
	/* Quiesce the NGD before the SoC resets */
	mb();
}

/* ---- PM ops -------------------------------------------------------------- */

static const struct dev_pm_ops msm8953_slim_pm_ops = {
	SET_RUNTIME_PM_OPS(msm8953_slim_runtime_suspend,
			   msm8953_slim_runtime_resume,
			   msm8953_slim_runtime_idle)
};

/* ---- Platform driver ----------------------------------------------------- */

static const struct of_device_id msm8953_slim_dt_match[] = {
	{ .compatible = "qcom,slim-ngd-msm8953" },
	{ }
};
MODULE_DEVICE_TABLE(of, msm8953_slim_dt_match);

static struct platform_driver msm8953_slim_driver = {
	.probe    = msm8953_slim_probe,
	.remove   = msm8953_slim_remove,
	.shutdown = msm8953_slim_shutdown,
	.driver = {
		.name           = "qcom-slim-ngd-msm8953",
		.pm             = &msm8953_slim_pm_ops,
		.of_match_table = msm8953_slim_dt_match,
	},
};
module_platform_driver(msm8953_slim_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm MSM8953/SDM632 SLIMbus NGD satellite controller");
MODULE_ALIAS("platform:qcom-slim-ngd-msm8953");
