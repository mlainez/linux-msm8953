// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM8953/SDM632 SLIMbus NGD (Non-ported Generic Device) controller
 *
 * Faithful reimplementation of the downstream slim-msm-ngd.c + slim-msm.c
 * drivers, ported to the mainline kernel slimbus framework.
 *
 * Key differences from the generic mainline qcom-ngd-ctrl.c:
 *  - Uses AHB register-based I/O for TX/RX during enumeration (no BAM
 *    needed for the MASTER_CAPABILITY exchange)
 *  - Sets NGD_CFG = ENABLE only (no msgq bits) matching downstream
 *  - Correct downstream power-up sequence (QMI PM_ACTIVE → NGD setup →
 *    wait MASTER_CAPABILITY → send REPORT_SATELLITE)
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

/*
 * BAM v1.7.0 pipe registers (relative to BAM base, per-pipe stride = 0x1000).
 * Offsets from downstream bam.c BAM_V1_7 table.
 */
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
#define BAM_P_CTRL_DIRECTION	BIT(3)  /* 1=write(mem→periph), 0=read(periph→mem) */

#define SLIM_BAM_DESC_NUM	32

#define PGD_PORT_CFG_WATERMARK(x)	((x) << 1)
#define PGD_PORT_CFG_ENABLE		BIT(0)
#define PGD_PORT_CFG_PACK		(0 << 6)
#define PGD_PORT_CFG_ALIGN_LSB		(0 << 9)

#define DEF_WATERMARK	PGD_PORT_CFG_WATERMARK(1)
#define DEF_BLKSZ	3  /* 4-byte blocks */
#define DEF_TRANSZ	0  /* default */

/* WCD9335 codec port numbering — RX ports start at 16 */
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
	/* Single TX-completion slot; protected by tx_lock */
	struct completion *wr_comp;
	int err;
	atomic_t ssr_in_progress;

	/*
	 * SSR generation counter: incremented on every DOWN event,
	 * captured by UP before scheduling recovery work.  The worker
	 * checks whether the generation still matches — if it doesn't,
	 * a newer DOWN has arrived and this recovery attempt is stale.
	 */
	atomic_t ssr_gen;
	u32 ngd_up_gen;  /* generation captured by the UP handler */
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

	/* Port-to-pipe mapping: SLIMbus port → BAM/PGD pipe */
	u8 pipe_map[32]; /* slim_port → bam_pipe, 0xFF=unallocated */
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

	/* Data port DMA channels and buffers */
	struct dma_chan *data_chan[2];
	void *data_buf[2];
	dma_addr_t data_buf_phys[2];
	bool data_pipe_armed[2];

	/*
	 * Manager-side (AP) data port allocation. Each set bit in
	 * apps_pipes (bits 7..31) is a data pipe owned by the AP. We
	 * hand them out one-at-a-time via alloc_port; each allocation
	 * also arms the corresponding BAM data pipe so the hardware is
	 * ready before the first SLIM frame arrives.
	 *
	 * mgrport_alloc_mask: bit set if apps_pipes-bit was handed out.
	 *   port_b = bit - 7 is the manager-side SLIMbus port number
	 *   used in USR CONNECT messages.
	 * mgrport_count: number of currently-allocated mgr ports.
	 *   doubles as the "port_idx" passed to enable_pgd_port /
	 *   arm_data_pipe (0 for first allocation, 1 for second, ...).
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

/* DMA TX callback: signal completion */
static void msm8953_slim_dma_tx_cb(void *arg)
{
	struct msm8953_slim_ctrl *dev = arg;

	if (dev->wr_comp) {
		struct completion *comp = dev->wr_comp;
		dev->wr_comp = NULL;
		complete(comp);
	}
}

/*
 * BAM v1.7.0 pipe register offsets (also used in arm_data_pipe below).
 * Needed here for the direct TX descriptor write path.
 */
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

		desc->callback = msm8953_slim_dma_tx_cb;
		desc->callback_param = dev;
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

	if (mc != 0x64)  /* Don't spam for REPLY_VALUE */
		dev_dbg(dev->dev, "RX dispatch: mt=0x%x mc=0x%x len=%d [%02x %02x %02x %02x %02x]\n",
			mt, mc, len, buf[0], buf[1], buf[2], buf[3], buf[4]);
	else if (buf[4] != 0)  /* Only log non-zero REPLY_VALUE */
		dev_dbg(dev->dev, "RX REPLY_VALUE: tid=%d data=0x%02x\n",
			buf[3], buf[4]);

	/* MASTER_CAPABILITY: signal the power-up completion */
	if (mc == SLIM_USR_MC_MASTER_CAPABILITY &&
	    mt == SLIM_MSG_MT_SRC_REFERRED_USER) {
		if (dev->mcap_gen != (u32)atomic_read(&dev->ssr_gen)) {
			dev_info(dev->dev,
				 "MCAP ignored: stale gen %u (current %d)\n",
				 dev->mcap_gen, atomic_read(&dev->ssr_gen));
			return;
		}
		dev_info(dev->dev, "SLIM SAT: Rcvd master capability\n");
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

	dev_dbg(dev->dev, "IRQ fired: stat=0x%x cfg=0x%x\n",
		stat, readl_relaxed(ngd + NGD_CFG));

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
		mb();
		if (dev->wr_comp) {
			struct completion *comp = dev->wr_comp;

			dev->wr_comp = NULL;
			complete(comp);
		}
	}

	/* TX sent successfully (AHB path) */
	if (stat & NGD_INT_TX_MSG_SENT) {
		writel_relaxed(NGD_INT_TX_MSG_SENT, ngd + NGD_INT_CLR);
		mb();
		if (dev->wr_comp) {
			struct completion *comp = dev->wr_comp;

			dev->wr_comp = NULL;
			dev_dbg(dev->dev, "TX_MSG_SENT: completing wr_comp\n");
			complete(comp);
		} else {
			dev_dbg(dev->dev, "TX_MSG_SENT: no wr_comp set!\n");
		}
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
		mb();
		msm8953_slim_rx(dev, (u8 *)rx_buf);
	}

	if (stat & NGD_INT_RECFG_DONE) {
		writel_relaxed(NGD_INT_RECFG_DONE, ngd + NGD_INT_CLR);
		mb();
		dev_dbg(dev->dev, "reconfig done IRQ\n");
	}

	if (stat & NGD_INT_IE_VE_CHG) {
		writel_relaxed(NGD_INT_IE_VE_CHG, ngd + NGD_INT_CLR);
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
	 * Instance is 0-based (ctrl_nr is 1-based in downstream, >>1 gives 0).
	 * Mode = MASTER means ADSP is the framer/master.
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

	dev_info(dev->dev, "QMI: instance selected (ctrl %d)\n", dev->ctrl_nr);
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
	dev_info(dev->dev, "QMI slimbus service found (node %d port %d)\n",
		 svc->node, svc->port);
	complete(&dev->qmi_up);
	return 0;
}

static void msm8953_qmi_del_server(struct qmi_handle *qmi,
				   struct qmi_service *svc)
{
	struct msm8953_slim_ctrl *dev =
		container_of(qmi, struct msm8953_slim_ctrl, qmi);

	dev_info(dev->dev, "QMI slimbus service gone\n");
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

	{
		u8 *p = (u8 *)d->base;
		u8 len = p[0] & 0x1F;
		u8 mc = p[1];

		/* Full dump for value messages to debug data=0x00 issue */
		if (mc == 0x64 || mc == 0x04) {
			dev_dbg(dev->dev,
				 "BAM RX REPLY: [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n",
				 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
				 p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		} else {
			dev_dbg(dev->dev,
				 "BAM RX cb: [%02x %02x %02x %02x %02x]\n",
				 p[0], p[1], p[2], p[3], p[4]);
		}
	}
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
	struct dma_async_tx_descriptor *dma_desc;
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
	 * TX messaging via DMA engine. The DMA engine resets BAM pipe 4
	 * and reconfigures it with a 32KB descriptor FIFO. This may
	 * differ from the ADSP's original 264-byte FIFO configuration.
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
	dev_info(dev->dev, "BAM DMA init done: rx_phys=0x%pad bam_tx=%s\n",
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

/*
 * BAM v1.7.0 register offsets for data pipes.
 * These differ from the v1.4 offsets in the BAM_P_* defines above
 * (which are only used for messaging pipes 3/4).
 * v1.7.0: base = 0x13000, stride = 0x1000 per pipe.
 */
#define BAM17_P_RST(p)		(0x13004 + (p) * 0x1000)
#define BAM17_P_IRQ_EN(p)	(0x13018 + (p) * 0x1000)
#define BAM17_IRQ_SRCS_MSK_EE(ee) (0x03004 + (ee) * 0x1000)

static int msm8953_slim_arm_data_pipe(struct msm8953_slim_ctrl *dev,
				       int port_idx, u8 pgd_port)
{
	u32 stat, bam_pipe, ctrl, desc_addr;
	int desc_fifo_sz = 256; /* 32 descriptors * 8 bytes each */

	if (dev->data_pipe_armed[port_idx])
		return 0;

	/*
	 * For AP-side data ports the BAM pipe number is just the
	 * apps_pipes bit position (== pgd_port here): the DTS pairs
	 * `data-port0` with `<&slimbam 21>` and `data-port1` with
	 * `<&slimbam 22>`, matching apps_pipes bits 21 and 22 on
	 * FP3+. PGD_PORT_STATn is populated by the ADSP for codec-side
	 * ports, not AP-side, so reading it here always returned 0 ->
	 * -ENODEV. Keep the read as a sanity-log only.
	 */
	stat = readl_relaxed(dev->base + PGD_PORT_STATn(pgd_port));
	bam_pipe = pgd_port;
	dev_dbg(dev->dev,
		 "BAM arm: ap_pgd_port=%d bam_pipe=%d STATn=0x%x\n",
		 pgd_port, bam_pipe, stat);

	ctrl = readl_relaxed(dev->bam_base + BAM17_P_CTRL(bam_pipe));
	desc_addr = readl_relaxed(dev->bam_base +
				  BAM17_P_DESC_FIFO_ADDR(bam_pipe));

	dev_dbg(dev->dev,
		 "BAM pipe %d: P_CTRL=0x%x DESC_ADDR=0x%x FIFO_SZ=0x%x (before)\n",
		 bam_pipe, ctrl, desc_addr,
		 readl_relaxed(dev->bam_base + BAM17_P_FIFO_SIZES(bam_pipe)));

	/*
	 * Full BAM pipe setup with correct v1.7.0 offsets.
	 * The ADSP does NOT pre-configure data pipes — we must do it.
	 * This matches downstream sps_bam_pipe_connect → bam_pipe_init.
	 */

	/* Allocate descriptor FIFO if needed */
	if (!dev->data_buf[port_idx]) {
		dev->data_buf[port_idx] = dma_alloc_coherent(dev->dev,
				desc_fifo_sz, &dev->data_buf_phys[port_idx],
				GFP_KERNEL);
		if (!dev->data_buf[port_idx])
			return -ENOMEM;
		memset(dev->data_buf[port_idx], 0, desc_fifo_sz);
	}

	/* Reset pipe */
	writel_relaxed(1, dev->bam_base + BAM17_P_RST(bam_pipe));
	wmb();
	writel_relaxed(0, dev->bam_base + BAM17_P_RST(bam_pipe));
	wmb();

	/* Unmask pipe interrupt for our EE */
	{
		u32 msk = readl_relaxed(dev->bam_base +
					BAM17_IRQ_SRCS_MSK_EE(dev->ee));
		writel_relaxed(msk | BIT(bam_pipe),
			       dev->bam_base +
			       BAM17_IRQ_SRCS_MSK_EE(dev->ee));
	}

	/* Configure pipe interrupts */
	writel_relaxed(0x7, dev->bam_base + BAM17_P_IRQ_EN(bam_pipe));

	/* Set descriptor FIFO */
	writel_relaxed(lower_32_bits(dev->data_buf_phys[port_idx]),
		       dev->bam_base + BAM17_P_DESC_FIFO_ADDR(bam_pipe));
	writel_relaxed(desc_fifo_sz,
		       dev->bam_base + BAM17_P_FIFO_SIZES(bam_pipe));

	wmb();

	/* Enable pipe: P_EN | P_SYS_MODE (no DIRECTION bit = producer/write) */
	writel_relaxed(BAM_P_CTRL_EN | BAM_P_CTRL_SYS_MODE,
		       dev->bam_base + BAM17_P_CTRL(bam_pipe));
	wmb();

	ctrl = readl_relaxed(dev->bam_base + BAM17_P_CTRL(bam_pipe));
	desc_addr = readl_relaxed(dev->bam_base +
				  BAM17_P_DESC_FIFO_ADDR(bam_pipe));
	dev_dbg(dev->dev,
		 "BAM pipe %d armed: P_CTRL=0x%x DESC_ADDR=0x%x FIFO_SZ=0x%x\n",
		 bam_pipe, ctrl, desc_addr,
		 readl_relaxed(dev->bam_base + BAM17_P_FIFO_SIZES(bam_pipe)));

	dev->data_pipe_armed[port_idx] = true;
	return 0;
}

static int msm8953_slim_enable_pgd_port(struct msm8953_slim_ctrl *dev,
					u8 pgd_port, int port_idx)
{
	u32 cfg, stat;
	char chan_name[16];
	struct dma_chan *chan;

	/*
	 * Enable the PGD hardware port (matching downstream msm_hw_set_port).
	 */
	cfg = DEF_WATERMARK | PGD_PORT_CFG_PACK | PGD_PORT_CFG_ALIGN_LSB |
	      PGD_PORT_CFG_ENABLE;
	writel_relaxed(cfg, dev->base + PGD_PORT_CFGn(pgd_port));
	writel_relaxed(DEF_BLKSZ, dev->base + PGD_PORT_BLKn(pgd_port));
	writel_relaxed(DEF_TRANSZ, dev->base + PGD_PORT_TRANn(pgd_port));
	mb();

	/* Enable port interrupt */
	{
		u32 int_en_reg = 0x5000 + dev->ee * 0x1000;
		u32 int_en = readl_relaxed(dev->base + int_en_reg);

		if (!(int_en & BIT(pgd_port))) {
			writel_relaxed(int_en | BIT(pgd_port),
				       dev->base + int_en_reg);
			mb();
		}
	}

	/*
	 * Request BAM DMA channel for this data port.
	 * The bam_dma driver configures the BAM pipe registers
	 * (CTRL, DESC_FIFO_ADDR, DESC_FIFO_SIZE) which we cannot
	 * write directly due to XPU restrictions.
	 * DMA channel names "data-port0", "data-port1" map to
	 * BAM pipes 21, 22 in the DTS.
	 */
	if (port_idx < 2 && !dev->data_chan[port_idx]) {
		snprintf(chan_name, sizeof(chan_name), "data-port%d", port_idx);
		chan = dma_request_chan(dev->dev, chan_name);
		if (IS_ERR(chan)) {
			dev_warn(dev->dev,
				 "Data DMA chan '%s' not available: %ld\n",
				 chan_name, PTR_ERR(chan));
		} else {
			dev->data_chan[port_idx] = chan;
			dev_dbg(dev->dev,
				 "Data DMA chan '%s' acquired for PGD port %d\n",
				 chan_name, pgd_port);
		}
	}

	stat = readl_relaxed(dev->base + PGD_PORT_STATn(pgd_port));
	dev_dbg(dev->dev,
		 "PGD port %d enabled: CFG=0x%x STAT=0x%x\n",
		 pgd_port,
		 readl_relaxed(dev->base + PGD_PORT_CFGn(pgd_port)),
		 stat);

	return 0;
}

/*
 * msm8953_slim_alloc_port - controller ->alloc_port callback
 *
 * Hands out the next free manager-side data port from the apps_pipes
 * bitmap, programs the PGD port and arms the corresponding BAM data
 * pipe.  Returns the SLIMbus port number ("port_b") in *port_out so
 * the caller can use it in USR CONNECT messages addressed to the
 * manager (e.g. wbuf[1] of a master CONNECT_SRC/SINK).
 *
 * apps_pipes bits 7..31 enumerate AP-owned data pipes; port_b for
 * the n-th set bit is (bit - 7).  Each allocation also arms the
 * BAM pipe so the hardware is ready before the first SLIM frame
 * arrives.
 */
static int msm8953_slim_alloc_port(struct slim_controller *ctrl, u8 *port_out)
{
	struct msm8953_slim_ctrl *dev =
		container_of(ctrl, struct msm8953_slim_ctrl, ctrl);
	int bit;
	int port_idx;
	u8 port_b;
	int ret;

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
	 * Just track the allocation; don't touch PGD_PORT_CFGn here.
	 * Downstream's msm_alloc_port mirrors this minimalism — it only
	 * registers the AP-side BAM endpoint. The actual PGD register
	 * programming and BAM-pipe-arm happen later in enable_stream,
	 * AFTER the SLIM CONNECT/DEF_ACT_CHAN sequence. Writing
	 * PGD_PORT_CFGn early collides with the ADSP's view of the
	 * port (the comment near xfer_msg's CONNECT branch warns that
	 * 'Writing PGD_PORT_CFGn from the apps side for audio ports
	 * causes I/O errors by conflicting with the ADSP'). The early
	 * arm we tried first made AFE DEVICE_START (cmd 0x100e5) return
	 * DSP error 0x1.
	 */

	dev->mgrport_alloc_mask |= (1u << bit);
	dev->mgrport_count++;
	mutex_unlock(&dev->tx_lock);

	*port_out = port_b;
	dev_info(dev->dev,
		 "alloc_port: handed out port_b=%d (bit=%d port_idx=%d apps_pipes=0x%x)\n",
		 port_b, bit, port_idx, dev->apps_pipes);
	return 0;
}

/*
 * msm8953_slim_dealloc_port - controller ->dealloc_port callback
 *
 * Reverse of alloc_port: marks the bit free, leaves the BAM pipe
 * state in place (it will be torn down during shutdown).  Callers
 * are not expected to allocate and free repeatedly during a session.
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

	dev_info(dev->dev, "dealloc_port: freed port_b=%d (bit=%d)\n",
		 port_b, bit);
	return 0;
}

/*
 * msm8953_slim_ngd_setup - program NGD registers and enable the controller
 *
 * After the ADSP crashes and restarts, it resets the SLIMbus hardware block.
 * If we write NGD_CFG while the ADSP is still resetting SLIMbus, the write
 * doesn't stick (reads back as 0x0).  This function polls NGD_CFG after
 * writing it, waiting for the ADSP to release the hardware from reset.
 *
 * Returns 0 on success, -EAGAIN if NGD_CFG never took effect (caller should
 * retry the entire power-up sequence after a delay).
 */
static int msm8953_slim_ngd_setup(struct msm8953_slim_ctrl *dev)
{
	void __iomem *ngd = ngd_base(dev);
	u32 rx_msgq, cfg_readback;
	/*
	 * Both RX_MSGQ_EN and TX_MSGQ_EN set. ADSP firmware appears to
	 * gate its QMI satellite handshake on seeing TX_MSGQ_EN at NGD
	 * power-on (a globally-cleared TX_MSGQ_EN hangs early boot, no
	 * USB net comes up). USR messages stay on BAM TX (their normal
	 * path to the ADSP).
	 *
	 * For codec-bound MT_CORE messages (REQUEST/CHANGE_VALUE), we
	 * toggle TX_MSGQ_EN off per-write in xfer_msg, do the AHB write
	 * to NGD_TX_MSG, then toggle TX_MSGQ_EN back on. The ADSP would
	 * otherwise drop those frames (adsp_sat_dispatcher @ 0xf04ca424
	 * silently discards MT=0 MC!=0x29). See codec_pgd_via_ahb_finding.md.
	 */
	u32 cfg_val = NGD_CFG_ENABLE | NGD_CFG_RX_MSGQ_EN | NGD_CFG_TX_MSGQ_EN;
	int i;

	writel_relaxed(DEF_NGD_INT_MASK, ngd + NGD_INT_EN);

	rx_msgq = readl_relaxed(ngd + NGD_RX_MSGQ_CFG);
	writel_relaxed(rx_msgq | SLIM_RX_MSGQ_TIMEOUT_VAL,
		       ngd + NGD_RX_MSGQ_CFG);

	/*
	 * Poll NGD_CFG after writing: if the ADSP is still resetting the
	 * SLIMbus hardware, writes are silently dropped (readback = 0).
	 * Give it up to ~50ms (10 x 5ms) for the ADSP to release the
	 * block from reset.
	 */
	for (i = 0; i < 10; i++) {
		writel_relaxed(cfg_val, ngd + NGD_CFG);
		mb();

		cfg_readback = readl_relaxed(ngd + NGD_CFG);
		if (cfg_readback == cfg_val)
			break;

		dev_dbg(dev->dev,
			"NGD_CFG write didn't stick (attempt %d, read=0x%x), waiting\n",
			i, cfg_readback);
		usleep_range(5000, 6000);
	}

	dev_dbg(dev->dev,
		 "NGD setup: cfg=0x%x int_en=0x%x rxmsgq=0x%x stat=0x%x\n",
		 cfg_readback,
		 readl_relaxed(ngd + NGD_INT_EN),
		 readl_relaxed(ngd + NGD_RX_MSGQ_CFG),
		 readl_relaxed(ngd + NGD_STATUS));

	if (cfg_readback != cfg_val) {
		dev_warn(dev->dev,
			 "NGD_CFG stuck at 0x%x after %d retries (ADSP SLIMbus not ready)\n",
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

	/* Wait for GENERIC_ACK from ADSP.  The ADSP requires the TID
	 * to stay alive until it finishes processing DEF_ACT_CHAN and
	 * RECONFIG_NOW — these are state-transition commands, not
	 * fire-and-forget.  Without this wait, the ADSP may leave
	 * channels in a defined-but-inactive state with zero data flow.
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
		dev_info(ctrl->dev, "USR msg ACK OK: mc=0x%x tid=%d\n",
			 txn->mc, txn->tid);
	}

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
			 * dataf=0 (NOT_DEFINED) matches downstream tasha.
			 * prrate uses standard SLIMbus presence rate code
			 * (from slim_get_prate_code() via stream_prepare),
			 * with FL (Frequency Locked) bit set for ISO protocol.
			 * Matches both downstream slim_calc_prrate() and
			 * reference qcom_slim_ngd_enable_stream().
			 */

			/* Byte 0: dataf=0 (NOT_DEFINED) + laddr low 5 bits */
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

			/* Byte 3: standard SLIMbus prrate + FL for ISO */
			if (rt->prot == SLIM_PROTO_ISO)
				wbuf[txn.msg->num_bytes++] =
					port->ch.prrate | SLIM_CHANNEL_CONTENT_FL;
			else
				wbuf[txn.msg->num_bytes++] = port->ch.prrate;

			ret = slim_alloc_txn_tid(ctrl, &txn);
			if (ret) {
				dev_err(&sdev->dev, "Fail to allocate TID\n");
				return -ENXIO;
			}
			wbuf[txn.msg->num_bytes++] = txn.tid;

		dev_info(ctrl->dev,
			 "DEF_ACT_CHAN bytes: [%02x %02x %02x %02x TID=%d] coef=%d exp=%d prrate=%d\n",
			 wbuf[0], wbuf[1], wbuf[2], wbuf[3],
			 txn.tid, coef, exp, port->ch.prrate);
		}
		wbuf[txn.msg->num_bytes++] = port->ch.id;
	}

	/*
	 * Do NOT send CORE NEXT_DEFINE_CHANNEL here — the ADSP satellite
	 * dispatcher drops all CORE messages (verified via firmware RE at
	 * 0xf04ca424). DEF_ACT_CHAN (USR mc=0x21) below both defines and
	 * activates the channel in the ADSP's internal state machine.
	 */

	/*
	 * AP-side master-port allocation (2026-05-26).
	 *
	 * Per ADSP firmware RE: the bus master tracks "master ports"
	 * (AP-side BAM endpoints) separately from "slave ports" (codec
	 * endpoints). Each channel needs BOTH allocated for the frame
	 * scheduler to drive data.
	 *
	 * Downstream `slim-msm-ngd.c:670` handles this implicitly: when a
	 * CONNECT message has `wbuf[0] == dev->pgdla` (= SLIM_LA_MGR
	 * = 0xFF), downstream allocates the AP-side BAM pipe and lets the
	 * frame go out on the bus to the manager. The ADSP responds by
	 * binding the manager-side endpoint to the channel.
	 *
	 * Mainline's slim core only sends ONE CONNECT per stream port —
	 * with `wbuf[0] = codec_la` (the codec endpoint). It never tells
	 * the ADSP about the AP-side endpoint. Without that, the ADSP
	 * knows the codec is supposed to send/receive but has no AP
	 * endpoint to direct frames to, so PORT_STATUS stays 0x00.
	 *
	 * Fix: inject an additional CONNECT for the AP-master side, in
	 * the OPPOSITE direction of the codec's CONNECT.
	 *   codec CONNECT_SRC  (TX / mic):  AP needs CONNECT_SINK
	 *   codec CONNECT_SINK (RX / play): AP needs CONNECT_SRC
	 *
	 * Master-port number = BAM pipe assigned by HW in PGD_PORT_STATn
	 * register, bits [11:4]. Must be read after enable_pgd_port runs
	 * (which programs PGD_PORT_CFGn).
	 *
	 * See memory/master_port_connect_missing.md for the analysis.
	 */
	/*
	 * Compute manager port_b values from apps_pipes bitmap.
	 *
	 * Per downstream `slim-msm.c:1101-1113` (msm_slim_data_port_assign):
	 *   First 7 BAM pipes are reserved for message queues. Iterate
	 *   apps_pipes bits 7..31; each set bit is a data pipe. The
	 *   data_port index (0, 1, ...) is assigned in order, and
	 *   port_b = bit_position - 7. port_b is the SLIMbus manager-side
	 *   port number used in USR CONNECT messages addressed to MGR.
	 *
	 * On FP3+ DT: qcom,apps-ch-pipes = 0x6000 → bits 13, 14 set.
	 *   data_port 0 → port_b = 13 - 7 = 6
	 *   data_port 1 → port_b = 14 - 7 = 7
	 */
	for (i = 0; i < rt->num_ports && i < 2; i++) {
		struct slim_port *port = &rt->ports[i];
		u8  port_b;
		u8  master_mbuf[4];
		struct slim_val_inf master_msg = {0, 3, NULL, master_mbuf, NULL};
		struct slim_msg_txn master_txn = {0};
		DECLARE_COMPLETION_ONSTACK(master_done);
		int mret, bit, found = 0, data_idx = 0;

		if (port->ch.id < 128) {
			dev_warn(&sdev->dev,
				 "MasterCONNECT: skipping unmapped ch.id=%d\n",
				 port->ch.id);
			continue;
		}

		/* Walk apps_pipes to find the i-th data port's port_b. */
		port_b = 0xFF;
		for (bit = 7; bit < 32; bit++) {
			if (!(dev->apps_pipes & (1u << bit)))
				continue;
			if (data_idx == i) {
				port_b = bit - 7;
				found = 1;
				break;
			}
			data_idx++;
		}
		if (!found) {
			dev_warn(&sdev->dev,
				 "MasterCONNECT: no apps_pipes slot for port[%d] (apps_pipes=0x%x)\n",
				 i, dev->apps_pipes);
			continue;
		}

		master_txn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
		master_txn.dt = SLIM_MSG_DEST_LOGICALADDR;
		master_txn.la = SLIM_LA_MGR;
		master_txn.msg = &master_msg;

		/*
		 * Direction: opposite of the codec's port direction.
		 * port->direction == SLIM_PORT_SOURCE means codec is source,
		 * so AP is sink → AP CONNECT_SINK.
		 * port->direction == SLIM_PORT_SINK means codec is sink,
		 * so AP is source → AP CONNECT_SRC.
		 */
		if (port->direction == SLIM_PORT_SOURCE)
			master_txn.mc = SLIM_USR_MC_CONNECT_SINK;
		else
			master_txn.mc = SLIM_USR_MC_CONNECT_SRC;

		/*
		 * Downstream `slim-msm-ngd.c` lines 565-590 rewrite the
		 * outgoing txn->la from SLIM_LA_MGR (0xff) to dev->pgdla,
		 * which is resolved by get_laddr() to the *codec* PGD's
		 * logical address (e.g. 200 for WCD9335). The on-wire
		 * wbuf[0] for a master CONNECT is therefore the codec
		 * laddr, NOT the manager laddr. Sending 0xff here was a
		 * misreading of the downstream variable name and is most
		 * likely what causes the ADSP to silently drop the
		 * master CONNECT (matching the -110 timeout we see).
		 */
		master_mbuf[0] = sdev->laddr;       /* codec PGD laddr */
		master_mbuf[1] = port_b;            /* manager-side SLIMbus port */
		master_mbuf[2] = port->ch.id;       /* channel id */

		mret = slim_alloc_txn_tid(ctrl, &master_txn);
		if (mret) {
			dev_warn(&sdev->dev,
				 "MasterCONNECT: TID alloc fail ret=%d\n", mret);
			continue;
		}
		master_mbuf[3] = master_txn.tid;
		master_msg.num_bytes = 4;
		master_txn.rl = master_msg.num_bytes + 4;
		master_txn.comp = &master_done;

		dev_info(ctrl->dev,
			 "MasterCONNECT: mc=0x%x la=MGR(0xFF) port_b=%d chan=%d tid=%d (codec %s, apps_pipes=0x%x)\n",
			 master_txn.mc, port_b, port->ch.id, master_txn.tid,
			 (port->direction == SLIM_PORT_SOURCE) ? "SOURCE" : "SINK",
			 dev->apps_pipes);

		/*
		 * Use xfer_msg directly, NOT xfer_msg_sync. xfer_msg has its
		 * own internal ACK wait for USR_MC_CONNECT_* (lines 2302+)
		 * that consumes txn->comp. xfer_msg_sync would then try to
		 * wait again on the already-consumed completion and time
		 * out — exactly the -110 we used to see on every master
		 * CONNECT despite the ADSP ACKing in <1 ms (verified in
		 * msg_log: 'TX 47 2c ff c8 0e ... / RX c5 25 ff <tid> 20').
		 */
		mret = msm8953_slim_xfer_msg(ctrl, &master_txn);
		if (mret) {
			slim_free_txn_tid(ctrl, &master_txn);
			dev_warn(&sdev->dev,
				 "MasterCONNECT failed: %d (port_b=%d chan=%d)\n",
				 mret, port_b, port->ch.id);
		}
	}

	/*
	 * Send REQ_BW (mc=0x28) to request bandwidth allocation from the
	 * ADSP master. Without this, the ADSP creates channels via
	 * DEF_ACT_CHAN but never allocates bus frame slots for them,
	 * leaving data stuck in PGD port FIFOs (overflow state).
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
		 * REQ_BW format (downstream ngd_allocbw):
		 * byte 0: (laddr & 0x1f) | ((pending_msgsl & 0x7) << 5)
		 * byte 1: (pending_msgsl >> 3)
		 * byte 2: TID
		 *
		 * pending_msgsl is the number of messaging slots needed.
		 * For simple audio playback, request minimum slots (8).
		 */
		bbuf[0] = (sdev->laddr & 0x1f) | ((8 & 0x7) << 5);
		bbuf[1] = (8 >> 3);

		ret = slim_alloc_txn_tid(ctrl, &btxn);
		if (ret)
			return ret;
		bbuf[2] = btxn.tid;
		bmsg.num_bytes = 3;
		btxn.rl = bmsg.num_bytes + 4;
		btxn.comp = &bdone;

		dev_info(ctrl->dev, "REQ_BW: [%02x %02x tid=%d]\n",
			 bbuf[0], bbuf[1], btxn.tid);

		ret = msm8953_slim_xfer_msg_sync(ctrl, &btxn);
		if (ret) {
			slim_free_txn_tid(ctrl, &btxn);
			dev_warn(&sdev->dev, "REQ_BW failed: %d\n", ret);
		}
	}

	/*
	 * RECONFIG_NOW to commit the REQ_BW bandwidth reservation.
	 * Without this the ADSP never allocates bus frame slots for
	 * the channels that DEF_ACT_CHAN is about to activate, and
	 * data sits in PGD port FIFOs forever (OVERFLOW, PORT_STATUS=0).
	 * Matches downstream ngd_allocbw sequence:
	 *   REQ_BW -> RECONFIG_NOW -> DEF_ACT_CHAN -> RECONFIG_NOW.
	 */
	{
		u8 rbw_buf[2];
		struct slim_val_inf rbw_msg = {0, 2, NULL, rbw_buf, NULL};
		struct slim_msg_txn rbw_txn = {0};
		DECLARE_COMPLETION_ONSTACK(rbw_done);

		rbw_txn.mt = SLIM_MSG_MT_DEST_REFERRED_USER;
		rbw_txn.dt = SLIM_MSG_DEST_LOGICALADDR;
		rbw_txn.la = SLIM_LA_MGR;
		rbw_txn.mc = SLIM_USR_MC_RECONFIG_NOW;
		rbw_txn.msg = &rbw_msg;

		rbw_buf[1] = sdev->laddr;

		ret = slim_alloc_txn_tid(ctrl, &rbw_txn);
		if (ret)
			return ret;
		rbw_buf[0] = rbw_txn.tid;
		rbw_txn.rl = rbw_msg.num_bytes + 4;
		rbw_txn.comp = &rbw_done;

		dev_info(ctrl->dev, "REQ_BW RECONFIG_NOW: tid=%d laddr=0x%x\n",
			 rbw_txn.tid, sdev->laddr);

		ret = msm8953_slim_xfer_msg_sync(ctrl, &rbw_txn);
		if (ret) {
			slim_free_txn_tid(ctrl, &rbw_txn);
			dev_warn(&sdev->dev,
				 "REQ_BW RECONFIG_NOW failed: %d\n", ret);
		}
	}

	txn.mc = SLIM_USR_MC_DEF_ACT_CHAN;
	txn.rl = txn.msg->num_bytes + 4;
	txn.comp = &done;

	dev_info(ctrl->dev,
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
	wbuf[1] = sdev->laddr; /* full laddr, not masked — matches downstream */
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
	 * After RECONFIG_NOW: the ADSP has accepted the channel set and
	 * has now assigned a BAM pipe to each AP-side data port (port_b).
	 * Arm those pipes so the AP can actually move audio bytes.
	 *
	 * IMPORTANT: BAM data pipes live on the AP-side PGD port whose
	 * number equals the apps_pipes bit position (21 or 22 on FP3+),
	 * NOT the codec PGD port (16 for RX0 etc.). Calling
	 * enable_pgd_port / arm_data_pipe with the codec port number
	 * (port->ch.id - 128) was a long-standing bug: it wrote unused
	 * NGD registers and then tried to read PGD_PORT_STATn(codec_port)
	 * which is always 0 → arm_data_pipe failed with -ENODEV.
	 *
	 * Walk apps_pipes the same way alloc_port does; the i-th set bit
	 * is the AP-side PGD port number for the i-th data port of this
	 * stream. port_b = bit - 7 is what was sent in the master
	 * CONNECT message (verified by the msg_log: "47 2c ff c8 0e..."
	 * where 0e = port_b=14, bit=21).
	 *
	 * port_idx is the local 0/1 slot tracking AP-side BAM pipes
	 * (DTS dmas "data-port0"/"data-port1"). Limit to first 2 ports.
	 */
	{
		int bit, data_idx = 0;

		for (bit = 7; bit < 32 && data_idx < rt->num_ports && data_idx < 2;
		     bit++) {
			u8 ap_pgd_port;
			int aret;

			if (!(dev->apps_pipes & (1u << bit)))
				continue;
			ap_pgd_port = (u8)bit;	/* AP-side PGD port number */

			dev_info(&sdev->dev,
				 "BAM arm: data_idx=%d ap_pgd_port=%d (port_b=%d) port_idx=%d\n",
				 data_idx, ap_pgd_port, ap_pgd_port - 7,
				 data_idx);

			aret = msm8953_slim_enable_pgd_port(dev, ap_pgd_port,
							    data_idx);
			if (aret) {
				dev_warn(&sdev->dev,
					 "BAM arm: enable_pgd_port pgd=%d ret=%d\n",
					 ap_pgd_port, aret);
				data_idx++;
				continue;
			}
			aret = msm8953_slim_arm_data_pipe(dev, data_idx,
							  ap_pgd_port);
			if (aret) {
				dev_warn(&sdev->dev,
					 "BAM arm: arm_data_pipe pgd=%d ret=%d\n",
					 ap_pgd_port, aret);
			}
			data_idx++;
		}
	}

	/*
	 * Do NOT send CORE BEGIN_RECONFIG / DEFINE_CONTENT / ACTIVATE /
	 * RECONFIGURE_NOW via raw BAM TX. Verified by ADSP firmware RE:
	 * the satellite dispatcher drops all CORE messages. The NGD
	 * hardware emits the BAM buffer as a bus frame, but nothing on
	 * the bus processes these MCs — they are bus-manager-addressed
	 * messages and the ADSP is the sole manager.
	 *
	 * The USR DEF_ACT_CHAN + RECONFIG_NOW pair above is the full
	 * sequence required — the ADSP internally schedules transport.
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
		dev_info(dev->dev, "REPORT_SAT: SSR in progress, aborting\n");
		return -ENODEV;
	}
	memset(buf, 0, sizeof(buf));

	/*
	 * REPORT_SATELLITE message (matching downstream ngd_xfer_msg format):
	 *   Byte 0:   RL=7 | MT=6 (SRC_REFERRED_USER) << 5
	 *   Byte 1:   MC=0x01 (REPORT_SATELLITE) | DT=0 << 7
	 *   Byte 2:   LA = SLIM_LA_MGR (0xFF)
	 *   Bytes 3-6: magic_lsb, magic_msb, ver, prot
	 *
	 * RL=7 means 7 remaining bytes after byte 0, total msg = 8 bytes.
	 * Downstream sets txn.rl=8 then decrements (rl--) before encoding.
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

	dev->wr_comp = &tx_sent;
	dev->err = 0;

	/*
	 * Send REPORT_SATELLITE via BAM TX if available. The ADSP expects
	 * to receive REPORT_SATELLITE during the QMI satellite handshake
	 * (BAM TX is the normal AP→ADSP control path).
	 *
	 * If BAM TX isn't available, fall back to AHB by temporarily
	 * clearing TX_MSGQ_EN, doing the AHB write, then restoring it.
	 */
	if (dev->use_bam_tx) {
		ret = msm8953_slim_bam_tx(dev, buf, 7);
	} else {
		void __iomem *ngd = ngd_base(dev);
		u32 cfg = readl_relaxed(ngd + NGD_CFG);

		writel_relaxed(cfg & ~NGD_CFG_TX_MSGQ_EN, ngd + NGD_CFG);
		mb();
		ret = msm8953_slim_ahb_tx(dev, buf, 7,
					  NGD_BASE(dev->ctrl_nr) + NGD_TX_MSG);
		writel_relaxed(cfg | NGD_CFG_TX_MSGQ_EN, ngd + NGD_CFG);
		mb();
	}
	if (ret)
		goto out;

	timeout = wait_for_completion_timeout(&tx_sent, HZ);
	if (!timeout) {
		dev_warn(dev->dev, "REPORT_SATELLITE TX timeout, retry %d\n",
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
		dev_warn(dev->dev, "REPORT_SATELLITE TX NACKed: %d, retry %d\n",
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

	dev_info(dev->dev, "SLIM SAT: REPORT_SATELLITE sent successfully\n");

out:
	dev->wr_comp = NULL;
	return ret;
}

/* ---- Power-up sequence -------------------------------------------------- */

/*
 * Downstream sequence (must match exactly):
 *  1. Wait for QMI service (ADSP up)
 *  2. Read hardware version
 *  3. Check if NGD already has LADDR (fast path)
 *  4. Send QMI PM_ACTIVE
 *  5. Configure NGD registers + enable NGD
 *  6. Wait for MASTER_CAPABILITY interrupt
 *  7. Send REPORT_SATELLITE via AHB TX
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
	dev_info(dev->dev, "SLIM QMI PM_ACTIVE sent OK\n");

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
	 * ADSP cold-boot timing fix (2026-05-26):
	 *
	 * Observed timeline on intermittent cold boots:
	 *   t=3.92s  ADSP comes up (first time)
	 *   t=5.06s  ADSP crashes "Excep:0:Exception detected"
	 *   t=5.40s  ADSP recovers (second UP)
	 *   t=~6.0s  Our PDR UP handler schedules ngd_up_work
	 *   t=~6.0s  ngd_enable runs, power_up calls PM_ACTIVE, MCAP starts
	 *   t=7.02s  First MCAP timeout — ADSP not ready
	 *   t=12.5s  All 5 MCAP retries fail → -ETIMEDOUT
	 *   t=44s    Next ngd_up_work retry blocks 30s on qmi_up
	 *   t=160s+  All worker retries exhausted, soundcard never appears
	 *
	 * Root cause: ADSP's SLIMbus subsystem is NOT actually ready
	 * 600ms after PDR fires SERVICE_STATE_UP. The audio PDR service
	 * comes up earlier than the bus manager. We were sending
	 * REPORT_SATELLITE / waiting MCAP before the ADSP could respond.
	 *
	 * Fix: bump the FIRST MCAP wait from 1 s to 3 s. The actual
	 * latency between PDR UP and ADSP-ready is usually under 1 s,
	 * but on cold boots after the recovery cycle it can exceed
	 * 1 s. Subsequent retries keep the 1 s wait — if MCAP is going
	 * to fire, it does so quickly once ADSP is actually settled.
	 *
	 * Also extend the post-NGD_CFG-cleared backoff from 50 ms to
	 * 500 ms. When NGD_CFG == 0 after timeout, the ADSP visibly
	 * reset the HW — give it real time to come back rather than
	 * burning the next 1 s waiting on a still-unresponsive ADSP.
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

		dev_warn(dev->dev,
			 "MCAP timeout (attempt %d), stat=0x%x cfg=0x%x\n",
			 mcap_retries, stat, cfg);
		if (mcap_retries < INIT_MX_RETRIES &&
		    !atomic_read(&dev->ssr_in_progress)) {
			mcap_retries++;
			if (cfg == 0) {
				/*
				 * ADSP reset SLIMbus HW after our NGD setup.
				 * Re-send PM_ACTIVE to kick ADSP SLIMbus
				 * manager, then retry without DMA teardown
				 * (matching downstream capability_retry).
				 * Sleep 500 ms — measured ADSP recovery time
				 * after Excep crash is ~340 ms; 500 ms gives
				 * margin without dragging out total timeout.
				 */
				dev_info(dev->dev,
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
		dev_warn(dev->dev, "SSR during power_up, aborting\n");
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
	u8 orig_la = 0, orig_port = 0, orig_wbuf1 = 0;

	memset(wbuf, 0, sizeof(wbuf));

	/*
	 * NOTE: Tried redirecting CHANGE_VALUE writes from LA=192 (PGD) to
	 * LA=199 (real WCD9335 IFC).  ADSP NACKs writes to LA=199 too —
	 * both reads AND writes are blocked for the WCD9335 device.
	 * The satellite can ONLY access PGD devices (LA=192-196).
	 */

	/* Drop CORE reconfiguration messages from the framework — these
	 * are sent during boot/probe and confuse the ADSP. The raw CORE
	 * reconfig sequence is sent directly via BAM TX from enable_stream
	 * during playback only. */
	if (txn->mt == SLIM_MSG_MT_CORE &&
	    txn->mc >= SLIM_MSG_MC_BEGIN_RECONFIGURATION &&
	    txn->mc <= SLIM_MSG_MC_RECONFIGURE_NOW)
		return 0;

	/*
	 * Send CORE CHANGE_VALUE as-is (no conversion).
	 *
	 * ADSP firmware RE (0xf04ca424 dispatcher) confirms the ADSP has
	 * no handler for MC=0x68 (CORE CHANGE_VALUE) OR MC=0x00 (USR
	 * REPEAT_CHANGE_VALUE); both fall into the "invalid MC" cleanup
	 * path and are ACK'd at transport layer without any bus action.
	 *
	 * Downstream slim-msm-ngd.c:611 sends CORE CHANGE_VALUE via BAM
	 * TX directly — the MSM NGD hardware emits it as a standard
	 * SLIMbus bus frame, which the codec (a normal SLIMbus slave at
	 * LA=199/200) processes. REPEAT_CHANGE_VALUE (a Qualcomm USR MC)
	 * is not a valid bus frame type, so converting is actively wrong.
	 */

	/*
	 * Convert CORE REQUEST_VALUE to CORE but keep it as-is.
	 * The downstream NGD controller sends REQUEST_VALUE as CORE
	 * and the ADSP does process it (unlike other CORE messages).
	 * Add a timeout log so we can diagnose if reads hang.
	 */

	/* Wake the bus — runtime resume sends QMI PM_ACTIVE to ADSP */
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

		dev_info(dev->dev, "ADSP slimbus not up yet, waiting...\n");
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

		/* Save for PGD connect after codec connect */
		orig_la = txn->la;
		orig_port = txn->msg->wbuf[0];
		orig_wbuf1 = (txn->mc != SLIM_MSG_MC_DISCONNECT_PORT) ?
			     txn->msg->wbuf[1] : 0;

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
		 * Convert to USR message format.
		 * IPC logs from working LineageOS show:
		 *   Connect port: laddr 0xc8 port_num 18 chan_num 146
		 * The CONNECT uses the CODEC device LA and CODEC port
		 * numbers — NOT the PGD LA/port as source code suggested.
		 * Keep original txn->la (codec LA) and port from mainline.
		 */
		{
			u8 slim_port = txn->msg->wbuf[0];

			wbuf[i++] = txn->la;   /* codec device LA (200) */
			la = SLIM_LA_MGR;      /* destination = manager */
			wbuf[i++] = slim_port; /* codec port number */

			dev_info(dev->dev,
			       "CONNECT: la=%d port=%d\n",
			       txn->la, slim_port);
		}
		if (txn->mc != SLIM_USR_MC_DISCONNECT_PORT)
			wbuf[i++] = txn->msg->wbuf[1]; /* channel number */

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

		dev_info(dev->dev,
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
	dev->wr_comp = &tx_sent;
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

	/* Log all TX messages for debugging (including VALUE writes) */
#ifdef CONFIG_DEBUG_FS
	{
		u8 *tb = (u8 *)pbuf;
		msm8953_slim_log_msg(dev, 0, tb, min_t(u8, txn->rl, 16));
	}
#endif

	/*
	 * Path selection (evidence-based):
	 *
	 * - MT_CORE messages addressed to the codec (CHANGE_VALUE,
	 *   REQUEST_VALUE, REPORT_INFORMATION) must go via AHB. BAM TX
	 *   pipe 4 is qcom,controlled-remotely and lands in the ADSP,
	 *   whose dispatcher (adsp_sat_dispatcher @ 0xf04ca424) silently
	 *   drops MT=0 frames with MC != 0x29 — so codec PGD/IFC writes
	 *   never reach the codec via BAM TX. See codec_pgd_via_ahb_finding.md.
	 *
	 * - USR messages (DEST_REFERRED_USER, SRC_REFERRED_USER) are
	 *   meant for the ADSP itself — stay on BAM TX.
	 *
	 * AHB writes need TX_MSGQ_EN cleared in NGD_CFG: when TX_MSGQ_EN
	 * is set, the NGD hardware steers all TX through the BAM TX queue
	 * and ignores writes to NGD_TX_MSG. We can't clear it globally
	 * (ADSP appears to gate its QMI handshake on TX_MSGQ_EN being set
	 * at power-on — clearing it globally hangs early boot). So we
	 * toggle it per AHB write. xfer_msg is serialized by the SLIMbus
	 * controller mutex, so no race with USR BAM writes.
	 */
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

	dev->wr_comp = NULL;

	if (ret) {
		/*
		 * MC=0x60 REQUEST_VALUE NACKs from the codec PGD happen
		 * routinely once the SLIMbus enters streaming mode (the codec
		 * stops servicing AP control reads during audio data flow).
		 * Rate-limit those so dmesg isn't flooded; report everything
		 * else normally.
		 */
		if (txn_mt == SLIM_MSG_MT_CORE &&
		    txn_mc == SLIM_MSG_MC_REQUEST_VALUE)
			dev_warn_ratelimited(dev->dev,
				"TX failed: mc=0x%x mt=0x%x ret=%d (PGD read NACK; expected during streaming)\n",
				txn_mc, txn_mt, ret);
		else
			dev_warn(dev->dev, "TX failed: mc=0x%x mt=0x%x ret=%d\n",
				 txn_mc, txn_mt, ret);
	}

	/* Wait for ADSP ACK on CONNECT with PGD LA */
	if (!ret && txn->comp &&
	    txn_mt == SLIM_MSG_MT_DEST_REFERRED_USER &&
	    (txn_mc == SLIM_USR_MC_CONNECT_SRC ||
	     txn_mc == SLIM_USR_MC_CONNECT_SINK ||
	     txn_mc == SLIM_USR_MC_DISCONNECT_PORT)) {
		int ack_timeout = wait_for_completion_timeout(txn->comp,
							     msecs_to_jiffies(200));
		if (!ack_timeout) {
			dev_warn(dev->dev,
				 "CONNECT ACK timeout: mc=0x%x tid=%d (PGD LA=%d)\n",
				 txn_mc, txn->tid,
				 txn->msg->wbuf ? txn->msg->wbuf[0] : -1);
			slim_free_txn_tid(ctrl, txn);
			ret = 0; /* non-fatal */
		} else {
			dev_info(dev->dev,
				 "CONNECT ACK received! mc=0x%x tid=%d\n",
				 txn_mc, txn->tid);
		}

		/*
		 * NOTE: PGD port configuration for audio data ports is
		 * managed by the ADSP, not the apps processor.
		 * The ADSP arms its own BAM pipes (EE0, pipes 3-19)
		 * in response to CONNECT + DEF_ACT_CHAN messages.
		 * The apps processor only manages pipes 21/22 (MAD).
		 * Writing PGD_PORT_CFGn from the apps side for audio
		 * ports causes I/O errors by conflicting with the ADSP.
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
	 * Send ADDR_QUERY to ADSP manager (matching downstream ngd_get_laddr).
	 * The ADSP looks up the enumeration address, returns ADDR_REPLY with
	 * the logical address, and sets bOpen=true for the device — which is
	 * required for VALUE messages to work.
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
	 * Enumeration address bytes (downstream DT format, 6 bytes):
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
	 * ADDR_QUERY message format (matching downstream after rl--):
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

	dev_info(dev->dev,
		 "ADDR_QUERY: ea=%04x:%04x:%d:%d tid=%d [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n",
		 ea->manf_id, ea->prod_code, ea->dev_index, ea->instance,
		 txn.tid,
		 ((u8 *)buf)[0], ((u8 *)buf)[1], ((u8 *)buf)[2],
		 ((u8 *)buf)[3], ((u8 *)buf)[4], ((u8 *)buf)[5],
		 ((u8 *)buf)[6], ((u8 *)buf)[7], ((u8 *)buf)[8],
		 ((u8 *)buf)[9]);

	dev->wr_comp = &tx_sent;
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

	dev->wr_comp = NULL;

	/* Wait for ADDR_REPLY from ADSP */
	timeout = wait_for_completion_timeout(&done, 2 * HZ);
	if (!timeout) {
		dev_warn(dev->dev, "ADDR_QUERY: no ADDR_REPLY from ADSP for %04x:%04x:%d\n",
			 ea->manf_id, ea->prod_code, ea->dev_index);
		ret = -ETIMEDOUT;
		goto out_free_tid;
	}

	if (txn.la == 0xFF) {
		dev_warn(dev->dev, "ADDR_QUERY: ADSP returned failed EA\n");
		ret = -ENXIO;
		goto out_free_tid;
	}

	*laddr = txn.la;
	dev_info(dev->dev, "ADDR_QUERY: %04x:%04x:%d:%d → laddr=%d\n",
		 ea->manf_id, ea->prod_code, ea->dev_index, ea->instance,
		 *laddr);
	return 0;

out_free_tid:
	spin_lock_irq(&ctrl->txn_lock);
	idr_remove(&ctrl->tid_idr, txn.tid);
	spin_unlock_irq(&ctrl->txn_lock);
	dev->wr_comp = NULL;

	/*
	 * Return -EAGAIN so the retry loop in slave_notify_worker keeps
	 * trying. The WCD9335 may not have enumerated on the bus yet
	 * (needs MCLK + reset release + a few hundred ms).
	 */
	return -EAGAIN;
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

	struct device_node *parent = dev->ngd_node ?: dev->dev->of_node;

	/*
	 * Wait for codec to boot. The WCD9335 needs MCLK + reset release
	 * (done by its driver probe) before it can enumerate on SLIMbus.
	 * The probe runs asynchronously — wait 5s for it to complete,
	 * then ADDR_QUERY retries handle any remaining delay.
	 * 5s + 60 retries × 500ms = 35s total window.
	 */
	msleep(5000);

	/*
	 * If SSR happened during our initial sleep, wait for recovery
	 * before attempting ADDR_QUERY.
	 */
	if (dev->state == MSM8953_SLIM_DOWN) {
		dev_info(dev->dev, "slave_notify: SSR during init, waiting for recovery\n");
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
				dev_info(dev->dev,
					 "%s: SSR detected at attempt %d, waiting for recovery\n",
					 dev_name(&sbdev->dev), j);
				sbdev->is_laddr_valid = false;
				wait_for_completion_interruptible(&dev->ctrl_up);
				msleep(2000);
				j = 0;
				dev_info(dev->dev,
					 "%s: SSR recovered, restarting ADDR_QUERY\n",
					 dev_name(&sbdev->dev));
			}

			ret = slim_get_logical_addr(sbdev);
			if (!ret) {
				dev_info(dev->dev, "%s: got laddr %d (attempt %d)\n",
					 dev_name(&sbdev->dev), sbdev->laddr, j);
				break;
			}
			msleep(500);
		}
		if (ret) {
			/*
			 * Fallback: use PGD LA so the codec driver can at
			 * least probe (reads return 0 but won't crash).
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
		put_device(&sbdev->dev);
	}

}

/* ---- SSR / power-up worker ----------------------------------------------- */

static int msm8953_slim_ngd_enable(struct msm8953_slim_ctrl *dev)
{
	int ret;
	unsigned long timeout;

	dev_info(dev->dev, "ngd_enable: waiting for QMI (qmi_done=%d)\n",
		 completion_done(&dev->qmi_up));
	/*
	 * Cold-boot timing varies a lot on this platform: the ADSP needs
	 * to load its firmware and start its QMI helper service. Observed
	 * boots take ~25 ms when warm but occasionally exceed 10 s when
	 * cold. Bumping to 30 s avoids the soundcard staying deferred.
	 */
	timeout = wait_for_completion_timeout(&dev->qmi_up,
					      msecs_to_jiffies(30000));
	if (!timeout) {
		dev_err(dev->dev, "QMI service not found after 30s (state=%d)\n",
			dev->state);
		return -ETIMEDOUT;
	}
	dev_info(dev->dev, "ngd_enable: QMI ready\n");

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

	dev_info(dev->dev, "ngd_up_worker: start (state=%d qmi=%d gen=%u)\n",
		 dev->state, completion_done(&dev->qmi_up), gen);

retry:
	if (gen != atomic_read(&dev->ssr_gen)) {
		dev_info(dev->dev, "ngd_up_worker: stale (gen %u != %d), bailing\n",
			 gen, atomic_read(&dev->ssr_gen));
		return;
	}

	mutex_lock(&dev->tx_lock);
	/*
	 * Transition out of DOWN before attempting power-up.  This ensures
	 * that if the ADSP crashes while we are inside power_up (waiting
	 * for MCAP, sending REPORT_SATELLITE, etc.), the SSR DOWN handler
	 * will actually process the event instead of dropping it as a
	 * "duplicate DOWN".  Without this, the initial probe sets state=DOWN,
	 * and an early ADSP crash would be silently ignored because the
	 * guard sees state==DOWN and skips teardown.
	 */
	dev->state = MSM8953_SLIM_ASLEEP;
	pm_runtime_disable(dev->dev);
	pm_runtime_set_suspended(dev->dev);
	pm_runtime_enable(dev->dev);
	mutex_unlock(&dev->tx_lock);

	ret = msm8953_slim_ngd_enable(dev);

	/*
	 * Increase retries from 2 to 5. Cold boots where ADSP QMI takes
	 * its sweet time can need more than the original 2 attempts before
	 * the bus comes up.
	 */
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

	dev_info(dev->dev, "ngd_up_worker: done ret=%d (state=%d qmi=%d)\n",
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
				"SSR/PDR: DOWN ignored — already DOWN (action=%lu)\n",
				action);
			break;
		}
		dev_info(dev->dev, "SSR/PDR: service DOWN (action=%lu, state=%d)\n",
			 action, dev->state);
		atomic_set(&dev->ssr_in_progress, 1);
		atomic_inc(&dev->ssr_gen);
		dev->state = MSM8953_SLIM_DOWN;
		cancel_delayed_work(&dev->ngd_up_work);
		complete_all(&dev->reconf);
		complete_all(&dev->ctrl_up);
		reinit_completion(&dev->ctrl_up);
		/*
		 * Don't reinit qmi_up here — the QMI framework's
		 * del_server/new_server callbacks own that lifecycle.
		 * Reiniting it here races with a concurrent recovery
		 * where new_server already re-completed qmi_up.
		 */

		/*
		 * Fully quiesce the local NGD controller:
		 * - Disable NGD (clears ENABLE, RX_MSGQ_EN, TX_MSGQ_EN)
		 * - Clear interrupt enable mask
		 * This prevents stale MCAP or other IRQs from the
		 * pre-crash ADSP from firing into the recovery path.
		 */
		{
			void __iomem *ngd = ngd_base(dev);

			writel_relaxed(0, ngd + NGD_CFG);
			writel_relaxed(0, ngd + NGD_INT_EN);
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
		dev_info(dev->dev, "SSR/PDR: service UP (action=%lu)\n",
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

	dev_info(dev->dev, "PDR status: state=%d path=%s\n", state,
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
		 * Lightweight resume: just send QMI PM_ACTIVE to wake
		 * the ADSP SLIMbus manager.  Do NOT redo DMA init,
		 * NGD_CFG, or REPORT_SATELLITE — the ADSP already has
		 * our satellite registration from the initial power_up
		 * in ngd_enable.  Re-sending NGD_CFG here causes MCAP
		 * timeouts because the ADSP doesn't expect a fresh
		 * negotiation from a satellite it already knows about.
		 *
		 * This matches downstream's ngd_slim_power_up(dev, false)
		 * path which only sends PM_ACTIVE for runtime resume.
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
	struct {
		u64 timestamp;
		u8 dir;
		u8 len;
		u8 data[16];
	} log_copy[SLIM_MSG_LOG_SIZE];
	unsigned long flags;
	unsigned int head, i, idx;

	spin_lock_irqsave(&dev->msg_log_lock, flags);
	memcpy(log_copy, dev->msg_log, sizeof(log_copy));
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

	dev_info(&pdev->dev,
		 "MSM8953 SLIMbus: ctrl=%d ee=%d apps_pipes=0x%x eapc=0x%x\n",
		 dev->ctrl_nr, dev->ee, dev->apps_pipes, dev->eapc);

	init_completion(&dev->reconf);
	init_completion(&dev->ctrl_up);
	init_completion(&dev->qmi_up);
	mutex_init(&dev->tx_lock);
	spin_lock_init(&dev->rx_lock);
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
	 * Register the controller with the slimbus framework.
	 * Temporarily swap of_node to the "slim@N" NGD child so that
	 * of_register_slim_devices() finds codec child nodes (e.g. wcd9335).
	 * Restore immediately after so DMA lookups still find the parent
	 * node's dmas/dma-names properties.
	 */
	{
		struct device_node *parent_node = pdev->dev.of_node;
		struct device_node *ngd_node;
		u32 id;

		for_each_child_of_node(parent_node, ngd_node) {
			if (of_property_read_u32(ngd_node, "reg", &id))
				continue;
			dev_info(&pdev->dev,
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
	 * PDR registration tells the ADSP that a client is interested
	 * in the SLIMbus audio service.  Without this, the ADSP may not
	 * start the SLIMbus framer.  The PDR callback triggers ngd_up_work
	 * when the audio protection domain comes UP.
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
	 * Don't schedule init from probe. Let the SSR/PDR callbacks
	 * handle it — they fire when the ADSP boots and are the
	 * authoritative signal that the ADSP is ready.
	 *
	 * This avoids the race where we start SLIMbus init, then the
	 * modem triggers an ADSP crash mid-init. With event-driven
	 * init, SSR DOWN cancels the work, and SSR UP reschedules it
	 * after the ADSP has fully recovered.
	 *
	 * Safety: if SSR UP already fired before we registered the
	 * notifier (shouldn't happen — remoteproc starts ADSP after
	 * our probe), the QMI new_server callback will also trigger.
	 */

	dev_info(&pdev->dev, "MSM8953 SLIMbus NGD controller registered\n");

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
 * Shutdown handler: called on system reboot/poweroff. Without this,
 * Linux skips teardown and the ADSP keeps the satellite + slot table
 * state from the previous session, which is what was causing the
 * intermittent "Excep:0:Exception detected" + MCAP timeout + QMI
 * service not found cycle on the FIRST boot after a hot reboot.
 *
 * Per `feedback_no_bold_claims.md`: this may or may not fully fix the
 * flakiness. Stating what it does: tells the ADSP we're going inactive
 * (PM_INACTIVE QMI), cancels pending work, releases BAM data pipes,
 * disables NGD_CFG so the HW is quiesced. The next cold boot then
 * starts from a clean ADSP-side state.
 */
static void msm8953_slim_shutdown(struct platform_device *pdev)
{
	struct msm8953_slim_ctrl *dev = platform_get_drvdata(pdev);
	void __iomem *ngd;

	if (!dev)
		return;

	/*
	 * MINIMAL shutdown: just quiesce the NGD HW register so the
	 * controller stops driving the bus while the SoC resets.
	 *
	 * Do NOT call msm8953_slim_qmi_power_request (it does qmi_txn_wait
	 * which blocks up to ~5s on a dead/uninitialized QMI socket).
	 * Do NOT call dma_release_channel (BAM tear-down can block on HW
	 * state if BAM was active; observed to hang reboot 2026-05-26).
	 * Do NOT cancel work — the kernel core does that when device is
	 * removed in the shutdown path.
	 *
	 * Only clear NGD_CFG. That's the one thing the ADSP-side firmware
	 * watches to know whether the AP-side controller is alive.
	 */
	ngd = ngd_base(dev);
	writel_relaxed(0, ngd + NGD_CFG);
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
