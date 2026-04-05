// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2018, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/soc/qcom/apr.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "q6dsp-errno.h"
#include "q6core.h"
#include "q6afe.h"

/* AFE CMDs */
#define AFE_PORT_CMD_DEVICE_START	0x000100E5
#define AFE_PORT_CMD_DEVICE_STOP	0x000100E6
#define AFE_PORT_CMD_SET_PARAM_V2	0x000100EF
#define AFE_SVC_CMD_SET_PARAM		0x000100f3
#define AFE_PORT_CMDRSP_GET_PARAM_V2	0x00010106
#define AFE_PARAM_ID_HDMI_CONFIG	0x00010210
#define AFE_MODULE_AUDIO_DEV_INTERFACE	0x0001020C
#define AFE_MODULE_TDM			0x0001028A

#define AFE_MODULE_CDC_DEV_CFG		0x00010234
#define AFE_PARAM_ID_CDC_SLIMBUS_SLAVE_CFG 0x00010235
#define AFE_PARAM_ID_CDC_REG_CFG	0x00010236
#define AFE_PARAM_ID_CDC_REG_CFG_INIT	0x00010237
#define AFE_PARAM_ID_USB_AUDIO_DEV_PARAMS    0x000102A5
#define AFE_PARAM_ID_USB_AUDIO_DEV_LPCM_FMT 0x000102AA

#define AFE_PARAM_ID_SET_TOPOLOGY	0x0001025A
#define AFE_PARAM_ID_LPAIF_CLK_CONFIG	0x00010238
#define AFE_PARAM_ID_INT_DIGITAL_CDC_CLK_CONFIG	0x00010239

#define AFE_PARAM_ID_SLIMBUS_CONFIG    0x00010212
#define AFE_PARAM_ID_I2S_CONFIG	0x0001020D
#define AFE_PARAM_ID_TDM_CONFIG	0x0001029D
#define AFE_PARAM_ID_PORT_SLOT_MAPPING_CONFIG	0x00010297
#define AFE_PARAM_ID_CODEC_DMA_CONFIG	0x000102B8
#define AFE_PARAM_ID_USB_AUDIO_CONFIG    0x000102A4
#define AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST	0x000100f4
#define AFE_CMD_RSP_REMOTE_LPASS_CORE_HW_VOTE_REQUEST   0x000100f5
#define AFE_CMD_REMOTE_LPASS_CORE_HW_DEVOTE_REQUEST	0x000100f6

/* I2S config specific */
#define AFE_API_VERSION_I2S_CONFIG	0x1
#define AFE_PORT_I2S_SD0		0x1
#define AFE_PORT_I2S_SD1		0x2
#define AFE_PORT_I2S_SD2		0x3
#define AFE_PORT_I2S_SD3		0x4
#define AFE_PORT_I2S_SD0_MASK		BIT(0x0)
#define AFE_PORT_I2S_SD1_MASK		BIT(0x1)
#define AFE_PORT_I2S_SD2_MASK		BIT(0x2)
#define AFE_PORT_I2S_SD3_MASK		BIT(0x3)
#define AFE_PORT_I2S_SD0_1_MASK		GENMASK(1, 0)
#define AFE_PORT_I2S_SD2_3_MASK		GENMASK(3, 2)
#define AFE_PORT_I2S_SD0_1_2_MASK	GENMASK(2, 0)
#define AFE_PORT_I2S_SD0_1_2_3_MASK	GENMASK(3, 0)
#define AFE_PORT_I2S_QUAD01		0x5
#define AFE_PORT_I2S_QUAD23		0x6
#define AFE_PORT_I2S_6CHS		0x7
#define AFE_PORT_I2S_8CHS		0x8
#define AFE_PORT_I2S_MONO		0x0
#define AFE_PORT_I2S_STEREO		0x1
#define AFE_PORT_CONFIG_I2S_WS_SRC_EXTERNAL	0x0
#define AFE_PORT_CONFIG_I2S_WS_SRC_INTERNAL	0x1
#define AFE_LINEAR_PCM_DATA				0x0

#define AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG 0x1

/* Port IDs */
#define AFE_API_VERSION_HDMI_CONFIG	0x1
#define AFE_PORT_ID_MULTICHAN_HDMI_RX	0x100E
#define AFE_PORT_ID_HDMI_OVER_DP_RX	0x6020

/* USB AFE port */
#define AFE_PORT_ID_USB_RX                       0x7000

#define AFE_API_VERSION_SLIMBUS_CONFIG 0x1
/* Clock set API version */
#define AFE_API_VERSION_CLOCK_SET 1
#define Q6AFE_LPASS_CLK_CONFIG_API_VERSION	0x1
#define AFE_MODULE_CLOCK_SET		0x0001028F
#define AFE_PARAM_ID_CLOCK_SET		0x00010290

/* SLIMbus Rx port on channel 0. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_RX      0x4000
/* SLIMbus Tx port on channel 0. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_TX      0x4001
/* SLIMbus Rx port on channel 1. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_RX      0x4002
/* SLIMbus Tx port on channel 1. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_TX      0x4003
/* SLIMbus Rx port on channel 2. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_RX      0x4004
/* SLIMbus Tx port on channel 2. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_TX      0x4005
/* SLIMbus Rx port on channel 3. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_RX      0x4006
/* SLIMbus Tx port on channel 3. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_TX      0x4007
/* SLIMbus Rx port on channel 4. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_RX      0x4008
/* SLIMbus Tx port on channel 4. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_TX      0x4009
/* SLIMbus Rx port on channel 5. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX      0x400a
/* SLIMbus Tx port on channel 5. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_TX      0x400b
/* SLIMbus Rx port on channel 6. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX      0x400c
/* SLIMbus Tx port on channel 6. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_TX      0x400d
#define AFE_PORT_ID_PRIMARY_MI2S_RX         0x1000
#define AFE_PORT_ID_PRIMARY_MI2S_TX         0x1001
#define AFE_PORT_ID_SECONDARY_MI2S_RX       0x1002
#define AFE_PORT_ID_SECONDARY_MI2S_TX       0x1003
#define AFE_PORT_ID_TERTIARY_MI2S_RX        0x1004
#define AFE_PORT_ID_TERTIARY_MI2S_TX        0x1005
#define AFE_PORT_ID_QUATERNARY_MI2S_RX      0x1006
#define AFE_PORT_ID_QUATERNARY_MI2S_TX      0x1007
#define AFE_PORT_ID_QUINARY_MI2S_RX	    0x1016
#define AFE_PORT_ID_QUINARY_MI2S_TX	    0x1017

/* Start of the range of port IDs for TDM devices. */
#define AFE_PORT_ID_TDM_PORT_RANGE_START	0x9000

/* End of the range of port IDs for TDM devices. */
#define AFE_PORT_ID_TDM_PORT_RANGE_END \
	(AFE_PORT_ID_TDM_PORT_RANGE_START+0x50-1)

/* Size of the range of port IDs for TDM ports. */
#define AFE_PORT_ID_TDM_PORT_RANGE_SIZE \
	(AFE_PORT_ID_TDM_PORT_RANGE_END - \
	AFE_PORT_ID_TDM_PORT_RANGE_START+1)

#define AFE_PORT_ID_PRIMARY_TDM_RX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x00)
#define AFE_PORT_ID_PRIMARY_TDM_RX_1 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x02)
#define AFE_PORT_ID_PRIMARY_TDM_RX_2 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x04)
#define AFE_PORT_ID_PRIMARY_TDM_RX_3 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x06)
#define AFE_PORT_ID_PRIMARY_TDM_RX_4 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x08)
#define AFE_PORT_ID_PRIMARY_TDM_RX_5 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x0A)
#define AFE_PORT_ID_PRIMARY_TDM_RX_6 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x0C)
#define AFE_PORT_ID_PRIMARY_TDM_RX_7 \
	(AFE_PORT_ID_PRIMARY_TDM_RX + 0x0E)

#define AFE_PORT_ID_PRIMARY_TDM_TX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x01)
#define AFE_PORT_ID_PRIMARY_TDM_TX_1 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x02)
#define AFE_PORT_ID_PRIMARY_TDM_TX_2 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x04)
#define AFE_PORT_ID_PRIMARY_TDM_TX_3 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x06)
#define AFE_PORT_ID_PRIMARY_TDM_TX_4 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x08)
#define AFE_PORT_ID_PRIMARY_TDM_TX_5 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x0A)
#define AFE_PORT_ID_PRIMARY_TDM_TX_6 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x0C)
#define AFE_PORT_ID_PRIMARY_TDM_TX_7 \
	(AFE_PORT_ID_PRIMARY_TDM_TX + 0x0E)

#define AFE_PORT_ID_SECONDARY_TDM_RX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x10)
#define AFE_PORT_ID_SECONDARY_TDM_RX_1 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x02)
#define AFE_PORT_ID_SECONDARY_TDM_RX_2 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x04)
#define AFE_PORT_ID_SECONDARY_TDM_RX_3 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x06)
#define AFE_PORT_ID_SECONDARY_TDM_RX_4 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x08)
#define AFE_PORT_ID_SECONDARY_TDM_RX_5 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x0A)
#define AFE_PORT_ID_SECONDARY_TDM_RX_6 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x0C)
#define AFE_PORT_ID_SECONDARY_TDM_RX_7 \
	(AFE_PORT_ID_SECONDARY_TDM_RX + 0x0E)

#define AFE_PORT_ID_SECONDARY_TDM_TX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x11)
#define AFE_PORT_ID_SECONDARY_TDM_TX_1 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x02)
#define AFE_PORT_ID_SECONDARY_TDM_TX_2 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x04)
#define AFE_PORT_ID_SECONDARY_TDM_TX_3 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x06)
#define AFE_PORT_ID_SECONDARY_TDM_TX_4 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x08)
#define AFE_PORT_ID_SECONDARY_TDM_TX_5 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x0A)
#define AFE_PORT_ID_SECONDARY_TDM_TX_6 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x0C)
#define AFE_PORT_ID_SECONDARY_TDM_TX_7 \
	(AFE_PORT_ID_SECONDARY_TDM_TX + 0x0E)

#define AFE_PORT_ID_TERTIARY_TDM_RX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x20)
#define AFE_PORT_ID_TERTIARY_TDM_RX_1 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x02)
#define AFE_PORT_ID_TERTIARY_TDM_RX_2 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x04)
#define AFE_PORT_ID_TERTIARY_TDM_RX_3 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x06)
#define AFE_PORT_ID_TERTIARY_TDM_RX_4 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x08)
#define AFE_PORT_ID_TERTIARY_TDM_RX_5 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x0A)
#define AFE_PORT_ID_TERTIARY_TDM_RX_6 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x0C)
#define AFE_PORT_ID_TERTIARY_TDM_RX_7 \
	(AFE_PORT_ID_TERTIARY_TDM_RX + 0x0E)

#define AFE_PORT_ID_TERTIARY_TDM_TX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x21)
#define AFE_PORT_ID_TERTIARY_TDM_TX_1 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x02)
#define AFE_PORT_ID_TERTIARY_TDM_TX_2 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x04)
#define AFE_PORT_ID_TERTIARY_TDM_TX_3 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x06)
#define AFE_PORT_ID_TERTIARY_TDM_TX_4 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x08)
#define AFE_PORT_ID_TERTIARY_TDM_TX_5 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x0A)
#define AFE_PORT_ID_TERTIARY_TDM_TX_6 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x0C)
#define AFE_PORT_ID_TERTIARY_TDM_TX_7 \
	(AFE_PORT_ID_TERTIARY_TDM_TX + 0x0E)

#define AFE_PORT_ID_QUATERNARY_TDM_RX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x30)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_1 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x02)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_2 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x04)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_3 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x06)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_4 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x08)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_5 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x0A)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_6 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x0C)
#define AFE_PORT_ID_QUATERNARY_TDM_RX_7 \
	(AFE_PORT_ID_QUATERNARY_TDM_RX + 0x0E)

#define AFE_PORT_ID_QUATERNARY_TDM_TX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x31)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_1 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x02)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_2 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x04)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_3 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x06)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_4 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x08)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_5 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x0A)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_6 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x0C)
#define AFE_PORT_ID_QUATERNARY_TDM_TX_7 \
	(AFE_PORT_ID_QUATERNARY_TDM_TX + 0x0E)

#define AFE_PORT_ID_QUINARY_TDM_RX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x40)
#define AFE_PORT_ID_QUINARY_TDM_RX_1 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x02)
#define AFE_PORT_ID_QUINARY_TDM_RX_2 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x04)
#define AFE_PORT_ID_QUINARY_TDM_RX_3 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x06)
#define AFE_PORT_ID_QUINARY_TDM_RX_4 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x08)
#define AFE_PORT_ID_QUINARY_TDM_RX_5 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x0A)
#define AFE_PORT_ID_QUINARY_TDM_RX_6 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x0C)
#define AFE_PORT_ID_QUINARY_TDM_RX_7 \
	(AFE_PORT_ID_QUINARY_TDM_RX + 0x0E)

#define AFE_PORT_ID_QUINARY_TDM_TX \
	(AFE_PORT_ID_TDM_PORT_RANGE_START + 0x41)
#define AFE_PORT_ID_QUINARY_TDM_TX_1 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x02)
#define AFE_PORT_ID_QUINARY_TDM_TX_2 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x04)
#define AFE_PORT_ID_QUINARY_TDM_TX_3 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x06)
#define AFE_PORT_ID_QUINARY_TDM_TX_4 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x08)
#define AFE_PORT_ID_QUINARY_TDM_TX_5 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x0A)
#define AFE_PORT_ID_QUINARY_TDM_TX_6 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x0C)
#define AFE_PORT_ID_QUINARY_TDM_TX_7 \
	(AFE_PORT_ID_QUINARY_TDM_TX + 0x0E)

/* AFE WSA Codec DMA Rx port 0 */
#define AFE_PORT_ID_WSA_CODEC_DMA_RX_0	0xB000
/* AFE WSA Codec DMA Tx port 0 */
#define AFE_PORT_ID_WSA_CODEC_DMA_TX_0	0xB001
/* AFE WSA Codec DMA Rx port 1 */
#define AFE_PORT_ID_WSA_CODEC_DMA_RX_1	0xB002
/* AFE WSA Codec DMA Tx port 1 */
#define AFE_PORT_ID_WSA_CODEC_DMA_TX_1	0xB003
/* AFE WSA Codec DMA Tx port 2 */
#define AFE_PORT_ID_WSA_CODEC_DMA_TX_2	0xB005
/* AFE VA Codec DMA Tx port 0 */
#define AFE_PORT_ID_VA_CODEC_DMA_TX_0	0xB021
/* AFE VA Codec DMA Tx port 1 */
#define AFE_PORT_ID_VA_CODEC_DMA_TX_1	0xB023
/* AFE VA Codec DMA Tx port 2 */
#define AFE_PORT_ID_VA_CODEC_DMA_TX_2	0xB025
/* AFE Rx Codec DMA Rx port 0 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_0	0xB030
/* AFE Tx Codec DMA Tx port 0 */
#define AFE_PORT_ID_TX_CODEC_DMA_TX_0	0xB031
/* AFE Rx Codec DMA Rx port 1 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_1	0xB032
/* AFE Tx Codec DMA Tx port 1 */
#define AFE_PORT_ID_TX_CODEC_DMA_TX_1	0xB033
/* AFE Rx Codec DMA Rx port 2 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_2	0xB034
/* AFE Tx Codec DMA Tx port 2 */
#define AFE_PORT_ID_TX_CODEC_DMA_TX_2	0xB035
/* AFE Rx Codec DMA Rx port 3 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_3	0xB036
/* AFE Tx Codec DMA Tx port 3 */
#define AFE_PORT_ID_TX_CODEC_DMA_TX_3	0xB037
/* AFE Rx Codec DMA Rx port 4 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_4	0xB038
/* AFE Tx Codec DMA Tx port 4 */
#define AFE_PORT_ID_TX_CODEC_DMA_TX_4	0xB039
/* AFE Rx Codec DMA Rx port 5 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_5	0xB03A
/* AFE Tx Codec DMA Tx port 5 */
#define AFE_PORT_ID_TX_CODEC_DMA_TX_5	0xB03B
/* AFE Rx Codec DMA Rx port 6 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_6	0xB03C
/* AFE Rx Codec DMA Rx port 7 */
#define AFE_PORT_ID_RX_CODEC_DMA_RX_7	0xB03E

#define Q6AFE_LPASS_MODE_CLK1_VALID 1
#define Q6AFE_LPASS_MODE_CLK2_VALID 2
#define Q6AFE_LPASS_CLK_SRC_INTERNAL 1
#define Q6AFE_LPASS_CLK_ROOT_DEFAULT 0
#define AFE_API_VERSION_TDM_CONFIG              1
#define AFE_API_VERSION_SLOT_MAPPING_CONFIG	1
#define AFE_API_VERSION_CODEC_DMA_CONFIG	1

#define TIMEOUT_MS 3000
#define AFE_CMD_RESP_AVAIL	0
#define AFE_CMD_RESP_NONE	1
#define AFE_CLK_TOKEN		1024

struct afe_param_cdc_slimbus_slave_cfg {
	u32 minor_version;
	u32 device_enum_addr_lsw;
	u32 device_enum_addr_msw;
	u16 tx_slave_port_offset;
	u16 rx_slave_port_offset;
} __packed;

struct q6afe {
	struct apr_device *apr;
	struct device *dev;
	struct q6core_svc_api_info ainfo;
	struct mutex lock;
	struct aprv2_ibasic_rsp_result_t result;
	wait_queue_head_t wait;
	struct list_head port_list;
	spinlock_t port_list_lock;
	bool slim_slave_cfg_sent;
	u32 mmap_handle;
};

struct afe_port_cmd_device_start {
	u16 port_id;
	u16 reserved;
} __packed;

struct afe_port_cmd_device_stop {
	u16 port_id;
	u16 reserved;
/* Reserved for 32-bit alignment. This field must be set to 0.*/
} __packed;

struct afe_port_param_data_v2 {
	u32 module_id;
	u32 param_id;
	u16 param_size;
	u16 reserved;
} __packed;

struct afe_svc_cmd_set_param {
	uint32_t payload_size;
	uint32_t payload_address_lsw;
	uint32_t payload_address_msw;
	uint32_t mem_map_handle;
} __packed;

struct afe_port_cmd_set_param_v2 {
	u16 port_id;
	u16 payload_size;
	u32 payload_address_lsw;
	u32 payload_address_msw;
	u32 mem_map_handle;
} __packed;

struct afe_param_id_hdmi_multi_chan_audio_cfg {
	u32 hdmi_cfg_minor_version;
	u16 datatype;
	u16 channel_allocation;
	u32 sample_rate;
	u16 bit_width;
	u16 reserved;
} __packed;

struct afe_param_id_slimbus_cfg {
	u32                  sb_cfg_minor_version;
/* Minor version used for tracking the version of the SLIMBUS
 * configuration interface.
 * Supported values: #AFE_API_VERSION_SLIMBUS_CONFIG
 */

	u16                  slimbus_dev_id;
/* SLIMbus hardware device ID, which is required to handle
 * multiple SLIMbus hardware blocks.
 * Supported values: - #AFE_SLIMBUS_DEVICE_1 - #AFE_SLIMBUS_DEVICE_2
 */
	u16                  bit_width;
/* Bit width of the sample.
 * Supported values: 16, 24
 */
	u16                  data_format;
/* Data format supported by the SLIMbus hardware. The default is
 * 0 (#AFE_SB_DATA_FORMAT_NOT_INDICATED), which indicates the
 * hardware does not perform any format conversions before the data
 * transfer.
 */
	u16                  num_channels;
/* Number of channels.
 * Supported values: 1 to #AFE_PORT_MAX_AUDIO_CHAN_CNT
 */
	u8  shared_ch_mapping[AFE_PORT_MAX_AUDIO_CHAN_CNT];
/* Mapping of shared channel IDs (128 to 255) to which the
 * master port is to be connected.
 * Shared_channel_mapping[i] represents the shared channel assigned
 * for audio channel i in multichannel audio data.
 */
	u32              sample_rate;
/* Sampling rate of the port.
 * Supported values:
 * - #AFE_PORT_SAMPLE_RATE_8K
 * - #AFE_PORT_SAMPLE_RATE_16K
 * - #AFE_PORT_SAMPLE_RATE_48K
 * - #AFE_PORT_SAMPLE_RATE_96K
 * - #AFE_PORT_SAMPLE_RATE_192K
 */
} __packed;

struct afe_clk_cfg {
	u32                  i2s_cfg_minor_version;
	u32                  clk_val1;
	u32                  clk_val2;
	u16                  clk_src;
	u16                  clk_root;
	u16                  clk_set_mode;
	u16                  reserved;
} __packed;

struct afe_digital_clk_cfg {
	u32                  i2s_cfg_minor_version;
	u32                  clk_val;
	u16                  clk_root;
	u16                  reserved;
} __packed;

struct afe_param_id_i2s_cfg {
	u32	i2s_cfg_minor_version;
	u16	bit_width;
	u16	channel_mode;
	u16	mono_stereo;
	u16	ws_src;
	u32	sample_rate;
	u16	data_format;
	u16	reserved;
} __packed;

struct afe_param_id_tdm_cfg {
	u32	tdm_cfg_minor_version;
	u32	num_channels;
	u32	sample_rate;
	u32	bit_width;
	u16	data_format;
	u16	sync_mode;
	u16	sync_src;
	u16	nslots_per_frame;
	u16	ctrl_data_out_enable;
	u16	ctrl_invert_sync_pulse;
	u16	ctrl_sync_data_delay;
	u16	slot_width;
	u32	slot_mask;
} __packed;

struct afe_param_id_cdc_dma_cfg {
	u32	cdc_dma_cfg_minor_version;
	u32	sample_rate;
	u16	bit_width;
	u16	data_format;
	u16	num_channels;
	u16	active_channels_mask;
} __packed;

struct afe_param_id_usb_cfg {
/* Minor version used for tracking USB audio device configuration.
 * Supported values: AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG
 */
	u32	cfg_minor_version;
/* Sampling rate of the port.
 * Supported values:
 * - AFE_PORT_SAMPLE_RATE_8K
 * - AFE_PORT_SAMPLE_RATE_11025
 * - AFE_PORT_SAMPLE_RATE_12K
 * - AFE_PORT_SAMPLE_RATE_16K
 * - AFE_PORT_SAMPLE_RATE_22050
 * - AFE_PORT_SAMPLE_RATE_24K
 * - AFE_PORT_SAMPLE_RATE_32K
 * - AFE_PORT_SAMPLE_RATE_44P1K
 * - AFE_PORT_SAMPLE_RATE_48K
 * - AFE_PORT_SAMPLE_RATE_96K
 * - AFE_PORT_SAMPLE_RATE_192K
 */
	u32	sample_rate;
/* Bit width of the sample.
 * Supported values: 16, 24
 */
	u16	bit_width;
/* Number of channels.
 * Supported values: 1 and 2
 */
	u16	num_channels;
/* Data format supported by the USB. The supported value is
 * 0 (#AFE_USB_AUDIO_DATA_FORMAT_LINEAR_PCM).
 */
	u16	data_format;
/* this field must be 0 */
	u16	reserved;
/* device token of actual end USB audio device */
	u32	dev_token;
/* endianness of this interface */
	u32	endian;
/* service interval */
	u32	service_interval;
} __packed;

/**
 * struct afe_param_id_usb_audio_dev_params
 * @cfg_minor_version: Minor version used for tracking USB audio device
 * configuration.
 * Supported values:
 *     AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG
 * @dev_token: device token of actual end USB audio device
 **/
struct afe_param_id_usb_audio_dev_params {
	u32	cfg_minor_version;
	u32	dev_token;
} __packed;

/**
 * struct afe_param_id_usb_audio_dev_lpcm_fmt
 * @cfg_minor_version: Minor version used for tracking USB audio device
 * configuration.
 * Supported values:
 *     AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG
 * @endian: endianness of this interface
 **/
struct afe_param_id_usb_audio_dev_lpcm_fmt {
	u32	cfg_minor_version;
	u32	endian;
} __packed;

#define AFE_PARAM_ID_USB_AUDIO_SVC_INTERVAL     0x000102B7

/**
 * struct afe_param_id_usb_audio_svc_interval
 * @cfg_minor_version: Minor version used for tracking USB audio device
 * configuration.
 * Supported values:
 *     AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG
 * @svc_interval: service interval
 **/
struct afe_param_id_usb_audio_svc_interval {
	u32	cfg_minor_version;
	u32	svc_interval;
} __packed;

union afe_port_config {
	struct afe_param_id_hdmi_multi_chan_audio_cfg hdmi_multi_ch;
	struct afe_param_id_slimbus_cfg           slim_cfg;
	struct afe_param_id_i2s_cfg	i2s_cfg;
	struct afe_param_id_tdm_cfg	tdm_cfg;
	struct afe_param_id_cdc_dma_cfg	dma_cfg;
	struct afe_param_id_usb_cfg usb_cfg;
} __packed;


struct afe_clk_set {
	uint32_t clk_set_minor_version;
	uint32_t clk_id;
	uint32_t clk_freq_in_hz;
	uint16_t clk_attri;
	uint16_t clk_root;
	uint32_t enable;
};

struct afe_param_id_slot_mapping_cfg {
	u32	minor_version;
	u16	num_channels;
	u16	bitwidth;
	u32	data_align_type;
	u16	ch_mapping[AFE_PORT_MAX_AUDIO_CHAN_CNT];
} __packed;

struct q6afe_port {
	wait_queue_head_t wait;
	union afe_port_config port_cfg;
	struct afe_param_id_slot_mapping_cfg *scfg;
	struct aprv2_ibasic_rsp_result_t result;
	int token;
	int id;
	int cfg_type;
	struct q6afe *afe;
	struct kref refcount;
	struct list_head node;
};

struct afe_cmd_remote_lpass_core_hw_vote_request {
	uint32_t  hw_block_id;
	char client_name[8];
} __packed;

struct afe_cmd_remote_lpass_core_hw_devote_request {
	uint32_t  hw_block_id;
	uint32_t client_handle;
} __packed;



struct afe_port_map {
	int port_id;
	int token;
	int is_rx;
	int is_dig_pcm;
};

/*
 * Mapping between Virtual Port IDs to DSP AFE Port ID
 * On B Family SoCs DSP Port IDs are consistent across multiple SoCs
 * on A Family SoCs DSP port IDs are same as virtual Port IDs.
 */

static struct afe_port_map port_maps[AFE_PORT_MAX] = {
	[HDMI_RX] = { AFE_PORT_ID_MULTICHAN_HDMI_RX, HDMI_RX, 1, 1},
	[SLIMBUS_0_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_RX,
				SLIMBUS_0_RX, 1, 1},
	[SLIMBUS_1_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_RX,
				SLIMBUS_1_RX, 1, 1},
	[SLIMBUS_2_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_RX,
				SLIMBUS_2_RX, 1, 1},
	[SLIMBUS_3_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_RX,
				SLIMBUS_3_RX, 1, 1},
	[SLIMBUS_4_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_RX,
				SLIMBUS_4_RX, 1, 1},
	[SLIMBUS_5_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX,
				SLIMBUS_5_RX, 1, 1},
	[SLIMBUS_6_RX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX,
				SLIMBUS_6_RX, 1, 1},
	[SLIMBUS_0_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_TX,
				SLIMBUS_0_TX, 0, 1},
	[SLIMBUS_1_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_TX,
				SLIMBUS_1_TX, 0, 1},
	[SLIMBUS_2_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_TX,
				SLIMBUS_2_TX, 0, 1},
	[SLIMBUS_3_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_TX,
				SLIMBUS_3_TX, 0, 1},
	[SLIMBUS_4_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_TX,
				SLIMBUS_4_TX, 0, 1},
	[SLIMBUS_5_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_TX,
				SLIMBUS_5_TX, 0, 1},
	[SLIMBUS_6_TX] = { AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_TX,
				SLIMBUS_6_TX, 0, 1},
	[PRIMARY_MI2S_RX] = { AFE_PORT_ID_PRIMARY_MI2S_RX,
				PRIMARY_MI2S_RX, 1, 1},
	[PRIMARY_MI2S_TX] = { AFE_PORT_ID_PRIMARY_MI2S_TX,
				PRIMARY_MI2S_RX, 0, 1},
	[SECONDARY_MI2S_RX] = { AFE_PORT_ID_SECONDARY_MI2S_RX,
				SECONDARY_MI2S_RX, 1, 1},
	[SECONDARY_MI2S_TX] = { AFE_PORT_ID_SECONDARY_MI2S_TX,
				SECONDARY_MI2S_TX, 0, 1},
	[TERTIARY_MI2S_RX] = { AFE_PORT_ID_TERTIARY_MI2S_RX,
				TERTIARY_MI2S_RX, 1, 1},
	[TERTIARY_MI2S_TX] = { AFE_PORT_ID_TERTIARY_MI2S_TX,
				TERTIARY_MI2S_TX, 0, 1},
	[QUATERNARY_MI2S_RX] = { AFE_PORT_ID_QUATERNARY_MI2S_RX,
				QUATERNARY_MI2S_RX, 1, 1},
	[QUATERNARY_MI2S_TX] = { AFE_PORT_ID_QUATERNARY_MI2S_TX,
				QUATERNARY_MI2S_TX, 0, 1},
	[QUINARY_MI2S_RX]  =  { AFE_PORT_ID_QUINARY_MI2S_RX,
				QUINARY_MI2S_RX, 1, 1},
	[QUINARY_MI2S_TX] =   { AFE_PORT_ID_QUINARY_MI2S_TX,
				QUINARY_MI2S_TX, 0, 1},
	[PRIMARY_TDM_RX_0] =  { AFE_PORT_ID_PRIMARY_TDM_RX,
				PRIMARY_TDM_RX_0, 1, 1},
	[PRIMARY_TDM_TX_0] =  { AFE_PORT_ID_PRIMARY_TDM_TX,
				PRIMARY_TDM_TX_0, 0, 1},
	[PRIMARY_TDM_RX_1] =  { AFE_PORT_ID_PRIMARY_TDM_RX_1,
				PRIMARY_TDM_RX_1, 1, 1},
	[PRIMARY_TDM_TX_1] =  { AFE_PORT_ID_PRIMARY_TDM_TX_1,
				PRIMARY_TDM_TX_1, 0, 1},
	[PRIMARY_TDM_RX_2] =  { AFE_PORT_ID_PRIMARY_TDM_RX_2,
				PRIMARY_TDM_RX_2, 1, 1},
	[PRIMARY_TDM_TX_2] =  { AFE_PORT_ID_PRIMARY_TDM_TX_2,
				PRIMARY_TDM_TX_2, 0, 1},
	[PRIMARY_TDM_RX_3] =  { AFE_PORT_ID_PRIMARY_TDM_RX_3,
				PRIMARY_TDM_RX_3, 1, 1},
	[PRIMARY_TDM_TX_3] =  { AFE_PORT_ID_PRIMARY_TDM_TX_3,
				PRIMARY_TDM_TX_3, 0, 1},
	[PRIMARY_TDM_RX_4] =  { AFE_PORT_ID_PRIMARY_TDM_RX_4,
				PRIMARY_TDM_RX_4, 1, 1},
	[PRIMARY_TDM_TX_4] =  { AFE_PORT_ID_PRIMARY_TDM_TX_4,
				PRIMARY_TDM_TX_4, 0, 1},
	[PRIMARY_TDM_RX_5] =  { AFE_PORT_ID_PRIMARY_TDM_RX_5,
				PRIMARY_TDM_RX_5, 1, 1},
	[PRIMARY_TDM_TX_5] =  { AFE_PORT_ID_PRIMARY_TDM_TX_5,
				PRIMARY_TDM_TX_5, 0, 1},
	[PRIMARY_TDM_RX_6] =  { AFE_PORT_ID_PRIMARY_TDM_RX_6,
				PRIMARY_TDM_RX_6, 1, 1},
	[PRIMARY_TDM_TX_6] =  { AFE_PORT_ID_PRIMARY_TDM_TX_6,
				PRIMARY_TDM_TX_6, 0, 1},
	[PRIMARY_TDM_RX_7] =  { AFE_PORT_ID_PRIMARY_TDM_RX_7,
				PRIMARY_TDM_RX_7, 1, 1},
	[PRIMARY_TDM_TX_7] =  { AFE_PORT_ID_PRIMARY_TDM_TX_7,
				PRIMARY_TDM_TX_7, 0, 1},
	[SECONDARY_TDM_RX_0] =  { AFE_PORT_ID_SECONDARY_TDM_RX,
				SECONDARY_TDM_RX_0, 1, 1},
	[SECONDARY_TDM_TX_0] =  { AFE_PORT_ID_SECONDARY_TDM_TX,
				SECONDARY_TDM_TX_0, 0, 1},
	[SECONDARY_TDM_RX_1] =  { AFE_PORT_ID_SECONDARY_TDM_RX_1,
				SECONDARY_TDM_RX_1, 1, 1},
	[SECONDARY_TDM_TX_1] =  { AFE_PORT_ID_SECONDARY_TDM_TX_1,
				SECONDARY_TDM_TX_1, 0, 1},
	[SECONDARY_TDM_RX_2] =  { AFE_PORT_ID_SECONDARY_TDM_RX_2,
				SECONDARY_TDM_RX_2, 1, 1},
	[SECONDARY_TDM_TX_2] =  { AFE_PORT_ID_SECONDARY_TDM_TX_2,
				SECONDARY_TDM_TX_2, 0, 1},
	[SECONDARY_TDM_RX_3] =  { AFE_PORT_ID_SECONDARY_TDM_RX_3,
				SECONDARY_TDM_RX_3, 1, 1},
	[SECONDARY_TDM_TX_3] =  { AFE_PORT_ID_SECONDARY_TDM_TX_3,
				SECONDARY_TDM_TX_3, 0, 1},
	[SECONDARY_TDM_RX_4] =  { AFE_PORT_ID_SECONDARY_TDM_RX_4,
				SECONDARY_TDM_RX_4, 1, 1},
	[SECONDARY_TDM_TX_4] =  { AFE_PORT_ID_SECONDARY_TDM_TX_4,
				SECONDARY_TDM_TX_4, 0, 1},
	[SECONDARY_TDM_RX_5] =  { AFE_PORT_ID_SECONDARY_TDM_RX_5,
				SECONDARY_TDM_RX_5, 1, 1},
	[SECONDARY_TDM_TX_5] =  { AFE_PORT_ID_SECONDARY_TDM_TX_5,
				SECONDARY_TDM_TX_5, 0, 1},
	[SECONDARY_TDM_RX_6] =  { AFE_PORT_ID_SECONDARY_TDM_RX_6,
				SECONDARY_TDM_RX_6, 1, 1},
	[SECONDARY_TDM_TX_6] =  { AFE_PORT_ID_SECONDARY_TDM_TX_6,
				SECONDARY_TDM_TX_6, 0, 1},
	[SECONDARY_TDM_RX_7] =  { AFE_PORT_ID_SECONDARY_TDM_RX_7,
				SECONDARY_TDM_RX_7, 1, 1},
	[SECONDARY_TDM_TX_7] =  { AFE_PORT_ID_SECONDARY_TDM_TX_7,
				SECONDARY_TDM_TX_7, 0, 1},
	[TERTIARY_TDM_RX_0] =  { AFE_PORT_ID_TERTIARY_TDM_RX,
				TERTIARY_TDM_RX_0, 1, 1},
	[TERTIARY_TDM_TX_0] =  { AFE_PORT_ID_TERTIARY_TDM_TX,
				TERTIARY_TDM_TX_0, 0, 1},
	[TERTIARY_TDM_RX_1] =  { AFE_PORT_ID_TERTIARY_TDM_RX_1,
				TERTIARY_TDM_RX_1, 1, 1},
	[TERTIARY_TDM_TX_1] =  { AFE_PORT_ID_TERTIARY_TDM_TX_1,
				TERTIARY_TDM_TX_1, 0, 1},
	[TERTIARY_TDM_RX_2] =  { AFE_PORT_ID_TERTIARY_TDM_RX_2,
				TERTIARY_TDM_RX_2, 1, 1},
	[TERTIARY_TDM_TX_2] =  { AFE_PORT_ID_TERTIARY_TDM_TX_2,
				TERTIARY_TDM_TX_2, 0, 1},
	[TERTIARY_TDM_RX_3] =  { AFE_PORT_ID_TERTIARY_TDM_RX_3,
				TERTIARY_TDM_RX_3, 1, 1},
	[TERTIARY_TDM_TX_3] =  { AFE_PORT_ID_TERTIARY_TDM_TX_3,
				TERTIARY_TDM_TX_3, 0, 1},
	[TERTIARY_TDM_RX_4] =  { AFE_PORT_ID_TERTIARY_TDM_RX_4,
				TERTIARY_TDM_RX_4, 1, 1},
	[TERTIARY_TDM_TX_4] =  { AFE_PORT_ID_TERTIARY_TDM_TX_4,
				TERTIARY_TDM_TX_4, 0, 1},
	[TERTIARY_TDM_RX_5] =  { AFE_PORT_ID_TERTIARY_TDM_RX_5,
				TERTIARY_TDM_RX_5, 1, 1},
	[TERTIARY_TDM_TX_5] =  { AFE_PORT_ID_TERTIARY_TDM_TX_5,
				TERTIARY_TDM_TX_5, 0, 1},
	[TERTIARY_TDM_RX_6] =  { AFE_PORT_ID_TERTIARY_TDM_RX_6,
				TERTIARY_TDM_RX_6, 1, 1},
	[TERTIARY_TDM_TX_6] =  { AFE_PORT_ID_TERTIARY_TDM_TX_6,
				TERTIARY_TDM_TX_6, 0, 1},
	[TERTIARY_TDM_RX_7] =  { AFE_PORT_ID_TERTIARY_TDM_RX_7,
				TERTIARY_TDM_RX_7, 1, 1},
	[TERTIARY_TDM_TX_7] =  { AFE_PORT_ID_TERTIARY_TDM_TX_7,
				TERTIARY_TDM_TX_7, 0, 1},
	[QUATERNARY_TDM_RX_0] =  { AFE_PORT_ID_QUATERNARY_TDM_RX,
				QUATERNARY_TDM_RX_0, 1, 1},
	[QUATERNARY_TDM_TX_0] =  { AFE_PORT_ID_QUATERNARY_TDM_TX,
				QUATERNARY_TDM_TX_0, 0, 1},
	[QUATERNARY_TDM_RX_1] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_1,
				QUATERNARY_TDM_RX_1, 1, 1},
	[QUATERNARY_TDM_TX_1] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_1,
				QUATERNARY_TDM_TX_1, 0, 1},
	[QUATERNARY_TDM_RX_2] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_2,
				QUATERNARY_TDM_RX_2, 1, 1},
	[QUATERNARY_TDM_TX_2] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_2,
				QUATERNARY_TDM_TX_2, 0, 1},
	[QUATERNARY_TDM_RX_3] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_3,
				QUATERNARY_TDM_RX_3, 1, 1},
	[QUATERNARY_TDM_TX_3] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_3,
				QUATERNARY_TDM_TX_3, 0, 1},
	[QUATERNARY_TDM_RX_4] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_4,
				QUATERNARY_TDM_RX_4, 1, 1},
	[QUATERNARY_TDM_TX_4] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_4,
				QUATERNARY_TDM_TX_4, 0, 1},
	[QUATERNARY_TDM_RX_5] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_5,
				QUATERNARY_TDM_RX_5, 1, 1},
	[QUATERNARY_TDM_TX_5] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_5,
				QUATERNARY_TDM_TX_5, 0, 1},
	[QUATERNARY_TDM_RX_6] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_6,
				QUATERNARY_TDM_RX_6, 1, 1},
	[QUATERNARY_TDM_TX_6] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_6,
				QUATERNARY_TDM_TX_6, 0, 1},
	[QUATERNARY_TDM_RX_7] =  { AFE_PORT_ID_QUATERNARY_TDM_RX_7,
				QUATERNARY_TDM_RX_7, 1, 1},
	[QUATERNARY_TDM_TX_7] =  { AFE_PORT_ID_QUATERNARY_TDM_TX_7,
				QUATERNARY_TDM_TX_7, 0, 1},
	[QUINARY_TDM_RX_0] =  { AFE_PORT_ID_QUINARY_TDM_RX,
				QUINARY_TDM_RX_0, 1, 1},
	[QUINARY_TDM_TX_0] =  { AFE_PORT_ID_QUINARY_TDM_TX,
				QUINARY_TDM_TX_0, 0, 1},
	[QUINARY_TDM_RX_1] =  { AFE_PORT_ID_QUINARY_TDM_RX_1,
				QUINARY_TDM_RX_1, 1, 1},
	[QUINARY_TDM_TX_1] =  { AFE_PORT_ID_QUINARY_TDM_TX_1,
				QUINARY_TDM_TX_1, 0, 1},
	[QUINARY_TDM_RX_2] =  { AFE_PORT_ID_QUINARY_TDM_RX_2,
				QUINARY_TDM_RX_2, 1, 1},
	[QUINARY_TDM_TX_2] =  { AFE_PORT_ID_QUINARY_TDM_TX_2,
				QUINARY_TDM_TX_2, 0, 1},
	[QUINARY_TDM_RX_3] =  { AFE_PORT_ID_QUINARY_TDM_RX_3,
				QUINARY_TDM_RX_3, 1, 1},
	[QUINARY_TDM_TX_3] =  { AFE_PORT_ID_QUINARY_TDM_TX_3,
				QUINARY_TDM_TX_3, 0, 1},
	[QUINARY_TDM_RX_4] =  { AFE_PORT_ID_QUINARY_TDM_RX_4,
				QUINARY_TDM_RX_4, 1, 1},
	[QUINARY_TDM_TX_4] =  { AFE_PORT_ID_QUINARY_TDM_TX_4,
				QUINARY_TDM_TX_4, 0, 1},
	[QUINARY_TDM_RX_5] =  { AFE_PORT_ID_QUINARY_TDM_RX_5,
				QUINARY_TDM_RX_5, 1, 1},
	[QUINARY_TDM_TX_5] =  { AFE_PORT_ID_QUINARY_TDM_TX_5,
				QUINARY_TDM_TX_5, 0, 1},
	[QUINARY_TDM_RX_6] =  { AFE_PORT_ID_QUINARY_TDM_RX_6,
				QUINARY_TDM_RX_6, 1, 1},
	[QUINARY_TDM_TX_6] =  { AFE_PORT_ID_QUINARY_TDM_TX_6,
				QUINARY_TDM_TX_6, 0, 1},
	[QUINARY_TDM_RX_7] =  { AFE_PORT_ID_QUINARY_TDM_RX_7,
				QUINARY_TDM_RX_7, 1, 1},
	[QUINARY_TDM_TX_7] =  { AFE_PORT_ID_QUINARY_TDM_TX_7,
				QUINARY_TDM_TX_7, 0, 1},
	[DISPLAY_PORT_RX] = { AFE_PORT_ID_HDMI_OVER_DP_RX,
				DISPLAY_PORT_RX, 1, 1},
	[WSA_CODEC_DMA_RX_0] = { AFE_PORT_ID_WSA_CODEC_DMA_RX_0,
				WSA_CODEC_DMA_RX_0, 1, 1},
	[WSA_CODEC_DMA_TX_0] = { AFE_PORT_ID_WSA_CODEC_DMA_TX_0,
				WSA_CODEC_DMA_TX_0, 0, 1},
	[WSA_CODEC_DMA_RX_1] = { AFE_PORT_ID_WSA_CODEC_DMA_RX_1,
				WSA_CODEC_DMA_RX_1, 1, 1},
	[WSA_CODEC_DMA_TX_1] = { AFE_PORT_ID_WSA_CODEC_DMA_TX_1,
				WSA_CODEC_DMA_TX_1, 0, 1},
	[WSA_CODEC_DMA_TX_2] = { AFE_PORT_ID_WSA_CODEC_DMA_TX_2,
				WSA_CODEC_DMA_TX_2, 0, 1},
	[VA_CODEC_DMA_TX_0] = { AFE_PORT_ID_VA_CODEC_DMA_TX_0,
				VA_CODEC_DMA_TX_0, 0, 1},
	[VA_CODEC_DMA_TX_1] = { AFE_PORT_ID_VA_CODEC_DMA_TX_1,
				VA_CODEC_DMA_TX_1, 0, 1},
	[VA_CODEC_DMA_TX_2] = { AFE_PORT_ID_VA_CODEC_DMA_TX_2,
				VA_CODEC_DMA_TX_2, 0, 1},
	[RX_CODEC_DMA_RX_0] = { AFE_PORT_ID_RX_CODEC_DMA_RX_0,
				RX_CODEC_DMA_RX_0, 1, 1},
	[TX_CODEC_DMA_TX_0] = { AFE_PORT_ID_TX_CODEC_DMA_TX_0,
				TX_CODEC_DMA_TX_0, 0, 1},
	[RX_CODEC_DMA_RX_1] = { AFE_PORT_ID_RX_CODEC_DMA_RX_1,
				RX_CODEC_DMA_RX_1, 1, 1},
	[TX_CODEC_DMA_TX_1] = { AFE_PORT_ID_TX_CODEC_DMA_TX_1,
				TX_CODEC_DMA_TX_1, 0, 1},
	[RX_CODEC_DMA_RX_2] = { AFE_PORT_ID_RX_CODEC_DMA_RX_2,
				RX_CODEC_DMA_RX_2, 1, 1},
	[TX_CODEC_DMA_TX_2] = { AFE_PORT_ID_TX_CODEC_DMA_TX_2,
				TX_CODEC_DMA_TX_2, 0, 1},
	[RX_CODEC_DMA_RX_3] = { AFE_PORT_ID_RX_CODEC_DMA_RX_3,
				RX_CODEC_DMA_RX_3, 1, 1},
	[TX_CODEC_DMA_TX_3] = { AFE_PORT_ID_TX_CODEC_DMA_TX_3,
				TX_CODEC_DMA_TX_3, 0, 1},
	[RX_CODEC_DMA_RX_4] = { AFE_PORT_ID_RX_CODEC_DMA_RX_4,
				RX_CODEC_DMA_RX_4, 1, 1},
	[TX_CODEC_DMA_TX_4] = { AFE_PORT_ID_TX_CODEC_DMA_TX_4,
				TX_CODEC_DMA_TX_4, 0, 1},
	[RX_CODEC_DMA_RX_5] = { AFE_PORT_ID_RX_CODEC_DMA_RX_5,
				RX_CODEC_DMA_RX_5, 1, 1},
	[TX_CODEC_DMA_TX_5] = { AFE_PORT_ID_TX_CODEC_DMA_TX_5,
				TX_CODEC_DMA_TX_5, 0, 1},
	[RX_CODEC_DMA_RX_6] = { AFE_PORT_ID_RX_CODEC_DMA_RX_6,
				RX_CODEC_DMA_RX_6, 1, 1},
	[RX_CODEC_DMA_RX_7] = { AFE_PORT_ID_RX_CODEC_DMA_RX_7,
				RX_CODEC_DMA_RX_7, 1, 1},
	[USB_RX] = { AFE_PORT_ID_USB_RX, USB_RX, 1, 1},
};

static void q6afe_port_free(struct kref *ref)
{
	struct q6afe_port *port;
	struct q6afe *afe;
	unsigned long flags;

	port = container_of(ref, struct q6afe_port, refcount);
	afe = port->afe;
	spin_lock_irqsave(&afe->port_list_lock, flags);
	list_del(&port->node);
	spin_unlock_irqrestore(&afe->port_list_lock, flags);
	kfree(port->scfg);
	kfree(port);
}

static struct q6afe_port *q6afe_find_port(struct q6afe *afe, int token)
{
	struct q6afe_port *p;
	struct q6afe_port *ret = NULL;

	guard(spinlock_irqsave)(&afe->port_list_lock);
	list_for_each_entry(p, &afe->port_list, node)
		if (p->token == token) {
			ret = p;
			kref_get(&p->refcount);
			break;
		}

	return ret;
}

static int q6afe_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct q6afe *afe = dev_get_drvdata(&adev->dev);
	struct aprv2_ibasic_rsp_result_t *res;
	struct apr_hdr *hdr = &data->hdr;
	struct q6afe_port *port;

	if (!data->payload_size)
		return 0;

	res = data->payload;
	switch (hdr->opcode) {
	case APR_BASIC_RSP_RESULT: {
		if (res->status) {
			dev_err(afe->dev, "cmd = 0x%x returned error = 0x%x\n",
				res->opcode, res->status);
		}
		switch (res->opcode) {
		case AFE_PORT_CMD_SET_PARAM_V2:
		case AFE_PORT_CMD_DEVICE_STOP:
		case AFE_PORT_CMD_DEVICE_START:
		case AFE_SVC_CMD_SET_PARAM:
		case 0x000100EA: /* AFE_SERVICE_CMD_SHARED_MEM_MAP_REGIONS */
			port = q6afe_find_port(afe, hdr->token);
			if (port) {
				port->result = *res;
				wake_up(&port->wait);
				kref_put(&port->refcount, q6afe_port_free);
			} else if (hdr->token == AFE_CLK_TOKEN) {
				afe->result = *res;
				wake_up(&afe->wait);
			}
			break;
		default:
			dev_info(afe->dev, "APR RSP unknown cmd=0x%x status=0x%x token=%d\n",
				 res->opcode, res->status, hdr->token);
			break;
		}
	}
		break;
	case AFE_CMD_RSP_REMOTE_LPASS_CORE_HW_VOTE_REQUEST:
		afe->result.opcode = hdr->opcode;
		afe->result.status = res->status;
		wake_up(&afe->wait);
		break;
	case 0x000100EB: /* AFE_SERVICE_CMDRSP_SHARED_MEM_MAP_REGIONS */
		afe->mmap_handle = *((u32 *)data->payload);
		/* Set opcode to the COMMAND we sent, not the response opcode,
		 * so the waiter's condition (result->opcode == rsp_opcode) matches */
		afe->result.opcode = 0x000100EA;
		afe->result.status = 0;
		dev_info(afe->dev, "AFE mmap_handle = 0x%x\n", afe->mmap_handle);
		wake_up(&afe->wait);
		break;
	default:
		dev_info(afe->dev, "APR unhandled opcode=0x%x token=%d\n",
			 hdr->opcode, hdr->token);
		break;
	}

	return 0;
}

/**
 * q6afe_get_port_id() - Get port id from a given port index
 *
 * @index: port index
 *
 * Return: Will be an negative on error or valid port_id on success
 */
int q6afe_get_port_id(int index)
{
	if (index < 0 || index >= AFE_PORT_MAX)
		return -EINVAL;

	return port_maps[index].port_id;
}
EXPORT_SYMBOL_GPL(q6afe_get_port_id);

static int afe_apr_send_pkt(struct q6afe *afe, struct apr_pkt *pkt,
			    struct q6afe_port *port, uint32_t rsp_opcode)
{
	wait_queue_head_t *wait;
	struct aprv2_ibasic_rsp_result_t *result;
	int ret;

	mutex_lock(&afe->lock);
	if (port) {
		wait = &port->wait;
		result = &port->result;
	} else {
		result = &afe->result;
		wait = &afe->wait;
	}

	result->opcode = 0;
	result->status = 0;

	ret = apr_send_pkt(afe->apr, pkt);
	if (ret < 0) {
		dev_err(afe->dev, "packet not transmitted (%d)\n", ret);
		ret = -EINVAL;
		goto err;
	}

	ret = wait_event_timeout(*wait, (result->opcode == rsp_opcode),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		ret = -ETIMEDOUT;
	} else if (result->status > 0) {
		dev_err(afe->dev, "DSP returned error[%x]\n",
			result->status);
		ret = -EINVAL;
	} else {
		ret = 0;
	}

err:
	mutex_unlock(&afe->lock);

	return ret;
}

static int q6afe_set_param(struct q6afe *afe, struct q6afe_port *port,
			   void *data, int param_id, int module_id, int psize,
			   int token)
{
	struct afe_svc_cmd_set_param *param;
	struct afe_port_param_data_v2 *pdata;
	struct apr_pkt *pkt;
	int ret, pkt_size = APR_HDR_SIZE + sizeof(*param) + sizeof(*pdata) + psize;
	void *pl;
	void *p __free(kfree) = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	pkt = p;
	param = p + APR_HDR_SIZE;
	pdata = p + APR_HDR_SIZE + sizeof(*param);
	pl = p + APR_HDR_SIZE + sizeof(*param) + sizeof(*pdata);
	if (data && psize)
		memcpy(pl, data, psize);

	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = token;
	pkt->hdr.opcode = AFE_SVC_CMD_SET_PARAM;

	param->payload_size = sizeof(*pdata) + psize;
	param->payload_address_lsw = 0x00;
	param->payload_address_msw = 0x00;
	param->mem_map_handle = 0x00;
	pdata->module_id = module_id;
	pdata->param_id = param_id;
	pdata->param_size = psize;

	ret = afe_apr_send_pkt(afe, pkt, port, AFE_SVC_CMD_SET_PARAM);
	if (ret)
		dev_err(afe->dev, "AFE set params failed %d\n", ret);

	return ret;
}

static int q6afe_port_set_param(struct q6afe_port *port, void *data,
				int param_id, int module_id, int psize)
{
	return q6afe_set_param(port->afe, port, data, param_id, module_id,
			       psize, port->token);
}

static int q6afe_port_set_param_v2(struct q6afe_port *port, void *data,
				   int param_id, int module_id, int psize)
{
	struct afe_port_cmd_set_param_v2 *param;
	struct afe_port_param_data_v2 *pdata;
	struct q6afe *afe = port->afe;
	struct apr_pkt *pkt;
	u16 port_id = port->id;
	int ret, pkt_size = APR_HDR_SIZE + sizeof(*param) + sizeof(*pdata) + psize;
	void *pl;
	void *p __free(kfree) = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	pkt = p;
	param = p + APR_HDR_SIZE;
	pdata = p + APR_HDR_SIZE + sizeof(*param);
	pl = p + APR_HDR_SIZE + sizeof(*param) + sizeof(*pdata);
	memcpy(pl, data, psize);

	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = port->token;
	pkt->hdr.opcode = AFE_PORT_CMD_SET_PARAM_V2;

	param->port_id = port_id;
	param->payload_size = sizeof(*pdata) + psize;
	param->payload_address_lsw = 0x00;
	param->payload_address_msw = 0x00;
	param->mem_map_handle = 0x00;
	pdata->module_id = module_id;
	pdata->param_id = param_id;
	pdata->param_size = psize;

	ret = afe_apr_send_pkt(afe, pkt, port, AFE_PORT_CMD_SET_PARAM_V2);
	if (ret)
		dev_err(afe->dev, "AFE enable for port 0x%x failed %d\n",
		       port_id, ret);

	return ret;
}

static int q6afe_port_set_lpass_clock(struct q6afe_port *port,
				 struct afe_clk_cfg *cfg)
{
	return q6afe_port_set_param_v2(port, cfg,
				       AFE_PARAM_ID_LPAIF_CLK_CONFIG,
				       AFE_MODULE_AUDIO_DEV_INTERFACE,
				       sizeof(*cfg));
}

static int q6afe_set_lpass_clock_v2(struct q6afe_port *port,
				 struct afe_clk_set *cfg)
{
	return q6afe_port_set_param(port, cfg, AFE_PARAM_ID_CLOCK_SET,
				    AFE_MODULE_CLOCK_SET, sizeof(*cfg));
}

static int q6afe_set_digital_codec_core_clock(struct q6afe_port *port,
					      struct afe_digital_clk_cfg *cfg)
{
	return q6afe_port_set_param_v2(port, cfg,
				       AFE_PARAM_ID_INT_DIGITAL_CDC_CLK_CONFIG,
				       AFE_MODULE_AUDIO_DEV_INTERFACE,
				       sizeof(*cfg));
}

int q6afe_set_lpass_clock(struct device *dev, int clk_id, int attri,
			  int clk_root, unsigned int freq)
{
	struct q6afe *afe = dev_get_drvdata(dev->parent);
	struct afe_clk_set cset = {0,};

	cset.clk_set_minor_version = AFE_API_VERSION_CLOCK_SET;
	cset.clk_id = clk_id;
	cset.clk_freq_in_hz = freq;
	cset.clk_attri = attri;
	cset.clk_root = clk_root;
	cset.enable = !!freq;

	return q6afe_set_param(afe, NULL, &cset, AFE_PARAM_ID_CLOCK_SET,
			       AFE_MODULE_CLOCK_SET, sizeof(cset),
			       AFE_CLK_TOKEN);
}
EXPORT_SYMBOL_GPL(q6afe_set_lpass_clock);

int q6afe_port_set_sysclk(struct q6afe_port *port, int clk_id,
			  int clk_src, int clk_root,
			  unsigned int freq, int dir)
{
	struct afe_clk_cfg ccfg = {0,};
	struct afe_clk_set cset = {0,};
	struct afe_digital_clk_cfg dcfg = {0,};
	int ret;

	switch (clk_id) {
	case LPAIF_DIG_CLK:
		dcfg.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
		dcfg.clk_val = freq;
		dcfg.clk_root = clk_root;
		ret = q6afe_set_digital_codec_core_clock(port, &dcfg);
		break;
	case LPAIF_BIT_CLK:
		ccfg.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
		ccfg.clk_val1 = freq;
		ccfg.clk_src = clk_src;
		ccfg.clk_root = clk_root;
		ccfg.clk_set_mode = Q6AFE_LPASS_MODE_CLK1_VALID;
		ret = q6afe_port_set_lpass_clock(port, &ccfg);
		break;

	case LPAIF_OSR_CLK:
		ccfg.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
		ccfg.clk_val2 = freq;
		ccfg.clk_src = clk_src;
		ccfg.clk_root = clk_root;
		ccfg.clk_set_mode = Q6AFE_LPASS_MODE_CLK2_VALID;
		ret = q6afe_port_set_lpass_clock(port, &ccfg);
		break;
	case Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT ... Q6AFE_LPASS_CLK_ID_QUI_MI2S_OSR:
	case Q6AFE_LPASS_CLK_ID_MCLK_1 ... Q6AFE_LPASS_CLK_ID_INT_MCLK_1:
	case Q6AFE_LPASS_CLK_ID_PRI_TDM_IBIT ... Q6AFE_LPASS_CLK_ID_QUIN_TDM_EBIT:
	case Q6AFE_LPASS_CLK_ID_WSA_CORE_MCLK ... Q6AFE_LPASS_CLK_ID_VA_CORE_2X_MCLK:
		cset.clk_set_minor_version = AFE_API_VERSION_CLOCK_SET;
		cset.clk_id = clk_id;
		cset.clk_freq_in_hz = freq;
		cset.clk_attri = clk_src;
		cset.clk_root = clk_root;
		cset.enable = !!freq;
		ret = q6afe_set_lpass_clock_v2(port, &cset);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(q6afe_port_set_sysclk);

/**
 * q6afe_port_stop() - Stop a afe port
 *
 * @port: Instance of port to stop
 *
 * Return: Will be an negative on packet size on success.
 */
int q6afe_port_stop(struct q6afe_port *port)
{
	struct afe_port_cmd_device_stop *stop;
	struct q6afe *afe = port->afe;
	struct apr_pkt *pkt;
	int port_id = port->id;
	int ret = 0;
	int index, pkt_size;
	void *p __free(kfree) = NULL;

	index = port->token;
	if (index < 0 || index >= AFE_PORT_MAX) {
		dev_err(afe->dev, "AFE port index[%d] invalid!\n", index);
		return -EINVAL;
	}

	pkt_size = APR_HDR_SIZE + sizeof(*stop);
	p = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	pkt = p;
	stop = p + APR_HDR_SIZE;

	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = index;
	pkt->hdr.opcode = AFE_PORT_CMD_DEVICE_STOP;
	stop->port_id = port_id;
	stop->reserved = 0;

	ret = afe_apr_send_pkt(afe, pkt, port, AFE_PORT_CMD_DEVICE_STOP);
	if (ret)
		dev_err(afe->dev, "AFE close failed %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(q6afe_port_stop);

/**
 * q6afe_slim_port_prepare() - Prepare slim afe port.
 *
 * @port: Instance of afe port
 * @cfg: SLIM configuration for the afe port
 *
 */
void q6afe_slim_port_prepare(struct q6afe_port *port,
			     struct q6afe_slim_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;

	pcfg->slim_cfg.sb_cfg_minor_version = AFE_API_VERSION_SLIMBUS_CONFIG;
	pcfg->slim_cfg.sample_rate = cfg->sample_rate;
	pcfg->slim_cfg.bit_width = cfg->bit_width;
	pcfg->slim_cfg.num_channels = cfg->num_channels;
	pcfg->slim_cfg.data_format = cfg->data_format;
	pcfg->slim_cfg.shared_ch_mapping[0] = cfg->ch_mapping[0];
	pcfg->slim_cfg.shared_ch_mapping[1] = cfg->ch_mapping[1];
	pcfg->slim_cfg.shared_ch_mapping[2] = cfg->ch_mapping[2];
	pcfg->slim_cfg.shared_ch_mapping[3] = cfg->ch_mapping[3];

}
EXPORT_SYMBOL_GPL(q6afe_slim_port_prepare);

/**
 * q6afe_tdm_port_prepare() - Prepare tdm afe port.
 *
 * @port: Instance of afe port
 * @cfg: TDM configuration for the afe port
 *
 */
void q6afe_tdm_port_prepare(struct q6afe_port *port,
			     struct q6afe_tdm_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;

	pcfg->tdm_cfg.tdm_cfg_minor_version = AFE_API_VERSION_TDM_CONFIG;
	pcfg->tdm_cfg.num_channels = cfg->num_channels;
	pcfg->tdm_cfg.sample_rate = cfg->sample_rate;
	pcfg->tdm_cfg.bit_width = cfg->bit_width;
	pcfg->tdm_cfg.data_format = cfg->data_format;
	pcfg->tdm_cfg.sync_mode = cfg->sync_mode;
	pcfg->tdm_cfg.sync_src = cfg->sync_src;
	pcfg->tdm_cfg.nslots_per_frame = cfg->nslots_per_frame;

	pcfg->tdm_cfg.slot_width = cfg->slot_width;
	pcfg->tdm_cfg.slot_mask = cfg->slot_mask;
	port->scfg = kzalloc(sizeof(*port->scfg), GFP_KERNEL);
	if (!port->scfg)
		return;

	port->scfg->minor_version = AFE_API_VERSION_SLOT_MAPPING_CONFIG;
	port->scfg->num_channels = cfg->num_channels;
	port->scfg->bitwidth = cfg->bit_width;
	port->scfg->data_align_type = cfg->data_align_type;
	memcpy(port->scfg->ch_mapping, cfg->ch_mapping,
			sizeof(u16) * AFE_PORT_MAX_AUDIO_CHAN_CNT);
}
EXPORT_SYMBOL_GPL(q6afe_tdm_port_prepare);

/**
 * afe_port_send_usb_dev_param() - Send USB dev token
 *
 * @port: Instance of afe port
 * @cardidx: USB SND card index to reference
 * @pcmidx: USB SND PCM device index to reference
 *
 * The USB dev token carries information about which USB SND card instance and
 * PCM device to execute the offload on.  This information is carried through
 * to the stream enable QMI request, which is handled by the offload class
 * driver.  The information is parsed to determine which USB device to query
 * the required resources for.
 */
int afe_port_send_usb_dev_param(struct q6afe_port *port, int cardidx, int pcmidx)
{
	struct afe_param_id_usb_audio_dev_params usb_dev;
	int ret;

	memset(&usb_dev, 0, sizeof(usb_dev));

	usb_dev.cfg_minor_version = AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG;
	usb_dev.dev_token = (cardidx << 16) | (pcmidx << 8);
	ret = q6afe_port_set_param_v2(port, &usb_dev,
				      AFE_PARAM_ID_USB_AUDIO_DEV_PARAMS,
				      AFE_MODULE_AUDIO_DEV_INTERFACE,
				      sizeof(usb_dev));
	if (ret)
		dev_err(port->afe->dev, "%s: AFE device param cmd failed %d\n",
			__func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(afe_port_send_usb_dev_param);

static int afe_port_send_usb_params(struct q6afe_port *port, struct q6afe_usb_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;
	struct afe_param_id_usb_audio_dev_lpcm_fmt lpcm_fmt;
	struct afe_param_id_usb_audio_svc_interval svc_int;
	int ret;

	if (!pcfg) {
		dev_err(port->afe->dev, "%s: Error, no configuration data\n", __func__);
		return -EINVAL;
	}

	memset(&lpcm_fmt, 0, sizeof(lpcm_fmt));
	memset(&svc_int, 0, sizeof(svc_int));

	lpcm_fmt.cfg_minor_version = AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG;
	lpcm_fmt.endian = pcfg->usb_cfg.endian;
	ret = q6afe_port_set_param_v2(port, &lpcm_fmt,
				      AFE_PARAM_ID_USB_AUDIO_DEV_LPCM_FMT,
				      AFE_MODULE_AUDIO_DEV_INTERFACE, sizeof(lpcm_fmt));
	if (ret) {
		dev_err(port->afe->dev, "%s: AFE device param cmd LPCM_FMT failed %d\n",
			__func__, ret);
		return ret;
	}

	svc_int.cfg_minor_version = AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG;
	svc_int.svc_interval = pcfg->usb_cfg.service_interval;
	ret = q6afe_port_set_param_v2(port, &svc_int,
				      AFE_PARAM_ID_USB_AUDIO_SVC_INTERVAL,
				      AFE_MODULE_AUDIO_DEV_INTERFACE, sizeof(svc_int));
	if (ret)
		dev_err(port->afe->dev, "%s: AFE device param cmd svc_interval failed %d\n",
			__func__, ret);

	return ret;
}

/**
 * q6afe_usb_port_prepare() - Prepare usb afe port.
 *
 * @port: Instance of afe port
 * @cfg: USB configuration for the afe port
 *
 */
void q6afe_usb_port_prepare(struct q6afe_port *port,
			    struct q6afe_usb_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;

	pcfg->usb_cfg.cfg_minor_version = AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG;
	pcfg->usb_cfg.sample_rate = cfg->sample_rate;
	pcfg->usb_cfg.num_channels = cfg->num_channels;
	pcfg->usb_cfg.bit_width = cfg->bit_width;

	afe_port_send_usb_params(port, cfg);
}
EXPORT_SYMBOL_GPL(q6afe_usb_port_prepare);

/**
 * q6afe_hdmi_port_prepare() - Prepare hdmi afe port.
 *
 * @port: Instance of afe port
 * @cfg: HDMI configuration for the afe port
 *
 */
void q6afe_hdmi_port_prepare(struct q6afe_port *port,
			     struct q6afe_hdmi_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;

	pcfg->hdmi_multi_ch.hdmi_cfg_minor_version =
					AFE_API_VERSION_HDMI_CONFIG;
	pcfg->hdmi_multi_ch.datatype = cfg->datatype;
	pcfg->hdmi_multi_ch.channel_allocation = cfg->channel_allocation;
	pcfg->hdmi_multi_ch.sample_rate = cfg->sample_rate;
	pcfg->hdmi_multi_ch.bit_width = cfg->bit_width;
}
EXPORT_SYMBOL_GPL(q6afe_hdmi_port_prepare);

/**
 * q6afe_i2s_port_prepare() - Prepare i2s afe port.
 *
 * @port: Instance of afe port
 * @cfg: I2S configuration for the afe port
 * Return: Will be an negative on error and zero on success.
 */
int q6afe_i2s_port_prepare(struct q6afe_port *port, struct q6afe_i2s_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;
	struct device *dev = port->afe->dev;
	int num_sd_lines;

	pcfg->i2s_cfg.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
	pcfg->i2s_cfg.sample_rate = cfg->sample_rate;
	pcfg->i2s_cfg.bit_width = cfg->bit_width;
	pcfg->i2s_cfg.data_format = AFE_LINEAR_PCM_DATA;

	switch (cfg->fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		pcfg->i2s_cfg.ws_src = AFE_PORT_CONFIG_I2S_WS_SRC_INTERNAL;
		break;
	case SND_SOC_DAIFMT_BC_FC:
		/* CPU is slave */
		pcfg->i2s_cfg.ws_src = AFE_PORT_CONFIG_I2S_WS_SRC_EXTERNAL;
		break;
	default:
		break;
	}

	num_sd_lines = hweight_long(cfg->sd_line_mask);

	switch (num_sd_lines) {
	case 0:
		dev_err(dev, "no line is assigned\n");
		return -EINVAL;
	case 1:
		switch (cfg->sd_line_mask) {
		case AFE_PORT_I2S_SD0_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_SD0;
			break;
		case AFE_PORT_I2S_SD1_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_SD1;
			break;
		case AFE_PORT_I2S_SD2_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_SD2;
			break;
		case AFE_PORT_I2S_SD3_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_SD3;
			break;
		default:
			dev_err(dev, "Invalid SD lines\n");
			return -EINVAL;
		}
		break;
	case 2:
		switch (cfg->sd_line_mask) {
		case AFE_PORT_I2S_SD0_1_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_QUAD01;
			break;
		case AFE_PORT_I2S_SD2_3_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_QUAD23;
			break;
		default:
			dev_err(dev, "Invalid SD lines\n");
			return -EINVAL;
		}
		break;
	case 3:
		switch (cfg->sd_line_mask) {
		case AFE_PORT_I2S_SD0_1_2_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_6CHS;
			break;
		default:
			dev_err(dev, "Invalid SD lines\n");
			return -EINVAL;
		}
		break;
	case 4:
		switch (cfg->sd_line_mask) {
		case AFE_PORT_I2S_SD0_1_2_3_MASK:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_8CHS;

			break;
		default:
			dev_err(dev, "Invalid SD lines\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(dev, "Invalid SD lines\n");
		return -EINVAL;
	}

	switch (cfg->num_channels) {
	case 1:
	case 2:
		switch (pcfg->i2s_cfg.channel_mode) {
		case AFE_PORT_I2S_QUAD01:
		case AFE_PORT_I2S_6CHS:
		case AFE_PORT_I2S_8CHS:
			pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_SD0;
			break;
		case AFE_PORT_I2S_QUAD23:
				pcfg->i2s_cfg.channel_mode = AFE_PORT_I2S_SD2;
			break;
		}

		if (cfg->num_channels == 2)
			pcfg->i2s_cfg.mono_stereo = AFE_PORT_I2S_STEREO;
		else
			pcfg->i2s_cfg.mono_stereo = AFE_PORT_I2S_MONO;

		break;
	case 3:
	case 4:
		if (pcfg->i2s_cfg.channel_mode < AFE_PORT_I2S_QUAD01) {
			dev_err(dev, "Invalid Channel mode\n");
			return -EINVAL;
		}
		break;
	case 5:
	case 6:
		if (pcfg->i2s_cfg.channel_mode < AFE_PORT_I2S_6CHS) {
			dev_err(dev, "Invalid Channel mode\n");
			return -EINVAL;
		}
		break;
	case 7:
	case 8:
		if (pcfg->i2s_cfg.channel_mode < AFE_PORT_I2S_8CHS) {
			dev_err(dev, "Invalid Channel mode\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(q6afe_i2s_port_prepare);

/**
 * q6afe_cdc_dma_port_prepare() - Prepare dma afe port.
 *
 * @port: Instance of afe port
 * @cfg: DMA configuration for the afe port
 *
 */
void q6afe_cdc_dma_port_prepare(struct q6afe_port *port,
				struct q6afe_cdc_dma_cfg *cfg)
{
	union afe_port_config *pcfg = &port->port_cfg;
	struct afe_param_id_cdc_dma_cfg *dma_cfg = &pcfg->dma_cfg;

	dma_cfg->cdc_dma_cfg_minor_version = AFE_API_VERSION_CODEC_DMA_CONFIG;
	dma_cfg->sample_rate = cfg->sample_rate;
	dma_cfg->bit_width = cfg->bit_width;
	dma_cfg->data_format = cfg->data_format;
	dma_cfg->num_channels = cfg->num_channels;
	if (!cfg->active_channels_mask)
		dma_cfg->active_channels_mask = (1 << cfg->num_channels) - 1;
}
EXPORT_SYMBOL_GPL(q6afe_cdc_dma_port_prepare);

/*
 * Minimal ACDB AFE calibration loader.
 * Parses ACDB file sections (AFECLUT0, AFECCDFT, AFECCDOT, DATAPOOL)
 * and sends the calibration blob inline via AFE_PORT_CMD_SET_PARAM_V2.
 */
static const u8 *acdb_find_tag(const u8 *data, size_t len, const char *tag)
{
	size_t tlen = strlen(tag);
	const u8 *p;

	for (p = data; p < data + len - tlen - 4; p++)
		if (memcmp(p, tag, tlen) == 0)
			return p;
	return NULL;
}

static int q6afe_map_memory(struct q6afe *afe, dma_addr_t phys, u32 size)
{
	struct {
		struct apr_hdr hdr;
		u16 mem_pool_id;
		u16 num_regions;
		u32 property_flag;
		u32 shm_addr_lsw;
		u32 shm_addr_msw;
		u32 mem_size_bytes;
	} __packed cmd = {};
	int ret;

	cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.src_port = 0;
	cmd.hdr.dest_port = 0;
	cmd.hdr.token = AFE_CLK_TOKEN;
	cmd.hdr.opcode = 0x000100EA; /* AFE_SERVICE_CMD_SHARED_MEM_MAP_REGIONS */
	cmd.mem_pool_id = 3; /* ADSP_MEMORY_MAP_SHMEM8_4K_POOL */
	cmd.num_regions = 1;
	cmd.property_flag = 0;
	cmd.shm_addr_lsw = lower_32_bits(phys);
	cmd.shm_addr_msw = upper_32_bits(phys);
	cmd.mem_size_bytes = PAGE_ALIGN(size);

	afe->mmap_handle = 0;
	ret = afe_apr_send_pkt(afe, (struct apr_pkt *)&cmd, NULL,
			       0x000100EA);
	if (ret) {
		dev_err(afe->dev, "AFE SHARED_MEM_MAP failed: %d\n", ret);
		return ret;
	}

	if (!afe->mmap_handle) {
		dev_err(afe->dev, "AFE SHARED_MEM_MAP: no handle returned\n");
		return -EINVAL;
	}

	dev_info(afe->dev, "AFE memory mapped: phys=0x%pad size=%u handle=0x%x\n",
		 &phys, size, afe->mmap_handle);
	return 0;
}

static int q6afe_send_cal_via_shmem(struct q6afe *afe, struct q6afe_port *port,
				    void *cal_data, u32 cal_size)
{
	struct afe_port_cmd_set_param_v2 *param;
	struct apr_pkt *pkt;
	dma_addr_t phys;
	void *virt;
	int ret, pkt_size;

	/* Allocate physically contiguous memory for cal data.
	 * The ADSP reads from physical address, so we need contiguous pages.
	 */
	{
		struct page *pg;
		int order = get_order(PAGE_ALIGN(cal_size));

		pg = alloc_pages(GFP_KERNEL | __GFP_ZERO, order);
		if (!pg)
			return -ENOMEM;
		virt = page_address(pg);
		phys = page_to_phys(pg);
	}
	if (!virt)
		return -ENOMEM;

	memcpy(virt, cal_data, cal_size);

	/* Map to ADSP */
	ret = q6afe_map_memory(afe, phys, cal_size);
	if (ret)
		goto free_mem;

	/* Send SET_PARAM_V2 referencing the shared memory */
	pkt_size = APR_HDR_SIZE + sizeof(*param);
	pkt = kzalloc(pkt_size, GFP_KERNEL);
	if (!pkt) {
		ret = -ENOMEM;
		goto free_mem;
	}

	param = (void *)pkt + APR_HDR_SIZE;
	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					    APR_HDR_LEN(APR_HDR_SIZE),
					    APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = port->token;
	pkt->hdr.opcode = AFE_PORT_CMD_SET_PARAM_V2;
	param->port_id = port->id;
	param->payload_size = cal_size;
	param->payload_address_lsw = lower_32_bits(phys);
	param->payload_address_msw = upper_32_bits(phys);
	param->mem_map_handle = afe->mmap_handle;

	dev_info(afe->dev,
		 "AFE cal via shmem: port=0x%x size=%u phys=0x%pad handle=0x%x\n",
		 port->id, cal_size, &phys, afe->mmap_handle);

	ret = afe_apr_send_pkt(afe, pkt, port, AFE_PORT_CMD_SET_PARAM_V2);
	if (ret)
		dev_err(afe->dev, "AFE cal shmem send failed: %d\n", ret);
	else
		dev_info(afe->dev, "AFE cal via shmem: SUCCESS\n");

	kfree(pkt);
free_mem:
	free_pages((unsigned long)virt, get_order(PAGE_ALIGN(cal_size)));
	return ret;
}

static int q6afe_send_acdb_afe_cal(struct q6afe *afe, struct q6afe_port *port,
				   u32 acdb_dev_id, u32 sample_rate)
{
	const struct firmware *fw;
	const char *fname = "acdbdata/MTP_WCD9335_Handset_cal.acdb";
	const u8 *lut_p, *cdft_p, *cdot_p, *pool_p;
	u32 lut_sz, cdft_sz, cdot_sz, pool_sz;
	u32 num_lut, i;
	int ret = -ENOENT;

	ret = request_firmware(&fw, fname, afe->dev);
	if (ret) {
		dev_info(afe->dev, "ACDB %s not found (%d), skipping cal\n",
			 fname, ret);
		return 0;
	}

	dev_info(afe->dev, "ACDB: loaded %s (%zu bytes)\n", fname, fw->size);

	lut_p = acdb_find_tag(fw->data, fw->size, "AFECLUT0");
	cdft_p = acdb_find_tag(fw->data, fw->size, "AFECCDFT");
	cdot_p = acdb_find_tag(fw->data, fw->size, "AFECCDOT");
	pool_p = acdb_find_tag(fw->data, fw->size, "DATAPOOL");
	if (!lut_p || !cdft_p || !cdot_p || !pool_p) {
		dev_warn(afe->dev, "ACDB: missing sections\n");
		ret = 0;
		goto out;
	}

	lut_sz = le32_to_cpup((__le32 *)(lut_p + 8)); lut_p += 12;
	cdft_sz = le32_to_cpup((__le32 *)(cdft_p + 8)); cdft_p += 12;
	cdot_sz = le32_to_cpup((__le32 *)(cdot_p + 8)); cdot_p += 12;
	pool_sz = le32_to_cpup((__le32 *)(pool_p + 8)); pool_p += 12;

	num_lut = le32_to_cpup((__le32 *)lut_p);
	dev_info(afe->dev, "ACDB: AFECLUT0 has %u entries\n", num_lut);

	for (i = 0; i < num_lut; i++) {
		const u8 *entry = lut_p + 4 + i * 16;
		u32 did = le32_to_cpup((__le32 *)(entry));
		u32 rate = le32_to_cpup((__le32 *)(entry + 4));
		u32 cdft_ofs = le32_to_cpup((__le32 *)(entry + 8));
		u32 cdot_ofs = le32_to_cpup((__le32 *)(entry + 12));
		u32 npairs, j, blob_sz = 0;
		u8 *blob, *bp;

		if (did != acdb_dev_id || rate != sample_rate)
			continue;

		if (cdft_ofs + 4 > cdft_sz) continue;
		npairs = le32_to_cpup((__le32 *)(cdft_p + cdft_ofs));
		if (cdft_ofs + 4 + npairs * 8 > cdft_sz) continue;
		if (cdot_ofs + 4 > cdot_sz) continue;

		for (j = 0; j < npairs; j++) {
			u32 pool_ofs = le32_to_cpup((__le32 *)(cdot_p + cdot_ofs + 4 + j * 4));
			u32 dsz;
			if (pool_ofs + 4 > pool_sz) continue;
			dsz = le32_to_cpup((__le32 *)(pool_p + pool_ofs));
			blob_sz += 12 + ALIGN(dsz, 4);
		}
		if (blob_sz == 0) continue;

		blob = kzalloc(blob_sz, GFP_KERNEL);
		if (!blob) { ret = -ENOMEM; goto out; }

		bp = blob;
		for (j = 0; j < npairs; j++) {
			u32 mid = le32_to_cpup((__le32 *)(cdft_p + cdft_ofs + 4 + j * 8));
			u32 pid = le32_to_cpup((__le32 *)(cdft_p + cdft_ofs + 4 + j * 8 + 4));
			u32 pool_ofs = le32_to_cpup((__le32 *)(cdot_p + cdot_ofs + 4 + j * 4));
			u32 dsz;
			if (pool_ofs + 4 > pool_sz) { kfree(blob); goto out; }
			dsz = le32_to_cpup((__le32 *)(pool_p + pool_ofs));
			*((u32 *)bp) = cpu_to_le32(mid); bp += 4;
			*((u32 *)bp) = cpu_to_le32(pid); bp += 4;
			*((u16 *)bp) = cpu_to_le16(dsz); bp += 2;
			*((u16 *)bp) = 0; bp += 2;
			if (dsz > 0 && pool_ofs + 4 + dsz <= pool_sz)
				memcpy(bp, pool_p + pool_ofs + 4, dsz);
			bp += ALIGN(dsz, 4);
		}

		/* Log module IDs in this cal blob */
		{
			u8 *p = blob;

			for (j = 0; j < npairs && p < blob + blob_sz; j++) {
				u32 m = le32_to_cpup((__le32 *)p);
				u32 pid2 = le32_to_cpup((__le32 *)(p + 4));
				u16 sz = le16_to_cpup((__le16 *)(p + 8));

				dev_info(afe->dev,
					 "ACDB cal[%d]: mid=0x%x pid=0x%x sz=%u\n",
					 j, m, pid2, sz);
				p += 12 + ALIGN(sz, 4);
			}
		}
		dev_info(afe->dev,
			 "ACDB: sending AFE cal dev=%u rate=%u (%u bytes, %u params)\n",
			 did, rate, blob_sz, npairs);

		/* Send cal via shared memory (matching downstream) */
		ret = q6afe_send_cal_via_shmem(afe, port, blob, blob_sz);
		if (ret)
			dev_warn(afe->dev, "ACDB shmem cal failed: %d\n", ret);
		kfree(blob);
		goto out;
	}

	dev_info(afe->dev, "ACDB: no AFE cal for dev=%u rate=%u\n",
		 acdb_dev_id, sample_rate);
	ret = 0;
out:
	release_firmware(fw);
	return ret;
}

/*
 * Send CDC configuration to ADSP, matching downstream msm8952-slimbus.c
 * ordering: REG_CFG → PAGE_CFG → SLAVE_CFG → (INIT triggered by SLAVE_CFG)
 * → PAGE_CFG again.
 *
 * Downstream sends REG_CFG BEFORE SLAVE_CFG. This matters because
 * SLAVE_CFG triggers REG_CFG_INIT internally.
 */
static int q6afe_send_cdc_slimbus_slave_cfg(struct q6afe *afe)
{
	struct afe_param_cdc_slimbus_slave_cfg slave_cfg = {};
	int ret, i;

	/* --- Step 1: CDC_REG_CFG entries (PGD port register metadata) --- */
	{
		struct afe_param_cdc_reg_cfg_t {
			u32 minor_version;
			u32 reg_logical_addr;
			u32 reg_field_type;
			u32 reg_field_bit_mask;
			u16 reg_bit_width;
			u16 reg_offset_scale;
		} __packed;

		/*
		 * Full downstream tasha audio_reg_cfg table (22 entries).
		 * Register addresses = TASHA_REGISTER_START_OFFSET (0x800) + reg.
		 * Field types from wcd9xxx-common-v2.h enum (starts at 0).
		 */
		static const struct afe_param_cdc_reg_cfg_t cdc_regs[] = {
			/* MAD (Microphone Activity Detection) */
			{ 1, 0x800+0x0281, 4, 0x01, 8, 0 },  /* HW_MAD_AUDIO_ENABLE */
			{ 1, 0x800+0x0285, 7, 0x0F, 8, 0 },  /* HW_MAD_AUDIO_SLEEP_TIME */
			{ 1, 0x800+0x0286, 10, 0x01, 8, 0 }, /* HW_MAD_TX_AUDIO_SWITCH_OFF */
			/* Interrupt routing */
			{ 1, 0x800+0x0081, 13, 0x02, 8, 0 }, /* MAD_AUDIO_INT_DEST_SELECT */
			{ 1, 0x800+0x00a4, 18, 0x01, 8, 0 }, /* MAD_AUDIO_INT_MASK */
			{ 1, 0x800+0x00ac, 23, 0x01, 8, 0 }, /* MAD_AUDIO_INT_STATUS */
			{ 1, 0x800+0x00b4, 28, 0x01, 8, 0 }, /* MAD_AUDIO_INT_CLEAR */
			/* VBAT */
			{ 1, 0x800+0x0081, 17, 0x02, 8, 0 }, /* VBAT_INT_DEST_SELECT */
			{ 1, 0x800+0x00a4, 22, 0x08, 8, 0 }, /* VBAT_INT_MASK */
			{ 1, 0x800+0x00ac, 27, 0x08, 8, 0 }, /* VBAT_INT_STATUS */
			{ 1, 0x800+0x00b4, 32, 0x08, 8, 0 }, /* VBAT_INT_CLEAR */
			/* VBAT release */
			{ 1, 0x800+0x0081, 216, 0x02, 8, 0 }, /* VBAT_RELEASE_INT_DEST */
			{ 1, 0x800+0x00a4, 217, 0x10, 8, 0 }, /* VBAT_RELEASE_INT_MASK */
			{ 1, 0x800+0x00ac, 218, 0x10, 8, 0 }, /* VBAT_RELEASE_INT_STATUS */
			{ 1, 0x800+0x00b4, 219, 0x10, 8, 0 }, /* VBAT_RELEASE_INT_CLEAR */
			/* SLIMbus PGD ports */
			{ 1, 0x850, 196, 0x1E, 8, 1 }, /* SB_PGD_PORT_TX_WATERMARK */
			{ 1, 0x850, 197, 0x01, 8, 1 }, /* SB_PGD_PORT_TX_ENABLE */
			{ 1, 0x840, 198, 0x1E, 8, 1 }, /* SB_PGD_PORT_RX_WATERMARK */
			{ 1, 0x840, 199, 0x01, 8, 1 }, /* SB_PGD_PORT_RX_ENABLE */
			/* AANC */
			{ 1, 0x800+0x0a0b, 204, 0x04, 8, 0 }, /* AANC_FF_GAIN_ADAPTIVE */
			{ 1, 0x800+0x0a0b, 205, 0x08, 8, 0 }, /* AANC_FFGAIN_ADAPTIVE_EN */
			{ 1, 0x800+0x0a0e, 206, 0xFF, 8, 0 }, /* AANC_GAIN_CONTROL */
		};
		int ok = 0;

		for (i = 0; i < ARRAY_SIZE(cdc_regs); i++) {
			ret = q6afe_set_param(afe, NULL,
					      (void *)&cdc_regs[i],
					      AFE_PARAM_ID_CDC_REG_CFG,
					      AFE_MODULE_CDC_DEV_CFG,
					      sizeof(cdc_regs[i]),
					      AFE_CLK_TOKEN);
			if (!ret)
				ok++;
			else
				dev_info(afe->dev,
					 "CDC_REG_CFG[%d] type=%d addr=0x%x: REJECTED\n",
					 i, cdc_regs[i].reg_field_type,
					 cdc_regs[i].reg_logical_addr);
		}
		dev_info(afe->dev, "CDC_REG_CFG: %d/%zu accepted\n",
			 ok, ARRAY_SIZE(cdc_regs));
	}

	/* --- Step 2: CDC_REG_PAGE_CFG (before SLAVE_CFG) --- */
	{
		struct {
			u32 minor_version;
			u32 enable;
			u32 proc_id;
		} __packed page_cfg = { 1, 1, 1 };

		ret = q6afe_set_param(afe, NULL, &page_cfg,
				      0x00010296,
				      AFE_MODULE_CDC_DEV_CFG,
				      sizeof(page_cfg), AFE_CLK_TOKEN);
		dev_info(afe->dev, "CDC_REG_PAGE_CFG: %d\n", ret);
	}

	/* --- Step 3: CDC_SLIMBUS_SLAVE_CFG --- */
	slave_cfg.minor_version = 1;
	slave_cfg.device_enum_addr_lsw = 0x01a00100;
	slave_cfg.device_enum_addr_msw = 0x0217;
	slave_cfg.tx_slave_port_offset = 0;
	slave_cfg.rx_slave_port_offset = 16;

	ret = q6afe_set_param(afe, NULL, &slave_cfg,
			      AFE_PARAM_ID_CDC_SLIMBUS_SLAVE_CFG,
			      AFE_MODULE_CDC_DEV_CFG,
			      sizeof(slave_cfg), AFE_CLK_TOKEN);
	dev_info(afe->dev, "CDC_SLIMBUS_SLAVE_CFG: %d\n", ret);

	/* --- Step 4: CDC_REG_CFG_INIT (downstream sends after SLAVE_CFG) --- */
	if (!ret) {
		ret = q6afe_set_param(afe, NULL, NULL,
				      AFE_PARAM_ID_CDC_REG_CFG_INIT,
				      AFE_MODULE_CDC_DEV_CFG,
				      0, AFE_CLK_TOKEN);
		dev_info(afe->dev, "CDC_REG_CFG_INIT: %d\n", ret);
	}

	/* --- Step 5: CDC_REG_PAGE_CFG again (downstream sends twice) --- */
	{
		struct {
			u32 minor_version;
			u32 enable;
			u32 proc_id;
		} __packed page_cfg = { 1, 1, 1 };

		ret = q6afe_set_param(afe, NULL, &page_cfg,
				      0x00010296,
				      AFE_MODULE_CDC_DEV_CFG,
				      sizeof(page_cfg), AFE_CLK_TOKEN);
		dev_info(afe->dev, "CDC_REG_PAGE_CFG(2): %d\n", ret);
	}

	return 0; /* non-fatal */
}

/**
 * q6afe_port_start() - Start a afe port
 *
 * @port: Instance of port to start
 *
 * Return: Will be an negative on packet size on success.
 */
int q6afe_port_start(struct q6afe_port *port)
{
	struct afe_port_cmd_device_start *start;
	struct q6afe *afe = port->afe;
	int port_id = port->id;
	int ret, param_id = port->cfg_type;
	struct apr_pkt *pkt;
	int pkt_size;
	void *p __free(kfree) = NULL;

	/*
	 * Send CDC SLIMbus slave config + register config once.
	 * Downstream techpack sends this during codec init via
	 * AFE_SVC_CMD_SET_PARAM with AFE_MODULE_CDC_DEV_CFG.
	 */
	if ((port_id == 0x4001 || port_id == 0x4000) &&
	    !afe->slim_slave_cfg_sent) {
		ret = q6afe_send_cdc_slimbus_slave_cfg(afe);
		if (ret)
			dev_warn(afe->dev,
				 "CDC slave cfg failed: %d (non-fatal)\n", ret);
		else
			afe->slim_slave_cfg_sent = true;
	}

	/* Debug: dump SLIMbus AFE config before sending */
	if (port_id == 0x4001 || port_id == 0x4000) {
		struct afe_param_id_slimbus_cfg *sc = &port->port_cfg.slim_cfg;

		dev_info(afe->dev,
			 "AFE SLIM port 0x%x: ver=%d dev=%d bw=%d rate=%d fmt=%d nch=%d ch=[%d,%d,%d,%d]\n",
			 port_id, sc->sb_cfg_minor_version,
			 sc->slimbus_dev_id, sc->bit_width,
			 sc->sample_rate, sc->data_format,
			 sc->num_channels,
			 sc->shared_ch_mapping[0], sc->shared_ch_mapping[1],
			 sc->shared_ch_mapping[2], sc->shared_ch_mapping[3]);
	}

	/* ACDB AFE cal: infrastructure ready (shared memory works) but
	 * earpiece doesn't need AFE-level cal — downstream also skips it.
	 * The ADSP rejects module-specific cal with EBADPARAM when no
	 * topology is loaded, and downstream also runs without topology.
	 */

	/* Send topology ID for SLIMbus ports */
	if (port_id == 0x4001 || port_id == 0x4000) {
		u32 topology = 0;

		ret = q6afe_port_set_param_v2(port, &topology,
					      AFE_PARAM_ID_SET_TOPOLOGY,
					      AFE_MODULE_AUDIO_DEV_INTERFACE,
					      sizeof(topology));
		if (ret)
			dev_warn(afe->dev,
				 "AFE SET_TOPOLOGY for port 0x%x failed %d\n",
				 port_id, ret);
	}

	/* Send PORT_MEDIA_TYPE for SLIMbus ports (required by ADSP to
	 * configure the audio data path before DEVICE_START) */
	if (port_id == 0x4001 || port_id == 0x4000) {
		struct {
			u32 minor_version;
			u32 sample_rate;
			u16 bit_width;
			u16 num_channels;
			u16 data_format;
			u16 reserved;
		} __packed media_type = {
			.minor_version = 1,
			.sample_rate = port->port_cfg.slim_cfg.sample_rate,
			.bit_width = port->port_cfg.slim_cfg.bit_width,
			.num_channels = port->port_cfg.slim_cfg.num_channels,
			.data_format = 0, /* AFE_PORT_DATA_FORMAT_PCM */
			.reserved = 0,
		};

		ret = q6afe_port_set_param_v2(port, &media_type,
					      0x000102a7, /* AFE_PARAM_ID_PORT_MEDIA_TYPE */
					      0x000102a6, /* AFE_MODULE_PORT */
					      sizeof(media_type));
		if (ret)
			dev_warn(afe->dev,
				 "AFE PORT_MEDIA_TYPE for port 0x%x: %d\n",
				 port_id, ret);
	}

	ret  = q6afe_port_set_param_v2(port, &port->port_cfg, param_id,
				       AFE_MODULE_AUDIO_DEV_INTERFACE,
				       sizeof(port->port_cfg));
	if (ret) {
		dev_err(afe->dev, "AFE enable for port 0x%x failed %d\n",
			port_id, ret);
		return ret;
	}

	/* Send AFE_PARAM_ID_ENABLE to activate the port's audio path */
	{
		u32 enable = 1;

		ret = q6afe_port_set_param_v2(port, &enable, 0x00010203,
					      AFE_MODULE_AUDIO_DEV_INTERFACE,
					      sizeof(enable));
		if (ret)
			dev_info(afe->dev,
				 "AFE ENABLE param for port 0x%x: %d (non-fatal)\n",
				 port_id, ret);
	}

	if (port->scfg) {
		ret  = q6afe_port_set_param_v2(port, port->scfg,
					AFE_PARAM_ID_PORT_SLOT_MAPPING_CONFIG,
					AFE_MODULE_TDM, sizeof(*port->scfg));
		if (ret) {
			dev_err(afe->dev, "AFE enable for port 0x%x failed %d\n",
			port_id, ret);
			return ret;
		}
	}

	pkt_size = APR_HDR_SIZE + sizeof(*start);
	p = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	pkt = p;
	start = p + APR_HDR_SIZE;

	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					    APR_HDR_LEN(APR_HDR_SIZE),
					    APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = port->token;
	pkt->hdr.opcode = AFE_PORT_CMD_DEVICE_START;

	start->port_id = port_id;

	ret = afe_apr_send_pkt(afe, pkt, port, AFE_PORT_CMD_DEVICE_START);
	if (ret)
		dev_err(afe->dev, "AFE enable for port 0x%x failed %d\n",
			port_id, ret);
	else
		dev_info(afe->dev, "AFE DEVICE_START port 0x%x: SUCCESS\n",
			 port_id);

	return ret;
}
EXPORT_SYMBOL_GPL(q6afe_port_start);

/**
 * q6afe_port_get_from_id() - Get port instance from a port id
 *
 * @dev: Pointer to afe child device.
 * @id: port id
 *
 * Return: Will be an error pointer on error or a valid afe port
 * on success.
 */
struct q6afe_port *q6afe_port_get_from_id(struct device *dev, int id)
{
	int port_id;
	struct q6afe *afe = dev_get_drvdata(dev->parent);
	struct q6afe_port *port;
	int cfg_type;

	if (id < 0 || id >= AFE_PORT_MAX) {
		dev_err(dev, "AFE port token[%d] invalid!\n", id);
		return ERR_PTR(-EINVAL);
	}

	/* if port is multiple times bind/unbind before callback finishes */
	port = q6afe_find_port(afe, id);
	if (port) {
		dev_err(dev, "AFE Port already open\n");
		return port;
	}

	port_id = port_maps[id].port_id;

	switch (port_id) {
	case AFE_PORT_ID_MULTICHAN_HDMI_RX:
	case AFE_PORT_ID_HDMI_OVER_DP_RX:
		cfg_type = AFE_PARAM_ID_HDMI_CONFIG;
		break;
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_TX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_RX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_RX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_RX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_RX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_RX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX:
	case AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX:
		cfg_type = AFE_PARAM_ID_SLIMBUS_CONFIG;
		break;

	case AFE_PORT_ID_PRIMARY_MI2S_RX:
	case AFE_PORT_ID_PRIMARY_MI2S_TX:
	case AFE_PORT_ID_SECONDARY_MI2S_RX:
	case AFE_PORT_ID_SECONDARY_MI2S_TX:
	case AFE_PORT_ID_TERTIARY_MI2S_RX:
	case AFE_PORT_ID_TERTIARY_MI2S_TX:
	case AFE_PORT_ID_QUATERNARY_MI2S_RX:
	case AFE_PORT_ID_QUATERNARY_MI2S_TX:
	case AFE_PORT_ID_QUINARY_MI2S_RX:
	case AFE_PORT_ID_QUINARY_MI2S_TX:
		cfg_type = AFE_PARAM_ID_I2S_CONFIG;
		break;
	case AFE_PORT_ID_PRIMARY_TDM_RX ... AFE_PORT_ID_QUINARY_TDM_TX_7:
		cfg_type = AFE_PARAM_ID_TDM_CONFIG;
		break;
	case AFE_PORT_ID_WSA_CODEC_DMA_RX_0 ... AFE_PORT_ID_RX_CODEC_DMA_RX_7:
		cfg_type = AFE_PARAM_ID_CODEC_DMA_CONFIG;
		break;
	case AFE_PORT_ID_USB_RX:
		cfg_type = AFE_PARAM_ID_USB_AUDIO_CONFIG;
		break;
	default:
		dev_err(dev, "Invalid port id 0x%x\n", port_id);
		return ERR_PTR(-EINVAL);
	}

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	init_waitqueue_head(&port->wait);

	port->token = id;
	port->id = port_id;
	port->afe = afe;
	port->cfg_type = cfg_type;
	kref_init(&port->refcount);

	guard(spinlock_irqsave)(&afe->port_list_lock);
	list_add_tail(&port->node, &afe->port_list);

	return port;

}
EXPORT_SYMBOL_GPL(q6afe_port_get_from_id);

/**
 * q6afe_port_put() - Release port reference
 *
 * @port: Instance of port to put
 */
void q6afe_port_put(struct q6afe_port *port)
{
	kref_put(&port->refcount, q6afe_port_free);
}
EXPORT_SYMBOL_GPL(q6afe_port_put);

int q6afe_unvote_lpass_core_hw(struct device *dev, uint32_t hw_block_id,
			       uint32_t client_handle)
{
	struct q6afe *afe = dev_get_drvdata(dev->parent);
	struct afe_cmd_remote_lpass_core_hw_devote_request *vote_cfg;
	struct apr_pkt *pkt;
	int ret = 0;
	int pkt_size = APR_HDR_SIZE + sizeof(*vote_cfg);
	void *p __free(kfree) = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	pkt = p;
	vote_cfg = p + APR_HDR_SIZE;

	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = hw_block_id;
	pkt->hdr.opcode = AFE_CMD_REMOTE_LPASS_CORE_HW_DEVOTE_REQUEST;
	vote_cfg->hw_block_id = hw_block_id;
	vote_cfg->client_handle = client_handle;

	ret = apr_send_pkt(afe->apr, pkt);
	if (ret < 0)
		dev_err(afe->dev, "AFE failed to unvote (%d)\n", hw_block_id);

	return ret;
}
EXPORT_SYMBOL(q6afe_unvote_lpass_core_hw);

int q6afe_vote_lpass_core_hw(struct device *dev, uint32_t hw_block_id,
			     const char *client_name, uint32_t *client_handle)
{
	struct q6afe *afe = dev_get_drvdata(dev->parent);
	struct afe_cmd_remote_lpass_core_hw_vote_request *vote_cfg;
	struct apr_pkt *pkt;
	int ret = 0;
	int pkt_size = APR_HDR_SIZE + sizeof(*vote_cfg);
	void *p __free(kfree) = kzalloc(pkt_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	pkt = p;
	vote_cfg = p + APR_HDR_SIZE;

	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.src_port = 0;
	pkt->hdr.dest_port = 0;
	pkt->hdr.token = hw_block_id;
	pkt->hdr.opcode = AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST;
	vote_cfg->hw_block_id = hw_block_id;
	strscpy(vote_cfg->client_name, client_name,
			sizeof(vote_cfg->client_name));

	ret = afe_apr_send_pkt(afe, pkt, NULL,
			       AFE_CMD_RSP_REMOTE_LPASS_CORE_HW_VOTE_REQUEST);
	if (ret)
		dev_err(afe->dev, "AFE failed to vote (%d)\n", hw_block_id);

	return ret;
}
EXPORT_SYMBOL(q6afe_vote_lpass_core_hw);

static int q6afe_probe(struct apr_device *adev)
{
	struct q6afe *afe;
	struct device *dev = &adev->dev;

	afe = devm_kzalloc(dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;

	q6core_get_svc_api_info(adev->svc_id, &afe->ainfo);
	afe->apr = adev;
	mutex_init(&afe->lock);
	init_waitqueue_head(&afe->wait);
	afe->dev = dev;
	INIT_LIST_HEAD(&afe->port_list);
	spin_lock_init(&afe->port_list_lock);

	dev_set_drvdata(dev, afe);

	return devm_of_platform_populate(dev);
}

#ifdef CONFIG_OF
static const struct of_device_id q6afe_device_id[]  = {
	{ .compatible = "qcom,q6afe" },
	{},
};
MODULE_DEVICE_TABLE(of, q6afe_device_id);
#endif

static struct apr_driver qcom_q6afe_driver = {
	.probe = q6afe_probe,
	.callback = q6afe_callback,
	.driver = {
		.name = "qcom-q6afe",
		.of_match_table = of_match_ptr(q6afe_device_id),

	},
};

module_apr_driver(qcom_q6afe_driver);
MODULE_DESCRIPTION("Q6 Audio Front End");
MODULE_LICENSE("GPL v2");
