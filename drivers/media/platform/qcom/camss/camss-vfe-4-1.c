// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-4-1.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v4.1
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-gen1.h"
#include "camss-vfe-vbif.h"

#define VFE_0_HW_VERSION		0x000

#define VFE_0_GLOBAL_RESET_CMD		0x00c
#define VFE_0_GLOBAL_RESET_CMD_CORE	BIT(0)
#define VFE_0_GLOBAL_RESET_CMD_CAMIF	BIT(1)
#define VFE_0_GLOBAL_RESET_CMD_BUS	BIT(2)
#define VFE_0_GLOBAL_RESET_CMD_BUS_BDG	BIT(3)
#define VFE_0_GLOBAL_RESET_CMD_REGISTER	BIT(4)
#define VFE_0_GLOBAL_RESET_CMD_TIMER	BIT(5)
#define VFE_0_GLOBAL_RESET_CMD_PM	BIT(6)
#define VFE_0_GLOBAL_RESET_CMD_BUS_MISR	BIT(7)
#define VFE_0_GLOBAL_RESET_CMD_TESTGEN	BIT(8)

#define VFE_0_MODULE_CFG		0x018
#define VFE_0_MODULE_CFG_DEMUX			BIT(2)
#define VFE_0_MODULE_CFG_CHROMA_UPSAMPLE	BIT(3)
#define VFE_0_MODULE_CFG_DEMOSAIC		BIT(4)
#define VFE_0_MODULE_CFG_WB			BIT(11)
#define VFE_0_MODULE_CFG_COLOR_CORRECT		BIT(13)
#define VFE_0_MODULE_CFG_RGB_LUT		BIT(14)
#define VFE_0_MODULE_CFG_CHROMA_ENHANCE		BIT(17)
#define VFE_0_MODULE_CFG_COLOR_XFORM_ENC	BIT(21)
#define VFE_0_MODULE_CFG_SCALE_ENC		BIT(23)
#define VFE_0_MODULE_CFG_CROP_ENC		BIT(27)

#define VFE_0_CORE_CFG			0x01c
#define VFE_0_CORE_CFG_PIXEL_PATTERN_GRGRGR	0x1
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR	0x4
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB	0x5
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY	0x6
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY	0x7

#define VFE_0_IRQ_CMD			0x024
#define VFE_0_IRQ_CMD_GLOBAL_CLEAR	BIT(0)

#define VFE_0_IRQ_MASK_0		0x028
#define VFE_0_IRQ_MASK_0_CAMIF_SOF			BIT(0)
#define VFE_0_IRQ_MASK_0_CAMIF_EOF			BIT(1)
#define VFE_0_IRQ_MASK_0_RDIn_REG_UPDATE(n)		BIT((n) + 5)
#define VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? BIT(4) : VFE_0_IRQ_MASK_0_RDIn_REG_UPDATE(n))
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 25)
#define VFE_0_IRQ_MASK_0_RESET_ACK			BIT(31)
#define VFE_0_IRQ_MASK_1		0x02c
#define VFE_0_IRQ_MASK_1_CAMIF_ERROR			BIT(0)
#define VFE_0_IRQ_MASK_1_VIOLATION			BIT(7)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		BIT(8)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	BIT((n) + 9)
#define VFE_0_IRQ_MASK_1_RDIn_SOF(n)			BIT((n) + 29)

#define VFE_0_IRQ_CLEAR_0		0x030
#define VFE_0_IRQ_CLEAR_1		0x034

#define VFE_0_IRQ_STATUS_0		0x038
#define VFE_0_IRQ_STATUS_0_CAMIF_SOF			BIT(0)
#define VFE_0_IRQ_STATUS_0_RDIn_REG_UPDATE(n)		BIT((n) + 5)
#define VFE_0_IRQ_STATUS_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? BIT(4) : VFE_0_IRQ_STATUS_0_RDIn_REG_UPDATE(n))
#define VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 25)
#define VFE_0_IRQ_STATUS_0_RESET_ACK			BIT(31)
#define VFE_0_IRQ_STATUS_1		0x03c
#define VFE_0_IRQ_STATUS_1_VIOLATION			BIT(7)
#define VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK		BIT(8)
#define VFE_0_IRQ_STATUS_1_RDIn_SOF(n)			BIT((n) + 29)

#define VFE_0_IRQ_COMPOSITE_MASK_0	0x40
#define VFE_0_VIOLATION_STATUS		0x48

#define VFE_0_BUS_CMD			0x4c
#define VFE_0_BUS_CMD_Mx_RLD_CMD(x)	BIT(x)

#define VFE_0_BUS_CFG			0x050

#define VFE_0_BUS_XBAR_CFG_x(x)		(0x58 + 0x4 * ((x) / 2))
#define VFE_0_BUS_XBAR_CFG_x_M_PAIR_STREAM_EN			BIT(1)
#define VFE_0_BUS_XBAR_CFG_x_M_PAIR_STREAM_SWAP_INTER_INTRA	(0x3 << 4)
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT		8
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_LUMA		0
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0	5
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1	6
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2	7

#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(n)		(0x06c + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_WR_PATH_SHIFT	0
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT	1
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(n)	(0x070 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(n)	(0x074 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(n)		(0x078 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT	2
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK	(0x1f << 2)

#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(n)		(0x07c + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT	16
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(n)	(0x080 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_BUFFER_CFG(n)	(0x084 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(n)	\
							(0x088 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(n)	\
							(0x08c + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF	0xffffffff

#define VFE_0_BUS_PING_PONG_STATUS	0x268

#define VFE_0_BUS_BDG_CMD		0x2c0
#define VFE_0_BUS_BDG_CMD_HALT_REQ	1

#define VFE_0_BUS_BDG_QOS_CFG_0		0x2c4
#define VFE_0_BUS_BDG_QOS_CFG_0_CFG	0xaaa5aaa5
#define VFE_0_BUS_BDG_QOS_CFG_1		0x2c8
#define VFE_0_BUS_BDG_QOS_CFG_2		0x2cc
#define VFE_0_BUS_BDG_QOS_CFG_3		0x2d0
#define VFE_0_BUS_BDG_QOS_CFG_4		0x2d4
#define VFE_0_BUS_BDG_QOS_CFG_5		0x2d8
#define VFE_0_BUS_BDG_QOS_CFG_6		0x2dc
#define VFE_0_BUS_BDG_QOS_CFG_7		0x2e0
#define VFE_0_BUS_BDG_QOS_CFG_7_CFG	0x0001aaa5

#define VFE_0_RDI_CFG_x(x)		(0x2e8 + (0x4 * (x)))
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT	28
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK	(0xf << 28)
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT	4
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK		(0xf << 4)
#define VFE_0_RDI_CFG_x_RDI_EN_BIT		BIT(2)
#define VFE_0_RDI_CFG_x_MIPI_EN_BITS		0x3
#define VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(r)	BIT(16 + (r))

#define VFE_0_CAMIF_CMD				0x2f4
#define VFE_0_CAMIF_CMD_DISABLE_FRAME_BOUNDARY	0
#define VFE_0_CAMIF_CMD_ENABLE_FRAME_BOUNDARY	1
#define VFE_0_CAMIF_CMD_NO_CHANGE		3
#define VFE_0_CAMIF_CMD_CLEAR_CAMIF_STATUS	BIT(2)
#define VFE_0_CAMIF_CFG				0x2f8
#define VFE_0_CAMIF_CFG_VFE_OUTPUT_EN		BIT(6)
#define VFE_0_CAMIF_FRAME_CFG			0x300
#define VFE_0_CAMIF_WINDOW_WIDTH_CFG		0x304
#define VFE_0_CAMIF_WINDOW_HEIGHT_CFG		0x308
#define VFE_0_CAMIF_SUBSAMPLE_CFG_0		0x30c
#define VFE_0_CAMIF_IRQ_SUBSAMPLE_PATTERN	0x314
#define VFE_0_CAMIF_STATUS			0x31c
#define VFE_0_CAMIF_STATUS_HALT			BIT(31)

#define VFE_0_REG_UPDATE			0x378
#define VFE_0_REG_UPDATE_RDIn(n)		BIT(1 + (n))
#define VFE_0_REG_UPDATE_line_n(n)		\
			((n) == VFE_LINE_PIX ? 1 : VFE_0_REG_UPDATE_RDIn(n))

#define VFE_0_DEMUX_CFG				0x424
#define VFE_0_DEMUX_CFG_PERIOD			0x3
#define VFE_0_DEMUX_GAIN_0			0x428
#define VFE_0_DEMUX_GAIN_0_CH0_EVEN		(0x80 << 0)
#define VFE_0_DEMUX_GAIN_0_CH0_ODD		(0x80 << 16)
#define VFE_0_DEMUX_GAIN_1			0x42c
#define VFE_0_DEMUX_GAIN_1_CH1			(0x80 << 0)
#define VFE_0_DEMUX_GAIN_1_CH2			(0x80 << 16)
#define VFE_0_DEMUX_R_GAIN_0			0x430
#define VFE_0_DEMUX_R_GAIN_1			0x434
#define VFE_0_DEMUX_EVEN_CFG			0x438
#define VFE_0_DEMUX_EVEN_CFG_PATTERN_GRBG	0x9c
#define VFE_0_DEMUX_EVEN_CFG_PATTERN_YUYV	0x9cac
#define VFE_0_DEMUX_EVEN_CFG_PATTERN_YVYU	0xac9c
#define VFE_0_DEMUX_EVEN_CFG_PATTERN_UYVY	0xc9ca
#define VFE_0_DEMUX_EVEN_CFG_PATTERN_VYUY	0xcac9
#define VFE_0_DEMUX_ODD_CFG			0x43c
#define VFE_0_DEMUX_ODD_CFG_PATTERN_GRBG	0xca
#define VFE_0_DEMUX_ODD_CFG_PATTERN_YUYV	0x9cac
#define VFE_0_DEMUX_ODD_CFG_PATTERN_YVYU	0xac9c
#define VFE_0_DEMUX_ODD_CFG_PATTERN_UYVY	0xc9ca
#define VFE_0_DEMUX_ODD_CFG_PATTERN_VYUY	0xcac9

#define VFE_0_DEMOSAIC_WB_GAIN_0		0x518
#define VFE_0_DEMOSAIC_WB_GAIN_1		0x51c
#define VFE_0_WB_CFG				0x580
#define VFE_0_CC_COEF(n)			(0x5d0 + 0x4 * (n))
#define VFE_0_CC_OFFSET(n)			(0x5f4 + 0x4 * (n))
#define VFE_0_CC_COEF_Q				0x600
#define VFE_0_RGB_LUT_BANK_SEL			0x638

#define VFE_0_DMI_CFG				0x910
#define VFE_0_DMI_CFG_AUTO_INC			BIT(8)
#define VFE_0_DMI_ADDR				0x914
#define VFE_0_DMI_DATA_LO			0x91c
#define VFE_0_DMI_RGB_LUT_CH0			0x9
#define VFE_0_DMI_RGB_LUT_CH1			0xb
#define VFE_0_DMI_RGB_LUT_CH2			0xd

#define VFE_0_STATS_CFG				0x888
#define VFE_0_STATS_CFG_COLOR_CONV_EN		BIT(0)

#define VFE_0_SCALE_ENC_Y_CFG			0x75c
#define VFE_0_SCALE_ENC_Y_H_IMAGE_SIZE		0x760
#define VFE_0_SCALE_ENC_Y_H_PHASE		0x764
#define VFE_0_SCALE_ENC_Y_V_IMAGE_SIZE		0x76c
#define VFE_0_SCALE_ENC_Y_V_PHASE		0x770
#define VFE_0_SCALE_ENC_CBCR_CFG		0x778
#define VFE_0_SCALE_ENC_CBCR_H_IMAGE_SIZE	0x77c
#define VFE_0_SCALE_ENC_CBCR_H_PHASE		0x780
#define VFE_0_SCALE_ENC_CBCR_V_IMAGE_SIZE	0x790
#define VFE_0_SCALE_ENC_CBCR_V_PHASE		0x794

#define VFE_0_CROP_ENC_Y_WIDTH			0x854
#define VFE_0_CROP_ENC_Y_HEIGHT			0x858
#define VFE_0_CROP_ENC_CBCR_WIDTH		0x85c
#define VFE_0_CROP_ENC_CBCR_HEIGHT		0x860

#define VFE_0_CLAMP_ENC_MAX_CFG			0x874
#define VFE_0_CLAMP_ENC_MAX_CFG_CH0		(0xff << 0)
#define VFE_0_CLAMP_ENC_MAX_CFG_CH1		(0xff << 8)
#define VFE_0_CLAMP_ENC_MAX_CFG_CH2		(0xff << 16)
#define VFE_0_CLAMP_ENC_MIN_CFG			0x878
#define VFE_0_CLAMP_ENC_MIN_CFG_CH0		(0x0 << 0)
#define VFE_0_CLAMP_ENC_MIN_CFG_CH1		(0x0 << 8)
#define VFE_0_CLAMP_ENC_MIN_CFG_CH2		(0x0 << 16)

#define VFE_0_CGC_OVERRIDE_1			0x974
#define VFE_0_CGC_OVERRIDE_1_IMAGE_Mx_CGC_OVERRIDE(x)	BIT(x)

#define CAMIF_TIMEOUT_SLEEP_US 1000
#define CAMIF_TIMEOUT_ALL_US 1000000

#define MSM_VFE_VFE0_UB_SIZE 1023
#define MSM_VFE_VFE0_UB_SIZE_RDI (MSM_VFE_VFE0_UB_SIZE / 3)

static u16 vfe_get_ub_size(u8 vfe_id)
{
	if (vfe_id == 0)
		return MSM_VFE_VFE0_UB_SIZE_RDI;

	return 0;
}

static inline void vfe_reg_clr(struct vfe_device *vfe, u32 reg, u32 clr_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits & ~clr_bits, vfe->base + reg);
}

static inline void vfe_reg_set(struct vfe_device *vfe, u32 reg, u32 set_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits | set_bits, vfe->base + reg);
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	u32 reset_bits = VFE_0_GLOBAL_RESET_CMD_TESTGEN		|
			 VFE_0_GLOBAL_RESET_CMD_BUS_MISR	|
			 VFE_0_GLOBAL_RESET_CMD_PM		|
			 VFE_0_GLOBAL_RESET_CMD_TIMER		|
			 VFE_0_GLOBAL_RESET_CMD_REGISTER	|
			 VFE_0_GLOBAL_RESET_CMD_BUS_BDG		|
			 VFE_0_GLOBAL_RESET_CMD_BUS		|
			 VFE_0_GLOBAL_RESET_CMD_CAMIF		|
			 VFE_0_GLOBAL_RESET_CMD_CORE;

	writel_relaxed(reset_bits, vfe->base + VFE_0_GLOBAL_RESET_CMD);
}

static void vfe_halt_request(struct vfe_device *vfe)
{
	writel_relaxed(VFE_0_BUS_BDG_CMD_HALT_REQ,
		       vfe->base + VFE_0_BUS_BDG_CMD);
}

static void vfe_halt_clear(struct vfe_device *vfe)
{
	writel_relaxed(0x0, vfe->base + VFE_0_BUS_BDG_CMD);
}

static void vfe_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			    1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_WR_PATH_SHIFT);
	else
		vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			    1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_WR_PATH_SHIFT);
}

static void vfe_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT);
	else
		vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT);
}

static void vfe_get_wm_sizes(struct v4l2_pix_format_mplane *pix, u8 plane,
			     u16 *width, u16 *height, u16 *bytesperline)
{
	*width = pix->width;
	*height = pix->height;
	*bytesperline = pix->plane_fmt[0].bytesperline;

	if (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
	    pix->pixelformat == V4L2_PIX_FMT_NV21)
		if (plane == 1)
			*height /= 2;
}

static void vfe_wm_line_based(struct vfe_device *vfe, u32 wm,
			      struct v4l2_pix_format_mplane *pix,
			      u8 plane, u32 enable)
{
	u32 reg;

	if (enable) {
		u16 width = 0, height = 0, bytesperline = 0, wpl;

		vfe_get_wm_sizes(pix, plane, &width, &height, &bytesperline);

		wpl = vfe_word_per_line(pix->pixelformat, width);

		reg = height - 1;
		reg |= ((wpl + 1) / 2 - 1) << 16;

		writel_relaxed(reg, vfe->base +
			       VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));

		wpl = vfe_word_per_line(pix->pixelformat, bytesperline);

		/* msm8953 moved the buffer height down a bit */
		reg = 0x3;
		reg |= (height - 1) << (vfe->camss->res->version == CAMSS_8x53 ? 3 : 4);
		reg |= wpl << 16;

		writel_relaxed(reg, vfe->base +
			       VFE_0_BUS_IMAGE_MASTER_n_WR_BUFFER_CFG(wm));
	} else {
		writel_relaxed(0, vfe->base +
			       VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));
		writel_relaxed(0, vfe->base +
			       VFE_0_BUS_IMAGE_MASTER_n_WR_BUFFER_CFG(wm));
	}
}

static void vfe_wm_set_framedrop_period(struct vfe_device *vfe, u8 wm, u8 per)
{
	u32 reg;

	reg = readl_relaxed(vfe->base +
			    VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	reg &= ~(VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK);

	reg |= (per << VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT)
		& VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK;

	writel_relaxed(reg,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));
}

static void vfe_wm_set_framedrop_pattern(struct vfe_device *vfe, u8 wm,
					 u32 pattern)
{
	writel_relaxed(pattern,
	       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(wm));
}

static void vfe_wm_set_ub_cfg(struct vfe_device *vfe, u8 wm,
			      u16 offset, u16 depth)
{
	u32 reg;

	reg = (offset << VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT) |
		depth;
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
}

static void vfe_bus_reload_wm(struct vfe_device *vfe, u8 wm)
{
	wmb();
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm), vfe->base + VFE_0_BUS_CMD);
	wmb();
}

static void vfe_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
}

static void vfe_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
}

static int vfe_wm_get_ping_pong_status(struct vfe_device *vfe, u8 wm)
{
	u32 reg;

	reg = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (reg >> wm) & 0x1;
}

static void vfe_bus_enable_wr_if(struct vfe_device *vfe, u8 enable)
{
	if (enable)
		writel_relaxed(0x10000009, vfe->base + VFE_0_BUS_CFG);
	else
		writel_relaxed(0, vfe->base + VFE_0_BUS_CFG);
}

static void vfe_bus_connect_wm_to_rdi(struct vfe_device *vfe, u8 wm,
				      enum vfe_line_id id)
{
	u32 reg;

	reg = VFE_0_RDI_CFG_x_MIPI_EN_BITS;
	reg |= VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(id);
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(0), reg);

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	reg |= ((3 * id) << VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT) &
		VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK;
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(id), reg);

	switch (id) {
	case VFE_LINE_RDI0:
	default:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	}

	if (wm % 2 == 1)
		reg <<= 16;

	vfe_reg_set(vfe, VFE_0_BUS_XBAR_CFG_x(wm), reg);
}

static void vfe_wm_set_subsample(struct vfe_device *vfe, u8 wm)
{
	writel_relaxed(VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(wm));
}

static void vfe_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
					   enum vfe_line_id id)
{
	u32 reg;

	reg = VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(id);
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(0), reg);

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(id), reg);

	switch (id) {
	case VFE_LINE_RDI0:
	default:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	}

	if (wm % 2 == 1)
		reg <<= 16;

	vfe_reg_clr(vfe, VFE_0_BUS_XBAR_CFG_x(wm), reg);
}

static void vfe_set_xbar_cfg(struct vfe_device *vfe, struct vfe_output *output,
			     u8 enable)
{
	struct vfe_line *line = container_of(output, struct vfe_line, output);
	u32 p = line->video_out.active_fmt.fmt.pix_mp.pixelformat;
	u32 reg;
	unsigned int i;

	for (i = 0; i < output->wm_num; i++) {
		if (i == 0) {
			reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_LUMA <<
				VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		} else if (i == 1) {
			reg = VFE_0_BUS_XBAR_CFG_x_M_PAIR_STREAM_EN;
			if (p == V4L2_PIX_FMT_NV12 || p == V4L2_PIX_FMT_NV16)
				reg |= VFE_0_BUS_XBAR_CFG_x_M_PAIR_STREAM_SWAP_INTER_INTRA;
		} else {
			/* On current devices output->wm_num is always <= 2 */
			break;
		}

		if (output->wm_idx[i] % 2 == 1)
			reg <<= 16;

		if (enable)
			vfe_reg_set(vfe,
				    VFE_0_BUS_XBAR_CFG_x(output->wm_idx[i]),
				    reg);
		else
			vfe_reg_clr(vfe,
				    VFE_0_BUS_XBAR_CFG_x(output->wm_idx[i]),
				    reg);
	}
}

static void vfe_set_realign_cfg(struct vfe_device *vfe, struct vfe_line *line,
				u8 enable)
{
	/* empty */
}
static void vfe_set_rdi_cid(struct vfe_device *vfe, enum vfe_line_id id, u8 cid)
{
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(id),
		    VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK);

	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(id),
		    cid << VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= VFE_0_REG_UPDATE_line_n(line_id);
	wmb();
	writel_relaxed(vfe->reg_update, vfe->base + VFE_0_REG_UPDATE);
	wmb();
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	vfe->reg_update &= ~VFE_0_REG_UPDATE_line_n(line_id);
}

static void vfe_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
				   enum vfe_line_id line_id, u8 enable)
{
	u32 irq_en0 = VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm) |
		      VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(line_id);
	u32 irq_en1 = VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(wm) |
		      VFE_0_IRQ_MASK_1_RDIn_SOF(line_id);

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, irq_en1);
	}
}

static void vfe_enable_irq_pix_line(struct vfe_device *vfe, u8 comp,
				    enum vfe_line_id line_id, u8 enable)
{
	struct vfe_output *output = &vfe->line[line_id].output;
	unsigned int i;
	u32 irq_en0;
	u32 irq_en1;
	u32 comp_mask = 0;

	irq_en0 = VFE_0_IRQ_MASK_0_CAMIF_SOF;
	irq_en0 |= VFE_0_IRQ_MASK_0_CAMIF_EOF;
	irq_en0 |= VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(comp);
	irq_en0 |= VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(line_id);
	irq_en1 = VFE_0_IRQ_MASK_1_CAMIF_ERROR;
	for (i = 0; i < output->wm_num; i++) {
		irq_en1 |= VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(
							output->wm_idx[i]);
		comp_mask |= (1 << output->wm_idx[i]) << comp * 8;
	}

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
		vfe_reg_set(vfe, VFE_0_IRQ_COMPOSITE_MASK_0, comp_mask);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, irq_en1);
		vfe_reg_clr(vfe, VFE_0_IRQ_COMPOSITE_MASK_0, comp_mask);
	}
}

static void vfe_enable_irq_common(struct vfe_device *vfe)
{
	u32 irq_en0 = VFE_0_IRQ_MASK_0_RESET_ACK;
	u32 irq_en1 = VFE_0_IRQ_MASK_1_VIOLATION |
		      VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK;

	vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
	vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
}

static bool vfe_is_bayer(struct vfe_line *line)
{
	return line->fmt[MSM_VFE_PAD_SINK].code == MEDIA_BUS_FMT_SGRBG10_1X10;
}

/*
 * Fixed Bayer pipeline: demosaic with its default interpolation
 * classifier, a gamma curve and full range BT.601 RGB to YCbCr
 * conversion, ahead of the encoder scaler. White balance comes from the
 * line's controls, colour correction from the sensor's calibration.
 */
static const struct {
	u16 offset;
	u32 value;
} vfe_bayer_cfg[] = {
	{ 0x440, 0x00000000 },	/* demosaic: BPC, BCC and ABF off */
	{ 0x520, 0x80003066 }, { 0x524, 0x81003066 }, { 0x528, 0x82003066 },
	{ 0x52c, 0x83003066 }, { 0x530, 0x84003066 }, { 0x534, 0x85003066 },
	{ 0x538, 0x86003066 }, { 0x53c, 0x87003066 }, { 0x540, 0x88003066 },
	{ 0x544, 0x803fe066 }, { 0x548, 0x813fe066 }, { 0x54c, 0x823fe066 },
	{ 0x550, 0x833fe066 }, { 0x554, 0x843fe066 }, { 0x558, 0x853fe066 },
	{ 0x55c, 0x863fe066 }, { 0x560, 0x873fe066 }, { 0x564, 0x883fe066 },
	{ 0x568, 0x0080ff00 },	/* demosaic interpolation, green */
	{ 0x56c, 0x0001a020 },
	{ 0x640, 0x04d },	/* RGB to Y: 0.299, 0.587, 0.114 in Q8 */
	{ 0x644, 0x096 },
	{ 0x648, 0x01d },
	{ 0x64c, 0x000 },
	{ 0x650, 0x00800080 },	/* chroma: a = 0.5 */
	{ 0x654, 0x0fa90fa9 },	/* b = -0.338 */
	{ 0x658, 0x00800080 },	/* c = 0.5 */
	{ 0x65c, 0x0fd70fd7 },	/* d = -0.162 */
	{ 0x660, 0x00800080 },	/* Cb and Cr offsets: 128 */
	{ 0x71c, 0x00000400 },	/* encoder colour transform: identity */
	{ 0x720, 0x00000000 },
	{ 0x724, 0x04000000 },
	{ 0x728, 0x00000000 },
	{ 0x72c, 0x00000000 },
	{ 0x730, 0x00000400 },
	{ 0x734, 0x00ffffff },
	{ 0x738, 0x00000000 },
};

/*
 * Gamma 2.2 over the 64 segments of each channel's RGB lookup table. Each
 * entry is the segment's 8-bit base and its rise to the next segment.
 */
static const u8 vfe_gamma_base[65] = {
	0, 39, 53, 63, 72, 80, 87, 93, 99, 105, 110, 115, 119, 124, 128, 132,
	136, 140, 143, 147, 150, 154, 157, 160, 163, 166, 169, 172, 175, 178,
	181, 183, 186, 189, 191, 194, 196, 199, 201, 204, 206, 208, 211, 213,
	215, 217, 219, 222, 224, 226, 228, 230, 232, 234, 236, 238, 240, 242,
	244, 246, 248, 249, 251, 253, 255,
};

static void vfe_set_rgb_lut(struct vfe_device *vfe)
{
	static const u8 rams[] = {
		VFE_0_DMI_RGB_LUT_CH0, VFE_0_DMI_RGB_LUT_CH1, VFE_0_DMI_RGB_LUT_CH2,
	};
	unsigned int r, i;

	for (r = 0; r < ARRAY_SIZE(rams); r++) {
		writel_relaxed(VFE_0_DMI_CFG_AUTO_INC | rams[r], vfe->base + VFE_0_DMI_CFG);
		writel_relaxed(0, vfe->base + VFE_0_DMI_ADDR);
		for (i = 0; i < 64; i++)
			writel_relaxed((vfe_gamma_base[i + 1] - vfe_gamma_base[i]) << 8 |
				       vfe_gamma_base[i], vfe->base + VFE_0_DMI_DATA_LO);
	}

	writel_relaxed(0, vfe->base + VFE_0_DMI_CFG);
	writel_relaxed(0, vfe->base + VFE_0_RGB_LUT_BANK_SEL);
}

/*
 * Colour correction of the S5KGM1SP under D50 from its Qualcomm
 * calibration, Q7, rows and columns in the hardware's G, B, R order.
 */
static const s16 vfe_ccm[9] = { 188, -36, -24, -91, 211, 8, -55, -20, 203 };

/* Sensor black level in the 12-bit pipeline */
#define VFE_BLACK_LEVEL		257

/*
 * The colour correction offsets take the black level out ahead of the
 * gamma curve: the pedestal has been through the white balance gains by
 * then, and the matrix mixes the channels.
 */
static void vfe_write_ccm(struct vfe_device *vfe, const u32 gains[3])
{
	unsigned int i, j;

	for (i = 0; i < 9; i++)
		writel_relaxed(vfe_ccm[i] & 0xfff, vfe->base + VFE_0_CC_COEF(i));

	for (i = 0; i < 3; i++) {
		s32 offset = 0;

		for (j = 0; j < 3; j++)
			offset -= vfe_ccm[i * 3 + j] * (s32)(VFE_BLACK_LEVEL * gains[j] / 128);
		offset /= 128;
		writel_relaxed(clamp(offset, -1024, 1023) & 0x7ff, vfe->base + VFE_0_CC_OFFSET(i));
	}

	writel_relaxed(0, vfe->base + VFE_0_CC_COEF_Q);
}

static void vfe_write_wb_gains(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 r = line->wb_red, b = line->wb_blue, g = 128;
	const u32 gains[3] = { g, b, r };

	writel_relaxed(g | b << 9 | r << 18, vfe->base + VFE_0_WB_CFG);
	vfe_write_ccm(vfe, gains);

	/* Demosaic interpolates with the same ratios to green */
	writel_relaxed(r | b << 15, vfe->base + VFE_0_DEMOSAIC_WB_GAIN_0);
	writel_relaxed(min(g * 128 / r, 511U) | min(g * 128 / b, 511U) << 15,
		       vfe->base + VFE_0_DEMOSAIC_WB_GAIN_1);
}

static void vfe_set_wb_gains(struct vfe_device *vfe, struct vfe_line *line)
{
	vfe_write_wb_gains(vfe, line);
	vfe_reg_update(vfe, line->id);
}

static void vfe_set_bayer_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vfe_bayer_cfg); i++)
		writel_relaxed(vfe_bayer_cfg[i].value,
			       vfe->base + vfe_bayer_cfg[i].offset);

	vfe_write_wb_gains(vfe, line);
	vfe_set_rgb_lut(vfe);

	writel_relaxed(VFE_0_MODULE_CFG_DEMUX | VFE_0_MODULE_CFG_DEMOSAIC |
		       VFE_0_MODULE_CFG_WB | VFE_0_MODULE_CFG_COLOR_CORRECT |
		       VFE_0_MODULE_CFG_RGB_LUT |
		       VFE_0_MODULE_CFG_CHROMA_ENHANCE |
		       VFE_0_MODULE_CFG_COLOR_XFORM_ENC |
		       VFE_0_MODULE_CFG_SCALE_ENC | VFE_0_MODULE_CFG_CROP_ENC,
		       vfe->base + VFE_0_MODULE_CFG);
	vfe_reg_set(vfe, VFE_0_STATS_CFG, VFE_0_STATS_CFG_COLOR_CONV_EN);
}

static void vfe_set_demux_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 val, even_cfg, odd_cfg;

	if (vfe_is_bayer(line)) {
		writel_relaxed(1, vfe->base + VFE_0_DEMUX_CFG);
		val = VFE_0_DEMUX_GAIN_0_CH0_EVEN | VFE_0_DEMUX_GAIN_0_CH0_ODD;
		writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_0);
		writel_relaxed(val, vfe->base + VFE_0_DEMUX_R_GAIN_0);
		val = VFE_0_DEMUX_GAIN_1_CH1 | VFE_0_DEMUX_GAIN_1_CH2;
		writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_1);
		writel_relaxed(val, vfe->base + VFE_0_DEMUX_R_GAIN_1);
		writel_relaxed(VFE_0_DEMUX_EVEN_CFG_PATTERN_GRBG,
			       vfe->base + VFE_0_DEMUX_EVEN_CFG);
		writel_relaxed(VFE_0_DEMUX_ODD_CFG_PATTERN_GRBG,
			       vfe->base + VFE_0_DEMUX_ODD_CFG);

		/* The rest of the Bayer pipeline follows the demux */
		vfe_set_bayer_cfg(vfe, line);
		return;
	}

	writel_relaxed(VFE_0_DEMUX_CFG_PERIOD, vfe->base + VFE_0_DEMUX_CFG);

	val = VFE_0_DEMUX_GAIN_0_CH0_EVEN | VFE_0_DEMUX_GAIN_0_CH0_ODD;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_0);

	val = VFE_0_DEMUX_GAIN_1_CH1 | VFE_0_DEMUX_GAIN_1_CH2;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_1);

	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
		even_cfg = VFE_0_DEMUX_EVEN_CFG_PATTERN_YUYV;
		odd_cfg = VFE_0_DEMUX_ODD_CFG_PATTERN_YUYV;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
		even_cfg = VFE_0_DEMUX_EVEN_CFG_PATTERN_YVYU;
		odd_cfg = VFE_0_DEMUX_ODD_CFG_PATTERN_YVYU;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	default:
		even_cfg = VFE_0_DEMUX_EVEN_CFG_PATTERN_UYVY;
		odd_cfg = VFE_0_DEMUX_ODD_CFG_PATTERN_UYVY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
		even_cfg = VFE_0_DEMUX_EVEN_CFG_PATTERN_VYUY;
		odd_cfg = VFE_0_DEMUX_ODD_CFG_PATTERN_VYUY;
		break;
	}

	writel_relaxed(even_cfg, vfe->base + VFE_0_DEMUX_EVEN_CFG);
	writel_relaxed(odd_cfg, vfe->base + VFE_0_DEMUX_ODD_CFG);
}

static void vfe_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 p = line->video_out.active_fmt.fmt.pix_mp.pixelformat;
	u32 reg;
	u16 input, output;
	u8 interp_reso;
	u32 phase_mult;

	writel_relaxed(0x3, vfe->base + VFE_0_SCALE_ENC_Y_CFG);

	input = line->fmt[MSM_VFE_PAD_SINK].width;
	output = line->compose.width;
	reg = (output << 16) | input;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_Y_H_IMAGE_SIZE);

	interp_reso = vfe_calc_interp_reso(input, output);
	phase_mult = input * (1 << (13 + interp_reso)) / output;
	reg = (interp_reso << 20) | phase_mult;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_Y_H_PHASE);

	input = line->fmt[MSM_VFE_PAD_SINK].height;
	output = line->compose.height;
	reg = (output << 16) | input;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_Y_V_IMAGE_SIZE);

	interp_reso = vfe_calc_interp_reso(input, output);
	phase_mult = input * (1 << (13 + interp_reso)) / output;
	reg = (interp_reso << 20) | phase_mult;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_Y_V_PHASE);

	writel_relaxed(0x3, vfe->base + VFE_0_SCALE_ENC_CBCR_CFG);

	input = line->fmt[MSM_VFE_PAD_SINK].width;
	output = line->compose.width / 2;
	reg = (output << 16) | input;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_CBCR_H_IMAGE_SIZE);

	interp_reso = vfe_calc_interp_reso(input, output);
	phase_mult = input * (1 << (13 + interp_reso)) / output;
	reg = (interp_reso << 20) | phase_mult;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_CBCR_H_PHASE);

	input = line->fmt[MSM_VFE_PAD_SINK].height;
	output = line->compose.height;
	if (p == V4L2_PIX_FMT_NV12 || p == V4L2_PIX_FMT_NV21)
		output = line->compose.height / 2;
	reg = (output << 16) | input;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_CBCR_V_IMAGE_SIZE);

	interp_reso = vfe_calc_interp_reso(input, output);
	phase_mult = input * (1 << (13 + interp_reso)) / output;
	reg = (interp_reso << 20) | phase_mult;
	writel_relaxed(reg, vfe->base + VFE_0_SCALE_ENC_CBCR_V_PHASE);
}

static void vfe_set_crop_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 p = line->video_out.active_fmt.fmt.pix_mp.pixelformat;
	u32 reg;
	u16 first, last;

	first = line->crop.left;
	last = line->crop.left + line->crop.width - 1;
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_Y_WIDTH);

	first = line->crop.top;
	last = line->crop.top + line->crop.height - 1;
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_Y_HEIGHT);

	first = line->crop.left / 2;
	last = line->crop.left / 2 + line->crop.width / 2 - 1;
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_CBCR_WIDTH);

	first = line->crop.top;
	last = line->crop.top + line->crop.height - 1;
	if (p == V4L2_PIX_FMT_NV12 || p == V4L2_PIX_FMT_NV21) {
		first = line->crop.top / 2;
		last = line->crop.top / 2 + line->crop.height / 2 - 1;
	}
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_CBCR_HEIGHT);
}

static void vfe_set_clamp_cfg(struct vfe_device *vfe)
{
	u32 val = VFE_0_CLAMP_ENC_MAX_CFG_CH0 |
		VFE_0_CLAMP_ENC_MAX_CFG_CH1 |
		VFE_0_CLAMP_ENC_MAX_CFG_CH2;

	writel_relaxed(val, vfe->base + VFE_0_CLAMP_ENC_MAX_CFG);

	val = VFE_0_CLAMP_ENC_MIN_CFG_CH0 |
		VFE_0_CLAMP_ENC_MIN_CFG_CH1 |
		VFE_0_CLAMP_ENC_MIN_CFG_CH2;

	writel_relaxed(val, vfe->base + VFE_0_CLAMP_ENC_MIN_CFG);
}

static void vfe_set_qos(struct vfe_device *vfe)
{
	u32 val = VFE_0_BUS_BDG_QOS_CFG_0_CFG;
	u32 val7 = VFE_0_BUS_BDG_QOS_CFG_7_CFG;
	int ret;

	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_0);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_1);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_2);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_3);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_4);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_5);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_6);
	writel_relaxed(val7, vfe->base + VFE_0_BUS_BDG_QOS_CFG_7);

	/* SoC-specific VBIF settings */
	if (vfe->res->has_vbif) {
		ret = vfe_vbif_apply_settings(vfe);
		if (ret < 0) {
			dev_err_ratelimited(vfe->camss->dev,
					    "VFE: VBIF error %d\n",
					    ret);
		}
	}
}

static void vfe_set_ds(struct vfe_device *vfe)
{
	/* empty */
}

static void vfe_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable)
{
	u32 val = VFE_0_CGC_OVERRIDE_1_IMAGE_Mx_CGC_OVERRIDE(wm);

	if (enable)
		vfe_reg_set(vfe, VFE_0_CGC_OVERRIDE_1, val);
	else
		vfe_reg_clr(vfe, VFE_0_CGC_OVERRIDE_1, val);

	wmb();
}

static void vfe_set_camif_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	unsigned int pixel_samples;
	u32 val;

	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GRGRGR;
		break;
	case MEDIA_BUS_FMT_YUYV8_1X16:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	default:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY;
		break;
	}

	writel_relaxed(val, vfe->base + VFE_0_CORE_CFG);

	/* CAMIF counts samples: two per YUV pixel, one per Bayer pixel */
	pixel_samples = vfe_is_bayer(line) ? 1 : 2;

	val = line->fmt[MSM_VFE_PAD_SINK].width * pixel_samples;
	val |= line->fmt[MSM_VFE_PAD_SINK].height << 16;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_FRAME_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].width * pixel_samples - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].height - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);

	val = 0xffffffff;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);

	val = 0xffffffff;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_IRQ_SUBSAMPLE_PATTERN);

	val = VFE_0_RDI_CFG_x_MIPI_EN_BITS;
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(0), val);

	val = VFE_0_CAMIF_CFG_VFE_OUTPUT_EN;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_CFG);
}

static void vfe_set_camif_cmd(struct vfe_device *vfe, u8 enable)
{
	u32 cmd;

	cmd = VFE_0_CAMIF_CMD_CLEAR_CAMIF_STATUS | VFE_0_CAMIF_CMD_NO_CHANGE;
	writel_relaxed(cmd, vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	if (enable)
		cmd = VFE_0_CAMIF_CMD_ENABLE_FRAME_BOUNDARY;
	else
		cmd = VFE_0_CAMIF_CMD_DISABLE_FRAME_BOUNDARY;

	writel_relaxed(cmd, vfe->base + VFE_0_CAMIF_CMD);
}

static void vfe_set_module_cfg(struct vfe_device *vfe, u8 enable)
{
	u32 val = VFE_0_MODULE_CFG_DEMUX |
		  VFE_0_MODULE_CFG_CHROMA_UPSAMPLE |
		  VFE_0_MODULE_CFG_SCALE_ENC |
		  VFE_0_MODULE_CFG_CROP_ENC;

	if (enable)
		writel_relaxed(val, vfe->base + VFE_0_MODULE_CFG);
	else
		writel_relaxed(0x0, vfe->base + VFE_0_MODULE_CFG);
}

static int vfe_camif_wait_for_stop(struct vfe_device *vfe, struct device *dev)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(vfe->base + VFE_0_CAMIF_STATUS,
				 val,
				 (val & VFE_0_CAMIF_STATUS_HALT),
				 CAMIF_TIMEOUT_SLEEP_US,
				 CAMIF_TIMEOUT_ALL_US);
	if (ret < 0)
		dev_err(dev, "%s: camif stop timeout\n", __func__);

	return ret;
}

static void vfe_isr_read(struct vfe_device *vfe, u32 *value0, u32 *value1)
{
	*value0 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_0);
	*value1 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_1);

	writel_relaxed(*value0, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(*value1, vfe->base + VFE_0_IRQ_CLEAR_1);

	wmb();
	writel_relaxed(VFE_0_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_0_IRQ_CMD);
}

static void vfe_violation_read(struct vfe_device *vfe)
{
	u32 violation = readl_relaxed(vfe->base + VFE_0_VIOLATION_STATUS);

	pr_err_ratelimited("VFE: violation = 0x%08x\n", violation);
}

/*
 * vfe_isr - VFE module interrupt handler
 * @irq: Interrupt line
 * @dev: VFE device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t vfe_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 value0, value1;
	int i, j;

	vfe->res->hw_ops->isr_read(vfe, &value0, &value1);

	dev_dbg(vfe->camss->dev, "VFE: status0 = 0x%08x, status1 = 0x%08x\n",
		value0, value1);

	if (value0 & VFE_0_IRQ_STATUS_0_RESET_ACK)
		vfe->isr_ops.reset_ack(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_VIOLATION)
		vfe->res->hw_ops->violation_read(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK)
		vfe->isr_ops.halt_ack(vfe);

	for (i = VFE_LINE_RDI0; i <= VFE_LINE_PIX; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_line_n_REG_UPDATE(i))
			vfe->isr_ops.reg_update(vfe, i);

	if (value0 & VFE_0_IRQ_STATUS_0_CAMIF_SOF)
		vfe->isr_ops.sof(vfe, VFE_LINE_PIX);

	for (i = VFE_LINE_RDI0; i <= VFE_LINE_RDI2; i++)
		if (value1 & VFE_0_IRQ_STATUS_1_RDIn_SOF(i))
			vfe->isr_ops.sof(vfe, i);

	for (i = 0; i < MSM_VFE_COMPOSITE_IRQ_NUM; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(i)) {
			vfe->isr_ops.comp_done(vfe, i);
			for (j = 0; j < ARRAY_SIZE(vfe->wm_output_map); j++)
				if (vfe->wm_output_map[j] == VFE_LINE_PIX)
					value0 &= ~VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(j);
		}

	for (i = 0; i < MSM_VFE_IMAGE_MASTERS_NUM; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(i))
			vfe->isr_ops.wm_done(vfe, i);

	return IRQ_HANDLED;
}

/*
 * vfe_pm_domain_off - Disable power domains specific to this VFE.
 * @vfe: VFE Device
 */
static void vfe_4_1_pm_domain_off(struct vfe_device *vfe)
{
	if (!vfe->res->has_pd)
		return;

	vfe_pm_domain_off(vfe);
}

/*
 * vfe_pm_domain_on - Enable power domains specific to this VFE.
 * @vfe: VFE Device
 */
static int vfe_4_1_pm_domain_on(struct vfe_device *vfe)
{
	if (!vfe->res->has_pd)
		return 0;

	return vfe_pm_domain_on(vfe);
}

static const struct vfe_hw_ops_gen1 vfe_ops_gen1_4_1 = {
	.bus_connect_wm_to_rdi = vfe_bus_connect_wm_to_rdi,
	.bus_disconnect_wm_from_rdi = vfe_bus_disconnect_wm_from_rdi,
	.bus_enable_wr_if = vfe_bus_enable_wr_if,
	.bus_reload_wm = vfe_bus_reload_wm,
	.camif_wait_for_stop = vfe_camif_wait_for_stop,
	.enable_irq_common = vfe_enable_irq_common,
	.enable_irq_pix_line = vfe_enable_irq_pix_line,
	.enable_irq_wm_line = vfe_enable_irq_wm_line,
	.get_ub_size = vfe_get_ub_size,
	.halt_clear = vfe_halt_clear,
	.halt_request = vfe_halt_request,
	.set_camif_cfg = vfe_set_camif_cfg,
	.set_camif_cmd = vfe_set_camif_cmd,
	.set_cgc_override = vfe_set_cgc_override,
	.set_clamp_cfg = vfe_set_clamp_cfg,
	.set_crop_cfg = vfe_set_crop_cfg,
	.set_demux_cfg = vfe_set_demux_cfg,
	.set_ds = vfe_set_ds,
	.set_module_cfg = vfe_set_module_cfg,
	.set_qos = vfe_set_qos,
	.set_rdi_cid = vfe_set_rdi_cid,
	.set_realign_cfg = vfe_set_realign_cfg,
	.set_scale_cfg = vfe_set_scale_cfg,
	.set_xbar_cfg = vfe_set_xbar_cfg,
	.wm_enable = vfe_wm_enable,
	.wm_frame_based = vfe_wm_frame_based,
	.wm_get_ping_pong_status = vfe_wm_get_ping_pong_status,
	.wm_line_based = vfe_wm_line_based,
	.wm_set_framedrop_pattern = vfe_wm_set_framedrop_pattern,
	.wm_set_framedrop_period = vfe_wm_set_framedrop_period,
	.wm_set_ping_addr = vfe_wm_set_ping_addr,
	.wm_set_pong_addr = vfe_wm_set_pong_addr,
	.wm_set_subsample = vfe_wm_set_subsample,
	.wm_set_ub_cfg = vfe_wm_set_ub_cfg,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->isr_ops = vfe_isr_ops_gen1;
	vfe->ops_gen1 = &vfe_ops_gen1_4_1;
	vfe->video_ops = vfe_video_ops_gen1;
}

const struct vfe_hw_ops vfe_ops_4_1 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr_read = vfe_isr_read,
	.isr = vfe_isr,
	.pm_domain_off = vfe_4_1_pm_domain_off,
	.pm_domain_on = vfe_4_1_pm_domain_on,
	.reg_update_clear = vfe_reg_update_clear,
	.reg_update = vfe_reg_update,
	.set_wb_gains = vfe_set_wb_gains,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_gen1_disable,
	.vfe_enable = vfe_gen1_enable,
	.vfe_halt = vfe_gen1_halt,
	.violation_read = vfe_violation_read,
};
