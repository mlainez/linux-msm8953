/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * tas2557.h - Texas Instruments TAS2557 Mono Audio Amplifier
 *
 * Register definitions for the TAS2557 book/page architecture.
 * Based on TI downstream driver (tas2557sw-android).
 */

#ifndef __TAS2557_H__
#define __TAS2557_H__

/*
 * The TAS2557 uses a book/page/register hierarchy:
 *   - Register 0 on any page = page select
 *   - Register 127 on page 0 of any book = book select
 *   - Each page has 128 registers (0-127)
 *   - Virtual address: TAS2557_REG(book, page, reg)
 */
#define TAS2557_REG(book, page, reg) \
	(((book) * 256 * 128) + ((page) * 128) + (reg))

#define TAS2557_PAGE_REG 0x00 /* Page select (any page) */
#define TAS2557_BOOK_REG 0x7f /* Book select (page 0, reg 127) */

/* Book 0, Page 0 registers */
#define TAS2557_SW_RESET_REG TAS2557_REG(0, 0, 1)
#define TAS2557_REV_PGID_REG TAS2557_REG(0, 0, 3)
#define TAS2557_POWER_CTRL1_REG TAS2557_REG(0, 0, 4)
#define TAS2557_POWER_CTRL2_REG TAS2557_REG(0, 0, 5)
#define TAS2557_SPK_CTRL_REG TAS2557_REG(0, 0, 6)
#define TAS2557_MUTE_REG TAS2557_REG(0, 0, 7)
#define TAS2557_CHANNEL_CTRL_REG TAS2557_REG(0, 0, 30)
#define TAS2557_SAFE_GUARD_REG TAS2557_REG(0, 0, 37)
#define TAS2557_CLK_ERR1_REG TAS2557_REG(0, 0, 44)
#define TAS2557_CLK_ERR2_REG TAS2557_REG(0, 0, 45)
#define TAS2557_CLK_ERR3_REG TAS2557_REG(0, 0, 46)

/* Book 0, Page 0: diagnostic / status registers */
#define TAS2557_DEBUG_1_REG		TAS2557_REG(0, 0, 53)  /* 0x35 — DPU_FRC */
#define TAS2557_POWER_UP_FLAG_REG	TAS2557_REG(0, 0, 100) /* 0x64 — power status */
#define TAS2557_FLAGS_1_REG		TAS2557_REG(0, 0, 104) /* 0x68 — INT_DET_1 sticky */
#define TAS2557_FLAGS_2_REG		TAS2557_REG(0, 0, 108) /* 0x6C — INT_DET_2 sticky */
#define TAS2557_LOW_POWER_REG		TAS2557_REG(0, 0, 121) /* 0x79 — VBAT_POR */

/* Book 0, Page 2 registers (boost/sleep config, from stereo driver) */
#define TAS2557_SLEEPMODE_CTL_REG TAS2557_REG(0, 2, 7)

/* Book 100 (0x64), Page 0: VBoost control (from stereo driver) */
#define TAS2557_VBOOST_CTL_REG TAS2557_REG(100, 0, 64)

/* Book 0, Page 0: I2S config */
#define TAS2557_ASI_CFG1_REG TAS2557_REG(0, 0, 42)
#define TAS2557_ASI_CFG2_REG TAS2557_REG(0, 0, 43)

/* Book 0, Page 1 registers (pin/GPIO config) */
#define TAS2557_ASI1_DAC_FORMAT_REG TAS2557_REG(0, 1, 1)
#define TAS2557_GPIO1_PIN_REG TAS2557_REG(0, 1, 61)
#define TAS2557_GPIO2_PIN_REG TAS2557_REG(0, 1, 62)
#define TAS2557_GPI_PIN_REG TAS2557_REG(0, 1, 77)
#define TAS2557_PIN_CTRL1_REG TAS2557_REG(0, 1, 62)
#define TAS2557_PIN_CTRL2_REG TAS2557_REG(0, 1, 63)
#define TAS2557_GPIO4_PIN_REG TAS2557_REG(0, 1, 64)
#define TAS2557_GPIO_HIZ_CTRL2_REG TAS2557_REG(0, 1, 80)
#define TAS2557_CLK_HALT_REG TAS2557_REG(0, 1, 106)
#define TAS2557_INT_GEN1_REG TAS2557_REG(0, 1, 108)
#define TAS2557_INT_GEN2_REG TAS2557_REG(0, 1, 109)
#define TAS2557_INT_GEN3_REG TAS2557_REG(0, 1, 110)
#define TAS2557_INT_GEN4_REG TAS2557_REG(0, 1, 111)
#define TAS2557_INT_MODE_REG TAS2557_REG(0, 1, 114)

/* Book 0, Page 0: SAR ADC (downstream: B0_P0_R20/R21) */
#define TAS2557_SAR_ADC1_REG TAS2557_REG(0, 0, 20)
#define TAS2557_SAR_ADC2_REG TAS2557_REG(0, 0, 21)

/* Book 0, Page 253: test mode / broadcast */
#define TAS2557_TEST_MODE_REG TAS2557_REG(0, 253, 13)
#define TAS2557_BROADCAST_REG TAS2557_REG(0, 253, 54)

/* Book 100 (0x64), Page 0: DSP soft mute */
#define TAS2557_DSP_MUTE_REG TAS2557_REG(0x64, 0, 7)

/* REV_PGID values */
#define TAS2557_PG_1P0 0x80
#define TAS2557_PG_2P0 0x90
#define TAS2557_PG_2P1 0xa0

/* SW_RESET */
#define TAS2557_SW_RESET BIT(0)

/* SAFE_GUARD pattern */
#define TAS2557_SAFE_GUARD_PATTERN 0x5a

/* Firmware */
#define TAS2557_FW_NAME "tas2557_uCDSP.bin"
#define TAS2557_FW_MAGIC 0x35353532
#define TAS2557_DEVICE_MONO 2
#define TAS2557_DEVICE_FAMILY 0

/* Firmware block types */
#define TAS2557_BLOCK_PLL 0x00
#define TAS2557_BLOCK_BASE_MAIN 0x01
#define TAS2557_BLOCK_CONF_COEFF 0x03
#define TAS2557_BLOCK_CONF_PRE 0x04
#define TAS2557_BLOCK_CONF_POST 0x05
#define TAS2557_BLOCK_CONF_POST_POWER 0x06
#define TAS2557_BLOCK_CONF_CAL 0x0a
#define TAS2557_BLOCK_PGM_ALL 0x0d

#endif /* __TAS2557_H__ */
