// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017, Linaro Ltd.

#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/slimbus.h>
#include <linux/module.h>

#include "internal.h"

/*
 * WCD9335/WCD934x codecs behind Qualcomm NGD controllers can NACK
 * value-element accesses, e.g. the first read after an idle period or
 * during codec init. As in the vendor driver, retry up to 3 times with a
 * 5 ms gap.
 */
#define REGMAP_SLIMBUS_MAX_TRIES	3
#define REGMAP_SLIMBUS_PAGED_TRIES	3
#define slim_io_retry_us		5000

static int regmap_slimbus_write(void *context, const void *data, size_t count)
{
	struct slim_device *sdev = context;
	int tries = REGMAP_SLIMBUS_MAX_TRIES;
	u16 addr = *(u16 *)data;
	int ret;

	do {
		ret = slim_write(sdev, addr, count - 2, (u8 *)data + 2);
		if (!ret || --tries == 0)
			break;
		usleep_range(slim_io_retry_us, slim_io_retry_us + 100);
	} while (1);

	return ret;
}

static int regmap_slimbus_read(void *context, const void *reg, size_t reg_size,
			       void *val, size_t val_size)
{
	struct slim_device *sdev = context;
	int tries = REGMAP_SLIMBUS_MAX_TRIES;
	u16 addr = *(u16 *)reg;
	int ret;

	do {
		ret = slim_read(sdev, addr, val_size, val);
		if (!ret || --tries == 0)
			break;
		usleep_range(slim_io_retry_us, slim_io_retry_us + 100);
	} while (1);

	return ret;
}

static const struct regmap_bus regmap_slimbus_bus = {
	.write = regmap_slimbus_write,
	.read = regmap_slimbus_read,
	.reg_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

/*
 * Paged register bus for WCD9335 and similar Qualcomm SLIMbus codecs.
 *
 * The codec exposes a paged register window in the value-element space:
 *   - VE 0x800:        page selector (write-only)
 *   - VE 0x801..0x8ff: registers 0x01..0xff of the selected page
 *
 * Register addresses are (page << 8) | offset. The selector cannot be read,
 * so regmap_range_cfg (which read-modify-writes it) cannot be used.
 *
 * The selector is written before every access rather than cached: once
 * the ADSP has been given AFE_PARAM_ID_CDC_REG_PAGE_CFG it programs codec
 * registers itself when it starts a SLIMbus port, moving the selector
 * behind our back. A cached page then sends the next access to whatever
 * page the ADSP left selected, while regcache records it as written.
 */
struct regmap_slimbus_paged_ctx {
	struct slim_device *sdev;
	struct mutex page_lock;
};

/* Offset 0 of every page is the selector itself. */
#define REGMAP_SLIMBUS_WCD_BASE		0x0800
#define REGMAP_SLIMBUS_WCD_SELECTOR	0x0800

static int regmap_slimbus_select_page(struct regmap_slimbus_paged_ctx *ctx,
				      u8 page)
{
	struct slim_device *sdev = ctx->sdev;
	int tries = REGMAP_SLIMBUS_PAGED_TRIES;
	int ret;

	do {
		ret = slim_write(sdev, REGMAP_SLIMBUS_WCD_SELECTOR, 1, &page);
		if (!ret || --tries == 0)
			break;
		usleep_range(slim_io_retry_us, slim_io_retry_us + 100);
	} while (1);

	return ret;
}

static int regmap_slimbus_paged_write(void *context, const void *data,
				      size_t count)
{
	struct regmap_slimbus_paged_ctx *ctx = context;
	struct slim_device *sdev = ctx->sdev;
	int tries = REGMAP_SLIMBUS_PAGED_TRIES;
	u16 vaddr = *(u16 *)data;
	u8 page = (vaddr >> 8) & 0xff;
	u8 offset = vaddr & 0xff;
	u16 hw_addr;
	int ret;

	mutex_lock(&ctx->page_lock);

	ret = regmap_slimbus_select_page(ctx, page);
	if (ret) {
		mutex_unlock(&ctx->page_lock);
		return ret;
	}

	hw_addr = REGMAP_SLIMBUS_WCD_BASE + offset;
	do {
		ret = slim_write(sdev, hw_addr, count - 2, (u8 *)data + 2);
		if (!ret || --tries == 0)
			break;
		usleep_range(slim_io_retry_us, slim_io_retry_us + 100);
	} while (1);

	/*
	 * A write can still miss its register although slim_write() succeeds:
	 * the ADSP may move the selector between our selector write and the
	 * data write while it starts a SLIMbus port, which is exactly when
	 * DAPM enables the mic bias, the EAR PA and the path clocks. Read
	 * single-register writes back and, on mismatch, re-select the page and
	 * write again. Page 0x00 (interrupt status/clear, efuse sensing) and
	 * page 0x0a below the TX decimators hold write-to-clear and
	 * self-clearing registers, so they are not verified.
	 */
	if (!ret && (count - 2) == 1 && page != 0x00 &&
	    !(page == 0x0a && offset < 0x31)) {
		u8 want = ((u8 *)data)[2];
		u8 got = 0;
		int vtries = 3;

		while (vtries--) {
			if (slim_read(sdev, hw_addr, 1, &got) == 0 && got == want)
				break;
			if (regmap_slimbus_select_page(ctx, page))
				break;
			slim_write(sdev, hw_addr, 1, (u8 *)data + 2);
		}
		/*
		 * Not an error: some registers (e.g. 0x0aa7) reject the write,
		 * and the codec changes clock-enable bits on its own, while it
		 * keeps working. Failing here would make ASoC abort the DAPM
		 * update.
		 */
		if (got != want)
			dev_dbg(&sdev->dev,
				"paged_write verify failed vaddr=0x%03x want=0x%02x got=0x%02x (non-fatal)\n",
				vaddr, want, got);
	}

	mutex_unlock(&ctx->page_lock);

	return ret;
}

static int regmap_slimbus_paged_read(void *context, const void *reg,
				     size_t reg_size, void *val,
				     size_t val_size)
{
	struct regmap_slimbus_paged_ctx *ctx = context;
	struct slim_device *sdev = ctx->sdev;
	int tries = REGMAP_SLIMBUS_PAGED_TRIES;
	u16 vaddr = *(u16 *)reg;
	u8 page = (vaddr >> 8) & 0xff;
	u8 offset = vaddr & 0xff;
	u16 hw_addr;
	int ret;

	mutex_lock(&ctx->page_lock);

	ret = regmap_slimbus_select_page(ctx, page);
	if (ret) {
		mutex_unlock(&ctx->page_lock);
		return ret;
	}

	hw_addr = REGMAP_SLIMBUS_WCD_BASE + offset;
	do {
		ret = slim_read(sdev, hw_addr, val_size, val);
		if (!ret || --tries == 0)
			break;
		usleep_range(slim_io_retry_us, slim_io_retry_us + 100);
	} while (1);

	mutex_unlock(&ctx->page_lock);

	return ret;
}

static void regmap_slimbus_paged_free_ctx(void *context)
{
	kfree(context);
}

static const struct regmap_bus regmap_slimbus_paged_bus = {
	.write = regmap_slimbus_paged_write,
	.read = regmap_slimbus_paged_read,
	.free_context = regmap_slimbus_paged_free_ctx,
	.reg_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

struct regmap *__regmap_init_slimbus_paged(struct slim_device *slimbus,
					   const struct regmap_config *config,
					   struct lock_class_key *lock_key,
					   const char *lock_name)
{
	struct regmap_slimbus_paged_ctx *ctx;

	if (config->val_bits != 8 || config->reg_bits != 16)
		return ERR_PTR(-ENOTSUPP);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->sdev = slimbus;
	mutex_init(&ctx->page_lock);

	return __regmap_init(&slimbus->dev, &regmap_slimbus_paged_bus, ctx,
			     config, lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__regmap_init_slimbus_paged);

static const struct regmap_bus *regmap_get_slimbus(struct slim_device *slim,
					const struct regmap_config *config)
{
	if (config->val_bits == 8 && config->reg_bits == 16)
		return &regmap_slimbus_bus;

	return ERR_PTR(-ENOTSUPP);
}

struct regmap *__regmap_init_slimbus(struct slim_device *slimbus,
				     const struct regmap_config *config,
				     struct lock_class_key *lock_key,
				     const char *lock_name)
{
	const struct regmap_bus *bus = regmap_get_slimbus(slimbus, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus);

	return __regmap_init(&slimbus->dev, bus, slimbus, config, lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__regmap_init_slimbus);

struct regmap *__devm_regmap_init_slimbus(struct slim_device *slimbus,
					  const struct regmap_config *config,
					  struct lock_class_key *lock_key,
					  const char *lock_name)
{
	const struct regmap_bus *bus = regmap_get_slimbus(slimbus, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus);

	return __devm_regmap_init(&slimbus->dev, bus, slimbus, config, lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__devm_regmap_init_slimbus);

MODULE_DESCRIPTION("Register map access API - SLIMbus support");
MODULE_LICENSE("GPL v2");
