// SPDX-License-Identifier: GPL-2.0
/* adspmem - read the ADSP firmware log ring (post `echo stop`) and DECODE the
 * args of key records to see the actual scheduled-bandwidth / channel params.
 * QDSP6 log record = [fmt_ptr][arg0][arg1]...; fmt_ptr is a runtime addr into
 * adsp.b04 (vaddr 0xf015f000, dump off 0x5f000), so for a string found at dump
 * offset O in that segment, its runtime addr V = 0xf0100000 + O. We find each
 * target format string, compute V, scan the region for records word[0]==V and
 * print the following arg words. */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>

#define ADSP_PA 0x8d600000UL
#define ADSP_SZ 0x01100000UL
#define B04_OFF_LO 0x5f000UL		/* adsp.b04 rodata window in dump */
#define B04_OFF_HI 0x6fc000UL
#define V_OF(O)   (0xf0100000u + (u32)(O))	/* dump off -> runtime addr (b04) */

struct tgt { const char *s; int nargs; };
/* records point to the string START incl the "[INFO] " level prefix, so match
 * the full prefixed string */
static const struct tgt tgts[] = {
	{ "[INFO] Bandwidth utilization stats on primary line", 4 },
	{ "[INFO] Active channel parameters", 5 },
	{ "[INFO] Channel %d was assigned data line %d", 3 },
	{ "[INFO] Got satellite define channel request", 6 },
	{ "[INFO] Processing reconfiguration sequence", 1 },
};

/* find first dump offset of needle within the b04 rodata window */
static long find_str(const u8 *p, const char *needle)
{
	size_t nl = strlen(needle), i;

	for (i = B04_OFF_LO; i + nl <= B04_OFF_HI; i++)
		if (p[i] == needle[0] && !memcmp(p + i, needle, nl))
			return (long)i;
	return -1;
}

static int __init adspmem_init(void)
{
	const u8 *p;
	const u32 *w;
	size_t nw, i;
	int t;

	p = memremap(ADSP_PA, ADSP_SZ, MEMREMAP_WB);
	if (!p) { pr_err("adspmem: memremap failed\n"); return -ENOMEM; }
	pr_info("adspmem: decoding log-record args\n");
	w = (const u32 *)p;
	nw = ADSP_SZ / 4;

	for (t = 0; t < ARRAY_SIZE(tgts); t++) {
		long off = find_str(p, tgts[t].s);
		u32 v;
		int hits = 0;

		if (off < 0) {
			pr_info("adspmem: fmt not found: \"%s\"\n", tgts[t].s);
			continue;
		}
		v = V_OF(off);
		pr_info("adspmem: \"%s\" fmt@0x%08x (off 0x%lx)\n",
			tgts[t].s, v, (unsigned long)off);
		for (i = 0; i + 6 < nw && hits < 8; i++) {
			if (w[i] == v) {
				/* a log record: word[0]=fmt, word[1..]=args */
				pr_info("adspmem:   rec@0x%lx args: %u %u %u %u %u %u\n",
					(unsigned long)(i * 4),
					w[i+1], w[i+2], w[i+3],
					w[i+4], w[i+5], w[i+6]);
				hits++;
			}
		}
		if (!hits)
			pr_info("adspmem:   (no log records reference this fmt)\n");
	}
	memunmap(p);
	return -EAGAIN;
}
module_init(adspmem_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADSP log-record arg decoder");
