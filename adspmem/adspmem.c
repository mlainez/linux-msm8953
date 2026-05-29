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
		/* print the FULL format string (decompressed rodata in the
		 * dump) so the %d arg positions can be interpreted */
		{
			char fmt[160]; size_t k;
			for (k = 0; k < sizeof(fmt) - 1 && p[off + k] &&
				    p[off + k] >= 0x20 && p[off + k] < 0x7f; k++)
				fmt[k] = p[off + k];
			fmt[k] = '\0';
			pr_info("adspmem: FMT@0x%08x: \"%s\"\n", v, fmt);
		}
		for (i = 0; i + 9 < nw && hits < 8; i++) {
			if (w[i] == v) {
				/* a log record: word[0]=fmt, word[1..]=args.
				 * print 8 args, signed+unsigned, to read negative
				 * slot counts and the full satellite-define fields */
				pr_info("adspmem:   rec@0x%lx args: %d %d %d %d %d %d %d %d\n",
					(unsigned long)(i * 4),
					(int)w[i+1], (int)w[i+2], (int)w[i+3], (int)w[i+4],
					(int)w[i+5], (int)w[i+6], (int)w[i+7], (int)w[i+8]);
				hits++;
			}
		}
		if (!hits)
			pr_info("adspmem:   (no log records reference this fmt)\n");
	}

	/* Hunt the SLIM controller dev struct via its device_table @ dev+2408
	 * (16 words, codec entries at idx7=la199 / idx8=la200). Heap objects
	 * appear to be in the same linear map as the image (VA=0xf0100000+off),
	 * so a heap pointer is a word in [0xf0100000, 0xf1200000). A device_table
	 * = 16 consecutive words, idx7 AND idx8 pointer-like, >=10 of 16 zero. */
#define HEAP_LO 0xf0100000u
#define HEAP_HI 0xf1200000u
#define ISPTR(x) ((x) >= HEAP_LO && (x) < HEAP_HI)
	{
		int cand = 0;
		for (i = 0; i + 16 < nw && cand < 12; i++) {
			int z = 0, j;
			u32 e8, e7;
			if (!ISPTR(w[i+7]) || !ISPTR(w[i+8]))
				continue;
			for (j = 0; j < 16; j++)
				if (w[i+j] == 0)
					z++;
			if (z < 10)
				continue;
			/* candidate device_table at word i => dev = i*4 - 2408 */
			e7 = w[i+7]; e8 = w[i+8];
			pr_info("adspmem: DEVTAB? tbl@0x%lx (dev@0x%lx) zeros=%d idx7=0x%08x idx8=0x%08x\n",
				(unsigned long)(i*4), (unsigned long)(i*4 - 2408), z, e7, e8);
			/* dev+88 flag */
			{
				long devoff = (long)(i*4) - 2408;
				if (devoff >= 0)
					pr_info("adspmem:   dev+88=0x%02x\n", p[devoff + 88]);
			}
			/* codec PGD entry (idx8) +296 (sat-ctrl) and +300 (gate) */
			{
				long eoff = (long)e8 - 0xf0100000;
				if (eoff >= 0 && eoff + 304 < (long)ADSP_SZ)
					pr_info("adspmem:   entry8@0x%lx +296=0x%08x +300=0x%08x\n",
						(unsigned long)eoff,
						*(const u32 *)(p + eoff + 296),
						*(const u32 *)(p + eoff + 300));
			}
			cand++;
		}
		if (!cand)
			pr_info("adspmem: no device_table candidate found (entries may be NULL => dev+88=0?)\n");
	}
	/* FULL log-ring dump: walk the ring region and resolve EVERY fmt-ptr
	 * (a word in the rodata range [0xf0710000,0xf0735000]) to its string +
	 * a few args. Reveals ALL ADSP diagnostics during the capture, incl.
	 * error/warn logs we aren't explicitly targeting. */
#define ROD_LO 0xf0710000u
#define ROD_HI 0xf0735000u
	{
		size_t lo = 0x940000/4, hi = 0x968000/4, n = 0;
		pr_info("adspmem: === FULL RING DUMP (0x940000-0x968000) ===\n");
		for (i = lo; i < hi && i + 5 < nw && n < 220; i++) {
			u32 fp = w[i];
			long fo;
			char s[120]; size_t k;
			if (fp < ROD_LO || fp >= ROD_HI)
				continue;
			fo = (long)fp - 0xf0100000;
			if (fo < 0 || fo + 4 >= (long)ADSP_SZ)
				continue;
			/* require it to look like a printable format string */
			if (p[fo] != '[' && (p[fo] < 0x20 || p[fo] >= 0x7f))
				continue;
			for (k = 0; k < sizeof(s) - 1 && p[fo + k] &&
				    p[fo + k] >= 0x20 && p[fo + k] < 0x7f; k++)
				s[k] = p[fo + k];
			s[k] = '\0';
			if (k < 6)
				continue;
			pr_info("adspmem: R@%06lx \"%s\" | %d %d %d %d\n",
				(unsigned long)(i*4), s,
				(int)w[i+1], (int)w[i+2], (int)w[i+3], (int)w[i+4]);
			n++;
		}
		pr_info("adspmem: === END RING DUMP (%zu records) ===\n", n);
	}
	memunmap(p);
	return -EAGAIN;
}
module_init(adspmem_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADSP log-record arg decoder");
