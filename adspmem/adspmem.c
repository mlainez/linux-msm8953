// SPDX-License-Identifier: GPL-2.0
/* adspmem - reconstruct the ADSP firmware log from the reclaimed reserved
 * region (read AFTER `echo stop > remoteproc2/state`; reading while running
 * reboots the AP). The QDSP6 log ring stores records [fmt_ptr][args...] where
 * fmt_ptr is a runtime addr into the code+rodata segment. We resolve every
 * word that points to a printable rodata string and print it — that IS the
 * ADSP's runtime log. Addr map: dump_off(V)=seg.paddr+(V-seg.vaddr)-0x8a200000.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ctype.h>

#define ADSP_PA   0x8d600000UL
#define ADSP_SZ   0x01100000UL
#define G_CTRL_TABLE 0xf0c854d0u

/* rodata/code segment that holds log format strings */
#define ROD_LO 0xf015f000u
#define ROD_HI 0xf07fc000u

struct seg { u32 vaddr, paddr, memsz; };
static const struct seg segs[] = {
	{ 0xf0100000, 0x8a200000, 0x02000 }, { 0xf0102000, 0x8a202000, 0x5d000 },
	{ 0xf015f000, 0x8a25f000, 0x69d000 }, { 0xf07fc000, 0x8a8fc000, 0x4b1000 },
	{ 0xf0cad000, 0x8adad000, 0x01000 }, { 0xf0cae000, 0x8adae000, 0x68000 },
	{ 0xf0000000, 0x8ae16000, 0x14000 }, { 0xf0014000, 0x8ae2a000, 0x08000 },
};
static long v2off(u32 v)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(segs); i++)
		if (v >= segs[i].vaddr && v < segs[i].vaddr + segs[i].memsz)
			return (long)segs[i].paddr + (long)(v - segs[i].vaddr) - 0x8a200000L;
	return -1;
}

/* resolve a runtime rodata ptr to its string; return len of printable run */
static int resolve_str(const u8 *base, u32 v, char *out, int max)
{
	long off = v2off(v);
	int i = 0;

	if (off < 0 || off >= (long)ADSP_SZ)
		return 0;
	while (i < max - 1 && off + i < (long)ADSP_SZ) {
		char c = base[off + i];
		if (c == 0)
			break;
		if (!isprint((unsigned char)c))
			return 0;	/* not a clean string -> probably not a fmt ptr */
		out[i++] = c;
	}
	out[i] = 0;
	return i;
}

static int __init adspmem_init(void)
{
	const u8 *p;
	const u32 *w;
	size_t nw, i;
	char s[128];
	int printed = 0;
	bool junk;
	u32 t;

	p = memremap(ADSP_PA, ADSP_SZ, MEMREMAP_WB);
	if (!p) { pr_err("adspmem: memremap failed\n"); return -ENOMEM; }
	pr_info("adspmem: mapped; reconstructing ADSP log ring\n");

	/* controller table entries 0..4 (find the non-NULL audio ctrl) */
	for (i = 0; i < 5; i++) {
		long o = v2off(G_CTRL_TABLE + i * 4);
		t = (o >= 0) ? *(const u32 *)(p + o) : 0;
		pr_info("adspmem: g_slim_ctrl_table[%zu]=0x%08x\n", i, t);
	}

	/* Walk the log-ring region (fmt ptrs clustered ~0x9b5000); resolve every
	 * rodata-pointing word to its string -> the runtime log, in memory order.
	 * Scan a generous window around the observed cluster. */
	w = (const u32 *)(p + 0x9b0000);
	nw = 0x10000 / 4;	/* 64 KB window */
	for (i = 0; i < nw && printed < 400; i++) {
		u32 v = w[i];

		if (v >= ROD_LO && v < ROD_HI) {
			int l = resolve_str(p, v, s, sizeof(s));
			/* log fmt strings start with '[' (level) or a filename */
			if (l >= 8 && (s[0] == '[' || strchr(s, ':'))) {
				pr_info("adspmem|%05lx| %s\n",
					(unsigned long)(0x9b0000 + i * 4), s);
				printed++;
			}
		}
	}
	pr_info("adspmem: done (%d log lines)\n", printed);
	(void)junk;
	memunmap(p);
	return -EAGAIN;
}
module_init(adspmem_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADSP firmware log-ring reconstructor");
