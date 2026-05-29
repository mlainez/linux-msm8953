// SPDX-License-Identifier: GPL-2.0
/* adspmem - memremap the ADSP reserved region and scan for firmware log
 * markers. /dev/mem is blocked by STRICT_DEVMEM; kernel memremap is not.
 * Counts occurrences of marker strings: a no-format-arg log string appearing
 * >1 time means it was actually LOGGED at runtime (rodata copy = 1).
 *
 * !! WARNING (verified 2026-05-29): loading this REBOOTS the device. The ADSP
 * reserved region (0x8d600000) is xPU/access-control protected WHILE THE ADSP
 * IS RUNNING — the AP faults reading it. Do NOT load against a running ADSP.
 * To read ADSP RAM, use a remoteproc coredump path instead (stop/crash + dump).
 * Kept as a research artifact documenting that ADSP state is HW-protected. */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>

#define ADSP_PA   0x8d600000UL
#define ADSP_SZ   0x01100000UL

static const char *marks[] = {
	"Skipping transmission of empty reconfiguration",
	"Processing reconfiguration",
	"BAM PA to VA mapping failed",
	"bam_drv_init is not called",
	"does not exist in target config",
	"Device Config failed",
	"Failed to open stream ports",
	"Failed to open data channels",
};

static int count_sub(const char *hay, size_t n, const char *needle)
{
	size_t nl = strlen(needle), i;
	int c = 0;

	if (nl == 0 || n < nl)
		return 0;
	for (i = 0; i + nl <= n; i++) {
		if (hay[i] == needle[0] && !memcmp(hay + i, needle, nl)) {
			c++;
			i += nl - 1;
		}
	}
	return c;
}

static int __init adspmem_init(void)
{
	void *p;
	int i;

	p = memremap(ADSP_PA, ADSP_SZ, MEMREMAP_WB);
	if (!p) {
		pr_err("adspmem: memremap failed\n");
		return -ENOMEM;
	}
	pr_info("adspmem: mapped ADSP 0x%lx +0x%lx\n", ADSP_PA, ADSP_SZ);
	for (i = 0; i < ARRAY_SIZE(marks); i++) {
		int c = count_sub((const char *)p, ADSP_SZ, marks[i]);

		pr_info("adspmem: [%d] \"%s\"\n", c, marks[i]);
	}
	memunmap(p);
	return -EAGAIN; /* don't stay loaded; we only needed the scan */
}
module_init(adspmem_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADSP reserved-RAM log-marker scanner");
