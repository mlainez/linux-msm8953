// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Qualcomm Audio Calibration driver for MSM8953.
 *
 * Provides /dev/msm_audio_cal for userspace acdb_loader to send
 * calibration data to the ADSP via Q6AFE/Q6ADM APR services.
 *
 * This is a stripped-down reimplementation of the downstream
 * techpack/audio/dsp/audio_calibration.c for mainline kernels.
 * Uses DMA-BUF heaps instead of ION for shared memory.
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>

/* Ioctl definitions matching downstream msm_audio_calibration.h */
#define CAL_IOCTL_MAGIC 'a'
#define AUDIO_ALLOCATE_CALIBRATION   _IOWR(CAL_IOCTL_MAGIC, 200, void *)
#define AUDIO_DEALLOCATE_CALIBRATION _IOWR(CAL_IOCTL_MAGIC, 201, void *)
#define AUDIO_PREPARE_CALIBRATION    _IOWR(CAL_IOCTL_MAGIC, 202, void *)
#define AUDIO_SET_CALIBRATION        _IOWR(CAL_IOCTL_MAGIC, 203, void *)
#define AUDIO_GET_CALIBRATION        _IOWR(CAL_IOCTL_MAGIC, 204, void *)
#define AUDIO_POST_CALIBRATION       _IOWR(CAL_IOCTL_MAGIC, 205, void *)

/* Cal type header — first field userspace sends */
struct audio_cal_header {
	int32_t  data_size;
	int32_t  version;
	int32_t  cal_type;
	int32_t  cal_type_size;
};

#define MAX_CAL_TYPES 64

struct cal_block {
	struct list_head list;
	int cal_type;
	void *cal_data;
	size_t cal_size;
	dma_addr_t cal_phys;
	int mem_handle;
};

static DEFINE_MUTEX(cal_lock);
static struct list_head cal_blocks = LIST_HEAD_INIT(cal_blocks);

/* Per cal_type ION mapping: ALLOCATE stores fd, SET uses it */
static int cal_type_ion_fd[MAX_CAL_TYPES];

/* Stored topology IDs from ACDB */
static int adm_topology; /* ADM_TOPOLOGY_CAL_TYPE = 9 */
static int afe_topology; /* AFE_TOPOLOGY_CAL_TYPE = 23 */
static int asm_topology; /* ASM_TOPOLOGY_CAL_TYPE = 13 */

/* ION shim forward declarations (full implementation below) */
struct ion_alloc_entry {
	int handle;
	void *vaddr;
	dma_addr_t paddr;
	size_t size;
	bool use_cma;
	struct list_head list;
};
static DEFINE_MUTEX(ion_lock);
static LIST_HEAD(ion_allocs);
static struct device *ion_cma_dev; /* ADSP-accessible CMA device */

static int msm_audio_cal_open(struct inode *inode, struct file *f)
{
	return 0;
}

static int msm_audio_cal_release(struct inode *inode, struct file *f)
{
	return 0;
}

static long msm_audio_cal_ioctl(struct file *f, unsigned int cmd,
				unsigned long arg)
{
	struct audio_cal_header hdr;
	void *data;
	int ret = 0;

	if (copy_from_user(&hdr, (void __user *)arg, sizeof(hdr)))
		return -EFAULT;

	if (hdr.data_size < sizeof(hdr) || hdr.data_size > 65536)
		return -EINVAL;

	pr_info("msm_audio_cal: ioctl cmd=%u cal_type=%d size=%d\n",
		_IOC_NR(cmd), hdr.cal_type, hdr.data_size);

	switch (cmd) {
	case AUDIO_ALLOCATE_CALIBRATION: {
		/*
		 * Userspace registers a cal block with an ION buffer.
		 * The full struct is: audio_cal_header + cal_type_header +
		 * cal_data { int32_t mem_handle; }.
		 * mem_handle is the ION shared fd from ION_IOC_SHARE.
		 * We need to store the mapping: mem_handle → ION alloc entry
		 * so that SET_CALIBRATION can find the buffer.
		 */
		struct {
			struct audio_cal_header hdr2;
			struct {
				int32_t buffer_number;
				int32_t version;
			} cal_hdr;
			struct {
				int32_t cal_size;
				int32_t mem_handle;
			} cal_data;
		} __packed alloc_data;

		if (hdr.data_size >= sizeof(hdr)) {
			/* Dump raw ALLOCATE data to understand the struct */
			u8 raw[64];
			int dump_sz = min((int)hdr.data_size, 64);

			if (!copy_from_user(raw, (void __user *)arg, dump_sz)) {
				u32 *w = (u32 *)raw;
				int ion_fd = (dump_sz >= 28) ? w[6] : 0;

				pr_info("msm_audio_cal: ALLOCATE cal_type=%d size=%d raw=[%08x %08x %08x %08x %08x %08x %08x %08x]\n",
					hdr.cal_type, dump_sz,
					w[0], w[1], w[2], w[3], w[4], w[5],
					(dump_sz >= 28) ? w[6] : 0,
					(dump_sz >= 32) ? w[7] : 0);

				/* Store ION fd for this cal_type so SET can find it */
				if (hdr.cal_type >= 0 && hdr.cal_type < MAX_CAL_TYPES)
					cal_type_ion_fd[hdr.cal_type] = ion_fd;

				/* Map the ION fd to our internal alloc entry.
				 * The ION fd was returned by ION_IOC_SHARE.
				 * private_data of that fd contains the handle.
				 */
				if (ion_fd > 0) {
					struct fd f = fdget(ion_fd);

					if (fd_file(f) && fd_file(f)->private_data) {
						int ion_handle = (int)(long)fd_file(f)->private_data;
						struct ion_alloc_entry *entry;

						mutex_lock(&ion_lock);
						list_for_each_entry(entry, &ion_allocs, list) {
							if (entry->handle == ion_handle) {
								/* Create alias: ion_fd → same entry */
								pr_info("msm_audio_cal: mapped ion_fd=%d → handle=%d phys=%pad\n",
									ion_fd, ion_handle, &entry->paddr);
								break;
							}
						}
						mutex_unlock(&ion_lock);
						fdput(f);
					}
				}
			}
		} else {
			pr_info("msm_audio_cal: ALLOCATE cal_type=%d (no mem_handle)\n",
				hdr.cal_type);
		}
		break;
	}

	case AUDIO_SET_CALIBRATION: {
		/*
		 * Userspace sends calibration metadata. The actual cal data
		 * is in the ION buffer referenced by mem_handle.
		 * Parse the header to get cal_type and mem_handle, then
		 * look up the ION allocation to find the data.
		 */
		struct {
			struct audio_cal_header hdr2;
			struct {
				int32_t buffer_number;
				int32_t version;
			} cal_hdr;
			struct {
				int32_t cal_size;
				int32_t mem_handle;
			} cal_data;
		} __packed set_data;

		if (hdr.data_size < sizeof(set_data)) {
			pr_warn("msm_audio_cal: SET too small %d\n", hdr.data_size);
			return -EINVAL;
		}

		if (copy_from_user(&set_data, (void __user *)arg, sizeof(set_data)))
			return -EFAULT;

		pr_info("msm_audio_cal: SET cal_type=%d mem_handle=%d buf=%d\n",
			hdr.cal_type, set_data.cal_data.mem_handle,
			set_data.cal_hdr.buffer_number);

		/* Extract topology IDs from topology cal types.
		 * The full struct has: audio_cal_header + cal_type_header +
		 * cal_data + cal_info (type-specific).
		 * For topology types, cal_info starts with int32_t topology.
		 */
		if (hdr.cal_type == 9 || hdr.cal_type == 13 || hdr.cal_type == 23) {
			/* Dump raw bytes to find topology offset */
			u8 raw[80];
			int dump_sz = min((int)hdr.data_size, 80);

			if (!copy_from_user(raw, (void __user *)arg, dump_sz)) {
				u32 *w = (u32 *)raw;

				pr_info("msm_audio_cal: cal_type=%d raw[%d]: "
					"[%08x %08x %08x %08x %08x %08x %08x %08x %08x %08x]\n",
					hdr.cal_type, dump_sz,
					w[0], w[1], w[2], w[3], w[4],
					w[5], w[6], w[7], w[8], w[9]);
				/* Try different offsets for topology:
				 * w[7] = offset 28, w[8] = offset 32 */
				/* Topology is at w[8] (offset 32) for all topo types */
				if (hdr.cal_type == 9) {
					adm_topology = w[8];
					pr_info("msm_audio_cal: ADM_TOPO = 0x%x\n", adm_topology);
				} else if (hdr.cal_type == 13) {
					asm_topology = w[8];
					pr_info("msm_audio_cal: ASM_TOPO = 0x%x\n", asm_topology);
				} else if (hdr.cal_type == 23) {
					afe_topology = w[8];
					pr_info("msm_audio_cal: AFE_TOPO = 0x%x\n", afe_topology);
				}
			}
		}

		/* Look up the ION allocation by mem_handle.
		 * mem_handle is the ION shared fd from ALLOCATE.
		 * Resolve: fd → file → private_data (ION handle) → alloc entry
		 */
		{
			struct ion_alloc_entry *entry;
			void *cal_buf = NULL;
			size_t cal_sz = 0;
			dma_addr_t cal_phys = 0;
			int ion_fd = set_data.cal_data.mem_handle;

			/* Try resolving as ION fd first */
			if (ion_fd > 0) {
				struct fd f = fdget(ion_fd);

				if (fd_file(f) && fd_file(f)->private_data) {
					int ion_handle = (int)(long)fd_file(f)->private_data;

					mutex_lock(&ion_lock);
					list_for_each_entry(entry, &ion_allocs, list) {
						if (entry->handle == ion_handle) {
							cal_buf = entry->vaddr;
							cal_sz = entry->size;
							cal_phys = entry->paddr;
							break;
						}
					}
					mutex_unlock(&ion_lock);
					fdput(f);
				}
			}

			/* Fallback 1: try as raw handle */
			if (!cal_buf) {
				mutex_lock(&ion_lock);
				list_for_each_entry(entry, &ion_allocs, list) {
					if (entry->handle == ion_fd) {
						cal_buf = entry->vaddr;
						cal_sz = entry->size;
						cal_phys = entry->paddr;
						break;
					}
				}
				mutex_unlock(&ion_lock);
			}

			/* Fallback 2: use the ION fd stored during ALLOCATE */
			if (!cal_buf && hdr.cal_type >= 0 &&
			    hdr.cal_type < MAX_CAL_TYPES &&
			    cal_type_ion_fd[hdr.cal_type] > 0) {
				int stored_fd = cal_type_ion_fd[hdr.cal_type];
				struct fd f2 = fdget(stored_fd);

				if (fd_file(f2) && fd_file(f2)->private_data) {
					int h = (int)(long)fd_file(f2)->private_data;

					mutex_lock(&ion_lock);
					list_for_each_entry(entry, &ion_allocs, list) {
						if (entry->handle == h) {
							cal_buf = entry->vaddr;
							cal_sz = entry->size;
							cal_phys = entry->paddr;
							pr_info("msm_audio_cal: resolved cal_type=%d via stored fd=%d → handle=%d\n",
								hdr.cal_type, stored_fd, h);
							break;
						}
					}
					mutex_unlock(&ion_lock);
					fdput(f2);
				}
			}

			if (cal_buf && cal_sz > 0) {
				/* Log first few bytes of cal data */
				u32 *p = (u32 *)cal_buf;

				pr_info("msm_audio_cal: cal_type=%d phys=%pad size=%zu "
					"first_words=[0x%08x 0x%08x 0x%08x 0x%08x]\n",
					hdr.cal_type, &cal_phys, cal_sz,
					p[0], p[1], p[2], p[3]);

				/* Store for later use by q6afe/q6adm */
				mutex_lock(&cal_lock);
				{
					struct cal_block *blk = kzalloc(sizeof(*blk), GFP_KERNEL);

					if (blk) {
						blk->cal_type = hdr.cal_type;
						blk->cal_data = cal_buf; /* point to ION buffer */
						blk->cal_size = cal_sz;
						blk->cal_phys = cal_phys;
						blk->mem_handle = set_data.cal_data.mem_handle;
						list_add_tail(&blk->list, &cal_blocks);
					}
				}
				mutex_unlock(&cal_lock);
			} else {
				pr_warn("msm_audio_cal: ION handle %d not found\n",
					set_data.cal_data.mem_handle);
			}
		}
		break;
	}

	case AUDIO_DEALLOCATE_CALIBRATION:
		pr_info("msm_audio_cal: DEALLOCATE cal_type=%d\n", hdr.cal_type);
		break;

	case AUDIO_PREPARE_CALIBRATION:
	case AUDIO_GET_CALIBRATION:
	case AUDIO_POST_CALIBRATION:
		/* Stub for now */
		break;

	default:
		pr_err("msm_audio_cal: unknown ioctl %u\n", cmd);
		ret = -EINVAL;
	}

	return ret;
}

static const struct file_operations msm_audio_cal_fops = {
	.owner = THIS_MODULE,
	.open = msm_audio_cal_open,
	.release = msm_audio_cal_release,
	.unlocked_ioctl = msm_audio_cal_ioctl,
	.compat_ioctl = msm_audio_cal_ioctl,
};

static struct miscdevice msm_audio_cal_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msm_audio_cal",
	.fops = &msm_audio_cal_fops,
};

/*
 * /dev/ion shim — the acdb_loader needs ION for shared memory.
 * Modern kernels don't have ION. This shim provides a minimal
 * /dev/ion that allocates DMA-coherent memory via dma_alloc_coherent
 * and returns an fd via dma_buf.
 */

/* ION ioctl definitions matching userspace expectations */
#define ION_IOC_MAGIC 'I'

struct ion_alloc_data_user {
	u64 len;
	u32 align;
	u32 heap_id_mask;
	u32 flags;
	u32 handle;
};

struct ion_fd_data_user {
	s32 handle;
	s32 fd;
};

struct ion_handle_data_user {
	s32 handle;
};

#define ION_IOC_ALLOC _IOWR(ION_IOC_MAGIC, 0, struct ion_alloc_data_user)
#define ION_IOC_FREE  _IOWR(ION_IOC_MAGIC, 1, struct ion_handle_data_user)
#define ION_IOC_MAP   _IOWR(ION_IOC_MAGIC, 2, struct ion_fd_data_user)
#define ION_IOC_SHARE _IOWR(ION_IOC_MAGIC, 4, struct ion_fd_data_user)
#define ION_IOC_IMPORT _IOWR(ION_IOC_MAGIC, 5, struct ion_fd_data_user)

static const struct file_operations ion_shim_fops;
static int ion_handle_counter = 1;
static struct device *ion_dev;

static int ion_shim_open(struct inode *inode, struct file *f)
{
	return 0;
}

static int ion_shim_release(struct inode *inode, struct file *f)
{
	return 0;
}

static long ion_shim_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case ION_IOC_ALLOC: {
		struct ion_alloc_data_user alloc;
		struct ion_alloc_entry *entry;

		if (copy_from_user(&alloc, (void __user *)arg, sizeof(alloc)))
			return -EFAULT;

		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if (!entry)
			return -ENOMEM;

		entry->size = PAGE_ALIGN(alloc.len);
		/*
		 * Allocate from the ADSP-accessible CMA region.
		 * The fastrpc device has the adsp_mem CMA pool attached.
		 * Find it and use its DMA allocator so the ADSP can read
		 * the cal data via SMMU.
		 */
		{
			struct device *cma_dev = NULL;

			/* Find the fastrpc device which has adsp_mem CMA */
			if (ion_cma_dev)
				cma_dev = ion_cma_dev;

			if (cma_dev) {
				entry->vaddr = dma_alloc_coherent(cma_dev,
					entry->size, &entry->paddr,
					GFP_KERNEL);
				entry->use_cma = true;
			}
		}
		if (!entry->vaddr) {
			/* Fallback to page allocator (not ADSP accessible) */
			entry->vaddr = (void *)__get_free_pages(
				GFP_KERNEL | __GFP_ZERO,
				get_order(entry->size));
			if (!entry->vaddr) {
				kfree(entry);
				return -ENOMEM;
			}
			entry->paddr = virt_to_phys(entry->vaddr);
			entry->use_cma = false;
		}

		mutex_lock(&ion_lock);
		entry->handle = ion_handle_counter++;
		list_add_tail(&entry->list, &ion_allocs);
		mutex_unlock(&ion_lock);

		alloc.handle = entry->handle;
		if (copy_to_user((void __user *)arg, &alloc, sizeof(alloc))) {
			dma_free_coherent(ion_dev, entry->size, entry->vaddr, entry->paddr);
			kfree(entry);
			return -EFAULT;
		}

		pr_info("ion_shim: alloc handle=%d size=%zu phys=%pad\n",
			entry->handle, entry->size, &entry->paddr);
		return 0;
	}

	case ION_IOC_MAP:
	case ION_IOC_IMPORT:
	case ION_IOC_SHARE: {
		struct ion_fd_data_user fd_data;
		/* For now, return a dummy fd - the kernel module handles
		 * the actual memory mapping via msm_audio_cal */
		if (copy_from_user(&fd_data, (void __user *)arg, sizeof(fd_data)))
			return -EFAULT;

		/* Return the handle as fd - the acdb_loader will mmap this.
		 * We can't easily create a real dma-buf fd here without more
		 * infrastructure. Return a dup of /dev/null as placeholder. */
		{
			struct file *filp;

			fd_data.fd = get_unused_fd_flags(O_CLOEXEC);
			if (fd_data.fd < 0)
				return fd_data.fd;

			/* Store handle in private_data so mmap knows which buffer */
			filp = anon_inode_getfile("ion_shim", &ion_shim_fops,
						  (void *)(long)fd_data.handle, O_RDWR);
			if (IS_ERR(filp)) {
				put_unused_fd(fd_data.fd);
				return PTR_ERR(filp);
			}
			fd_install(fd_data.fd, filp);
		}

		if (copy_to_user((void __user *)arg, &fd_data, sizeof(fd_data)))
			return -EFAULT;

		pr_info("ion_shim: share handle=%d fd=%d\n",
			fd_data.handle, fd_data.fd);
		return 0;
	}

	case ION_IOC_FREE: {
		struct ion_handle_data_user handle_data;
		struct ion_alloc_entry *entry, *tmp;

		if (copy_from_user(&handle_data, (void __user *)arg, sizeof(handle_data)))
			return -EFAULT;

		mutex_lock(&ion_lock);
		list_for_each_entry_safe(entry, tmp, &ion_allocs, list) {
			if (entry->handle == handle_data.handle) {
				list_del(&entry->list);
				free_pages((unsigned long)entry->vaddr,
				   get_order(entry->size));
				kfree(entry);
				break;
			}
		}
		mutex_unlock(&ion_lock);
		return 0;
	}

	default:
		pr_warn("ion_shim: unknown ioctl 0x%x\n", cmd);
		return -ENOTTY;
	}
}

static int ion_shim_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct ion_alloc_entry *entry;
	size_t size = vma->vm_end - vma->vm_start;
	int target_handle = (int)(long)f->private_data;

	mutex_lock(&ion_lock);
	list_for_each_entry(entry, &ion_allocs, list) {
		if (entry->handle == target_handle) {
			int ret;

			vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
			ret = remap_pfn_range(vma, vma->vm_start,
					      entry->paddr >> PAGE_SHIFT,
					      min(size, entry->size),
					      vma->vm_page_prot);
			mutex_unlock(&ion_lock);
			pr_info("ion_shim: mmap handle=%d size=%zu phys=%pad\n",
				target_handle, entry->size, &entry->paddr);
			return ret;
		}
	}
	mutex_unlock(&ion_lock);
	pr_warn("ion_shim: mmap handle=%d not found\n", target_handle);
	return -EINVAL;
}

static const struct file_operations ion_shim_fops = {
	.owner = THIS_MODULE,
	.open = ion_shim_open,
	.release = ion_shim_release,
	.unlocked_ioctl = ion_shim_ioctl,
	.compat_ioctl = ion_shim_ioctl,
	.mmap = ion_shim_mmap,
};

static struct miscdevice ion_shim_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ion",
	.fops = &ion_shim_fops,
};

/**
 * msm_audio_cal_get_topology - Get stored ACDB topology ID
 * @type: 0=ADM, 1=AFE, 2=ASM
 * Returns: topology ID or 0 if not set
 */
int msm_audio_cal_get_topology(int type)
{
	switch (type) {
	case 0: return adm_topology;
	case 1: return afe_topology;
	case 2: return asm_topology;
	default: return 0;
	}
}
EXPORT_SYMBOL_GPL(msm_audio_cal_get_topology);

/**
 * msm_audio_cal_get_cal - Get stored calibration data for a cal_type
 * @cal_type: calibration type (e.g., 17 = AFE_COMMON_TX)
 * @buf: output pointer to cal data buffer
 * @size: output cal data size
 * @phys: output physical address of cal data
 * Returns: 0 on success, -ENOENT if not found
 */
int msm_audio_cal_get_cal(int cal_type, void **buf, size_t *size,
			  dma_addr_t *phys)
{
	struct cal_block *blk;

	mutex_lock(&cal_lock);
	list_for_each_entry(blk, &cal_blocks, list) {
		if (blk->cal_type == cal_type && blk->cal_data) {
			*buf = blk->cal_data;
			*size = blk->cal_size;
			*phys = blk->cal_phys;
			mutex_unlock(&cal_lock);
			return 0;
		}
	}
	mutex_unlock(&cal_lock);
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(msm_audio_cal_get_cal);

static int __init msm_audio_cal_init(void)
{
	int ret;

	pr_info("msm_audio_cal: initializing\n");

	ret = misc_register(&msm_audio_cal_misc);
	if (ret)
		return ret;

	/* Register ION shim */
	ion_dev = msm_audio_cal_misc.this_device;
	ret = misc_register(&ion_shim_misc);
	if (ret)
		pr_warn("msm_audio_cal: failed to register /dev/ion shim: %d\n", ret);
	else
		pr_info("msm_audio_cal: /dev/ion shim registered\n");

	/*
	 * Find the FastRPC device which has ADSP CMA memory attached.
	 * ION allocations will use this device's DMA allocator so the
	 * ADSP can access the cal buffers.
	 */
	{
		struct device *dev;

		dev = bus_find_device_by_name(&platform_bus_type, NULL,
			"c200000.remoteproc:smd-edge:fastrpc:cb@1");
		if (dev) {
			ion_cma_dev = dev;
			pr_info("msm_audio_cal: using fastrpc cb@1 for CMA alloc\n");
		} else {
			/* Try the fastrpc parent */
			dev = bus_find_device_by_name(&platform_bus_type, NULL,
				"c200000.remoteproc:smd-edge:fastrpc");
			if (dev) {
				ion_cma_dev = dev;
				pr_info("msm_audio_cal: using fastrpc device for CMA alloc\n");
			} else {
				pr_warn("msm_audio_cal: no CMA device found, ION allocs may not be ADSP-accessible\n");
			}
		}
	}

	return 0;
}

static void __exit msm_audio_cal_exit(void)
{
	struct cal_block *blk, *tmp;

	mutex_lock(&cal_lock);
	list_for_each_entry_safe(blk, tmp, &cal_blocks, list) {
		list_del(&blk->list);
		kfree(blk->cal_data);
		kfree(blk);
	}
	mutex_unlock(&cal_lock);

	misc_deregister(&msm_audio_cal_misc);
}

module_init(msm_audio_cal_init);
module_exit(msm_audio_cal_exit);

MODULE_DESCRIPTION("Minimal Qualcomm Audio Calibration driver");
MODULE_LICENSE("GPL");
