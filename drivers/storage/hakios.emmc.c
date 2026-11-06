/*
 * The eMMC of the Xiaomi Mi Box S Gen 3 (Amlogic S905X5M, SD_EMMC controller at 0xfe08c000) as block
 * devices, /dev/emmc_NAME and /dev/remmc_NAME:
 *
 *   emmc=1         emmc_darwin: the GPT partition "darwin" (the tvOS system volume at its start), read only
 *   emmc=2         the same, read only, and emmc_darwin_data: darwin from EMMC_DATA_FIRST to its end, writable --
 *                  the data volume (HFS Plus, init mounts it on /private/var: var_hfs=emmc_darwin_data; made by
 *                  tools/twl-datavol.py)
 *
 * The controller is used as U-Boot leaves it: clocked, tuned, the bus 8 bits wide and the card selected
 * (RCA 1) in its transfer state, the user area current. Nothing of that is changed; a request is one
 * command in the controller's own registers (no descriptor chain), CMD23 then CMD18 or CMD25, with the
 * data in a physically contiguous bounce buffer, and the status register is watched until the end of the
 * chain. No interrupt.
 *
 * Only the partitions named are reachable: every request is checked against the partition's extent,
 * which comes from the GPT on the card itself, and only emmc_darwin_data takes writes.
 */

#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/hakios.emmc.h>
#include <vm/vm_kern.h>
#include <arm/caches_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);

#define EMMC_BASE               0xfe08c000UL
#define SD_EMMC_START           0x40
#define SD_EMMC_CFG             0x44
#define SD_EMMC_STATUS          0x48
#define SD_EMMC_IRQ_EN          0x4c
#define SD_EMMC_CMD_CFG         0x50
#define SD_EMMC_CMD_ARG         0x54
#define SD_EMMC_CMD_DAT         0x58
#define SD_EMMC_CMD_RSP         0x5c

#define CMD_CFG_BLOCK_MODE      (1u << 9)
#define CMD_CFG_R1B             (1u << 10)
#define CMD_CFG_END_OF_CHAIN    (1u << 11)
#define CMD_CFG_TIMEOUT_4S      (12u << 12)
#define CMD_CFG_DATA_IO         (1u << 18)
#define CMD_CFG_DATA_WR         (1u << 19)
#define CMD_CFG_RESP_NUM        (1u << 22)
#define CMD_CFG_OWNER           (1u << 31)
#define STATUS_ERRORS           0x1fffu          /* CRC (rx, tx, descriptor, response), timeouts */
#define STATUS_END_OF_CHAIN     (1u << 13)
#define STATUS_BUSY             (1u << 31)

#define EMMC_SECTOR             512u
#define EMMC_MAX_XFER           (128u * 1024)    /* 256 blocks: the command length field has 9 bits */
#define EMMC_MAX_BLOCKS         (EMMC_MAX_XFER / EMMC_SECTOR)
#define EMMC_RCA                1u
#define EMMC_MAX_PARTS          4
/*
 * Where in darwin the data volume begins: 4 GiB in, past the tvOS system volume (out/tvos-system.apfs, an APFS
 * container of 4,027,199,488 bytes, and the read-only containers that may follow it there). darwin is 10 GiB:
 * 6 GiB of data volume.
 */
#define EMMC_DATA_FIRST         ((4ULL << 30) / EMMC_SECTOR)

struct emmc_part {
	char     name[36];
	uint64_t first, count;          /* in sectors */
	bool     writable;
};

static struct {
	vm_offset_t      regs;
	vm_offset_t      bounce;
	uint64_t         bounce_phys;
	lck_mtx_t        lock;
	bool             ready, broken;
	struct emmc_part part[EMMC_MAX_PARTS];
	int              nparts;
	int              bmajor, cmajor;
	uint64_t         reads, writes, errors;
} emmc = { .bmajor = -1, .cmajor = -1 };

static LCK_GRP_DECLARE(emmc_lck_grp, "hakios-emmc");

static inline uint32_t
rd(uint32_t reg)
{
	return *(volatile uint32_t *)(emmc.regs + reg);
}

static inline void
wr(uint32_t reg, uint32_t v)
{
	*(volatile uint32_t *)(emmc.regs + reg) = v;
}

/* One command; blocks != 0: a data transfer of that many sectors to or from the bounce buffer. */
static int
emmc_cmd(uint32_t index, uint32_t arg, uint32_t flags, uint32_t blocks, uint32_t *resp)
{
	uint32_t cfg = CMD_CFG_OWNER | CMD_CFG_END_OF_CHAIN | CMD_CFG_TIMEOUT_4S | CMD_CFG_RESP_NUM | (index << 24) | flags;
	uint64_t deadline, now;
	uint32_t status;

	if (blocks != 0) {
		cfg |= CMD_CFG_DATA_IO | CMD_CFG_BLOCK_MODE | blocks;
	}
	wr(SD_EMMC_START, 0);
	wr(SD_EMMC_STATUS, 0xffff);
	wr(SD_EMMC_CMD_CFG, cfg);
	wr(SD_EMMC_CMD_DAT, (uint32_t)emmc.bounce_phys);
	wr(SD_EMMC_CMD_RSP, 0);
	__builtin_arm_dsb(0xf);     /* dsb sy */
	wr(SD_EMMC_CMD_ARG, arg);

	clock_interval_to_deadline(5, NSEC_PER_SEC, &deadline);
	for (;;) {
		status = rd(SD_EMMC_STATUS);
		if (status & (STATUS_END_OF_CHAIN | STATUS_ERRORS)) {
			break;
		}
		now = mach_absolute_time();
		if (now > deadline) {
			printf("emmc: CMD%u(0x%x): no end after 5 s, status 0x%x\n", index, arg, status);
			wr(SD_EMMC_START, 0);
			return ETIMEDOUT;
		}
	}
	if (resp != NULL) {
		*resp = rd(SD_EMMC_CMD_RSP);
	}
	wr(SD_EMMC_STATUS, 0xffff);
	if (status & STATUS_ERRORS) {
		printf("emmc: CMD%u(0x%x): error status 0x%x\n", index, arg, status);
		return EIO;
	}
	return 0;
}

/* The card back in its transfer state after a write (busy while it programs). */
static int
emmc_wait_ready(void)
{
	uint64_t deadline;
	uint32_t resp = 0;

	clock_interval_to_deadline(10, NSEC_PER_SEC, &deadline);
	do {
		if (emmc_cmd(13, EMMC_RCA << 16, 0, 0, &resp) != 0) {
			return EIO;
		}
		/* READY_FOR_DATA, CURRENT_STATE tran */
		if ((resp & (1u << 8)) && ((resp >> 9) & 0xf) == 4) {
			return 0;
		}
	} while (mach_absolute_time() < deadline);
	printf("emmc: the card stays busy (status 0x%x)\n", resp);
	return ETIMEDOUT;
}

/* Sectors [lba, lba + count) of the whole card, through the bounce buffer. Lock held. */
static int
emmc_xfer(bool write, uint64_t lba, uint32_t count, void *data)
{
	int error;

	if (count == 0 || count > EMMC_MAX_BLOCKS) {
		return EINVAL;
	}
	if (write) {
		memcpy((void *)emmc.bounce, data, (size_t)count * EMMC_SECTOR);
	}
	FlushPoC_DcacheRegion(emmc.bounce, (size_t)count * EMMC_SECTOR);
	error = emmc_cmd(23, count, 0, 0, NULL);
	if (error == 0) {
		error = emmc_cmd(write ? 25 : 18, (uint32_t)lba, write ? CMD_CFG_DATA_WR : 0, count, NULL);
	}
	if (error == 0 && write) {
		error = emmc_wait_ready();
	}
	FlushPoC_DcacheRegion(emmc.bounce, (size_t)count * EMMC_SECTOR);
	if (error == 0 && !write) {
		memcpy(data, (const void *)emmc.bounce, (size_t)count * EMMC_SECTOR);
	}
	if (error != 0) {
		emmc.errors++;
	} else if (write) {
		emmc.writes++;
	} else {
		emmc.reads++;
	}
	return error;
}

static struct emmc_part *
emmc_part_of(dev_t dev)
{
	int unit = minor(dev);

	return (emmc.ready && unit >= 0 && unit < emmc.nparts) ? &emmc.part[unit] : NULL;
}

bool
hakios_emmc_dev(dev_t dev)
{
	return (emmc.bmajor >= 0 && major(dev) == emmc.bmajor) || (emmc.cmajor >= 0 && major(dev) == emmc.cmajor);
}

static int
emmc_open(dev_t dev, int flags, __unused int devtype, __unused proc_t p)
{
	struct emmc_part *pt = emmc_part_of(dev);

	if (pt == NULL) {
		return ENXIO;
	}
	if ((flags & FWRITE) && !pt->writable) {
		return EROFS;
	}
	return 0;
}

static int
emmc_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	return 0;
}

static void
emmc_strategy(buf_t bp)
{
	struct emmc_part *pt = emmc_part_of(buf_device(bp));
	bool write = !(buf_flags(bp) & B_READ);
	uint64_t first, len;
	caddr_t mapped;
	int error = 0;

	if (pt == NULL) {
		buf_seterror(bp, ENXIO);
		buf_biodone(bp);
		return;
	}
	if (write && !pt->writable) {
		buf_seterror(bp, EROFS);
		buf_biodone(bp);
		return;
	}
	first = (uint64_t)buf_blkno(bp);
	len = buf_count(bp);
	if ((len % EMMC_SECTOR) != 0 || first > pt->count || len / EMMC_SECTOR > pt->count - first) {
		buf_seterror(bp, first == pt->count ? 0 : EINVAL);
		buf_setresid(bp, buf_count(bp));
		buf_biodone(bp);
		return;
	}
	if ((error = buf_map(bp, &mapped)) != 0) {
		buf_seterror(bp, error);
		buf_biodone(bp);
		return;
	}
	lck_mtx_lock(&emmc.lock);
	uint64_t done = 0;
	while (done < len && error == 0) {
		uint32_t n = (uint32_t)MIN((len - done) / EMMC_SECTOR, EMMC_MAX_BLOCKS);
		error = emmc_xfer(write, pt->first + first + done / EMMC_SECTOR, n, (uint8_t *)mapped + done);
		if (error == 0) {
			done += (uint64_t)n * EMMC_SECTOR;
		}
	}
	lck_mtx_unlock(&emmc.lock);
	buf_unmap(bp);
	buf_setresid(bp, (uint32_t)(len - done));
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
emmc_ioctl(dev_t dev, u_long cmd, caddr_t data, __unused int fflag, __unused proc_t p)
{
	struct emmc_part *pt = emmc_part_of(dev);

	if (pt == NULL) {
		return ENXIO;
	}
	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*(uint32_t *)data = EMMC_SECTOR;
		return 0;
	case DKIOCSETBLOCKSIZE:
		return *(uint32_t *)data == EMMC_SECTOR ? 0 : EINVAL;
	case DKIOCGETBLOCKCOUNT:
		*(uint64_t *)data = pt->count;
		return 0;
	case DKIOCISWRITABLE:
		*(uint32_t *)data = pt->writable;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*(uint64_t *)data = EMMC_MAX_BLOCKS;
		return 0;
	case DKIOCGETPHYSICALBLOCKSIZE:
		*(uint32_t *)data = EMMC_SECTOR;
		return 0;
	case DKIOCSYNCHRONIZE:
	case DKIOCSYNCHRONIZECACHE:
		return 0;               /* a write is done when the card is back in its transfer state */
	default:
		return ENOTTY;
	}
}

static int
emmc_size(dev_t dev)
{
	return emmc_part_of(dev) ? (int)EMMC_SECTOR : -1;
}

static int
emmc_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(emmc_strategy, NULL, dev, B_READ, (u_int (*)(buf_t))emmc_size, uio, EMMC_SECTOR);
}

static int
emmc_write(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(emmc_strategy, NULL, dev, B_WRITE, (u_int (*)(buf_t))emmc_size, uio, EMMC_SECTOR);
}

static const struct bdevsw emmc_bdevsw = {
	.d_open     = emmc_open,
	.d_close    = emmc_close,
	.d_strategy = emmc_strategy,
	.d_ioctl    = emmc_ioctl,
	.d_dump     = eno_dump,
	.d_psize    = emmc_size,
	.d_type     = D_DISK,
};

static const struct cdevsw emmc_cdevsw = {
	.d_open       = emmc_open,
	.d_close      = emmc_close,
	.d_read       = emmc_read,
	.d_write      = emmc_write,
	.d_ioctl      = emmc_ioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

/* ---- the GPT ---- */

static void
emmc_utf16_name(const uint8_t *u, char *out, size_t size)
{
	size_t i;

	for (i = 0; i + 1 < size && i < 36 && (u[2 * i] || u[2 * i + 1]); i++) {
		out[i] = u[2 * i + 1] ? '?' : (char)u[2 * i];
	}
	out[i] = 0;
}

static int
emmc_read_gpt(bool with_data)
{
	uint8_t *buf;
	uint64_t entries_lba;
	uint32_t n_entries, entry_size;
	int error;

	buf = kalloc_data(EMMC_SECTOR * 32, Z_WAITOK | Z_ZERO);
	if (buf == NULL) {
		return ENOMEM;
	}
	lck_mtx_lock(&emmc.lock);
	error = emmc_xfer(false, 1, 1, buf);
	lck_mtx_unlock(&emmc.lock);
	if (error != 0 || memcmp(buf, "EFI PART", 8) != 0) {
		printf("emmc: no GPT header at LBA 1 (error %d)\n", error);
		kfree_data(buf, EMMC_SECTOR * 32);
		return error ? error : EINVAL;
	}
	entries_lba = *(uint64_t *)(buf + 72);
	n_entries = *(uint32_t *)(buf + 80);
	entry_size = *(uint32_t *)(buf + 84);
	if (entry_size != 128 || n_entries > 128) {
		kfree_data(buf, EMMC_SECTOR * 32);
		return EINVAL;
	}
	lck_mtx_lock(&emmc.lock);
	error = emmc_xfer(false, entries_lba, 32, buf);
	lck_mtx_unlock(&emmc.lock);
	if (error != 0) {
		kfree_data(buf, EMMC_SECTOR * 32);
		return error;
	}
	for (uint32_t i = 0; i < n_entries && emmc.nparts < EMMC_MAX_PARTS; i++) {
		const uint8_t *e = buf + i * 128;
		char name[36];
		uint64_t first = *(const uint64_t *)(e + 32), last = *(const uint64_t *)(e + 40);

		emmc_utf16_name(e + 56, name, sizeof(name));
		if (strcmp(name, "darwin") != 0 || last < first) {
			continue;
		}
		struct emmc_part *pt = &emmc.part[emmc.nparts++];
		strlcpy(pt->name, name, sizeof(pt->name));
		pt->first = first;
		pt->count = last - first + 1;
		pt->writable = false;           /* the system volume: never written from here */
		/* emmc=2: the rest of it, the data volume, the one stretch of the card this driver writes */
		if (with_data && pt->count > EMMC_DATA_FIRST && emmc.nparts < EMMC_MAX_PARTS) {
			struct emmc_part *data = &emmc.part[emmc.nparts++];

			strlcpy(data->name, "darwin_data", sizeof(data->name));
			data->first = pt->first + EMMC_DATA_FIRST;
			data->count = pt->count - EMMC_DATA_FIRST;
			data->writable = true;
		}
	}
	kfree_data(buf, EMMC_SECTOR * 32);
	return 0;
}

void
hakios_emmc_init(void)
{
	uint32_t mode = 0, resp = 0;

	if (!PE_parse_boot_argn("emmc", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	lck_mtx_init(&emmc.lock, &emmc_lck_grp, LCK_ATTR_NULL);
	emmc.regs = ml_io_map(EMMC_BASE, 0x1000);
	if (kmem_alloc_contig(kernel_map, &emmc.bounce, EMMC_MAX_XFER, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("emmc: no contiguous bounce buffer\n");
		return;
	}
	emmc.bounce_phys = kvtophys(emmc.bounce);
	if (emmc.bounce_phys >> 32) {
		printf("emmc: the bounce buffer is above 4 GB (0x%llx): the controller cannot reach it\n", emmc.bounce_phys);
		return;
	}
	wr(SD_EMMC_IRQ_EN, 0);
	if (emmc_cmd(13, EMMC_RCA << 16, 0, 0, &resp) != 0) {
		printf("emmc: the card does not answer CMD13 (cfg 0x%x)\n", rd(SD_EMMC_CFG));
		return;
	}
	printf("emmc: card status 0x%x, controller cfg 0x%x, bounce at 0x%llx\n", resp, rd(SD_EMMC_CFG), emmc.bounce_phys);
	if (emmc_read_gpt(mode == 2) != 0 || emmc.nparts == 0) {
		printf("emmc: no partition \"darwin\" in the GPT\n");
		return;
	}
	emmc.bmajor = bdevsw_add(-1, &emmc_bdevsw);
	if (emmc.bmajor < 0) {
		printf("emmc: bdevsw_add failed\n");
		return;
	}
	emmc.cmajor = cdevsw_add_with_bdev(-1, &emmc_cdevsw, emmc.bmajor);
	if (emmc.cmajor < 0) {
		printf("emmc: cdevsw_add failed\n");
		return;
	}
	emmc.ready = true;
	for (int i = 0; i < emmc.nparts; i++) {
		struct emmc_part *pt = &emmc.part[i];

		devfs_make_node(makedev(emmc.bmajor, i), DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "emmc_%s", pt->name);
		devfs_make_node(makedev(emmc.cmajor, i), DEVFS_CHAR, UID_ROOT, GID_OPERATOR, 0640, "remmc_%s", pt->name);
		printf("emmc: /dev/emmc_%s: sectors %llu..%llu (%llu MB)%s\n", pt->name, pt->first, pt->first + pt->count - 1,
		    pt->count * EMMC_SECTOR >> 20, pt->writable ? ", writable" : "");
	}
}
