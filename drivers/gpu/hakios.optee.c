/*
 * A minimal OP-TEE client for the Amlogic box (S905X5M "S7D"), for the video decoders' firmware, which is
 * encrypted and only the secure world can load into the cores. Boot-arg optee=N, off by default: without it
 * nothing here calls the secure world.
 *
 *   optee=1   the secure world's UID, API revision, OS UUID and revision, capabilities and static SHM config
 *   optee=2   also: a session with the video TA 526fc4fc-7ee6-4a12-96e3-83da9565bce8 (the TA's bytes come from
 *             fw/<uuid>.ta through the LOAD_TA RPC, as tee-supplicant would give them), the firmware package
 *             fw/video_ucode.bin handed to it, the session closed. Then hakios_optee_load_video_fw() works.
 *   optee_delay=S   do it S seconds later, in a thread (the early boot log gets lost)
 *   optee_ta=NAME   fw/NAME instead of fw/<uuid>.ta for the video TA; optee_fw=NAME instead of video_ucode.bin
 *
 * What the preload does is CoreELEC's libtee_preload_fw.so (load_sec_firmware(path, 0, 0), "VDEC CA" 1.0.21):
 *   - the file: if it starts with "PWEN" and the word at 0x8100 is not "KCAP", the first 0x8000 bytes (the plain
 *     text headers) are skipped; the rest (412480 bytes for the S905X5M package) is the payload
 *   - OpenSession(uuid, TEEC_LOGIN_PUBLIC): param 0 VALUE_INPUT (payload size, 0), param 1 VALUE_INPUT (1, 0),
 *     the CA's version
 *   - InvokeCommand(0) per 128 KiB of payload: param 0 MEMREF_TEMP_INPUT (chunk); TEEC_SUCCESS and TEEC_ERROR_BUSY
 *     both go on to the next chunk
 *   - CloseSession. The TA keeps the firmware; the kernel then loads each core by the fast call 0xB200000F.
 *
 * The OP-TEE side (3.18 on this box, dynamic shared memory: the rescue kernel's log): std calls are
 * OPTEE_SMC_CALL_WITH_ARG with the page-aligned optee_msg_arg's physical address, RPCs come back for the RPC
 * argument page (ALLOC/FREE), for foreign interrupts, and as commands (LOAD_TA, SHM_ALLOC/FREE, GET_TIME, ...).
 * Memory references go as OPTEE_MSG_ATTR_NONCONTIG lists of 4 KiB pages. All of it lives in one physically
 * contiguous, normal (cached) kernel pool taken at boot, while there is contiguous memory to take.
 * Note: fast call 15 is Amlogic's LOAD_VIDEO_FW, which upstream OP-TEE numbers GET_THREAD_COUNT: never probed.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.optee.h>
#include <vm/vm_kern.h>

extern uint64_t kvtophys(vm_offset_t va);
extern void IOSleep(unsigned int milliseconds);
extern kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
extern void thread_deallocate(thread_t thread);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

/* ---- the ABI (optee_smc.h, optee_msg.h, optee_rpc_cmd.h of OP-TEE 3.18) ---- */

#define SMC_CALLS_UID           0xbf00ff01u
#define SMC_CALLS_REVISION      0xbf00ff03u
#define SMC_GET_OS_UUID         0xb2000000u
#define SMC_GET_OS_REVISION     0xb2000001u
#define SMC_CALL_WITH_ARG       0x32000004u
#define SMC_RETURN_FROM_RPC     0x32000003u
#define SMC_GET_SHM_CONFIG      0xb2000007u
#define SMC_EXCHANGE_CAPS       0xb2000009u
#define SMC_LOAD_VIDEO_FW       0xb200000fu     /* Amlogic: TEE_SMC_LOAD_VIDEO_FW */

#define OPTEE_UID_0             0x384fb3e0u
#define OPTEE_UID_1             0xe7f811e3u
#define OPTEE_UID_2             0xaf630002u
#define OPTEE_UID_3             0xa5d5c51bu

#define SEC_CAP_HAVE_RESERVED_SHM (1u << 0)
#define SEC_CAP_UNREGISTERED_SHM  (1u << 1)
#define SEC_CAP_DYNAMIC_SHM       (1u << 2)
#define SEC_CAP_VIRTUALIZATION    (1u << 3)
#define SEC_CAP_MEMREF_NULL       (1u << 4)
#define SEC_CAP_ASYNC_NOTIF       (1u << 5)
#define SEC_CAP_RPC_ARG           (1u << 6)

#define RET_OK                  0x0u
#define RET_ETHREAD_LIMIT       0x1u
#define RET_UNKNOWN_FUNCTION    0xffffffffu
#define RET_IS_RPC(r)           (((r) & 0xffff0000u) == 0xffff0000u && (r) != RET_UNKNOWN_FUNCTION)
#define RPC_FUNC(r)             ((r) & 0xffffu)
#define RPC_FUNC_ALLOC          0
#define RPC_FUNC_FREE           2
#define RPC_FUNC_FOREIGN_INTR   4
#define RPC_FUNC_CMD            5

#define MSG_CMD_OPEN_SESSION    0
#define MSG_CMD_INVOKE_COMMAND  1
#define MSG_CMD_CLOSE_SESSION   2

#define ATTR_NONE               0x0u
#define ATTR_VALUE_INPUT        0x1u
#define ATTR_VALUE_OUTPUT       0x2u
#define ATTR_VALUE_INOUT        0x3u
#define ATTR_RMEM_INPUT         0x5u
#define ATTR_RMEM_OUTPUT        0x6u
#define ATTR_RMEM_INOUT         0x7u
#define ATTR_TMEM_INPUT         0x9u
#define ATTR_TMEM_OUTPUT        0xau
#define ATTR_TMEM_INOUT         0xbu
#define ATTR_TYPE_MASK          0xffu
#define ATTR_META               (1u << 8)
#define ATTR_NONCONTIG          (1u << 9)

#define RPC_CMD_LOAD_TA         0
#define RPC_CMD_RPMB            1
#define RPC_CMD_FS              2
#define RPC_CMD_GET_TIME        3
#define RPC_CMD_NOTIFICATION    4       /* WAIT_QUEUE in older OP-TEE */
#define RPC_CMD_SUSPEND         5
#define RPC_CMD_SHM_ALLOC       6
#define RPC_CMD_SHM_FREE        7

#define TEEC_SUCCESS            0x00000000u
#define TEEC_ERROR_GENERIC      0xffff0000u
#define TEEC_ERROR_BAD_PARAMS   0xffff0006u
#define TEEC_ERROR_ITEM_NOT_FOUND 0xffff0008u
#define TEEC_ERROR_NOT_SUPPORTED 0xffff000au
#define TEEC_ERROR_OUT_OF_MEMORY 0xffff000cu
#define TEEC_ERROR_BUSY         0xffff000du
#define TEEC_ERROR_SHORT_BUFFER 0xffff0010u

struct msg_param {
	uint64_t attr;
	union {
		struct { uint64_t buf_ptr, size, shm_ref; } tmem;
		struct { uint64_t offs, size, shm_ref; } rmem;
		struct { uint64_t a, b, c; } value;
		uint8_t octets[24];
	} u;
};

struct msg_arg {
	uint32_t cmd;
	uint32_t func;
	uint32_t session;
	uint32_t cancel_id;
	uint32_t pad;
	uint32_t ret;
	uint32_t ret_origin;
	uint32_t num_params;
	struct msg_param params[];
};

_Static_assert(sizeof(struct msg_param) == 32, "optee_msg_param");
_Static_assert(sizeof(struct msg_arg) == 32, "optee_msg_arg");

/* ---- the video TA and its package ---- */

static const uint8_t video_ta_uuid[16] = {     /* 526fc4fc-7ee6-4a12-96e3-83da9565bce8, as octets */
	0x52, 0x6f, 0xc4, 0xfc, 0x7e, 0xe6, 0x4a, 0x12, 0x96, 0xe3, 0x83, 0xda, 0x95, 0x65, 0xbc, 0xe8,
};
#define PRELOAD_CMD             0u              /* load_sec_firmware's third argument (tee_preload_fw -e makes it 1) */
#define PRELOAD_CHUNK           0x20000u
#define PKG_MAGIC               0x4e455750u     /* "PWEN" */
#define PKG_KCAP                0x5041434bu     /* "KCAP" at 0x8100: no REE header to skip */
#define PKG_REE_HEADER          0x8000u
#define CA_VERSION_MAJOR        1u              /* "1.0.21-g006fb97(build:487)" */
#define CA_VERSION_MINOR        0u

/* ---- shared memory: one contiguous pool, handed out in 4 KiB pages ---- */

#define PG                      4096u
#define POOL_BYTES              (1024u * 1024u)
#define POOL_PAGES              (POOL_BYTES / PG)
#define LIST_ENTRIES            (PG / 8 - 1)    /* page addresses in a page of the NONCONTIG list; the last: next */
#define NO_LIST                 0xffffffffu
#define MAX_SHM                 32

struct shm {
	bool     live;
	uint64_t cookie;
	uint32_t first, pages;  /* pool pages */
	uint32_t list;          /* the page holding its NONCONTIG list, or NO_LIST */
	uint64_t size;
};

static struct {
	uint8_t  *va;
	uint64_t  pa;
	uint8_t   used[POOL_PAGES];
	struct shm shm[MAX_SHM];
	uint64_t  next_cookie;
} pool;

static int level;
static uint32_t delay_secs;
static char ta_name[64], fw_name[64];
static volatile bool fw_ready;
static LCK_GRP_DECLARE(optee_lck_grp, "hakios_optee");
static LCK_MTX_DECLARE(optee_lock, &optee_lck_grp);    /* one std call at a time, and the pool */

static struct {
	uint32_t alloc, free, foreign, cmd, load_ta, shm_alloc, shm_free, time, notif, suspend, refused, retry;
} rpcs;

static void
smc8(uint64_t r[8])
{
	register uint64_t x0 __asm__("x0") = r[0];
	register uint64_t x1 __asm__("x1") = r[1];
	register uint64_t x2 __asm__("x2") = r[2];
	register uint64_t x3 __asm__("x3") = r[3];
	register uint64_t x4 __asm__("x4") = r[4];
	register uint64_t x5 __asm__("x5") = r[5];
	register uint64_t x6 __asm__("x6") = r[6];
	register uint64_t x7 __asm__("x7") = r[7];

	__asm__ volatile ("smc #0"
	    : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3), "+r" (x4), "+r" (x5), "+r" (x6), "+r" (x7)
	    :
	    : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	r[0] = x0; r[1] = x1; r[2] = x2; r[3] = x3; r[4] = x4; r[5] = x5; r[6] = x6; r[7] = x7;
}

static void
fast(uint32_t fn, uint64_t a1, uint64_t a2, uint64_t a3, uint64_t out[4])
{
	uint64_t r[8] = { fn, a1, a2, a3, 0, 0, 0, 0 };

	smc8(r);
	for (int i = 0; i < 4; i++) {
		out[i] = r[i] & 0xffffffffull;  /* SMC32: the upper halves mean nothing */
	}
}

static void *page_va(uint32_t page) { return pool.va + (uint64_t)page * PG; }
static uint64_t page_pa(uint32_t page) { return pool.pa + (uint64_t)page * PG; }

static bool
pages_take(uint32_t n, uint32_t *first)
{
	for (uint32_t i = 0; i + n <= POOL_PAGES; i++) {
		uint32_t j;

		for (j = 0; j < n && !pool.used[i + j]; j++) {
		}
		if (j == n) {
			memset(&pool.used[i], 1, n);
			memset(page_va(i), 0, (size_t)n * PG);
			*first = i;
			return true;
		}
		i += j;
	}
	return false;
}

static void
pages_give(uint32_t first, uint32_t n)
{
	memset(&pool.used[first], 0, n);
}

/* size bytes of shared memory; with_list: also its page list, for a NONCONTIG reference */
static struct shm *
shm_alloc(uint64_t size, bool with_list)
{
	uint32_t n = (uint32_t)((size + PG - 1) / PG);
	struct shm *s = NULL;

	if (n == 0) {
		n = 1;
	}
	if (size > POOL_BYTES || n > LIST_ENTRIES) {
		return NULL;
	}
	for (int i = 0; i < MAX_SHM; i++) {
		if (!pool.shm[i].live) {
			s = &pool.shm[i];
			break;
		}
	}
	if (s == NULL || !pages_take(n, &s->first)) {
		return NULL;
	}
	s->list = NO_LIST;
	if (with_list) {
		uint64_t *list;

		if (!pages_take(1, &s->list)) {
			pages_give(s->first, n);
			return NULL;
		}
		list = (uint64_t *)page_va(s->list);
		for (uint32_t i = 0; i < n; i++) {
			list[i] = page_pa(s->first + i);
		}
	}
	s->pages = n;
	s->size = size;
	s->cookie = pool.next_cookie++;
	s->live = true;
	return s;
}

static void
shm_free(struct shm *s)
{
	if (s == NULL || !s->live) {
		return;
	}
	pages_give(s->first, s->pages);
	if (s->list != NO_LIST) {
		pages_give(s->list, 1);
	}
	s->live = false;
}

static struct shm *
shm_find(uint64_t cookie)
{
	for (int i = 0; i < MAX_SHM; i++) {
		if (pool.shm[i].live && pool.shm[i].cookie == cookie) {
			return &pool.shm[i];
		}
	}
	return NULL;
}

static void *shm_va(struct shm *s) { return page_va(s->first); }
static uint64_t shm_pa(struct shm *s) { return page_pa(s->first); }

/* a NONCONTIG temporary memory reference to s (buffers start on a page: the offset bits stay 0) */
static void
tmem_ref(struct msg_param *p, uint32_t type, struct shm *s, uint64_t size)
{
	p->attr = type | ATTR_NONCONTIG;
	p->u.tmem.buf_ptr = page_pa(s->list);
	p->u.tmem.size = size;
	p->u.tmem.shm_ref = s->cookie;
}

/* ---- RPC commands ---- */

static void
uuid_name(const uint8_t o[16], char *out, size_t len)
{
	snprintf(out, len, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x.ta",
	    o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7], o[8], o[9], o[10], o[11], o[12], o[13], o[14], o[15]);
}

/* the memory a memref parameter of an RPC names: our pool, or NULL (no buffer) */
static uint8_t *
rpc_memref(struct msg_param *p, uint64_t *cap)
{
	uint32_t type = (uint32_t)(p->attr & ATTR_TYPE_MASK);
	struct shm *s;

	*cap = 0;
	if (type >= ATTR_RMEM_INPUT && type <= ATTR_RMEM_INOUT) {
		if (p->u.rmem.shm_ref == 0) {
			return NULL;
		}
		s = shm_find(p->u.rmem.shm_ref);
		if (s == NULL || p->u.rmem.offs > (uint64_t)s->pages * PG) {
			return NULL;
		}
		*cap = (uint64_t)s->pages * PG - p->u.rmem.offs;
		return (uint8_t *)shm_va(s) + p->u.rmem.offs;
	}
	if (type >= ATTR_TMEM_INPUT && type <= ATTR_TMEM_INOUT && p->u.tmem.buf_ptr != 0) {
		uint64_t pa = p->u.tmem.buf_ptr;

		if (pa >= pool.pa && pa < pool.pa + POOL_BYTES) {
			*cap = pool.pa + POOL_BYTES - pa;
			return pool.va + (pa - pool.pa);
		}
	}
	return NULL;
}

static void
rpc_load_ta(struct msg_arg *arg)
{
	struct msg_param *p = arg->params;
	uint8_t uuid[16];
	char name[64];
	const uint8_t *ta;
	uint64_t ta_size = 0, cap = 0;
	uint8_t *buf;
	uint32_t t1;

	rpcs.load_ta++;
	if (arg->num_params != 2 || (p[0].attr & ATTR_TYPE_MASK) != ATTR_VALUE_INPUT) {
		arg->ret = TEEC_ERROR_BAD_PARAMS;
		return;
	}
	t1 = (uint32_t)(p[1].attr & ATTR_TYPE_MASK);
	if (t1 != ATTR_TMEM_OUTPUT && t1 != ATTR_RMEM_OUTPUT && t1 != ATTR_TMEM_INOUT && t1 != ATTR_RMEM_INOUT) {
		arg->ret = TEEC_ERROR_BAD_PARAMS;
		return;
	}
	memcpy(uuid, &p[0].u.value, sizeof(uuid));
	uuid_name(uuid, name, sizeof(name));
	ta = hakios_image_file("fw", memcmp(uuid, video_ta_uuid, 16) == 0 && ta_name[0] ? ta_name : name, &ta_size, NULL);
	if (ta == NULL) {
		printf("optee: LOAD_TA %s: no fw/%s in the image\n", name,
		    memcmp(uuid, video_ta_uuid, 16) == 0 && ta_name[0] ? ta_name : name);
		arg->ret = TEEC_ERROR_ITEM_NOT_FOUND;
		return;
	}
	buf = rpc_memref(&p[1], &cap);
	if (buf != NULL && cap > p[1].u.tmem.size) {
		cap = p[1].u.tmem.size;
	}
	printf("optee: LOAD_TA %s: %llu bytes, buffer %s (%llu)\n", name, ta_size, buf ? "given" : "none", cap);
	p[1].u.tmem.size = ta_size;     /* tmem.size and rmem.size are the same field */
	if (buf == NULL) {
		arg->ret = TEEC_SUCCESS;        /* the size it should allocate */
		return;
	}
	if (ta_size > cap) {
		arg->ret = TEEC_ERROR_SHORT_BUFFER;
		return;
	}
	memcpy(buf, ta, (size_t)ta_size);
	arg->ret = TEEC_SUCCESS;
}

static void
rpc_shm_alloc(struct msg_arg *arg)
{
	struct msg_param *p = arg->params;
	struct shm *s;
	uint64_t type, size;

	rpcs.shm_alloc++;
	if (arg->num_params < 1 || p[0].attr != ATTR_VALUE_INPUT) {
		arg->ret = TEEC_ERROR_BAD_PARAMS;
		return;
	}
	for (uint32_t n = 1; n < arg->num_params; n++) {
		if (p[n].attr != ATTR_NONE) {
			arg->ret = TEEC_ERROR_BAD_PARAMS;
			return;
		}
	}
	type = p[0].u.value.a;
	size = p[0].u.value.b;
	s = shm_alloc(size, true);
	if (s == NULL) {
		printf("optee: SHM_ALLOC type %llu, %llu bytes: the pool is full\n", type, size);
		arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
		return;
	}
	if (level >= 3) {
		printf("optee: SHM_ALLOC type %llu, %llu bytes -> %#llx (%u pages), cookie %llu\n",
		    type, size, shm_pa(s), s->pages, s->cookie);
	}
	memset(&p[0], 0, sizeof(p[0]));
	tmem_ref(&p[0], ATTR_TMEM_OUTPUT, s, size);
	arg->ret = TEEC_SUCCESS;
}

static void
rpc_shm_free(struct msg_arg *arg)
{
	struct msg_param *p = arg->params;
	struct shm *s;

	rpcs.shm_free++;
	if (arg->num_params != 1 || p[0].attr != ATTR_VALUE_INPUT) {
		arg->ret = TEEC_ERROR_BAD_PARAMS;
		return;
	}
	s = shm_find(p[0].u.value.b);
	if (s == NULL) {
		printf("optee: SHM_FREE of an unknown cookie %llu\n", p[0].u.value.b);
		arg->ret = TEEC_ERROR_BAD_PARAMS;
		return;
	}
	shm_free(s);
	arg->ret = TEEC_SUCCESS;
}

static void
rpc_cmd(uint64_t cookie)
{
	struct shm *s = shm_find(cookie);
	struct msg_arg *arg;

	rpcs.cmd++;
	if (s == NULL) {
		printf("optee: RPC command in an unknown buffer (cookie %llu)\n", cookie);
		return;
	}
	arg = (struct msg_arg *)shm_va(s);
	if (arg->num_params > ((uint64_t)s->pages * PG - sizeof(*arg)) / sizeof(struct msg_param)) {
		arg->ret = TEEC_ERROR_BAD_PARAMS;
		return;
	}
	switch (arg->cmd) {
	case RPC_CMD_LOAD_TA:
		rpc_load_ta(arg);
		break;
	case RPC_CMD_SHM_ALLOC:
		rpc_shm_alloc(arg);
		break;
	case RPC_CMD_SHM_FREE:
		rpc_shm_free(arg);
		break;
	case RPC_CMD_GET_TIME: {
		clock_sec_t secs;
		clock_nsec_t nsecs;

		rpcs.time++;
		if (arg->num_params != 1 || (arg->params[0].attr & ATTR_TYPE_MASK) != ATTR_VALUE_OUTPUT) {
			arg->ret = TEEC_ERROR_BAD_PARAMS;
			break;
		}
		clock_get_calendar_nanotime(&secs, &nsecs);
		arg->params[0].u.value.a = secs;
		arg->params[0].u.value.b = nsecs;
		arg->ret = TEEC_SUCCESS;
		break;
	}
	case RPC_CMD_NOTIFICATION:
		/* one client and one call at a time: nobody else to wait for; wait = sleep a little, OP-TEE looks again */
		rpcs.notif++;
		if (arg->num_params == 1 && arg->params[0].u.value.a == 0) {
			IOSleep(1);
		}
		arg->ret = TEEC_SUCCESS;
		break;
	case RPC_CMD_SUSPEND:
		rpcs.suspend++;
		if (arg->num_params == 1) {
			uint64_t ms = arg->params[0].u.value.a;

			IOSleep((unsigned int)(ms > 1000 ? 1000 : ms));
		}
		arg->ret = TEEC_SUCCESS;
		break;
	default:
		/* RPMB, the REE file system, sockets, ...: tee-supplicant's work, which this client does not do */
		rpcs.refused++;
		printf("optee: RPC command %u refused (params %u, param 0 %#llx: %#llx %#llx)\n", arg->cmd,
		    arg->num_params, arg->num_params ? arg->params[0].attr : 0, arg->num_params ? arg->params[0].u.value.a : 0,
		    arg->num_params ? arg->params[0].u.value.b : 0);
		arg->ret = TEEC_ERROR_NOT_SUPPORTED;
		break;
	}
}

/* one RPC return: p[0..3] hold what the secure world returned, p[4], p[5] what we last gave back */
static void
rpc(uint64_t p[8])
{
	uint32_t fn = RPC_FUNC((uint32_t)p[0]);

	switch (fn) {
	case RPC_FUNC_ALLOC: {
		struct shm *s;

		rpcs.alloc++;
		s = shm_alloc((uint32_t)p[1], false);
		if (s != NULL) {
			p[1] = shm_pa(s) >> 32;
			p[2] = shm_pa(s) & 0xffffffffull;
			p[4] = s->cookie >> 32;
			p[5] = s->cookie & 0xffffffffull;
		} else {
			printf("optee: RPC alloc of %llu bytes: the pool is full\n", p[1] & 0xffffffffull);
			p[1] = p[2] = p[4] = p[5] = 0;
		}
		break;
	}
	case RPC_FUNC_FREE: {
		struct shm *s = shm_find(((p[1] & 0xffffffffull) << 32) | (p[2] & 0xffffffffull));

		rpcs.free++;
		if (s != NULL) {
			shm_free(s);
		}
		break;
	}
	case RPC_FUNC_FOREIGN_INTR:
		/* a normal-world interrupt came: it is taken as soon as the SMC returns (interrupts are on here) */
		rpcs.foreign++;
		break;
	case RPC_FUNC_CMD:
		rpc_cmd(((p[1] & 0xffffffffull) << 32) | (p[2] & 0xffffffffull));
		break;
	default:
		printf("optee: unknown RPC %#x\n", fn);
		break;
	}
	p[0] = SMC_RETURN_FROM_RPC;
}

/* a std call on the argument in s; returns the SMC status (0: look at arg->ret) */
static uint32_t
std_call(struct shm *s)
{
	uint64_t p[8] = { SMC_CALL_WITH_ARG, shm_pa(s) >> 32, shm_pa(s) & 0xffffffffull, 0, 0, 0, 0, 0 };

	for (uint32_t rounds = 0; rounds < 2000000; rounds++) {
		uint64_t r[8];

		memcpy(r, p, sizeof(r));
		smc8(r);
		r[0] &= 0xffffffffull;
		if (r[0] == RET_ETHREAD_LIMIT) {
			rpcs.retry++;
			IOSleep(1);
			continue;
		}
		if (!RET_IS_RPC((uint32_t)r[0])) {
			return (uint32_t)r[0];
		}
		p[0] = r[0];
		p[1] = r[1] & 0xffffffffull;
		p[2] = r[2] & 0xffffffffull;
		p[3] = r[3] & 0xffffffffull;
		rpc(p);
	}
	printf("optee: a std call still had RPCs after 2000000 rounds: given up\n");
	return 0xffffffffu;
}

static struct msg_arg *
arg_init(struct shm *s, uint32_t cmd, uint32_t num_params)
{
	struct msg_arg *arg = (struct msg_arg *)shm_va(s);

	memset(arg, 0, sizeof(*arg) + num_params * sizeof(struct msg_param));
	arg->cmd = cmd;
	arg->num_params = num_params;
	return arg;
}

/* ---- probing (optee=1) ---- */

static bool
probe(uint32_t *caps_out)
{
	uint64_t r[4];
	bool is_optee;

	fast(SMC_CALLS_UID, 0, 0, 0, r);
	is_optee = r[0] == OPTEE_UID_0 && r[1] == OPTEE_UID_1 && r[2] == OPTEE_UID_2 && r[3] == OPTEE_UID_3;
	printf("optee: UID %08llx-%08llx-%08llx-%08llx (%s)\n", r[0], r[1], r[2], r[3],
	    is_optee ? "OP-TEE" : "NOT OP-TEE: nothing more is asked");
	if (!is_optee) {
		return false;
	}
	fast(SMC_CALLS_REVISION, 0, 0, 0, r);
	printf("optee: API revision %llu.%llu\n", r[0], r[1]);
	fast(SMC_GET_OS_UUID, 0, 0, 0, r);
	printf("optee: OS UUID %08llx-%08llx-%08llx-%08llx\n", r[0], r[1], r[2], r[3]);
	fast(SMC_GET_OS_REVISION, 0, 0, 0, r);
	printf("optee: OS revision %llu.%llu (build %#llx)\n", r[0], r[1], r[2]);
	fast(SMC_EXCHANGE_CAPS, 0, 0, 0, r);       /* our capabilities: none (not uniprocessor) */
	printf("optee: exchange capabilities: status %#llx, caps %#llx [%s%s%s%s%s%s%s], a2 %#llx, a3 %#llx\n", r[0], r[1],
	    (r[1] & SEC_CAP_HAVE_RESERVED_SHM) ? " reserved-shm" : "", (r[1] & SEC_CAP_UNREGISTERED_SHM) ? " unregistered-shm" : "",
	    (r[1] & SEC_CAP_DYNAMIC_SHM) ? " dynamic-shm" : "", (r[1] & SEC_CAP_VIRTUALIZATION) ? " virtualization" : "",
	    (r[1] & SEC_CAP_MEMREF_NULL) ? " memref-null" : "", (r[1] & SEC_CAP_ASYNC_NOTIF) ? " async-notif" : "",
	    (r[1] & SEC_CAP_RPC_ARG) ? " rpc-arg" : "", r[2], r[3]);
	*caps_out = r[0] == RET_OK ? (uint32_t)r[1] : 0;
	fast(SMC_GET_SHM_CONFIG, 0, 0, 0, r);
	if (r[0] == RET_OK) {
		printf("optee: static SHM at %#llx, %#llx bytes, %s\n", r[1], r[2], (r[3] & 1) ? "cached" : "uncached");
	} else {
		printf("optee: no static SHM (status %#llx)\n", r[0]);
	}
	return true;
}

/* ---- the video TA (optee=2) ---- */

static bool
preload_video_fw(void)
{
	const uint8_t *file;
	uint64_t file_size = 0, skip = 0, payload, done = 0;
	struct shm *argshm, *buf;
	struct msg_arg *arg;
	uint32_t status, session, chunks = 0;
	bool ok = false;

	file = hakios_image_file("fw", fw_name, &file_size, NULL);
	if (file == NULL) {
		printf("optee: no fw/%s in the image\n", fw_name);
		return false;
	}
	if (file_size >= PKG_REE_HEADER + 0x104) {
		uint32_t magic, kcap;

		memcpy(&magic, file, 4);
		memcpy(&kcap, file + 0x8100, 4);
		if (magic == PKG_MAGIC && kcap != PKG_KCAP) {
			skip = PKG_REE_HEADER;
		}
	}
	payload = file_size - skip;
	if (file_size <= skip || payload <= 0xff || payload > 0xffffffffull) {
		printf("optee: fw/%s: bad size %llu\n", fw_name, file_size);
		return false;
	}
	printf("optee: fw/%s: %llu bytes, REE header %#llx skipped, %llu to the TA\n", fw_name, file_size, skip, payload);

	lck_mtx_lock(&optee_lock);
	argshm = shm_alloc(PG, false);
	buf = shm_alloc(PRELOAD_CHUNK, true);
	if (argshm == NULL || buf == NULL) {
		printf("optee: no shared memory for the call\n");
		goto out;
	}

	/* OpenSession(526fc4fc-..., public): (payload size, 0), (CA version 1, 0) */
	arg = arg_init(argshm, MSG_CMD_OPEN_SESSION, 4);
	arg->params[0].attr = ATTR_VALUE_INPUT | ATTR_META;
	memcpy(&arg->params[0].u.value, video_ta_uuid, 16);
	arg->params[1].attr = ATTR_VALUE_INPUT | ATTR_META;
	arg->params[1].u.value.c = 0;                   /* TEEC_LOGIN_PUBLIC; client UUID zero */
	arg->params[2].attr = ATTR_VALUE_INPUT;
	arg->params[2].u.value.a = payload;
	arg->params[2].u.value.b = 0;
	arg->params[3].attr = ATTR_VALUE_INPUT;
	arg->params[3].u.value.a = CA_VERSION_MAJOR;
	arg->params[3].u.value.b = CA_VERSION_MINOR;
	status = std_call(argshm);
	printf("optee: open session: status %#x, ret %#x origin %u, session %#x\n", status, arg->ret, arg->ret_origin,
	    arg->session);
	if (status != RET_OK || arg->ret != TEEC_SUCCESS) {
		goto out;
	}
	session = arg->session;

	/* InvokeCommand(0, memref input) per 128 KiB */
	while (done < payload) {
		uint64_t n = payload - done > PRELOAD_CHUNK ? PRELOAD_CHUNK : payload - done;

		memcpy(shm_va(buf), file + skip + done, (size_t)n);
		arg = arg_init(argshm, MSG_CMD_INVOKE_COMMAND, 4);
		arg->func = PRELOAD_CMD;
		arg->session = session;
		tmem_ref(&arg->params[0], ATTR_TMEM_INPUT, buf, n);
		status = std_call(argshm);
		chunks++;
		if (status != RET_OK || (arg->ret != TEEC_SUCCESS && arg->ret != TEEC_ERROR_BUSY)) {
			printf("optee: invoke %u (bytes %llu..%llu): status %#x, ret %#x origin %u\n", chunks, done, done + n,
			    status, arg->ret, arg->ret_origin);
			break;
		}
		if (arg->ret != TEEC_SUCCESS || level >= 3) {
			printf("optee: invoke %u (bytes %llu..%llu): ret %#x\n", chunks, done, done + n, arg->ret);
		}
		done += n;
	}
	ok = done == payload && arg->ret == TEEC_SUCCESS;

	arg = arg_init(argshm, MSG_CMD_CLOSE_SESSION, 0);
	arg->session = session;
	status = std_call(argshm);
	printf("optee: close session: status %#x, ret %#x\n", status, arg->ret);
out:
	shm_free(argshm);
	shm_free(buf);
	lck_mtx_unlock(&optee_lock);
	printf("optee: RPCs: alloc %u free %u foreign-intr %u cmd %u (load-ta %u shm-alloc %u shm-free %u time %u notif %u"
	    " suspend %u refused %u), thread-limit retries %u\n", rpcs.alloc, rpcs.free, rpcs.foreign, rpcs.cmd, rpcs.load_ta,
	    rpcs.shm_alloc, rpcs.shm_free, rpcs.time, rpcs.notif, rpcs.suspend, rpcs.refused, rpcs.retry);
	printf("optee: video firmware %s (%llu of %llu bytes in %u chunks)\n", ok ? "PRELOADED" : "NOT preloaded", done,
	    payload, chunks);
	return ok;
}

static void
optee_run(void)
{
	uint32_t caps = 0;

	if (!probe(&caps) || level < 2) {
		return;
	}
	if (pool.va == NULL) {
		printf("optee: no shared memory pool (taken at boot): no session\n");
		return;
	}
	if (!(caps & SEC_CAP_DYNAMIC_SHM)) {
		printf("optee: no dynamic shared memory: this client only does NONCONTIG references, no session\n");
		return;
	}
	printf("optee: shared memory pool %#llx..%#llx\n", pool.pa, pool.pa + POOL_BYTES);
	fw_ready = preload_video_fw();
}

static void
optee_later(void *arg __unused, wait_result_t result __unused)
{
	if (delay_secs) {
		IOSleep(delay_secs * 1000);
	}
	optee_run();
}

void
hakios_optee_init(void)
{
	uint32_t n = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("optee", &n, sizeof(n)) || n == 0) {
		return;
	}
	level = (int)n;
	PE_parse_boot_argn("optee_delay", &delay_secs, sizeof(delay_secs));
	if (!PE_parse_boot_argn("optee_fw", fw_name, sizeof(fw_name)) || fw_name[0] == 0) {
		strlcpy(fw_name, "video_ucode.bin", sizeof(fw_name));
	}
	if (!PE_parse_boot_argn("optee_ta", ta_name, sizeof(ta_name))) {
		ta_name[0] = 0;
	}
	if (level >= 2) {
		/* contiguous memory now, while there is some: later in the boot there is none */
		vm_offset_t va = 0;

		if (kmem_alloc_contig(kernel_map, &va, POOL_BYTES, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
		    VM_KERN_MEMORY_IOKIT) == KERN_SUCCESS) {
			pool.va = (uint8_t *)va;
			pool.pa = kvtophys(va);
			pool.next_cookie = 1;
		}
	}
	if (kernel_thread_start(optee_later, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("optee: no thread\n");
	}
}

bool
hakios_optee_video_fw_ready(void)
{
	return fw_ready;
}

int
hakios_optee_load_video_fw(uint32_t index, uint32_t core, bool swap)
{
	uint64_t r[4];

	if (level == 0) {
		return -1;
	}
	if (!fw_ready) {
		printf("optee: load video fw %#x core %u before the package was preloaded\n", index, core);
	}
	fast(SMC_LOAD_VIDEO_FW, index, core, swap ? 1 : 0, r);
	if ((int32_t)r[0] < 0 || level >= 3) {
		printf("optee: load video fw %#x core %u swap %d: %d\n", index, core, swap, (int32_t)r[0]);
	}
	return (int32_t)r[0];
}
