/*
 * cpu-sample=1: what the processors run. Every cpu-sample-us microseconds (default 4000) each processor's running thread is
 * noted by process, thread name and scheduling priority; each second the six most seen are logged with the share of samples
 * that found a processor idle: "cpus: t=SECONDS idle P% | process/thread pri N ...". For telling whether a thread that waits
 * for a processor (the Metal plug-in's mali-sched=1) waits behind someone, and whom.
 *
 * A diagnostic: the threads are read without holding them, so a name can be of a thread that has just ended.
 */
#include <kern/processor.h>
#include <kern/thread.h>
#include <kern/task.h>
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <pexpert/pexpert.h>
#include <kern/misc_protos.h>
#include <string.h>

extern char *proc_name_address(void *p);
extern void *get_bsdtask_info(task_t);
extern void thread_get_thread_name(thread_t th, char *name);

#define CPUSAMPLE_SLOTS 64

static struct cpusample_slot {
	char name[44];
	int pri;
	unsigned n;
} cpusample_slots[CPUSAMPLE_SLOTS];
static unsigned cpusample_idle, cpusample_total;
static int cpusample_event;

static void
cpusample_note(thread_t t)
{
	char proc[20] = "?", thread_name[MAXTHREADNAMESIZE] = "";
	char name[44];
	task_t task = get_threadtask(t);
	void *bsd = task ? get_bsdtask_info(task) : NULL;
	unsigned i;

	if (task == kernel_task) {
		strlcpy(proc, "kernel", sizeof(proc));
	} else if (bsd != NULL) {
		strlcpy(proc, proc_name_address(bsd), sizeof(proc));
	}
	thread_get_thread_name(t, thread_name);
	snprintf(name, sizeof(name), "%s/%.22s", proc, thread_name[0] ? thread_name : "-");
	for (i = 0; i < CPUSAMPLE_SLOTS && cpusample_slots[i].n != 0; i++) {
		if (strcmp(cpusample_slots[i].name, name) == 0) {
			break;
		}
	}
	if (i == CPUSAMPLE_SLOTS) {
		return;
	}
	if (cpusample_slots[i].n == 0) {
		strlcpy(cpusample_slots[i].name, name, sizeof(cpusample_slots[i].name));
	}
	cpusample_slots[i].n++;
	cpusample_slots[i].pri = t->sched_pri;
}

static void
cpusample_report(uint64_t seconds)
{
	char line[512];
	int at;

	if (cpusample_total == 0) {
		return;
	}
	at = snprintf(line, sizeof(line), "cpus: t=%llu idle %u%%", seconds, cpusample_idle * 100 / cpusample_total);
	for (int k = 0; k < 6; k++) {
		int best = -1;

		for (int i = 0; i < CPUSAMPLE_SLOTS; i++) {
			if (cpusample_slots[i].n != 0 && (best < 0 || cpusample_slots[i].n > cpusample_slots[best].n)) {
				best = i;
			}
		}
		if (best < 0 || at >= (int) sizeof(line) - 60) {
			break;
		}
		at += snprintf(line + at, sizeof(line) - at, " | %s pri %d %u%%", cpusample_slots[best].name, cpusample_slots[best].pri,
		    cpusample_slots[best].n * 100 / cpusample_total);
		cpusample_slots[best].n = 0;    /* (taken: the next round finds the next) */
	}
	printf("%s\n", line);
	bzero(cpusample_slots, sizeof(cpusample_slots));
	cpusample_idle = cpusample_total = 0;
}

static void
cpusample_loop(void *arg __unused, wait_result_t result __unused)
{
	uint32_t us = 4000;
	uint64_t second = 0;

	PE_parse_boot_argn("cpu-sample-us", &us, sizeof(us));
	if (us < 500) {
		us = 500;
	}
	for (;;) {
		uint64_t now, ns;

		assert_wait_timeout(&cpusample_event, THREAD_UNINT, us, NSEC_PER_USEC);
		thread_block(THREAD_CONTINUE_NULL);
		for (processor_t p = processor_list; p != NULL; p = p->processor_list) {
			thread_t t = p->active_thread;

			cpusample_total++;
			if (t == THREAD_NULL || t == p->idle_thread || t == current_thread()) {
				cpusample_idle++;
				continue;
			}
			cpusample_note(t);
		}
		now = mach_absolute_time();
		absolutetime_to_nanoseconds(now, &ns);
		if (ns / NSEC_PER_SEC != second) {
			if (second != 0) {
				cpusample_report(second);
			} else {
				bzero(cpusample_slots, sizeof(cpusample_slots));
				cpusample_idle = cpusample_total = 0;
			}
			second = ns / NSEC_PER_SEC;
		}
	}
}

/*
 * twl_canary_range=BASE+SIZE (twloader's twl_canary=1, an experiment): RAM the boot loader filled with a pattern (each 64-bit
 * word its own address ^ 0x5a5aa5a5c3c33c3c) and named a hole, so that nobody of the kernel's writes there. Every 10 s the
 * range is read back through an uncached mapping, and what changed is reported: which pages, the first word that differs.
 */
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void ml_io_unmap(vm_offset_t addr, vm_size_t sz);
extern void IOSleep(unsigned int milliseconds);

static uint64_t canary_base, canary_size;
#define CANARY_CHUNK (2u << 20)       /* mapped a piece at a time: io_map's space is far smaller than the range */

static uint64_t
canary_hex(const char **pp)
{
	const char *p = *pp;
	uint64_t v = 0;
	if (p[0] == '0' && p[1] == 'x') {
		p += 2;
	}
	for (;; p++) {
		if (*p >= '0' && *p <= '9') {
			v = v * 16 + (uint64_t)(*p - '0');
		} else if (*p >= 'a' && *p <= 'f') {
			v = v * 16 + (uint64_t)(*p - 'a' + 10);
		} else {
			break;
		}
	}
	*pp = p;
	return v;
}

static void
canary_loop(__unused void *arg, __unused wait_result_t wr)
{
	unsigned int rounds = 0, reported = 0;
	static uint8_t seen[4096];       /* a bit per 2 MB... per 4 KB page up to 128 MB a byte: pages already reported */

	for (;;) {
		IOSleep(10000);
		rounds++;
		uint64_t changed = 0, first = 0;
		vm_offset_t chunk = 0;
		for (uint64_t off = 0; off < canary_size; off += 4096) {
			if ((off % CANARY_CHUNK) == 0) {
				if (chunk) {
					ml_io_unmap(chunk, CANARY_CHUNK);
				}
				chunk = ml_io_map(canary_base + off, CANARY_CHUNK);
				if (!chunk) {
					break;
				}
			}
			const volatile uint64_t *w = (const volatile uint64_t *)(chunk + off % CANARY_CHUNK);
			uint64_t pa = canary_base + off;
			for (unsigned int i = 0; i < 512; i++) {
				uint64_t want = (pa + i * 8) ^ 0x5a5aa5a5c3c33c3cULL, got = w[i];
				if (got != want) {
					unsigned int bad = 1;
					for (unsigned int j = i + 1; j < 512; j++) {
						bad += w[j] != ((pa + j * 8) ^ 0x5a5aa5a5c3c33c3cULL);
					}
					changed++;
					if (!first) {
						first = pa;
					}
					uint64_t page = off >> 12;
					if (reported < 200 && page / 8 < sizeof(seen) * 8 / 8 && !(seen[(page / 8) % sizeof(seen)] & (1u << (page % 8)))) {
						seen[(page / 8) % sizeof(seen)] |= (uint8_t)(1u << (page % 8));
						reported++;
						printf("canary: page %#llx written (round %u): word %u = %#llx (was %#llx), %u of its 512 words differ\n",
						    (unsigned long long)pa, rounds, i, (unsigned long long)got, (unsigned long long)want, bad);
					}
					break;
				}
			}
		}
		if (chunk) {
			ml_io_unmap(chunk, CANARY_CHUNK);
		}
		printf("canary: round %u: %llu of %llu pages changed%s%#llx\n", rounds, (unsigned long long)changed,
		    (unsigned long long)(canary_size >> 12), changed ? ", the lowest " : "", (unsigned long long)first);
	}
}

static void
hakios_canary_start(void)
{
	char arg[64];
	const char *p = arg;
	thread_t thread;

	if (!PE_parse_boot_arg_str("twl_canary_range", arg, sizeof(arg))) {
		return;
	}
	canary_base = canary_hex(&p);
	if (*p++ != '+') {
		return;
	}
	canary_size = canary_hex(&p) & ~(uint64_t)(CANARY_CHUNK - 1);
	if (canary_size == 0) {
		return;
	}
	if (kernel_thread_start_priority(canary_loop, NULL, MINPRI_KERNEL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
		printf("canary: watching %#llx..%#llx for writes\n", (unsigned long long)canary_base, (unsigned long long)(canary_base + canary_size));
	}
}

void hakios_cpusample_start(void);

void
hakios_cpusample_start(void)
{
	uint32_t on = 0;
	thread_t thread;

	hakios_canary_start();

	if (!PE_parse_boot_argn("cpu-sample", &on, sizeof(on)) || on == 0) {
		return;
	}
	if (kernel_thread_start_priority(cpusample_loop, NULL, MAXPRI_KERNEL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
		printf("cpus: sampling the processors every %s\n", "cpu-sample-us (default 4 ms)");
	}
}

/*
 * The time a processor has spent idle since boot (Mach time units; ~0 for a processor that is not running), for the
 * Amlogic box's clock governor (bsd/dev/hakios.amlcpu.c).
 */
uint64_t hakios_cpu_idle_mach(int cpu);

uint64_t
hakios_cpu_idle_mach(int cpu)
{
	processor_t p = cpu_to_processor(cpu);
	struct recount_usage usage = { 0 };
	uint64_t idle = 0;

	if (p == PROCESSOR_NULL || p->state == PROCESSOR_OFF_LINE || p->state == PROCESSOR_SHUTDOWN ||
	    p->state == PROCESSOR_START) {
		return ~0ull;
	}
	recount_processor_usage(&p->pr_recount, &usage, &idle);
	return idle;
}
