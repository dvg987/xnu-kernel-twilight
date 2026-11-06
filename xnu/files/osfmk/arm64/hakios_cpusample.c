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

void hakios_cpusample_start(void);

void
hakios_cpusample_start(void)
{
	uint32_t on = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("cpu-sample", &on, sizeof(on)) || on == 0) {
		return;
	}
	if (kernel_thread_start_priority(cpusample_loop, NULL, MAXPRI_KERNEL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
		printf("cpus: sampling the processors every %s\n", "cpu-sample-us (default 4 ms)");
	}
}
