/*
 * The Amlogic box's watchdog (the SoC's own, RESETCTRL_WATCHDOG at 0xfe002100, "amlogic,meson-sc2-wdt"): armed at
 * boot and fed every second by a kernel thread, so that a box that hangs (a stalled bus stops every core) resets
 * itself after wdt=<seconds> (default 20, 0: off) instead of waiting for someone to pull its plug. Left at 65 s on
 * the way back to the loader, whose fastboot does not feed it.
 *
 * Registers (Linux's meson_gxbb_wdt.c): CTRL0 [25] divider on, [24] clock on, [21] reset the SoC when it fires,
 * [18] enable, [17:0] the 24 MHz crystal's divider (24000: millisecond ticks); TCNT [15:0] the timeout in ticks;
 * RESET: any write starts the count again.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);
extern kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
extern void thread_deallocate(thread_t thread);

void hakios_wdt_init(void);
void hakios_wdt_stop(void);

#define WDT_PAGE        0xfe002000UL
#define WDT_CTRL0       0x100
#define WDT_CTRL1       0x104
#define WDT_TCNT        0x108
#define WDT_RESET       0x10c

#define CTRL_CLKDIV_EN  (1u << 25)
#define CTRL_CLK_EN     (1u << 24)
#define CTRL_EE_RESET   (1u << 21)
#define CTRL_EN         (1u << 18)

static vm_offset_t wdt;
static volatile bool armed;

/*
 * Each processor's heartbeat: a thread bound to it ticks every 500 ms. The feeder feeds only while every processor
 * that has ticked once keeps ticking (a processor that never comes up is left out). One processor stalled on a bus read
 * (a decoder's register with its clock gated) took the interrupts with it, USB and Wi-Fi died, and the feeder on another
 * processor kept the box from resetting.
 */
#define BEAT_CPUS       8
#define BEAT_LOST_S     10
extern struct processor *cpu_to_processor(int cpu);
extern struct processor *thread_bind(struct processor *processor);
extern wait_result_t thread_block(thread_continue_t continuation);
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void *parameter, int priority,
    thread_t *new_thread);
extern int ml_get_max_cpu_number(void);
extern uint64_t mach_absolute_time(void);
extern void absolutetime_to_nanoseconds(uint64_t abstime, uint64_t *result);
static volatile uint64_t beat[BEAT_CPUS];

static void
wdt_beat(void *arg, __unused wait_result_t wr)
{
	int cpu = (int)(uintptr_t)arg;
	struct processor *p = cpu_to_processor(cpu);

	if (p == NULL) {
		return;
	}
	thread_bind(p);
	thread_block(THREAD_CONTINUE_NULL);     /* (now on that processor) */
	while (armed) {
		beat[cpu] = mach_absolute_time();
		IOSleep(500);
	}
}

/* every processor that has ticked has ticked lately */
static bool
beats_ok(void)
{
	uint64_t now = mach_absolute_time();

	for (int cpu = 0; cpu < BEAT_CPUS; cpu++) {
		uint64_t b = beat[cpu], ns;

		if (b == 0) {
			continue;
		}
		absolutetime_to_nanoseconds(now - b, &ns);
		if (ns > BEAT_LOST_S * 1000000000ull) {
			return false;
		}
	}
	return true;
}

static uint32_t wdt_rd(uint32_t off) { return *(volatile uint32_t *)(wdt + off); }
static void wdt_wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(wdt + off) = v; __builtin_arm_dsb(0xf); }

static void
wdt_feeder(__unused void *arg, __unused wait_result_t wr)
{
	bool beats_started = false;
	unsigned turns = 0;

	while (armed) {
		/* the heartbeats once the other processors are up (the watchdog is armed early in boot) */
		if (!beats_started && ++turns >= 30) {
			int n = ml_get_max_cpu_number() + 1;

			beats_started = true;
			for (int cpu = 0; cpu < n && cpu < BEAT_CPUS; cpu++) {
				thread_t t;

				if (kernel_thread_start_priority(wdt_beat, (void *)(uintptr_t)cpu, 95, &t) == KERN_SUCCESS) {
					thread_deallocate(t);
				}
			}
		}
		if (beats_ok()) {
			wdt_wr(WDT_RESET, 0);
		}
		IOSleep(1000);
	}
}

void
hakios_wdt_init(void)
{
	unsigned int seconds = 20;
	thread_t thread;

	(void)PE_parse_boot_argn("wdt", &seconds, sizeof(seconds));
	wdt = ml_io_map(WDT_PAGE, 0x1000);
	if (!wdt) {
		printf("wdt: cannot map the watchdog\n");
		return;
	}
	printf("wdt: as the loader left it: CTRL0 %#x CTRL1 %#x TCNT %#x\n", wdt_rd(WDT_CTRL0), wdt_rd(WDT_CTRL1),
	    wdt_rd(WDT_TCNT));
	if (seconds == 0) {
		return;
	}
	if (seconds > 65) {
		seconds = 65;   /* 16 bits of milliseconds */
	}
	wdt_wr(WDT_CTRL0, CTRL_EE_RESET | CTRL_CLK_EN | CTRL_CLKDIV_EN | 24000);
	wdt_wr(WDT_TCNT, seconds * 1000);
	wdt_wr(WDT_RESET, 0);
	armed = true;
	if (kernel_thread_start(wdt_feeder, NULL, &thread) != KERN_SUCCESS) {
		armed = false;
		printf("wdt: no thread to feed it; left off\n");
		return;
	}
	thread_deallocate(thread);
	wdt_wr(WDT_CTRL0, wdt_rd(WDT_CTRL0) | CTRL_EN);
	printf("wdt: armed, %u s: CTRL0 %#x TCNT %#x\n", seconds, wdt_rd(WDT_CTRL0), wdt_rd(WDT_TCNT));
}

/*
 * On the way back to the loader (no printf: the other processors are stopped, a console lock may be held): not off
 * but the longest timeout, 65 s, fed once. The way back and the loader's fastboot can hang too (a box once stopped
 * there for good with it off); the next kernel arms it again, and a box left waiting in fastboot just restarts.
 */
void
hakios_wdt_stop(void)
{
	if (!wdt || !armed) {
		return;
	}
	armed = false;          /* the feeder stops at its next turn */
	wdt_wr(WDT_TCNT, 65000);
	wdt_wr(WDT_RESET, 0);
}
