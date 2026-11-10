/*
 * The Amlogic box's processor clock and core voltage (S7D, four Cortex-A55 on one clock and one rail), and the governor
 * that moves them with the load (amlcpu=1; amlcpu=2 with the governor off, the rate set by hand).
 *
 * The clock is the secure firmware's: SCMI's clock protocol (0x14) over SMC 0x820000c1 with the message in the shared
 * page at 0x050f6000 ("arm,scmi-smc"), clock 14 the cores' and 15 the DSU's (L3, the cores' shared logic). The rail is
 * a PWM regulator on the secure PWM J: SMC 0x820000c0 (0, 9, 0, high << 16 | low) sets its counts, 30 counts of the
 * 24 MHz crystal a period, 979 mV with the output always low and 10 mV less for each count high. Which of the device
 * tree's four voltage tables a chip takes is the firmware's word too (SMC 0x82000088, from the chip's fuses).
 *
 * Order (Amlogic's aml_cpufreq_v2.c): going up the voltage, then the DSU's clock, then the cores'; going down the
 * cores' clock, the DSU's, then the voltage. The voltage moves 30 mV at a time, 200 us apart.
 *
 * The governor: every 20 ms the busiest core's share of the time decides. Above 70% the top rate at once; otherwise
 * the lowest rate that would leave that core 60% busy, taken only after 200 ms have asked for no more. Asleep (the
 * HDMI driver's word) the clock stays at 500 MHz.
 *
 * Heat, as the stock Android's device tree has it (thermal-zones/soc_thermal, Linux's "power allocator"): the chip's
 * sensor (0xfe020000, its calibration the firmware's: SMC 0x82000047) read every second, every 100 ms from 85 C. From
 * there the cores get a power budget: 1160 mW, more below 95 C (232 mW a degree) and less above (116 mW a degree, and
 * 1 mW more off for each degree-tick spent above), and the highest rate whose power (230 * MHz * V^2, a core, by how
 * busy the four were) fits is the ceiling -- never below 1200 MHz, the device tree's eight steps. Under 85 C no
 * ceiling. At 105 C (Android powers off) the lowest rate; at 110 C the sensor itself resets the chip (the loader's
 * setting, left as found).
 *
 * The GPU (the Mali driver's clock levels: 285, 400, 500, 666, 800 MHz and 852, the vendor's top, from gp1_pll, SCMI
 * clock 17, which the firmware switches on for us; one voltage): sampled every 4 ms, and every
 * 48 ms its busy share decides, with the vendor's thresholds: above 47% a level up (from 90% the top at once), below
 * 31% for half a second a level down. Asleep the lowest. It shares the heat's budget with the cores as Linux divides
 * it, each by what it asks for (the GPU's power 140 * MHz / 1024 mW by its busy share, Amlogic's gpu_cooling.c), and
 * its ceiling comes down to 400 MHz at most.
 *
 * kern.hakios_cpu_khz (the rate, settable with the governor off), kern.hakios_cpu_gov, kern.hakios_cpu_min and _max,
 * kern.hakios_cpu_info, kern.hakios_cpu_temp (millidegrees; written: a temperature to pretend, 0 the sensor's again).
 * kern.hakios_gpu_mhz (settable with its governor off), kern.hakios_gpu_gov, kern.hakios_gpu_min and _max.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <kern/thread.h>
#include <kern/locks.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void *parameter, int priority,
    thread_t *new_thread);
extern void thread_deallocate(thread_t thread);
extern uint64_t mach_absolute_time(void);
extern int ml_get_max_cpu_number(void);
extern uint64_t hakios_cpu_idle_mach(int cpu);
extern int hakios_mali_active(void);
extern int hakios_mali_level(int to);

void hakios_amlcpu_init(void);
void hakios_amlcpu_sleep(bool asleep);

#define SCMI_SHMEM      0x050f6000UL
#define SCMI_SMC        0x820000c1UL
#define SCMI_CLOCK      0x14u
#define CLOCK_RATE_SET  0x5u
#define CLOCK_RATE_GET  0x6u
#define CLK_CPU         14u
#define CLK_DSU         15u

#define TABLE_SMC       0x82000088UL

#define PWM_SMC         0x820000c0UL
#define PWM_TEE_ID      9u
#define PWM_J           0xfe059000UL    /* (the page; the block at +0x200) */
#define PWM_J_REG       0x200
#define PWM_COUNTS      30u
#define UV_TOP          979000u
#define UV_STEP         10000u

#define SECMON_OUT      0x82000021UL    /* where the firmware leaves its answers */
#define TS_TRIM_SMC     0x82000047UL
#define TS_ID           1u
#define TS_BASE         0xfe020000UL
#define CLKCTRL          0xfe000000UL
#define CLKCTRL_TS       0x158           /* the sensor's clock: bit 8 on, 7-0 the crystal divided (less one) */
#define TS_CFG1         0x04
#define TS_CFG2         0x08
#define TS_STAT0        0x40
#define TS_A            270
#define TS_B            370
#define TS_C            2747
#define TS_D            9164

#define TEMP_ON         85000           /* millidegrees: the budget starts */
#define TEMP_TARGET     95000           /* and what it holds the chip at */
#define TEMP_CRITICAL   105000
#define TEMP_RESET      110000
#define SUSTAIN_MW      1160
#define DYN_COEFF       230
#define COOL_STEPS      8               /* rates below the top the ceiling may come down */

#define GPU_LEVELS      6
#define CLK_GP1         17u
#define CLOCK_CONFIG_SET 0x7u
#define GPU_COEFF       140
#define GPU_COOL_FLOOR  1               /* the level the heat's ceiling stops at */
#define GPU_WINDOW      12              /* samples (4 ms apart) a decision */
#define GPU_UP          47              /* per cent busy: the vendor's 0x78 and 0x50 of 255 */
#define GPU_DOWN        31
#define GPU_TOP         90
static const unsigned gpu_mhz[GPU_LEVELS] = { 285, 400, 500, 666, 800, 852 };

#define OPPS            14
#define SLEEP_OPP       2       /* 500 MHz (at 100 the box is too slow to answer the network) */
static const uint32_t opp_khz[OPPS] = { 100000, 250000, 500000, 666666, 1000000, 1200000, 1404000, 1608000, 1800000,
	                                2004000, 2208000, 2304000, 2400000, 2508000 };
static const uint32_t opp_uv[4][OPPS] = {
	{ 729000, 729000, 729000, 729000, 729000, 739000, 769000, 799000, 829000, 919000, 919000, 919000, 949000, 959000 },
	{ 729000, 729000, 729000, 729000, 729000, 739000, 769000, 799000, 829000, 919000, 919000, 919000, 949000, 959000 },
	{ 729000, 729000, 729000, 729000, 729000, 729000, 729000, 759000, 789000, 879000, 879000, 879000, 909000, 919000 },
	{ 709000, 709000, 709000, 709000, 709000, 709000, 709000, 719000, 749000, 829000, 829000, 829000, 859000, 869000 },
};
/* the DSU's rate for a cores' rate at or above the first column (below them all: the cores' own) */
static const uint32_t dsu_khz[][2] = { { 1200000, 1000000 }, { 1404000, 1128000 }, { 1608000, 1296000 },
	                               { 1800000, 1440000 }, { 2004000, 1872000 }, { 2400000, 2004000 } };

static struct {
	vm_offset_t shmem, pwm;
	lck_mtx_t *lock;
	unsigned table;
	unsigned token;
	int opp;                /* the rate now (index); -1 unknown */
	int min, max;           /* the governor's bounds */
	int gov;
	bool ready, asleep;
	unsigned changes, failures;
	uint32_t boot_khz, boot_uv;
	vm_offset_t ts;
	uint32_t trim;
	char sensor[120];       /* what the probe found */
	int temp, fake_temp;    /* millidegrees (0: no sensor); a temperature to pretend */
	int cap;                /* the heat's ceiling (index; OPPS - 1: none) */
	int64_t err_sum;
	int budget;             /* mW, the last */
	unsigned capped;        /* 100 ms periods with a ceiling */
	uint64_t ticks[OPPS];   /* the governor's 20 ms periods at each rate */
} c;

static struct {
	int level;              /* -1: no GPU */
	int min, max, cap;
	int gov;
	unsigned util;          /* per cent busy, the last window */
	unsigned changes;
	int granted;            /* mW of the heat's budget, the last */
	uint64_t ticks[GPU_LEVELS];
} g = { .level = -1, .max = GPU_LEVELS - 1, .cap = GPU_LEVELS - 1, .gov = 1 };

static uint64_t
smc(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3, uint64_t a4)
{
	register uint64_t x0 __asm__("x0") = function;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;
	register uint64_t x4 __asm__("x4") = a4;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3), "+r" (x4) : : "x5", "x6", "x7", "x8",
	    "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

/* one SCMI message: status (0 fine), the answer's words after the status in rx */
static int
scmi(uint32_t protocol, uint32_t message, const uint32_t *tx, unsigned ntx, uint32_t *rx, unsigned nrx)
{
	volatile uint32_t *s = (volatile uint32_t *)c.shmem;
	uint64_t r;

	s[1] = 0;                                       /* the channel: taken */
	s[4] = 0;                                       /* no interrupt for the answer */
	s[5] = 4 + ntx * 4;
	s[6] = message | protocol << 10 | (c.token++ & 0x3ff) << 18;
	for (unsigned i = 0; i < ntx; i++) {
		s[7 + i] = tx[i];
	}
	__builtin_arm_dsb(0xf);
	r = smc(SCMI_SMC, 0, 0, 0, 0);
	if (r != 0) {
		return (int)r ? (int)r : -1;
	}
	if (s[5] < 8) {
		return -1000;
	}
	for (unsigned i = 0; i < nrx && 8 + i * 4 < s[5]; i++) {
		rx[i] = s[8 + i];
	}
	return (int)s[7];
}

static int
clock_get(uint32_t id, uint32_t *khz)
{
	uint32_t rx[2] = { 0, 0 };
	int e = scmi(SCMI_CLOCK, CLOCK_RATE_GET, &id, 1, rx, 2);

	*khz = (uint32_t)((((uint64_t)rx[1] << 32) | rx[0]) / 1000);
	return e;
}

static int
clock_set(uint32_t id, uint32_t khz)
{
	uint64_t hz = (uint64_t)khz * 1000;
	uint32_t tx[4] = { 0, id, (uint32_t)hz, (uint32_t)(hz >> 32) };

	return scmi(SCMI_CLOCK, CLOCK_RATE_SET, tx, 4, NULL, 0);
}

/* the rail now, from the PWM's counts; 0 when they are not this regulator's */
static uint32_t
volt_get(void)
{
	uint32_t reg = *(volatile uint32_t *)(c.pwm + PWM_J_REG), high = reg >> 16, low = reg & 0xffff;

	if (high + low + 2 != PWM_COUNTS) {
		return 0;
	}
	return UV_TOP - (high + 1) * UV_STEP;
}

static int
volt_set(uint32_t uv)
{
	uint32_t now = volt_get();

	if (now == 0 || uv > UV_TOP - UV_STEP || uv < UV_TOP - (PWM_COUNTS - 1) * UV_STEP) {
		return -1;
	}
	while (now != uv) {
		uint32_t next = uv > now ? MIN(uv, now + 3 * UV_STEP) : MAX(uv, now - 3 * UV_STEP);
		uint32_t n = (UV_TOP - next + UV_STEP / 2) / UV_STEP;   /* counts high */

		if (smc(PWM_SMC, 0, PWM_TEE_ID, 0, (n - 1) << 16 | (PWM_COUNTS - n - 1)) != 0) {
			return -2;
		}
		IODelay(200);
		now = volt_get();
		if (now != UV_TOP - n * UV_STEP) {
			return -3;                              /* (the counts did not take) */
		}
	}
	return 0;
}

static uint32_t
dsu_for(uint32_t khz)
{
	for (int i = (int)(sizeof(dsu_khz) / sizeof(dsu_khz[0])) - 1; i >= 0; i--) {
		if (khz >= dsu_khz[i][0]) {
			return dsu_khz[i][1];
		}
	}
	return khz;
}

/* the sensor: millidegrees, 0 when it has nothing (Amlogic's code_to_temp, calibration type 1) */
static int
temp_read(void)
{
	int64_t code = *(volatile uint32_t *)(c.ts + TS_STAT0) & 0xffff, fuse = c.trim & 0xffff, t1, t2;

	if (c.fake_temp != 0) {
		return c.fake_temp;
	}
	if (code < 0x1500 || code > 0x3500) {
		return 0;
	}
	t1 = TS_A * code / 100;
	t2 = code * TS_B / 100 * 65536 / (65536 + t1);
	t1 = ((fuse & 0x8000) ? t2 - (fuse & 0x7fff) : t2 + fuse) * TS_D / 65536;
	return (int)((t1 - TS_C) * 100);
}

static uint32_t
opp_mw(int i)
{
	uint64_t mv = opp_uv[c.table][i] / 1000;

	return (uint32_t)((uint64_t)DYN_COEFF * (opp_khz[i] / 1000) * mv * mv / 1000000000);
}

/* the ceiling for this temperature and load (the four cores' busy shares summed, per mille), every 100 ms */
static void
thermal(unsigned load)
{
	int64_t err, p, i, next, max_c = 4 * (int64_t)opp_mw(OPPS - 1), max_g = 0, max, range, each, ask_c, ask_g = 0, all;
	int64_t give_c, give_g;
	int temp = temp_read(), cap;

	c.temp = temp;
	if (temp == 0 || temp < TEMP_ON) {
		c.err_sum = 0;
		c.cap = OPPS - 1;
		g.cap = GPU_LEVELS - 1;
		return;
	}
	if (g.level >= 0) {
		max_g = gpu_mhz[GPU_LEVELS - 1] * GPU_COEFF / 1024;
		ask_g = (int64_t)gpu_mhz[g.level] * GPU_COEFF * g.util / 102400;
	}
	max = max_c + max_g;
	c.capped++;
	if (temp >= TEMP_CRITICAL) {
		if (c.cap != 0) {
			printf("amlcpu: %d.%d C: the lowest rate\n", temp / 1000, temp % 1000 / 100);
		}
		c.cap = 0;
		g.cap = 0;
		return;
	}
	err = TEMP_TARGET - temp;
	p = err * (err < 0 ? 1 : 2) * SUSTAIN_MW / (TEMP_TARGET - TEMP_ON);
	i = c.err_sum / 100;
	if (err < 0) {
		next = i + err / 100;
		if (next > -max && next < max) {
			i = next;
			c.err_sum += err;
		}
	}
	range = MIN(MAX(SUSTAIN_MW + p + i, 0), max);
	c.budget = (int)range;
	if (range >= max) {
		c.cap = OPPS - 1;
		g.cap = GPU_LEVELS - 1;
		g.granted = (int)max_g;
		return;
	}
	/* each its share of what both ask for; what one cannot use goes to the other */
	ask_c = (int64_t)opp_mw(MAX(c.opp, 0)) * load / 1000;
	all = MAX(ask_c + ask_g, 1);
	give_c = ask_c * range / all;
	give_g = ask_g * range / all;
	if (give_c > max_c) {
		give_g = MIN(give_g + give_c - max_c, max_g);
		give_c = max_c;
	} else if (give_g > max_g) {
		give_c = MIN(give_c + give_g - max_g, max_c);
		give_g = max_g;
	}
	g.granted = (int)give_g;
	for (cap = GPU_LEVELS - 1; cap > GPU_COOL_FLOOR && gpu_mhz[cap] * GPU_COEFF > give_g * 1024 + 1023; cap--) {
	}
	g.cap = cap;
	each = give_c * 1000 / MAX(load, 1u);
	for (cap = OPPS - 1; cap > OPPS - 1 - COOL_STEPS && opp_mw(cap) > each; cap--) {
	}
	c.cap = cap;
}

static void thermal_init(void);

/* to a rate of the table (the lock held) */
static int
opp_set(int to)
{
	uint32_t khz = opp_khz[to], uv = opp_uv[c.table][to], got = 0;
	bool up = c.opp < 0 || to > c.opp;
	int e = 0;

	if (to == c.opp) {
		return 0;
	}
	if (up) {
		/* (never below what the rail has now while the clock is still the old one) */
		if (volt_get() < uv && (e = volt_set(uv)) != 0) {
			goto out;
		}
		(void)clock_set(CLK_DSU, dsu_for(khz));
	}
	e = clock_set(CLK_CPU, khz);
	if (e == 0) {
		e = clock_get(CLK_CPU, &got);
		if (e == 0 && (got > khz + 10000 || got + 10000 < khz)) {
			e = -4;
		}
	}
	if (e != 0) {
		goto out;
	}
	c.opp = to;
	c.changes++;
	if (!up) {
		(void)clock_set(CLK_DSU, dsu_for(khz));
		e = volt_set(uv);
	}
out:
	if (e != 0 && c.failures++ < 8) {
		printf("amlcpu: to %u kHz at %u uV: error %d (the clock %u kHz, the rail %u uV)\n", khz, uv, e, got, volt_get());
	}
	return e;
}

static int
opp_for(uint32_t khz)
{
	int i;

	for (i = 0; i < OPPS - 1 && opp_khz[i] < khz; i++) {
	}
	return i;
}

/* the GPU's clock (the lock held) */
static void
gpu_set(int to)
{
	if (to != g.level && hakios_mali_level(to) == to) {
		g.level = to;
		g.changes++;
	}
}

/* every 4 ms: the GPU busy or not; a window of them decides its clock */
static void
gpu_sample(void)
{
	static unsigned samples, busy, calm;
	int on = hakios_mali_active(), want;

	if (on < 0) {
		return;
	}
	if (g.level < 0) {
		uint32_t tx[2] = { CLK_GP1, 1 }, khz = 0;

		g.level = hakios_mali_level(-1);
		g.max = g.level;
		lck_mtx_lock(c.lock);
		if (g.level == GPU_LEVELS - 2 && scmi(SCMI_CLOCK, CLOCK_CONFIG_SET, tx, 2, NULL, 0) == 0 &&
		    clock_get(CLK_GP1, &khz) == 0 && khz == gpu_mhz[GPU_LEVELS - 1] * 1000) {
			IODelay(500);
			g.max = GPU_LEVELS - 1;
		}
		lck_mtx_unlock(c.lock);
		printf("amlcpu: gpu: %u MHz, the top %u MHz (gp1_pll %u kHz)\n", gpu_mhz[g.level], gpu_mhz[g.max], khz);
		return;
	}
	busy += on;
	if (++samples < GPU_WINDOW && !c.asleep) {
		return;
	}
	g.util = busy * 100 / samples;
	samples = busy = 0;
	g.ticks[g.level]++;
	want = g.level;
	if (c.asleep) {
		want = 0;
	} else if (g.gov) {
		if (g.util >= GPU_TOP) {
			want = g.max;
		} else if (g.util > GPU_UP) {
			want = g.level + 1;
		} else if (g.util < GPU_DOWN) {
			want = ++calm >= 10 ? g.level - 1 : g.level;
		}
		if (want != g.level || g.util >= GPU_DOWN) {
			calm = 0;
		}
		want = MIN(MAX(want, g.min), g.max);
	}
	want = MIN(want, g.cap);
	if (want != g.level) {
		lck_mtx_lock(c.lock);
		gpu_set(want);
		lck_mtx_unlock(c.lock);
	}
}

static void
governor(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t idle_before[8] = { 0 }, then = mach_absolute_time(), busy_sum = 0, span_sum = 0;
	unsigned calm = 0, tick = 0, wait = 0, tries = 0, sub = 0;
	int cpus = MIN(ml_get_max_cpu_number() + 1, 8);

	for (;;) {
		uint64_t now, span;
		unsigned busiest = 0;           /* per mille */
		int want;

		if (c.asleep) {
			IOSleep(20);
			gpu_sample();
		} else {
			IOSleep(4);
			gpu_sample();
			if (++sub % 5 != 0) {
				continue;
			}
		}
		now = mach_absolute_time();
		span = now - then;
		then = now;
		for (int cpu = 0; cpu < cpus; cpu++) {
			uint64_t idle = hakios_cpu_idle_mach(cpu), d;

			if (idle == ~0ull) {
				continue;
			}
			d = idle - idle_before[cpu];
			idle_before[cpu] = idle;
			if (span != 0 && d < span) {
				busiest = MAX(busiest, (unsigned)((span - d) * 1000 / span));
				busy_sum += span - d;
			}
		}
		span_sum += span;
		if (c.ts == 0 && tries < 5 && ++wait % 250 == 0) {
			tries++;
			thermal_init();
			printf("amlcpu: sensor: %s\n", c.sensor);
		}
		if (c.ts != 0 && ++tick % (c.temp >= TEMP_ON - 2000 ? 5 : 50) == 0) {
			thermal(span_sum != 0 ? (unsigned)(busy_sum * 1000 / span_sum) : 0);
			busy_sum = span_sum = 0;
		}
		if (c.opp > c.cap) {
			lck_mtx_lock(c.lock);
			(void)opp_set(c.cap);
			lck_mtx_unlock(c.lock);
		}
		if (c.opp >= 0) {
			c.ticks[c.opp]++;
		}
		if (!c.gov || c.opp < 0) {
			continue;
		}
		if (c.asleep) {
			want = SLEEP_OPP;
		} else if (busiest > 700) {
			want = c.max;
		} else {
			want = opp_for((uint32_t)((uint64_t)opp_khz[c.opp] * busiest / 600));
			want = MIN(MAX(want, c.min), c.max);
		}
		want = MIN(want, c.cap);
		if (want < c.opp && !c.asleep && ++calm < 10) {
			continue;                               /* down only once it has been calm a while */
		}
		calm = 0;
		if (want != c.opp) {
			lck_mtx_lock(c.lock);
			if (c.gov) {
				(void)opp_set(want);
			}
			lck_mtx_unlock(c.lock);
		}
	}
}

/* the HDMI driver's: tvOS asleep (the lowest rate) or awake again (the top rate at once, for the wake's work) */
void
hakios_amlcpu_sleep(bool asleep)
{
	if (!c.ready) {
		return;
	}
	c.asleep = asleep;
	if (!asleep) {
		lck_mtx_lock(c.lock);
		if (c.gov) {
			(void)opp_set(MIN(c.max, c.cap));
		}
		if (g.gov && g.level >= 0) {
			gpu_set(MIN(g.max, g.cap));
		}
		lck_mtx_unlock(c.lock);
	}
}

static int
khz_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	uint32_t khz = 0;
	int v, error;

	if (!c.ready) {
		return ENODEV;
	}
	lck_mtx_lock(c.lock);
	(void)clock_get(CLK_CPU, &khz);
	lck_mtx_unlock(c.lock);
	v = (int)khz;
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (c.gov) {
		return EBUSY;
	}
	lck_mtx_lock(c.lock);
	error = opp_set(MIN(opp_for((uint32_t)v), c.cap)) ? EIO : 0;
	lck_mtx_unlock(c.lock);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_khz, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, khz_sysctl, "I",
    "the cores' clock, kHz (settable with the governor off: the table's rate at or above)");

/* which: 0 the governor, 1 the lowest rate, 2 the highest */
static int
bound_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, int which, struct sysctl_req *req)
{
	int v = which == 0 ? c.gov : (int)opp_khz[which == 1 ? c.min : c.max], error = sysctl_handle_int(oidp, &v, 0, req);

	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (which == 0) {
		c.gov = v != 0;
	} else if (which == 1) {
		c.min = MIN(opp_for((uint32_t)v), c.max);
	} else {
		c.max = MAX(opp_for((uint32_t)v), c.min);
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_gov, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, bound_sysctl, "I",
    "1: the clock follows the load");
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_min, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 1, bound_sysctl, "I",
    "the governor's lowest rate awake, kHz");
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_max, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 2, bound_sysctl, "I",
    "the governor's highest rate, kHz");

static int
gpu_level_for(int mhz)
{
	int i;

	for (i = 0; i < GPU_LEVELS - 1 && (int)gpu_mhz[i] < mhz; i++) {
	}
	return i;
}

/* which: 0 the governor, 1 the lowest rate, 2 the highest, 3 the rate now */
static int
gpu_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, int which, struct sysctl_req *req)
{
	int v, error;

	if (g.level < 0) {
		return ENODEV;
	}
	v = which == 0 ? g.gov : (int)gpu_mhz[which == 1 ? g.min : which == 2 ? g.max : g.level];
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (which == 0) {
		g.gov = v != 0;
	} else if (which == 1) {
		g.min = MIN(gpu_level_for(v), g.max);
	} else if (which == 2) {
		g.max = MAX(gpu_level_for(v), g.min);
	} else if (g.gov) {
		return EBUSY;
	} else {
		lck_mtx_lock(c.lock);
		gpu_set(MIN(gpu_level_for(v), g.cap));
		lck_mtx_unlock(c.lock);
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_gpu_gov, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, gpu_sysctl, "I",
    "1: the GPU's clock follows its load");
SYSCTL_PROC(_kern, OID_AUTO, hakios_gpu_min, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 1, gpu_sysctl, "I",
    "the GPU governor's lowest rate, MHz");
SYSCTL_PROC(_kern, OID_AUTO, hakios_gpu_max, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 2, gpu_sysctl, "I",
    "the GPU governor's highest rate, MHz");
SYSCTL_PROC(_kern, OID_AUTO, hakios_gpu_mhz, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 3, gpu_sysctl, "I",
    "the GPU's clock, MHz (settable with its governor off)");

static int
temp_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int v = c.ts != 0 ? temp_read() : 0, error = sysctl_handle_int(oidp, &v, 0, req);

	if (!error && req->newptr != USER_ADDR_NULL) {
		c.fake_temp = v;
	}
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_temp, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, temp_sysctl, "I",
    "the chip's temperature, millidegrees (written: one to pretend; 0: the sensor's)");

static int
info_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char line[900];
	uint64_t all = 0, gall = 0;
	size_t n;
	uint32_t khz = 0, dsu = 0;

	if (!c.ready) {
		return ENODEV;
	}
	lck_mtx_lock(c.lock);
	(void)clock_get(CLK_CPU, &khz);
	(void)clock_get(CLK_DSU, &dsu);
	lck_mtx_unlock(c.lock);
	snprintf(line, sizeof(line), "cpu %u kHz dsu %u kHz rail %u uV table %u gov %d asleep %d changes %u failures %u "
	    "boot %u kHz %u uV", khz, dsu, volt_get(), c.table, c.gov, c.asleep, c.changes, c.failures, c.boot_khz, c.boot_uv);
	for (int i = 0; i < OPPS; i++) {
		all += c.ticks[i];
	}
	n = strlen(line);
	n += snprintf(line + n, sizeof(line) - n, "; %d mC ceiling %u MHz budget %d mW capped %u", c.temp, opp_khz[c.cap] / 1000,
	    c.budget, c.capped);
	n += snprintf(line + n, sizeof(line) - n, "; sensor: %s; time at", c.sensor);
	for (int i = 0; i < OPPS && all != 0; i++) {
		if (c.ticks[i] != 0) {
			n += snprintf(line + n, sizeof(line) - n, " %u:%u%%", opp_khz[i] / 1000, (unsigned)(c.ticks[i] * 100 / all));
		}
	}
	if (g.level >= 0) {
		n += snprintf(line + n, sizeof(line) - n, "; gpu %u MHz busy %u%% gov %d ceiling %u MHz granted %d mW changes %u, "
		    "time at", gpu_mhz[g.level], g.util, g.gov, gpu_mhz[g.cap], g.granted, g.changes);
		for (int i = 0; i < GPU_LEVELS; i++) {
			gall += g.ticks[i];
		}
		for (int i = 0; i < GPU_LEVELS && gall != 0; i++) {
			if (g.ticks[i] != 0) {
				n += snprintf(line + n, sizeof(line) - n, " %u:%u%%", gpu_mhz[i], (unsigned)(g.ticks[i] * 100 / gall));
			}
		}
	}
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_info, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, info_sysctl, "A",
    "the cores' clock, the rail and the governor");

/*
 * The sensor on, as Amlogic's driver has it (the loader leaves it off): its clock 500 kHz, the band gap, the ADC and
 * the filter; a reset of the chip from 110 C (the device tree's "rtemp"). From the governor's thread: it sleeps, and
 * the firmware's calibration is not there in the first seconds.
 */
static void
thermal_init(void)
{
	uint64_t out = smc(SECMON_OUT, 0, 0, 0, 0);
	vm_offset_t ts = ml_io_map(TS_BASE, 0x1000), clk = ml_io_map(CLKCTRL, 0x1000), page;
	uint32_t trim, version, fuse;
	int64_t t1, t2;

	if (out == 0 || out >= 0x80000000 || ts == 0 || clk == 0 || smc(TS_TRIM_SMC, TS_ID, 0, 0, 0) != 0 ||
	    (page = ml_io_map(out & ~0xfffUL, 0x1000)) == 0) {
		snprintf(c.sensor, sizeof(c.sensor), "no calibration (the firmware's buffer %#llx)", out);
		return;
	}
	trim = *(volatile uint32_t *)(page + (out & 0xfff));
	version = trim >> 24;
	if (!(version & 0x80) || (version & 0xf) >> 2 == 0) {
		snprintf(c.sensor, sizeof(c.sensor), "not calibrated (%#x)", trim);
		return;
	}
	*(volatile uint32_t *)(clk + CLKCTRL_TS) = 1 << 8 | (24000 / 500 - 1);
	/* the code for 110 C, rising (Amlogic's temp_to_code) */
	fuse = trim & 0xffff;
	t2 = (TS_C + TEMP_RESET / 100) * 65536 / TS_D;
	t2 = (fuse & 0x8000) ? t2 + (fuse & 0x7fff) : t2 - (fuse & 0x7fff);
	t1 = TS_B - TS_A * t2 / 65536;
	t2 = t2 * 100 / t1;
	*(volatile uint32_t *)(ts + TS_CFG2) |= (uint32_t)(((t2 >> 4) & 0xfff) + 1) << 4 | 1u << 31 | 1 << 30 | 0xff << 16;
	*(volatile uint32_t *)(ts + TS_CFG1) = (*(volatile uint32_t *)(ts + TS_CFG1) & ~(1u << 12 | 1 << 11)) | 1 << 15 |
	    1 << 10 | 1 << 9 | 1 << 5 | 1 << 3 | 3;
	IOSleep(50);
	c.trim = trim;
	c.ts = ts;
	c.temp = temp_read();
	snprintf(c.sensor, sizeof(c.sensor), "calibration %#x, control %#x %#x, reading %#x: %d mC", trim,
	    *(volatile uint32_t *)(ts + TS_CFG1), *(volatile uint32_t *)(ts + TS_CFG2), *(volatile uint32_t *)(ts + TS_STAT0), c.temp);
	if (c.temp < 5000 || c.temp > 120000) {
		c.ts = 0;
		c.temp = 0;
	}
}

/* the sensor by hand: "r OFF", "w OFF VAL" (its registers), "t ID" (the firmware's calibration for a sensor), "i" (probe again) */
static char ts_out[160];
static int
ts_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char in[48];
	vm_offset_t ts;
	char *e;

	if (req->newptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, ts_out, strlen(ts_out) + 1);
	}
	if (req->newlen >= sizeof(in) || SYSCTL_IN(req, in, req->newlen) != 0) {
		return EINVAL;
	}
	in[req->newlen] = 0;
	ts = ml_io_map(TS_BASE, 0x1000);
	if (in[0] == 'r' || in[0] == 'w') {
		unsigned long off = strtoul(in + 1, &e, 16) & 0xffc;

		if (in[0] == 'w') {
			*(volatile uint32_t *)(ts + off) = (uint32_t)strtoul(e, NULL, 16);
		}
		snprintf(ts_out, sizeof(ts_out), "%03lx=%08x", off, *(volatile uint32_t *)(ts + off));
	} else if (in[0] == 't') {
		uint64_t out = smc(SECMON_OUT, 0, 0, 0, 0), r = smc(TS_TRIM_SMC, strtoul(in + 1, NULL, 16), 0, 0, 0);
		volatile uint32_t *w = (volatile uint32_t *)(ml_io_map(out & ~0xfffUL, 0x1000) + (out & 0xfff));

		snprintf(ts_out, sizeof(ts_out), "out %#llx r %#llx: %08x %08x %08x %08x", out, r, w[0], w[1], w[2], w[3]);
	} else if (in[0] == 's') {
		uint32_t tx[5] = { 0 }, rx[8] = { 0 }, msg;
		unsigned n = 0;
		int r;

		msg = (uint32_t)strtoul(in + 1, &e, 16);
		while (n < 5 && *e == ' ') {
			tx[n++] = (uint32_t)strtoul(e, &e, 16);
		}
		lck_mtx_lock(c.lock);
		r = scmi(SCMI_CLOCK, msg, tx, n, rx, 8);
		lck_mtx_unlock(c.lock);
		snprintf(ts_out, sizeof(ts_out), "%d: %08x %08x %08x %08x %08x %08x %08x %08x", r, rx[0], rx[1], rx[2], rx[3],
		    rx[4], rx[5], rx[6], rx[7]);
	} else if (in[0] == 'a') {
		vm_offset_t ana = ml_io_map(0xfe008000UL, 0x1000);
		unsigned long off = strtoul(in + 1, &e, 16) & 0xffc;

		if (*e == ' ') {
			*(volatile uint32_t *)(ana + off) = (uint32_t)strtoul(e, NULL, 16);
		}
		snprintf(ts_out, sizeof(ts_out), "ana %03lx=%08x", off, *(volatile uint32_t *)(ana + off));
	} else if (in[0] == 'g') {
		extern unsigned hakios_mali_mhz(void);
		vm_offset_t clk = ml_io_map(0xfe000000UL, 0x1000);

		snprintf(ts_out, sizeof(ts_out), "gpu %u MHz measured, level %d, clock registers %08x %08x", hakios_mali_mhz(), hakios_mali_level(-1),
		    *(volatile uint32_t *)(clk + 0x15c), *(volatile uint32_t *)(clk + 0x160));
	} else if (in[0] == 'i') {
		thermal_init();
		snprintf(ts_out, sizeof(ts_out), "%s", c.sensor);
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, hakios_cpu_ts, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, ts_sysctl, "A",
    "the temperature sensor by hand");

void
hakios_amlcpu_init(void)
{
	unsigned int on = 0;
	uint32_t khz = 0, dsu = 0, uv;
	uint64_t table;
	thread_t thread;
	int e, e2;

	if (!PE_parse_boot_argn("amlcpu", &on, sizeof(on)) || on == 0) {
		return;
	}
	c.shmem = ml_io_map(SCMI_SHMEM, 0x1000);
	c.pwm = ml_io_map(PWM_J, 0x1000);
	c.lock = lck_mtx_alloc_init(lck_grp_alloc_init("amlcpu", LCK_GRP_ATTR_NULL), LCK_ATTR_NULL);
	if (!c.shmem || !c.pwm || c.lock == NULL) {
		printf("amlcpu: cannot map the firmware's page or the PWM\n");
		return;
	}
	table = smc(TABLE_SMC, 0, 0, 0, 0);
	e = clock_get(CLK_CPU, &khz);
	e2 = clock_get(CLK_DSU, &dsu);
	uv = volt_get();
	printf("amlcpu: table %lld; cores %u kHz (%d), DSU %u kHz (%d); PWM %#x: %u uV\n", (long long)table, khz, e, dsu, e2,
	    *(volatile uint32_t *)(c.pwm + PWM_J_REG), uv);
	if (table > 3 || e != 0 || khz < 100000 || uv == 0) {
		printf("amlcpu: not what this driver knows; the clock is left alone\n");
		return;
	}
	c.table = (unsigned)table;
	c.boot_khz = khz;
	c.boot_uv = uv;
	c.opp = opp_for(khz);
	if (opp_khz[c.opp] != khz || uv < opp_uv[c.table][c.opp]) {
		/* (between two rates, or a rail below the table's: the next change starts from "unknown" and raises first) */
		printf("amlcpu: the loader's rate is not the table's (%u kHz wants %u uV)\n", opp_khz[c.opp], opp_uv[c.table][c.opp]);
	}
	c.max = c.opp;
	c.min = MIN(opp_for(1000000), c.max);
	c.gov = on == 1;
	c.cap = OPPS - 1;
	c.ready = true;
	if (kernel_thread_start_priority(governor, NULL, 90, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
