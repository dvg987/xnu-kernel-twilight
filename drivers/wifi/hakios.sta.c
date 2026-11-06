/*
 * Darwin-on-hakios: what stands between the two sides of the station boundary
 * (sys/hakios.sta.h).
 *
 * Whoever wants a network asks from wherever it happens to be: an ioctl of wifid's, the thread
 * that brings the radio up. The station side wants to be called by one thread, one call at a
 * time, and a scan or a join takes seconds. So what is asked for is noted here and done by a
 * thread of this file's; the asker returns at once and hears the outcome as an event. One
 * request of each kind can wait: a second scan asked for while the first has not begun
 * replaces it, as does a second join.
 *
 * What the last scan heard is kept here, so that it can be read without entering the station
 * side, and so is the list of channels.
 *
 * The network the kernel was given in a file is a standing request: joined at the start, and
 * again whenever the radio is turned back on (Settings' switch), until somebody asks for
 * another network -- as Darwin itself rejoins the networks it knows. If that other network cannot
 * be joined (a wrong passphrase), the standing one is joined again.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/hakios.input.h>
#include <sys/hakios.sta.h>
#include <sys/systm.h>

extern void thread_set_thread_name(thread_t thread, const char *name);

#define HEARD_MOST      128
#define CHANNELS_MOST   96

static LCK_GRP_DECLARE(sta_lock_group, "hakios-sta");
static LCK_MTX_DECLARE(sta_lock, &sta_lock_group);

static const struct hakios_sta_ops *sta_ops;   /* the station side, while there is one */
static bool sta_thread_made;
static bool sta_calls;                          /* the thread is in a call of the station side's */
static unsigned sta_reads;                      /* ... and so many others are in its state() */
static bool sta_on;                             /* the radio's power, as last asked for */

/* what has been asked for and not begun */
static bool asked_power, asked_power_on, asked_scan, asked_join, asked_leave;
static struct hakios_sta_scan asked_scan_what;
static struct hakios_sta_join asked_join_what;
static struct hakios_sta_join standing;        /* the network joined whenever the radio comes on (ssid_length 0: none) */
static struct hakios_sta_join fallback;        /* ... as it was before a join that has not come through yet */
/*
 * The network of fw/network, kept whatever wifid asks for: when the link has been down for a while and nobody has
 * asked for a network, it is joined again. wifid leaves the network when tvOS goes to sleep (this radio has no
 * wake-on-wireless for it) and does not join again by itself on this port; without this the box falls off the
 * network for good after 15 minutes of nobody using it. If wifid leaves again soon after such a join, the wait
 * before the next one doubles (up to 10 minutes): the two take turns rather than fight.
 */
static struct hakios_sta_join configured;
static bool sta_link;                           /* joined, as far as the events say */
static unsigned rejoin_wait = 15;               /* seconds */
static uint64_t rejoined_at;                    /* (nanoseconds since boot) */

static uint64_t
sta_now_ns(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns;
}

static struct hakios_sta_bss *heard;           /* by the last scan that came to an end */
static unsigned heard_count;
static struct hakios_sta_channel channels[CHANNELS_MOST];
static unsigned channel_count;

static void
heard_one(const struct hakios_sta_bss *bss, __unused void *context)
{
	/* (only the thread writes the list, and it reads the count without the lock) */
	if (heard_count < HEARD_MOST) {
		lck_mtx_lock(&sta_lock);
		heard[heard_count++] = *bss;
		lck_mtx_unlock(&sta_lock);
	}
}

static void
sta_runs(__unused void *parameter, __unused wait_result_t result)
{
	static struct hakios_sta_scan scan;
	static struct hakios_sta_join join;

	for (;;) {
		const struct hakios_sta_ops *ops;
		bool power = false, power_on = false, leave = false, do_join = false, do_scan = false;
		int error = 0;

		lck_mtx_lock(&sta_lock);
		while (sta_ops == NULL || !(asked_power || asked_leave || asked_join || asked_scan)) {
			if (sta_ops != NULL && sta_on && !sta_link && configured.ssid_length != 0) {
				uint64_t deadline;

				nanoseconds_to_absolutetime((uint64_t)rejoin_wait * NSEC_PER_SEC, &deadline);
				deadline += mach_absolute_time();
				if (lck_mtx_sleep_deadline(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_ops, THREAD_UNINT, deadline) ==
				    THREAD_TIMED_OUT && sta_on && !sta_link && !(asked_power || asked_leave || asked_join || asked_scan)) {
					printf("sta: no network for %u s and nobody asked for one: the configured one again\n", rejoin_wait);
					asked_join_what = configured;
					asked_join = true;
					rejoined_at = sta_now_ns();
				}
			} else {
				lck_mtx_sleep(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_ops, THREAD_UNINT);
			}
		}
		ops = sta_ops;
		/* one thing at a time, in the order that makes sense when several wait */
		if (asked_power) {
			power = true;
			power_on = asked_power_on;
			asked_power = false;
		} else if (asked_leave) {
			leave = true;
			asked_leave = false;
		} else if (asked_join) {
			do_join = true;
			join = asked_join_what;
			bzero(&asked_join_what, sizeof(asked_join_what));
			asked_join = false;
		} else {
			do_scan = true;
			scan = asked_scan_what;
			asked_scan = false;
		}
		sta_calls = true;
		lck_mtx_unlock(&sta_lock);

		if (power) {
			struct hakios_sta_state *state = kalloc_data(sizeof(*state), Z_WAITOK | Z_ZERO);
			bool was_joined = false;

			if (state != NULL) {
				ops->state(state);
				was_joined = state->phase == HAKIOS_STA_JOINED;
				kfree_data(state, sizeof(*state));
			}
			error = ops->power(power_on);
			if (!power_on && was_joined) {
				hakios_apple80211_event(HAKIOS_STA_LINK, 0);
			}
			sta_link = sta_link && power_on;
			hakios_apple80211_event(HAKIOS_STA_POWER, error == 0 ? power_on : !power_on);
			lck_mtx_lock(&sta_lock);
			if (error == 0 && power_on && standing.ssid_length != 0 && !asked_join) {
				asked_join_what = standing;
				asked_join = true;
			}
			lck_mtx_unlock(&sta_lock);
		} else if (leave) {
			ops->leave();
			sta_link = false;
			hakios_apple80211_event(HAKIOS_STA_LINK, 0);
		} else if (do_join) {
			error = ops->join(&join);
			bzero(&join, sizeof(join));
			hakios_apple80211_event(HAKIOS_STA_JOIN_DONE, error);
			if (error == 0) {
				hakios_apple80211_event(HAKIOS_STA_LINK, 1);
			}
			sta_link = error == 0;
			/* a join that did not come through (a wrong passphrase) leaves things as they were: the standing network again */
			lck_mtx_lock(&sta_lock);
			if (error != 0 && fallback.ssid_length != 0 && !asked_join && !asked_leave && !asked_power) {
				standing = asked_join_what = fallback;
				asked_join = true;
			}
			bzero(&fallback, sizeof(fallback));
			lck_mtx_unlock(&sta_lock);
		} else if (do_scan) {
			error = ops->scan(&scan);
			lck_mtx_lock(&sta_lock);
			heard_count = 0;
			lck_mtx_unlock(&sta_lock);
			ops->heard(heard_one, NULL);
			hakios_apple80211_event(HAKIOS_STA_SCAN_DONE, error);
		}

		lck_mtx_lock(&sta_lock);
		sta_calls = false;
		lck_mtx_unlock(&sta_lock);
		thread_wakeup((event_t)&sta_calls);
	}
}

/* ---- the station side ---------------------------------------------------------------------------------- */

void
hakios_sta_attach(const struct hakios_sta_ops *ops)
{
	thread_t thread;

	if (ops == NULL) {
		lck_mtx_lock(&sta_lock);
		sta_ops = NULL;
		asked_power = asked_scan = asked_join = asked_leave = false;
		while (sta_calls || sta_reads != 0) {
			lck_mtx_sleep(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_calls, THREAD_UNINT);
		}
		heard_count = channel_count = 0;
		lck_mtx_unlock(&sta_lock);
		return;
	}
	if (heard == NULL) {
		heard = kalloc_data(HEARD_MOST * sizeof(*heard), Z_WAITOK | Z_ZERO);
	}
	if (heard == NULL) {
		hakios_wifi_tell("wifi: no memory for what a scan hears: nobody can ask the radio for a network\n");
		return;
	}
	if (!sta_thread_made) {
		if (kernel_thread_start(sta_runs, NULL, &thread) != KERN_SUCCESS) {
			hakios_wifi_tell("wifi: no thread for the station: nobody can ask the radio for a network\n");
			return;
		}
		thread_set_thread_name(thread, "wifi-station");
		thread_deallocate(thread);
		sta_thread_made = true;
	}
	channel_count = ops->channels(channels, CHANNELS_MOST);
	lck_mtx_lock(&sta_lock);
	sta_ops = ops;
	sta_on = true;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	hakios_apple80211_event(HAKIOS_STA_POWER, 1);
}

void
hakios_sta_lost(int error)
{
	sta_link = false;
	thread_wakeup((event_t)&sta_ops);
	hakios_apple80211_event(HAKIOS_STA_LINK, error != 0 ? error : ECONNRESET);
}

/* ---- who wants a network ------------------------------------------------------------------------------- */

bool
hakios_sta_present(void)
{
	return sta_ops != NULL;
}

bool
hakios_sta_powered(void)
{
	return sta_ops != NULL && sta_on;
}

bool
hakios_sta_busy(void)
{
	bool busy;

	lck_mtx_lock(&sta_lock);
	busy = sta_ops != NULL && (sta_calls || asked_power || asked_leave || asked_join || asked_scan);
	lck_mtx_unlock(&sta_lock);
	return busy;
}

int
hakios_sta_ask_power(bool on)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	sta_on = on;
	asked_power = true;
	asked_power_on = on;
	if (!on) {
		asked_scan = asked_join = asked_leave = false;
	}
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
hakios_sta_ask_scan(const struct hakios_sta_scan *what)
{
	lck_mtx_lock(&sta_lock);
	/* a scan asked for before the radio is there (wifid's first, at boot) waits for it: the thread runs it then */
	if (sta_ops != NULL && !sta_on) {
		lck_mtx_unlock(&sta_lock);
		return ENETDOWN;
	}
	asked_scan_what = *what;
	asked_scan = true;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
hakios_sta_ask_join_standing(const struct hakios_sta_join *what)
{
	int error = hakios_sta_ask_join(what);

	lck_mtx_lock(&sta_lock);
	if (error == 0) {
		standing = *what;
		configured = *what;
	}
	lck_mtx_unlock(&sta_lock);
	return error;
}

int
hakios_sta_ask_join(const struct hakios_sta_join *what)
{
	lck_mtx_lock(&sta_lock);
	/* (as a scan: a join asked for before the radio is there waits for it) */
	if (sta_ops != NULL && !sta_on) {
		lck_mtx_unlock(&sta_lock);
		return ENETDOWN;
	}
	if (standing.ssid_length != 0) {
		fallback = standing;
	}
	bzero(&standing, sizeof(standing));     /* (whoever asks for a network now has the say) */
	asked_join_what = *what;
	asked_join = true;
	asked_leave = false;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
hakios_sta_ask_leave(void)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	if (rejoined_at != 0 && sta_now_ns() - rejoined_at < 120 * NSEC_PER_SEC) {
		rejoin_wait = rejoin_wait * 2 > 600 ? 600 : rejoin_wait * 2;
		printf("sta: left soon after the configured network was joined again: the next try in %u s\n", rejoin_wait);
	} else {
		rejoin_wait = 15;
	}
	rejoined_at = 0;
	asked_leave = true;
	asked_join = false;
	bzero(&asked_join_what, sizeof(asked_join_what));
	bzero(&fallback, sizeof(fallback));
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

bool
hakios_sta_heard(unsigned index, struct hakios_sta_bss *bss)
{
	bool there;

	lck_mtx_lock(&sta_lock);
	if ((there = index < heard_count)) {
		*bss = heard[index];
	}
	lck_mtx_unlock(&sta_lock);
	return there;
}

void
hakios_sta_state(struct hakios_sta_state *state)
{
	const struct hakios_sta_ops *ops;

	bzero(state, sizeof(*state));
	lck_mtx_lock(&sta_lock);
	if ((ops = sta_ops) != NULL) {
		sta_reads++;
	}
	lck_mtx_unlock(&sta_lock);
	if (ops != NULL) {
		ops->state(state);
		lck_mtx_lock(&sta_lock);
		sta_reads--;
		lck_mtx_unlock(&sta_lock);
		thread_wakeup((event_t)&sta_calls);
	}
}

unsigned
hakios_sta_channels(struct hakios_sta_channel *out, unsigned most)
{
	unsigned count;

	lck_mtx_lock(&sta_lock);
	count = channel_count < most ? channel_count : most;
	memcpy(out, channels, count * sizeof(*out));
	lck_mtx_unlock(&sta_lock);
	return count;
}

bool
hakios_sta_address(uint8_t *address)
{
	const struct hakios_sta_ops *ops = sta_ops;

	if (ops != NULL) {
		memcpy(address, ops->address, 6);
	}
	return ops != NULL;
}
