/*
 * Game controllers for programs: there is no IOHIDFamily here (hakios.hidres.cpp reads the HID devices programs make), so
 * GameController never hears of a gamepad. The HID driver decodes a gamepad's reports into the state below and hands it
 * over; sysctl kern.hakios_gamepads gives the four slots to whoever asks -- a game's injected GameController stand-in
 * (tvos/src/twlpad.c) asks some 120 times a second.
 *
 * A read is also a claim: for half a second after one, the HID driver keeps a gamepad's buttons to the game and does not
 * turn them into the remote's keys (which a game would get as well); Home goes to tvOS whatever the claim.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_time.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#define GAMEPAD_SLOTS   4

/* as user space sees it: 56 bytes a slot */
struct hakios_gamepad {
	uint32_t seq;                   /* changes with every report */
	uint32_t connected;
	uint32_t buttons;               /* HAKIOS_PAD_* below, as hakios.hidres.cpp and twlpad.c number them */
	int16_t lx, ly, rx, ry;         /* -32767..32767, up and right positive */
	uint16_t l2, r2;                /* 0..65535 */
	char name[32];
};

static struct hakios_gamepad pads[GAMEPAD_SLOTS];
static uint64_t claimed_until;
static LCK_GRP_DECLARE(gamepad_grp, "hakios-gamepad");
static LCK_SPIN_DECLARE(gamepad_lock, &gamepad_grp);

int hakios_gamepad_attach(const char *name);
void hakios_gamepad_detach(int slot);
void hakios_gamepad_update(int slot, uint32_t buttons, int16_t lx, int16_t ly, int16_t rx, int16_t ry, uint16_t l2, uint16_t r2);
int hakios_gamepad_claimed(void);

int
hakios_gamepad_attach(const char *name)
{
	int slot = -1;

	lck_spin_lock(&gamepad_lock);
	for (int i = 0; i < GAMEPAD_SLOTS; i++) {
		if (!pads[i].connected) {
			memset(&pads[i], 0, sizeof(pads[i]));
			pads[i].connected = 1;
			pads[i].seq = 1;
			strlcpy(pads[i].name, name, sizeof(pads[i].name));
			slot = i;
			break;
		}
	}
	lck_spin_unlock(&gamepad_lock);
	if (slot >= 0) {
		printf("gamepad: \"%s\" is gamepad %d\n", name, slot);
	}
	return slot;
}

void
hakios_gamepad_detach(int slot)
{
	if (slot < 0 || slot >= GAMEPAD_SLOTS) {
		return;
	}
	lck_spin_lock(&gamepad_lock);
	pads[slot].connected = 0;
	pads[slot].seq++;
	lck_spin_unlock(&gamepad_lock);
	printf("gamepad: %d gone\n", slot);
}

void
hakios_gamepad_update(int slot, uint32_t buttons, int16_t lx, int16_t ly, int16_t rx, int16_t ry, uint16_t l2, uint16_t r2)
{
	if (slot < 0 || slot >= GAMEPAD_SLOTS) {
		return;
	}
	lck_spin_lock(&gamepad_lock);
	struct hakios_gamepad *p = &pads[slot];
	if (p->buttons != buttons || p->lx != lx || p->ly != ly || p->rx != rx || p->ry != ry || p->l2 != l2 || p->r2 != r2) {
		p->buttons = buttons;
		p->lx = lx;
		p->ly = ly;
		p->rx = rx;
		p->ry = ry;
		p->l2 = l2;
		p->r2 = r2;
		p->seq++;
	}
	lck_spin_unlock(&gamepad_lock);
}

int
hakios_gamepad_claimed(void)
{
	return mach_absolute_time() < claimed_until;
}

static int
gamepads_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	struct hakios_gamepad copy[GAMEPAD_SLOTS];
	uint64_t half;

	nanoseconds_to_absolutetime(500 * NSEC_PER_MSEC, &half);
	lck_spin_lock(&gamepad_lock);
	memcpy(copy, pads, sizeof(copy));
	claimed_until = mach_absolute_time() + half;
	lck_spin_unlock(&gamepad_lock);
	return SYSCTL_OUT(req, copy, sizeof(copy));
}

SYSCTL_PROC(_kern, OID_AUTO, hakios_gamepads, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED | CTLFLAG_ANYBODY,
    NULL, 0, gamepads_sysctl, "S,hakios_gamepad", "the game controllers: four slots of 56 bytes (tvos/src/twlpad.c reads them)");
