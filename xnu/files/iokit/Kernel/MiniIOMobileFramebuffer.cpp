/*
 * MiniIOMobileFramebuffer: a stand-in for Apple's display driver family, as seen by
 * IOMobileFramebuffer.framework (and through it by CoreAnimation's window server).
 *
 * The display is the framebuffer the boot loader left us (the one the kernel's text console
 * draws on). There is no hardware behind it that we drive: a "swap" copies the pixels of the
 * surfaces of its layers into that memory, and "vsync" is a timer.
 *
 * The protocol (selectors and the layout of the swap request) was worked out from the
 * user-space side of Darwin userland 16.5. Everything not implemented is logged once.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSString.h>
#include <kern/thread_call.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
static int gFbTrace;
SYSCTL_INT(_kern, OID_AUTO, hakios_fb_trace, CTLFLAG_RW | CTLFLAG_LOCKED, &gFbTrace, 0, "");

extern "C" {
#include <console/video_console.h>
extern struct vc_info vinfo;            /* the boot framebuffer, as the text console knows it */
}

#include <IOKit/IOBufferMemoryDescriptor.h>

#include "MiniIOSurface.h"

extern "C" void hakios_tl(const char * format, ...) __printflike(1, 2);
/* hakios.vplane.c: the video plane shows through where this frame has its key colour (alpha cleared there) */
extern "C" {
#include "../../bsd/sys/hakios.hdmitx.h"
}
extern "C" void hakios_vplane_osd_frame(uint8_t * pixels, uint32_t bytesPerRow, uint32_t width, uint32_t height);
/* ... and shows a swap's video layer under it (sys/hakios.vplane.h; the token is sys/hakios.vdec.h's) */
struct hakios_vplane_surface {
	const uint8_t * luma, * chroma;
	uint32_t lumaStride, chromaStride;
	uint32_t width, height;
	const void * token;
	uint32_t x, y, w, h;
};
extern "C" void hakios_vplane_surface(const struct hakios_vplane_surface * surface);
extern "C" void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
#define TWL_VPLANE_TOKEN_KEY    "TwlVideoPlanePicture"
#define TWL_VPLANE_TOKEN_SIZE   24

/* IOConnectCallMethod selectors of the framebuffer user client (Darwin userland 16.5) */
enum {
	kFBMethodGetLayerDefaultSurface = 3,
	kFBMethodSwapBegin              = 4,
	kFBMethodSwapEnd                = 5,
	kFBMethodSwapWait               = 6,
	kFBMethodGetID                  = 7,
	kFBMethodGetDisplaySize         = 8,
	kFBMethodRequestPowerChange     = 12,
	kFBMethodSwapSignal             = 20,
	kFBMethodSetDigitalOutMode      = 23,
	kFBMethodGetDigitalOutState     = 25,
	kFBMethodGetDotPitch            = 28,
	kFBMethodGetDisplayArea         = 29,
	kFBMethodSwapCancel             = 52,
	kFBMethodGetColorRemapMode      = 57,
	kFBMethodNotifications          = 72,
	kFBMethodChangeFrameInfo        = 73,
	kFBMethodSupportedFrameInfo     = 74,
	kFBMethodGetCurrentAbsoluteTime = 77,
	kFBMethodGetBlock               = 79,
	kFBMethodSwapCancelAll          = 81,
	kFBMethodGetProtectionOptions   = 82,
};

/* The swap request (IOMobileFramebufferSwapEnd): 0x46c bytes, of which we read these. */
#define SWAP_SIZE               0x46c
#define SWAP_ID                 0x50
#define SWAP_SURFACE(layer)     (0x54 + 4 * (layer))    /* surface number, 0 = none */
#define SWAP_SOURCE(layer)      (0x64 + 16 * (layer))   /* x, y, width, height in the surface */
#define SWAP_ENABLED(layer)     (0xa4 + 4 * (layer))
#define SWAP_DEST(layer)        (0xc4 + 16 * (layer))   /* x, y, width, height on the display */
#define SWAP_LAYERS_SET         0x104                   /* bit per layer given in this swap */
#define SWAP_LAYERS_CLEARED     0x108
#define SWAP_LAYERS             4

#define kFBNotifyVSync          5
#define kFBNotifyTypes          8
#define kFBRefreshHz            60                      /* unless boot-arg fb_hz says otherwise */

#define FRAME_TIMES             64                      /* milliseconds told apart between two frames */
#define FRAME_PAUSE             90                      /* from so many on, nothing was moving: not a slow frame */
#define SWAP_GPU_MS             500                     /* a frame the GPU has not drawn by then is shown as it is */
#define SWAP_HELD_MS            700                     /* a swap that cannot be taken by then is not shown at all */

#define FOURCC(a, b, c, d) ((uint32_t)(a) << 24 | (uint32_t)(b) << 16 | (uint32_t)(c) << 8 | (uint32_t)(d))

static uint64_t gFBLogged[2];           /* selectors already reported */
static uint8_t  gFBCalls[128];          /* how many calls of each selector were traced */

/*
 * The display user space is told about. The system carries the identity of the iPhone X
 * (see the loader), and Darwin knows what screen that phone has: with any other size
 * MobileGestalt reports an unknown screen class and the user interface has no layout for it.
 * So the display is always the iPhone's, and a swap scales the picture to the panel that is
 * really there (in QEMU the loader makes the panel exactly this size and nothing is scaled).
 * Boot-args fbwidth= and fbheight= change the size reported.
 */
#define kFBDefaultWidth         1125
#define kFBDefaultHeight        2436
#define kFBDotsPerInch          458

static TUNABLE(uint32_t, gFBWidth, "fbwidth", kFBDefaultWidth);
static TUNABLE(uint32_t, gFBHeight, "fbheight", kFBDefaultHeight);
/*
 * Boot-arg fbscale=2: the same 375 x 812 points drawn at two pixels a point instead of three.
 * Everything that is drawn is drawn by the processor, and this is 2.25 times fewer pixels to
 * draw; the picture is then enlarged to the panel and is that much less sharp.
 */
#define kFBPointsWide           375
#define kFBPointsHigh           812
#define kFBDefaultScale         3
static TUNABLE(uint32_t, gFBScale, "fbscale", 0);
/* Boot-arg fbtrace=N: log the first N calls of every selector (default 3). */
static TUNABLE(uint32_t, gFBTrace, "fbtrace", 3);
/*
 * IOMobileFramebufferGetDigitalOutState (selector 25): what a TV-out display (fb_external) says of its HDMI link.
 * CoreAnimation asks it twice at start and draws nothing while it is 0. Boot-arg fb_dostate=N, sysctl kern.fb_dostate.
 */
static uint32_t gDigitalOutState = 2;
SYSCTL_UINT(_kern, OID_AUTO, fb_dostate, CTLFLAG_RW | CTLFLAG_LOCKED, &gDigitalOutState, 0, "IOMFB digital out state (selector 25)");

/*
 * A TV out (fb_external, 2 by default): to CoreAnimation an Apple TV's HDMI display, with its modes' colour and HDR.
 * backboardd's hook (tvos/src/twlhook.c) keeps it drawing 1920x1080 frames, which the OSD scans and the VPP scales to the
 * timing. The hooks of backboardd and PineBoard read kern.fb_external.
 */
static uint32_t gTVOut;
SYSCTL_UINT(_kern, OID_AUTO, fb_external, CTLFLAG_RD | CTLFLAG_LOCKED, &gTVOut, 0, "IOMFB: the display is a TV out (1 external, 2 AppleTV)");
static TUNABLE(uint32_t, gTVSize, "fb_tvsize", 0);

/* the last "frames now" line (kern.fb_frames): what the window server draws, for when the log has gone round */
static char gFramesNow[160];
SYSCTL_STRING(_kern, OID_AUTO, fb_frames, CTLFLAG_RD | CTLFLAG_LOCKED, gFramesNow, sizeof(gFramesNow), "IOMFB: the frames last seen");

static void
shownSize(uint32_t fw, uint32_t fh, uint32_t * w, uint32_t * h)
{
	/* (boot-arg fb_tvsize=1: the timing's size; by default the framebuffer's, which backboardd's hook gives the window server) */
	uint32_t tw = gTVOut && gTVSize ? hakios_hdmitx_output_width() : 0, th = gTVOut && gTVSize ? hakios_hdmitx_output_height() : 0;

	*w = tw != 0 && th != 0 ? tw : fw;
	*h = tw != 0 && th != 0 ? th : fh;
}

/*
 * A recording of the screen, for a machine one cannot look at while it runs (the phone is
 * reached only through the boot loader, before and after the kernel).
 * Boot-args rec_secs=N [rec_start=S] [rec_mb=M]: from S seconds after the first frame, for N
 * seconds, every frame that is shown is also kept -- one pixel of every REC_STEP in each
 * direction, three bytes a pixel -- in a piece of memory of M megabytes (default 480, which
 * is 520 frames) that is left in place when the kernel returns to the loader. The loader
 * hands it to the host ("xl recording"), where tools/recording.py makes a video of it.
 *
 *   struct rec_header, then per frame: struct rec_frame, width * height pixels (R, G, B)
 *
 * Boot-arg rec_ring=1: when the memory is full the recording does not stop but goes on over its
 * oldest frames, so that what it holds at the end is the last of the run: for something that
 * happens once in a long session, the run is ended right after it (five fingers) and the frames
 * before it are there. The header ("RWREC003") says where the oldest frame is (frames are all
 * the same size, one after the other) and when the recording began, in the kernel's time, for the
 * frames to be matched with what the window server's per-frame lines (glframes=1,
 * userland/src/eagl.c) say. rec_step=N (2 to 8, default 3) keeps one pixel of every N instead of 3:
 * N = 4 holds 1.8 times as many frames, 6 four times. The memory is taken from the system's for the
 * whole run: 640 MB of it under a long session of applications had Darwin killing processes for
 * memory, SpringBoard and the window server among them (two respring in one run); 256 did not.
 */
static TUNABLE(uint32_t, gRecSeconds, "rec_secs", 0);
static TUNABLE(uint32_t, gRecStart, "rec_start", 0);
static TUNABLE(uint32_t, gRecMegabytes, "rec_mb", 480);
static TUNABLE(uint32_t, gRecRing, "rec_ring", 0);
static TUNABLE(uint32_t, gRecStep, "rec_step", 3);
static TUNABLE(uint32_t, gDiff, "fb_diff", 0);
/*
 * Boot-arg fb_filter=0: the picture is scaled to the panel nearest neighbour, as it once was. The
 * iPhone's 1125 x 2436 on this panel's 1080 x 2400 is 45 columns and 36 rows fewer: nearest
 * neighbour leaves out every 25th column and every 67th row, and where one of them goes through a
 * letter or a slanting line, that is a step in it or a stroke a pixel thinner -- text and lines a
 * little crooked, here and there and never in the same places twice. By default every pixel of the
 * panel is worked out from the four of the picture around where it falls (bilinear).
 */
static TUNABLE(uint32_t, gFilter, "fb_filter", 2);
/*
 * fb_filter=2 (the default): nearest neighbour while frames come, bilinear once they stop. Bilinear costs: a frame
 * took 3.6 ms to scale onto the panel as a rule and 11 ms for one in ten, at any priority (fb_pri), against 1.7 and
 * 4.3 by nearest neighbour (touch_lat=1, 30 October) -- and at 120 Hz a refresh is 8.3 ms. What nearest neighbour
 * gets wrong (a column in 25 and a row in 67 left out) does not show while the picture moves; so a frame drawn so is
 * drawn again, filtered, when no other has come for SETTLE_MS: what stands still on the screen is the bilinear one.
 */
#define SETTLE_MS       40
static TUNABLE(uint32_t, gRefreshHz, "fb_hz", kFBRefreshHz);
/*
 * Boot-arg fb_ca_hz=N: the refresh rate told to CoreAnimation (IOMFBDisplayRefresh) when it is not to be fb_hz's. On
 * this system CoreAnimation draws at half the rate it is told unless something asks it for more: told 120 it gave 60
 * (tools/frames.py: frames every 16.7 ms, with or without CADisableMinimumFrameDurationOnPhone, CA_FORCE_120HZ and
 * CA_ENABLE_HIGH_FRAME_RATE_RESTRICTION=0, 30 October). The refreshes themselves still come at fb_hz.
 */
static TUNABLE(uint32_t, gCARefreshHz, "fb_ca_hz", 0);
static TUNABLE(uint32_t, gOnTheBeat, "fb_beat", 1);
/*
 * Boot-arg fb_threads=N (1 to 8): so many processors scale a frame onto the panel together (see
 * drawRows()). Three by default: a frame is in the panel's memory 3 ms after its turn instead of 9
 * to 10 (the phone, 15 October; fb_threads=1 is how it was).
 */
static TUNABLE(uint32_t, gThreads, "fb_threads", 3);
/*
 * Boot-arg fb_pri=N: the scheduling priority of the threads that put frames on the panel (the presenter and its
 * helpers); 0 is the kernel's default for its threads. By default 92 (BASEPRI_PREEMPT): at the kernel's they lost the
 * processors to the window server's own real-time threads under load, and a frame took 3.6 ms to put on the panel
 * as a rule but 10.9 for one in ten (touch_lat=1, 30 October), more than the whole of a refresh at 120 Hz.
 */
static TUNABLE(uint32_t, gThreadPriority, "fb_pri", 92);
extern "C" kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void * parameter, integer_t priority,
    thread_t * new_thread);

/* a thread of the panel's, at fb_pri */
static kern_return_t
panel_thread_start(thread_continue_t continuation, void * parameter, thread_t * thread)
{
	if (gThreadPriority >= 1 && gThreadPriority <= 95) {
		return kernel_thread_start_priority(continuation, parameter, (integer_t) gThreadPriority, thread);
	}
	return kernel_thread_start(continuation, parameter, thread);
}
#define SWAP_QUEUE              2                       /* frames that may wait for the panel at most (fb_queue) */
static TUNABLE(uint32_t, gQueue, "fb_queue", 2);
static inline unsigned
swapQueue(void)
{
	return gQueue >= 1 && gQueue <= SWAP_QUEUE ? gQueue : 1;
}
/*
 * Boot-arg touch_lat=1: a line for every swap handed over ("tl: s ID at US") and for every frame
 * put on the panel ("tl: v ID ready US shown US done US": the GPU done with it, its beat come, the
 * pixels in the panel's memory), in microseconds of mach_absolute_time, for measuring how long a
 * touch takes to show (tools/latency.py; the touch's own lines: drivers/input/hakios.input.c).
 */
static TUNABLE(uint32_t, gTouchLat, "touch_lat", 0);

static inline uint64_t
micros(uint64_t absolute)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(absolute, &ns);
	return ns / 1000;
}

/*
 * Boot-arg fb_measure=1: how often the panel is really refreshed. The boot loader leaves the display controller running
 * and this driver only writes into the memory it scans, so fb_hz says how often frames are made, not how often the
 * panel shows one. The frame counter of the controller's DSI interface (SM7325's DPU: INTF_1 at 0x0ae36000,
 * INTF_FRAME_COUNT at 0xac, as Linux's catalogue for sc7280 has them) is read twice, a second apart, and the
 * difference printed. Reads only, once, a minute after the driver starts (what the kernel says in its first minute
 * does not come back with the phone's log), in a thread of its own; off by default.
 */
static TUNABLE(uint32_t, gMeasure, "fb_measure", 0);
static TUNABLE(uint32_t, gProbe, "fb_probe", 0);
/*
 * A panel of 24 bits a pixel (the Amlogic box: U-Boot leaves its display controller reading B, G, R from a frame of
 * fb_stride bytes a row): the frames are put together in 32 bits as everywhere else, in memory of the driver's own,
 * and each one is then packed into the panel's memory.
 */
static TUNABLE(uint32_t, gPanelBpp, "fb_bpp", 32);
static TUNABLE(uint32_t, gPanelStride, "fb_stride", 0);
#define DPU_INTF1_PAGE          0x0ae34000UL            /* the 16 KB page INTF_1 is in */
#define DPU_INTF1_OFFSET        0x2000
#define INTF_TIMING_ENGINE_EN   0x000
#define INTF_FRAME_COUNT        0x0ac
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
static inline uint32_t refreshHz(void);

static void
measurePanel(__unused void * arg, __unused wait_result_t wr)
{
	vm_offset_t intf;
	uint32_t on, first, second;

	IOSleep(60000);
	intf = ml_io_map(DPU_INTF1_PAGE, 0x4000) + DPU_INTF1_OFFSET;
	on = *(volatile uint32_t *)(intf + INTF_TIMING_ENGINE_EN);
	first = *(volatile uint32_t *)(intf + INTF_FRAME_COUNT);
	IOSleep(1000);
	second = *(volatile uint32_t *)(intf + INTF_FRAME_COUNT);
	IOLog("MiniIOMobileFramebuffer: the panel: DSI interface's timing engine %s, %u refreshes in one second "
	    "(frames are made at %u a second: fb_hz)\n", (on & 1) ? "on" : "OFF", second - first, refreshHz());
}

static inline uint32_t
refreshHz(void)
{
	return gRefreshHz >= 1 && gRefreshHz <= 240 ? gRefreshHz : kFBRefreshHz;
}
static inline uint32_t
recStep(void)
{
	return gRecStep >= 2 && gRecStep <= 8 ? gRecStep : 3;
}
#define REC_STEP        recStep()

struct rec_header {
	char     magic[8];              /* "RWREC003" */
	uint32_t width, height;
	uint32_t frames;
	uint32_t oldest;                /* the place of the oldest frame (rec_ring; 0 until the memory has once been full) */
	uint64_t began;                 /* when the first frame was recorded, in nanoseconds of the kernel's time */
};
struct rec_frame {
	uint64_t nanoseconds;           /* since the first frame of the recording */
};

extern "C" uint64_t loader_recording[3];        /* { physical address, bytes, kernel address }: osfmk/arm64/loader_return.c */

static uint32_t
swapWord(const uint8_t * swap, unsigned offset)
{
	uint32_t value;

	memcpy(&value, swap + offset, sizeof(value));
	return value;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

struct RowJob;

class IOMobileFramebuffer : public IOService
{
	OSDeclareDefaultStructors(IOMobileFramebuffer);

public:
	IOLock *        fLock;
	uint8_t *       fBase;                  /* the boot framebuffer: the panel */
	uint8_t *       fPacked;                /* fb_bpp=24: the panel itself, fBase being the 32-bit frame of ours */
	uint32_t        fPackedRowBytes;
	void            packFrame(void);
	uint32_t        fPanelWidth, fPanelHeight, fRowBytes;
	uint32_t        fWidth, fHeight;        /* the display as user space sees it */
	MiniIOSurface * fDefaultSurface;
	uint32_t        fSwapID;
	uint32_t        fSwapsShown;
	uint32_t        fLayersShown;           /* which layers the last swap had a surface in */
	bool            fGraphics;              /* the text console has been told to stop */
	/*
	 * A frame goes to the panel when the GPU is done with it: a thread of ours waits for that
	 * and copies it there, and whoever made the frame is back at the next one meanwhile. One
	 * frame waits at a time: the swap after it is held, in SwapEnd, until this one is being
	 * copied to the panel. Whoever draws is so at most two frames ahead of the panel, and
	 * needs a third surface to be that: CoreAnimation makes itself a new one for every frame.
	 * By default two frames may wait (boot-arg fb_queue=1: one, as before): CoreAnimation draws on a
	 * clock of its own, and a SwapEnd held past its next tick makes that tick late, the clock going
	 * on from there (rc.jobs.fps: 28% of the frames over 16.7 ms with two, 33% with one).
	 */
	IOLock *        fSwapLock;              /* around the next five */
	uint8_t         fSwap[SWAP_QUEUE][SWAP_SIZE];   /* the swaps on their way to the panel (fb_queue), from fSwapFirst on */
	MiniIOSurface * fSwapSurfaces[SWAP_QUEUE][SWAP_LAYERS]; /* ... their surfaces, held from SwapEnd on: their maker may let go of them */
	unsigned        fSwapFirst, fSwapsWaiting;
	/*
	 * ... and not whenever the GPU happens to be done, but on a beat of the refresh rate
	 * (boot-arg fb_beat=0: at once): frames then follow each other at an even pace whatever
	 * each took to draw, as long as there is one for every beat. The beat is this driver's own.
	 * (CoreAnimation does not go by the refresh notifications of this driver -- it never asks
	 * for them -- but by a clock of its own at the rate it reads here; and where the panel's
	 * scan is nobody can read: the display block's register clock is off.) It starts with a
	 * frame that comes later than a beat and a half after the one before -- the first after a
	 * pause, or every one of a renderer that does not keep up, the software one: such a frame
	 * is shown at once, a beat would only make it later still -- and gives way by an eighth of
	 * what a frame is early or late, so that frames are ready some six milliseconds before
	 * their beat: neither a slow one nor a quick one then lands on the wrong side of it.
	 */
	uint64_t        fBeat;                  /* the beat the last frame was shown on */
	uint64_t        fReady;                 /* when the last frame was ready to be shown */
	uint32_t        fStatOnBeat;            /* frames that waited for a refresh, and for how long together */
	uint32_t        fStatSettled;           /* frames drawn again, filtered, once they stood still (fb_filter=2) */
	uint64_t        fStatBeatWait;
	uint32_t        fStatSurfaces[8];       /* the first different surfaces the bottom layer was given */
	uint32_t        fStatHeld, fStatLost;   /* swaps that waited for the one before; that gave up waiting */
	uint64_t        fSwapTime;              /* when the last swap was handed over */
	uint32_t        fStatSwapsLate;         /* swaps that came more than a refresh and a quarter after the one before (and no pause) */
	uint64_t        fStatWaiting;           /* how long frames waited for the GPU here */
	/* how the display is doing: frames and time spent drawing them here since fStatSince */
	uint32_t        fStatFrames;
	uint64_t        fStatSince, fStatDrawing;
	/* ... and how evenly: the time from each frame to the next, counted by the millisecond (the last count: longer) */
	uint64_t        fLastFrame;
	uint32_t        fFrameTimes[FRAME_TIMES + 1], fFramePauses, fFrameLongest;
	/* boot-arg fb_diff=1: which pixels of the panel each frame changed (it costs: the panel's memory is slow to read) */
	bool            fDiff;
	uint64_t        fDiffPixels;
	uint32_t        fDiffLeft, fDiffTop, fDiffRight, fDiffBottom;
	/* the screen recording, if one was asked for */
	uint32_t *      fColumn;                /* drawLayer(): source column for every panel column */
	uint16_t *      fColumnWeight;          /* ... with fb_filter: how much of the column after it, of 256 */
	uint32_t *      fBlendRow;              /* ... and a row of the picture blended from two (fBlendRowSize pixels) */
	uint32_t        fBlendRowSize;
	uint8_t *       fRecBase;
	uint64_t        fRecSize, fRecUsed;
	uint64_t        fFirstFrame, fRecFirst; /* when the first frame was shown, and the first recorded */
	bool            fRecDone;
	bool            fRecWrapped;            /* (rec_ring) the memory has been full once: frames go over the oldest */
	uint64_t        fRecCost, fRecCostMax;  /* what copying the recorded frames took (mach_absolute_time), and the worst */
	uint32_t        fRecCosted;             /* ... of how many frames: a line every 120 */

	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	void publishModes(void);
	void publishRefresh(void);
	MiniIOSurface * defaultSurface(void);
	void present(const uint8_t * swap);
	void show(const uint8_t * swap, MiniIOSurface ** surfaces);
	void showVideo(const uint8_t * swap, MiniIOSurface * v, const int32_t src[4], const int32_t dst[4], const int32_t mainDst[4],
	    uint32_t mainW, uint32_t mainH);
	void settle(const uint8_t * swap, MiniIOSurface ** surfaces);
	bool fSettling;                 /* drawing the last frame again, filtered (fb_filter=2) */
	bool fScanning;                 /* fb_flip: the display is scanning a surface, not the panel's memory */
	uint32_t fStatFlips, fStatFlipsKept, fStatFlipsLeft;
	bool scanFrom(MiniIOSurface * s);
	void scanPanel(void);
	void presenter(void);
	static void presenterMain(void * self, wait_result_t result);
	void reportFrameTimes(void);
	void drawLayer(MiniIOSurface * s, const int32_t * src, const int32_t * dst);
	void drawRows(const struct RowJob * job, int32_t y0, int32_t y1, uint32_t * blendRow);
	static void helperMain(void * self, wait_result_t result);
	/* the scaling shared out (fb_threads): fThreads drawers, the presenter and fThreads - 1 helpers */
	unsigned        fThreads;
	IOLock *        fHelperLock;
	const struct RowJob * fHelperJob;
	uint64_t        fHelperGeneration;
	unsigned        fHelpersBusy, fHelpersStarted;
	uint32_t *      fHelperBlend[7];
	void record(MiniIOSurface * s);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

static void
setNumber(OSDictionary * dict, const char * key, uint64_t value)
{
	OSNumber * n = OSNumber::withNumber(value, 32);

	if (n != NULL) {
		dict->setObject(key, n);
		n->release();
	}
}

/*
 * The modes of the display, the way CoreAnimation reads them (the properties TimingElements
 * and ColorElements; without a mode it draws nothing). One timing, the panel's size at
 * 60 Hz (boot-arg fb_hz=N: at N), with one colour mode, 8-bit RGB. The identifiers are what "the current mode"
 * (block 0x40) answers with.
 */
#define kFBTimingID     1
#define kFBColorID      1

/*
 * The refresh the window server clocks its frames by: the property IOMFBDisplayRefresh, a dictionary of the shortest
 * interval between refreshes, the longest, and the step between them (QuartzCore, windowserver-apple-display.cpp:
 * displayMinRefreshInterval, displayRefreshStepMachTime; CoreMedia's FPSupport: displayMaxRefreshInterval,
 * displayRefreshStep). Without it CoreAnimation takes a built-in panel for a 60 Hz one whatever its timing says: the
 * mode it makes is "internal_panel (fixed)" at 60, UIScreen's maximumFramesPerSecond is 60, and frames came at 60 on
 * the 120 Hz panel (userland/src/fps_info.m, 30 October).
 * The intervals are in units of 2^-32 of a second (32.32 fixed point), not in mach_absolute_time's: given 1/120 s in
 * the phone's ticks (160000 of 19.2 MHz) CoreAnimation answered a refresh rate of 26844 a second, which is
 * 2^32 / 160000. Only the step whose name says MachTime is in ticks. This panel does not vary its rate, so all of
 * them are the one interval, fb_hz's.
 */
void
IOMobileFramebuffer::publishRefresh(void)
{
	OSDictionary * refresh = OSDictionary::withCapacity(4);
	uint32_t told = gCARefreshHz >= 1 && gCARefreshHz <= 480 ? gCARefreshHz : refreshHz();
	uint64_t interval = (1ULL << 32) / told, ticks;

	if (refresh == NULL) {
		return;
	}
	nanoseconds_to_absolutetime(NSEC_PER_SEC / told, &ticks);
	setNumber(refresh, "displayMinRefreshInterval", interval);
	setNumber(refresh, "displayMaxRefreshInterval", interval);
	setNumber(refresh, "displayRefreshStep", interval);
	setNumber(refresh, "displayRefreshStepMachTime", ticks);
	setProperty("IOMFBDisplayRefresh", refresh);
	refresh->release();

	/*
	 * The largest source a layer of a swap can have (QuartzCore's AppleDisplay reads IOMFBMaxSrcPixels; without it a
	 * layer more than 1920 wide is drawn into the frame by the GPU instead). The video plane takes up to 4096x2304 and
	 * scales it to its rectangle (hakios.vplane.c): a 1440p or 4K video layer goes to it as it is. (AppleDisplay keeps
	 * the smaller of the buffer's width and height and holds both of a layer's sides to it: so both 4096.)
	 */
	OSDictionary * limits = OSDictionary::withCapacity(4);
	if (limits != NULL) {
		setNumber(limits, "MaxSrcBufferWidth", 4096);
		setNumber(limits, "MaxSrcBufferHeight", 4096);
		setNumber(limits, "MaxSrcRectWidth", 4096);
		setNumber(limits, "MaxSrcRectTotal", 4096 * 2304);
		setNumber(limits, "MaxVideoSrcDownscalingWidth", 4096);
		setProperty("IOMFBMaxSrcPixels", limits);
		limits->release();
	}
	/*
	 * How far a layer of a swap may be scaled (QuartzCore's DisplayLimits::update_display_scaling: the smallest scale is
	 * 1 / *_MinScaleFraction, the largest *_MaxScale). Without it a video layer smaller than its picture -- YouTube's
	 * player shrunk beside its settings -- was drawn into the frame by the GPU, which converts every new video buffer
	 * into an image of its own: backboardd grew by 16 MB a frame to 634 MB and was killed. The video plane scales a
	 * picture down to a quarter and up as far as the screen; RGB layers other than the frame are not shown, so not those.
	 */
	OSDictionary * scaling = OSDictionary::withCapacity(4);
	if (scaling != NULL) {
		setNumber(scaling, "RGBLayer_MinScaleFraction", 1);
		setNumber(scaling, "RGBLayer_MaxScale", 1);
		setNumber(scaling, "YUVLayer_MinScaleFraction", 4);
		setNumber(scaling, "YUVLayer_MaxScale", 8);
		setProperty("IOMFBScalingLimits", scaling);
		scaling->release();
	}
	IOLog("MiniIOMobileFramebuffer: IOMFBDisplayRefresh: %u Hz (the panel: %u), an interval of %llu (2^-32 s), %llu ticks\n",
	    told, refreshHz(), interval, ticks);
}

/*
 * With a TV's EDID (hakios.hdmitx.c): one TimingElement per timing the TV lists that the box can drive, each with its
 * SDR colour elements (YCbCr 4:2:2 12-bit, 4:4:4, 4:2:0, RGB as the link allows), the IDs the HDMI driver uses, so
 * SetDigitalOutMode (selector 23) and the current mode (block 0x40) speak the same numbers. The display stays the
 * framebuffer's size (GetDisplaySize): CoreAnimation draws 1920x1080 and the VPU scales it to the timing.
 */
static OSDictionary *
colorElement(const struct hakios_hdmitx_color * k)
{
	OSDictionary * c = OSDictionary::withCapacity(8);

	if (c != NULL) {
		setNumber(c, "ID", k->id);
		setNumber(c, "PixelEncoding", k->encoding);
		setNumber(c, "Depth", k->depth);
		setNumber(c, "Colorimetry", k->colorimetry);
		setNumber(c, "EOTF", k->eotf);
		setNumber(c, "DynamicRange", k->range);
		setNumber(c, "Score", k->id == 3 ? 4 : k->id == 2 ? 3 : k->id == 4 ? 2 : 1);
		c->setObject("IsVirtual", kOSBooleanFalse);
	}
	return c;
}

static bool
publishTVModes(IOService * fb)
{
	unsigned count = 0;
	const struct hakios_hdmitx_timing * t = hakios_hdmitx_timings(&count);
	OSArray * timings, * colors;

	if (t == NULL || count == 0) {
		return false;
	}
	timings = OSArray::withCapacity(count);
	colors = OSArray::withCapacity(4);
	if (timings == NULL || colors == NULL) {
		OSSafeReleaseNULL(timings);
		OSSafeReleaseNULL(colors);
		return false;
	}
	for (unsigned c = 0; c < hakios_hdmitx_color_count(); c++) {    /* (SDR; HDR10 too with hdmitx_hdrmodes=1) */
		OSDictionary * e = colorElement(&hakios_hdmitx_colors[c]);

		if (e != NULL) {
			colors->setObject(e);
			e->release();
		}
	}
	for (unsigned i = 0; i < count; i++, t++) {
		OSDictionary * timing = OSDictionary::withCapacity(8);
		OSDictionary * horizontal = OSDictionary::withCapacity(4);
		OSDictionary * vertical = OSDictionary::withCapacity(4);
		OSArray * modes = OSArray::withCapacity(4);
		unsigned hz = t->rate >> 16;

		if (timing == NULL || horizontal == NULL || vertical == NULL || modes == NULL) {
			OSSafeReleaseNULL(timing);
			OSSafeReleaseNULL(horizontal);
			OSSafeReleaseNULL(vertical);
			OSSafeReleaseNULL(modes);
			continue;
		}
		for (unsigned c = 0; c < hakios_hdmitx_color_count(); c++) {
			if (t->colors & (1u << c)) {
				OSDictionary * e = colorElement(&hakios_hdmitx_colors[c]);

				if (e != NULL) {
					modes->setObject(e);
					e->release();
				}
			}
		}
		setNumber(horizontal, "Active", t->width);
		setNumber(horizontal, "Total", t->htotal);
		setNumber(horizontal, "SyncRate", t->rate);
		setNumber(horizontal, "PreciseSyncRate", t->rate);
		setNumber(vertical, "Active", t->height);
		setNumber(vertical, "Total", t->vtotal);
		setNumber(vertical, "SyncRate", t->rate);
		setNumber(vertical, "PreciseSyncRate", t->rate);
		setNumber(timing, "ID", t->id);
		setNumber(timing, "Score", (uint32_t) t->height * 1000 + (hz == 60 ? 300 : hz == 50 ? 200 : hz) + (t->fractional ? 0 : 1));
		timing->setObject("IsVirtual", kOSBooleanFalse);
		timing->setObject("IsOverscanned", kOSBooleanFalse);
		timing->setObject("HorizontalAttributes", horizontal);
		timing->setObject("VerticalAttributes", vertical);
		timing->setObject("ColorModes", modes);
		if (modes->getCount() != 0) {
			timings->setObject(timing);
		}
		timing->release();
		horizontal->release();
		vertical->release();
		modes->release();
	}
	fb->setProperty("TimingElements", timings);
	fb->setProperty("ColorElements", colors);
	{
		/* what tvOS reads of the TV (CADisplayAttributes: pqEOTF, bt2020YCC, hdrStaticMetadataType1, the product) */
		bool pq = false, hlg = false, bt2020 = false;
		const char * name = hakios_hdmitx_tv(&pq, &hlg, &bt2020);
		OSDictionary * attributes = OSDictionary::withCapacity(8);
		OSDictionary * product = OSDictionary::withCapacity(2);

		if (attributes != NULL && product != NULL) {
			attributes->setObject("SupportsPQEOTF", pq ? kOSBooleanTrue : kOSBooleanFalse);
			attributes->setObject("SupportsHLGEOTF", hlg ? kOSBooleanTrue : kOSBooleanFalse);
			attributes->setObject("SupportsBT2020YCC", bt2020 ? kOSBooleanTrue : kOSBooleanFalse);
			attributes->setObject("SupportsHDRStaticMetadataType1", pq ? kOSBooleanTrue : kOSBooleanFalse);
			attributes->setObject("SupportsDiscreteMediaRefreshRate", kOSBooleanFalse);
			if (name != NULL && name[0] != 0) {
				OSString * n = OSString::withCString(name);

				if (n != NULL) {
					product->setObject("ProductName", n);
					n->release();
				}
			}
			attributes->setObject("ProductAttributes", product);
			fb->setProperty("DisplayAttributes", attributes);
		}
		OSSafeReleaseNULL(attributes);
		OSSafeReleaseNULL(product);
	}
	IOLog("MiniIOMobileFramebuffer: %u modes of the TV published (TimingElements)\n", timings->getCount());
	timings->release();
	colors->release();
	return true;
}

static IOMobileFramebuffer * gModesFB;

/* the HDMI driver read an EDID or set a mode: the modes again (from its hot-plug thread) */
static void
tvChanged(void)
{
	if (gModesFB != NULL) {
		(void) publishTVModes(gModesFB);
	}
}

void
IOMobileFramebuffer::publishModes(void)
{
	OSDictionary * timing = OSDictionary::withCapacity(8);
	OSDictionary * horizontal = OSDictionary::withCapacity(4);
	OSDictionary * vertical = OSDictionary::withCapacity(4);
	OSDictionary * color = OSDictionary::withCapacity(8);
	OSArray * timings = OSArray::withCapacity(1);
	OSArray * colors = OSArray::withCapacity(1);
	const uint32_t rate = refreshHz() << 16;                /* 16.16 fixed point */

	if (timing == NULL || horizontal == NULL || vertical == NULL || color == NULL || timings == NULL || colors == NULL) {
		return;
	}
	gModesFB = this;
	hakios_hdmitx_set_listener(tvChanged);
	if (publishTVModes(this)) {
		publishRefresh();
		OSSafeReleaseNULL(timings);
		OSSafeReleaseNULL(colors);
		OSSafeReleaseNULL(timing);
		OSSafeReleaseNULL(horizontal);
		OSSafeReleaseNULL(vertical);
		OSSafeReleaseNULL(color);
		return;
	}
	setNumber(color, "ID", kFBColorID);
	setNumber(color, "PixelEncoding", 0);                   /* RGB 4:4:4 */
	setNumber(color, "Depth", 8);
	setNumber(color, "Colorimetry", 1);
	setNumber(color, "EOTF", 1);
	setNumber(color, "DynamicRange", 0);
	setNumber(color, "Score", 1);
	color->setObject("IsVirtual", kOSBooleanFalse);
	colors->setObject(color);

	setNumber(horizontal, "Active", fWidth);
	setNumber(horizontal, "Total", fWidth);
	setNumber(horizontal, "SyncRate", rate);
	setNumber(horizontal, "PreciseSyncRate", rate);
	setNumber(vertical, "Active", fHeight);
	setNumber(vertical, "Total", fHeight);
	setNumber(vertical, "SyncRate", rate);
	setNumber(vertical, "PreciseSyncRate", rate);
	setNumber(timing, "ID", kFBTimingID);
	setNumber(timing, "Score", 1);
	timing->setObject("IsVirtual", kOSBooleanFalse);
	timing->setObject("IsOverscanned", kOSBooleanFalse);
	timing->setObject("HorizontalAttributes", horizontal);
	timing->setObject("VerticalAttributes", vertical);
	timing->setObject("ColorModes", colors);
	timings->setObject(timing);

	setProperty("TimingElements", timings);
	setProperty("ColorElements", colors);
	publishRefresh();
	OSSafeReleaseNULL(timings);
	OSSafeReleaseNULL(colors);
	OSSafeReleaseNULL(timing);
	OSSafeReleaseNULL(horizontal);
	OSSafeReleaseNULL(vertical);
	OSSafeReleaseNULL(color);
}

static void gopInit(void * base, uint32_t width, uint32_t height, uint32_t rowBytes);   /* (below, with the GPU hand-off) */
static IOMobileFramebuffer * gScanner;  /* fb_flip: whose display the GPU's finished frames go to */

bool
IOMobileFramebuffer::start(IOService * provider)
{
	thread_t thread;

	if (!IOService::start(provider)) {
		return false;
	}
	if (vinfo.v_baseaddr == 0 || vinfo.v_width == 0 || vinfo.v_height == 0 || vinfo.v_depth != 32) {
		IOLog("MiniIOMobileFramebuffer: no usable boot framebuffer (%ux%u, depth %u)\n", vinfo.v_width, vinfo.v_height,
		    vinfo.v_depth);
		return false;
	}
	if (gMeasure && kernel_thread_start(measurePanel, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	fLock = IOLockAlloc();
	fSwapLock = IOLockAlloc();
	if (panel_thread_start(&IOMobileFramebuffer::presenterMain, this, &thread) != KERN_SUCCESS) {
		IOLog("MiniIOMobileFramebuffer: no thread to put frames on the panel with\n");
		return false;
	}
	thread_set_thread_name(thread, "frames to the panel");
	thread_deallocate(thread);
	fThreads = gThreads >= 1 && gThreads <= 8 ? gThreads : 1;
	fHelperLock = IOLockAlloc();
	for (unsigned i = 0; i + 1 < fThreads; i++) {
		if (panel_thread_start(&IOMobileFramebuffer::helperMain, this, &thread) != KERN_SUCCESS) {
			fThreads = i + 1;
			break;
		}
		thread_set_thread_name(thread, "frames to the panel: a helper");
		thread_deallocate(thread);
	}
	fBase = (uint8_t *) vinfo.v_baseaddr;
	fPanelWidth = vinfo.v_width;
	fPanelHeight = vinfo.v_height;
	fRowBytes = vinfo.v_rowbytes;
	gopInit((void *) vinfo.v_baseaddr, fPanelWidth, fPanelHeight, fRowBytes);
	gScanner = this;
	if (gPanelBpp == 24) {
		fPacked = fBase;
		fPackedRowBytes = gPanelStride ? gPanelStride : fPanelWidth * 3;
		fRowBytes = fPanelWidth * 4;
		fBase = (uint8_t *) IOMallocData((size_t) fRowBytes * fPanelHeight);
		if (fBase == NULL) {
			IOLog("MiniIOMobileFramebuffer: no memory for a 32-bit frame\n");
			return false;
		}
		bzero(fBase, (size_t) fRowBytes * fPanelHeight);
		IOLog("MiniIOMobileFramebuffer: the panel takes 24 bits a pixel, %u bytes a row: frames are packed into it\n",
		    fPackedRowBytes);
	}
	fWidth = gFBWidth ? gFBWidth : fPanelWidth;
	fHeight = gFBHeight ? gFBHeight : fPanelHeight;
	if (gFBScale != 0 && gFBScale != kFBDefaultScale) {
		static const char * const places[][2] = { { "/chosen", "display-scale" }, { "/product", "artwork-scale-factor" } };
		uint32_t scale = gFBScale;

		fWidth = kFBPointsWide * scale;
		fHeight = kFBPointsHigh * scale;
		/* what the loader's device tree says about the scale is where MobileGestalt has it from */
		for (unsigned i = 0; i < sizeof(places) / sizeof(places[0]); i++) {
			IORegistryEntry * node = IORegistryEntry::fromPath(places[i][0], gIODTPlane);

			if (node != NULL) {
				node->setProperty(places[i][1], &scale, sizeof(scale));
				node->release();
			}
		}
	}
	fColumn = IONewZero(uint32_t, fPanelWidth);
	fColumnWeight = IONewZero(uint16_t, fPanelWidth);
	fDiff = gDiff != 0;
	fDiffLeft = fDiffTop = ~0u;
	IOLog("MiniIOMobileFramebuffer: display %ux%u on a panel of %ux%u (%u bytes per row)\n", fWidth, fHeight,
	    fPanelWidth, fPanelHeight, fRowBytes);

	if (gRecSeconds != 0) {
		/* one piece of physical memory: the loader will send it over USB as it lies */
		IOBufferMemoryDescriptor * memory = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, (mach_vm_size_t) gRecMegabytes << 20, ~0ULL);

		if (memory != NULL) {
			struct rec_header * header = (struct rec_header *) memory->getBytesNoCopy();

			fRecBase = (uint8_t *) header;
			fRecSize = (uint64_t) gRecMegabytes << 20;
			fRecUsed = sizeof(*header);
			bzero(header, sizeof(*header));
			memcpy(header->magic, "RWREC003", 8);
			header->width = fWidth / REC_STEP;
			header->height = fHeight / REC_STEP;
			loader_recording[0] = memory->getPhysicalSegment(0, NULL);
			loader_recording[1] = fRecUsed;
			loader_recording[2] = (uint64_t) (uintptr_t) fRecBase;
			IOLog("MiniIOMobileFramebuffer: recording %u s of the screen at %ux%u from second %u, into %u MB at %#llx%s\n",
			    gRecSeconds, header->width, header->height, gRecStart, gRecMegabytes, loader_recording[0],
			    gRecRing ? ", the last of it kept (rec_ring)" : "");
		} else {
			IOLog("MiniIOMobileFramebuffer: no memory for a recording of %u MB\n", gRecMegabytes);
		}
	}

	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	/* for who draws with the GPU (userland/src/eagl.c): a frame need not be waited for before it is handed over */
	setProperty("HakiosWaitsForGPU", kOSBooleanTrue);
	publishModes();
	/*
	 * Boot-arg fb_external (2): the display is an Apple TV's HDMI display to IOMobileFramebuffer.framework (it searches
	 * the registry for a boolean "AppleTV"; 1: "external"), so CoreAnimation treats it as a TV: HDMI modes with colour
	 * and HDR. 0: a built-in panel (its modes all "fixed", SDR 8-bit).
	 */
	{
		uint32_t external = 2;

		(void) PE_parse_boot_argn("fb_external", &external, sizeof(external));
		(void) PE_parse_boot_argn("fb_dostate", &gDigitalOutState, sizeof(gDigitalOutState));
		gTVOut = external;
		if (external == 1) {
			setProperty("external", kOSBooleanTrue);
		} else if (external == 2) {
			setProperty("AppleTV", kOSBooleanTrue);           /* (what the framework looks for on an Apple TV's display) */
		}
	}
	setName("disp0");
	registerService();
	return true;
}

/*
 * Are the first `rows` rows of a surface, `columns` 32-bit pixels of each, inside the memory it
 * has? Its stride, element height and size come from whoever made it, so they are checked here,
 * before anything reads the rows: in 64 bits, so that nothing wraps.
 */
static bool
surfaceHolds(const MiniIOSurface * s, uint64_t rows, uint64_t columns)
{
	if (rows == 0 || columns == 0) {
		return true;
	}
	return (uint64_t) s->info.offset + (rows - 1) * s->info.bytesPerRow + columns * 4 <= s->info.allocSize;
}

/* Keep a small copy of the frame that is about to be shown (the surface is the whole screen). */
void
IOMobileFramebuffer::record(MiniIOSurface * s)
{
	struct rec_header * header = (struct rec_header *) fRecBase;
	uint64_t now = mach_absolute_time(), since_first, since_start = 0, frame_size;
	struct rec_frame * frame;
	uint8_t * to;

	if (fFirstFrame == 0) {
		fFirstFrame = now;
	}
	if (fRecBase == NULL || fRecDone || s->info.width < fWidth || s->info.height < fHeight) {
		return;
	}
	/* a surface whose rows do not fit in its memory is not read */
	if (!surfaceHolds(s, fHeight, fWidth)) {
		return;
	}
	absolutetime_to_nanoseconds(now - fFirstFrame, &since_first);
	if (since_first < (uint64_t) gRecStart * NSEC_PER_SEC) {
		return;
	}
	if (fRecFirst == 0) {
		uint64_t began;

		fRecFirst = now;
		absolutetime_to_nanoseconds(now, &began);
		header->began = began;
		IOLog("MiniIOMobileFramebuffer: the recording begins at %llu.%03llu s of the kernel's time (mach_absolute_time)\n",
		    began / NSEC_PER_SEC, began / NSEC_PER_MSEC % 1000);
	}
	absolutetime_to_nanoseconds(now - fRecFirst, &since_start);
	frame_size = sizeof(*frame) + (uint64_t) header->width * header->height * 3;
	if (since_start > (uint64_t) gRecSeconds * NSEC_PER_SEC || (fRecUsed + frame_size > fRecSize && !gRecRing)) {
		fRecDone = true;
		IOLog("MiniIOMobileFramebuffer: recording finished: %u frames, %llu MB\n", header->frames, fRecUsed >> 20);
		return;
	}
	if (fRecUsed + frame_size > fRecSize) {
		/* (rec_ring) round again: over the oldest, which is the next one */
		fRecUsed = sizeof(*header);
		fRecWrapped = true;
	}
	frame = (struct rec_frame *) (fRecBase + fRecUsed);
	frame->nanoseconds = since_start;
	to = (uint8_t *) (frame + 1);
	for (uint32_t y = 0; y < header->height; y++) {
		const uint32_t * from = (const uint32_t *) (s->pixels() + (size_t) (y * REC_STEP + 1) * s->info.bytesPerRow);

		for (uint32_t x = 0; x < header->width; x++) {
			uint32_t pixel = from[x * REC_STEP + 1];        /* B G R A in memory */

			*to++ = (uint8_t) (pixel >> 16);
			*to++ = (uint8_t) (pixel >> 8);
			*to++ = (uint8_t) pixel;
		}
	}
	fRecUsed += frame_size;
	/* what the copy cost: it runs in the display's thread, between one frame and the next */
	now = mach_absolute_time() - now;
	fRecCost += now;
	fRecCostMax = now > fRecCostMax ? now : fRecCostMax;
	if (++fRecCosted == 120) {
		IOLog("MiniIOMobileFramebuffer: recording a frame took %llu us on average, %llu at worst (the last 120)\n",
		    micros(fRecCost) / 120, micros(fRecCostMax));
		fRecCost = fRecCostMax = 0;
		fRecCosted = 0;
	}
	if (!fRecWrapped) {
		header->frames++;
		loader_recording[1] = fRecUsed;
	} else {
		/* the memory full of frames: all of them are handed over, from the one after this */
		header->oldest = (uint32_t) ((fRecUsed - sizeof(*header)) / frame_size);
		if (header->oldest >= header->frames) {
			header->oldest = 0;
		}
		loader_recording[1] = sizeof(*header) + (uint64_t) header->frames * frame_size;
	}
}

MiniIOSurface *
IOMobileFramebuffer::defaultSurface(void)
{
	IOLockLock(fLock);
	if (fDefaultSurface == NULL) {
		fDefaultSurface = MiniIOSurfaceCreateSimple(fWidth, fHeight, FOURCC('B', 'G', 'R', 'A'));
	}
	IOLockUnlock(fLock);
	return fDefaultSurface;
}

/*
 * Two pixels mixed: `w` of 256 of the second. Two of the four channels at a time, in one word.
 */
static inline uint32_t
blend(uint32_t a, uint32_t b, uint32_t w)
{
	uint32_t rb = ((a & 0xff00ff) * (256 - w) + (b & 0xff00ff) * w) >> 8 & 0xff00ff;
	uint32_t ga = ((a >> 8 & 0xff00ff) * (256 - w) + (b >> 8 & 0xff00ff) * w) & 0xff00ff00;

	return rb | ga;
}

/*
 * Where the middle of the `i`th of `out` pixels falls among `in` of them, in 256ths of a pixel,
 * kept within the first and the last.
 */
static inline int64_t
sample_at(int64_t i, int64_t in, int64_t out)
{
	int64_t at = ((2 * i + 1) * in - out) * 128 / out;

	return at < 0 ? 0 : at > (in - 1) * 256 ? (in - 1) * 256 : at;
}

/*
 * The rows of one layer, from `y0` to `y1` (panel rows), as drawLayer() has worked them out:
 * scaling a whole frame takes one processor some 9 ms, so with boot-arg fb_threads=N it is shared
 * out between N of them, each with rows of its own and a row of blending of its own.
 */
/* A layer's rectangle on the panel, in pixels, further out than this is not drawn (the panel is a few thousand wide). */
#define DST_LIMIT       (1 << 24)

struct RowJob {
	const uint8_t * pixels;
	uint32_t        stride;
	int32_t         sx, sy, sw, sh, px0, py0, px1, py1, cx0, cx1;
	int32_t         ry0, ry1;       /* the rows of the rectangle that are on the panel: all that is looked at, whatever its size */
	bool            filter, unscaled;
};

void
IOMobileFramebuffer::drawRows(const RowJob * job, int32_t y0, int32_t y1, uint32_t * blendRow)
{
	const uint8_t * pixels = job->pixels;
	uint32_t stride = job->stride;
	int32_t sx = job->sx, sy = job->sy, sw = job->sw, sh = job->sh;
	int32_t px0 = job->px0, py0 = job->py0, px1 = job->px1, py1 = job->py1, cx0 = job->cx0, cx1 = job->cx1;
	bool filter = job->filter;

	for (int32_t y = y0; y < y1; y++) {
		const uint32_t * from;
		uint32_t * to;

		if (y < 0 || (uint32_t) y >= fPanelHeight) {
			continue;
		}
		to = (uint32_t *) (fBase + (size_t) y * fRowBytes);
		if (job->unscaled) {
			/* not scaled at all */
			memcpy(to + px0, pixels + (size_t) (sy + y - py0) * stride + (size_t) sx * 4, (size_t) sw * 4);
			continue;
		}
		if (filter) {
			int64_t at = sample_at(y - py0, sh, py1 - py0);
			uint32_t row = (uint32_t) (at >> 8), weight = (uint32_t) (at & 255);
			const uint32_t * upper = (const uint32_t *) (pixels + (size_t) (sy + row) * stride) + sx;

			if (weight == 0 || row + 1 >= (uint32_t) sh) {
				from = upper;
			} else {
				const uint32_t * lower = (const uint32_t *) (pixels + (size_t) (sy + row + 1) * stride) + sx;
				uint32_t first = fColumn[cx0], last = fColumn[cx1 - 1] + 1 < (uint32_t) sw ? fColumn[cx1 - 1] + 1 : fColumn[cx1 - 1];

				uint32_t * __restrict mixed = blendRow;

				for (uint32_t c = first; c <= last; c++) {
					uint32_t a = upper[c], b = lower[c];

					mixed[c] = a == b ? a : blend(a, b, weight);           /* (most of a screen is the same as next to it) */
				}
				from = blendRow;
			}
		} else {
			from = (const uint32_t *) (pixels + (size_t) (sy + (int64_t) (y - py0) * sh / (py1 - py0)) * stride);
		}
		if (fDiff) {
			for (int32_t x = cx0; x < cx1; x++) {
				uint32_t value = filter ? blend(from[fColumn[x]], from[fColumn[x] + (fColumnWeight[x] != 0)], fColumnWeight[x]) :
				    from[fColumn[x]];

				if (to[x] != value) {
					fDiffPixels++;
					fDiffLeft = (uint32_t) x < fDiffLeft ? (uint32_t) x : fDiffLeft;
					fDiffRight = (uint32_t) x > fDiffRight ? (uint32_t) x : fDiffRight;
					fDiffTop = (uint32_t) y < fDiffTop ? (uint32_t) y : fDiffTop;
					fDiffBottom = (uint32_t) y > fDiffBottom ? (uint32_t) y : fDiffBottom;
				}
			}
		}
		if (filter) {
			const uint32_t * __restrict column = fColumn + cx0;
			const uint16_t * __restrict weights = fColumnWeight + cx0;
			uint32_t * __restrict out = to + cx0;

			for (uint32_t i = 0, n = (uint32_t) (cx1 - cx0); i < n; i++) {
				uint32_t c = column[i], w = weights[i], a = from[c];

				out[i] = w == 0 || a == from[c + 1] ? a : blend(a, from[c + 1], w);
			}
		} else {
			for (int32_t x = cx0; x < cx1; x++) {
				to[x] = from[fColumn[x]];
			}
		}
	}
}

/* A helper of the scaling (fb_threads): waits for rows to do, does them, says so. */
void
IOMobileFramebuffer::helperMain(void * self, wait_result_t result __unused)
{
	IOMobileFramebuffer * fb = (IOMobileFramebuffer *) self;
	unsigned index;
	uint64_t done = 0;

	IOLockLock(fb->fHelperLock);
	index = fb->fHelpersStarted++;
	IOLockUnlock(fb->fHelperLock);
	for (;;) {
		const RowJob * job;
		int32_t rows, y0, y1;
		uint32_t * blend;

		IOLockLock(fb->fHelperLock);
		while (fb->fHelperGeneration == done) {
			IOLockSleep(fb->fHelperLock, &fb->fHelperGeneration, THREAD_UNINT);
		}
		done = fb->fHelperGeneration;
		job = fb->fHelperJob;
		IOLockUnlock(fb->fHelperLock);
		/* share index + 1 of fThreads (the presenter takes share 0) */
		rows = job->ry1 - job->ry0;
		y0 = job->ry0 + (int32_t) ((int64_t) rows * (index + 1) / fb->fThreads);
		y1 = job->ry0 + (int32_t) ((int64_t) rows * (index + 2) / fb->fThreads);
		blend = fb->fHelperBlend[index];
		fb->drawRows(job, y0, y1, blend);
		IOLockLock(fb->fHelperLock);
		fb->fHelpersBusy--;
		if (fb->fHelpersBusy == 0) {
			IOLockWakeup(fb->fHelperLock, &fb->fHelpersBusy, true);
		}
		IOLockUnlock(fb->fHelperLock);
	}
}

/*
 * Put a rectangle of a 32-bit surface on the screen. `dst` is in display coordinates; the
 * panel may be of another size, and the layer itself may be scaled: both are done in one
 * step, bilinear (or nearest neighbour: fb_filter=0).
 */
void
IOMobileFramebuffer::drawLayer(MiniIOSurface * s, const int32_t * src, const int32_t * dst)
{
	int32_t sx = src[0], sy = src[1], sw = src[2], sh = src[3];
	int32_t dx = dst[0], dy = dst[1], dw = dst[2], dh = dst[3];
	int32_t px0, py0, px1, py1, cx0, cx1;
	bool filter = gFilter == 1 || (gFilter == 2 && fSettling);
	RowJob job;

	if (sw <= 0 || sh <= 0) {
		sx = sy = 0;
		sw = (int32_t) s->info.width;
		sh = (int32_t) s->info.height;
	}
	uint32_t shownW, shownH;

	shownSize(fWidth, fHeight, &shownW, &shownH);
	if (dw <= 0 || dh <= 0) {
		dx = dy = 0;
		dw = (int32_t) shownW;
		dh = (int32_t) shownH;
	}
	if (sx < 0 || sy < 0 || (uint32_t) (sx + sw) > s->info.width || (uint32_t) (sy + sh) > s->info.height) {
		return;
	}
	/* the rows and columns named must be in the surface's memory, whatever stride and size it was made with */
	if (!surfaceHolds(s, (uint32_t) (sy + sh), (uint32_t) (sx + sw))) {
		return;
	}
	/*
	 * the rectangle on the panel, worked out in 64 bits: the request's numbers are the caller's, and a
	 * rectangle of billions of rows is not drawn (nor looked at row by row)
	 */
	int64_t qx0 = (int64_t) dx * fPanelWidth / shownW;
	int64_t qy0 = (int64_t) dy * fPanelHeight / shownH;
	int64_t qx1 = ((int64_t) dx + dw) * fPanelWidth / shownW;
	int64_t qy1 = ((int64_t) dy + dh) * fPanelHeight / shownH;

	if (qx0 < -DST_LIMIT || qy0 < -DST_LIMIT || qx1 > DST_LIMIT || qy1 > DST_LIMIT) {
		return;
	}
	px0 = (int32_t) qx0;
	py0 = (int32_t) qy0;
	px1 = (int32_t) qx1;
	py1 = (int32_t) qy1;
	if (px1 <= px0 || py1 <= py0) {
		return;
	}
	/* nothing of it on the panel: nothing to draw (and the columns below are only worked out for ones that are) */
	if (px1 <= 0 || px0 >= (int32_t) fPanelWidth || py1 <= 0 || py0 >= (int32_t) fPanelHeight) {
		return;
	}
	/* (blending, a row of the picture mixed from two: room for it, for every one who draws rows) */
	if (filter && fBlendRowSize < (uint32_t) sw) {
		if (fBlendRow != NULL) {
			IODelete(fBlendRow, uint32_t, fBlendRowSize);
		}
		for (unsigned i = 0; i + 1 < fThreads; i++) {
			if (fHelperBlend[i] != NULL) {
				IODelete(fHelperBlend[i], uint32_t, fBlendRowSize);
			}
		}
		fBlendRowSize = (uint32_t) sw;
		fBlendRow = IONew(uint32_t, fBlendRowSize);
		bool allocated = fBlendRow != NULL;

		for (unsigned i = 0; i + 1 < fThreads; i++) {
			fHelperBlend[i] = IONew(uint32_t, fBlendRowSize);
			allocated = allocated && fHelperBlend[i] != NULL;
		}
		if (!allocated) {
			/* every array that did get made is given back at the size it was made with, so that no later growth frees one with the wrong size */
			if (fBlendRow != NULL) {
				IODelete(fBlendRow, uint32_t, fBlendRowSize);
				fBlendRow = NULL;
			}
			for (unsigned i = 0; i + 1 < fThreads; i++) {
				if (fHelperBlend[i] != NULL) {
					IODelete(fHelperBlend[i], uint32_t, fBlendRowSize);
					fHelperBlend[i] = NULL;
				}
			}
			fBlendRowSize = 0;
		}
	}
	filter = filter && fBlendRow != NULL;
	/* which pixel of a source row each pixel of a panel row takes: worked out once, not 2.6 million times */
	cx0 = px0 < 0 ? 0 : px0;
	cx1 = (uint32_t) px1 > fPanelWidth ? (int32_t) fPanelWidth : px1;
	for (int32_t x = cx0; x < cx1; x++) {
		if (filter) {
			int64_t at = sample_at(x - px0, sw, px1 - px0);

			fColumn[x] = (uint32_t) (at >> 8);              /* (in the blended row: from sx on) */
			fColumnWeight[x] = (uint16_t) (fColumn[x] + 1 < (uint32_t) sw ? at & 255 : 0);
		} else {
			fColumn[x] = (uint32_t) (sx + (int64_t) (x - px0) * sw / (px1 - px0));
		}
	}
	job.pixels = s->pixels();
	job.stride = s->info.bytesPerRow;
	job.sx = sx;
	job.sy = sy;
	job.sw = sw;
	job.sh = sh;
	job.px0 = px0;
	job.py0 = py0;
	job.px1 = px1;
	job.py1 = py1;
	job.cx0 = cx0;
	job.cx1 = cx1;
	job.ry0 = py0 < 0 ? 0 : py0;
	job.ry1 = py1 > (int32_t) fPanelHeight ? (int32_t) fPanelHeight : py1;
	job.filter = filter;
	job.unscaled = px1 - px0 == sw && py1 - py0 == sh && px0 >= 0 && (uint32_t) px1 <= fPanelWidth;
	if (fThreads <= 1 || fDiff || fHelpersStarted + 1 < fThreads) {
		drawRows(&job, job.ry0, job.ry1, fBlendRow);
		return;
	}
	/* shared out: the helpers take their rows, this thread the first share, then waits for them */
	IOLockLock(fHelperLock);
	fHelperJob = &job;
	fHelpersBusy = fThreads - 1;
	fHelperGeneration++;
	IOLockWakeup(fHelperLock, &fHelperGeneration, false);
	IOLockUnlock(fHelperLock);
	drawRows(&job, job.ry0, job.ry0 + (int32_t) ((int64_t) (job.ry1 - job.ry0) / fThreads), fBlendRow);
	IOLockLock(fHelperLock);
	while (fHelpersBusy != 0) {
		IOLockSleep(fHelperLock, &fHelpersBusy, THREAD_UNINT);
	}
	IOLockUnlock(fHelperLock);
}

/*
 * How evenly the frames of the last seconds came: how many followed the one before within the
 * time of one refresh at 120, 60, 40 and 30 a second, and the times half and 95 in 100 of them
 * stayed under. With the lock held.
 */
void
IOMobileFramebuffer::reportFrameTimes(void)
{
	static const uint32_t limits[] = { 10, 18, 26, 35, FRAME_TIMES + 1 };
	uint32_t within[sizeof(limits) / sizeof(limits[0])] = { 0 }, total = 0, sum = 0, half = 0, most = 0;

	for (uint32_t ms = 0; ms <= FRAME_TIMES; ms++) {
		for (uint32_t i = 0; i < sizeof(limits) / sizeof(limits[0]); i++) {
			if (ms < limits[i]) {
				within[i] += fFrameTimes[ms];
				break;
			}
		}
		total += fFrameTimes[ms];
	}
	for (uint32_t ms = 0; ms <= FRAME_TIMES && total != 0; ms++) {
		sum += fFrameTimes[ms];
		if (half == 0 && 2 * sum >= total) {
			half = ms + 1;
		}
		if (most == 0 && 100 * sum >= 95 * total) {
			most = ms + 1;
		}
	}
	if (total != 0) {
		IOLog("MiniIOMobileFramebuffer: from one frame to the next: %u times under 10 ms, %u under 18, %u under 26, %u under 35, "
		    "%u longer (the longest %u ms); half under %u ms, 95 in 100 under %u ms; %u pauses\n", within[0], within[1],
		    within[2], within[3], within[4], fFrameLongest, half, most, fFramePauses);
	}
	bzero(fFrameTimes, sizeof(fFrameTimes));
	fFramePauses = fFrameLongest = 0;
}

/* SwapEnd: the swap is handed to the thread that puts frames on the panel, as soon as that has no other waiting. */
/* The 32-bit frame into the 24-bit panel: four pixels into three words, B G R in memory. */
void
IOMobileFramebuffer::packFrame(void)
{
	for (uint32_t y = 0; y < fPanelHeight; y++) {
		const uint32_t * from = (const uint32_t *) (fBase + (size_t) y * fRowBytes);
		volatile uint32_t * to = (volatile uint32_t *) (fPacked + (size_t) y * fPackedRowBytes);
		uint32_t x = 0;

		for (; x + 4 <= fPanelWidth; x += 4, from += 4, to += 3) {
			uint32_t a = from[0] & 0xffffff, b = from[1] & 0xffffff, c = from[2] & 0xffffff, d = from[3] & 0xffffff;

			to[0] = a | b << 24;
			to[1] = b >> 8 | c << 16;
			to[2] = c >> 16 | d << 8;
		}
	}
	__builtin_arm_dsb(0xf);
}

void
IOMobileFramebuffer::present(const uint8_t * swap)
{
	AbsoluteTime deadline;

	uint64_t now = mach_absolute_time(), period, pause;

	/* (a frame that is late on the panel was late here already, or it was not: for telling whose it is) */
	nanoseconds_to_absolutetime(NSEC_PER_SEC / refreshHz() * 5 / 4, &period);
	nanoseconds_to_absolutetime(FRAME_PAUSE * NSEC_PER_MSEC, &pause);
	clock_interval_to_deadline(SWAP_HELD_MS, kMillisecondScale, &deadline);
	if (gTouchLat) {
		hakios_tl("tl: s %u at %llu\n", swapWord(swap, SWAP_ID), micros(now));
	}
	IOLockLock(fSwapLock);
	fStatSwapsLate += fSwapTime != 0 && now - fSwapTime > period && now - fSwapTime < pause;
	fSwapTime = now;
	fStatHeld += fSwapsWaiting >= swapQueue();
	while (fSwapsWaiting >= swapQueue()) {
		if (IOLockSleepDeadline(fSwapLock, &fSwapsWaiting, deadline, THREAD_UNINT) == THREAD_TIMED_OUT &&
		    fSwapsWaiting >= swapQueue()) {
			fStatLost++;                    /* (the GPU has stopped answering: the frame before is waiting for it) */
			IOLockUnlock(fSwapLock);
			return;
		}
	}
	unsigned slot = (fSwapFirst + fSwapsWaiting) % SWAP_QUEUE;

	memcpy(fSwap[slot], swap, SWAP_SIZE);
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		uint32_t sid = swapWord(swap, SWAP_SURFACE(layer));

		fSwapSurfaces[slot][layer] = sid != 0 ? MiniIOSurfaceCopy(sid) : NULL;
	}
	fSwapsWaiting++;
	IOLockWakeup(fSwapLock, fSwap, true);
	IOLockUnlock(fSwapLock);
}

void
IOMobileFramebuffer::presenterMain(void * self, wait_result_t result __unused)
{
	((IOMobileFramebuffer *) self)->presenter();
}

void
IOMobileFramebuffer::presenter(void)
{
	static uint8_t swap[SWAP_SIZE];         /* (one thread) */
	MiniIOSurface * surfaces[SWAP_LAYERS] = { NULL };
	bool unsettled = false;                 /* the frame on the panel was drawn unfiltered, and its surfaces are held */

	for (;;) {
		IOLockLock(fSwapLock);
		while (fSwapsWaiting == 0) {
			uint64_t deadline;

			if (!unsettled) {
				IOLockSleep(fSwapLock, fSwap, THREAD_UNINT);
				continue;
			}
			clock_interval_to_deadline(SETTLE_MS, NSEC_PER_MSEC, &deadline);
			if (IOLockSleepDeadline(fSwapLock, fSwap, deadline, THREAD_UNINT) == THREAD_TIMED_OUT && fSwapsWaiting == 0) {
				IOLockUnlock(fSwapLock);
				settle(swap, surfaces);
				unsettled = false;
				IOLockLock(fSwapLock);
			}
		}
		IOLockUnlock(fSwapLock);
		/* the frame before is done with now, settled or not */
		for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
			if (surfaces[layer] != NULL) {
				MiniIOSurfaceDrop(surfaces[layer]);
				surfaces[layer] = NULL;
			}
		}
		IOLockLock(fSwapLock);
		memcpy(swap, fSwap[fSwapFirst], SWAP_SIZE);
		memcpy(surfaces, fSwapSurfaces[fSwapFirst], sizeof(surfaces));
		IOLockUnlock(fSwapLock);
		show(swap, surfaces);
		unsettled = gFilter == 2;
		if (!unsettled) {
			for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
				if (surfaces[layer] != NULL) {
					MiniIOSurfaceDrop(surfaces[layer]);
					surfaces[layer] = NULL;
				}
			}
		}
		MiniIOSurfaceClearKept();               /* (the next frame is some milliseconds away) */
	}
}

/*
 * The frame on the panel once more, filtered (fb_filter=2): the same layers from the same surfaces where show() put
 * them, nothing else (no turn to wait for, no swap to hand on, nothing counted). Its surfaces were held since.
 */
void
IOMobileFramebuffer::settle(const uint8_t * swap, MiniIOSurface ** surfaces)
{
	int32_t src[4], dst[4];

	if (!fGraphics || fScanning) {
		return;
	}
	fSettling = true;
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		MiniIOSurface * s = surfaces[layer];

		if (s == NULL || s->info.bytesPerElement != 4 || s->info.planeCount != 0) {
			continue;
		}
		memcpy(src, swap + SWAP_SOURCE(layer), sizeof(src));
		memcpy(dst, swap + SWAP_DEST(layer), sizeof(dst));
		drawLayer(s, src, dst);
	}
	fSettling = false;
	fStatSettled++;
}

/*
 * The GPU's hand-off (on by default; boot-arg fb_gpu=0: never): a frame of one full-screen layer is not drawn into the panel by the processor
 * (8 MB through write-combined memory a frame) but offered to the Mali's Metal plug-in in the window server, which waits
 * for frames (MALI_GOP of drivers/gpu/hakios.mali.cpp, which calls native_gop_handoff_call below), has the GPU copy the
 * surface's pages into the panel (native_gop_handoff_memory, mapped into it once) and completes the frame. The protocol is
 * Ken-u's MaliMetal's (GopFramebuffer's NativeGOPPresenter on UniXNU): 0 the mode, 2 wait for a frame (its pages mapped into
 * the caller), 3 the frame done. A frame nobody takes in time, or with no presenter waiting, is drawn as before.
 */
static TUNABLE(uint32_t, gGpuHandoff, "fb_gpu", 1);
#define GOP_TAKE_MS     100             /* how long a frame waits for the presenter to finish it */
#define GOP_IDLE_MS     500             /* a presenter that has not waited for this long is gone */

struct gop_mode { uint32_t version, format; uint64_t width, height, stride, bytes; };
struct gop_swap { uint64_t sequence, address, bytes, stride; uint32_t width, height, format, reserved; };

static struct {
	IOLock *                lock;
	uint64_t                panelPhys, panelBytes, stride;
	uint32_t                width, height;
	uint64_t                lastWait;       /* mach_absolute_time of the presenter's last wait */
	MiniIOSurface *         offered;        /* the frame being handed over, retained */
	bool                    handed, done;
	uint32_t                status;
	uint64_t                sequence;
	IOMemoryMap *           map;            /* its pages in the presenter's task */
	uint64_t                frames, fallbacks;
} gGop;

extern "C" uint64_t kvtophys(vm_offset_t va);

/*
 * Boot-arg fb_flip=1: a frame of one full-screen layer is not copied at all. CoreAnimation hands a frame over before it commits
 * the drawing, so the swap comes first: when the GPU has finished the surface, the window server's plug-in names it (op 4 below)
 * and the Amlogic OSD1 is pointed at it there and then. A swap of such a frame does nothing more; any other frame is drawn into
 * the panel's memory from the surface being scanned and the OSD goes back there.
 */
static TUNABLE(uint32_t, gFlip, "fb_flip", 0);
/* Boot-arg fb_flip_lag=N: a surface counts as on the display for N refreshes more than the next change of frame. */
static TUNABLE(uint32_t, gFlipLag, "fb_flip_lag", 0);
#define VPU_BASE                0xff000000UL    /* VCBUS: register number * 4 */
#define VPU_BYTES               0x10000
#define VIU_OSD1_BLK1_CFG_W4    0x1a14          /* the frame's address >> 4 */
#define ENCP_INFO_READ          0x271d          /* [28:16] the line being sent */
#define ENCP_VIDEO_MAX_LNCNT    0x1bae          /* the lines of a frame, less one */
#define ENCP_DE_V_BEGIN         0x1c3c          /* the first line of the picture */
#define ENCP_DE_V_END           0x1c3d          /* the line after its last */
#define FLIP_WAIT_MS            51              /* two refreshes: past this a surface is drawn into anyway */

static struct {
	volatile uint32_t *     vpu;
	MiniIOSurface *         next;           /* the surface of the swap to come, held */
	MiniIOSurface *         current;        /* the surface being scanned, held */
	uint32_t                leaving;        /* the surface scanned before it, on the panel until ... */
	uint64_t                leftAt;         /* ... this time (mach_absolute_time): the OSD takes a new address between frames */
	uint32_t                waits, waited;  /* of the window server for a surface to leave the display; microseconds */
	uint32_t                blankMisses;    /* flips that did not find the blanking in time */
} gScan;

static bool scannableSize(const MiniIOSurface * s);

static void
gopInit(void * base, uint32_t width, uint32_t height, uint32_t rowBytes)
{
	if (!gGpuHandoff || gGop.lock != NULL) {
		return;
	}
	gGop.lock = IOLockAlloc();
	gGop.panelPhys = kvtophys((vm_offset_t) base);
	gGop.width = width;
	gGop.height = height;
	gGop.stride = rowBytes;
	gGop.panelBytes = (uint64_t) rowBytes * height;
	if (gFlip) {
		gScan.vpu = (volatile uint32_t *) ml_io_map(VPU_BASE, VPU_BYTES);
		IOLog("MiniIOMobileFramebuffer: fb_flip=1: OSD1 scans %#x0, line %u\n", gScan.vpu[VIU_OSD1_BLK1_CFG_W4],
		    gScan.vpu[ENCP_INFO_READ] >> 16 & 0x1fff);
	}
	IOLog("MiniIOMobileFramebuffer: fb_gpu=1: frames may go to the GPU's presenter (panel at %#llx, %ux%u, %u bytes a row)\n",
	    (unsigned long long) gGop.panelPhys, width, height, rowBytes);
}

extern "C" IOMemoryDescriptor *
native_gop_handoff_memory(void)
{
	if (gGop.lock == NULL || gGop.panelPhys == 0) {
		return NULL;
	}
	return IOMemoryDescriptor::withPhysicalAddress((IOPhysicalAddress) gGop.panelPhys, (IOByteCount) round_page(gGop.panelBytes),
	           kIODirectionInOut);
}

extern "C" IOReturn
native_gop_handoff_call(uint32_t op, task_t task, const void * in, size_t inSize, void * out, size_t outSize)
{
	if (gGop.lock == NULL) {
		return kIOReturnUnsupported;
	}
	switch (op) {
	case 0: {
		gop_mode mode = { 1, 1, gGop.width, gGop.height, gGop.stride, gGop.panelBytes };

		if (out == NULL || outSize < sizeof(mode)) {
			return kIOReturnBadArgument;
		}
		memcpy(out, &mode, sizeof(mode));
		return kIOReturnSuccess;
	}
	case 2: {
		uint64_t timeoutMs = 100, deadline;
		IOReturn result = kIOReturnTimeout;

		if (in != NULL && inSize >= sizeof(timeoutMs)) {
			memcpy(&timeoutMs, in, sizeof(timeoutMs));
		}
		if (out == NULL || outSize < sizeof(gop_swap)) {
			return kIOReturnBadArgument;
		}
		clock_interval_to_deadline((uint32_t) (timeoutMs ? timeoutMs : 1), NSEC_PER_MSEC, &deadline);
		IOLockLock(gGop.lock);
		gGop.lastWait = mach_absolute_time();
		while (gGop.offered == NULL || gGop.handed) {
			if (IOLockSleepDeadline(gGop.lock, &gGop.offered, deadline, THREAD_INTERRUPTIBLE) != THREAD_AWAKENED) {
				break;
			}
		}
		if (gGop.offered != NULL && !gGop.handed) {
			MiniIOSurface * f = gGop.offered;
			uint64_t offset = (uint64_t) (f->kpixels - (uint8_t *) f->memory->getBytesNoCopy()) + f->info.offset;

			gGop.map = f->memory->createMappingInTask(task, 0, kIOMapAnywhere, offset, f->info.allocSize);
			if (gGop.map != NULL) {
				gop_swap swap = { gGop.sequence, gGop.map->getAddress(), f->info.allocSize, f->info.bytesPerRow, f->info.width,
					          f->info.height, 1, 0 };

				memcpy(out, &swap, sizeof(swap));
				gGop.handed = true;
				result = kIOReturnSuccess;
			} else {
				gGop.done = true;               /* (drawn by the processor) */
				gGop.status = 1;
				IOLockWakeup(gGop.lock, &gGop.done, true);
				result = kIOReturnNoMemory;
			}
		}
		gGop.lastWait = mach_absolute_time();
		IOLockUnlock(gGop.lock);
		return result;
	}
	case 3: {
		struct { uint64_t sequence; uint32_t status, reserved; } done;

		if (in == NULL || inSize < sizeof(done)) {
			return kIOReturnBadArgument;
		}
		memcpy(&done, in, sizeof(done));
		IOLockLock(gGop.lock);
		if (gGop.handed && done.sequence == gGop.sequence) {
			gGop.status = done.status;
			gGop.done = true;
			IOLockWakeup(gGop.lock, &gGop.done, true);
		}
		gGop.lastWait = mach_absolute_time();
		IOLockUnlock(gGop.lock);
		return kIOReturnSuccess;
	}
	case 4: {
		uint32_t sid = 0;
		MiniIOSurface * s, * old;

		if (gScan.vpu == NULL) {
			return kIOReturnUnsupported;
		}
		if (in == NULL || inSize < sizeof(sid)) {
			return kIOReturnBadArgument;
		}
		memcpy(&sid, in, sizeof(sid));
		s = MiniIOSurfaceCopy(sid);
		if (s == NULL) {
			return kIOReturnNotFound;
		}
		if (!scannableSize(s) || MiniIOSurfaceScanAddress(s) == 0) {
			static uint32_t refused;

			if (refused++ % 200 == 0) {
				IOLog("MiniIOMobileFramebuffer: flip: surface %u refused (%ux%u, %u bytes a row, scan address %#llx; refusal %u)\n", sid,
				    s->info.width, s->info.height, s->info.bytesPerRow, (unsigned long long) MiniIOSurfaceScanAddress(s), refused);
			}
			MiniIOSurfaceDrop(s);
			return kIOReturnNoResources;
		}
		if (gScanner != NULL && gScanner->fGraphics) {
			if (!gScanner->scanFrom(s)) {
				MiniIOSurfaceDrop(s);
			}
			return kIOReturnSuccess;
		}
		IOLockLock(gGop.lock);
		old = gScan.next;
		gScan.next = s;
		IOLockUnlock(gGop.lock);
		if (old != NULL) {
			MiniIOSurfaceDrop(old);
		}
		return kIOReturnSuccess;
	}
	case 5: {
		uint32_t sid = 0;
		uint64_t began = mach_absolute_time(), deadline, now;

		if (gScan.vpu == NULL) {
			return kIOReturnUnsupported;
		}
		if (in == NULL || inSize < sizeof(sid)) {
			return kIOReturnBadArgument;
		}
		memcpy(&sid, in, sizeof(sid));
		clock_interval_to_deadline(FLIP_WAIT_MS, NSEC_PER_MSEC, &deadline);
		for (now = began; now < deadline; now = mach_absolute_time()) {
			bool shown;

			IOLockLock(gGop.lock);
			shown = (gScan.current != NULL && gScan.current->sid == sid) || (gScan.leaving == sid && now < gScan.leftAt);
			IOLockUnlock(gGop.lock);
			if (!shown) {
				break;
			}
			IODelay(200);
		}
		if (now != began) {
			uint64_t nanos;

			absolutetime_to_nanoseconds(now - began, &nanos);
			gScan.waits++;
			gScan.waited += (uint32_t) (nanos / NSEC_PER_USEC);
		}
		return kIOReturnSuccess;
	}
	default:
		return kIOReturnBadArgument;
	}
}

/* true: the presenter put the surface on the panel; false: the caller draws it */
static bool
gopOffer(MiniIOSurface * s)
{
	uint64_t now = mach_absolute_time(), idle, deadline;
	bool shown;

	if (gGop.lock == NULL || s->memory == NULL || s->client != NULL) {
		return false;
	}
	nanoseconds_to_absolutetime((uint64_t) GOP_IDLE_MS * NSEC_PER_MSEC, &idle);
	IOLockLock(gGop.lock);
	if (gGop.lastWait == 0 || now - gGop.lastWait > idle) {
		IOLockUnlock(gGop.lock);
		return false;
	}
	s->retain();
	gGop.offered = s;
	gGop.handed = gGop.done = false;
	gGop.status = 1;
	gGop.sequence++;
	IOLockWakeup(gGop.lock, &gGop.offered, true);
	clock_interval_to_deadline(GOP_TAKE_MS, NSEC_PER_MSEC, &deadline);
	while (!gGop.done) {
		if (IOLockSleepDeadline(gGop.lock, &gGop.done, deadline, THREAD_UNINT) != THREAD_AWAKENED) {
			break;
		}
	}
	shown = gGop.done && gGop.status == 0;
	gGop.offered = NULL;
	gGop.handed = false;
	if (gGop.map != NULL) {
		gGop.map->release();
		gGop.map = NULL;
	}
	shown ? gGop.frames++ : gGop.fallbacks++;
	if ((gGop.frames + gGop.fallbacks) % 600 == 1) {
		IOLog("MiniIOMobileFramebuffer: GPU hand-off: %llu frames by the GPU, %llu by the processor\n", gGop.frames, gGop.fallbacks);
	}
	IOLockUnlock(gGop.lock);
	s->release();
	return shown;
}

/* Until the encoder is between pictures: a new address then takes effect with no line of two frames on the screen. */
static void
waitForBlanking(void)
{
	const uint32_t lines = gScan.vpu[ENCP_VIDEO_MAX_LNCNT] + 1, begin = gScan.vpu[ENCP_DE_V_BEGIN], end = gScan.vpu[ENCP_DE_V_END];
	/* the mode's own frame (41.7 ms at 23.976 Hz: two frames at refreshHz()'s 60 gave up before its blanking came) */
	uint64_t frame = hakios_hdmitx_frame_ns();
	if (frame == 0) {
		frame = NSEC_PER_SEC / refreshHz();
	}
	const uint64_t perLine = frame / lines;
	uint64_t ticks, deadline;
	uint32_t line = gScan.vpu[ENCP_INFO_READ] >> 16 & 0x1fff;

	if (end <= begin || end >= lines || line >= end || line < begin) {
		return;
	}
	clock_interval_to_deadline(2 * frame / NSEC_PER_USEC, NSEC_PER_USEC, &deadline);
	while (mach_absolute_time() < deadline) {
		line = gScan.vpu[ENCP_INFO_READ] >> 16 & 0x1fff;
		if (line >= end || line < begin) {
			return;
		}
		/* asleep for most of the way, which can run over by a millisecond and more; the last of it by the line, past a blanking overslept too */
		if ((end - line) * perLine > 2500 * NSEC_PER_USEC) {
			nanoseconds_to_absolutetime((end - line) * perLine - 2000 * NSEC_PER_USEC, &ticks);
			clock_delay_until(mach_absolute_time() + ticks);
		} else {
			IODelay(5);
		}
	}
	gScan.blankMisses++;
}

/*
 * A frame the display can scan as it is: the panel's size. In a 720p mode the window server still draws 1920x1080 pictures
 * into its 1920x1080 surfaces, but names the swap's rectangles in the mode's pixels (from and to 0,0 1280x720): the OSD
 * scans the whole surface and its scaler makes the 720p picture. What the OSD is set to scan, to change it only when needed.
 */
static uint32_t gScanW, gScanH, gScanStride;

static bool
scannableSize(const MiniIOSurface * s)
{
	return s->info.width == gGop.width && s->info.height == gGop.height && s->info.bytesPerRow == gGop.stride;
}

/* a 720p mode on the wire (with the panel's 1920x1080 frames) */
static bool
smallOutput(void)
{
	return hakios_hdmitx_output_height() == 720 && gGop.width >= 1280 && gGop.height >= 720;
}


/* (in the blanking) */
static void
scanSource(uint32_t w, uint32_t h, uint32_t stride)
{
	if (gScanW == 0) {
		gScanW = gGop.width;                    /* (what U-Boot and twloader left: the panel) */
		gScanH = gGop.height;
		gScanStride = gGop.stride;
	}
	if (w != gScanW || h != gScanH || stride != gScanStride) {
		hakios_hdmitx_osd_source(w, h, stride);
		gScanW = w;
		gScanH = h;
		gScanStride = stride;
	}
}

/* The display scans this surface from now on; its hold passes to gScan. false: it cannot. */
bool
IOMobileFramebuffer::scanFrom(MiniIOSurface * s)
{
	uint64_t address = MiniIOSurfaceScanAddress(s);

	if (address == 0) {
		return false;
	}
	if (fStatFlips + fStatFlipsKept == 0 && !fScanning) {
		IOLog("MiniIOMobileFramebuffer: flip: surface %u at %#llx, line %u\n", s->sid, (unsigned long long) address,
		    gScan.vpu[ENCP_INFO_READ] >> 16 & 0x1fff);
	}
	uint64_t left;
	MiniIOSurface * old;

	if (s->kpixels != NULL) {
		hakios_vplane_osd_frame(s->kpixels + s->info.offset, s->info.bytesPerRow, s->info.width, s->info.height);
	}
	waitForBlanking();
	/* (the old surface is done with as the next picture starts, at most the blanking away) */
	nanoseconds_to_absolutetime(1500 * NSEC_PER_USEC + (uint64_t) gFlipLag * (NSEC_PER_SEC / refreshHz()), &left);
	scanSource(s->info.width, s->info.height, s->info.bytesPerRow);
	gScan.vpu[VIU_OSD1_BLK1_CFG_W4] = (uint32_t) (address >> 4);
	IOLockLock(gGop.lock);
	old = gScan.current;
	gScan.current = s;
	if (old != NULL && old != s) {
		gScan.leaving = old->sid;
		gScan.leftAt = mach_absolute_time() + left;
	}
	IOLockUnlock(gGop.lock);
	if (old != NULL) {
		MiniIOSurfaceDrop(old);
	}
	fScanning = true;
	fStatFlips++;
	return true;
}

/* The display goes back to the panel's memory. */
void
IOMobileFramebuffer::scanPanel(void)
{
	MiniIOSurface * old;

	scanSource(gGop.width, gGop.height, gGop.stride);
	gScan.vpu[VIU_OSD1_BLK1_CFG_W4] = (uint32_t) (gGop.panelPhys >> 4);
	IOLockLock(gGop.lock);
	old = gScan.current;
	gScan.current = NULL;
	IOLockUnlock(gGop.lock);
	if (old != NULL) {
		MiniIOSurfaceDrop(old);
	}
	fScanning = false;
	fStatFlipsLeft++;
}

/*
 * A swap's video layer to the video plane, in the frame's own pixels (the swap places both on the display); a few swaps
 * in a row without one and the plane is let go (the window server leaves it out of a swap now and then).
 */
void
IOMobileFramebuffer::showVideo(const uint8_t * swap, MiniIOSurface * v, const int32_t src[4], const int32_t dst[4],
    const int32_t mainDst[4], uint32_t mainW, uint32_t mainH)
{
	static uint32_t without, reports;
	struct hakios_vplane_surface p = {};

	if (v == NULL || v->kpixels == NULL || mainW == 0 || mainH == 0 || mainDst[2] <= 0 || mainDst[3] <= 0) {
		if (++without == 15) {
			hakios_vplane_surface(NULL);
		}
		return;
	}
	without = 0;
	int64_t x0 = (int64_t) (dst[0] - mainDst[0]) * mainW / mainDst[2], y0 = (int64_t) (dst[1] - mainDst[1]) * mainH / mainDst[3];
	int64_t x1 = (int64_t) (dst[0] + dst[2] - mainDst[0]) * mainW / mainDst[2];
	int64_t y1 = (int64_t) (dst[1] + dst[3] - mainDst[1]) * mainH / mainDst[3];
	int32_t sx = src[0] < 0 ? 0 : src[0] & ~1, sy = src[1] < 0 ? 0 : src[1] & ~1;
	uint32_t sw = src[2] > 0 ? (uint32_t) src[2] : v->info.width, sh = src[3] > 0 ? (uint32_t) src[3] : v->info.height;

	x0 = x0 < 0 ? 0 : x0;
	y0 = y0 < 0 ? 0 : y0;
	x1 = x1 > mainW ? mainW : x1;
	y1 = y1 > mainH ? mainH : y1;
	if ((uint32_t) sx + sw > v->planes[0].width) {
		sw = v->planes[0].width > (uint32_t) sx ? v->planes[0].width - (uint32_t) sx : 0;
	}
	if ((uint32_t) sy + sh > v->planes[0].height) {
		sh = v->planes[0].height > (uint32_t) sy ? v->planes[0].height - (uint32_t) sy : 0;
	}
	if (reports < 6) {
		reports++;
		IOLog("MiniIOMobileFramebuffer: swap %u: video layer surface %u (%ux%u '%c%c%c%c', rows %u/%u at %u/%u) from %d,%d %dx%d "
		    "to %d,%d %dx%d; the frame %ux%u to %d,%d %dx%d: %lld,%lld to %lld,%lld in it\n", swapWord(swap, SWAP_ID), v->sid,
		    v->info.width, v->info.height, (int) (v->info.pixelFormat >> 24 & 0xff), (int) (v->info.pixelFormat >> 16 & 0xff),
		    (int) (v->info.pixelFormat >> 8 & 0xff), (int) (v->info.pixelFormat & 0xff), v->planes[0].bytesPerRow,
		    v->planes[1].bytesPerRow, v->planes[0].offset, v->planes[1].offset, src[0], src[1], src[2], src[3], dst[0], dst[1],
		    dst[2], dst[3], mainW, mainH, mainDst[0], mainDst[1], mainDst[2], mainDst[3], x0, y0, x1, y1);
	}
	if (x1 - x0 < 16 || y1 - y0 < 16 || sw < 16 || sh < 16) {
		return;
	}
	p.width = sw;
	p.height = sh;
	p.x = (uint32_t) x0;
	p.y = (uint32_t) y0;
	p.w = (uint32_t) (x1 - x0);
	p.h = (uint32_t) (y1 - y0);
	OSData * token = v->values != NULL ? OSDynamicCast(OSData, v->values->getObject(TWL_VPLANE_TOKEN_KEY)) : NULL;
	if (token != NULL && token->getLength() == TWL_VPLANE_TOKEN_SIZE) {
		p.token = token->getBytesNoCopy();
	} else {
		/* (whoever wrote it, the CPU or the GPU: what is read is memory) */
		uint8_t * base = v->kpixels;

		p.lumaStride = v->planes[0].bytesPerRow;
		p.chromaStride = v->planes[1].bytesPerRow;
		p.luma = base + v->planes[0].offset + (size_t) sy * p.lumaStride + (uint32_t) sx;
		p.chroma = base + v->planes[1].offset + (size_t) (sy / 2) * p.chromaStride + (uint32_t) sx;
		FlushPoC_DcacheRegion((vm_offset_t) p.luma, (size_t) sh * p.lumaStride);
		FlushPoC_DcacheRegion((vm_offset_t) p.chroma, (size_t) (sh / 2) * p.chromaStride);
	}
	hakios_vplane_surface(&p);
}

/* A swap's surfaces onto the panel, once the GPU is done with them. */
void
IOMobileFramebuffer::show(const uint8_t * swap, MiniIOSurface ** surfaces)
{
	uint32_t set = swapWord(swap, SWAP_LAYERS_SET);
	uint64_t began = mach_absolute_time(), nanos, ready, shown, drawn;
	uint32_t layers = 0, mainW = 0, mainH = 0;
	int32_t videoSrc[4] = {}, videoDst[4] = {}, mainDst[4] = {};
	MiniIOSurface * recorded = NULL, * next = NULL;
	bool report, scanned = false, alone = true;
	uint32_t sid = 0;

	if (gScan.vpu != NULL) {
		IOLockLock(gGop.lock);
		next = gScan.next;
		gScan.next = NULL;
		sid = next == NULL && gScan.current != NULL ? gScan.current->sid : 0;
		IOLockUnlock(gGop.lock);
		next = sid != 0 ? MiniIOSurfaceCopy(sid) : next;
	}
	/*
	 * The frame: the lowest layer with a 32-bit surface; the window server moves it to layer 1 when a video layer comes
	 * (an NV12 surface: AVSampleBufferDisplayLayer's frames, which it does not draw), and that goes to the video plane,
	 * under the frame.
	 */
	int mainLayer = -1, videoLayer = -1;
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		MiniIOSurface * t = surfaces[layer];

		if (swapWord(swap, SWAP_SURFACE(layer)) == 0 || t == NULL) {
			continue;
		}
		if (t->info.bytesPerElement == 4 && t->info.planeCount == 0) {
			mainLayer = mainLayer < 0 ? (int) layer : mainLayer;
		} else if (videoLayer < 0 && t->info.planeCount == 2 && t->planes[0].bytesPerElement == 1 &&
		    t->planes[1].bytesPerElement == 2) {
			videoLayer = (int) layer;
		} else {
			static uint32_t odd;

			if (odd++ < 4) {
				IOLog("MiniIOMobileFramebuffer: swap %u layer %u: surface %u ('%c%c%c%c', %u planes) is not one shown\n",
				    swapWord(swap, SWAP_ID), layer, t->sid, (int) (t->info.pixelFormat >> 24 & 0xff),
				    (int) (t->info.pixelFormat >> 16 & 0xff), (int) (t->info.pixelFormat >> 8 & 0xff),
				    (int) (t->info.pixelFormat & 0xff), t->info.planeCount);
			}
		}
	}
	if (mainLayer < 0) {
		mainLayer = 0;
	}
	/* (diagnosis: what the window server's frame holds, 16 x 9 pixels looked at: transparent (the video shows), opaque
	 * black, or anything else; a frame with nothing else and no video under it is a black screen) */
	if (surfaces[mainLayer] != NULL && surfaces[mainLayer]->kpixels != NULL && surfaces[mainLayer]->info.bytesPerElement == 4) {
		static uint32_t empties, said, last_kind = 99;
		MiniIOSurface * f = surfaces[mainLayer];
		const uint8_t * px = f->kpixels + f->info.offset;
		unsigned clear = 0, black = 0, other = 0;

		for (unsigned gy = 0; gy < 9; gy++) {
			for (unsigned gx = 0; gx < 16; gx++) {
				const uint8_t * p = px + (size_t) (f->info.height * (2 * gy + 1) / 18) * f->info.bytesPerRow +
				    (size_t) (f->info.width * (2 * gx + 1) / 32) * 4;
				if (p[3] < 8) {
					clear++;
				} else if (p[0] < 8 && p[1] < 8 && p[2] < 8) {
					black++;
				} else {
					other++;
				}
			}
		}
		/* kinds: 0 picture, 1 all transparent, 2 all opaque black, 3 transparent and black only */
		uint32_t kind = other != 0 ? 0 : black == 0 ? 1 : clear == 0 ? 2 : 3;
		if ((kind != last_kind || (kind >= 2 && empties++ % 120 == 0)) && said++ < 600) {
			IOLog("MiniIOMobileFramebuffer: swap %u: frame surface %u: %u clear, %u black, %u else (kind %u); video layer %d\n",
			    swapWord(swap, SWAP_ID), f->sid, clear, black, other, kind, videoLayer);
		}
		last_kind = kind;
	}
	uint32_t others = (1u << mainLayer) | (videoLayer >= 0 ? 1u << videoLayer : 0), present = 0;
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		present |= swapWord(swap, SWAP_SURFACE(layer)) != 0 ? 1u << layer : 0;
	}
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		alone = alone && ((others >> layer & 1) || swapWord(swap, SWAP_SURFACE(layer)) == 0);
	}
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		if (surfaces[layer] != NULL && !MiniIOSurfaceDrawn(surfaces[layer], SWAP_GPU_MS)) {
			IOLog("MiniIOMobileFramebuffer: swap %u: the GPU is not done with surface %u after %u ms; shown as it is\n",
			    swapWord(swap, SWAP_ID), swapWord(swap, SWAP_SURFACE(layer)), SWAP_GPU_MS);
		}
	}
	fStatWaiting += mach_absolute_time() - began;
	began = ready = mach_absolute_time();
	if (gOnTheBeat && !fScanning) {
		uint64_t period, aim, next;

		nanoseconds_to_absolutetime(NSEC_PER_SEC / refreshHz(), &period);
		aim = period * 3 / 8;
		next = fReady;
		fReady = began;
		if (fBeat == 0 || began - next > period * 3 / 2) {
			fBeat = began;
		} else {
			for (next = fBeat + period; next < began; next += period) {
			}
			if (next - began > aim) {
				next -= (next - began - aim) / 8;
			} else {
				next += (aim - (next - began)) / 8;
			}
			clock_delay_until(next);
			fBeat = next;
			fStatOnBeat++;
			fStatBeatWait += mach_absolute_time() - began;
			began = mach_absolute_time();
		}
	}
	shown = began;
	/* its turn has come: the swap after it may be handed over */
	IOLockLock(fSwapLock);
	fSwapFirst = (fSwapFirst + 1) % SWAP_QUEUE;
	fSwapsWaiting--;
	IOLockWakeup(fSwapLock, &fSwapsWaiting, true);
	IOLockUnlock(fSwapLock);
	IOLockLock(fLock);
	if (fLastFrame != 0) {
		absolutetime_to_nanoseconds(began - fLastFrame, &nanos);
		nanos /= NSEC_PER_MSEC;
		if (nanos >= FRAME_PAUSE) {
			fFramePauses++;
		} else {
			fFrameTimes[nanos < FRAME_TIMES ? nanos : FRAME_TIMES]++;
			fFrameLongest = nanos > fFrameLongest ? (uint32_t) nanos : fFrameLongest;
		}
	}
	fLastFrame = began;
	report = fSwapsShown < 8;
	if (report) {
		IOLog("MiniIOMobileFramebuffer: swap %u (#%u): surfaces %u %u %u %u, layers set 0x%x cleared 0x%x\n",
		    swapWord(swap, SWAP_ID), fSwapsShown, swapWord(swap, SWAP_SURFACE(0)), swapWord(swap, SWAP_SURFACE(1)),
		    swapWord(swap, SWAP_SURFACE(2)), swapWord(swap, SWAP_SURFACE(3)), set, swapWord(swap, SWAP_LAYERS_CLEARED));
	}
	for (unsigned layer = 0; layer < SWAP_LAYERS; layer++) {
		uint32_t sid = swapWord(swap, SWAP_SURFACE(layer));
		int32_t src[4], dst[4];
		MiniIOSurface * s;

		layers |= sid != 0 ? 1u << layer : 0;
		/* the frames' surfaces: is it a few, in turn? */
		for (unsigned i = 0; (int) layer == mainLayer && sid != 0 && i < 8; i++) {
			if (fStatSurfaces[i] == 0 || fStatSurfaces[i] == sid) {
				fStatSurfaces[i] = sid;
				break;
			}
		}
		if (sid == 0) {
			/* the bottom layer taken away: nothing is shown, the screen is black */
			if (layer == 0 && mainLayer == 0 && (set & 1) && fGraphics) {
				for (uint32_t y = 0; y < fPanelHeight; y++) {
					bzero(fBase + (size_t) y * fRowBytes, (size_t) fPanelWidth * 4);
				}
			}
			continue;
		}
		memcpy(src, swap + SWAP_SOURCE(layer), sizeof(src));
		memcpy(dst, swap + SWAP_DEST(layer), sizeof(dst));
		if ((int) layer == videoLayer) {
			memcpy(videoSrc, src, sizeof(src));
			memcpy(videoDst, dst, sizeof(dst));
			continue;
		}
		s = (int) layer == mainLayer && next != NULL ? next : surfaces[layer];
		if ((int) layer == mainLayer && s != NULL) {
			memcpy(mainDst, dst, sizeof(dst));
			mainW = s->info.width;
			mainH = s->info.height;
		}
		if (report) {
			IOLog("MiniIOMobileFramebuffer: swap %u layer %u: surface %u%s (%ux%u '%c%c%c%c') "
			    "from %d,%d %dx%d to %d,%d %dx%d, layers set 0x%x\n", swapWord(swap, SWAP_ID), layer, sid,
			    s ? "" : " MISSING", s ? s->info.width : 0, s ? s->info.height : 0,
			    s ? (int) (s->info.pixelFormat >> 24 & 0xff) : '?', s ? (int) (s->info.pixelFormat >> 16 & 0xff) : '?',
			    s ? (int) (s->info.pixelFormat >> 8 & 0xff) : '?', s ? (int) (s->info.pixelFormat & 0xff) : '?',
			    src[0], src[1], src[2], src[3], dst[0], dst[1], dst[2], dst[3], set);
		}
		if (s == NULL) {
			continue;
		}
		if (s->info.bytesPerElement == 4 && s->info.planeCount == 0) {
			if (!fGraphics) {
				/* the first picture: the kernel's text console must stop drawing over it */
				fGraphics = true;
				PE_initialize_console(NULL, kPEGraphicsMode);
			}
			/* (in a 720p mode the swap may name the whole picture in the mode's pixels, 0,0 1280x720: still the whole surface) */
			bool small = smallOutput() && src[2] == 1280 && src[3] == 720 && dst[2] == 1280 && dst[3] == 720;
			uint32_t shownW, shownH;

			/* (a TV out: the whole frame onto the whole timing, which the VPP scales) */
			shownSize(fPanelWidth, fPanelHeight, &shownW, &shownH);
			/* (a layer the swap only clears is no other picture: in HDR modes the window server clears layer 0 in every swap,
			 * and its frames, read as not whole, were drawn from the swap's surface the GPU never wrote: the boot logo) */
			bool full = (int) layer == mainLayer && (set & ~others & present) == 0 && fPacked == NULL && src[0] == 0 && src[1] == 0 && dst[0] == 0 && dst[1] == 0 &&
			    (small || (src[2] == (int32_t) s->info.width && src[3] == (int32_t) s->info.height && dst[2] == (int32_t) shownW &&
			    dst[3] == (int32_t) shownH));
			bool whole = full && s->info.width == fPanelWidth && s->info.height == fPanelHeight;
			/*
			 * A TV out's swap names window server's scan-out surface of the timing's size (3840x2160 in 2160p), which nothing
			 * draws in here (backboardd's copy into it is off, tvos/src/twlhook.c): the picture is the frame it drew, flipped
			 * to the display. That surface is not drawn into the panel (it showed as a solid colour).
			 */
			bool tvScanout = gTVOut && full && !whole && s->info.width == shownW && s->info.height == shownH;
			static uint32_t lastW, lastH, reports;
			static int32_t lastDst[4];

			if ((int) layer == mainLayer && (s->info.width != lastW || s->info.height != lastH || memcmp(dst, lastDst, sizeof(lastDst)) != 0)) {
				snprintf(gFramesNow, sizeof(gFramesNow), "%ux%u '%c%c%c%c' from %d,%d %dx%d to %d,%d %dx%d: %s",
				    s->info.width, s->info.height, (int) (s->info.pixelFormat >> 24 & 0xff), (int) (s->info.pixelFormat >> 16 & 0xff),
				    (int) (s->info.pixelFormat >> 8 & 0xff), (int) (s->info.pixelFormat & 0xff), src[0], src[1], src[2], src[3],
				    dst[0], dst[1], dst[2], dst[3], whole ? "scanned" : tvScanout ? "TV scan-out, the flipped frame shown" : "drawn");
			}
			if ((int) layer == mainLayer && reports < 40 && (s->info.width != lastW || s->info.height != lastH || memcmp(dst, lastDst, sizeof(lastDst)) != 0)) {
				reports++;
				memcpy(lastDst, dst, sizeof(lastDst));
				IOLog("MiniIOMobileFramebuffer: frames now %ux%u ('%c%c%c%c'), from %d,%d %dx%d to %d,%d %dx%d: %s\n",
				    s->info.width, s->info.height, (int) (s->info.pixelFormat >> 24 & 0xff), (int) (s->info.pixelFormat >> 16 & 0xff),
				    (int) (s->info.pixelFormat >> 8 & 0xff), (int) (s->info.pixelFormat & 0xff), src[0], src[1], src[2], src[3],
				    dst[0], dst[1], dst[2], dst[3], whole ? "scanned as they are" : "drawn into the panel's memory");
				lastW = s->info.width;
				lastH = s->info.height;
			}

			int branch = 0;
			if (tvScanout && alone) {
				branch = 1;
				if (next != NULL && scanFrom(next)) {
					next = NULL;
				}
				scanned = fScanning;
			} else if (whole && alone && fScanning && (branch = 2)) {
				/* (the frame goes on the display when the GPU has finished it, and the surface the swap names has not been written) */
				fStatFlipsKept++;
				scanned = true;
			} else if (whole && alone && next != NULL && (branch = 3) && scanFrom(next)) {
				next = NULL;
				scanned = true;
			} else if (!(whole && !small && (branch = 4) && gopOffer(s))) {
				branch = branch == 4 ? 5 : 6;
				drawLayer(s, src, dst);
			}
			/* kern.hakios_fb_trace=N: the next N swaps' choice (1 TV scan-out, 2 kept, 3 flipped, 4 GPU hand-off,
			 * 5 hand-off refused then drawn, 6 drawn) and what it rests on */
			if (gFbTrace > 0 && (int) layer == mainLayer) {
				gFbTrace--;
				IOLog("MiniIOMobileFramebuffer: trace swap %u: layer %u surface %u %ux%u src %d,%d %dx%d dst %d,%d %dx%d; set %#x "
				    "others %#x video %d; full %d whole %d tvScanout %d alone %d small %d; scanning %d next %u; branch %d "
				    "scanned %d; panel %ux%u shown %ux%u\n", swapWord(swap, SWAP_ID), layer, sid, s->info.width,
				    s->info.height, src[0], src[1], src[2], src[3], dst[0], dst[1], dst[2], dst[3], set, others, videoLayer,
				    full, whole, tvScanout, alone, small, fScanning, next ? next->sid : 0, branch, scanned, fPanelWidth,
				    fPanelHeight, shownW, shownH);
			}
			if ((int) layer == mainLayer) {
				recorded = s;   /* (recorded once the frame is in the panel's memory: see below) */
			}
			/* boot-arg fb_probe=1: is there anything in the surface at all (one pixel in 16 x 16) */
			if (gProbe && (int) layer == mainLayer && s->kpixels != NULL && (fSwapsShown < 16 || fSwapsShown % 25 == 0)) {
				uint32_t nonzero = 0, opaque = 0, samples = 0, first = 0;

				for (uint32_t y = 0; y < s->info.height; y += 16) {
					for (uint32_t x = 0; x < s->info.width; x += 16) {
						uint32_t v = *(const uint32_t *)(s->kpixels + (size_t) y * s->info.bytesPerRow + (size_t) x * 4);

						samples++;
						nonzero += v != 0;
						opaque += (v >> 24) != 0;
						first = first == 0 ? v : first;
					}
				}
				IOLog("MiniIOMobileFramebuffer: probe swap %u surface %u: %u of %u samples non-zero, %u with alpha, first %#x\n",
				    swapWord(swap, SWAP_ID), sid, nonzero, samples, opaque, first);

			}
		}
	}
	/*
	 * A video layer with no frame over it (the screensaver, IdleScreen, shows its aerial alone): its place is in the
	 * display's terms all the same, so it is placed against a frame of the whole display (the panel's memory, cleared
	 * when the bottom layer went, is transparent over the plane). Without this the plane never came on: a black screen.
	 */
	if (videoLayer >= 0 && (mainW == 0 || mainH == 0)) {
		uint32_t w, h;

		shownSize(fPanelWidth, fPanelHeight, &w, &h);
		mainDst[0] = mainDst[1] = 0;
		mainDst[2] = (int32_t) w;
		mainDst[3] = (int32_t) h;
		mainW = fPanelWidth;
		mainH = fPanelHeight;
	}
	showVideo(swap, videoLayer >= 0 ? surfaces[videoLayer] : NULL, videoSrc, videoDst, mainDst, mainW, mainH);
	if (fPacked != NULL) {
		packFrame();
	}
	if (fScanning && !scanned) {
		scanPanel();
	}
	drawn = mach_absolute_time();
	if (recorded != NULL) {
		record(recorded);
	}
	if (next != NULL) {
		MiniIOSurfaceDrop(next);
	}
	fSwapsShown++;
	if (gTouchLat) {
		hakios_tl("tl: v %u ready %llu shown %llu done %llu\n", swapWord(swap, SWAP_ID), micros(ready), micros(shown),
		    micros(drawn));
	}
	if (layers != fLayersShown) {
		IOLog("MiniIOMobileFramebuffer: swap #%u: surfaces in layers 0x%x now (0x%x before): %u %u %u %u\n", fSwapsShown, layers,
		    fLayersShown, swapWord(swap, SWAP_SURFACE(0)), swapWord(swap, SWAP_SURFACE(1)), swapWord(swap, SWAP_SURFACE(2)),
		    swapWord(swap, SWAP_SURFACE(3)));
		fLayersShown = layers;
	}

	/* every five seconds in which something was drawn: how many frames, and what each cost here */
	fStatFrames++;
	fStatDrawing += mach_absolute_time() - began;
	if (fStatSince == 0) {
		fStatSince = began;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - fStatSince, &nanos);
	if (nanos >= 5 * NSEC_PER_SEC) {
		uint64_t drawing, waiting, beat;
		unsigned different;

		absolutetime_to_nanoseconds(fStatDrawing, &drawing);
		absolutetime_to_nanoseconds(fStatWaiting, &waiting);
		absolutetime_to_nanoseconds(fStatBeatWait, &beat);
		IOLog("MiniIOMobileFramebuffer: %u frames in %llu ms (%llu a second), %llu us each in the driver, after %llu us "
		    "waiting for the GPU and, %u of them, %llu us for the next refresh; %u swaps waited for the frame before, "
		    "%u gave up; %u swaps came a refresh late themselves; %u frames drawn again filtered\n", fStatFrames, nanos / NSEC_PER_MSEC,
		    fStatFrames * NSEC_PER_SEC / nanos, drawing / fStatFrames / NSEC_PER_USEC,
		    waiting / fStatFrames / NSEC_PER_USEC, fStatOnBeat, fStatOnBeat ? beat / fStatOnBeat / NSEC_PER_USEC : 0,
		    fStatHeld, fStatLost, fStatSwapsLate, fStatSettled);
		fStatSwapsLate = 0;
		for (different = 0; different < 8 && fStatSurfaces[different] != 0; different++) {
		}
		if (different == 8 && fStatFrames > 8) {
			uint32_t made, freed, again;
			uint64_t making, freeing;

			/* (CoreAnimation has made itself a new surface for every frame: see "the frames' surfaces" above) */
			MiniIOSurfaceCost(&made, &making, &freed, &freeing, &again);
			IOLog("MiniIOMobileFramebuffer: the frames came in surfaces %u, %u, %u, %u, %u and on, none twice; in that time "
			    "%u surfaces of all kinds were made, in %llu us each (%u of them of memory kept from one before), and %u "
			    "freed, in %llu us each\n", fStatSurfaces[0], fStatSurfaces[1], fStatSurfaces[2], fStatSurfaces[3],
			    fStatSurfaces[4], made, making, again, freed, freeing);
		}
		bzero(fStatSurfaces, sizeof(fStatSurfaces));
		if (gScan.vpu != NULL) {
			IOLog("MiniIOMobileFramebuffer: flip: %u frames flipped, %u kept, left the panel's memory %u times, line %u of %u; "
			    "%u waits for a surface to leave the display, %u us each; %u flips missed the blanking\n", fStatFlips, fStatFlipsKept, fStatFlipsLeft,
			    gScan.vpu[ENCP_INFO_READ] >> 16 & 0x1fff, gScan.vpu[ENCP_VIDEO_MAX_LNCNT] + 1, gScan.waits,
			    gScan.waits ? gScan.waited / gScan.waits : 0, gScan.blankMisses);
			gScan.blankMisses = 0;
			fStatFlips = fStatFlipsKept = fStatFlipsLeft = 0;
			gScan.waits = gScan.waited = 0;
		}
		fStatHeld = fStatLost = fStatOnBeat = fStatSettled = 0;
		fStatWaiting = fStatBeatWait = 0;
		reportFrameTimes();
		if (fDiff) {
			IOLog("MiniIOMobileFramebuffer: %llu pixels of the panel changed a frame, all within %u,%u to %u,%u\n",
			    fDiffPixels / fStatFrames, fDiffLeft, fDiffTop, fDiffRight, fDiffBottom);
			fDiffPixels = 0;
			fDiffLeft = fDiffTop = ~0u;
			fDiffRight = fDiffBottom = 0;
		}
		fStatFrames = 0;
		fStatDrawing = 0;
		fStatSince = 0;
	}
	IOLockUnlock(fLock);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class IOMobileFramebufferUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

	task_t                  fTask;
	IOMobileFramebuffer *   fOwner;
	IOLock *                fLock;
	mach_port_t             fPorts[kFBNotifyTypes];
	mach_vm_address_t       fCallbacks[kFBNotifyTypes];
	io_user_reference_t     fRefcons[kFBNotifyTypes];
	thread_call_t           fVSyncCall;
	uint64_t                fVSyncCount;
	uint64_t                fVSyncTime;             /* when the refresh being waited for is due */
	bool                    fVSyncRunning;
	bool                    fClosed;

	static void vsyncFired(thread_call_param_t self, thread_call_param_t unused);
	void vsync(void);
	void armVSync(void);

public:
	virtual bool initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments * args,
	    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type, properties)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService * provider)
{
	fOwner = OSDynamicCast(IOMobileFramebuffer, provider);
	if (fOwner == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fLock = IOLockAlloc();
	fVSyncCall = thread_call_allocate(&IOMobileFramebufferUserClient::vsyncFired, this);
	return fLock != NULL && fVSyncCall != NULL;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fLock != NULL) {
		IOLockLock(fLock);
		fClosed = true;
		IOLockUnlock(fLock);
	}
	if (fVSyncCall != NULL) {
		thread_call_cancel_wait(fVSyncCall);
	}
	if (!isInactive()) {
		terminate();
	}
	return kIOReturnSuccess;
}

void
IOMobileFramebufferUserClient::free(void)
{
	if (fVSyncCall != NULL) {
		thread_call_cancel_wait(fVSyncCall);
		thread_call_free(fVSyncCall);
		fVSyncCall = NULL;
	}
	for (unsigned i = 0; i < kFBNotifyTypes; i++) {
		if (fPorts[i] != NULL) {
			releaseNotificationPort(fPorts[i]);
			fPorts[i] = NULL;
		}
	}
	if (fLock != NULL) {
		IOLockFree(fLock);
		fLock = NULL;
	}
	IOUserClient::free();
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
{
	mach_port_t old;

	IOLog("MiniIOMobileFramebuffer: notification port for type %u\n", (unsigned) type);
	if (type >= kFBNotifyTypes) {
		return kIOReturnBadArgument;
	}
	IOLockLock(fLock);
	old = fPorts[type];
	fPorts[type] = port;
	IOLockUnlock(fLock);
	if (old != NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

void
IOMobileFramebufferUserClient::vsyncFired(thread_call_param_t self, thread_call_param_t unused)
{
	((IOMobileFramebufferUserClient *) self)->vsync();
}

/* With the lock held. */
void
IOMobileFramebufferUserClient::armVSync(void)
{
	uint64_t period, now = mach_absolute_time();

	/* one refresh after the last, not after whenever we got round to this: the rate does not drift */
	nanoseconds_to_absolutetime(NSEC_PER_SEC / refreshHz(), &period);
	fVSyncTime += period;
	if (fVSyncTime <= now || fVSyncTime > now + 2 * period) {
		fVSyncTime = now + period;
	}
	thread_call_enter_delayed(fVSyncCall, fVSyncTime);
}

/*
 * One refresh: tell the client, which gets three 64-bit numbers as six 32-bit halves --
 * the time of this refresh, the same again (the time it was presented), and a counter.
 */
void
IOMobileFramebufferUserClient::vsync(void)
{
	OSAsyncReference64 ref;
	io_user_reference_t args[6];
	uint64_t now;

	IOLockLock(fLock);
	now = fVSyncTime;
	if (fClosed || fCallbacks[kFBNotifyVSync] == 0 || fPorts[kFBNotifyVSync] == NULL) {
		fVSyncRunning = false;
	} else {
		fVSyncCount++;
		setAsyncReference64(ref, fPorts[kFBNotifyVSync], fCallbacks[kFBNotifyVSync], fRefcons[kFBNotifyVSync], fTask);
		args[0] = (uint32_t) now;
		args[1] = now >> 32;
		args[2] = (uint32_t) now;
		args[3] = now >> 32;
		args[4] = (uint32_t) fVSyncCount;
		args[5] = fVSyncCount >> 32;
		armVSync();
		/*
		 * sent with the lock held: registerNotificationPort() swaps the port under this lock and
		 * only then lets go of the old one, so the port in `ref` cannot be released while it is sent to
		 * (the send does not wait: a full queue drops the message)
		 */
		IOReturn sent = sendAsyncResult64WithOptions(ref, kIOReturnSuccess, args, 6, kIOUserNotifyOptionCanDrop);
		if (gTouchLat) {
			/* "tl: y COUNT due US at US SENT": the refresh told to the window server, when it was due, when it went, and
			 * whether it went (0) or was dropped on a full queue -- for the window server's idle gaps (tools/frames.py) */
			hakios_tl("tl: y %llu due %llu at %llu %#x\n", fVSyncCount, micros(now), micros(mach_absolute_time()), sent);
		}
	}
	IOLockUnlock(fLock);
}

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments * args,
    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference)
{
	const uint64_t * in = args->scalarInput;
	uint64_t * out = args->scalarOutput;
	MiniIOSurface * s;

	if (selector < 128 && gFBCalls[selector] < gFBTrace) {
		char procname[32] = "?";

		gFBCalls[selector]++;
		proc_selfname(procname, sizeof(procname));
		IOLog("MiniIOMobileFramebuffer: %s[%d] selector %u (scalars %u in [0x%llx 0x%llx 0x%llx 0x%llx], %u out; "
		    "structure %u in, %u out)\n", procname, proc_selfpid(), selector, args->scalarInputCount,
		    args->scalarInputCount > 0 ? in[0] : 0, args->scalarInputCount > 1 ? in[1] : 0,
		    args->scalarInputCount > 2 ? in[2] : 0, args->scalarInputCount > 3 ? in[3] : 0, args->scalarOutputCount,
		    args->structureInputSize, args->structureOutputSize);
	}

	switch (selector) {
	case kFBMethodGetDisplaySize:                   /* out: width, height */
		if (args->scalarOutputCount < 2) {
			return kIOReturnBadArgument;
		}
		{
			uint32_t w, h;

			shownSize(fOwner->fWidth, fOwner->fHeight, &w, &h);
			out[0] = w;
			out[1] = h;
		}
		args->scalarOutputCount = 2;
		return kIOReturnSuccess;

	case kFBMethodGetID:
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = 1;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodGetDotPitch:                      /* out: pixels per inch */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = kFBDotsPerInch * fOwner->fWidth / kFBDefaultWidth;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodGetLayerDefaultSurface:           /* in: size wanted (or 0, 0); out: a send right for the surface */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		s = fOwner->defaultSurface();
		if (s == NULL) {
			return kIOReturnNoMemory;
		}
		out[0] = MiniIOSurfaceMakeSendRight(fTask, s);
		args->scalarOutputCount = 1;
		return out[0] != 0 ? kIOReturnSuccess : kIOReturnNoMemory;

	case kFBMethodSwapBegin:                        /* out: the number of the new swap */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = OSIncrementAtomic(&fOwner->fSwapID) + 1;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodSwapEnd:
		if (args->structureInput == NULL || args->structureInputSize < SWAP_SIZE) {
			IOLog("MiniIOMobileFramebuffer: swap request of %u bytes, expected %u\n", args->structureInputSize, SWAP_SIZE);
			return kIOReturnBadArgument;
		}
		fOwner->present((const uint8_t *) args->structureInput);
		return kIOReturnSuccess;

	case kFBMethodSwapWait:                         /* a swap is on the screen as soon as it is made */
		if (gTouchLat) {
			hakios_tl("tl: w %llu %#llx %#llx at %llu\n", args->scalarInputCount > 0 ? in[0] : 0,
			    args->scalarInputCount > 1 ? in[1] : 0, args->scalarInputCount > 2 ? in[2] : 0, micros(mach_absolute_time()));
		}
		return kIOReturnSuccess;
	case kFBMethodSwapSignal:
	case kFBMethodSwapCancel:
	case kFBMethodSwapCancelAll:
	case kFBMethodRequestPowerChange:
	case kFBMethodChangeFrameInfo:
		return kIOReturnSuccess;

	case kFBMethodNotifications:                    /* in: callback (0 = off), its argument, type, 0 */
		if (args->scalarInputCount < 3 || in[2] >= kFBNotifyTypes) {
			return kIOReturnBadArgument;
		}
		IOLockLock(fLock);
		fCallbacks[in[2]] = in[0];
		fRefcons[in[2]] = in[1];
		if (in[2] == kFBNotifyVSync && in[0] != 0 && !fVSyncRunning && !fClosed) {
			fVSyncRunning = true;
			armVSync();
		}
		IOLockUnlock(fLock);
		return kIOReturnSuccess;

	case kFBMethodSupportedFrameInfo:               /* in: type; out: how many, and their tags: none */
		if (args->scalarOutputCount >= 1) {
			out[0] = 0;
			args->scalarOutputCount = 1;
		}
		if (args->structureOutput != NULL) {
			bzero(args->structureOutput, args->structureOutputSize);
		}
		return kIOReturnSuccess;

	case kFBMethodGetBlock:                         /* in: which block; out: its bytes */
		if (args->structureOutput == NULL) {
			return kIOReturnBadArgument;
		}
		bzero(args->structureOutput, args->structureOutputSize);
		if (args->scalarInputCount >= 1 && in[0] == 0x40 && args->structureOutputSize >= 12) {
			/* the current mode: its colour and timing identifiers */
			uint32_t ids[2] = { kFBColorID, kFBTimingID };
			unsigned timingID, colorID;

			hakios_hdmitx_current(&timingID, &colorID);
			if (timingID != 0) {
				ids[0] = colorID;
				ids[1] = timingID;
			}

			memcpy((uint8_t *) args->structureOutput + 4, ids, sizeof(ids));
		}
		return kIOReturnSuccess;

	case kFBMethodGetDigitalOutState:               /* out: the state of the digital output (HDMI) */
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = gDigitalOutState;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	case kFBMethodSetDigitalOutMode:                /* in: colour element, timing element (Settings > Video and Audio) */
		if (args->scalarInputCount < 2) {
			return kIOReturnBadArgument;
		}
		return hakios_hdmitx_set_output((unsigned) in[1], (unsigned) in[0]) ? kIOReturnSuccess : kIOReturnUnsupported;

	case kFBMethodGetCurrentAbsoluteTime:
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		out[0] = mach_absolute_time();
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;

	default:
		break;
	}

	if (selector < 128 && !(gFBLogged[selector / 64] & (1ULL << (selector % 64)))) {
		gFBLogged[selector / 64] |= 1ULL << (selector % 64);
		IOLog("MiniIOMobileFramebuffer: selector %u not implemented (scalars %u in [0x%llx 0x%llx 0x%llx], %u out; "
		    "structure %u in, %u out)\n", selector, args->scalarInputCount, args->scalarInputCount > 0 ? in[0] : 0,
		    args->scalarInputCount > 1 ? in[1] : 0, args->scalarInputCount > 2 ? in[2] : 0, args->scalarOutputCount,
		    args->structureInputSize, args->structureOutputSize);
	}
	/* a request that only sets something is accepted; one that wants an answer gets zeros */
	if (args->scalarOutputCount != 0) {
		bzero(out, args->scalarOutputCount * sizeof(out[0]));
	}
	if (args->structureOutput != NULL && args->structureOutputSize != 0) {
		bzero(args->structureOutput, args->structureOutputSize);
	}
	return kIOReturnSuccess;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * IOMobileFramebuffer.framework counts the displays it should wait for by looking for
 * children of /arm-io in the device tree whose device_type is "display-subsystem".
 */
static void
publishDisplayNode(void)
{
	IORegistryEntry * armio = IORegistryEntry::fromPath("/arm-io", gIODTPlane);
	IORegistryEntry * node;
	OSDictionary * props;
	OSData * data;

	if (armio == NULL) {
		IOLog("MiniIOMobileFramebuffer: no /arm-io in the device tree\n");
		return;
	}
	node = OSTypeAlloc(IORegistryEntry);
	props = OSDictionary::withCapacity(4);
	if (node != NULL && props != NULL) {
		data = OSData::withBytes("display-subsystem", sizeof("display-subsystem"));
		props->setObject("device_type", data);
		OSSafeReleaseNULL(data);
		data = OSData::withBytes("redwood,framebuffer", sizeof("redwood,framebuffer"));
		props->setObject("compatible", data);
		OSSafeReleaseNULL(data);
		data = OSData::withBytes("disp0", sizeof("disp0"));
		props->setObject("name", data);
		OSSafeReleaseNULL(data);
		if (node->init(props)) {
			node->setName("disp0");
			node->attachToParent(armio, gIODTPlane);
		}
	}
	OSSafeReleaseNULL(props);
	OSSafeReleaseNULL(node);
	armio->release();
}

/*
 * The graphics accelerator. There is none, but CoreAnimation waits half a minute for a
 * service of this class to appear before it looks for a GPU at all (once per process). An
 * accelerator without the properties that name its Metal and OpenGL ES drivers is passed
 * over by both, which leaves CoreAnimation with its software renderer, as before.
 */
class IOAcceleratorES : public IOService
{
	OSDeclareDefaultStructors(IOAcceleratorES);
};

OSDefineMetaClassAndStructors(IOAcceleratorES, IOService);

static void
publishAccelerator(IOService * provider)
{
	IOAcceleratorES * accelerator = OSTypeAlloc(IOAcceleratorES);

	if (accelerator != NULL && accelerator->init() && accelerator->attach(provider)) {
		uint32_t metal = 0;

		accelerator->setName("sgx");
		/*
		 * Boot-arg metal=1: say where Metal's device is (docs/metal-plan.md, step 1). Metal reads these two
		 * properties, opens /System/Library/Extensions/<name>.bundle and takes the class (userland/src/HakiosMetal.m).
		 * Never by default: with them a program that asks for Metal gets a device, and CoreAnimation would choose
		 * Metal for the screen; until that device can draw, a screen drawn without it is the working one.
		 */
		if (PE_parse_boot_argn("metal", &metal, sizeof(metal)) && metal != 0) {
			accelerator->setProperty("MetalPluginName", "NativeMaliMetal");
			accelerator->setProperty("MetalPluginClassName", "NativeMaliMetalDevice");
			IOLog("MiniIOMobileFramebuffer: metal=1: the accelerator names NativeMaliMetal (the Mali-G310's, over PanVK) as its Metal plug-in\n");
			/*
			 * Boot-arg gles=1 as well: OpenGL ES (EAGLContext; libGFXShared reads these) is Apple's GL-on-Metal
			 * renderer, AppleMetalGLRenderer.bundle, drawing with that Metal device -- what an AGX GPU names.
			 */
			uint32_t gles = 0;
			if (PE_parse_boot_argn("gles", &gles, sizeof(gles)) && gles != 0) {
				accelerator->setProperty("IOGLESBundleName", "AppleMetalGLRenderer");
				accelerator->setProperty("IOGLESMetalBundleName", "AppleMetalGLRenderer");
				IOLog("MiniIOMobileFramebuffer: gles=1: OpenGL ES through AppleMetalGLRenderer\n");
			}
		}
		accelerator->registerService();
	}
	OSSafeReleaseNULL(accelerator);
}

/*
 * A picture from inside the kernel (the GPU driver's first ones): rows of 32-bit pixels, put at
 * the top left corner of the panel as they are and cut to its size.
 */
extern "C" void
MiniIOMobileFramebufferShow(const uint8_t * pixels, uint32_t width, uint32_t height, uint32_t rowBytes)
{
	uint32_t columns = width < vinfo.v_width ? width : (uint32_t) vinfo.v_width;
	uint32_t rows = height < vinfo.v_height ? height : (uint32_t) vinfo.v_height;

	if (vinfo.v_baseaddr == 0 || vinfo.v_depth != 32) {
		return;
	}
	for (uint32_t y = 0; y < rows; y++) {
		memcpy((void *) (vinfo.v_baseaddr + (size_t) y * vinfo.v_rowbytes), pixels + (size_t) y * rowBytes,
		    (size_t) columns * 4);
	}
}

void
MiniIOMobileFramebufferStart(IOService * provider)
{
	IOMobileFramebuffer * fb = OSTypeAlloc(IOMobileFramebuffer);

	if (fb == NULL || !fb->init() || !fb->attach(provider)) {
		OSSafeReleaseNULL(fb);
		return;
	}
	if (fb->start(provider)) {
		publishDisplayNode();
		publishAccelerator(provider);
	} else {
		fb->detach(provider);
	}
	fb->release();
}
