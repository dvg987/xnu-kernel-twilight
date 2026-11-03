/*
 * IOHIDResource: where a program makes a HID device of its own (IOHIDUserDeviceCreateWithOptions), as BTLEServer does
 * for each Bluetooth LE accessory it has paired. IOHIDFamily, whose service this is on Apple's systems, is not here,
 * and nothing would take such a device's events either; so this one reads each device's reports itself, with the
 * report descriptor the program gave, and posts the keys pressed to /dev/hakios-touch, where backboardd's remote
 * (tvos/src/twlinput.inc) has them: whatever remote is paired, tvOS sees its own.
 *
 * The contract, as IOKit.framework's IOHIDUserDevice of tvOS 16.5 keeps it (tvos/docs/w2-bt.md has the evidence):
 *   IOServiceOpen type 0; selector 0 create (scalar: options, struct: the properties as XML), 2 a report (scalar:
 *   time, struct: the report with its ID byte if the descriptor has IDs), 3 a get/set report's answer, 4 publish the
 *   device (options bit 0: not before), 5 large report memory given back. IOConnectMapMemory type 0 must give an
 *   IODataQueue (get/set report requests come that way: none are sent here) and IOConnectGetService the device.
 */
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOSharedDataQueue.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSUnserialize.h>

extern "C" void hakios_touch_post(uint32_t type, uint32_t finger, uint32_t x, uint32_t y);
extern "C" void HakiosHIDResourceStart(void);

#define HAKIOS_BUTTON_DOWN      4               /* sys/hakios.input.h's, which iokit/ does not see */
#define HAKIOS_BUTTON_UP        5

#define HID_FIELDS      64
#define HID_USAGES      16
#define HID_HELD        16

/* one Input main item of a report descriptor */
struct hid_field {
	uint8_t id;                     /* its report's ID, 0 without IDs */
	uint8_t size, count;            /* of each value, in bits; how many */
	bool array;                     /* values are usage indexes (else one value per usage) */
	uint16_t bit;                   /* where its first value starts in the report (after the ID byte) */
	uint16_t page;
	int32_t min, max;               /* logical */
	uint32_t umin, umax;            /* usages as a range (umax >= umin), or ... */
	uint8_t nusages;                /* ... as a list */
	uint32_t usages[HID_USAGES];    /* page << 16 | usage */
};

struct hid_parser {
	struct hid_field field[HID_FIELDS];
	unsigned fields;
	bool ids;                       /* the reports start with an ID byte */
};

/* a usage of a report -> the code of a key record, 0 if tvOS has nothing for it (what each does was found on the box) */
static uint32_t
hid_key(uint32_t usage)
{
	uint16_t page = usage >> 16, u = usage & 0xffff;

	if (page == 0x0c) {
		switch (u) {
		case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: /* select, up, down, left, right */
		case 0xcd: case 0xe9: case 0xea: case 0xe2:             /* play/pause, volume, mute */
			return usage;
		case 0x224:
			return 0x070029;                /* AC Back: a keyboard's Escape is tvOS's back */
		case 0x223: case 0x40:
			return 0x0c0040;                /* AC Home, Menu: the consumer Menu is tvOS's home */
		}
		return 0;
	}
	if (page == 0x07 && u >= 0x04 && u <= 0x65) {
		return usage;                           /* a keyboard's key (arrows, return, escape: the remote's own) */
	}
	return 0;
}

static void
hid_parse(struct hid_parser * h, const uint8_t * d, unsigned n)
{
	struct { uint16_t page; int32_t min, max; uint32_t umax; uint8_t size, count, id; } g = {}, stack[4];
	unsigned depth = 0;
	uint32_t usages[HID_USAGES] = {}, umin = 0, umax = 0;
	unsigned nusages = 0;
	bool range = false;
	uint16_t bits[256] = {};

	h->fields = 0;
	h->ids = false;
	for (unsigned i = 0; i < n;) {
		uint8_t b = d[i];
		unsigned size = (b & 3) == 3 ? 4 : (b & 3), type = (b >> 2) & 3, tag = b >> 4;
		uint32_t u = 0;
		int32_t s;

		if (b == 0xfe) {                                        /* a long item */
			i += i + 1 < n ? 3 + d[i + 1] : n;
			continue;
		}
		if (i + 1 + size > n) {
			break;
		}
		for (unsigned k = 0; k < size; k++) {
			u |= (uint32_t)d[i + 1 + k] << (8 * k);
		}
		s = size == 1 ? (int8_t)u : size == 2 ? (int16_t)u : (int32_t)u;
		i += 1 + size;
		switch (type) {
		case 0:                                                 /* main */
			if (tag == 0x8 && g.count != 0 && g.size != 0 && g.size <= 32) {       /* Input */
				if (!(u & 1) && h->fields < HID_FIELDS) {      /* not constant (padding) */
					struct hid_field * f = &h->field[h->fields++];

					f->id = g.id;
					f->size = g.size;
					f->count = g.count;
					f->array = !(u & 2);
					f->bit = bits[g.id];
					f->page = g.page;
					f->min = g.min;
					f->max = g.min >= 0 && g.max < 0 ? (int32_t)g.umax : g.max;    /* 0..255 written as 1 byte */
					f->umin = range ? umin : 0;
					f->umax = range ? umax : 0;
					f->nusages = range ? 0 : (uint8_t)nusages;
					memcpy(f->usages, usages, sizeof(usages));
				}
				bits[g.id] += g.size * g.count;
			}
			nusages = 0;
			range = false;
			break;
		case 1:                                                 /* global */
			switch (tag) {
			case 0x0: g.page = (uint16_t)u; break;
			case 0x1: g.min = s; break;
			case 0x2: g.max = s; g.umax = u; break;
			case 0x7: g.size = (uint8_t)u; break;
			case 0x8: g.id = (uint8_t)u; h->ids = true; break;
			case 0x9: g.count = (uint8_t)u; break;
			case 0xa: if (depth < 4) { stack[depth++] = g; } break;
			case 0xb: if (depth > 0) { g = stack[--depth]; } break;
			}
			break;
		case 2:                                                 /* local: a usage of 4 bytes has its own page */
			if (size < 4) {
				u |= (uint32_t)g.page << 16;
			}
			if (tag == 0x0 && nusages < HID_USAGES) {
				usages[nusages++] = u;
			} else if (tag == 0x1) {
				umin = u;
				range = true;
			} else if (tag == 0x2) {
				umax = u;
				range = true;
			}
			break;
		}
	}
}

static uint32_t
hid_bits(const uint8_t * p, unsigned n, unsigned bit, unsigned size)
{
	uint32_t v = 0;

	for (unsigned k = 0; k < size; k++, bit++) {
		if (bit / 8 < n && (p[bit / 8] >> (bit % 8) & 1)) {
			v |= 1u << k;
		}
	}
	return v;
}

/* the usages of report `id' held down in `p' (its bytes after the ID), as key codes */
static unsigned
hid_held(const struct hid_parser * h, uint8_t id, const uint8_t * p, unsigned n, uint32_t * held)
{
	unsigned count = 0;

	for (unsigned i = 0; i < h->fields; i++) {
		const struct hid_field * f = &h->field[i];

		if (f->id != id) {
			continue;
		}
		for (unsigned k = 0; k < f->count && count < HID_HELD; k++) {
			uint32_t v = hid_bits(p, n, f->bit + k * f->size, f->size), usage = 0;

			if (f->array) {
				int32_t index = (int32_t)v - f->min;

				if (v == 0 || (int32_t)v < f->min || (int32_t)v > f->max) {
					continue;
				}
				if (f->umax >= f->umin && f->umax != 0) {
					usage = f->umin + (uint32_t)index;
					if ((usage >> 16) == 0) {
						usage |= (uint32_t)f->page << 16;
					}
				} else if ((unsigned)index < f->nusages) {
					usage = f->usages[index];
				}
			} else {
				if (v == 0 || f->size > 8) {                    /* an axis or a count, not a key */
					continue;
				}
				if (f->umax >= f->umin && f->umax != 0) {
					usage = f->umin + k;
					if ((usage >> 16) == 0) {
						usage |= (uint32_t)f->page << 16;
					}
				} else if (f->nusages != 0) {
					usage = f->usages[k < f->nusages ? k : f->nusages - 1];
				}
			}
			if (hid_key(usage) != 0) {
				held[count++] = hid_key(usage);
			}
		}
	}
	return count;
}

class IOHIDResource : public IOService
{
	OSDeclareDefaultStructors(IOHIDResource);
};

OSDefineMetaClassAndStructors(IOHIDResource, IOService);

/* the device a program made: its properties, for whoever looks (IOHIDUserDeviceCopyProperty) */
class IOHIDUserDevice : public IOService
{
	OSDeclareDefaultStructors(IOHIDUserDevice);
};

OSDefineMetaClassAndStructors(IOHIDUserDevice, IOService);

class IOHIDResourceDeviceUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(IOHIDResourceDeviceUserClient);

	IOHIDUserDevice *       fDevice;
	IOSharedDataQueue *     fQueue;
	uint64_t                fOptions;
	char                    fName[48];
	struct hid_parser *     fParser;
	struct { uint8_t id; uint8_t count; uint32_t code[HID_HELD]; } fHeld[8];
	unsigned                fReports;

	IOReturn create(IOExternalMethodArguments * args);
	IOReturn report(IOExternalMethodArguments * args);
	void stopDevice(void);

public:
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientDied(void) APPLE_KEXT_OVERRIDE;
	virtual IOService * getService(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits * options, IOMemoryDescriptor ** memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments * args,
	    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(IOHIDResourceDeviceUserClient, IOUserClient);

bool
IOHIDResourceDeviceUserClient::start(IOService * provider)
{
	if (!IOUserClient::start(provider)) {
		return false;
	}
	fQueue = IOSharedDataQueue::withCapacity(16384);
	fParser = (struct hid_parser *)IOMallocZero(sizeof(struct hid_parser));
	return fQueue != NULL && fParser != NULL;
}

void
IOHIDResourceDeviceUserClient::free(void)
{
	OSSafeReleaseNULL(fQueue);
	OSSafeReleaseNULL(fDevice);
	if (fParser != NULL) {
		IOFree(fParser, sizeof(struct hid_parser));
		fParser = NULL;
	}
	IOUserClient::free();
}

/* whatever is still held is let go: a remote that leaves keeps no key down */
void
IOHIDResourceDeviceUserClient::stopDevice(void)
{
	for (unsigned r = 0; r < 8; r++) {
		for (unsigned k = 0; k < fHeld[r].count; k++) {
			hakios_touch_post(HAKIOS_BUTTON_UP, fHeld[r].code[k], 0, 0);
		}
		fHeld[r].count = 0;
	}
	if (fDevice != NULL) {
		fDevice->terminate();
		IOLog("HakiosHID: \"%s\" is gone (%u reports)\n", fName, fReports);
	}
}

IOReturn
IOHIDResourceDeviceUserClient::clientClose(void)
{
	stopDevice();
	terminate();
	return kIOReturnSuccess;
}

IOReturn
IOHIDResourceDeviceUserClient::clientDied(void)
{
	return clientClose();
}

IOService *
IOHIDResourceDeviceUserClient::getService(void)
{
	return fDevice;
}

IOReturn
IOHIDResourceDeviceUserClient::clientMemoryForType(UInt32 type, IOOptionBits * options, IOMemoryDescriptor ** memory)
{
	if (type != 0 || fQueue == NULL) {
		return kIOReturnBadArgument;
	}
	*memory = fQueue->getMemoryDescriptor();         /* a new one: the caller releases it */
	*options = 0;
	return *memory != NULL ? kIOReturnSuccess : kIOReturnNoMemory;
}

IOReturn
IOHIDResourceDeviceUserClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
{
	(void)type;
	(void)refCon;
	if (fQueue != NULL) {
		fQueue->setNotificationPort(port);
	}
	return kIOReturnSuccess;
}

/* the struct input of a call, from the message or (more than 4 KiB) from the caller's memory; IOFree it */
static uint8_t *
struct_input(IOExternalMethodArguments * args, unsigned * length)
{
	uint8_t * bytes;
	unsigned n;

	if (args->structureInputDescriptor != NULL) {
		n = (unsigned)args->structureInputDescriptor->getLength();
	} else {
		n = args->structureInputSize;
	}
	if (n == 0 || n > 65536 || (bytes = (uint8_t *)IOMalloc(n + 1)) == NULL) {
		return NULL;
	}
	if (args->structureInputDescriptor != NULL) {
		if (args->structureInputDescriptor->prepare() != kIOReturnSuccess) {
			IOFree(bytes, n + 1);
			return NULL;
		}
		args->structureInputDescriptor->readBytes(0, bytes, n);
		args->structureInputDescriptor->complete();
	} else {
		memcpy(bytes, args->structureInput, n);
	}
	bytes[n] = 0;
	*length = n;
	return bytes;
}

IOReturn
IOHIDResourceDeviceUserClient::create(IOExternalMethodArguments * args)
{
	unsigned length = 0;
	uint8_t * xml;
	OSObject * object;
	OSDictionary * props;
	OSData * descriptor;
	OSString * product;

	if (fDevice != NULL) {
		return kIOReturnBusy;
	}
	if ((xml = struct_input(args, &length)) == NULL) {
		return kIOReturnBadArgument;
	}
	object = OSUnserializeXML((const char *)xml, length + 1);
	IOFree(xml, length + 1);
	props = OSDynamicCast(OSDictionary, object);
	if (props == NULL) {
		OSSafeReleaseNULL(object);
		return kIOReturnBadArgument;
	}
	fOptions = args->scalarInputCount > 0 ? args->scalarInput[0] : 0;
	product = OSDynamicCast(OSString, props->getObject("Product"));
	strlcpy(fName, product != NULL ? product->getCStringNoCopy() : "HID device", sizeof(fName));
	descriptor = OSDynamicCast(OSData, props->getObject("ReportDescriptor"));
	if (descriptor != NULL) {
		hid_parse(fParser, (const uint8_t *)descriptor->getBytesNoCopy(), descriptor->getLength());
	}
	fDevice = OSTypeAlloc(IOHIDUserDevice);
	if (fDevice == NULL || !fDevice->init(props)) {
		OSSafeReleaseNULL(fDevice);
		props->release();
		return kIOReturnNoMemory;
	}
	props->release();
	fDevice->setProperty("DeviceOpenedByEventSystem", true);     /* (BTLEServer's Siri Remote holds its keys till then) */
	if (!fDevice->attach(this)) {
		OSSafeReleaseNULL(fDevice);
		return kIOReturnNoMemory;
	}
	if (!(fOptions & 1)) {
		fDevice->registerService();
	}
	IOLog("HakiosHID: \"%s\": descriptor of %u bytes, %u input fields%s\n", fName,
	    descriptor != NULL ? descriptor->getLength() : 0, fParser->fields, fParser->ids ? " (report IDs)" : "");
	return kIOReturnSuccess;
}

IOReturn
IOHIDResourceDeviceUserClient::report(IOExternalMethodArguments * args)
{
	unsigned length = 0, n, slot, count;
	uint8_t * bytes = struct_input(args, &length);
	const uint8_t * p;
	uint8_t id = 0;
	uint32_t held[HID_HELD];

	if (bytes == NULL) {
		return kIOReturnBadArgument;
	}
	p = bytes;
	n = length;
	if (fParser->ids && n > 0) {
		id = p[0];
		p++;
		n--;
	}
	if (fReports++ < 24) {
		char hex[3 * 20 + 1];
		unsigned k;

		for (k = 0; k < length && k < 20; k++) {
			snprintf(hex + 3 * k, 4, " %02x", bytes[k]);
		}
		hex[3 * k] = 0;
		IOLog("HakiosHID: \"%s\": report%s\n", fName, hex);
	}
	for (slot = 0; slot < 8 && fHeld[slot].count != 0 && fHeld[slot].id != id; slot++) {
	}
	if (slot == 8) {
		slot = 0;
	}
	count = hid_held(fParser, id, p, n, held);
	for (unsigned k = 0; k < fHeld[slot].count; k++) {                     /* let go */
		bool still = false;

		for (unsigned j = 0; j < count; j++) {
			still |= held[j] == fHeld[slot].code[k];
		}
		if (!still) {
			hakios_touch_post(HAKIOS_BUTTON_UP, fHeld[slot].code[k], 0, 0);
		}
	}
	for (unsigned j = 0; j < count; j++) {                                 /* pressed */
		bool before = false;

		for (unsigned k = 0; k < fHeld[slot].count; k++) {
			before |= held[j] == fHeld[slot].code[k];
		}
		if (!before) {
			hakios_touch_post(HAKIOS_BUTTON_DOWN, held[j], 0, 0);
			IOLog("HakiosHID: \"%s\": key %#x\n", fName, held[j]);
		}
	}
	fHeld[slot].id = id;
	fHeld[slot].count = (uint8_t)count;
	memcpy(fHeld[slot].code, held, sizeof(held));
	IOFree(bytes, length + 1);
	return kIOReturnSuccess;
}

IOReturn
IOHIDResourceDeviceUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments * args,
    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference)
{
	IOReturn ret;

	(void)dispatch;
	(void)target;
	(void)reference;
	switch (selector) {
	case 0:
		return create(args);
	case 1:
		stopDevice();
		return kIOReturnSuccess;
	case 2:
		if (fDevice == NULL) {
			return kIOReturnNotReady;
		}
		ret = report(args);
		if (args->asyncWakePort != MACH_PORT_NULL) {
			sendAsyncResult64(args->asyncReference, ret, NULL, 0);
		}
		return ret;
	case 3:                                 /* no get/set report is ever asked here */
	case 5:
		return kIOReturnSuccess;
	case 4:
		if (fDevice != NULL) {
			fDevice->registerService();
		}
		return kIOReturnSuccess;
	default:
		return kIOReturnBadArgument;
	}
}

/* once: when Darwin's Bluetooth has a controller (drivers/bluetooth/hakios.btsvc.cpp) */
void
HakiosHIDResourceStart(void)
{
	IOService * resources = IOService::getResourceService();
	IOHIDResource * service = OSTypeAlloc(IOHIDResource);

	if (resources == NULL || service == NULL || !service->init()) {
		OSSafeReleaseNULL(service);
		IOLog("HakiosHID: cannot create IOHIDResource\n");
		return;
	}
	service->setProperty(kIOUserClientClassKey, "IOHIDResourceDeviceUserClient");
	if (!service->attach(resources) || !service->start(resources)) {
		service->release();
		return;
	}
	service->registerService();
	service->release();
}
