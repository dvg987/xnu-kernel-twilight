/*
 * HakiosBluetooth: the registry entry by which Darwin's Bluetooth side finds its controller.
 *
 * bluetoothd and BlueTool look up a service named "bluetooth" (or "marconi-bt", which must not exist) and read three
 * properties of it, as a phone or an Apple TV has them in its device tree's bluetooth node: transport-encoding (3: H4
 * on a UART, with BT_WAKE), transport-speed, and local-mac-address -- without an address, or with one of zeros,
 * bluetoothd says "No bluetooth on this device" and leaves for good. The controller behind it is whatever the kernel
 * bridges to the UART's kernel control (com.apple.uart.bluetooth; the W2's in drivers/wifi/hakios.w2bt.c).
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSData.h>

extern "C" void HakiosBluetoothStart(const uint8_t * address);

class HakiosBluetooth : public IOService
{
	OSDeclareDefaultStructors(HakiosBluetooth);
};

OSDefineMetaClassAndStructors(HakiosBluetooth, IOService);

static void
data_property(IOService * service, const char * key, const void * bytes, unsigned length)
{
	OSData * data = OSData::withBytes(bytes, length);

	if (data != NULL) {
		service->setProperty(key, data);
		data->release();
	}
}

/* Once, when the controller's address is known (`address' as it is written: the most significant byte first). */
void
HakiosBluetoothStart(const uint8_t * address)
{
	IOService * platform = IOService::getPlatform();
	HakiosBluetooth * bt = OSTypeAlloc(HakiosBluetooth);
	const uint32_t encoding = 3, speed = 3000000;

	if (platform == NULL || bt == NULL || !bt->init()) {
		OSSafeReleaseNULL(bt);
		IOLog("HakiosBluetooth: cannot create the service\n");
		return;
	}
	data_property(bt, "transport-encoding", &encoding, sizeof(encoding));
	data_property(bt, "transport-speed", &speed, sizeof(speed));
	data_property(bt, "local-mac-address", address, 6);
	bt->setProperty("device_type", "bluetooth");
	if (!bt->attach(platform)) {
		bt->release();
		return;
	}
	bt->setName("bluetooth");
	if (!bt->start(platform)) {
		bt->detach(platform);
		bt->release();
		return;
	}
	bt->registerService();
	IOLog("HakiosBluetooth: the controller's service is there\n");
	bt->release();
}
