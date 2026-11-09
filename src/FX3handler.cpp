//
// FX3handler.cpp 
// 2020 10 12  Oscar Steila ik1xpv
// loading arm code.img from resource by Howard Su and Hayati Ayguen
// This module was previous named:
// openFX3.cpp MIT License Copyright (c) 2016 Booya Corp.
// booyasdr@gmail.com, http://booyasdr.sf.net
// modified 2017 11 30 ik1xpv@gmail.com, http://www.steila.com/blog
// 
#include <windows.h>
#include <setupapi.h>   // v4.195: UsbDevicePresent bus pre-check
#include <string.h>
#include "config.h"
#include "FX3handler.h"
#include "CyAPI.h"
#include "cyioctl.h"
#include "firmware.h"

#define firmware_data ((const UCHAR *)FIRMWARE)
#define firmware_size sizeof(FIRMWARE)

// v4.173: FX3Command values sent to the RX666 firmware. Disassembled from the
// official ExtIO_RX666.dll (symbols intact, 2026-09-18):
//   - GPIO writes use vendor request 0xBC (not GPIOFX3/0xAD), Value=0x00C0,
//     Index=0, one data byte (GPIO bit mask; same HF attenuator bit layout
//     as BBRF103 family: SEL0=bit3, SEL1=bit4).
//   - STARTADC(0xB2), TUNERSTDBY(0xB8), RESETFX3(0xB1) are NOT implemented by
//     the RX666 firmware (measured FAILED; official DLL never sends them).
//   - STARTFX3(0xAA)/STOPFX3(0xAB)/TESTFX3(0xAC)/I2C(0xAE/0xAF) are shared.
#define FX3CMD_RX666_GPIO       0xBC
#define FX3CMD_RX666_GPIO_VALUE 0x00C0

// Which firmware the streamer is running. Set in Open() from the USB device
// name ('FX3' = RX666.img firmware, 'SDR' = RX888-family firmware).
static bool gFwIsRx666 = false;

// ============================================================================
// v4.195 guarded-reopen ownership protocol (the 10-27 fix). READ BEFORE TOUCHING.
//
// 10-27 incident: after a USB CRC error the firmware DMA was dead, and
// recovery attempt 2/3 hung FOREVER inside CyAPI's CreateFile (kernel PnP
// lock, no timeout) -> the ADC thread froze for 44 minutes and the skimmer
// died silently. The recovery path now re-opens the device inside a bounded
// guard thread so the ADC thread can never hang again.
//
// Ownership (who-allocates-deletes, strictly enforced):
//   - The guard thread allocates its own CCyFX3Device, publishes it into
//     fx3dev only while Open() runs, and is the ONLY one who ever deletes
//     it. If the recovery thread times out, the guard parks on its scene
//     event, wakes when the kernel wait finally breaks (VBUS replug/driver
//     reset), closes+deletes its own device and exits.
//   - The collection path (wdCollectGuard) NEVER touches the abandoned
//     device: no Close(), no delete — the guard may still be executing
//     inside it. It only clears the bookkeeping (busy flag + scene pointer
//     + member pointer if it still points at the zombie).
//   - Structured teardown (wdTeardownStrayGuard): fire the abandoned
//     guard's scene event so it can finish; still no device touch.
//   - Result: no double-free, no use-after-free window, and the ADC thread
//     always keeps a 10 s bound on any re-open attempt.
// ============================================================================

// v4.173: map our 32-bit gpios mask to the RX666 8-bit GPIO byte.

// v4.173: map our 32-bit gpios mask to the RX666 8-bit GPIO byte.
// Our layout: ATT_SEL0=bit13, ATT_SEL1=bit14 (RX888 32-bit GPIO words).
// RX666 byte: SEL0=bit3, SEL1=bit4; SHDWN=bit5, DITH=bit6, RAND=bit7 are
// already identical. bits 13/14 -> 3/4 via >>10.
static UINT8 Rx666GpioByte(UINT32 mask)
{
	return (UINT8)((mask & 0xFF) | ((mask >> 10) & 0x18));
}


fx3class* CreateUsbHandler()
{
	return new fx3handler();
}

fx3handler::fx3handler():
	fx3dev (nullptr),
	Fx3IsOn (false),
	devidx (0)
{
	// v4.195: watchdog state is per-scene (allocated by ReopenWatchdog),
	// nothing to pre-create here anymore.
}

// v4.195: structured teardown of a guard thread left behind by a timed-out
// ReopenWatchdog. Called from the destructor / Close(). Fires the abandoned
// guard's scene event so it can wake, self-clean and exit whenever its
// kernel wait finally breaks. We do NOT wait for it (it may be wedged
// forever) and we do NOT touch its device/scene/event (guard-owned).
void fx3handler::wdTeardownStrayGuard()
{
	if (!wdGuardBusy || !wdGuardScene)
		return;   // no abandoned guard outstanding
	SetEvent(wdGuardScene->doneEvt);
	wdGuardScene = nullptr;
	wdGuardBusy = 0;
	DbgPrintf("wdTeardownStrayGuard: abandoned guard released\n");
}


fx3handler::~fx3handler() // reset USB device and exit
{
	DbgPrintf("\r\n~fx3handler\r\n");
	// v4.195: release a stray guard before destroying the handler itself
	wdTeardownStrayGuard();
	Close();
}

char* wchar2char(const wchar_t* wchar)
{
	char* m_char;
	int len = WideCharToMultiByte(CP_ACP, 0, wchar, wcslen(wchar), NULL, 0, NULL, NULL);
	m_char = new char[len + 1];
	WideCharToMultiByte(CP_ACP, 0, wchar, wcslen(wchar), m_char, len, NULL, NULL);
	m_char[len] = '\0';
	return m_char;
}

bool fx3handler::GetFx3DeviceStreamer() {   // open class 
	bool r = false;
	if (fx3dev == NULL) return r;
	fx3dev->Open(devidx);
	if ((fx3dev->VendorID == VENDOR_ID) && (fx3dev->ProductID == STREAMER_ID)) r = true;
	if (r == false)
		fx3dev->Close();
	return r;
}

bool fx3handler::Enumerate(unsigned char& idx, char* lbuf)
{
	bool r = false;
	strcpy(lbuf, "");
	if (fx3dev == nullptr)
		fx3dev = new CCyFX3Device;              // instantiate the device
	if (fx3dev == nullptr) return r;		// return if failed
	if (!fx3dev->Open(idx)) return r;

	// v4.172: RX666 firmware selection. Measured facts (fx3_probe, 2026-09-18):
	//  - Built-in SDDC_FX3.img (RX888 build) on RX666 hardware: name='SDR',
	//    Model=0, control commands accepted, ADC never starts (bulk-in silent).
	//  - RX666.img on RX666 hardware: name='FX3', Model=0, ADC works.
	//  - RX666 firmware does NOT implement RESETFX3 (write fails); the RX888
	//    firmware does. So: only the 'SDR'-named firmware can be software-reset.
	//  - TESTFX3 payload differs per firmware (RX666: boot counter, no version
	//    field), so firmware identity is decided by the USB device name.
	// Policy:
	//  - Bootloader state -> load RX666.img if present, else built-in image.
	//  - Running 'SDR' (RX888 fw) with Model=0 on this host -> likely wrong
	//    firmware for RX666 hardware: RESETFX3 (supported) -> load RX666.img.
	//  - Running 'FX3' (RX666 fw): keep, no reload (cannot reset anyway).
	// v4.189: img 缺失时拒绝 fallback 到内置 RX888 固件（作用域：整个 Enumerate，
	// 在下方代码块设置、函数尾判断）。
	// 本机是 RX666 硬件，内置 SDDC_FX3.img（RX888 build）刷进 RX666 会得到
	// 'SDR'+Model=0 的半死状态（能 Open 但 bulk-in 永远无声，v4.172 实测），
	// 且这个错固件靠 USB 供电驻留 RAM，之后每次会话都卡在同一状态。
	// 处置：bootloader 状态 + img 缺失 → 不下载任何固件，报警并让 Enumerate
	// 失败（lbuf 置空），促使调用方报 no device——用户看到日志即知放 img。
	bool imgMissingAlarm = false;

	{
		bool bootNow = fx3dev->IsBootLoaderRunning();
		DbgPrintf("Enumerate: boot=%d name='%s'\n", bootNow, fx3dev->DeviceName);

		char imgPath[MAX_PATH];
		DWORD n = GetModuleFileNameA(nullptr, imgPath, MAX_PATH); // host exe dir
		if (n == 0 || n >= MAX_PATH) imgPath[0] = 0;
		else {
			char* slash = strrchr(imgPath, '\\');
			if (slash) *(slash + 1) = 0; else imgPath[0] = 0;
			strcat(imgPath, "RX666.img");
		}
		DWORD imgAttr = GetFileAttributesA(imgPath);
		bool imgExists = (imgAttr != INVALID_FILE_ATTRIBUTES);
		DbgPrintf("Enumerate: RX666.img probe %s (%s)\n", imgExists ? "FOUND" : "absent", imgPath);

		if (!bootNow && imgExists &&
			strcmp(fx3dev->DeviceName, "SDR") == 0)
		{
			// RX888-build firmware running. Read Model to see if it recognized
			// the hardware. Model=0 on an 'SDR'-named firmware means the
			// hardware strap was not recognized (RX666 case) -> reload.
			uint32_t info = 0;
			uint8_t infoData[4] = {0,0,0,0};
			{
				CCyControlEndPoint* ept = fx3dev->ControlEndPt;
				ept->ReqCode = TESTFX3;
				ept->Value = 0;
				ept->Index = 0;
				long infoLen = 4;
				if (ept->Read((PUCHAR)infoData, infoLen)) info = *(uint32_t*)infoData;
			}
			DbgPrintf("Enumerate: 'SDR' fw TESTFX3 info=0x%08X (Model=%u)\n", info, (unsigned)(info & 0xFF));
			if ((info & 0xFF) == 0) {
				DbgPrintf("Enumerate: wrong firmware on RX666 - RESETFX3 + load RX666.img\n");
				uint8_t rst = 0;
				long one = 1;
				{
					CCyControlEndPoint* ept = fx3dev->ControlEndPt;
					ept->ReqCode = RESETFX3;
					ept->Value = 0;
					ept->Index = 0;
					ept->Write((PUCHAR)&rst, one);
				}
				fx3dev->Close();
				Sleep(3000);  // USB re-enumeration settle time
				if (!fx3dev->Open(idx)) { Sleep(2000); fx3dev->Open(idx); }
				DbgPrintf("Enumerate: after reset open=%d boot=%d\n",
					fx3dev->IsOpen() ? 1 : 0, fx3dev->IsOpen() ? (fx3dev->IsBootLoaderRunning() ? 1 : 0) : -1);
			}
		}

		if (fx3dev->IsOpen() && fx3dev->IsBootLoaderRunning()) {
			// v4.172: bootloader state — prefer local RX666.img, fallback to built-in.
			const UCHAR* fwData = firmware_data;
			UINT fwSize = (UINT)firmware_size;
			const char* fwName = "built-in SDDC_FX3.img";
			if (imgExists) {
				FILE* f = fopen(imgPath, "rb");
				if (f) {
					static unsigned char imgBuf[262144];
					size_t got = fread(imgBuf, 1, sizeof(imgBuf), f);
					fclose(f);
					if (got > 0 && got <= sizeof(imgBuf)) {
						fwData = imgBuf;
						fwSize = (UINT)got;
						fwName = "RX666.img";
					}
				}
				DbgPrintf("Enumerate: bootloader - using %s (%u bytes)\n", fwName, fwSize);
			}
			// v4.189: img 不在（或读到 0 字节）——绝不能把内置 RX888 固件刷进
			// RX666 硬件制造 'SDR' 僵死态。报警、跳过下载、本次 Enumerate 失败。
			else {
				DbgPrintf("Enumerate: *** RX666.img MISSING (%s) - refusing built-in "
					"RX888 firmware fallback to avoid 'SDR' zombie state. "
					"Copy RX666.img next to the DLL/exe and replug. ***\n", imgPath);
				imgMissingAlarm = true;
			}
			if (!imgMissingAlarm && fx3dev->DownloadFwToRam(fwData, fwSize) != SUCCESS) {
				DbgPrintf("Failed to DownloadFwToRam device(%x)\n", idx);
			}
			else if (!imgMissingAlarm) {
				fx3dev->Close();
				Sleep(800);					    // wait after firmware change ?
				fx3dev->Open(idx);
			}
		}
		if (fx3dev->IsOpen() && !fx3dev->IsBootLoaderRunning()) {
			uint32_t info2 = 0;
			uint8_t info2Data[4] = {0,0,0,0};
			CCyControlEndPoint* ept2 = fx3dev->ControlEndPt;
			ept2->ReqCode = TESTFX3;
			ept2->Value = 0;
			ept2->Index = 0;
			long infoLen2 = 4;
			if (ept2->Read((PUCHAR)info2Data, infoLen2)) info2 = *(uint32_t*)info2Data;
			DbgPrintf("Enumerate: final fw name='%s' TESTFX3=0x%08X (Model=%u)\n",
				fx3dev->DeviceName, info2, (unsigned)(info2 & 0xFF));
		}
	}
	// v4.189: img 缺失报警路径——不把 'WestBridge'（bootloader）当作可用设备
	// 报告成功，否则 GetSdrInfo 会 present=1 但 StartRx 永远等不到 ADC 数据，
	// 用户无从知道根因。置空 lbuf 并返回 false，让 UsbTryOpen 记 no device。
	if (imgMissingAlarm) {
		strcpy(lbuf, "");
		fx3dev->Close();
		devidx = 0;
		return false;
	}
	strcpy (lbuf, fx3dev->DeviceName);
	while (strlen(lbuf) < 18) strcat(lbuf, " ");
	strcat(lbuf, "sn:");
	strcat(lbuf, wchar2char((wchar_t*)fx3dev->SerialNumber));
	fx3dev->Close();
	devidx = idx;  // -> devidx
	return true;
}

// v4.195: bus-presence pre-check. Enumerate any Cypress device (VID 0x04B4)
// via SetupAPI. This walks only the device list — it never touches the PnP
// lock, so it cannot hang the way CyAPI CreateFile did on 10-27.
// Returns true when at least one Cypress VID device node is present.
bool fx3handler::UsbDevicePresent()
{
	HDEVINFO hDevInfo = SetupDiGetClassDevsA(nullptr, "USB", nullptr,
		DIGCF_PRESENT | DIGCF_ALLCLASSES);
	if (hDevInfo == INVALID_HANDLE_VALUE) {
		DbgPrintf("UsbDevicePresent: SetupDiGetClassDevs failed\n");
		return false;  // conservative: no bus visibility -> don't even try
	}

	bool present = false;
	SP_DEVINFO_DATA devInfo;
	devInfo.cbSize = sizeof(devInfo);
	for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfo); i++) {
		char devId[MAX_PATH];
		DWORD sz = 0;
		if (!SetupDiGetDeviceInstanceIdA(hDevInfo, &devInfo, devId, sizeof(devId), &sz))
			continue;
		// match "USB\VID_04B4&..." (Cypress VID; RX888 streamer 00F1, RX666
		// streamer 00F1, bootloader 00F3 — one check covers all states)
		if (strncmp(devId, "USB\\VID_04B4", 12) == 0) {
			DbgPrintf("UsbDevicePresent: found %s\n", devId);
			present = true;
			break;
		}
	}
	SetupDiDestroyDeviceInfoList(hDevInfo);
	if (!present)
		DbgPrintf("UsbDevicePresent: no Cypress VID_04B4 device on the bus\n");
	return present;
}

bool  fx3handler::Open() {
	bool r = false;

	// v4.195: never leave a stale endpoint from a previous device here —
	// Open() failure paths must leave EndPt null (the guard-thread protocol
	// reads it via scene->endPt only on success).
	EndPt = nullptr;

	if (!GetFx3DeviceStreamer()) {
		DbgPrintf("Failed to open device\n");
		return r;
	}
	// v4.173: remember which firmware is running (name is decisive, see
	// Enumerate() notes: 'FX3' = RX666 fw, 'SDR' = RX888-family fw).
	gFwIsRx666 = (strcmp(fx3dev->DeviceName, "FX3") == 0);
	DbgPrintf("Open: device name='%s' -> command set %s\n",
		fx3dev->DeviceName, gFwIsRx666 ? "RX666 (GPIO=0xBC)" : "RX888-family (GPIO=0xAD)");
	EndPt = fx3dev->BulkInEndPt;
	if (!EndPt) {
		DbgPrintf("No Bulk In end point\n");
		return r;      // init failed
	}

	long pktSize = EndPt->MaxPktSize;
	EndPt->SetXferSize(transferSize);
	long ppx = transferSize / pktSize;
	DbgPrintf("buffer transferSize = %d. packet size = %ld. packets per transfer = %ld\n"
		, transferSize, pktSize, ppx);

	uint8_t data[4] = {0,0,0,0};
	// v4.195: GetHardwareInfo failure closes+deletes the device (upstream
	// behavior). Fail the open cleanly here instead of falling into the
	// version-mismatch branch below, which would Control(RESETFX3) on the
	// deleted object (latent NULL-deref on RX888-family firmware; RX666 was
	// silently saved by the unsupported-command suppression).
	if (!GetHardwareInfo((uint32_t*)&data)) {
		DbgPrintf("Open: GetHardwareInfo failed - device closed, aborting open\n");
		return false;
	}

	// v4.172: TESTFX3 payload layout is firmware-specific. The RX888 firmware
	// returns {Model, VerMajor, VerMinor, 0} (2.2 = 0x00020200); the RX666
	// firmware returns {Model=0, bootLo, 0, bootHi} — a boot counter with no
	// version field (measured 0xFF000000→0xFC030000 across resets). Version
	// checking only applies when the payload looks like a version pair;
	// otherwise (RX666 fw) skip it.
	bool looksLikeVersion = (data[3] == 0) && (data[1] <= 16) && (data[2] <= 99);
	if (looksLikeVersion &&
		(data[1] != FIRMWARE_VER_MAJOR ||
		data[2] != FIRMWARE_VER_MINOR))
	{
		DbgPrintf("Firmware version mismatch %d.%d != %d.%d (actual)\n", FIRMWARE_VER_MAJOR, FIRMWARE_VER_MINOR, data[1], data[2]);
		Control(RESETFX3);
		return false;
	}
	if (!looksLikeVersion) {
		DbgPrintf("Open: fw payload 0x%02X%02X%02X%02X - non-version layout, skip version check\n",
			data[3], data[2], data[1], data[0]);
	}

	Fx3IsOn = true;
	return Fx3IsOn;          // init success
}


using namespace std;

bool fx3handler::Control(FX3Command command, UINT8 data) { // firmware control BBRF
	long lgt = 1;

	// v4.173: commands the RX666 firmware does not implement (official DLL
	// never sends them; measured FAILED). Report success to keep the caller's
	// init flow unchanged, no USB transaction is attempted.
	if (gFwIsRx666 &&
		(command == STARTADC || command == TUNERSTDBY || command == RESETFX3)) {
		DbgPrintf("FX3FWControl %x .%x %x (RX666: unsupported cmd, suppressed)\n", 1, command, data);
		return true;
	}

	if (gFwIsRx666 && command == GPIOFX3) {
		// v4.173: translate GPIO write to the RX666 vendor request 0xBC
		// (Value=0xC0, Index=0, 1 data byte, attenuator bits remapped).
		CCyControlEndPoint* ept = fx3dev->ControlEndPt;
		ept->ReqCode = (UCHAR)FX3CMD_RX666_GPIO;
		ept->Value = (USHORT)FX3CMD_RX666_GPIO_VALUE;
		ept->Index = (USHORT)0;
		UINT8 b = Rx666GpioByte((UINT32)data);
		bool rr = ept->Write(&b, lgt);
		DbgPrintf("FX3FWControl %x .%x %x (RX666 GPIO as 0xBC V=0x%x)\n", rr, FX3CMD_RX666_GPIO, b, FX3CMD_RX666_GPIO_VALUE);
		if (rr == false)
		{
			DbgPrintf("FX3FWControl FAILED (cmd=0x%x data=0x%x) - NOT closing device\n", command, data);
		}
		return rr;
	}

	fx3dev->ControlEndPt->ReqCode = command;
	fx3dev->ControlEndPt->Value = (USHORT)0;
	fx3dev->ControlEndPt->Index = (USHORT)0;
	bool r = fx3dev->ControlEndPt->Write(&data, lgt);
	DbgPrintf("FX3FWControl %x .%x %x\n", r, command, data);
	if (r == false)
	{
		// v4.24p: Don't Close() on control failure during USB reconnect
		// The device is still physically connected, just temporarily unavailable
		DbgPrintf("FX3FWControl FAILED (cmd=0x%x data=0x%x) - NOT closing device\n", command, data);
	}
	return r;
}

bool fx3handler::Control(FX3Command command, UINT32 data) { // firmware control BBRF
	long lgt = 4;

	// v4.173: same RX666 handling as the UINT8 overload. STARTADC arrives
	// here (adc_rate as UINT32); suppress instead of sending an unsupported
	// request. GPIOFX3 with a 32-bit mask also lands here.
	if (gFwIsRx666 &&
		(command == STARTADC || command == TUNERSTDBY || command == RESETFX3)) {
		DbgPrintf("FX3FWControl %x .%x %x (RX666: unsupported cmd, suppressed)\n", 1, command, data);
		return true;
	}
	if (gFwIsRx666 && command == GPIOFX3) {
		// forward to the 1-byte path (GPIO payload is one byte on RX666)
		return Control(command, Rx666GpioByte(data));
	}

	fx3dev->ControlEndPt->ReqCode = command;
	fx3dev->ControlEndPt->Value = (USHORT)0;
	fx3dev->ControlEndPt->Index = (USHORT)0;
	bool r = fx3dev->ControlEndPt->Write((PUCHAR)&data, lgt);
	DbgPrintf("FX3FWControl %x .%x %x\n", r, command, data);
	if (r == false)
	{
		DbgPrintf("FX3FWControl FAILED (cmd=0x%x) - NOT closing device\n", command);
	}
	return r;
}

bool fx3handler::Control(FX3Command command, UINT64 data) { // firmware control BBRF
	long lgt = 8;

	fx3dev->ControlEndPt->ReqCode = command;
	fx3dev->ControlEndPt->Value = (USHORT)0;
	fx3dev->ControlEndPt->Index = (USHORT)0;
	bool r = fx3dev->ControlEndPt->Write((PUCHAR)&data, lgt);
	DbgPrintf("FX3FWControl %x .%x %llx\n", r, command, data);
	if (r == false)
	{
		DbgPrintf("FX3FWControl FAILED (cmd=0x%x) - NOT closing device\n", command);
	}
	return r;
}


bool fx3handler::SetArgument(UINT16 index, UINT16 value) { // firmware control BBRF
	long lgt = 1;
	uint8_t data = 0;

	fx3dev->ControlEndPt->ReqCode = SETARGFX3;
	fx3dev->ControlEndPt->Value = (USHORT)value;
	fx3dev->ControlEndPt->Index = (USHORT)index;
	bool r = fx3dev->ControlEndPt->Write((PUCHAR)&data, lgt);
	DbgPrintf("SetArgument %x .%x (%x, %x)\n", r, SETARGFX3, index, value);
	if (r == false)
	{
		Close();
	}
	return r;
}

bool fx3handler::GetHardwareInfo(UINT32* data) { // firmware control BBRF
	long lgt = 4;

	fx3dev->ControlEndPt->ReqCode = TESTFX3;
#ifdef _DEBUG
	fx3dev->ControlEndPt->Value = (USHORT) 1;
#else
	fx3dev->ControlEndPt->Value = (USHORT) 0;
#endif
	fx3dev->ControlEndPt->Index = (USHORT)0;
	bool r = fx3dev->ControlEndPt->Read((PUCHAR)data, lgt);
	DbgPrintf("GetHardwareInfo %x .%x %x\n", r, TESTFX3, *data);
	if (r == false)
	{
		Close();
	}
	return r;

}

bool fx3handler::ReadDebugTrace(uint8_t* pdata, uint8_t len)
{
	long lgt = len;
	bool r;
	fx3dev->ControlEndPt->ReqCode = READINFODEBUG;
	fx3dev->ControlEndPt->Value = (USHORT) pdata[0]; // upstream char
	r = fx3dev->ControlEndPt->Read((PUCHAR)pdata, lgt);
	return r;
}

bool fx3handler::SendI2cbytes(UINT8 i2caddr, UINT8 regaddr, PUINT8 pdata, UINT8 len)
{
	bool r = false;
	LONG lgt = len;
	fx3dev->ControlEndPt->ReqCode = I2CWFX3;
	fx3dev->ControlEndPt->Value = (USHORT)i2caddr;
	fx3dev->ControlEndPt->Index = (USHORT)regaddr;
	Sleep(10);
	r = fx3dev->ControlEndPt->Write(pdata, lgt);
	if (r == false)
		DbgPrintf("\nfx3FWSendI2cbytes 0x%02x regaddr 0x%02x  1data 0x%02x len 0x%02x \n",
			i2caddr, regaddr, *pdata, len);
	return r;
}

bool fx3handler::ReadI2cbytes(UINT8 i2caddr, UINT8 regaddr, PUINT8 pdata, UINT8 len)
{
	bool r = false;
	LONG lgt = len;
	WORD saveValue, saveIndex;
	saveValue = fx3dev->ControlEndPt->Value;
	saveIndex = fx3dev->ControlEndPt->Index;

	fx3dev->ControlEndPt->ReqCode = I2CRFX3;
	fx3dev->ControlEndPt->Value = (USHORT)i2caddr;
	fx3dev->ControlEndPt->Index = (USHORT)regaddr;
	r = fx3dev->ControlEndPt->Read(pdata, lgt);
	if (r == false)
		printf("fx3FWReadI2cbytes %x : %02x %02x %02x %02x : %02x\n", r, I2CRFX3, i2caddr, regaddr, len, *pdata);
	fx3dev->ControlEndPt->Value = saveValue;
	fx3dev->ControlEndPt->Index = saveIndex;
	return r;
}

bool fx3handler::Close() {
	// v4.195: release a guard left behind by a timed-out ReopenWatchdog
	// BEFORE closing our own device state (keeps who-allocates-deletes).
	wdTeardownStrayGuard();
	if (fx3dev) {               // v4.195: null-safe (fx3dev may be null after
		fx3dev->Close();        // a watchdog timeout: ownership passed to guard)
		delete fx3dev;
		fx3dev = nullptr;
	}
	Fx3IsOn = false;
	return true;
}

bool fx3handler::Reopen() {
	// v4.24q: Close and re-open USB device to get fresh handle and endpoint
	// Called from AdcSamplesProcess when USB pipe is permanently broken after VMP
	DbgPrintf("Reopen: closing and re-opening USB device\n");

	// Close current device
	if (fx3dev) {
		fx3dev->Close();
		delete fx3dev;
		fx3dev = nullptr;
	}
	Fx3IsOn = false;

	// Create new device instance
	fx3dev = new CCyFX3Device;
	if (!fx3dev) {
		DbgPrintf("Reopen: failed to create new CCyFX3Device\n");
		return false;
	}

	// Re-open and re-initialize
	if (!Open()) {
		DbgPrintf("Reopen: Open() failed\n");
		return false;
	}

	DbgPrintf("Reopen: USB device re-opened successfully, EndPt=%p\n", EndPt);
	return true;
}

// v4.195: guarded reopen block — the 10-27 fix. The device re-open runs
// inside a bounded guard thread so the ADC thread can NEVER hang again.
// Ownership rules are documented at the top of this file. The recovery loop
// in AdcSamplesProcess is the only caller.

// v4.195: static trampoline — runs the actual device open inside the guard
// thread, then exits (clean) or parks until the scene is collected (abandoned).
DWORD WINAPI fx3handler::WdOpenThunk(LPVOID param)
{
	WdOpenScene* scene = (WdOpenScene*)param;
	fx3handler* self = scene->self;

	const DWORD t0 = GetTickCount();
	scene->ok = false;

	// Fresh device instance: old object was closed/deleted by
	// ReopenWatchdog() before the guard started. The guard owns this object
	// from here on — no one else ever deletes it (who-allocates-deletes).
	CCyFX3Device* dev = new CCyFX3Device;
	if (!dev) {
		DbgPrintf("WdOpen: CCyFX3Device alloc failed\n");
		scene->dev = nullptr;
		scene->ok = false;
		return 0;   // thread ends; ReopenWatchdog wait sees ok=false
	}
	scene->dev = dev;

	// Open() routes through the member fx3dev pointer (GetFx3DeviceStreamer),
	// so publish our device there for the duration of the attempt. On the
	// clean path ReopenWatchdog takes it over; on the timeout path
	// wdCollectGuard() nulls the member before the guard ever deletes dev.
	self->fx3dev = dev;

	// Bind and open. Open() runs GetFx3DeviceStreamer -> CyAPI Open ->
	// CreateFile — the exact call that hung forever on 10-27. From inside
	// this dedicated thread it can only burn THIS thread, never the ADC
	// pipeline. Open() leaves EndPt null on failure (v4.195 fix: the old
	// code left a dangling endpoint from the previous device there).
	if (self->Open()) {
		scene->ok = true;
		scene->endPt = self->EndPt;
		DbgPrintf("WdOpen: Open() ok (t=%lu ms)\n", GetTickCount() - t0);
	}
	else {
		scene->ok = false;
		DbgPrintf("WdOpen: Open() failed (t=%lu ms)\n", GetTickCount() - t0);
	}

	// v4.195: ownership re-check. Open()'s internal failure paths
	// (GetHardwareInfo) close+delete the member device; if that consumed our
	// dev, forget it — the pointer is dangling and must never be touched.
	if (self->fx3dev != dev) {
		scene->dev = nullptr;
		dev = nullptr;
	}

	if (InterlockedCompareExchange(&scene->abandoned, 0, 0)) {
		// Recovery thread gave up on us. Park on the scene event; when
		// wdCollectGuard()/wdTeardownStrayGuard() fires it, clean up and
		// exit. Ownership of dev/scene/event passed to us at the timeout;
		// nobody else ever frees them.
		DbgPrintf("WdOpen: abandoned by recovery path, parking\n");
		WaitForSingleObject(scene->doneEvt, INFINITE);
		// Belt-and-braces: detach the member slot if it still points at our
		// device (cannot happen with the publish-once protocol, but stays
		// safe under future refactors).
		if (self->fx3dev == dev)
			self->fx3dev = nullptr;
		if (dev) {
			dev->Close();
			delete dev;
		}
		CloseHandle(scene->doneEvt);
		delete scene;
		DbgPrintf("WdOpen: abandoned guard cleaned up, exiting\n");
	}
	return 0;
}

// v4.195: bounded device re-open. Returns true only when Open() completed
// within WD_OPEN_TIMEOUT_MS. On timeout the guard thread is detached with
// its own device; wdGuardBusy stays 1 until wdCollectGuard().
bool fx3handler::ReopenWatchdog()
{
	const DWORD WD_OPEN_TIMEOUT_MS = 10000;

	// close + delete whatever the previous state left, so the guard starts
	// from a clean slate (only legal while wdGuardBusy == 0)
	if (fx3dev) {
		fx3dev->Close();
		delete fx3dev;
		fx3dev = nullptr;
	}
	Fx3IsOn = false;
	EndPt = nullptr;

	WdOpenScene* scene = new WdOpenScene;
	scene->self = this;
	scene->dev = nullptr;
	scene->endPt = nullptr;
	scene->ok = false;
	scene->abandoned = 0;

	// per-scene collect event (manual reset, initially unsignalled)
	scene->doneEvt = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!scene->doneEvt) {
		DbgPrintf("ReopenWatchdog: CreateEvent failed\n");
		delete scene;
		return false;
	}

	wdGuardThread = CreateThread(nullptr, 0, WdOpenThunk, scene, 0, nullptr);
	if (!wdGuardThread) {
		DbgPrintf("ReopenWatchdog: CreateThread failed, falling back to sync open\n");
		// No guard thread available — better to try synchronously than to
		// give up without trying (hang risk accepted in this degraded mode).
		CloseHandle(scene->doneEvt);
		delete scene;
		return Reopen();
	}

	DWORD w = WaitForSingleObject(wdGuardThread, WD_OPEN_TIMEOUT_MS);
	if (w == WAIT_OBJECT_0) {
		// guard exited on its own (clean or failed-open path)
		CloseHandle(wdGuardThread);
		wdGuardThread = nullptr;
		CloseHandle(scene->doneEvt);
		bool ok = scene->ok;
		if (scene->dev && ok) {
			fx3dev = scene->dev;          // adopt the guard's device
			EndPt = scene->endPt;
			Fx3IsOn = true;
		}
		else if (scene->dev) {
			// open failed: fx3dev still points at the guard's closed device
			// (needed for logging); mark not-on. The guard already exited.
			fx3dev = scene->dev;
			EndPt = nullptr;
			Fx3IsOn = false;
		}
		// guard exited; fx3dev points at its device — we adopt it even on
		// failure so the next ReopenWatchdog close+delete stays balanced.
		delete scene;
		DbgPrintf("ReopenWatchdog: finished ok=%d\n", ok ? 1 : 0);
		return ok;
	}

	// ---- timeout path: detach, the guard keeps its device forever ----
	DbgPrintf("ReopenWatchdog: TIMEOUT after %lu ms — device open hung "
		"(10-27 signature). Detaching guard thread.\n", WD_OPEN_TIMEOUT_MS);
	InterlockedExchange(&scene->abandoned, 1);

	// v4.195: 500 ms grace window. If the guard was already past its
		// abandoned check (read 0 before our write) it exits without parking;
	// taking the clean path here avoids leaking its device/scene/event.
	// A guard that saw abandoned==1 parks forever, so exit-within-grace
	// unambiguously means it skipped the park.
	DWORD w2 = WaitForSingleObject(wdGuardThread, 500);
	if (w2 == WAIT_OBJECT_0) {
		CloseHandle(wdGuardThread);
		wdGuardThread = nullptr;
		bool ok = scene->ok;
		if (scene->dev && ok) {
			fx3dev = scene->dev;
			EndPt = scene->endPt;
			Fx3IsOn = true;
		}
		else if (scene->dev) {
			fx3dev = scene->dev;      // closed-but-allocated; next round frees it
			EndPt = nullptr;
			Fx3IsOn = false;
		}
		CloseHandle(scene->doneEvt);
		delete scene;
		DbgPrintf("ReopenWatchdog: guard exited during grace, ok=%d\n", ok ? 1 : 0);
		return ok;
	}

	CloseHandle(wdGuardThread);   // detach (thread keeps running)
	wdGuardThread = nullptr;
	fx3dev = nullptr;             // member never dangles at the guard's device
	EndPt = nullptr;
	Fx3IsOn = false;
	wdGuardScene = scene;         // remember for collect/teardown; busy stays 1
	wdGuardBusy = 1;
	return false;
}

// v4.195: release an abandoned guard scene. Called at the start of every
// recovery attempt that follows a watchdog timeout, and from teardown.
// NEVER touches the abandoned device: the guard may still be executing
// inside its kernel wait — it will close+delete its own device, scene and
// event after waking (ownership of all three passed to the guard at
// InterlockedExchange(&scene->abandoned, 1)).
void fx3handler::wdCollectGuard()
{
	if (!wdGuardBusy || !wdGuardScene)
		return;
	SetEvent(wdGuardScene->doneEvt);  // wake the parked guard; it self-cleans
	wdGuardScene = nullptr;
	wdGuardBusy = 0;
	DbgPrintf("wdCollectGuard: abandoned guard released, budget re-armed\n");
}

#define BLOCK_TIMEOUT (80) // block 65.536 ms timeout is 80

struct ReadContext
{
	PUCHAR context;
	OVERLAPPED overlap;
	SINGLE_TRANSFER transfer;
	uint8_t* buffer;
	long size;
};

bool fx3handler::BeginDataXfer(UINT8 *buffer, long transferSize, void** context)
{
	ReadContext *readContext = (ReadContext *)(*context);

	if (!EndPt)
		return false;

	if (*context == nullptr)
	{
		// first time call, allocate the context structure
		readContext = new ReadContext;
		*context = readContext;
		memset(&readContext->overlap, 0, sizeof(readContext->overlap));
		readContext->overlap.hEvent = CreateEvent(NULL, false, false, NULL);
	}

	readContext->buffer = buffer;
	readContext->size = transferSize;

	readContext->context = EndPt->BeginDataXfer(readContext->buffer, transferSize, &readContext->overlap);
	if (EndPt->NtStatus || EndPt->UsbdStatus) {// BeginDataXfer failed
		DbgPrintf("Xfer request rejected. 1 STATUS = %ld %ld\n", EndPt->NtStatus, EndPt->UsbdStatus);
		return false;
	}

	return true;
}

bool fx3handler::FinishDataXfer(void** context)
{
	ReadContext *readContext = (ReadContext *)(*context);

	if (!readContext)
	{
		return false;
	}

	if (!EndPt->WaitForXfer(&readContext->overlap, BLOCK_TIMEOUT)) { // block on transfer
		// v4.195: log fix — both numbers used to be readContext->size (the
		// REQUESTED length), which carried no information. Report the
		// requested length once plus the live NTSTATUS.
		DbgPrintf("WaitForXfer timeout after %d ms (requested %ld bytes) NTSTATUS = 0x%08X\n",
			(int)BLOCK_TIMEOUT, readContext->size, EndPt->NtStatus);
		EndPt->Abort(); // abort if timeout
		return false;
	}

	auto requested_size = readContext->size;
	if (!EndPt->FinishDataXfer(readContext->buffer, readContext->size, &readContext->overlap, readContext->context)) {
		DbgPrintf("FinishDataXfer Failed. NTSTATUS = 0x%08X UsbdStatus=0x%08X (size %ld)\n",
			EndPt->NtStatus, EndPt->UsbdStatus, requested_size);
		return false;
	}

	if (readContext->size < requested_size)
		DbgPrintf("only read %ld but requested %ld\n",  readContext->size, requested_size);

	return true;
}

void fx3handler::CleanupDataXfer(void** context)
{
	ReadContext *readContext = (ReadContext *)(*context);

	CloseHandle(readContext->overlap.hEvent);
	delete (readContext);
}

#define USB_READ_CONCURRENT 4

// v4.174: maximum auto-recovery attempts before giving up.
// Each attempt does STOPFX3 -> Close -> Reopen -> STARTFX3 -> re-queue.
// v4.195: budget now counts ATTEMPTS (not successes): a firmware-DMA
// half-death can produce endless fail->recover->fail-again cycles that all
// "succeed" individually while the flow never survives a full second —
// that storm is capped by the same budget via USB_STORM_WINDOW_MS.
#define USB_MAX_RECOVER 3

// v4.174: recovery delay between attempts (ms), gives USB/firmware time to settle.
#define USB_RECOVER_DELAY_MS 1000

// v4.195: recovery-storm window. A recovery that survives less than this
// long before failing again counts as the SAME incident (the counter is NOT
// reset by micro-recoveries). Only genuine stability (>= USB_STORM_WINDOW_MS
// of continuous data flow) grants a fresh budget.
#define USB_STORM_WINDOW_MS 60000

void fx3handler::AdcSamplesProcess()
{
	DbgPrintf("AdcSamplesProc thread runs\n");
	int buf_idx;            // queue index
	int read_idx;
	unsigned long usbBlockCount = 0;   // v4.175: write-side counter (diagnostic)
	void*		contexts[USB_READ_CONCURRENT];

	memset(contexts, 0, sizeof(contexts));

	// Queue-up the first batch of transfer requests
	for (int n = 0; n < USB_READ_CONCURRENT; n++) {
		auto ptr = inputbuffer->peekWritePtr(n);
		if (!BeginDataXfer((uint8_t*)ptr, transferSize, &contexts[n])) {
			DbgPrintf("Xfer request rejected.\n");
			return;
		}
	}

	read_idx = 0;	// context cycle index
	buf_idx = 0;	// buffer cycle index

	// The infinite xfer loop.
	int recoverCount = 0;   // v4.174: attempts used of the current budget
	unsigned long lastRecoverTick = 0;  // v4.195: GetTickCount of last recovery end
	bool recoveryFirstAttempt = true;   // v4.195: true until first recovery in this incident
	while (run) {
 		if (!FinishDataXfer(&contexts[read_idx])) {
 			// v4.174: auto-recovery instead of immediate break.
 			// On FinishDataXfer failure the USB bulk pipe or firmware DMA
 			// is in a bad state.  Try to recover by cycling the USB
 			// handle and restarting the FX3 producer.  If recovery
 			// succeeds, data flow resumes seamlessly.
 			// v4.195: budget counts ATTEMPTS within a storm window — a
 			// fail->micro-recover->fail-again cycle burns the same budget
 			// as clean failures; only 60 s of stable flow refills it.
 			// If all attempts fail, the thread exits with a loud alarm
 			// block (was: silent exit, skimmer died without a trace).
			if (recoverCount >= USB_MAX_RECOVER) {
				DbgPrintfForce("\n");
				DbgPrintfForce("======================================================================\n");
				DbgPrintfForce("  USB RECOVERY EXHAUSTED after %d attempts (" RX888INTF_VERSION ")\n", recoverCount);
				DbgPrintfForce("  Root cause: firmware DMA / USB pipe hard-dead (CRC errors,\n");
				DbgPrintfForce("  10-27 incident signature). RX666 firmware has no software reset.\n");
				DbgPrintfForce("  ACTION REQUIRED: unplug+replug the USB cable, then restart\n");
				DbgPrintfForce("  SkimSrv. The ADC thread is stopping NOW to avoid writing a\n");
				DbgPrintfForce("  half-dead stream into the DSP.\n");
				DbgPrintfForce("======================================================================\n");
				DbgPrintfForce("\n");
				break;
			}

			// v4.195: re-entry after a watchdog timeout — the abandoned guard
			// owns its own scene/device/event; release the scene BEFORE
			// touching anything (no device touch, just bookkeeping).
			if (wdGuardBusy)
				wdCollectGuard();

			// Clean up pending transfers (contexts may be in indeterminate
			// state) — MUST happen on every failure path, including the
			// pre-check/continue loops below, or contexts leak per round.
			for (int n = 0; n < USB_READ_CONCURRENT; n++) {
				if (contexts[n])
					CleanupDataXfer(&contexts[n]);
				contexts[n] = nullptr;
			}

 			// v4.195: bus pre-check — if no Cypress device is on the bus, a
 			// CyAPI Open can hang on the kernel PnP lock (10-27 signature).
 			// Skip the open entirely and let the budget burn down.
 			if (!UsbDevicePresent()) {
 				DbgPrintf("Recovery: bus pre-check FAILED - no Cypress device on bus\n");
 				Sleep(USB_RECOVER_DELAY_MS);
 				recoverCount++;   // this attempt IS consumed (prevents infinite retry)
 				continue;
 			}

 			recoverCount++;
 			DbgPrintf("AdcSamplesProc: USB failure, attempting recovery %d/%d\n",
 				recoverCount, USB_MAX_RECOVER);

			// STOPFX3 — tell firmware to stop the producer (only when the
			// device object is still alive: after a timeout+takeOwnership the
			// member pointer is null and Control() would dereference null)
			if (fx3dev) {
				DbgPrintf("Recovery: sending STOPFX3\n");
				Control(STOPFX3);
			} else {
				DbgPrintf("Recovery: no device object, skipping STOPFX3\n");
			}

 			Sleep(USB_RECOVER_DELAY_MS);

 			// v4.195: guarded reopen with 10 s watchdog (was: naked Reopen()
 			// that could hang forever inside CyAPI CreateFile, 10-27).
 			DbgPrintf("Recovery: reopening USB device (guarded)\n");
 			if (!ReopenWatchdog()) {
				// Budget was already consumed above; let the loop head decide
				// whether another attempt is allowed (was: break — that
				// resurrected the 10-27 silent-exit bug for this path).
				DbgPrintf("Recovery: guarded reopen failed, retry budget pending\n");
				continue;
			}

 			Sleep(USB_RECOVER_DELAY_MS);

 			// STARTFX3 — restart the producer
 			DbgPrintf("Recovery: sending STARTFX3\n");
 			Control(STARTFX3);

 			Sleep(500);  // brief settle for DMA to spin up

 			// Re-queue all transfer requests with the fresh endpoint
 			inputbuffer->Start();  // reset ring buffer indices
 			bool queueOk = true;
 			for (int n = 0; n < USB_READ_CONCURRENT; n++) {
 				auto ptr = inputbuffer->peekWritePtr(n);
 				if (!BeginDataXfer((uint8_t*)ptr, transferSize, &contexts[n])) {
 					DbgPrintf("Recovery: BeginDataXfer rejected at n=%d\n", n);
 					queueOk = false;
 					break;
 				}
 			}
 			if (!queueOk) {
				// contexts are all nullptr here (cleanup ran at failure entry,
				// re-queue cleared the ones that did submit? no: re-queue fills
				// contexts[n] as it goes, so clean up whatever got created)
				for (int n = 0; n < USB_READ_CONCURRENT; n++) {
					if (contexts[n])
						CleanupDataXfer(&contexts[n]);
					contexts[n] = nullptr;
				}
				DbgPrintf("Recovery: re-queue failed, retry budget pending\n");
				continue;
			}

 			read_idx = 0;
 			buf_idx = 0;
 			lastRecoverTick = GetTickCount();
 			recoveryFirstAttempt = false;
 			DbgPrintf("Recovery %d/%d: re-queued, resuming data flow\n",
 				recoverCount, USB_MAX_RECOVER);
 			continue;  // retry the loop with fresh transfers
 		}

		// v4.195: recovery-storm accounting — only a flow that has been
		// running stably for the full storm window closes the incident and
		// grants a fresh budget; micro-recoveries do NOT reset the counter.
		if (recoverCount > 0 && !recoveryFirstAttempt &&
			lastRecoverTick != 0 &&
			(GetTickCount() - lastRecoverTick) >= USB_STORM_WINDOW_MS) {
			DbgPrintf("Recovery: flow stable >=%d ms, fresh recovery budget\n",
				(int)USB_STORM_WINDOW_MS);
			recoverCount = 0;
			recoveryFirstAttempt = true;
			lastRecoverTick = 0;
		}

		inputbuffer->WriteDone();

		// v4.175: per-100-block diagnostic (write-side)
		if ((++usbBlockCount % 100) == 0)
			DbgPrintf("adc: %d blocks written to ring\n", usbBlockCount);

		// Re-submit this queue element to keep the queue full
		auto ptr = inputbuffer->peekWritePtr(USB_READ_CONCURRENT - 1);
		if (!BeginDataXfer((uint8_t*)ptr, transferSize, &contexts[read_idx])) { // BeginDataXfer failed
			DbgPrintf("Xfer request rejected.\n");
			break;
		}

		buf_idx = (buf_idx + 1) % QUEUE_SIZE;
		read_idx = (read_idx + 1) % USB_READ_CONCURRENT;
	}  // End of the infinite loop

	for (int n = 0; n < USB_READ_CONCURRENT; n++) {
		if (contexts[n])
			CleanupDataXfer(&contexts[n]);
	}

	DbgPrintf("AdcSamplesProc thread_exit\n");
	return;  // void *
}

// v4.176: 用 Win32 线程替代 std::thread——MinGW 32 位 + -fopenmp 下
// std::thread 析构（delete）在 join 后仍崩溃（0xC0000005），疑 libstdc++/libgomp
// 线程局部存储冲突。CreateThread/WaitForSingleObject/CloseHandle 不受影响。
static DWORD WINAPI AdcSamplesProcThunk(LPVOID param) {
    auto self = static_cast<fx3handler*>(param);
    self->AdcSamplesProcess();
    return 0;
}

void fx3handler::StartStream(ringbuffer<int16_t>& input, int numofblock)
{
	// Allocate the context and buffers
	inputbuffer = &input;

	inputbuffer->setBlockSize(transferSamples);

	// create the thread
	this->numofblock = numofblock;
	run = true;
	adc_thread_handle = CreateThread(nullptr, 0, AdcSamplesProcThunk, this, 0, nullptr);
}

void fx3handler::StopStream()
{
	DbgPrintf("StopStream: set run=false\n");
	run = false;
	DbgPrintf("StopStream: WaitForSingleObject begin\n");
	if (adc_thread_handle) {
		DWORD w = WaitForSingleObject(adc_thread_handle, 10000);
		if (w == WAIT_TIMEOUT) {
			// v4.195: recovery rounds can now take up to ~13 s each (watchdog
			// bound), so a stop during recovery may legitimately exceed the
			// 10 s join. Detach (handle close) but DO NOT null inputbuffer:
			// the ring object outlives this call (StopRx never deletes it,
			// v4.177 leak policy), and the ADC thread touches it once more
			// before honouring run=false. Nulling it here was a use-after-
			// null race (StopStream caller vs recovery path).
			// v4.198: DbgPrintf -> DbgPrintfForce——脱管属于必须知晓的事件
			//（残留 ADC 线程会拖住进程退出，v4.191 实测 ~40 s），LogMode=0
			// 静默模式下同样必须落盘。
			DbgPrintfForce("StopStream: ADC thread still busy after 10 s (recovery in "
				"progress?) - detached, inputbuffer left valid (process exit "
				"may be delayed by up to ~40 s)\n");
		}
		else {
			DbgPrintf("StopStream: WaitForSingleObject done\n");
			inputbuffer = nullptr;
		}
		CloseHandle(adc_thread_handle);
		adc_thread_handle = nullptr;
		DbgPrintf("StopStream: CloseHandle done\n");
	}
}

void fx3handler::ForceStopStream()
{
	// Set flag and detach thread (safe when VMP suspended the thread)
	run = false;
	if (adc_thread_handle) {
		CloseHandle(adc_thread_handle);  // 线程仍运行，仅释放句柄
		adc_thread_handle = nullptr;
		DbgPrintf("AdcSamplesProc thread handle closed (detached)\n");
	}
	// Don't null inputbuffer — the detached thread still needs it briefly
}

void fx3handler::ForceStartStream(ringbuffer<int16_t>& input, int numofblock)
{
	// Start after ForceStopStream (no StopStream call)
	inputbuffer = &input;
	inputbuffer->setBlockSize(transferSamples);

	this->numofblock = numofblock;
	run = true;
	adc_thread_handle = CreateThread(nullptr, 0, AdcSamplesProcThunk, this, 0, nullptr);
}

void fx3handler::CancelPendingIO()
{
	// Cancel all pending USB I/O on the device endpoint
	// This forces any blocked FinishDataXfer calls to return with an error
	if (fx3dev) {
		HANDLE hDev = fx3dev->DeviceHandle();
		if (hDev != INVALID_HANDLE_VALUE) {
			CancelIoEx(hDev, NULL);
		}
	}
}
