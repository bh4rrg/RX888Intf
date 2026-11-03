#ifndef FX3HANDLER_H
#define FX3HANDLER_H

//
// FX3handler.cpp 
// 2020 10 12  Oscar Steila ik1xpv
// loading arm code.img from resource by Howard Su and Hayati Ayguen
// This module was previous named:openFX3.cpp
// MIT License Copyright (c) 2016 Booya Corp.
// booyasdr@gmail.com, http://booyasdr.sf.net
// modified 2017 11 30 ik1xpv@gmail.com, http://www.steila.com/blog
// 

#include <sys/stat.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "ringbuffer.h"

#define	VENDOR_ID     (0x04B4)
#define	STREAMER_ID   (0x00F1)
#define	BOOTLOADER_ID (0x00F3)

#include "FX3Class.h"

class CCyFX3Device;
class CCyUSBEndPoint;
class fx3handler;

// v4.195: per-attempt scene captured by the guard thread. Defined here so
// both FX3handler.h (member declarations) and FX3handler.cpp agree on it.
// LIFECYCLE: ReopenWatchdog() allocates the scene + its doneEvt; on the
// clean path it frees both after the guard exited; on the timeout path the
// abandoned guard becomes the owner and frees both after waking. Exactly
// one side ever frees — never both.
struct WdOpenScene
{
	fx3handler*    self = nullptr;
	CCyFX3Device*  dev = nullptr;   // device the guard owns (its own allocation)
	CCyUSBEndPoint* endPt = nullptr; // BulkInEndPt after a successful open
	HANDLE         doneEvt = nullptr; // manual-reset, per-scene collect event
	bool           ok = false;
	volatile LONG  abandoned = 0;   // 1: recovery thread timed out on us (interlocked)
};

class fx3handler : public fx3class
{
public:
	fx3handler();
	virtual ~fx3handler(void);

	bool Open();
	bool IsOn() { return Fx3IsOn; }
	bool Control(FX3Command command, uint8_t data);
	bool Control(FX3Command command, uint32_t data = 0);
	bool Control(FX3Command command, uint64_t data);
	bool SetArgument(uint16_t index, uint16_t value);
	bool GetHardwareInfo(uint32_t* data);
	bool ReadDebugTrace(uint8_t* pdata, uint8_t len);
	void StartStream(ringbuffer<int16_t>& input, int numofblock);
	void StopStream();
	void ForceStopStream();  // Detach thread instead of join (safe when VMP suspended)
	void ForceStartStream(ringbuffer<int16_t>& input, int numofblock);  // Start after ForceStopStream
 	void CancelPendingIO();  // Cancel all pending USB I/O (force FinishDataXfer to return)
 	bool Reopen();  // v4.24q: Close + Open to get fresh USB handle and endpoint
 	// v4.195: hardened recovery — Reopen with watchdog timeout + bus-presence
 	// pre-check + recovery-storm guard. Fix for the 10-27 incident: the ADC
 	// thread hung forever inside CyAPI CreateFile (kernel PnP lock) when the
 	// FX3 was half-dead, leaving the skimmer deaf with no alarm.
 	bool UsbDevicePresent(); // pre-check: any Cypress (VID_04B4) device on the bus?
 	bool Enumerate(unsigned char &idx, char *lbuf);
private:
	bool SendI2cbytes(uint8_t i2caddr, uint8_t regaddr, uint8_t* pdata, uint8_t len);
	bool ReadI2cbytes(uint8_t i2caddr, uint8_t regaddr, uint8_t* pdata, uint8_t len);

	bool BeginDataXfer(uint8_t *buffer, long transferSize, void** context);
	bool FinishDataXfer(void** context);
	void CleanupDataXfer(void** context);

	CCyFX3Device* fx3dev;
	CCyUSBEndPoint* EndPt;

    HANDLE adc_thread_handle = nullptr;   // v4.176: Win32 线程（替代 std::thread）

	// v4.195: guarded-Reopen watchdog state (ADC 恢复路径专用；所有权协议见
	// FX3handler.cpp 顶部状态机注释)
	static DWORD WINAPI WdOpenThunk(LPVOID param);
	bool        ReopenWatchdog();   // spawn guard thread + bounded wait
	void        wdCollectGuard();   // release an abandoned guard scene (no device touch)
	void        wdTeardownStrayGuard();  // release a timed-out guard at teardown

	HANDLE        wdGuardThread = nullptr;  // live guard thread handle
	volatile LONG wdGuardBusy  = 0;        // 1: an abandoned guard scene exists
	WdOpenScene*  wdGuardScene = nullptr;   // scene of a detached guard thread

	bool GetFx3DeviceStreamer();
	bool Fx3IsOn;
	bool Close(void);
public:
	void AdcSamplesProcess();   // v4.176: public for Win32 thunk access
private:

	ringbuffer<int16_t> *inputbuffer;
	int numofblock;
	bool run;
	UCHAR devidx;
};


#endif // FX3HANDLER_H
