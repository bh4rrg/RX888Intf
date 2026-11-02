// usb_shim.cpp - FX3handler USB 底座的 C 壳层
// v4.182: UsbOpen 补齐老驱动 RF 前端配置序列（v4.131 成功会话时序）。
// 内部持有 fx3handler* + ringbuffer<int16_t>*，
// 提供 skimsrv_intf.cpp 所需的 Usb* 接口；同时提供 Release 下
// config.h 要求的 DdcLog 桥与全局符号。
#include "usb_shim.h"
#include <windows.h>
#include <cstdio>
#include <cstdarg>
#include <cstring>

// config.h 中声明、由原 RX888Intf.cpp 提供的全局符号（此处补齐定义）
#include "config.h"
bool saveADCsamplesflag = false;
uint32_t adcnominalfreq = DEFAULT_ADC_FREQ;
uint32_t MIN_ADC_FREQ = 1000000;
uint32_t MAX_ADC_FREQ = 64000000;
uint32_t N2_BANDSWITCH = 64000000;

// v4.180: 统一日志通道——不再独立 fopen 同一文件（双 FILE* 并发写腐蚀堆）
// 改为调用 skimsrv_intf.cpp 的 SharedDdcLog（共用 FILE* + CRITICAL_SECTION）
extern "C" void SharedDdcLog(const char *fmt, va_list args);

// va_list 版本（config.h 的 DbgPrintf 调用）
extern "C" void DdcLog(const char* fmt, va_list args) {
    SharedDdcLog(fmt, args);
}

// printf 风格内部辅助（shim 自身使用）——v4.180 统一到 SharedDdcLog
static void UsbLog(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    SharedDdcLog(fmt, ap);
    va_end(ap);
}

// ---------------------------------------------------------------------------
// USB 底层
// ---------------------------------------------------------------------------
#include "FX3handler.h"
#include "ringbuffer.h"
#include "Interface.h"

static fx3handler* gDev = nullptr;

// v4.182: GPIO 状态镜像（老驱动 RadioHardware::gpios 同职责）。
// BBRF103 家族 GPIO 布局（Interface.h）：
//   SHDWN=bit5, DITH=bit6, RANDO=bit7, BIAS_HF=bit8,
//   ATT_SEL0=bit13, ATT_SEL1=bit14
static uint32_t gGpios = 0;

static bool GpioSet(uint32_t mask) {
    gGpios |= mask;
    return gDev->Control(GPIOFX3, gGpios);
}

static bool GpioUnset(uint32_t mask) {
    gGpios &= ~mask;
    return gDev->Control(GPIOFX3, gGpios);
}

bool UsbTryOpen(void** dev, int32_t idx) {
    if (gDev) { *dev = gDev; return gDev->IsOn(); }
    gDev = new fx3handler();
    if (!gDev) return false;
    unsigned char i = (unsigned char)(idx & 0xFF);
    char lbuf[128] = "";
    bool ok = false;
    try { ok = gDev->Enumerate(i, lbuf); } catch (...) { ok = false; }
    if (ok) {
        UsbLog("UsbTryOpen: device '%s'", lbuf);
        *dev = gDev;
    } else {
        UsbLog("UsbTryOpen: no device (idx=%d)", (int)idx);
    }
    return ok;
}

bool UsbOpen(void** dev, uint32_t adcFreq) {
    if (!gDev) return false;
    if (!gDev->Open()) {
        UsbLog("UsbOpen: Open() failed");
        return false;
    }

    // -----------------------------------------------------------------
    // v4.182: RF 前端配置序列——完全对齐老驱动 v4.131 成功会话时序：
    //   UpdBiasT_HF(false) -> UpdatemodeRF(HFMODE) -> UpdateattRF -> UptDither(false)
    // 老会话实测：UpdatemodeRF(HFMODE)=OK, att 钳制到 2(0dB), dither off。
    // 此前新驱动只发 STARTADC+STARTFX3，天线信号根本进不了 ADC
    // （spectrum_probe 实测 8 通道全噪声，top 峰仅高出噪底 +9~12dB），
    // 这是 decoders=0/零 spots 的硬件层根因。
    // 设备 Model=3（RX888 gen1）属 BBRF103 家族：HF 路径 =
    // TUNERSTDBY + SetGPIO(ATT_SEL0|ATT_SEL1) + 3 档 GPIO 衰减。
    // -----------------------------------------------------------------
    gGpios = 0;

    // (1) BIAS_HF off —— 无源天线，不用 bias-T
    GpioUnset(BIAS_HF);
    UsbLog("UsbOpen: RF(1) BIAS_HF off");

    // (2) HFMODE —— TUNERSTDBY（停内置调谐器）+ SetGPIO(ATT_SEL0|ATT_SEL1) 切 HF 天线路径
    bool tunerOk = gDev->Control(TUNERSTDBY);
    bool antOk = GpioSet(ATT_SEL0 | ATT_SEL1);
    UsbLog("UsbOpen: RF(2) HFMODE: TUNERSTDBY=%d ANT_SEL(SEL0|SEL1)=%d", (int)tunerOk, (int)antOk);

    // (3) HF 衰减档位：老会话 ini HfAttenuation=0 -> attIdx=2 -> SEL1=1/SEL0=0 -> 0dB
    //     （hfsteps={-20,-10,0}，attIdx 0/1/2 对应 -20/-10/0dB；SEL0=1/SEL1=0 是 -10dB，
    //       SEL0=1/SEL1=1 是 -20dB——BBRF103Radio::UpdateattRF 的 case 映射）
    // v4.182: 从 RX888Intf.ini 读 HfAttenuation（0/10/20 三档换算同老驱动）
    int attIdx = 2;  // 默认 0dB（老会话生产配置）
    {
        char iniPath[MAX_PATH] = "";
        HMODULE hm = nullptr;
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                          GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          (LPCSTR)&UsbOpen, &hm);
        if (hm) GetModuleFileNameA(hm, iniPath, MAX_PATH);
        char* slash = strrchr(iniPath, '\\');
        if (slash) strcpy(slash + 1, "RX888Intf.ini");
        int hfAtt = GetPrivateProfileIntA("RX888", "HfAttenuation", 0, iniPath);
        // 老驱动换算（3 档舍入）：d=(hfAtt+5)/10, attIdx=2-d，夹到 0..2
        int d = (hfAtt + 5) / 10;
        attIdx = 2 - d;
        if (attIdx < 0) attIdx = 0;
        if (attIdx > 2) attIdx = 2;
        UsbLog("UsbOpen: RF(3) HfAttenuation=%d dB -> attIdx=%d", hfAtt, attIdx);
    }
    switch (attIdx) {
    case 0:  // -20dB：SEL0=1, SEL1=1
        GpioSet(ATT_SEL0 | ATT_SEL1);
        break;
    case 1:  // -10dB：SEL0=1, SEL1=0
        GpioSet(ATT_SEL0);
        GpioUnset(ATT_SEL1);
        break;
    case 2:  // 0dB：SEL0=0, SEL1=1
    default:
        GpioSet(ATT_SEL1);
        GpioUnset(ATT_SEL0);
        break;
    }
    UsbLog("UsbOpen: RF(3) att set, gpios=0x%08X", gGpios);

    // (4) DITH off + RAND off（CW 不需要抖动；老会话同此）
    GpioUnset(DITH);
    GpioUnset(RANDO);
    // 保险：SHDWN 清零（前放开）。上电默认即 0，写一次幂等。
    GpioUnset(SHDWN);
    UsbLog("UsbOpen: RF(4) DITH/RAND off, SHDWN cleared, gpios=0x%08X", gGpios);

    // STARTADC：ADC 直采 64M（与 RadioHandler 一致）
    if (!gDev->Control(STARTADC, adcFreq)) {
        UsbLog("UsbOpen: STARTADC(%u) failed", (unsigned)adcFreq);
        delete gDev; gDev = nullptr;   // 析构内 Close()
        return false;
    }
    // STARTFX3：启动 FX3 USB 数据流 producer（与 RadioHandler::Start 的
    // FX3producerOn() 一致）。只有 STARTADC 不启动数据，USB 首包会超时。
    if (!gDev->Control(STARTFX3)) {
        UsbLog("UsbOpen: STARTFX3 failed");
        delete gDev; gDev = nullptr;
        return false;
    }
    UsbLog("UsbOpen: ok, RF chain configured, STARTADC=%u STARTFX3, gpios=0x%08X",
        (unsigned)adcFreq, gGpios);
    *dev = gDev;
    return true;
}

void UsbClose(void** dev) {
    // v4.176: 不 delete gDev——CCyFX3Device 析构在 MinGW 32 位下崩溃（CyAPI
    // 析构器 throw + USB 句柄释放时序问题）。改为仅 Close() 并泄漏对象。
    // StopRx 不频繁，泄漏可接受；根治需重写 CyAPI 析构或换用 WinUSB。
    if (gDev) {
        // gDev->Close() 已在 StopStream/StopRx 流程中间接完成（fx3dev->Close）；
        // 此处不再调用 delete，仅置 nullptr 释放 DLL 侧引用。
        gDev = nullptr;
    }
    *dev = nullptr;
}

bool UsbStartStream(void* dev, void** ring) {
    (void)dev;
    if (!gDev) return false;
    ringbuffer<int16_t>* rb = new ringbuffer<int16_t>(QUEUE_SIZE);
    if (!rb) return false;
    // StartStream 内部 setBlockSize(transferSamples) + 启动 USB 采集线程
    gDev->StartStream(*rb, QUEUE_SIZE);
    *ring = rb;
    UsbLog("UsbStartStream: %d blocks x %d samples", QUEUE_SIZE, transferSamples);
    return true;
}

void UsbStopStream(void* dev, void* ring) {
    UsbLog("UsbStopStream: ring Stop() begin");
    if (ring) static_cast<ringbuffer<int16_t>*>(ring)->Stop();  // 唤醒 getReadPtr 阻塞
    UsbLog("UsbStopStream: gDev StopStream() begin");
    if (gDev) gDev->StopStream();
    UsbLog("UsbStopStream: done");
}

void UsbFreeRing(void** ring) {
    UsbLog("UsbFreeRing: begin");
    if (ring && *ring) {
        auto rb = static_cast<ringbuffer<int16_t>*>(*ring);
        UsbLog("UsbFreeRing: fullCount=%d writeCount=%d", rb->getFullCount(), rb->getWriteCount());
        // v4.176: ringbuffer 析构 delete[] buffers[0] 在 MinGW 32 位下崩溃（疑堆元数据
        // 被 USB DMA 踩踏或 libgomp TLS 干扰）。StopRx 不频繁，4MB 泄漏可接受；
        // 生产中改为延迟到 DllMain DETACH 或下次 StartRx 前释放。
        // TODO: 根治需排查 CyAPI BeginDataXfer/FinishDataXfer 在 Stop 后的 DMA 生命周期。
        *ring = nullptr;   // 释放句柄，内存泄漏（4MB，可接受）
        UsbLog("UsbFreeRing: ring handle released (memory intentionally leaked)");
    }
    UsbLog("UsbFreeRing: done");
}

void* UsbPeekReadPtr(void* ring) {
    if (!ring) return nullptr;
    return (void*)static_cast<ringbuffer<int16_t>*>(ring)->getReadPtr(); // 阻塞取读指针
}

void UsbReadDone(void* ring) {
    if (ring) static_cast<ringbuffer<int16_t>*>(ring)->ReadDone();
}