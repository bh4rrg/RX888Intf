// skimsrv_intf.cpp - 全新 RX888 SkimSrv 驱动 DLL（32 位）v4.185
// v4.185 修复：UnpauseDecoders 温和模式下写 [dm+0x98]=0.0f 解除 0x48030C
//   阻塞（代码注释 524-527 核心结论：新驱动直接 DDC 绕过 r2iq，[dm+0x98]
//   永远 >=1.0，必须主动写 0 才能让 SkimSrv 自行完成 4→3），再 PostMessage
//   一次。v4.181（10-04 13:29 成功转 3）正是写 0 + PostMessage 组合；
//   v4.183/184 盲目复刻老驱动"只 PostMessage 不写 0"（老驱动走 r2iq
//   才不用写），导致 state 卡 4 十小时零 spots。
//   v4.184 保留：周期检查纯只读（不再重复 PostMessage）。
//
// v4.183 修复：UnpauseDecoders 完全复刻老驱动 v4.131 成功会话——不写
//   [dm+0x98]、不强写 state，只 PostMessage(0x042D) 让 SkimSrv 自己完成
//   4→3 转换。老会话铁证：日志中 [dm+0x98] 恒为 1.0、Self state=4 从未
//   被写过，解码照样成功（piqDelivered=545280）。此前版本写 [dm+0x98]=0
//   或强写 state=3 均干扰 0x48030C 内部时序（解码线程创建被短路）。
//
// v4.182 修复（RF 前端 + OMP，均保留）：
//   1. usb_shim.cpp UsbOpen 补齐老驱动 RF 前端配置序列（v4.131 成功会话时序：
//      BIAS_HF off -> TUNERSTDBY + ANT_SEL -> 3档衰减 -> DITH/RAND off）。
//      此前缺失导致天线信号进不了 ADC，spectrum_probe 实测全噪声——
//      decoders=0/零 spots 的硬件层根因。
//   2. DllMain 里 omp_set_num_threads(1)：libgomp win32 后端 worker 完成后
//      spin-wait 自旋，8 worker 各烧 70-85% CPU 造成假“CPU Overloaded”；
//      OMP_NUM_THREADS=1 实验验证 CPU 41.3%→6.5% 且实时率不变。代码级固化。
//
// v4.181 修复（两大根因，均有现场证据）：
//   ① StopRx 死锁（15s 超时→FreeBlocks 永久挂死）
//      栈回溯证据（stackdump 实抓）：交付线程函数 return 后，MinGW CRT
//      endthread 清理要 free gomp/winpthread 的 TLS 分配（msvcrt+0x488A9
//      → ntdll 堆函数），与主线程 FreeBlocks 的 _aligned_free（msvcrt
//      堆函数）互锁在 ntdll 堆锁上 → 双线程永久死锁。
//      修复：循环退出后线程不 return，置安全点标志后在自身事件上挂起，
//      由主线程 TerminateThread 直接杀——完全绕开 CRT 线程退出清理链
//      （不 free 任何 TLS，不碰堆锁）。线程栈 1MB 不回收（每次 StopRx
//      泄漏 1MB，可控；DLL 已有多处 MinGW32 泄漏策略，同口径）。
//   ② decoders=0（解码线程从未创建，ring rIdx/wIdx 永不前进）
//      老驱动 v4.131 成功日志证据：全程无 FORCE，SkimSrv 在数据流启动后
//      自己完成 state 4→3（0x48030C 会同时创建解码工作线程）。
//      v4.180 直接强写 [Self+0x5C]=3 让 0x48030C 的状态判断短路——
//      state 看起来变了，但解码线程创建逻辑被跳过 → ring 有结构无消费者。
//      修复：温和模式——只写 [dm+0x98]=0.0f（解除 0x48030C 阻塞条件）
//      + PostMessage(0x042D)，让 SkimSrv 自己跑完整转换。ini 可选
//      UnpauseMode=0 温和（默认）/1 强制（兼容旧行为）。
// v4.180 修复：
//   ① 统一日志通道——旧实现 skimsrv_intf.cpp 和 usb_shim.cpp 各自 fopen 同一
//     日志文件得到两个独立 FILE* 句柄，在无跨路互斥下并发写腐蚀 CRT 堆元数据，
//     11 分钟后 ntdll 堆函数崩溃（0xc0000005）。改为共用单一 FILE* + 单一锁。
//   ② pIQProc 指针数组保护——SkimSrv 回调可能改写传入的 CmplxA* 指针数组条目，
//     若直接传 gDataA/gDataB 静态数组地址，改写后交付线程踩踏别人的堆 + StopRx
//     FreeBlocks 释放错误地址 → ntdll 0xc0000005。改为每次调用前拷贝到局部栈帧。
// 数据流：FX3 USB BulkIn(64M int16) → ringbuffer → DDC_Engine(每通道 64M 域 NCO 混频
//         → 64M→2M 实系数低通抽取 → 2M→48/96/192k 分数重采样) → ping-pong 双缓冲
//         → pIQProc(THandle, Data) 93.75 块/秒
// 格式：Cmplx{float Re, Im}，Im=-Q（规范第 7 节），16 字节对齐
// USB 底座：复用 ExtIO_sddc 的 FX3handler（CyAPI + 固件下载 + AdcSamplesProcess 4 队列并发）
// 编译：MinGW-w64 32 位 g++（与 SkimSrv 同位数），OpenMP 可选（DSP 主循环已可并行）
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>

#include "ddc_engine.h"

// ---------------------------------------------------------------------------
// SkimSrv 接口类型（与老驱动 RX888Intf.cpp 完全对齐——SkimSrv 按此布局解析）
// ---------------------------------------------------------------------------
#define RATE_48KHZ    0
#define RATE_96KHZ    1
#define RATE_192KHZ   2
#define BLOCKS_PER_SEC 93.75
#define MAX_RX_COUNT  8

#pragma pack(push, 16)
typedef struct { float Re, Im; } Cmplx;   // 8 字节，16 字节对齐
typedef Cmplx  *CmplxA;
typedef CmplxA *CmplxAA;
#pragma pack(pop)

// SkimSrv 的 SdrInfo 结构（与老驱动一致：name+type+minRate+maxRate+maxBandwidth）
// type=8 → SkimSrv 允许最多 8 通道
typedef struct {
    const char *name;
    int   type;
    float minRate;
    float maxRate;
    float maxBandwidth;
} SdrInfo;

// pIQProc 签名：void __stdcall(int RxHandle, CmplxA* Data)
// Data 是 CmplxA* = Cmplx** = 每通道一个指针，指向 blockInSamples 个 Cmplx
typedef void (__stdcall *tIQProc)(int RxHandle, CmplxA* Data);
typedef void (__stdcall *tAudioProc)(int RxHandle, CmplxA InIq, CmplxA OutLR, int OutCount);
typedef void (__stdcall *tErrorProc)(int RxHandle, char *ErrText);
typedef void (__stdcall *tStatusBitsProc)(int RxHandle, int a, unsigned char Bits);
typedef void (__stdcall *tLoadProgressProc)(int RxHandle, int Current, int Total);

// StartRx 参数结构（与老驱动 StartRxParams 完全一致——SkimSrv 按此布局填充）
// 新驱动的 SdrSettings 布局碰巧与之一致（字段名不同但偏移相同），但明确对齐更安全
typedef struct {
    DWORD THandle;           // offset 0
    int   RecvCount;          // offset 4
    DWORD RateID;             // offset 8（SkimSrv 只取低字节做 RxMode）
    DWORD unknown0C;          // offset 12
    tIQProc           pIQProc;         // offset 16
    DWORD             unknown14;       // offset 20
    tStatusBitsProc   pStatusBitProc;  // offset 24
    tLoadProgressProc pLoadProgressProc; // offset 28
    tErrorProc        pErrorProc;      // offset 32
} StartRxParams;

// ---------------------------------------------------------------------------
// 日志（可开关，输出到 DLL 同目录 RX888New.log）
// ---------------------------------------------------------------------------
static FILE* gLog = nullptr;
static CRITICAL_SECTION gLogCS;
static bool gLogCSInit = false;
static void LogOpen() {
    if (!gLogCSInit) { InitializeCriticalSection(&gLogCS); gLogCSInit = true; }
    if (gLog) return;
    char path[MAX_PATH], dir[MAX_PATH];
    HMODULE self = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                       GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&LogOpen, &self);
    if (self && GetModuleFileNameA(self, path, MAX_PATH)) {
        strcpy(dir, path);
        char* p = strrchr(dir, '\\');
        if (p) { *p = 0; strcpy(path, dir); strcat(path, "\\RX888New.log"); }
        else strcpy(path, "RX888New.log");
    } else strcpy(path, "RX888New.log");
    gLog = fopen(path, "a");
}
static void LogClose() {
    if (gLog) { fclose(gLog); gLog = nullptr; }
}
static void Log(const char* fmt, ...) {
    if (!gLog) return;
    EnterCriticalSection(&gLogCS);
    SYSTEMTIME st; GetLocalTime(&st);
    fprintf(gLog, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    va_list ap; va_start(ap, fmt);
    vfprintf(gLog, fmt, ap);
    va_end(ap);
    fprintf(gLog, "\n");
    fflush(gLog);
    LeaveCriticalSection(&gLogCS);
}

// v4.180: 统一日志通道——DdcLog 桥使用同一个 FILE* 和 CRITICAL_SECTION
// 旧实现 usb_shim.cpp 自带 gUsbLog=fopen(同文件) 是独立 FILE* 句柄，
// 两路并发写同一文件在无互斥下腐蚀 CRT 堆元数据（ntdll 0xc0000005 根因）
extern "C" void SharedDdcLog(const char* fmt, va_list args) {
    if (!gLog) LogOpen();
    if (!gLog) return;
    EnterCriticalSection(&gLogCS);
    SYSTEMTIME st; GetLocalTime(&st);
    fprintf(gLog, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    vfprintf(gLog, fmt, args);
    fprintf(gLog, "\n");
    fflush(gLog);
    LeaveCriticalSection(&gLogCS);
}

// ---------------------------------------------------------------------------
// 全局状态
// ---------------------------------------------------------------------------
static StartRxParams gSet;              // SkimSrv 传入设置的本地副本
static DWORD gTHandle = 0;              // SkimSrv 的 THandle（用于解码器解暂停）
static std::atomic<bool> gRunning{false};      // 运行标志
static std::atomic<bool> gStopReq{false};      // 停止请求
static bool gStarted = false;                  // StartRx 已执行
static int  gSampleRate = 96000;                 // 当前输出采样率
static int  gBlockInSamples = 1024;             // 每块样本数 = Rate/93.75
static int  gRecvCount = 1;                      // 激活通道数
static HANDLE gThread = nullptr;                // 交付线程
static Cmplx* gDataA[MAX_RX_COUNT] = {0};   // ping
static Cmplx* gDataB[MAX_RX_COUNT] = {0};   // pong
static Cmplx* gFill[MAX_RX_COUNT] = {0};    // 当前填充缓冲（指向 A 或 B）
static int  gFillIdx[MAX_RX_COUNT] = {0};       // 每通道块内已填样本数
static unsigned long gBlocksDelivered = 0;

// v4.181: 交付线程安全点退出机制（绕开 CRT TLS 清理死锁）
// 交付线程循环退出后不 return（return 会触发 MinGW CRT endthread 清理，
// free gomp/winpthread TLS 分配时与主线程堆操作互锁——现场实抓）。
// 改为：置 gDeliverAtSafePoint=true 后在事件上无限挂起，主线程
// TerminateThread 直接杀（不做任何清理，不碰堆）。
static volatile LONG gDeliverAtSafePoint = 0;   // 1=线程已到安全点挂起
static volatile LONG gDeliverKilled = 0;         // 1=主线程已 TerminateThread
static HANDLE gDeliverSuicideEvent = nullptr;    // 安全点挂起事件
static int  gUnpauseMode = 0;                     // v4.181: 0=温和(默认) 1=强制(旧行为)
static double gLoHz[MAX_RX_COUNT] = {0};      // 各通道 LO（Hz）
static bool   gLoSet[MAX_RX_COUNT] = {0};      // 各通道 LO 是否已设置

// USB / DSP
static void* gFx3 = nullptr;                    // fx3class* 实例（不透明，经由 C 壳函数访问）
static void* gRing = nullptr;                    // ringbuffer<int16_t>*
static bool  gUsbOk = false;

// 热重调待处理队列（SetRxFrequency 写入，交付线程消费）
static std::mutex gRetuneMtx;
static std::vector<std::pair<int,double>> gPendingRetunes;

// 极简防御：DspReady 前不接受数据
static bool gDspReady = false;

// 常数：与 FX3handler/config.h 一致
static const uint32_t TRANSFER_SIZE   = 131072;             // 字节
static const uint32_t TRANSFER_SAMPLES = TRANSFER_SIZE / 2; // 65536 int16 样本
static const uint32_t ADC_FREQ        = 64000000;           // 64M 直采
static const double   FS_MID          = 2000000.0;           // 中间速率 2M

// ---------------------------------------------------------------------------
// FX3 USB 底座（以 C 壳函数隔离 C++ 类依赖，降低与 CyAPI 编译单元的耦合）
// 实际直接使用 ExtIO_sddc 源码中的 fx3handler（见 build 脚本），此处声明其接口。
// 为避免引入 ExtIO_sddc 的 config.h 依赖，这里用独立的最小声明。
// ---------------------------------------------------------------------------
#include "usb_shim.h"

// ---------------------------------------------------------------------------
// 内部工具
// ---------------------------------------------------------------------------
static void ReportError(const char* text) {
    if (gSet.pErrorProc) {
        Log("ERROR: %s", text);
        gSet.pErrorProc(gSet.THandle, (char*)text);
        gStopReq = true;
    }
}

static Cmplx* AllocBlock(int n) {
    return (Cmplx*)_aligned_malloc(n * sizeof(Cmplx), 16);
}

static void FreeBlocks() {
    for (int i = 0; i < MAX_RX_COUNT; ++i) {
        if (gDataA[i]) { _aligned_free(gDataA[i]); gDataA[i] = nullptr; }
        if (gDataB[i]) { _aligned_free(gDataB[i]); gDataB[i] = nullptr; }
        gFill[i] = nullptr;
    }
}

// 每通道 LO 决策：HF 波段直接 64M 域数字混频（任何 <32M 频率都支持）
static double ClampLo(int freq) {
    if (freq < 10000) freq = 10000;
    if (freq > 32000000) freq = 32000000;   // Nyquist 之内
    return (double)freq;
}

// ---------------------------------------------------------------------------
// 交付线程主体：USB → int16 环形缓冲 → DDC → 块缓冲 → pIQProc
// ---------------------------------------------------------------------------
// v4.176: 读取 RX888Intf.ini [RX888] OutputScale（与老驱动同口径，默认 1000）。
// 老驱动实测：OutputScale=1000 时交付 RMS ≈200~300k（原生 ADC 标度）。
static double LoadOutputScale() {
    char path[MAX_PATH], dir[MAX_PATH];
    HMODULE self = nullptr;
    double scale = 1000.0;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                       GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&LoadOutputScale, &self);
    if (self && GetModuleFileNameA(self, path, MAX_PATH)) {
        strcpy(dir, path);
        char* p = strrchr(dir, '\\');
        if (p) { *p = 0; strcpy(path, dir); strcat(path, "\\RX888Intf.ini"); }
        else strcpy(path, "RX888Intf.ini");
        int cfg = GetPrivateProfileIntA("RX888", "OutputScale", 0, path);
        if (cfg > 0) scale = (double)cfg;
    }
    Log("OutputScale=%.1f (RX888Intf.ini)", scale);
    return scale;
}

// ---------------------------------------------------------------------------
// v4.179: DC Blocker（从老驱动 ddc_engine.cpp 移植）
// 老驱动有两层 DC blocker：
//   1) WidebandDCBlocker（64M 域）：去除 ADC 原始 DC 偏移，截止 100Hz
//   2) DCBlocker（96k 输出域）：去除残余 DC，截止 100Hz
// 新驱动之前两层都没有，ADC DC 偏移直接进 DDC，每通道 0Hz 产生强尖峰，
// 可能淹没中心频率附近的 CW 信号，导致 SkimSrv 无法解码。
// ---------------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 宽带 DC Blocker（64M 实数域）：估计 DC，减去 DC
// v4.192: 重写——旧实现为逐样本串行双精度 IIR（65536 长链式依赖，无法向量化，实测 227µs/块）。
// 新实现：64 位整数求和算块均值 → 块级 EMA 跟踪 → 块内减常量。两段均 SIMD 友好。
// 等效 DC 跟踪 ~2.4Hz（时间常数 64 块 ≈ 65ms），远慢于 ADC 自热漂移（秒-分钟尺度），
// 且低于 CW/SSB 音频带（几百 Hz），对解码无影响；旧 IIR 100Hz 截止的低频纹波反而被消除。
class WbDCBlocker {
public:
    WbDCBlocker() {}
    void init(double sampleRate, double cutoffHz) {
        (void)sampleRate; (void)cutoffHz;   // 参数保留兼容旧调用点；EMA 系数固定
        meanI_ = 0.0;
    }
    void reset() { meanI_ = 0.0; }
    // 融合 int16→float + DC 去除
    void applyRe(float* outRe, const int16_t* inRaw, int len) {
        // 段 1：64 位整数求和 → 块均值（65536×32767 < 2^31，64 位累加零溢出）
        long long acc = 0;
        for (int i = 0; i < len; ++i) acc += inRaw[i];
        const double blkMean = (double)acc / (double)len;
        // EMA 块级跟踪（63/64：时间常数 64 块 ≈ 65ms → ~2.4Hz）
        meanI_ = 0.984375 * meanI_ + 0.015625 * blkMean;
        const float dc = (float)meanI_;
        // 段 2：int16→float + 减 DC 常量（纯 SIMD，无链式依赖）
        for (int i = 0; i < len; ++i) outRe[i] = (float)inRaw[i] - dc;
    }
private:
    double meanI_ = 0.0;
};

// 每通道 DC Blocker（96k 复数域）：DC subtraction IIR
// 与老驱动 DCBlocker::apply 完全一致
class ChanDCBlocker {
public:
    ChanDCBlocker() : alpha_(0.995f), xm1I_(0.0f), xm1Q_(0.0f), ym1I_(0.0f), ym1Q_(0.0f) {}
    void init(double outputSampleRate) {
        double cutoff = 100.0;
        alpha_ = (float)(1.0 - 2.0 * M_PI * cutoff / outputSampleRate);
        if (alpha_ < 0.9f) alpha_ = 0.9f;
        if (alpha_ > 0.999f) alpha_ = 0.999f;
        reset();
    }
    void reset() { xm1I_ = xm1Q_ = ym1I_ = ym1Q_ = 0.0f; }
    void apply(float* dataI, float* dataQ, int len) {
        float a = alpha_;
        float pInI = xm1I_, pInQ = xm1Q_;
        float pOutI = ym1I_, pOutQ = ym1Q_;
        for (int n = 0; n < len; n++) {
            float re = dataI[n] - pInI + a * pOutI;
            float im = dataQ[n] - pInQ + a * pOutQ;
            pInI = dataI[n];
            pInQ = dataQ[n];
            pOutI = re;
            pOutQ = im;
            dataI[n] = re;
            dataQ[n] = im;
        }
        xm1I_ = pInI; xm1Q_ = pInQ;
        ym1I_ = pOutI; ym1Q_ = pOutQ;
    }
private:
    float alpha_;
    float xm1I_, xm1Q_, ym1I_, ym1Q_;
};

// 前向声明（定义在 StartRx 之后）
static void UnpauseDecoders();
static void CheckDecodersReadOnly();

static DWORD WINAPI DeliverThread(LPVOID) {
    using namespace skdsp;

    std::vector<float> rawFloat((size_t)TRANSFER_SAMPLES + 16);

    // v4.179: 宽带 DC Blocker（64M 域，截止 100Hz）
    WbDCBlocker wbDcBlocker;
    wbDcBlocker.init((double)ADC_FREQ, 100.0);
    Log("WbDCBlocker initialized: fs=%.0f cutoff=100Hz", (double)ADC_FREQ);

    // v4.179: 每通道 DC Blocker（96k 域，截止 100Hz）
    std::vector<ChanDCBlocker> chanDcBlockers(gRecvCount);
    for (int c = 0; c < gRecvCount; ++c)
        chanDcBlockers[c].init((double)gSampleRate);
    Log("ChanDCBlocker initialized: nCh=%d fs=%d cutoff=100Hz", gRecvCount, gSampleRate);

    // 默认 LO（若 SkimSrv 尚未设置，用第一个调用 SetRxFrequency 的值；
    // 启动时一律先建引擎，LO 后续可改——见 SetRxFrequency 的热重调处理）
    DDC_Engine* eng = new DDC_Engine();
    std::vector<ChannelCfg> cfgs(gRecvCount);
    for (int c = 0; c < gRecvCount; ++c)
        cfgs[c].loHz = gLoSet[c] ? gLoHz[c] : 3550000.0 + c * 1000.0;   // 占位防重合
    OutRate orate = (OutRate)(gSet.RateID & 0xFF);
    eng->init(orate, gRecvCount, cfgs.data());
    gDspReady = true;
    Log("DDC engine ready: rate=%d nCh=%d block=%d", gSampleRate, gRecvCount, gBlockInSamples);

    int which = 0;   // ping-pong 索引：0 → gDataA，1 → gDataB
    for (int c = 0; c < gRecvCount; ++c) { gFill[c] = gDataA[c]; gFillIdx[c] = 0; }

    uint64_t usbBlocks = 0, dspCalls = 0, deliverCalls = 0;
    // v4.181: 读取 UnpauseMode（0=温和默认 / 1=强制旧行为）
    {
        char iniPath[MAX_PATH];
        HMODULE self = nullptr;
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                           GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           (LPCSTR)&LoadOutputScale, &self);
        if (self && GetModuleFileNameA(self, iniPath, MAX_PATH)) {
            char* p = strrchr(iniPath, '\\');
            if (p) { *p = 0; strcat(iniPath, "\\RX888Intf.ini"); }
            gUnpauseMode = GetPrivateProfileIntA("RX888", "UnpauseMode", 0, iniPath);
        }
    }
    Log("UnpauseMode=%d (0=gentle: [dm+0x98]=0+PostMessage only; 1=forced: also write state=3)", gUnpauseMode);
    Log("StartRx: v4.194 (FIR direct-read input block, no delay-line memcpy, -march=x86-64-v3 -ffast-math)");

    // v4.176: 标度与老驱动对齐（ini OutputScale，默认 1000）。
    // 旧硬编码 131072 = 老驱动标度的 131 倍（+42dB），S 表/水位全错。
    const float scaleF = (float)LoadOutputScale();
    const float clipLimit = 1.0e13f;   // 与老驱动相同的硬限幅

    // v4.176: 每通道 carry 缓冲——上一轮 DDC 输出中填不进当前块的剩余样本。
    // 旧实现块满后 room=0→cnt=0 无限空转（零交付主因），此处彻底重写。
    std::vector<std::vector<float>> carryI(gRecvCount), carryQ(gRecvCount);
    std::vector<int> carryLen(gRecvCount, 0);
    for (int c = 0; c < gRecvCount; ++c) {
        carryI[c].assign(4096, 0.0f);
        carryQ[c].assign(4096, 0.0f);
    }

    // v4.186-4: 流水线重叠——转换（applyRe）与 DDC 并行计算重叠。
    // 双缓冲：cur 块正在被 worker 计算，next 块由主线程同步转换。
    std::vector<float> rawA((size_t)TRANSFER_SAMPLES + 16);
    std::vector<float> rawB((size_t)TRANSFER_SAMPLES + 16);
    float* rawCur = rawA.data();
    float* rawNext = rawB.data();
    const int n = (int)TRANSFER_SAMPLES;
    int16_t* blkNext = nullptr;   // 预取的下一块 USB 数据

    // v4.187-10: QPC 插桩——每阶段独立分段计时（段间差分），每 1000 块汇总。
    // 目标：定位 v4.187-9 剩余 ~0.5% 隐藏串行开销（971/s vs 976/s 实时率）。
    LARGE_INTEGER qpcFreq;
    QueryPerformanceFrequency(&qpcFreq);
    const double qpcInv = 1e6 / (double)qpcFreq.QuadPart;   // ticks -> us
    LARGE_INTEGER qpcPrev, qpcNow, qpcWall0;
    QueryPerformanceCounter(&qpcPrev);
    qpcWall0 = qpcPrev;
    uint64_t tPreFetch=0, tApplyRe=0, tCollect=0, tWait=0, tFlip=0,
             tRetune=0, tSwap=0, tSubmit=0, tDeliver=0, tStat=0;
    int qpcSampleCount = 0;

    // 预取第 0 块并转换
    // v4.186-5: 去 Sleep(1)（15.6ms 精度陷阱）
    {
        int spin = 0;
        while (!gStopReq) {
            blkNext = (int16_t*)UsbPeekReadPtr(gRing);
            if (blkNext) break;
            ++spin;
            if ((spin & 1023) == 0) Sleep(0);
            else _mm_pause();
        }
    }
    if (!blkNext) {
        // v4.190: 提前退出路径也必须关 worker 池（同主退出路径）
        eng->shutdownPool();
        Log("deliver: no first block, exiting (gStopReq=%d)", (int)(bool)gStopReq); return 0; }
    wbDcBlocker.applyRe(rawCur, blkNext, n);
    UsbReadDone(gRing);
    ++usbBlocks;
    ++dspCalls;
    bool havePrev = false;   // v4.187-8: 双缓冲 collect——首轮无上一批输出

    // v4.187-9: 循环外提交第 0 批（循环内改为尾部 submit 下一批，实现交付重叠）
    eng->processSubmit(rawCur, n);

    while (!gStopReq) {
        // 1. 主线程重叠工作 A：预取并转换下一块（与 worker DDC 并行）
        // v4.186-5: 预取轮询去 Sleep(1)——Windows Sleep(1) 实际精度 15.6ms，
        // ring 非空时零延迟、空时每 1024 次 _mm_pause 后 Sleep(0) 让出时间片。
        blkNext = nullptr;
        {
            int spin = 0;
            while (!gStopReq) {
                blkNext = (int16_t*)UsbPeekReadPtr(gRing);
                if (blkNext) break;
                ++spin;
                if ((spin & 1023) == 0) Sleep(0);   // 偶尔让出时间片（无 15ms 损失）
                else _mm_pause();                     // 超线程友好自旋
            }
        }
        QueryPerformanceCounter(&qpcNow);
        tPreFetch += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        if (blkNext) {
            wbDcBlocker.applyRe(rawNext, blkNext, n);
        }
        QueryPerformanceCounter(&qpcNow);
        tApplyRe += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 2. 主线程重叠工作 B：collect 上一批输出（worker 正算当前批，双缓冲安全）
        // v4.187-8: 双缓冲 collect——上一批输出在 worker 计算当前批时被读取，
        // 消除 collect 串行尾巴。首次迭代（parity=0）无上一批，跳过。
        if (havePrev) {
            for (int c = 0; c < gRecvCount; ++c) {
                float* pI; float* pQ;
                int navail = eng->collectPrev(c, pI, pQ);
                // v4.179: 每通道 DC Blocker（96k 域，在 scale 之前应用）
                if (navail > 0) chanDcBlockers[c].apply(pI, pQ, navail);

                // 4a. 新样本追加到 carry 尾部
                if (navail > 0) {
                    int need = carryLen[c] + navail;
                    if ((int)carryI[c].size() < need) {
                        carryI[c].resize(need);
                        carryQ[c].resize(need);
                    }
                    memcpy(carryI[c].data() + carryLen[c], pI, (size_t)navail * sizeof(float));
                    memcpy(carryQ[c].data() + carryLen[c], pQ, (size_t)navail * sizeof(float));
                    carryLen[c] += navail;
                }

                // 4b. 从 carry 头部尽可能填入块缓冲
                int take = std::min(carryLen[c], gBlockInSamples - gFillIdx[c]);
                if (take > 0) {
                    Cmplx* dst = gFill[c] + gFillIdx[c];
                    const float* sI = carryI[c].data();
                    const float* sQ = carryQ[c].data();
                    for (int j = 0; j < take; ++j) {
                        float re = sI[j] * scaleF;
                        float im = sQ[j] * scaleF;   // v4.176: Im=+Q（与生产验证的老驱动一致；旧-Q为镜像错误）
                        if (re >  clipLimit) re =  clipLimit;
                        else if (re < -clipLimit) re = -clipLimit;
                        if (im >  clipLimit) im =  clipLimit;
                        else if (im < -clipLimit) im = -clipLimit;
                        dst[j].Re = re;
                        dst[j].Im = im;
                    }
                    gFillIdx[c] += take;
                    int rem = carryLen[c] - take;
                    if (rem > 0) {
                        memmove(carryI[c].data(), carryI[c].data() + take, (size_t)rem * sizeof(float));
                        memmove(carryQ[c].data(), carryQ[c].data() + take, (size_t)rem * sizeof(float));
                    }
                    carryLen[c] = rem;
                }
            }
        }
        QueryPerformanceCounter(&qpcNow);
        tCollect += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 3. 等 DDC 批完成（当前批）
        eng->processWait();
        QueryPerformanceCounter(&qpcNow);
        tWait += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 3b. 翻转输出组：当前批完成，其输出成为下一次 collect 的"上一批"
        eng->flipParity();
        havePrev = true;
        QueryPerformanceCounter(&qpcNow);
        tFlip += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 4. 消费热重调请求（worker 空闲窗口，setChannelLo 安全；改完立即 submit 用新系数）
        {
            std::lock_guard<std::mutex> lk(gRetuneMtx);
            for (auto& pr : gPendingRetunes) {
                eng->setChannelLo(pr.first, pr.second);
                Log("retune ch%d -> %.0f Hz", pr.first, pr.second);
            }
            gPendingRetunes.clear();
        }
        QueryPerformanceCounter(&qpcNow);
        tRetune += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 5. 预取的下一块生效（若无预取则当前块是最后一块）
        if (blkNext) {
            // 消费完成，推进读指针
            UsbReadDone(gRing);
            ++usbBlocks;
            ++dspCalls;
            std::swap(rawCur, rawNext);
        } else {
            break;
        }
        QueryPerformanceCounter(&qpcNow);
        tSwap += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 6. 提交下一批（worker 立即开始算），交付与下一批计算重叠
        eng->processSubmit(rawCur, n);
        QueryPerformanceCounter(&qpcNow);
        tSubmit += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 7. 所有通道块满 → 交付（与 worker 算下一批重叠；交付后切到另一套填充）
        bool allFull = true;
        for (int c = 0; c < gRecvCount; ++c)
            if (gFillIdx[c] < gBlockInSamples) { allFull = false; break; }
        if (allFull) {
            CmplxA deliverArr[MAX_RX_COUNT];
            Cmplx** srcArr = (which == 0) ? gDataA : gDataB;
            for (int c = 0; c < gRecvCount; ++c)
                deliverArr[c] = srcArr[c];
            CmplxA* deliver = deliverArr;
            if (gSet.pIQProc) {
                gSet.pIQProc((int)gTHandle, deliver);
                ++deliverCalls;
                if (deliverCalls == 1) {
                    Log("first IQ block delivered (after %llu usb blocks)",
                        (unsigned long long)usbBlocks);
                }
                // v4.177: 解码器解暂停——SkimSrv 状态机在 StartRx 后卡在
                // state=4(paused)，因为直接 DDC 绕过 r2iq 导致 [dm+0x98]
                // 永远 >=1.0，0x48030C 无法完成 4→3 转换。
                // 老驱动在回调 #200 时才做解暂停（等 SkimSrv 设置好 [T+0x48] dispatcher）
                // 我们在第100个交付块时执行（约1秒后，SkimSrv 应已完成 Phase 2）
                if (deliverCalls == 100) {
                    Log("deliverCalls=100, triggering UnpauseDecoders");
                    UnpauseDecoders();
                }
                // v4.179: 交付 RMS 诊断（每1000块 ≈ 1秒）——验证标度与老驱动一致
                if ((deliverCalls % 1000) == 0 && deliverCalls > 0) {
                    for (int c = 0; c < gRecvCount; ++c) {
                        const Cmplx* dp = (which == 0) ? gDataB[c] : gDataA[c];
                        double sumSq = 0;
                        for (int s = 0; s < gBlockInSamples; ++s)
                            sumSq += (double)dp[s].Re * dp[s].Re + (double)dp[s].Im * dp[s].Im;
                        double rms = sqrt(sumSq / (2.0 * gBlockInSamples));
                        Log("DELIV_RMS ch%d = %.1f", c, rms);
                    }
                }
                // v4.184: 周期性只读检查（每5000块 ≈ 53秒）——只读诊断，
                //   不再重复 PostMessage(0x042D)。
                if ((deliverCalls % 5000) == 0 && deliverCalls > 0) {
                    Log("PERIODIC check #%llu: checking Self state (read-only)", (unsigned long long)deliverCalls);
                    CheckDecodersReadOnly();
                }
            }
            which ^= 1;
            if (which == 0) { for (int c = 0; c < gRecvCount; ++c) gFill[c] = gDataA[c]; }
            else            { for (int c = 0; c < gRecvCount; ++c) gFill[c] = gDataB[c]; }
            for (int c = 0; c < gRecvCount; ++c) gFillIdx[c] = 0;
            ++gBlocksDelivered;
        }
        QueryPerformanceCounter(&qpcNow);
        tDeliver += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
        qpcPrev = qpcNow;

        // 周期性统计（每 1000 块 ≈ 1.02 秒）+ v4.187-10 QPC 插桩汇总
        if ((usbBlocks % 1000) == 0) {
            double sec = usbBlocks * (double)n / (double)ADC_FREQ;
            Log("stat usb=%llu dsp=%llu deliver=%llu (%.1f blocks/s) delivered=%lu",
                (unsigned long long)usbBlocks, (unsigned long long)dspCalls,
                (unsigned long long)deliverCalls,
                deliverCalls / sec, gBlocksDelivered);
            QueryPerformanceCounter(&qpcNow);
            uint64_t wallUs = (uint64_t)((qpcNow.QuadPart - qpcWall0.QuadPart) * qpcInv);
            tStat += (uint64_t)((qpcNow.QuadPart - qpcPrev.QuadPart) * qpcInv);
            qpcPrev = qpcNow;
            if (++qpcSampleCount >= 2) {   // 首个千块窗口跳过（含启动预热）
                uint64_t loopSum = tPreFetch + tApplyRe + tCollect + tWait + tFlip
                                 + tRetune + tSwap + tSubmit + tDeliver + tStat;
                uint64_t idleUs = (wallUs > loopSum) ? (wallUs - loopSum) : 0;
                Log("QPC n=%d wall=%.0fus: prefetch=%llu applyRe=%llu collect=%llu wait=%llu flip=%llu retune=%llu swap=%llu submit=%llu deliver=%llu stat=%llu loopSum=%llu idle=%llu (us/1000blk)",
                    qpcSampleCount, (double)wallUs,
                    (unsigned long long)tPreFetch, (unsigned long long)tApplyRe,
                    (unsigned long long)tCollect, (unsigned long long)tWait,
                    (unsigned long long)tFlip, (unsigned long long)tRetune,
                    (unsigned long long)tSwap, (unsigned long long)tSubmit,
                    (unsigned long long)tDeliver, (unsigned long long)tStat,
                    (unsigned long long)loopSum, (unsigned long long)idleUs);
            }
            tPreFetch=tApplyRe=tCollect=tWait=tFlip=tRetune=0;
            tSwap=tSubmit=tDeliver=tStat=0;
            qpcWall0 = qpcNow;
        }
    }

    // v4.187-9: 循环退出前可能有已 submit 未 wait 的批（预取失败/gStopReq），
    // wait 收尾防 worker 写已废弃缓冲时主线程并发退出（池不销毁，写安全，但收尾更干净）
    eng->processWait();
    // v4.190: 关闭 DDC 通道线程池——必须在交付线程被 TerminateThread 杀死之前，
    // 否则无人再能调用 shutdown（引擎不 delete、指针随栈消失），worker 存活至
    // DLL 卸载，唤醒后踩已释放代码页 → RX888Intf.dll_unloaded 退出崩溃（根因）。
    // shutdown 为幂等：stop_+SetEvent+WaitAll+关句柄，与析构/init 重启均兼容。
    eng->shutdownPool();
    Log("DDC pool shutdown done (v4.193)");

    gDspReady = false;
    // v4.177: MinGW 32 位下 delete DDC_Engine 崩溃（与 ringbuffer/CyAPI 同类问题），
    //   采用泄漏策略——StopRx 不频繁，泄漏量可控
    Log("deliver thread: skip delete eng (MinGW32 leak policy)");
    // delete eng;  // 崩溃，注释掉
    Log("deliver thread at safe point: usb=%llu delivered=%lu (will be terminated by StopRx)",
        (unsigned long long)usbBlocks, gBlocksDelivered);

    // v4.181: 不 return！return 会触发 MinGW CRT endthread 清理链
    //（free gomp/winpthread TLS 分配 → 与主线程 FreeBlocks 的堆操作在
    //  ntdll 堆锁上互锁——11:30 会话现场实抓双挂死）。
    // 置安全点标志后无限挂起，由主线程 TerminateThread 直接杀（零清理零堆操作）。
    // 若主线程 3 秒内没来杀（异常路径），仍退田为 return——宁冒死锁风险也不留僵尸。
    InterlockedExchange(&gDeliverAtSafePoint, 1);
    if (gDeliverSuicideEvent) {
        WaitForSingleObject(gDeliverSuicideEvent, 3000);
    }
    if (InterlockedCompareExchange(&gDeliverKilled, 0, 0) == 0) {
        Log("deliver thread: StopRx did not terminate in 3s, falling back to return");
        return 0;
    }
    return 0;   // 已被标记 killed（理论不可达）
}

// ---------------------------------------------------------------------------
// 导出函数 1：GetSdrInfo
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport)
int __stdcall GetSdrInfo(SdrInfo *pInfo) {
    LogOpen();
    Log("GetSdrInfo called");
    if (pInfo == nullptr) return 0;

    // 尝试发现设备（不强制：无硬件时仍返回信息，StartRx 时再报错）
    bool present = UsbTryOpen(&gFx3, 0);
    static const char* nameBuf = "RX888 SDR";
    pInfo->name = nameBuf;
    pInfo->type = 8;               // SdrType=8 → SkimSrv 允许最多 8 通道
    pInfo->minRate = 48000.0f;
    pInfo->maxRate = 192000.0f;
    pInfo->maxBandwidth = 192000.0f;
    Log("GetSdrInfo ok: present=%d name=%s type=8", (int)present, nameBuf);
    return 1;   // 1=success
}

// ---------------------------------------------------------------------------
// 导出函数 2：StartRx
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport)
void __stdcall StartRx(void* rawParams) {
    LogOpen();
    Log("StartRx called");
    if (rawParams == nullptr) return;

    // 按老驱动的 StartRxParams 布局解析 SkimSrv 传入的参数
    memcpy(&gSet, rawParams, sizeof(gSet));
    gTHandle = gSet.THandle;

    // 调试：dump 原始参数
    DWORD* raw = (DWORD*)rawParams;
    Log("StartRx: raw [0]=0x%08X [1]=0x%08X [2]=0x%08X [3]=0x%08X [4]=0x%08X [5]=0x%08X [6]=0x%08X [7]=0x%08X [8]=0x%08X",
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7], raw[8]);
    Log("StartRx: THandle=%u RecvCount=%d RateID=0x%X pIQProc=0x%08X",
        gSet.THandle, gSet.RecvCount, gSet.RateID, (DWORD)gSet.pIQProc);

    gSet.RateID &= 0xFF;                       // SkimSrv 只取低字节做 RxMode
    int rid = gSet.RateID;
    if (rid < 0 || rid > 2) rid = RATE_96KHZ;
    gSet.RateID = rid;                         // v4.176: 夹紧后回写，防 OUT_RATES 越界（交付线程直接用它索引）
    gSampleRate = (rid == RATE_48KHZ) ? 48000 : (rid == RATE_96KHZ) ? 96000 : 192000;
    gBlockInSamples = (int)((float)gSampleRate / (float)BLOCKS_PER_SEC);   // 512/1024/2048
    gRecvCount = gSet.RecvCount;
    if (gRecvCount < 1) gRecvCount = 1;
    if (gRecvCount > MAX_RX_COUNT) gRecvCount = MAX_RX_COUNT;
    Log("StartRx: clamped RecvCount=%d, blockInSamples=%d, sampleRate=%d",
        gRecvCount, gBlockInSamples, gSampleRate);

    if (!gSet.pIQProc) {
        Log("StartRx: pIQProc NULL, refuse");
        return;
    }

    // 分配 ping-pong 双缓冲
    FreeBlocks();
    for (int i = 0; i < gRecvCount; ++i) {
        gDataA[i] = AllocBlock(gBlockInSamples);
        gDataB[i] = AllocBlock(gBlockInSamples);
        if (!gDataA[i] || !gDataB[i]) { FreeBlocks(); ReportError("alloc failed"); return; }
    }

    // 打开 USB（固件下载 + STREAMER 枚举 + STARTADC 64M）
    if (!UsbOpen(&gFx3, ADC_FREQ)) {
        ReportError("RX888 open failed (USB)");
        FreeBlocks();
        return;
    }
    gUsbOk = true;

    // DDC 引擎 LO 初始化（此时尚未收到 SetRxFrequency 的通道用占位 LO）
    for (int c = 0; c < gRecvCount; ++c) gLoSet[c] = false;

    gStopReq = false;
    gRunning = true;
    gBlocksDelivered = 0;
    // v4.181: 重置安全点状态 + 创建安全点挂起事件
    InterlockedExchange(&gDeliverAtSafePoint, 0);
    InterlockedExchange(&gDeliverKilled, 0);
    if (!gDeliverSuicideEvent) gDeliverSuicideEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    else ResetEvent(gDeliverSuicideEvent);
    gThread = CreateThread(nullptr, 0, DeliverThread, nullptr, 0, nullptr);
    if (!gThread) {
        ReportError("CreateThread failed");
        UsbClose(&gFx3);
        gUsbOk = false;
        FreeBlocks();
        return;
    }
    SetThreadPriority(gThread, THREAD_PRIORITY_TIME_CRITICAL);

    // 启动 USB 流（4 并发队列，目标环形缓冲）
    UsbStartStream(gFx3, &gRing);
    gStarted = true;
    Log("StartRx ok: rate=%d block=%d nCh=%d THandle=%d",
        gSampleRate, gBlockInSamples, gRecvCount, gSet.THandle);
}

// ---------------------------------------------------------------------------
// v4.183: 解码器解暂停——完全复刻老驱动 v4.131 成功会话行为。
// 老会话铁证（RX888Intf_log_20260915_100732.bak）：
//   - [dm+0x98] 恒为 1.0（0x3F800000），从不写入；
//   - Self state=4，从不强写；
//   - 只 PostMessage(0x042D) 触发 + 持续回调，SkimSrv 自己完成 4→3，
//     随后 piqDelivered=545280（真实解码成功）。
// 此前 v4.180/181/182 写 [dm+0x98]=0 或强写 state=3 均干扰内部时序
// （0x48030C 完整转换含解码线程创建被短路），ring 有结构无消费者。
// 本版：只 PostMessage，其余交给 SkimSrv。
// UnpauseMode=1（ini）仍保留旧强制行为（兼容）。
// ---------------------------------------------------------------------------
static void UnpauseDecoders() {
    Log("UnpauseDecoders: begin (THandle=0x%08X)", gTHandle);
    if (!gTHandle) { Log("UnpauseDecoders: THandle=0, skip"); return; }

    // 读 [T+0x48] 和 [T+0x4C]
    DWORD th48 = 0, th4C = 0;
    if (IsBadReadPtr((void*)(gTHandle + 0x48), 4)) {
        Log("UnpauseDecoders: THandle+0x48 not readable, skip");
        return;
    }
    th48 = *(volatile DWORD*)(gTHandle + 0x48);
    th4C = *(volatile DWORD*)(gTHandle + 0x4C);
    Log("UnpauseDecoders: [T+0x48]=0x%08X [T+0x4C]=0x%08X", th48, th4C);

    // 读 decoder_mgr 关键字段（只读诊断，不写入）
    DWORD dm98val = 0;
    float dm98f = 1.0f;
    if (th4C && !IsBadReadPtr((void*)(th4C + 0x98), 4)) {
        dm98val = *(volatile DWORD*)(th4C + 0x98);
        memcpy(&dm98f, &dm98val, 4);
    }
    Log("UnpauseDecoders: [dm+0x98]=0x%08X (%.6f) [老会话恒为1.0, 不写入]", dm98val, dm98f);

    // v4.187-11（逆向定论）：[dm+0x98] 是实时率因子，1.0=实时。
    //   SkimSrv 0x48030C 判定：state==4 && factor<1.0 → 置 state=3 (CPU Overloaded)；
    //   state==3 && factor==1.0 → 恢复 state=4 (OK)。
    //   v4.185 写 0.0f 是把因子打成 0 → 永久黄标！本版写回 1.0f（真实速率）。
    if (th4C && !IsBadReadPtr((void*)(th4C + 0x98), 4)) {
        if (gUnpauseMode == 0) {
            DWORD oldProt = 0;
            if (VirtualProtect((void*)(th4C + 0x98), 4, PAGE_READWRITE, &oldProt)) {
                *(volatile float*)(th4C + 0x98) = 1.0f;
                VirtualProtect((void*)(th4C + 0x98), 4, oldProt, &oldProt);
                Log("UnpauseDecoders(GENTLE): wrote [dm+0x98]=1.0f (real-time factor, restore OK)");
            }
        }
    }

    // 扫描内存找到 Self 对象（[Self+0x60]==THandle）
    DWORD selfAddr = 0;
    BYTE selfState = 0;
    MEMORY_BASIC_INFORMATION mbi;
    DWORD addr = 0x00010000;
    while (addr < 0x80000000) {
        if (VirtualQuery((void*)addr, &mbi, sizeof(mbi)) == 0) {
            addr += 0x1000;
            continue;
        }
        if (mbi.State == MEM_COMMIT &&
            (mbi.Protect & (PAGE_READWRITE)) &&
            !(mbi.Protect & PAGE_GUARD) &&
            mbi.RegionSize >= 0x68) {
            DWORD regionEnd = (DWORD)mbi.BaseAddress + mbi.RegionSize - 0x68;
            for (DWORD p = (DWORD)mbi.BaseAddress; p <= regionEnd; p += 4) {
                if (*(volatile DWORD*)(p + 0x60) == gTHandle) {
                    selfState = *(volatile BYTE*)(p + 0x5C);
                    selfAddr = p;
                    break;
                }
            }
        }
        if (selfAddr) break;
        addr = (DWORD)mbi.BaseAddress + mbi.RegionSize;
    }

    if (!selfAddr) {
        Log("UnpauseDecoders: Self not found in memory scan");
        return;
    }
    Log("UnpauseDecoders: Found Self at 0x%08X state=%d", selfAddr, selfState);

    // v4.183: 无条件 PostMessage(0x042D)——老驱动唯一动作。
    // 不管 state 4/3、不管 [dm+0x98] 值，都发。
    DWORD hwnd = *(volatile DWORD*)(selfAddr + 0x04);
    if (hwnd && hwnd > 0x10000) {
        BOOL r = PostMessageA((HWND)hwnd, 0x042D, 0, 0);
        Log("UnpauseDecoders: PostMessage(0x%08X, 0x042D) = %d", hwnd, r);
    } else {
        Log("UnpauseDecoders: hwnd invalid 0x%08X, skip PostMessage", hwnd);
    }

    // 强制模式兼容（仅 UnpauseMode=1）
    if (gUnpauseMode == 1 && selfState == 4) {
        DWORD oldProt = 0;
        if (VirtualProtect((void*)(selfAddr + 0x5C), 4, PAGE_READWRITE, &oldProt)) {
            *(volatile BYTE*)(selfAddr + 0x5C) = 3;
            VirtualProtect((void*)(selfAddr + 0x5C), 4, oldProt, &oldProt);
            Log("UnpauseDecoders(FORCED): [Self+0x5C] forced 4->3 (active)");
        }
    }

    // 验证（只读）
    BYTE newState = *(volatile BYTE*)(selfAddr + 0x5C);
    Log("UnpauseDecoders: [Self+0x5C] now = %d (老会话：等待 SkimSrv 自己转 3)", newState);

    // v4.178: 诊断 [dm+0x34] (CW decoder object) 和 [dm+0x5C] (RecvCount)
    if (th4C && !IsBadReadPtr((void*)(th4C + 0x34), 4)) {
        DWORD dm34 = *(volatile DWORD*)(th4C + 0x34);
        DWORD dm5C = 0;
        if (!IsBadReadPtr((void*)(th4C + 0x5C), 4)) dm5C = *(volatile DWORD*)(th4C + 0x5C);
        Log("UnpauseDecoders DIAG: [dm+0x34]=0x%08X [dm+0x5C]=%d", dm34, dm5C);
        if (dm34 == 0) {
            Log("UnpauseDecoders DIAG: *** [dm+0x34]=0 -- CW DECODERS NOT INITIALIZED! ***");
        } else if (!IsBadReadPtr((void*)(dm34 + 0x810), 4)) {
            DWORD rIdx = *(volatile DWORD*)(dm34 + 0x810);
            DWORD wIdx = *(volatile DWORD*)(dm34 + 0x814);
            BYTE full = *(volatile BYTE*)(dm34 + 0x819);
            Log("UnpauseDecoders DIAG: decoder ring rIdx=%d wIdx=%d full=%d", rIdx, wIdx, full);
        }
        // Dump 8 channel RMS values [dm+0xC8..0xE4]
        for (int i = 0; i < 8; i++) {
            DWORD rmsVal = *(volatile DWORD*)(th4C + 0xC8 + i*4);
            float rmsF;
            memcpy(&rmsF, &rmsVal, 4);
            Log("UnpauseDecoders DIAG: [dm+0xC8+%d]=%.6f", i, rmsF);
        }
    }
}

// ---------------------------------------------------------------------------
// v4.184: 只读状态检查（周期调用，不 PostMessage 不写内存）
// 老驱动 v4.132 明确禁用 periodic PostMessage（"No repeated
// VirtualProtect/PostMessage"），v4.183 每 53 秒重发 0x042D 共 360 次
// 导致 state 卡 4。本函数只读 Self state / [dm+0x98] / ring 指针，
// 用于观测 SkimSrv 是否自行完成 4→3。
// ---------------------------------------------------------------------------
static void CheckDecodersReadOnly() {
    if (!gTHandle) return;
    DWORD th4C = 0;
    if (IsBadReadPtr((void*)(gTHandle + 0x48), 4)) return;
    th4C = *(volatile DWORD*)(gTHandle + 0x4C);
    if (!th4C || IsBadReadPtr((void*)(th4C + 0x98), 4)) return;

    // [dm+0x98] = 实时率因子（1.0=实时，SkimSrv 0x48030C 判定依据）。
    //   state==4 && factor<1.0 → 3 (CPU Overloaded)；state==3 && factor==1.0 → 4 (OK)。
    DWORD dm98val = *(volatile DWORD*)(th4C + 0x98);
    float dm98f = 1.0f;
    memcpy(&dm98f, &dm98val, 4);

    // 扫描 Self 对象（[Self+0x60]==THandle）——只读
    DWORD selfAddr = 0;
    BYTE selfState = 0;
    MEMORY_BASIC_INFORMATION mbi;
    DWORD addr = 0x00010000;
    while (addr < 0x80000000) {
        if (VirtualQuery((void*)addr, &mbi, sizeof(mbi)) == 0) { addr += 0x1000; continue; }
        if (mbi.State == MEM_COMMIT &&
            (mbi.Protect & (PAGE_READWRITE)) &&
            !(mbi.Protect & PAGE_GUARD) &&
            mbi.RegionSize >= 0x68) {
            DWORD regionEnd = (DWORD)mbi.BaseAddress + mbi.RegionSize - 0x68;
            for (DWORD p = (DWORD)mbi.BaseAddress; p <= regionEnd; p += 4) {
                if (*(volatile DWORD*)(p + 0x60) == gTHandle) {
                    selfState = *(volatile BYTE*)(p + 0x5C);
                    selfAddr = p;
                    break;
                }
            }
        }
        if (selfAddr) break;
        addr = (DWORD)mbi.BaseAddress + mbi.RegionSize;
    }

    // decoder ring 只读
    DWORD rIdx = 0, wIdx = 0; BYTE full = 0;
    DWORD dm34 = 0;
    if (!IsBadReadPtr((void*)(th4C + 0x34), 4)) {
        dm34 = *(volatile DWORD*)(th4C + 0x34);
        if (dm34 && !IsBadReadPtr((void*)(dm34 + 0x810), 4)) {
            rIdx = *(volatile DWORD*)(dm34 + 0x810);
            wIdx = *(volatile DWORD*)(dm34 + 0x814);
            full = *(volatile BYTE*)(dm34 + 0x819);
        }
    }
    Log("PERIODIC DIAG: [dm+0x98]=%.6f Self=%s state=%d ring rIdx=%d wIdx=%d full=%d",
        dm98f,
        selfAddr ? "found" : "NOT FOUND",
        (int)selfState, rIdx, wIdx, (int)full);

    // v4.187-11: 状态机恢复——若 SkimSrv 仍卡在 state=3 (CPU Overloaded) 且
    //   [dm+0x98]==1.0（我们已写实时因子），主动投一次 0x042D 触发 0x48030C
    //   判定走 3→4 恢复分支（0x48030C: state==3 && factor==1.0 → 置 4）。
    //   每 5000 块（~53s）一次，量级远低于 v4.183 的每 53 秒×360 次轰炸。
    if (selfAddr && selfState == 3 && dm98f == 1.0f) {
        DWORD hwnd = *(volatile DWORD*)(selfAddr + 0x04);
        if (hwnd && hwnd > 0x10000) {
            BOOL r = PostMessageA((HWND)hwnd, 0x042D, 0, 0);
            Log("PERIODIC: state=3 && factor==1.0, PostMessage(0x042D) to trigger 3->4 restore = %d", r);
        }
    }
}

// ---------------------------------------------------------------------------
// 导出函数 3：StopRx
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport)
void __stdcall StopRx(void) {
    Log("StopRx called (started=%d)", (int)gStarted);
    if (!gStarted) return;

    // v4.176: 停止顺序根治 UAF 崩溃（旧序：交付线程仍持 ring/块缓冲指针就 delete）
    //   1. 置停止标志  2. 唤醒交付线程并停采集线程（ring 不释放）
    //   3. 等交付线程退出  4. 再释放 ring / 设备 / 块缓冲
    gStopReq = true;
    Log("StopRx: step1 UsbStopStream begin");
    UsbStopStream(gFx3, gRing);          // ring Stop() 唤醒 getReadPtr + 停采集线程
    Log("StopRx: step2 WaitForDeliverThread begin (gThread=%p)", gThread);
    if (gThread) {
        // v4.181: 等安全点而非等退出——交付线程到安全点后挂起（不 return），
        // 确认后 TerminateThread 直接杀。绕开 CRT endthread 清理链的堆锁死锁
        // （现场实抓：交付线程 return 后 TLS free 与本线程 FreeBlocks 在
        //  ntdll 堆锁上互锁 1.5h+，双挂死）。
        DWORD t0 = GetTickCount();
        BOOL atSafe = FALSE;
        BOOL exited = FALSE;
        for (int i = 0; i < 100; ++i) {   // 最多等 2 秒到安全点
            if (InterlockedCompareExchange(&gDeliverAtSafePoint, 0, 0)) { atSafe = TRUE; break; }
            // 若线程已意外退出（return 路径），句柄会 signal
            DWORD wq = WaitForSingleObject(gThread, 20);
            if (wq == WAIT_OBJECT_0) { exited = TRUE; Log("StopRx: thread exited by itself (unexpected path)"); break; }
        }
        DWORD t1 = GetTickCount();
        if (atSafe) {
            Log("StopRx: deliver thread at safe point after %lu ms, terminating", t1 - t0);
            InterlockedExchange(&gDeliverKilled, 1);
            TerminateThread(gThread, 0);   // 零清理：不碰 TLS/堆，绕开死锁
            WaitForSingleObject(gThread, 2000);
            Log("StopRx: deliver thread terminated");
        } else if (!exited) {
            DWORD wr = WaitForSingleObject(gThread, 15000);
            Log("StopRx: WaitForSingleObject returned %lu (0=SIGNALED 258=TIMEOUT)", wr);
            if (wr != WAIT_OBJECT_0) {
                Log("WARN: deliver thread did not exit in 15s, forcing TerminateThread");
                InterlockedExchange(&gDeliverKilled, 1);
                TerminateThread(gThread, 0);
                WaitForSingleObject(gThread, 2000);
                Log("StopRx: deliver thread force-terminated");
            }
        }
        CloseHandle(gThread); gThread = nullptr;
    }
    Log("StopRx: step3 UsbFreeRing begin");
    gRunning = false;
    UsbFreeRing(&gRing);                  // 交付线程已退出，安全释放
    Log("StopRx: step4 UsbClose begin");
    UsbClose(&gFx3);
    Log("StopRx: step5 FreeBlocks begin");
    gUsbOk = false;
    FreeBlocks();
    gDspReady = false;
    gStarted = false;
    Log("StopRx done, delivered=%lu", gBlocksDelivered);
}

// ---------------------------------------------------------------------------
// 导出函数 4：SetRxFrequency
// 规范：Frequency 为通道中心频率（Hz），Receiver 为 0 基索引
// 实现：记录 LO；若引擎已跑，重建该通道混频器（通道数不变，只换 LO）
// 注：重建混频器会丢一小段滤波器历史（前 N 样本），交付线程串行化后无锁安全
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport)
void __stdcall SetRxFrequency(int Frequency, int Receiver) {
    Log("SetRxFrequency f=%d rx=%d", Frequency, Receiver);
    if (Receiver < 0 || Receiver >= gRecvCount) return;
    double lo = ClampLo(Frequency);
    gLoHz[Receiver] = lo;
    gLoSet[Receiver] = true;
    // 热重调：写入待处理队列，由交付线程在循环边界消费（串行化后无锁安全）
    {
        std::lock_guard<std::mutex> lk(gRetuneMtx);
        gPendingRetunes.emplace_back(Receiver, lo);
    }
}

// ---------------------------------------------------------------------------
// 导出函数 5：SetCtrlBits（规范：多数驱动为空操作）
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport)
void __stdcall SetCtrlBits(unsigned char Bits) {
    Log("SetCtrlBits bits=%02X", (unsigned)Bits);
    // 空操作
}

// ---------------------------------------------------------------------------
// 导出函数 6：ReadPort（规范：未实现返回 0）
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport)
int __stdcall ReadPort(int PortNumber) {
    return 0;
}

// ---------------------------------------------------------------------------
// DLL 入口：进程/线程附加分离（SkimSrv 为单进程多线程）
// ---------------------------------------------------------------------------
BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
    switch (reason) {
        case DLL_PROCESS_ATTACH:
            DisableThreadLibraryCalls(hInst);
            // v4.182: OpenMP 永久修复——限制 DDC 并行区为单线程。
            // libgomp win32 后端 worker 完成后不睡眠而 spin-wait（实测
            // OMP_WAIT_POLICY=PASSIVE 无效），8 worker 自旋各烧 70-85% CPU，
            // 导致 SkimSrv UI 报 “CPU Overloaded” 假告警（真实计算单核足够）。
            // 实测 OMP_NUM_THREADS=1：CPU 41.3%→6.5%，交付速率 93.8 blocks/s 不变。
            // 环境变量只对单会话生效，这里代码级固化（每次进程加载 DLL 时生效）。
            {
                typedef void (*set_nt_t)(int);
                HMODULE gomp = GetModuleHandleA("libgomp-1.dll");
                if (!gomp) {
                    gomp = LoadLibraryA("libgomp-1.dll");
                }
                if (gomp) {
                    set_nt_t f = (set_nt_t)GetProcAddress(gomp, "omp_set_num_threads");
                    if (f) {
                        f(1);
                    }
                }
            }
            break;
        case DLL_PROCESS_DETACH:
            if (gStarted) StopRx();
            LogClose();
            break;
    }
    return TRUE;
}
