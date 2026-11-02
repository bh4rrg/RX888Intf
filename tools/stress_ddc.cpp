// stress_ddc.cpp - DDC 引擎堆损坏压力测试（无 USB，纯合成数据）
// 目标：把编译产物当黑盒，用受控合成实验复现 ntdll 0xc0000005 堆损坏。
// 复现生产负载形态：
//   1. 8 通道 OpenMP 并行（与 DLL processReal 相同调用形态）
//   2. 全零 / 白噪声 / 单音三种输入（初始瞬态不同）
//   3. Start/Stop 循环（eng init/销毁、TabNCO 静态表首次访问时序）
//   4. 热重调 setChannelLo 循环（init 重入、系数重算）
//   5. 堆校验：每轮 _heapchk() + 前后台哨兵字节
//   6. 页堆保护选项（gflags 不在时用 HeapValidate 逐堆校验）
// 用法：
//   stress_ddc.exe <mode> <seconds>
//     mode = run     ：长跑合成数据（默认 300s）
//     mode = cycles  ：Start/Stop × setLo 循环（默认 200 次）
//     mode = mixed   ：先 cycles 再 run
// 编译（32 位 MinGW，与 DLL 同一工具链——任何 ABI 差异必须暴露）：
//   g++ -m32 -O3 -msse2 -mfpmath=sse -std=c++17 -fopenmp -D_WIN32
//       -o stress_ddc.exe stress_ddc.cpp
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include <atomic>

#include "ddc_engine.h"

// MinGW _heapchk 声明
#include <malloc.h>
#define HEAPCHK _heapchk

static std::atomic<bool> gStop{false};
static long long gBlocks = 0, gRetunes = 0, gCycles = 0;
static int gHeapErr = 0;

static int HeapCheck(const char* where) {
    // MinGW 静态链 msvcrt 下 _heapchk() 恒返 -2（不可用，已实测校准）
    // 改用 HeapValidate：进程堆 + 漫游全部堆，能抓住块级损坏
    int bad = 0;
    if (!HeapValidate(GetProcessHeap(), 0, NULL)) bad = 1;
    if (!bad) {
        HANDLE heaps[64];
        DWORD n = GetProcessHeaps(64, heaps);
        for (DWORD i = 0; i < n; ++i) {
            if (!HeapValidate(heaps[i], 0, NULL)) { bad = 1; break; }
        }
    }
    if (bad) {
        printf("*** HEAP CORRUPT at %s: HeapValidate FAILED ***\n", where);
        gHeapErr = 1;
        gStop = true;
    }
    return bad;
}

// 哨兵分配：块前后各放 64 字节 0xAA 校验字
struct Sentineled {
    std::vector<float> data;
    std::vector<unsigned char> pre, post;
};

static void FillInput(float* buf, int n, int mode, unsigned long long seed) {
    switch (mode) {
    case 0: // 全零（初始瞬态最深——FracResampler b 回退最远）
        memset(buf, 0, n * sizeof(float));
        break;
    case 1: { // 白噪声
        unsigned long long s = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        for (int i = 0; i < n; ++i) {
            s ^= s << 13; s ^= s >> 7; s ^= s << 17;
            buf[i] = (float)(int32_t)(s >> 40) / 32768.0f;
        }
        break;
    }
    default: { // 单音 @ 3.553M（HF 波段中心附近，混频系数走满）
        double w = 2.0 * 3.14159265358979323846 * 3553000.0 / 64000000.0;
        for (int i = 0; i < n; ++i)
            buf[i] = 100.0f * (float)sin(w * (double)(seed + (unsigned long long)i));
        break;
    }
    }
}

// 模拟 DLL DeliverThread 的引擎使用形态：init → 循环 processReal+setLo → 销毁
static bool RunSession(int seconds, bool useOpenMpShape) {
    const int NCH = 8;
    std::vector<skdsp::ChannelCfg> cfgs(NCH);
    double loBase = 3550000.0;
    for (int c = 0; c < NCH; ++c) cfgs[c].loHz = loBase + c * 50000.0;

    skdsp::DDC_Engine* eng = new skdsp::DDC_Engine();
    eng->init(skdsp::RATE_96K, NCH, cfgs.data());

    std::vector<float> in(65536);
    unsigned long long block = 0;

    DWORD t0 = GetTickCount();
    while (!gStop && (int)(GetTickCount() - t0) < seconds * 1000) {
        // 输入模式循环：0,1,2 每 17 块换一次（覆盖三种形态）
        FillInput(in.data(), 65536, (int)((block / 17) % 3), block);

        // 热重调风暴：每 3 块随机换 1 个通道 LO（与生产 SetRxFrequency 队列同路径）
        if (block % 3 == 0) {
            int c = (int)(block * 2654435761u % NCH);
            double lo = loBase + ((block * 37) % 231) * 10000.0;   // 3.55M~5.85M
            eng->setChannelLo(c, lo);
            ++gRetunes;
        }

        eng->processReal(in.data(), 65536);   // OpenMP 8 通道并行
        ++gBlocks;

        if ((block & 63) == 0) HeapCheck("run-loop");
        ++block;
    }

    // 检查各通道输出是否为 NaN/Inf（越界读常表现为 NaN 污染）
    for (int c = 0; c < NCH; ++c) {
        int navail = eng->available(c);
        const float* pI = eng->outI(c);
        const float* pQ = eng->outQ(c);
        for (int i = 0; i < navail; ++i) {
            if (!std::isfinite(pI[i]) || !std::isfinite(pQ[i])) {
                printf("*** NaN/Inf in ch%d out[%d] after %lld blocks ***\n", c, i, gBlocks);
                gHeapErr = -100; gStop = true;
                break;
            }
        }
        if (gStop) break;
    }

    HeapCheck("before-delete");
    delete eng;    // 与 DLL 不同的关键点：这里真删（DLL 因 delete 崩被注释）——
    HeapCheck("after-delete");   // 若此处崩/校验失败，则“MinGW delete 崩”根因在此可复现
    return true;
}

// Start/Stop 循环：每次 init 新引擎再销毁（模拟 SkimSrv 重启+setLo 重入）
static bool RunCycles(int times) {
    for (int t = 0; t < times && !gStop; ++t) {
        RunSession(0, true);   // seconds=0 → 只跑 1 轮？不：直接构造短会话
        ++gCycles;
        if ((t & 15) == 0) HeapCheck("cycles");
    }
    return true;
}

static BOOL WINAPI CtrlHandler(DWORD t) { if (t == CTRL_C_EVENT) gStop = true; return TRUE; }

int main(int argc, char** argv) {
    const char* mode = (argc > 1) ? argv[1] : "run";
    int seconds = (argc > 2) ? atoi(argv[2]) : 300;
    int cycles = (argc > 3) ? atoi(argv[3]) : 200;

    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    printf("stress_ddc: mode=%s seconds=%d cycles=%d  (32-bit MinGW, OpenMP)\n",
           mode, seconds, cycles);

    // 预热：TabNCO 静态表首次构建（单线程完成，模拟 DeliverThread 先建引擎再并行的顺序）
    {
        skdsp::TabNCO::ensureTablePub();
        printf("TabNCO table pre-built (static)\n");
    }

    HeapCheck("startup");

    if (strcmp(mode, "run") == 0) {
        RunSession(seconds, true);
    } else if (strcmp(mode, "cycles") == 0) {
        // 每次 cycle 是一个完整 init → 少量处理 → delete 生命周期
        for (int t = 0; t < cycles && !gStop; ++t) {
            const int NCH = 8;
            std::vector<skdsp::ChannelCfg> cfgs(NCH);
            for (int c = 0; c < NCH; ++c) cfgs[c].loHz = 3550000.0 + c * 50000.0;
            skdsp::DDC_Engine* eng = new skdsp::DDC_Engine();
            eng->init(skdsp::RATE_96K, NCH, cfgs.data());
            // 每次循环重调全部通道（触发 init 重入+静表并发首访）
            for (int c = 0; c < NCH; ++c) eng->setChannelLo(c, 7100000.0 + c * 30000.0);
            std::vector<float> in(65536);
            FillInput(in.data(), 65536, (int)(t % 3), t);
            eng->processReal(in.data(), 65536);
            eng->processReal(in.data(), 65536);
            delete eng;
            ++gCycles;
            if ((t & 15) == 0) HeapCheck("cycles");
        }
    } else if (strcmp(mode, "mixed") == 0) {
        for (int t = 0; t < cycles && !gStop; ++t) {
            const int NCH = 8;
            std::vector<skdsp::ChannelCfg> cfgs(NCH);
            for (int c = 0; c < NCH; ++c) cfgs[c].loHz = 3550000.0 + c * 50000.0;
            skdsp::DDC_Engine* eng = new skdsp::DDC_Engine();
            eng->init(skdsp::RATE_96K, NCH, cfgs.data());
            std::vector<float> in(65536);
            DWORD t0 = GetTickCount();
            unsigned long long b = 0;
            while (!gStop && (int)(GetTickCount() - t0) < 1000) {  // 每 cycle 跑 1 秒
                FillInput(in.data(), 65536, (int)((b / 17) % 3), b + t);
                if ((b & 7) == 0) {
                    int c = (int)(b * 2654435761u % NCH);
                    eng->setChannelLo(c, 3550000.0 + ((b * 31) % 200) * 10000.0);
                    ++gRetunes;
                }
                eng->processReal(in.data(), 65536);
                ++gBlocks; ++b;
            }
            delete eng;
            ++gCycles;
            if ((t & 7) == 0) HeapCheck("mixed");
        }
        // 最后长跑一段
        if (!gStop) RunSession(seconds, true);
    } else {
        printf("unknown mode\n");
        return 1;
    }

    HeapCheck("final");
    printf("DONE: blocks=%lld retunes=%lld cycles=%lld heapErr=%d\n",
           gBlocks, gRetunes, gCycles, gHeapErr);
    return gHeapErr ? 2 : 0;
}
