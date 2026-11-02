// pipeline_bench.cpp — 生产 worker 管道分段微基准（v4.193 8559U wait≈600µs/千块 归因）
// 目的：把每块每通道的 worker 工作拆成 6 段实测，回答「生产 wait 构成」：
//   full   = 生产原路径 MixDecFIR::process + FracResampler::process（对照生产 wait）
//   mixdec = 仅 MixDecFIR::process（输入环分组 memcpy + FIR 内积 + rot 查表复乘）
//   frac   = 仅 FracResampler::process（其逐样本双写输入环 + ~49 输出×330tap 内积）
//   fir    = 仅 emitOne 的 FIR 内积部分（4 路部分和，I/Q 两套 288tap×2048 输出）
//   rot    = 仅 emitOne 的旋转因子查表+插值+复乘（2048 次/块，512KB 共享表随机访问）
//   memcpy = 仅输入环分组双写（2048 次 2×memcpy(128B)，v4.193 A 项形态）
// 对账式（自洽检查）：mixdec ≈ memcpy+fir+rot；full ≈ mixdec+frac（差值=缓存/交织交互）
// 口径：MixDecFIR N=288tap(R=32×K=9)、fc=110k、att=120dB、LO=7.023MHz（成本代表值）；
//       FracResampler K=330、fc=0.48×fsOut/2M；采样率默认 48k（L=3,M=125，生产口径），
//       可传第二参数 96 切 L=6,M=125。32 位 MinGW 编译（生产 DLL 为 32 位，代码gen同构）。
// 用法：pipeline_bench [1t|all] [48|96]
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include <windows.h>
#include "polyphase.h"

static const int N_TAP = 288;              // 生产 MixDecFIR: R*K = 32*9
static const int R_DEC = 32;               // 64M -> 2M
static const int N_IN   = 65536;           // 每块每通道输入（USB 块）
static const int N_OUT  = N_IN / R_DEC;    // 2048 中频输出/块/通道
static const int FR_K   = 330;             // 生产 FracResampler taps
static const double LO_HZ = 7023000.0;
static const double FS_IN = 64000000.0;
static const int REPS  = 100;              // 1T 计时重复
static const int MTREP = 20;               // 8T 计时批次
static const int NMODE = 7;
static const char* MODE_NAMES[NMODE] = { "full", "mixdec", "frac", "fir", "rot", "memcpy", "split" };

static int FR_L = 3, FR_M = 125;           // 48k 默认（生产口径）

static double now_us() {
    static LARGE_INTEGER f = {};
    if (!f.QuadPart) QueryPerformanceFrequency(&f);
    LARGE_INTEGER c; QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1e6 / (double)f.QuadPart;
}

static unsigned rngState = 0x12345678u;
static float frand() {
    rngState = rngState * 1664525u + 1013904223u;
    return (float)((rngState >> 8) & 0xFFFF) / 32768.0f - 1.0f;
}

struct WorkerData {
    skdsp::MixDecFIR mix;
    skdsp::FracResampler frac;
    std::vector<float> in, mI, mQ, oI, oQ;
    std::vector<float> win, cI, cQ, fo;   // fir-only 独立数据
    std::vector<float> rout;              // rot-only 输出
    std::vector<float> dl;                // memcpy-only 延迟线
    uint64_t rotInc;
    int nM, nO;
};

static void wd_init(WorkerData& w) {
    w.mix.init(LO_HZ, FS_IN, R_DEC, 9, 110000.0, 120.0);
    w.frac.init(FR_L, FR_M, FR_K, (0.48 * 48000.0) / 2000000.0, 120.0);
    w.in.resize(N_IN);
    for (int i = 0; i < N_IN; ++i)
        w.in[i] = 0.5f * sinf(2.f * 3.14159265f * 976.5625f * (float)i / 2000000.f) + 0.2f * frand();
    w.mI.assign(N_OUT + 64, 0); w.mQ.assign(N_OUT + 64, 0);
    w.oI.assign(N_OUT + 64, 0); w.oQ.assign(N_OUT + 64, 0);
    // frac 独立跑前预置中频输入（避免读到未初始化；正常由 full/mixdec 先填充）
    for (int i = 0; i < N_OUT; ++i) {
        w.mI[i] = 0.1f * sinf(0.001f * (float)i);
        w.mQ[i] = 0.1f * cosf(0.001f * (float)i);
    }
    // fir-only：窗口（N_TAP 预置历史 + 本块输入）与折叠系数（与 MixDecFIR::init 同公式）
    w.win.resize(N_TAP + N_IN);
    for (int j = 0; j < N_TAP; ++j) w.win[j] = frand();
    for (int i = 0; i < N_IN; ++i) w.win[N_TAP + i] = w.in[i];
    {
        std::vector<float> h(N_TAP);
        skdsp::design_lpf(h.data(), N_TAP, 110000.0 / FS_IN, 120.0);
        w.cI.resize(N_TAP); w.cQ.resize(N_TAP);
        const double w0 = 2.0 * 3.14159265358979323846 * LO_HZ / FS_IN;
        for (int jp = 0; jp < N_TAP; ++jp) {
            int j = N_TAP - 1 - jp;
            w.cI[jp] = (float)(h[j] * cos(w0 * j));
            w.cQ[jp] = (float)(h[j] * sin(w0 * j));
        }
    }
    w.fo.assign(N_OUT, 0);
    w.rout.assign(2 * N_OUT, 0);
    w.dl.assign(2 * N_TAP, 0);
    double incTurns = fmod(LO_HZ * R_DEC / FS_IN, 1.0);
    if (incTurns < 0) incTurns += 1.0;
    w.rotInc = (uint64_t)(incTurns * 18446744073709551616.0);
    w.nM = 0; w.nO = 0;
}

__declspec(noinline) static int mode_full(WorkerData& w) {
    int nM = w.mix.process(w.in.data(), N_IN, w.mI.data(), w.mQ.data(), (int)w.mI.size());
    int nO = w.frac.process(w.mI.data(), w.mQ.data(), nM, w.oI.data(), w.oQ.data(), (int)w.oI.size());
    w.nM = nM; w.nO = nO;
    return nM + nO;
}
__declspec(noinline) static int mode_mixdec(WorkerData& w) {
    return w.mix.process(w.in.data(), N_IN, w.mI.data(), w.mQ.data(), (int)w.mI.size());
}
__declspec(noinline) static int mode_frac(WorkerData& w) {
    return w.frac.process(w.mI.data(), w.mQ.data(), N_OUT, w.oI.data(), w.oQ.data(), (int)w.oI.size());
}
__declspec(noinline) static int mode_fir(WorkerData& w) {
    const float* win = w.win.data();
    const float* cI = w.cI.data();
    const float* cQ = w.cQ.data();
    float* fo = w.fo.data();
    for (int m = 0; m < N_OUT; ++m) {
        const float* p = win + (size_t)m * R_DEC;
        float aI0 = 0, aI1 = 0, aI2 = 0, aI3 = 0;
        float aQ0 = 0, aQ1 = 0, aQ2 = 0, aQ3 = 0;
        int jp = 0;
        for (; jp + 4 <= N_TAP; jp += 4) {
            aI0 += p[jp  ] * cI[jp  ]; aQ0 += p[jp  ] * cQ[jp  ];
            aI1 += p[jp+1] * cI[jp+1]; aQ1 += p[jp+1] * cQ[jp+1];
            aI2 += p[jp+2] * cI[jp+2]; aQ2 += p[jp+2] * cQ[jp+2];
            aI3 += p[jp+3] * cI[jp+3]; aQ3 += p[jp+3] * cQ[jp+3];
        }
        float accI = (aI0 + aI1) + (aI2 + aI3);
        float accQ = (aQ0 + aQ1) + (aQ2 + aQ3);
        fo[m] = accI + accQ;
    }
    return N_OUT;
}
__declspec(noinline) static int mode_rot(WorkerData& w) {
    const float* tbl = skdsp::TabNCO::table();
    const float* win = w.win.data();
    float* ro = w.rout.data();
    uint64_t ph = 0x4000000000000000ull;   // 任意起始相位
    for (int m = 0; m < N_OUT; ++m) {
        uint32_t idx = (uint32_t)(ph >> 48);
        uint32_t frac = (uint32_t)(ph >> 33) & 0x7FFF;
        const float* e0 = tbl + ((size_t)idx << 1);
        const float* e1 = tbl + ((size_t)((idx + 1) & 0xFFFF) << 1);
        float k = frac * (1.0f / 32768.0f);
        float rc = e0[0] + (e1[0] - e0[0]) * k;
        float rs = e0[1] + (e1[1] - e0[1]) * k;
        float x = win[(size_t)m * R_DEC], y = win[(size_t)m * R_DEC + 1];
        ro[2 * m]     = x * rc + y * rs;
        ro[2 * m + 1] = y * rc - x * rs;
        ph += w.rotInc;
    }
    return N_OUT;
}
__declspec(noinline) static int mode_memcpy(WorkerData& w) {
    float* d = w.dl.data();
    const float* in = w.in.data();
    const int N = N_TAP, R = R_DEC;
    int wpos = N;
    for (int i = 0; i + R <= N_IN; i += R) {
        memcpy(d + wpos, in + i, R * sizeof(float));
        memcpy(d + wpos - N, in + i, R * sizeof(float));
        wpos += R;
        if (wpos == 2 * N) { memcpy(d, d + N, N * sizeof(float)); wpos = N; }
    }
    return wpos;
}

// split 模式：两阶段 MixDecFIR——阶段1 全块 memcpy 进延迟线（模拟"写完再算"），
// 阶段2 从稳定延迟线跑 2048 次 FIR+rot。若 split ≈ memcpy+fir+rot 之和，
// 则「原位写读紧邻的存储转发冲突」即原位惩罚主因（实验性诊断模式，
// 非生产等价路径：生产必须逐组写因为窗口需要最新 R 样本）。
__declspec(noinline) static int mode_split(WorkerData& w) {
    float* d = w.dl.data();
    const float* in = w.in.data();
    const int N = N_TAP, R = R_DEC;
    // 阶段1：整块分组 memcpy（把 dl 当作"本块延迟线"从 N 开始写满 N_IN）
    int wpos = N;
    for (int i = 0; i + R <= N_IN; i += R) {
        memcpy(d + wpos, in + i, R * sizeof(float));
        wpos += R;
        if (wpos == 2 * N) { memcpy(d, d + N, N * sizeof(float)); wpos = N; }
    }
    // 阶段2：从稳定数据跑 FIR+rot（win 路径与 fir/rot 模式同形）
    const float* win = w.win.data();
    const float* cI = w.cI.data();
    const float* cQ = w.cQ.data();
    const float* tbl = skdsp::TabNCO::table();
    uint64_t ph = 0x4000000000000000ull;
    for (int m = 0; m < N_OUT; ++m) {
        const float* p = win + (size_t)m * R_DEC;
        float aI0 = 0, aI1 = 0, aI2 = 0, aI3 = 0;
        float aQ0 = 0, aQ1 = 0, aQ2 = 0, aQ3 = 0;
        int jp = 0;
        for (; jp + 4 <= N_TAP; jp += 4) {
            aI0 += p[jp  ] * cI[jp  ]; aQ0 += p[jp  ] * cQ[jp  ];
            aI1 += p[jp+1] * cI[jp+1]; aQ1 += p[jp+1] * cQ[jp+1];
            aI2 += p[jp+2] * cI[jp+2]; aQ2 += p[jp+2] * cQ[jp+2];
            aI3 += p[jp+3] * cI[jp+3]; aQ3 += p[jp+3] * cQ[jp+3];
        }
        float accI = (aI0 + aI1) + (aI2 + aI3);
        float accQ = (aQ0 + aQ1) + (aQ2 + aQ3);
        uint32_t idx = (uint32_t)(ph >> 48);
        uint32_t frac = (uint32_t)(ph >> 33) & 0x7FFF;
        const float* e0 = tbl + ((size_t)idx << 1);
        const float* e1 = tbl + ((size_t)((idx + 1) & 0xFFFF) << 1);
        float k = frac * (1.0f / 32768.0f);
        float rc = e0[0] + (e1[0] - e0[0]) * k;
        float rs = e0[1] + (e1[1] - e0[1]) * k;
        w.oI[m] = accI * rc + accQ * rs;
        w.oQ[m] = accQ * rc - accI * rs;
        ph += w.rotInc;
    }
    return N_OUT;
}

static int run_mode(int mode, WorkerData& w) {
    switch (mode) {
        case 0: return mode_full(w);
        case 1: return mode_mixdec(w);
        case 2: return mode_frac(w);
        case 3: return mode_fir(w);
        case 4: return mode_rot(w);
        case 5: return mode_memcpy(w);
        default: return mode_split(w);
    }
}

struct MtCtx {
    WorkerData* wd;
    int mode;
    int reps;
    HANDLE begEv, doneEv;   // 均 auto-reset
};
static DWORD WINAPI mt_worker(LPVOID param) {
    MtCtx* c = (MtCtx*)param;
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    for (int r = 0; r < c->reps; ++r) {
        WaitForSingleObject(c->begEv, INFINITE);
        run_mode(c->mode, *c->wd);
        SetEvent(c->doneEv);
    }
    return 0;
}

int main(int argc, char** argv) {
    const char* sec = (argc > 1) ? argv[1] : "all";
    if (argc > 2 && strcmp(argv[2], "96") == 0) { FR_L = 6; FR_M = 125; }
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    FILE* fo = fopen("pipeline_bench_result.txt", "w");
    if (!fo) fo = stdout;
    #define LOG(...) do { fprintf(fo, __VA_ARGS__); fflush(fo); if (fo != stdout) { printf(__VA_ARGS__); fflush(stdout); } } while (0)
    LOG("=== pipeline_bench  N=%d R=%d N_IN=%d N_OUT=%d  Frac L/M=%d/%d K=%d ===\n",
            N_TAP, R_DEC, N_IN, N_OUT, FR_L, FR_M, FR_K);
    SYSTEM_INFO si; GetSystemInfo(&si);
    LOG("LogicalCPU=%d  (32-bit MinGW, same codegen as production DLL)\n", si.dwNumberOfProcessors);

    skdsp::TabNCO::ensureTablePub();   // 预建 512KB 共享正弦表（与生产一致）

    // ---- 1T 分段 ----
    WorkerData wd; wd_init(wd);
    double t1[NMODE];
    for (int mode = 0; mode < NMODE; ++mode) {
        for (int r = 0; r < 3; ++r) run_mode(mode, wd);
        double t0 = now_us();
        for (int r = 0; r < REPS; ++r) run_mode(mode, wd);
        t1[mode] = (now_us() - t0) / REPS;
    }
    LOG("[1T]");
    for (int m = 0; m < NMODE; ++m) LOG(" %s=%.1fus", MODE_NAMES[m], t1[m]);
    LOG("\n");
    double fullFlop = (double)N_OUT * (2 * N_TAP * 2)          // FIR I+Q
                    + (double)wd.nO * FR_K * 2 * 2             // frac 内积
                    + (double)N_OUT * 8;                        // rot 复乘
    LOG("[1T-FLOPS] full=%.2f GFLOPS/worker  (fir 段 %.2f GFLOPS)\n",
            fullFlop / t1[0] / 1000.0,
            ((double)N_OUT * N_TAP * 2 * 2) / t1[3] / 1000.0);
    LOG("[NUM] nM=%d (期望 2048)  nO=%d (期望约 %d)\n", wd.nM, wd.nO, (int)((long long)N_OUT * FR_L / FR_M));
    LOG("[SUM-1T] mixdec=%.1f vs memcpy+fir+rot=%.1f vs split=%.1f  |  full=%.1f vs mixdec+frac=%.1f  (us)\n",
            t1[1], t1[5] + t1[3] + t1[4], t1[6], t1[0], t1[1] + t1[2]);

    // ---- 8T（模拟生产 8 worker；本地有生产进程干扰时仅作参考） ----
    if (strcmp(sec, "1t") != 0) {
        static WorkerData wds[8];
        std::vector<MtCtx> ctx(8);
        std::vector<HANDLE> done(8), begs(8), th(8);
        for (int i = 0; i < 8; ++i) {
            wd_init(wds[i]);
            ctx[i].wd = &wds[i];
            ctx[i].mode = 0;
            ctx[i].reps = NMODE * (2 + MTREP);   // 七模式共用配额（每模式 2 预热 + 20 计时）
            ctx[i].begEv = CreateEvent(nullptr, FALSE, FALSE, nullptr);
            ctx[i].doneEv = CreateEvent(nullptr, FALSE, FALSE, nullptr);
            begs[i] = ctx[i].begEv; done[i] = ctx[i].doneEv;
            th[i] = CreateThread(nullptr, 0, mt_worker, &ctx[i], 0, nullptr);
        }
        WaitForMultipleObjects(8, th.data(), TRUE, 10000);   // 等 worker 就绪进入等待
        double t8[NMODE]; bool abortFlags[NMODE] = { false };
        for (int mode = 0; mode < NMODE; ++mode) {
            for (int i = 0; i < 8; ++i) ctx[i].mode = mode;
            for (int r = 0; r < 2; ++r) {
                for (int i = 0; i < 8; ++i) SetEvent(begs[i]);
                WaitForMultipleObjects(8, done.data(), TRUE, INFINITE);
            }
            double t0 = now_us();
            for (int r = 0; r < MTREP; ++r) {
                for (int i = 0; i < 8; ++i) SetEvent(begs[i]);
                DWORD w = WaitForMultipleObjects(8, done.data(), TRUE, 30000);
                if (w != WAIT_OBJECT_0) {
                    LOG("[8T] %s ABORTED (rc=%lu r=%d)\n", MODE_NAMES[mode], w, r);
                    abortFlags[mode] = true;
                    break;
                }
            }
            if (abortFlags[mode]) continue;
            t8[mode] = (now_us() - t0) / MTREP;
            LOG("[8T] %s=%.1fus\n", MODE_NAMES[mode], t8[mode]);
        }
        if (!abortFlags[1] && !abortFlags[3] && !abortFlags[4] && !abortFlags[5] && !abortFlags[6])
            LOG("[SUM-8T] mixdec=%.1f vs memcpy+fir+rot=%.1f vs split=%.1f\n", t8[1], t8[5] + t8[3] + t8[4], t8[6]);
        if (!abortFlags[0] && !abortFlags[1] && !abortFlags[2])
            LOG("[SUM-8T] full=%.1f vs mixdec+frac=%.1f  (对照生产机 wait≈600us/块)\n", t8[0], t8[1] + t8[2]);
    }

    LOG("=== end ===\n");
    if (fo != stdout) fclose(fo);
    printf("done, result written to pipeline_bench_result.txt\n");
    return 0;
}
