// ddc_engine.h v3 - 高速多通道 DDC 引擎（用于新 RX888 SkimSrv 驱动）
// v4.196：waitBatch 混合同步（短自旋 + WaitAll doneEv_）——CDC 归因（2026-11-06，
//   实机 QPC 263 样本）：主线程 wait 段 332µs/块为纯自旋烧核（主线程唯一真实烧核
//   等待点，prefetch 段在 ringbuffer 内早已是 CV 休眠）。改为短自旋覆盖快路径后
//   落内核事件，预期整机进程 CPU 60% → ~57%。worker 完成处新增 SetEvent(doneEv_)。
// v3.1（v4.190）：DDC_Engine 增加 shutdownPool()——退出路径必须显式关闭
//   worker 线程池。引擎按 MinGW32 泄漏策略不 delete，析构永远不执行；
//   交付线程被 TerminateThread 后无人能再 shutdown，worker 存活至 DLL
//   卸载，唤醒后踩已释放代码页（RX888Intf.dll_unloaded 崩溃根因）。
// v3（v4.186）：processReal 恢复并行——v4.182 串行化（libgomp spin-wait）是误判：
//   当时"速率不变"的结论来自 stat 公式假象（deliver/(usb*0.001024) 恒 93.8），
//   墙钟实测串行后掉到 43/s（应 700/s），交付仅 4%，音频碎片致解码全灭。
//   v3 改用 Win32 手动线程池（不用 libgomp，无 spin-wait）：每 worker 绑定
//   一个通道，主线程 SetEvent 唤醒全部 worker 并行处理，WaitAll 同步。
// v2 重构：v1 基准实测 8ch 超时 50.7 倍，v2 消灭 sin/cos 与环形取模两大开销
// 数据流：64M 实 ADC int16 → 每通道：
//   TabNCO 复混频（查表，64M 域） → PolyDecFIR 多相抽取 R=32（64M→2M）
//   → FracResampler v2 分数重采样（2M→48/96/192k）→ 复 IQ 输出
// 性能设计：
//   64M 域每样本：1 次表查（2×插值乘加）+ 4 乘 2 加（混频）≈ 10 flop
//   2M 域每输出：N=R*K=640 taps 供 FIR（每 32 样本 1 次输出 → 20 flop/输入样本均摊）
//   v4.186 实测验证：串行单核仅 43/s（需 ~700/s），8 worker 并行可满足
// 注意：混频输出 I/Q 分别进两个实 FIR（同一实系数）——等价于复 FIR，
//       且完全可被编译器自动向量化（无复数类型交错）
#pragma once

#include "polyphase.h"
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <windows.h>
#include <immintrin.h>

namespace skdsp {

enum OutRate { RATE_48K = 0, RATE_96K = 1, RATE_192K = 2 };
static const int OUT_RATES[3] = { 48000, 96000, 192000 };

struct ChannelCfg {
    double loHz;
};

// ---------------------------------------------------------------------------
// 单通道：64M 实 → 混频+抽取融合（MixDecFIR）→ 2M 复
// v2.1：NCO 折叠进 FIR 系数，消灭逐样本混频；ADC 实数据直接进延迟线
// ---------------------------------------------------------------------------
class ChannelDDC {
public:
    ChannelDDC() : inited_(false) {}

    void init(double loHz, int R, int K, double fcHz, double attenDb) {
        dec_.init(loHz, 64000000.0, R, K, fcHz, attenDb);
        lo_ = loHz;
        inited_ = true;
    }
    void setLo(double loHz) {
        // 重新折叠系数（LO 变更时整套系数重算，非热路径）
        int R = dec_.R(), K = dec_.K();
        double fc = dec_.fcHz();
        dec_.init(loHz, 64000000.0, R, K, fc, attenCur_);
        lo_ = loHz;
    }
    double lo() const { return lo_; }

    int process(const float* in, int nin, float* outI, float* outQ, int maxOut) {
        if (!inited_) return 0;
        return dec_.process(in, nin, outI, outQ, maxOut);
    }

private:
    MixDecFIR dec_;
    double lo_ = 0;
    double attenCur_ = 120.0;
    bool inited_;
};

// ---------------------------------------------------------------------------
// Win32 手动通道并行线程池（v4.186）：
//   - N worker 线程，worker i 固定处理通道 i（无任务抢夺、无队列锁）
//   - 主线程 startBatch() 设置输入指针 → SetEvent 唤醒全部 worker →
//     WaitForMultipleObjects 等全部完成 → 输出就绪
//   - 完全绕开 libgomp：无 spin-wait，空闲时 worker 在事件上睡眠（0% CPU）
// ---------------------------------------------------------------------------
class DdcChannelPool {
public:
    DdcChannelPool() : nWorkers_(0), stop_(false) {
                ZeroMemory(workers_, sizeof(workers_));
        ZeroMemory(doneEv_, sizeof(doneEv_));
        ZeroMemory(wakeEv_, sizeof(wakeEv_));
        memset((void*)doneEpoch_, 0, sizeof(doneEpoch_));   // volatile 数组不能 ZeroMemory
    }
    ~DdcChannelPool() { shutdown(); }

    // startBatch 前 thread 主线程调用：设置本轮任务参数
    struct Job {
        ChannelDDC* ch;
        FracResampler* fr;
        const float* in;
        int nin;
        float* mI; float* mQ; int mCap;
        float* oI; float* oQ; int oCap;
        int* outCount;
    };

    void start(int nWorkers) {
        if (nWorkers_ > 0) return;
        if (nWorkers < 1) nWorkers = 1;
        if (nWorkers > 8) nWorkers = 8;
        stop_ = false;
        running_ = 0;                    // v4.186-2: 已启动 worker 计数
        nextSlot_ = 0;                    // v4.186 修正：初始 0（InterlockedIncrement 先加后取），worker 占位 0-based
        for (int i = 0; i < nWorkers; ++i) {
            wakeEv_[i] = CreateEventA(nullptr, FALSE, FALSE, nullptr);  // auto-reset
            doneEv_[i] = CreateEventA(nullptr, TRUE,  FALSE, nullptr);  // manual-reset
            ResetEvent(doneEv_[i]);
            // Job 模板由 DDC_Engine::init 构建（每通道固定字段），池内不存 Job 数据
            workers_[i] = CreateThread(nullptr, 0, WorkerTramp, this, 0, nullptr);
        }
        nWorkers_ = nWorkers;
        // v4.186-2: 等所有 worker 就绪（running_ 计数轮询，5 秒兜底）
        DWORD t0 = GetTickCount();
        while (running_ < nWorkers && GetTickCount() - t0 < 5000) Sleep(1);
    }

    void shutdown() {
        if (nWorkers_ <= 0) return;
        stop_ = true;
        for (int i = 0; i < nWorkers_; ++i) {
            if (wakeEv_[i]) SetEvent(wakeEv_[i]);
        }
        WaitForMultipleObjects(nWorkers_, workers_, TRUE, 3000);
        for (int i = 0; i < nWorkers_; ++i) {
            if (workers_[i]) { CloseHandle(workers_[i]); workers_[i] = nullptr; }
            if (wakeEv_[i]) { CloseHandle(wakeEv_[i]); wakeEv_[i] = nullptr; }
            if (doneEv_[i]) { CloseHandle(doneEv_[i]); doneEv_[i] = nullptr; }
        }
        nWorkers_ = 0;
    }

    // v4.186-4: runBatch 拆分为 submit + wait 两段，支持流水线重叠——
    // 主线程 submit 后可预取/转换下一 USB 块（与 worker 并行 DDC 重叠），
    // 再 wait 等待全部通道完成。提交后等待前不得调用 setChannelLo。
    void submitBatch(Job* jobs, int n) {
        if (nWorkers_ <= 0 || n <= 0) {   // 无池：退化为串行（同步完成）
            for (int i = 0; i < n; ++i) runOne(jobs[i]);
            return;
        }
        if (n > nWorkers_) n = nWorkers_;
        batchJobs_ = jobs;                 // 先写任务（x86 TSO：先于 epoch 自增可见）
        batchN_ = n;
        LONG myEpoch = InterlockedIncrement(&epoch_);
        curEpoch_ = myEpoch;               // v4.186-4: 保存本批 epoch 供 waitBatch
        for (int i = 0; i < n; ++i) ResetEvent(doneEv_[i]);
        for (int i = 0; i < n; ++i) SetEvent(wakeEv_[i]);   // 唤醒睡眠中的 worker
        // 不等待，立即返回——主线程去做重叠工作
    }

    // v4.196: 混合同步——短自旋（快路径零内核开销：实机实测 wait 仅 ~1.2µs/批，
    // 2 万次轻读自旋 ≈ 100µs 窗口完全覆盖）→ 未完成则 WaitForMultipleObjects
    // WaitAll 等 batchN_ 个 manual-reset doneEv_（慢路径内核休眠）。CDC 归因
    //（2026-11-06 实机 QPC 263 样本）：wait 段 332µs/块为纯自旋烧核，是主线程
    // 唯一真实烧核等待点（prefetch 段在 ringbuffer 内早已是 CV 休眠）；本改动
    // 预期整机进程 CPU 60% → ~57%。唤醒延迟 ~10-50µs << 1.024ms 块周期，
    // 且 prefetch 段还有 ~565µs USB 等待缓冲，实时性无损。
    // 事件序：worker 先写 doneEpoch_ 后 SetEvent（锁前缀 + 内核调用双重栅栏），
    // 主线程见事件置位则 epoch 必可见；WAIT_TIMEOUT/提前返回后一律回查 epoch
    // 确认，事件不作为完成依据。已知的极端边界（worker 在 epoch 写与 SetEvent
    // 两条指令间被长时间抢占，陈旧 SetEvent 污染后续批次事件）后果为有界内核
    // 速轮询直至该 worker 追平，自愈无丢失，概率可忽略。worker 侧回查逻辑
    // 不变：完成后立即看新批（epoch 已增则继续做），不额外休眠。
    void waitBatch() {
        if (nWorkers_ <= 0 || batchN_ <= 0) return;   // 串行退化：submit 已同步完成
        LONG myEpoch = curEpoch_;
        // 快路径：短自旋（结构同 ringbuffer spin_count_boost：轻读为主，偶发 pause）
        for (int spin = 0; spin < 20000; ++spin) {
            bool all = true;
            for (int i = 0; i < batchN_; ++i) {
                if (doneEpoch_[i] < myEpoch) { all = false; break; }
            }
            if (all) return;
            if ((spin & 1023) == 1023) _mm_pause();   // 超线程友好
        }
        // 慢路径：内核 WaitAll（doneEv_ 为 manual-reset，submitBatch 已 ResetEvent）；
        // 5 秒超时兑底：worker 全部停摆时每 5s 重查一次（与旧版无限等语义一致）
        for (;;) {
            bool all = true;
            for (int i = 0; i < batchN_; ++i) {
                if (doneEpoch_[i] < myEpoch) { all = false; break; }
            }
            if (all) return;
            DWORD w = WaitForMultipleObjects((DWORD)batchN_, doneEv_, TRUE, 5000);
            if (w == WAIT_FAILED) Sleep(1);   // 句柄异常退避，防忙循环
        }
    }

    // 老同步接口（= submit + wait）：processReal/stress 测试使用
    void runBatch(Job* jobs, int n) {
        submitBatch(jobs, n);
        waitBatch();
    }

private:
    static void runOne(Job& j) {
        int nM = j.ch->process(j.in, j.nin, j.mI, j.mQ, j.mCap);
        int m = j.fr->process(j.mI, j.mQ, nM, j.oI, j.oQ, j.oCap);
        *j.outCount = m;
    }

    static DWORD WINAPI WorkerTramp(LPVOID param);
    void workerLoop(int idx) {
        InterlockedIncrement(&running_);
        while (!stop_) {
            // v4.186-2c: 完成一批后立即回查新批（无事件往返延迟）。
            // 高频时 worker 连续处理批次；低频（ADC 无新数据）才进事件等待休眠。
            // v4.186-5a: epoch 读用 volatile（非原子）——主线程 InterlockedIncrement
            // 写 epoch 是原子的，worker 只需读到最新值；避免 Interlocked 锁前缀与
            // 其他 worker 抢内存带宽（8 worker 高频轮询时开销可观）。
            LONG myEpoch = epoch_;   // volatile 读（主线程原子写，worker 读最新即可）
            if (doneEpoch_[idx] >= myEpoch) {
                // 无新任务 → 事件等待（唤醒或超时回检）
                DWORD w = WaitForSingleObject(wakeEv_[idx], 1000);
                if (stop_) break;
                continue;
            }
            // v4.186-7: 双检——快照后若 epoch 已被主线程推进（提交了新批），
            // 说明 jobs_ 的 in 已更新，本快照失效，必须重读最新 epoch 后再做。
            // 否则 worker 会拿新批数据却记成旧批号（数据错乱）。
            if (epoch_ != myEpoch) continue;
            // 有任务：执行本通道
            if (idx < batchN_) {
                runOne(batchJobs_[idx]);
            }
            // v4.186-7: 完成只写 doneEpoch_（主线程快路径自旋读）
            // v4.196: 完成后 SetEvent(doneEv_)——主线程 waitBatch 慢路径 WaitAll
            // 的唤醒源。先写 epoch 后 SetEvent（栅栏序），submitBatch 处已有
            // ResetEvent 清理跨批残留。代价 ~1µs/次 × 8ch × 93.7/s ≈ 0.1% 单核。
            InterlockedExchange(&doneEpoch_[idx], myEpoch);
            SetEvent(doneEv_[idx]);
        }
    }

    // WorkerTramp 入口：worker 启动后注册自己的槽位（原子），再进入循环。
    // CreateThread 无法直接传 idx，worker 启动后用 Interlocked 原子占坑。
    int workerEntry() {
        LONG idx = InterlockedIncrement(&nextSlot_) - 1;  // 0-based
        if (idx < 0 || idx > 7) return 0;   // 防御（越界槽位直接退出）
        slot_[idx] = GetCurrentThreadId();  // 调试可读
        workerLoop((int)idx);
        return 0;
    }

public:
    // 仅供 WorkerTramp 调用
    int workerEntryPub() { return workerEntry(); }

private:
    // ---- 数据成员（v4.186 修正：必须全部在类内，曾误被切到类外）----
    volatile LONG nextSlot_ = 0;       // worker 槽位分配器（start 时重置）
    volatile LONG running_ = 0;        // v4.186-2: 已启动 worker 计数
    DWORD slot_[8] = {0};              // 调试用：各槽位的线程 ID
    int nWorkers_ = 0;
    HANDLE workers_[8] = {0};
    HANDLE wakeEv_[8] = {0};   // auto-reset：一次唤醒一个 worker
    HANDLE doneEv_[8] = {0};   // manual-reset：主线程 WaitAll
    Job*  batchJobs_ = nullptr;
    int   batchN_ = 0;
    volatile LONG doneEpoch_[8] = {0}; // v4.186-2: 每 worker 已完成的最大 epoch
    volatile LONG epoch_ = 0;          // v4.186-2: 批次号
    volatile LONG curEpoch_ = 0;       // v4.186-4: 当前批 epoch（submit 保存、wait 使用）
    volatile bool stop_ = false;
};

inline DWORD WINAPI DdcChannelPool::WorkerTramp(LPVOID param) {
    DdcChannelPool* p = (DdcChannelPool*)param;
    // worker 索引：通过 thread 在 workers_ 数组中的句柄匹配确定
    // （CreateThread 返回句柄后 worker 可能已运行，因此用槽位注册表）
    // 简化：用 TLS 不行（CreateThread 无法传 idx），改用原子槽位分配
    // 注意：start() 中创建顺序即槽位顺序，但并发启动可能乱序 → 用槽位注册
    return p->workerEntry();
}

// ---------------------------------------------------------------------------
// DDC 引擎：64M 实输入 → 多通道复 IQ（48k/96k/192k）
// ---------------------------------------------------------------------------
class DDC_Engine {
public:
    DDC_Engine() : inited_(false), nCh_(0) {}
    ~DDC_Engine() { pool_.shutdown(); }

    void init(OutRate rate, int nCh, const ChannelCfg* cfgs) {
        rate_ = rate;
        nCh_ = nCh;
        fsOut_ = OUT_RATES[rate_];
        fsMid_ = 2000000.0;
        R_ = 32;   // 64M → 2M

        chans_.clear();
        fracs_.clear();
        outI_.clear();
        outQ_.clear();
        outCount_.clear();
        tmpI_.clear();
        tmpQ_.clear();

        for (int c = 0; c < nCh_; ++c) {
            ChannelDDC ch;
            // K=9 → N=288 taps。Kaiser 120dB 过渡带需求 ≈264 taps（见推导），
            // 288 留 9% 余量；阻带起点 ≈110k+1.74M < 2M 抽取镜像点，安全
            ch.init(cfgs[c].loHz, R_, (getenv("DDC_K") ? atoi(getenv("DDC_K")) : 9), 110000.0, 120.0);
            chans_.push_back(ch);

            int L, M;
            switch (rate_) {
                case RATE_48K:  L = 3;  M = 125; break;
                case RATE_96K:  L = 6;  M = 125; break;
                case RATE_192K: L = 12; M = 125; break;
                default:        L = 6;  M = 125; break;
            }
            FracResampler fr;
            double fcNorm = (0.48 * fsOut_) / fsMid_;
            fr.init(L, M, 330, fcNorm, 120.0);
            fracs_.push_back(fr);

            outI_.push_back(std::vector<float>(65536 / R_ + 64));
            outQ_.push_back(std::vector<float>(65536 / R_ + 64));
            // v4.187: 第二组输出缓冲（ping-pong）——collect 上一批输出与
            // 当前批 DDC 计算重叠，消除主线程 collect 串行尾巴
            outI_B_.push_back(std::vector<float>(65536 / R_ + 64));
            outQ_B_.push_back(std::vector<float>(65536 / R_ + 64));
            // 每通道独立的中间混频缓冲：OpenMP 并行时不得共享
            tmpI_.push_back(std::vector<float>(65536 / R_ + 64));
            tmpQ_.push_back(std::vector<float>(65536 / R_ + 64));
            outCount_.push_back(0);
            outCount_B_.push_back(0);
        }
        inited_ = true;

        // v4.186: 预构建 Job 模板（固定字段：通道/缓冲指针）+ 启动通道并行线程池
        jobs_.resize(nCh_);
        for (int c = 0; c < nCh_; ++c) {
            jobs_[c].ch   = &chans_[c];
            jobs_[c].fr   = &fracs_[c];
            jobs_[c].in   = nullptr;
            jobs_[c].nin  = 0;
            jobs_[c].mI   = tmpI_[c].data();
            jobs_[c].mQ   = tmpQ_[c].data();
            jobs_[c].mCap = (int)tmpI_[c].size();
            jobs_[c].oI   = outI_[c].data();
            jobs_[c].oQ   = outQ_[c].data();
            jobs_[c].oCap = (int)outI_[c].size();
            jobs_[c].outCount = &outCount_[c];
        }
        bufParity_ = 0;   // v_.187: 初始用 A 组
        pool_.shutdown();
        pool_.start(nCh_ < 8 ? nCh_ : 8);
    }

    void setChannelLo(int c, double loHz) {
        if (c < 0 || c >= nCh_) return;
        chans_[c].setLo(loHz);
    }

    // 输入 nin 个 64M 实样本，驱动所有通道。
    // v4.186: 恢复并行——v4.182 的串行化基于 stat 公式假象（93.8 blocks/s
    // 是 deliver/(usb*0.001024) 恒定值，与墙钟无关），墙钟实测串行仅 43/s，
    // 音频交付率 4%，CW 解码全灭。现在每 worker 固定绑定一个通道：主线程
    // 只更新每轮变化的 in/nin，SetEvent 唤醒全部 worker，WaitAll 同步。
    // 调用方保证：processReal 与 setChannelLo 同线程串行（worker 空闲时
    // 在事件上睡眠，不持有通道引用，主线程改系数安全）。
    void processReal(const float* in, int nin) {
        if (!inited_) return;
        for (int c = 0; c < nCh_; ++c) {
            jobs_[c].in  = in;
            jobs_[c].nin = nin;
        }
        pool_.runBatch(jobs_.data(), nCh_);
    }

    // v4.186-4: 流水线接口——submit 后 worker 并行计算，主线程可做重叠工作
    //（预取下一 USB 块 + applyRe 转换），wait 等全部通道完成。
    // 注意：submit 与 wait 之间不得调用 setChannelLo（worker 正在读 jobs_）。
    // v4.187-8: 双缓冲输出——submit 切换输出组（parity 翻转），主线程在
    // wait 前 collect 上一组（上一批）输出，与当前批 worker 计算重叠。
    void processSubmit(const float* in, int nin) {
        if (!inited_) return;
        int parity = bufParity_;   // 当前批写入组
        for (int c = 0; c < nCh_; ++c) {
            jobs_[c].in  = in;
            jobs_[c].nin = nin;
            if (parity) {
                jobs_[c].oI = outI_B_[c].data();
                jobs_[c].oQ = outQ_B_[c].data();
                jobs_[c].oCap = (int)outI_B_[c].size();
                jobs_[c].outCount = &outCount_B_[c];
            } else {
                jobs_[c].oI = outI_[c].data();
                jobs_[c].oQ = outQ_[c].data();
                jobs_[c].oCap = (int)outI_[c].size();
                jobs_[c].outCount = &outCount_[c];
            }
        }
        pool_.submitBatch(jobs_.data(), nCh_);
    }

    void processWait() {
        if (!inited_) return;
        pool_.waitBatch();
    }

    // v4.190: 主动关闭通道线程池（幂等）。退出路径（交付线程终止/提前 return）
    // 必须显式调用——引擎按 MinGW32 泄漏策略不 delete，析构永远不会执行，
    // 不关闭则 worker 线程存活至 DLL 卸载，唤醒后踩已释放代码页（崩溃根因）。
    void shutdownPool() {
        pool_.shutdown();
    }

    // v4.187-8: 翻转输出组——submit 前调用（切换 worker 写入组）
    void flipParity() {
        bufParity_ ^= 1;
    }

    // v4.187-8: 读取上一批（非当前写入组）输出。返回输出样本数。
    int collectPrev(int c, float*& oI, float*& oQ) {
        int parity = bufParity_ ^ 1;   // 上一批写入组
        if (parity) {
            oI = outI_B_[c].data();
            oQ = outQ_B_[c].data();
            return outCount_B_[c];
        } else {
            oI = outI_[c].data();
            oQ = outQ_[c].data();
            return outCount_[c];
        }
    }

    // v4.187-8: 读取当前批（最近完成的写入组）输出
    int availCur(int c, float*& oI, float*& oQ) {
        int parity = bufParity_;
        if (parity) {
            oI = outI_B_[c].data();
            oQ = outQ_B_[c].data();
            return outCount_B_[c];
        } else {
            oI = outI_[c].data();
            oQ = outQ_[c].data();
            return outCount_[c];
        }
    }

    int available(int c) const { return outCount_[c]; }
    float* outI(int c) { return outI_[c].data(); }
    float* outQ(int c) { return outQ_[c].data(); }
    int channels() const { return nCh_; }
    double fsOut() const { return fsOut_; }

private:
    bool inited_;
    OutRate rate_;
    int nCh_;
    double fsOut_, fsMid_;
    int R_;
    std::vector<ChannelDDC> chans_;
    std::vector<FracResampler> fracs_;
    std::vector<std::vector<float>> outI_, outQ_;
    std::vector<std::vector<float>> outI_B_, outQ_B_;   // v4.187-8: ping-pong B 组
    std::vector<int> outCount_, outCount_B_;
    std::vector<std::vector<float>> tmpI_, tmpQ_;
    DdcChannelPool pool_;                    // v4.186: 通道并行线程池
    std::vector<DdcChannelPool::Job> jobs_;  // v4.186: 每通道 Job 模板
    int bufParity_ = 0;                      // v4.187-8: 当前批写入组（0=A,1=B）
};

} // namespace skdsp
