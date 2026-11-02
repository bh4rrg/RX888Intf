// polyphase.h v2 - 高速 DSP 原语（用于新 RX888 SkimSrv 驱动）
// v2 重构动机（基准实测：v1 引擎 8ch 超时 50.7 倍，必须消灭两大开销）：
//   1) sin/cos 超越函数（每样本 2 次）→ 查表 NCO（2^16 复正弦表 + 相位累加）
//   2) 环形缓冲卷积每 tap 取模 → 线性双缓冲延迟线（滑窗只在块边界 memcpy 一次）
// 架构：
//   TabNCO      — 查表复 NCO（相位累加 + 表索引，2^16 点，线性插值可选）
//   PolyDecFIR  — 整数抽取多相 FIR（R 相位 × K taps，线性延迟线，实/复输入通用）
//   FracResampler v2 — 分数重采样 L/M（多相，线性延迟线，复输入）
//   PolyPhaseMixerDec — 复混频 + 多相抽取一体化（64M→2M 每通道主力单元）
// 设计原则：无任何 64M 域逐样本 sin/cos；所有 FIR 输出每样本 K taps 线性访问
#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace skdsp {

// ---------------------------------------------------------------------------
// Kaiser 窗工具（与 v1 相同，已验证）
// ---------------------------------------------------------------------------
static inline double bessel_i0(double x) {
    double sum = 1.0, term = 1.0;
    for (int k = 1; k <= 24; ++k) {
        term *= (x / 2.0) / k;
        term *= (x / 2.0) / k;
        sum += term;
    }
    return sum;
}

static inline double kaiser_beta(double atten_db) {
    if (atten_db <= 21.0)  return 0.0;
    if (atten_db > 50.0)   return 0.1102 * (atten_db - 8.7);
    return 0.5842 * std::pow(atten_db - 21.0, 0.4) + 0.07886 * (atten_db - 21.0);
}

inline void design_lpf(float* h, int N, double fc, double atten_db) {
    const double beta = kaiser_beta(atten_db);
    const int M = N - 1;
    double sum = 0.0;
    for (int n = 0; n < N; ++n) {
        double x = 2.0 * fc * (n - M / 2.0);
        double s = (x == 0.0) ? 2.0 * fc : std::sin(M_PI * x) / (M_PI * x);
        double a = 2.0 * n / M - 1.0;
        double w = bessel_i0(beta * std::sqrt(std::max(0.0, 1.0 - a * a))) / bessel_i0(beta);
        double v = s * w;
        h[n] = (float)v;
        sum += v;
    }
    for (int n = 0; n < N; ++n) h[n] = (float)(h[n] / sum);
}

// ---------------------------------------------------------------------------
// TabNCO — 查表复 NCO
// 相位累加器 64 位定点（单位：2^-64 圈），表 2^TBL 点 float (cos,sin)
// 频率精度：64 位相位 → 任意可表示频率误差 < 1e-16 圈/样本，远超需求
// 谱纯度：表直接截断的杂散约 -96dBc（2^16 点 float 表），
//         加一级线性插值 → 杂散 <-140dBc，且成本仅 1 次额外乘加
// 用法：每样本 mixOne(x) 返回 x * e^{-j2πf n/fs}（下变频）
// ---------------------------------------------------------------------------
class TabNCO {
public:
    static const int TBL_BITS = 16;
    static const int TBL_SIZE = 1 << TBL_BITS;   // 65536

    TabNCO() : inited_(false) {}

    void init(double freqHz, double sampleRate) {
        ensureTable();
        freq_ = freqHz;
        // 每样本相位增量（圈）× 2^64；fmod 防 >1 圈时的 UB（双精度→uint64 超范围）
        double t = std::fmod(freqHz / sampleRate, 1.0);
        if (t < 0) t += 1.0;
        inc_ = (uint64_t)(t * 18446744073709551616.0);
        phase_ = 0;
        inited_ = true;
    }

    void reset() { phase_ = 0; }

    // 单样本复混频（下变频：乘 e^{-jωn}）
    // 输入实数 x，输出 (x*cos, -x*sin)
    inline void mixOne(float x, float& outI, float& outQ) {
        uint32_t idx = (uint32_t)(phase_ >> (64 - TBL_BITS));
        uint32_t frac = (uint32_t)(phase_ >> (64 - TBL_BITS - 15)) & 0x7FFF;
        const float* e0 = &table_[idx << 1];
        const float* e1 = &table_[((idx + 1) & (TBL_SIZE - 1)) << 1];
        // 线性插值（frac ∈ [0, 32767] / 32768）
        float c = e0[0] + (e1[0] - e0[0]) * (frac * (1.0f / 32768.0f));
        float s = e0[1] + (e1[1] - e0[1]) * (frac * (1.0f / 32768.0f));
        outI = x * c;
        outQ = -x * s;
        phase_ += inc_;
    }

    // 只取当前相位（不推进）——供批量混频用
    inline void peek(float& c, float& s) const {
        uint32_t idx = (uint32_t)(phase_ >> (64 - TBL_BITS));
        uint32_t frac = (uint32_t)(phase_ >> (64 - TBL_BITS - 15)) & 0x7FFF;
        const float* e0 = &table_[idx << 1];
        const float* e1 = &table_[((idx + 1) & (TBL_SIZE - 1)) << 1];
        c = e0[0] + (e1[0] - e0[0]) * (frac * (1.0f / 32768.0f));
        s = e0[1] + (e1[1] - e0[1]) * (frac * (1.0f / 32768.0f));
    }
    inline void step() { phase_ += inc_; }

    double freq() const { return freq_; }

    // 供 MixDecFIR 等融合单元复用查表（公开静态表）
    static void ensureTablePub() { ensureTable(); }
    static const float* table() { return table_.data(); }
    static const int TBL_BITS_PUB = TBL_BITS;
    static const int TBL_SIZE_PUB = TBL_SIZE;

private:
    static void ensureTable() {
        if (!table_.empty()) return;
        table_.resize((size_t)TBL_SIZE * 2);
        for (int i = 0; i < TBL_SIZE; ++i) {
            double a = 2.0 * M_PI * i / TBL_SIZE;
            table_[(size_t)i * 2]     = (float)std::cos(a);
            table_[(size_t)i * 2 + 1] = (float)std::sin(a);
        }
    }
    static std::vector<float> table_;   // 交错 cos/sin
    uint64_t inc_ = 0, phase_ = 0;
    double freq_ = 0;
    bool inited_;
};

// ---------------------------------------------------------------------------
// PolyDecFIR — 整数 R 抽取 FIR（线性双缓冲延迟线）
//   实输入版本：64M 实 → 每通道复基带 → 抽取 R=32 → 2M 复
// 实现口径（等价于标准抽取卷积）：
//   对输入序号 n（0 基），每当 (n+1) % R == 0 产生输出，
//   输出 = Σ_{j=0..N-1} h[j]·x[n-j]，h[j] 设计在 64M 域（N = R*K taps）
//   即：每个输出使用全部 N 个系数（多相分解的 p≠0 分支不能丢弃，
//   否则丢弃的输入样本贡献（混叠）会直接进入通带）。
// 延迟线保存最近 N 个复样本（双倍容量 2N 线性双缓冲，回卷时 memcpy 一次）
// ---------------------------------------------------------------------------
class PolyDecFIR {
public:
    PolyDecFIR() : inited_(false) {}

    void init(int R, int K, double fcNorm, double attenDb) {
        R_ = R; K_ = K; N_ = R * K;
        std::vector<float> h(N_);
        design_lpf(h.data(), N_, fcNorm, attenDb);
        // 预翻转存正序：hr[j] = h[N-1-j]，内积循环正向连续访问，
        // 利于编译器自动向量化（SIMD 4-wide float）
        hr_.resize(N_);
        for (int j = 0; j < N_; ++j) hr_[j] = h[N_ - 1 - j];
        // 双倍延迟线：窗口恒线性，无逐样本取模
        delayI_.assign(2 * N_, 0.0f);
        delayQ_.assign(2 * N_, 0.0f);
        w_ = N_;
        total_ = 0;
        inited_ = true;
    }

    void reset() {
        std::fill(delayI_.begin(), delayI_.end(), 0.0f);
        std::fill(delayQ_.begin(), delayQ_.end(), 0.0f);
        w_ = N_; total_ = 0;
    }

    // 复输入（已混频）批量处理：每 R 个输入产生 1 个输出
    // 输出 y[m] = Σ_{j=0..N-1} h[j]·x[mR - j]   （x 为混频后复信号，m 为输出序号）
    // 双缓冲延迟线：每个 x 样本写入 delay[w] 与 delay[w-N]（线性），
    // 回卷时 memcpy 后半到前半；输出时窗 [w-N, w) 恒线性。
    int process(const float* inI, const float* inQ, int nin,
                float* outI, float* outQ, int maxOut) {
        int nout = 0;
        const int N = N_;
        const int R = R_;
        float* dI = delayI_.data();
        float* dQ = delayQ_.data();
        for (int i = 0; i < nin; ++i) {
            // 写入延迟线（线性双缓冲，2N 容量）
            dI[w_] = inI[i];
            dQ[w_] = inQ[i];
            dI[w_ - N] = inI[i];
            dQ[w_ - N] = inQ[i];
            ++w_;
            if (w_ == 2 * N) {
                memcpy(dI, dI + N, N * sizeof(float));
                memcpy(dQ, dQ + N, N * sizeof(float));
                w_ = N;
            }
            ++total_;
            if ((total_ % R) == 0) {   // R=32，% 可用迭代维护
                if (nout >= maxOut) break;
                // 窗口 [w_-N, w_)：pI[0]=x[m-N+1] … pI[N-1]=x[m]
                // y[m] = Σ_{j=0..N-1} h[j]·x[m-j] = Σ_j hr[j]·pI[j]（正序连续）
                const float* pI = dI + w_ - N;
                const float* pQ = dQ + w_ - N;
                const float* hr = hr_.data();
                float accI = 0.0f, accQ = 0.0f;
                for (int j = 0; j < N; ++j) {
                    accI += pI[j] * hr[j];
                    accQ += pQ[j] * hr[j];
                }
                outI[nout] = accI;
                outQ[nout] = accQ;
                ++nout;
            }
        }
        return nout;
    }

private:
    int R_ = 1, K_ = 1, N_ = 1;
    std::vector<float> hr_;   // 预翻转系数（正序内积）
    std::vector<float> delayI_, delayQ_;
    int w_ = 0;
    long long total_ = 0;
    bool inited_ = false;
};

// ---------------------------------------------------------------------------
// MixDecFIR — 混频+整数抽取一体化（实 ADC 输入，64M→2M 每通道主力单元）
// 原理（频率偏移定理折叠混频进系数，消灭逐样本 NCO）：
//   y[m] = Σ_{j=0..N-1} h[j]·s[mR-j]·e^{-jω(mR-j)}
//        = e^{-jωmR} · Σ_{j=0..N-1} h[j]·e^{+jωj}·s[mR-j]
//   预计算复系数 c[j] = h[j]·e^{+jωj}（静态，N 长度，实输入只需 2 套实系数 cI/cQ）
//   每输出做 I/Q 两套实内积（均可向量化），最后乘每输出旋转因子
//   rot[m] = e^{-jωmR}（64 位相位累加 + 查表，每输出 1 次）。
//
// v4.194 消灭原位惩罚（本会话 pipeline_bench 双机实测驱动，见 daily-log）：
//   v4.193 的 576 样本双倍延迟线需把输入按 R 组 memcpy 进来才读，FIR 读到的
//   最新 32 样本是几 µs 前刚写入的（store-to-load 转发/新鲜数据读惩罚），
//   8559U 8T 实测 mixdec=347.7µs vs split（先写完再算）=244.2µs，惩罚 ~104µs/块
//   占 full 的 27%；AVX2 加宽向量在 8559U 反而 2.4 倍负优化，路线堵死。
//   v4.194 直读方案：FIR 窗口跨块回看仅 N-R 样本（N=288, R=32），用独立
//   tail_ 缓冲保存逻辑流最后 N-1 样本（块末一次 memcpy，距下次读取隔一个
//   块周期，数据已冷），主路径 FIR 直接在输入大数组上做窗口内积，
//   整个 memcpy 进延迟线 + 回卷 memcpy 消灭。生产块 65536=2048R 时，
//   前 8 个输出窗口跨 tail_/in 边界（双段求和），其余 2040 个输出直读。
// ---------------------------------------------------------------------------
class MixDecFIR {
public:
    MixDecFIR() : inited_(false) {}

    void init(double loHz, double fsIn, int R, int K, double fcHz, double attenDb) {
        R_ = R; K_ = K; N_ = R * K;
        fsIn_ = fsIn;
        fcHz_ = fcHz; attenDb_ = attenDb;
        // 1) 设计实 LPF（64M 域）
        std::vector<double> h(N_);
        {
            std::vector<float> hf(N_);
            design_lpf(hf.data(), N_, fcHz / fsIn, attenDb);
            for (int j = 0; j < N_; ++j) h[j] = hf[j];
        }
        // 2) 折叠混频进系数：c[j] = h[j]·e^{+jωj}，ω=2π·lo/fs
        //    窗口数组 p[j'] 存 s[mR-N+1+j']（j'=0 最老），对应 tap j = N-1-j'
        //    → 正序复系数 cwin[j'] = h[N-1-j']·e^{+jω(N-1-j')}
        cI_.resize(N_); cQ_.resize(N_);
        const double w0 = 2.0 * M_PI * loHz / fsIn;
        for (int jp = 0; jp < N_; ++jp) {
            int j = N_ - 1 - jp;
            cI_[jp] = (float)(h[j] * std::cos(w0 * j));
        }
        for (int jp = 0; jp < N_; ++jp) {
            int j = N_ - 1 - jp;
            cQ_[jp] = (float)(h[j] * std::sin(w0 * j));
        }
        // 3) v4.194: 跨块历史缓冲 tail_（逻辑流最后 N-1 样本，init 全 0 =
        //    虚拟零历史，与旧版延迟线 0 填充形态一致）；kToNext_ = 距下次
        //    发射还需消费的样本数（init = R，每消费 R 个发射 1 次）
        tail_.assign(N_ - 1, 0.0f);
        kToNext_ = R_;        // 4) 每输出旋转相位累加器：rot[m] = e^{-jω·t_m}，t_m = mR−1（首输出时刻 R−1）
        //    ★ 关键：lo×R/fs 可能 >1 圈，double×2^64 超 uint64 范围的转换是 UB
        //      （32 位 MinGW 返回垃圾值，TEST1/2 频率全错的根因）。
        //      必须先 fmod 取小数圈再转 uint64。
        double incTurns = std::fmod(loHz * R / fsIn, 1.0);
        if (incTurns < 0) incTurns += 1.0;
        rotInc_ = (uint64_t)(incTurns * 18446744073709551616.0);
        double startTurns = std::fmod(-(loHz * (R - 1) / fsIn), 1.0);
        if (startTurns < 0) startTurns += 1.0;
        rotStart_ = (uint64_t)(startTurns * 18446744073709551616.0);
        rotPhase_ = rotStart_;
        TabNCO::ensureTablePub();
        inited_ = true;
    }

    void reset() {
        std::fill(tail_.begin(), tail_.end(), 0.0f);
        kToNext_ = R_;
        rotPhase_ = rotStart_;
    }

    int R() const { return R_; }
    int K() const { return K_; }
    int N() const { return N_; }
    double fcHz() const { return fcHz_; }

    // 实 ADC 输入批量处理：每 R 个输入产生 1 个复输出
    // v4.194 直读版：FIR 窗口直接在输入大数组上滑动做内积，消灭 v4.193 的
    //   "按 R 组 memcpy 进 576 样本延迟线"（原位惩罚根因：FIR 读到 5µs 前刚
    //   memcpy 的最新 32 样本，store-to-load 转发/新鲜数据读惩罚，8559U 8T 实测
    //   mixdec 段 347.7µs vs split 244.2µs，惩罚占 full 的 27%）。
    // 节拍语义（与 v4.193 延迟线版逐点对齐）：
    //   ①每消费第 R 个逻辑样本发射一次（kToNext_ 跨调用连续，自动继承截断风暴
    //     下的相位连续性）；发射时窗口 = [k-N, k-1]（k = 本调用内已消费数），
    //     负坐标读 tail_（i+N-1，即虚拟零历史/上块尾部）——首输出在 k=R
    //     （256 虚拟零 + 32 样本），与旧版延迟线逐点一致
    //   ②截断返回：窗口未完全落在已写入数据内的输出跳过不补发（nout 保持，
    //     剩余输入不消费），同 v4.193 "nout>=maxOut 提前 return" 语义
    //   ③块末 tail_ 重写为逻辑流最后 N-1 样本（一次 memcpy，块间冷数据）
    // 直读窗口的求和树与 v4.193 完全一致（bit-exact）；跨界窗口为双段求和，
    // 数值差异为舍入级（verify_v4194 以最大相对偏差<1e-4 + Goertzel 音频保真
    // 判据验收，同 v4.193 口径）。
    int process(const float* in, int nin,
                float* outI, float* outQ, int maxOut) {
        int nout = 0;
        const int N = N_, R = R_;
        const float* cI = cI_.data();
        const float* cQ = cQ_.data();
        const float* t = tail_.data();
        // 每输出一次的发射逻辑：k = 本调用内已消费数（窗口右端+1）
        // 窗口（调用内坐标）[k-N, k-1]：负坐标 c 映射 tail_ 索引 t[H+c]（H=N-1，
        // t[H-1] 即调用前一逻辑样本），即 t[k-1+jp]；正坐标直接读 in
        auto emitWindow = [&](int k) {
            const int S = k - N;         // 窗口起点（调用内坐标，可为负）
            float accI = 0.f, accQ = 0.f;
            if (S >= 0) {
                // 窗口完全在当前输入块内：直读大数组（与 v4.193 同求和树）
                const float* p = in + S;
                float aI0 = 0.f, aI1 = 0.f, aI2 = 0.f, aI3 = 0.f;
                float aQ0 = 0.f, aQ1 = 0.f, aQ2 = 0.f, aQ3 = 0.f;
                int jp = 0;
                for (; jp + 4 <= N; jp += 4) {
                    aI0 += p[jp  ] * cI[jp  ]; aQ0 += p[jp  ] * cQ[jp  ];
                    aI1 += p[jp+1] * cI[jp+1]; aQ1 += p[jp+1] * cQ[jp+1];
                    aI2 += p[jp+2] * cI[jp+2]; aQ2 += p[jp+2] * cQ[jp+2];
                    aI3 += p[jp+3] * cI[jp+3]; aQ3 += p[jp+3] * cQ[jp+3];
                }
                accI = (aI0 + aI1) + (aI2 + aI3);
                accQ = (aQ0 + aQ1) + (aQ2 + aQ3);
                for (; jp < N; ++jp) { accI += p[jp] * cI[jp]; accQ += p[jp] * cQ[jp]; }
            } else {
                // 窗口跨 tail_/in 边界：双段求和（仅前 8 个输出，非热路径）
                //   坐标 j∈[S,0)  → tail_[H+j]（jp=j-S → t[k-1+jp]）
                //   坐标 j∈[0,k)  → in[j]（jp=j-S → cI[nT+j0]，nT=-S）
                const int nT = -S;       // 窗口内 tail_ 样本数（0<nT<N）
                const int nB = k;        // 窗口内 in 样本数（=k≥R>0）
                for (int jp = 0; jp < nT; ++jp) {
                    accI += t[k - 1 + jp] * cI[jp];
                    accQ += t[k - 1 + jp] * cQ[jp];
                }
                float aI0 = 0.f, aI1 = 0.f, aI2 = 0.f, aI3 = 0.f;
                float aQ0 = 0.f, aQ1 = 0.f, aQ2 = 0.f, aQ3 = 0.f;
                int j0 = 0;
                for (; j0 + 4 <= nB; j0 += 4) {
                    aI0 += in[j0  ] * cI[nT+j0  ]; aQ0 += in[j0  ] * cQ[nT+j0  ];
                    aI1 += in[j0+1] * cI[nT+j0+1]; aQ1 += in[j0+1] * cQ[nT+j0+1];
                    aI2 += in[j0+2] * cI[nT+j0+2]; aQ2 += in[j0+2] * cQ[nT+j0+2];
                    aI3 += in[j0+3] * cI[nT+j0+3]; aQ3 += in[j0+3] * cQ[nT+j0+3];
                }
                accI += (aI0 + aI1) + (aI2 + aI3);
                accQ += (aQ0 + aQ1) + (aQ2 + aQ3);
                for (; j0 < nB; ++j0) {
                    accI += in[j0] * cI[nT + j0];
                    accQ += in[j0] * cQ[nT + j0];
                }
            }
            // 旋转因子 rot[m] = e^{-jωmR}：64 位相位查表
            uint32_t idx = (uint32_t)(rotPhase_ >> (64 - TabNCO::TBL_BITS_PUB));
            uint32_t frac = (uint32_t)(rotPhase_ >> (64 - TabNCO::TBL_BITS_PUB - 15)) & 0x7FFF;
            const float* tbl = TabNCO::table();
            const float* e0 = &tbl[idx << 1];
            const float* e1 = &tbl[((idx + 1) & (TabNCO::TBL_SIZE_PUB - 1)) << 1];
            float rc = e0[0] + (e1[0] - e0[0]) * (frac * (1.0f / 32768.0f));
            float rs = e0[1] + (e1[1] - e0[1]) * (frac * (1.0f / 32768.0f));
            // 复数乘法：(accI + j·accQ)·(rc - j·rs)   [下变频取负角]
            outI[nout] = accI * rc + accQ * rs;
            outQ[nout] = accQ * rc - accI * rs;
            ++nout;
            rotPhase_ += rotInc_;
        };
        int k = 0;
        in_ = in;                    // saveTail 用
        // 主循环：每消费满 R 个样本发射一次（kToNext_ 跨调用连续）
        while (k < nin) {
            int room = nin - k;
            if (kToNext_ > room) {   // 尾部：不足 R 个，全部计入节拍留给下次
                kToNext_ -= room;
                k = nin;
                break;
            }
            k += kToNext_;           // 消费到发射点（样本已消费，同 v4.193 写入延迟线）
            if (nout >= maxOut) {    // 截断：输出跳过不补发，样本已消费（同 v4.193 return 语义）
                kToNext_ = R;        // 发射点已对齐，下次再消费 R 发射下一个
                saveTail(k);
                return nout;
            }
            emitWindow(k);
            kToNext_ = R;
        }
        saveTail(k);
        return nout;
    }

private:
    // v4.194: 块末保存逻辑流最后 H=N-1 样本到 tail_（k=本调用已消费数）
    // 逻辑流 = 旧 tail_（调用前历史 H 个）+ in[0..k)
    void saveTail(int k) {
        const int H = N_ - 1;
        float* t = tail_.data();
        if (k >= H) {
            memcpy(t, in_ + (k - H), H * sizeof(float));   // 全部来自当前块
        } else {
            // 新 tail_ = 逻辑流最后 H 个：旧 tail_ 的 [k..H) 段（H-k 个，
            // 覆盖前部）+ 当前块 in[0..k)（k 个，接尾部）
            const int nT = H - k;
            memmove(t, t + k, nT * sizeof(float));
            memcpy(t + nT, in_, k * sizeof(float));
        }
    }

private:
    int R_ = 1, K_ = 1, N_ = 1;
    double fsIn_ = 0;
    double fcHz_ = 0, attenDb_ = 120;
    std::vector<float> cI_, cQ_;   // 复系数（窗口正序）
    std::vector<float> tail_;      // v4.194: 逻辑流最后 N-1 样本（跨块历史，init 全 0）
    int kToNext_ = 0;              // v4.194: 节拍计数（距下次发射还需消费的样本数）
    const float* in_ = nullptr;    // v4.194: saveTail 用的当前块指针（process 入口缓存）
    uint64_t rotInc_ = 0, rotPhase_ = 0, rotStart_ = 0;
    bool inited_ = false;
};


// ---------------------------------------------------------------------------
// FracResampler v2 — 有理 L/M 分数重采样（复输入，2M→48/96/192k）
// 多相：poly_[p*K+k] = h[k*L + p]，p ∈ [0,L)，fc 相对输入域
// 输出 y[m] = Σ_k x[b-k]·poly[r][k]，b = floor(mM/L)，r = (mM) mod L
// 线性化：与 PolyDecFIR 同款双倍延迟线（容量 2(K+1)），无取模
// v4.192: 容量根修——旧 cap_=K+2 不足：正常流相邻输出 b 步进最大 ⌈M/L⌉（L=3 时 41），
//   窗口深度 = K + maxdist = 330+41 = 371 > 332，回卷低位段（w_∈[332,349]）
//   且 dist>w_-K 时窗口起点读穿 buffer 前的堆元数据（黑盒 bit-exact 实验
//   抓到 1e8 量级垃圾值证实）。新容量 K+M/L+3 保证正常流永不越界、永不丢输出；
//   dist 防御阈值同步收紧为真实边界 cap_-K（冻结恢复时数据确实不存在的输出照旧丢弃）。
// ---------------------------------------------------------------------------
class FracResampler {
public:
    FracResampler() : inited_(false) {}

    void init(int L, int M, int K, double fcNorm, double attenDb) {
        L_ = L; M_ = M; K_ = K;
        N_ = L_ * K_;
        std::vector<float> h(N_);
        double fcUp = fcNorm / (double)L_;
        design_lpf(h.data(), N_, fcUp, attenDb);
        // 插值归一化：design_lpf 保证 Σh=1（插零后 DC 分量=1/L），
        // 上采样需 ×L 使 DC 增益=1（多相子滤波器幅度=1/L → 补回 L）
        double scale = (double)L_;
        poly_.resize(N_);
        for (int p = 0; p < L_; ++p)
            for (int k = 0; k < K_; ++k)
                poly_[(size_t)p * K_ + k] = (float)(h[(size_t)k * L_ + p] * scale);
        cap_ = K_ + M_ / L_ + 3;   // v4.192: 根修容量（旧 K+2 不足，见类头注释）
        delayI_.assign(2 * cap_, 0.0f);
        delayQ_.assign(2 * cap_, 0.0f);
        w_ = cap_;
        total_ = 0; mNext_ = 0;
        bCur_ = 0; rCur_ = 0;
        q0_ = M_ / L_; rm_ = M_ % L_;
        inited_ = true;
    }

    void reset() {
        std::fill(delayI_.begin(), delayI_.end(), 0.0f);
        std::fill(delayQ_.begin(), delayQ_.end(), 0.0f);
        w_ = cap_; total_ = 0; mNext_ = 0;
        bCur_ = 0; rCur_ = 0;
    }

    // 复输入流式处理；跨块状态（total_/mNext_）保证输出速率精确 L/M
    int process(const float* inI, const float* inQ, int nin,
                float* outI, float* outQ, int maxOut) {
        int nout = 0;
        const int K = K_, cap = cap_;
        float* dI = delayI_.data();
        float* dQ = delayQ_.data();
        for (int i = 0; i < nin; ++i) {
            dI[w_] = inI[i]; dQ[w_] = inQ[i];
            dI[w_ - cap] = inI[i]; dQ[w_ - cap] = inQ[i];
            ++w_;
            if (w_ == 2 * cap) {
                memcpy(dI, dI + cap, cap * sizeof(float));
                memcpy(dQ, dQ + cap, cap * sizeof(float));
                w_ = cap;
            }
            ++total_;
            // 产生所有相位已成熟的输出
            while (nout < maxOut) {
                // v4.192: 商余递推替代逐输出 64 位软除法（___divmoddi4）
                // 维护 bCur_=floor(mNext_*M/L)、rCur_=(mNext_*M)%L；
                // 前进：bCur_+=q0_、rCur_+=rm_（q0_=M/L，rm_=M%L<L 故一步回卷即可）
                // 与原 t2/L、t2%L 逐点精确相等（纯整数运算，无漂移无溢出）
                long long b = bCur_;
                int r = rCur_;
                // 需要 x[b] 已写入：b < total_（0 基：写入第 total_ 个样本后 b 最大 total_-1）
                if (b > total_ - 1) break;
                // v4.181: 越界防御——回退距离 dist = total_-1-b 不得超过 cap-1。
                // 正常流不会触发；但 maxOut 提前截断时 mNext_ 冻结、total_ 继续
                // 累加，下一轮 b 恢复时 dist 可能深度超限 → base[-k] 负偏移越界读
                // （堆前哨兵被读穿，v4.180 现场嫌疑点之一）。超限时丢弃该输出
                //（数据已陈旧）并同步相位，保证后续输出索引连续。
                long long dist = total_ - 1 - b;
                if (dist > (long long)cap_ - K_) {   // v4.192: 真实边界（旧 cap_-1 过松，低位段越界）
                    ++mNext_;
                    bCur_ += q0_; rCur_ += rm_;
                    if (rCur_ >= L_) { rCur_ -= L_; ++bCur_; }
                    continue;
                }
                const float* base = dI + w_ - 1 - (long)dist;   // 指向 x[b]（最新→回退）
                const float* baseQ = dQ + w_ - 1 - (long)dist;
                const float* ph = &poly_[(size_t)r * K];
                // v4.193: 4 路部分和打破 FMA 串行依赖链（8559U 8 线程实测 -47%）
                // x[b-k]：b 往回 k 步。基址向回走 → 负步长线性访问（容量 2cap 保证线性）
                // 注意 b-k ≥ total_-cap+... 由延迟线深度保证（K ≤ cap）
                float aI0 = 0.f, aI1 = 0.f, aI2 = 0.f, aI3 = 0.f;
                float aQ0 = 0.f, aQ1 = 0.f, aQ2 = 0.f, aQ3 = 0.f;
                int k = 0;
                for (; k + 4 <= K; k += 4) {
                    aI0 += base[-k    ] * ph[k    ]; aQ0 += baseQ[-k    ] * ph[k    ];
                    aI1 += base[-k - 1] * ph[k + 1]; aQ1 += baseQ[-k - 1] * ph[k + 1];
                    aI2 += base[-k - 2] * ph[k + 2]; aQ2 += baseQ[-k - 2] * ph[k + 2];
                    aI3 += base[-k - 3] * ph[k + 3]; aQ3 += baseQ[-k - 3] * ph[k + 3];
                }
                float accI = (aI0 + aI1) + (aI2 + aI3);
                float accQ = (aQ0 + aQ1) + (aQ2 + aQ3);
                for (; k < K; ++k) {
                    accI += base[-k] * ph[k];
                    accQ += baseQ[-k] * ph[k];
                }
                outI[nout] = accI;
                outQ[nout] = accQ;
                ++nout;
                ++mNext_;
                bCur_ += q0_; rCur_ += rm_;
                if (rCur_ >= L_) { rCur_ -= L_; ++bCur_; }
            }
        }
        return nout;
    }

private:
    int L_ = 1, M_ = 1, K_ = 1, N_ = 1, cap_ = 4;
    std::vector<float> poly_, delayI_, delayQ_;
    int w_ = 0;
    long long total_ = 0, mNext_ = 0;
    long long bCur_ = 0;           // v4.192: floor(m*M/L) 递推状态
    int rCur_ = 0;                 // v4.192: (m*M)%L 递推状态
    int q0_ = 0, rm_ = 0;          // v4.192: M/L 与 M%L（init 预计算）
    bool inited_ = false;
};

// 静态成员表定义（类外）
std::vector<float> TabNCO::table_;

} // namespace skdsp
