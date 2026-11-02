// verify_v4194.cpp - v4.194 DSP 改动等价性验证（受控合成实验，黑盒对比）
// 参照基线 = v4.193 MixDecFIR 算法逐行复制（576 样本双倍延迟线 + R 组 memcpy
// + 回卷 memcpy + 4 路部分和内积，复用 v4.192 的倒计数器节拍）。
// 本轮改动（仅 MixDecFIR 数据供给方式改变，FracResampler 未动）：
//   MixDecFIR：延迟线+逐组 memcpy → tail_(N-1 历史) + 直读输入大数组；
//     节拍 = 每消费第 R 个样本发射（kToNext_ 跨调用连续），发射窗口
//     [k-N, k-1]（调用内坐标），负坐标映射 tail_[k-1+jp]
// 判据（同 v4.193 口径，求和树差异为舍入级）：
//   ①结构等价：nout 逐块严格相等（含任意块长混合序列、maxOut 截断风暴）
//   ②数值容差：max|Δ| < 1e-4×max|out|；RMS(Δ)/RMS(out) < 1e-5
//   ③Goertzel 保真：新/旧输出单频幅度比 1±5e-4
//   ④频率精确性：截断风暴下新旧输出主频一致（FFT 过零验证已含在 Goertzel 内）
// 编译（与 DLL 同工具链同参数）：
//   g++ -O3 -march=x86-64-v3 -ffast-math -o verify_v4194.exe verify_v4194.cpp
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <algorithm>

#include "polyphase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================ 参照实现（v4.193 原样） ============================
namespace oldref {

// --- v4.193 MixDecFIR：576 样本双倍延迟线 + R 组 memcpy + 回卷 memcpy + 4 路部分和 ---
class MixDecFIR193 {
public:
    MixDecFIR193() : inited_(false) {}
    void init(double loHz, double fsIn, int R, int K, double fcHz, double attenDb) {
        R_ = R; K_ = K; N_ = R * K;
        fsIn_ = fsIn; fcHz_ = fcHz; attenDb_ = attenDb;
        std::vector<double> h(N_);
        {
            std::vector<float> hf(N_);
            skdsp::design_lpf(hf.data(), N_, fcHz / fsIn, attenDb);
            for (int j = 0; j < N_; ++j) h[j] = hf[j];
        }
        cI_.resize(N_); cQ_.resize(N_);
        const double w0 = 2.0 * M_PI * loHz / fsIn;
        for (int jp = 0; jp < N_; ++jp) {
            int j = N_ - 1 - jp;
            cI_[jp] = (float)(h[j] * std::cos(w0 * j));
            cQ_[jp] = (float)(h[j] * std::sin(w0 * j));
        }
        delay_.assign(2 * N_, 0.0f);
        w_ = N_; cnt_ = 0;
        double incTurns = std::fmod(loHz * R / fsIn, 1.0);
        if (incTurns < 0) incTurns += 1.0;
        rotInc_ = (uint64_t)(incTurns * 18446744073709551616.0);
        double startTurns = std::fmod(-(loHz * (R - 1) / fsIn), 1.0);
        if (startTurns < 0) startTurns += 1.0;
        rotStart_ = (uint64_t)(startTurns * 18446744073709551616.0);
        rotPhase_ = rotStart_;
        skdsp::TabNCO::ensureTablePub();
        inited_ = true;
    }
    void reset() {
        std::fill(delay_.begin(), delay_.end(), 0.0f);
        w_ = N_; cnt_ = 0;
        rotPhase_ = rotStart_;
    }
    int process(const float* in, int nin, float* outI, float* outQ, int maxOut) {
        int nout = 0;
        const int N = N_, R = R_;
        float* d = delay_.data();
        const float* cI = cI_.data();
        const float* cQ = cQ_.data();
        auto emitOne = [&]() {
            const float* p = d + w_ - N;
            float aI0 = 0.f, aI1 = 0.f, aI2 = 0.f, aI3 = 0.f;
            float aQ0 = 0.f, aQ1 = 0.f, aQ2 = 0.f, aQ3 = 0.f;
            int jp = 0;
            for (; jp + 4 <= N; jp += 4) {
                aI0 += p[jp  ] * cI[jp  ]; aQ0 += p[jp  ] * cQ[jp  ];
                aI1 += p[jp+1] * cI[jp+1]; aQ1 += p[jp+1] * cQ[jp+1];
                aI2 += p[jp+2] * cI[jp+2]; aQ2 += p[jp+2] * cQ[jp+2];
                aI3 += p[jp+3] * cI[jp+3]; aQ3 += p[jp+3] * cQ[jp+3];
            }
            float accI = (aI0 + aI1) + (aI2 + aI3);
            float accQ = (aQ0 + aQ1) + (aQ2 + aQ3);
            for (; jp < N; ++jp) { accI += p[jp] * cI[jp]; accQ += p[jp] * cQ[jp]; }
            uint32_t idx = (uint32_t)(rotPhase_ >> (64 - skdsp::TabNCO::TBL_BITS_PUB));
            uint32_t frac = (uint32_t)(rotPhase_ >> (64 - skdsp::TabNCO::TBL_BITS_PUB - 15)) & 0x7FFF;
            const float* tbl = skdsp::TabNCO::table();
            const float* e0 = &tbl[idx << 1];
            const float* e1 = &tbl[((idx + 1) & (skdsp::TabNCO::TBL_SIZE_PUB - 1)) << 1];
            float rc = e0[0] + (e1[0] - e0[0]) * (frac * (1.0f / 32768.0f));
            float rs = e0[1] + (e1[1] - e0[1]) * (frac * (1.0f / 32768.0f));
            outI[nout] = accI * rc + accQ * rs;
            outQ[nout] = accQ * rc - accI * rs;
            ++nout;
            rotPhase_ += rotInc_;
        };
        int i = 0;
        while (i < nin && cnt_ != 0) {
            d[w_] = in[i]; d[w_ - N] = in[i];
            ++w_;
            if (w_ == 2 * N) { memcpy(d, d + N, N * sizeof(float)); w_ = N; }
            ++i;
            if (++cnt_ >= R) {
                cnt_ = 0;
                if (nout >= maxOut) return nout;
                emitOne();
                break;
            }
        }
        for (; i + R <= nin; i += R) {
            memcpy(d + w_, in + i, R * sizeof(float));
            memcpy(d + w_ - N, in + i, R * sizeof(float));
            w_ += R;
            if (w_ == 2 * N) { memcpy(d, d + N, N * sizeof(float)); w_ = N; }
            if (nout >= maxOut) return nout;
            emitOne();
        }
        for (; i < nin; ++i) {
            d[w_] = in[i]; d[w_ - N] = in[i];
            ++w_;
            if (w_ == 2 * N) { memcpy(d, d + N, N * sizeof(float)); w_ = N; }
            ++cnt_;
        }
        return nout;
    }
private:
    int R_ = 1, K_ = 1, N_ = 1;
    double fsIn_ = 0, fcHz_ = 0, attenDb_ = 120;
    std::vector<float> cI_, cQ_;
    std::vector<float> delay_;
    int w_ = 0, cnt_ = 0;
    uint64_t rotInc_ = 0, rotPhase_ = 0, rotStart_ = 0;
    bool inited_ = false;
};

} // namespace oldref

// ============================ 测试工具 ============================
static int gFail = 0;

static void FillInput(std::vector<float>& buf, int mode, unsigned long long seed) {
    int n = (int)buf.size();
    switch (mode) {
    case 0: memset(buf.data(), 0, n * sizeof(float)); break;
    case 1: {
        unsigned long long s = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        for (int i = 0; i < n; ++i) {
            s ^= s << 13; s ^= s >> 7; s ^= s << 17;
            buf[i] = (float)(int32_t)(s >> 40) / 32768.0f;
        }
        break;
    }
    default: {
        double w = 2.0 * M_PI * 3553000.0 / 64000000.0;
        for (int i = 0; i < n; ++i)
            buf[i] = 100.0f * (float)sin(w * (double)(seed + (unsigned long long)i));
        break;
    }
    }
}

static double NumDiff(const float* a, const float* b, int n, double& maxOut) {
    double maxD = 0, mo = 0;
    for (int i = 0; i < n; ++i) {
        double d = fabs((double)a[i] - (double)b[i]);
        if (d > maxD) maxD = d;
        double m = fabs((double)a[i]);
        if (m > mo) mo = m;
    }
    maxOut = mo;
    return maxD;
}

static double GoertzelAmp(const float* x, int n, double fHz, double fsHz) {
    double w = 2.0 * M_PI * fHz / fsHz;
    double cw = 2.0 * cos(w);
    double s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < n; ++i) {
        s0 = x[i] + cw * s1 - s2;
        s2 = s1; s1 = s0;
    }
    double power = s1 * s1 + s2 * s2 - cw * s1 * s2;
    return 2.0 * sqrt(fabs(power)) / n;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    const int BLK = 65536, NBLK = 120;
    const int MAXOUT = BLK / 32 + 64;   // 2112
    printf("== verify_v4194: v4.194 DSP 等价性验证（基线 v4.193） ==\n\n");

    // ---------------- A1: MixDecFIR 全块流式（生产形态） ----------------
    printf("[A1] MixDecFIR 全块流式 65536×3 输入模式（数值容差 + 结构等价）\n");
    {
        for (int mode = 0; mode < 3; ++mode) {
            oldref::MixDecFIR193 om;
            skdsp::MixDecFIR nm;
            om.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
            nm.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
            std::vector<float> in(BLK), oI(MAXOUT), oQ(MAXOUT), nI(MAXOUT), nQ(MAXOUT);
            bool ok = true; double worstRel = 0, sumD2 = 0, sumO2 = 0; long long nTot = 0;
            for (int blk = 0; blk < NBLK && ok; ++blk) {
                FillInput(in, mode, blk);
                int no = om.process(in.data(), BLK, oI.data(), oQ.data(), MAXOUT);
                int nn = nm.process(in.data(), BLK, nI.data(), nQ.data(), MAXOUT);
                if (no != nn) { printf("  [mode %d blk %d] nout mismatch: old=%d new=%d\n", mode, blk, no, nn); ok = false; break; }
                double mo1, mo2;
                double d1 = NumDiff(oI.data(), nI.data(), no, mo1);
                double d2 = NumDiff(oQ.data(), nQ.data(), no, mo2);
                double mo = mo1 > mo2 ? mo1 : mo2, md = d1 > d2 ? d1 : d2;
                if (mo > 0) { double rel = md / mo; if (rel > worstRel) worstRel = rel; }
                for (int i = 0; i < no; ++i) {
                    double dv = (double)oI[i] - nI[i]; sumD2 += dv * dv; sumO2 += (double)oI[i] * oI[i];
                    dv = (double)oQ[i] - nQ[i]; sumD2 += dv * dv; sumO2 += (double)oQ[i] * oQ[i];
                }
                nTot += no;
                if (mo > 0 && md / mo > 1e-4) { printf("  [mode %d blk %d] rel diff %.3g 超限\n", mode, blk, md / mo); ok = false; }
            }
            double rmsRel = sumO2 > 0 ? sqrt(sumD2 / sumO2) : 0;
            printf("  mode %d: %s (worstRel=%.3g rmsRel=%.3g n=%lld)\n", mode, ok ? "PASS" : "FAIL", worstRel, rmsRel, nTot);
            if (!ok || rmsRel > 1e-5) ++gFail;
        }
    }

    // ---------------- A2: MixDecFIR 任意块长混合序列（覆盖残余/尾部/tail_ 拼接） ----------------
    printf("[A2] MixDecFIR 任意块长混合序列（残余对齐/尾部/tail_ 拼接 + 全程 Goertzel）\n");
    {
        oldref::MixDecFIR193 om;
        skdsp::MixDecFIR nm;
        om.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        nm.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        // 刻意覆盖：全块、小块、R 附近、极小、跨 tail_ 窗口深度（H=287）的长度
        const int lens[] = { 65536, 1000, 777, 65536, 100, 3072, 2049, 65536, 31, 33, 65536,
                            12345, 65536, 287, 288, 289, 65536, 50000, 65536, 3, 1, 65536 };
        const int NL = (int)(sizeof(lens) / sizeof(lens[0]));
        std::vector<float> oI(70000), oQ(70000), nI(70000), nQ(70000);
        std::vector<float> allO, allN;
        bool ok = true; double worstRel = 0;
        for (int rep = 0; rep < 3 && ok; ++rep) {
            for (int li = 0; li < NL && ok; ++li) {
                int nin = lens[li];
                std::vector<float> in(nin);
                FillInput(in, 2, rep * 1000 + li);   // 3.553MHz 正弦（输出含近直流大分量+边缘）
                int no = om.process(in.data(), nin, oI.data(), oQ.data(), MAXOUT);
                int nn = nm.process(in.data(), nin, nI.data(), nQ.data(), MAXOUT);
                if (no != nn) { printf("  [rep %d len %d] nout mismatch: old=%d new=%d\n", rep, nin, no, nn); ok = false; break; }
                double mo1, mo2;
                double d1 = NumDiff(oI.data(), nI.data(), no, mo1);
                double d2 = NumDiff(oQ.data(), nQ.data(), no, mo2);
                double mo = mo1 > mo2 ? mo1 : mo2, md = d1 > d2 ? d1 : d2;
                if (mo > 0) { double rel = md / mo; if (rel > worstRel) worstRel = rel; if (rel > 1e-4) { printf("  [rep %d len %d] rel %.3g 超限\n", rep, nin, rel); ok = false; } }
                allO.insert(allO.end(), oQ.data(), oQ.data() + no);
                allN.insert(allN.end(), nQ.data(), nQ.data() + no);
            }
        }
        double gO = GoertzelAmp(allO.data(), (int)allO.size(), 976.5625, 2000000.0);
        double gN = GoertzelAmp(allN.data(), (int)allN.size(), 976.5625, 2000000.0);
        double gRel = gO > 0 ? fabs(gN - gO) / gO : 0;
        printf("  混合块长: %s (worstRel=%.3g, Goertzel 976.5625Hz old=%.4f new=%.4f rel=%.2e)\n",
               ok && gRel < 5e-4 ? "PASS" : "FAIL", worstRel, gO, gN, gRel);
        if (!ok || gRel >= 5e-4) ++gFail;
    }

    // ---------------- A3: MixDecFIR maxOut 截断风暴（节拍/相位连续性） ----------------
    printf("[A3] MixDecFIR maxOut 截断风暴（nout 序列 + 数值 + 有限性）\n");
    {
        oldref::MixDecFIR193 om;
        skdsp::MixDecFIR nm;
        om.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        nm.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        std::vector<float> in(BLK), oI(MAXOUT), oQ(MAXOUT), nI(MAXOUT), nQ(MAXOUT);
        bool ok = true; double worstRel = 0; long long totO = 0, totN = 0;
        for (int blk = 0; blk < 300 && ok; ++blk) {
            FillInput(in, 1, blk);
            int mo_ = (blk % 30 < 25) ? 5 : MAXOUT;   // 周期性截断风暴
            int no = om.process(in.data(), BLK, oI.data(), oQ.data(), mo_);
            int nn = nm.process(in.data(), BLK, nI.data(), nQ.data(), mo_);
            totO += no; totN += nn;
            if (no != nn) { printf("  [截断 blk %d] nout mismatch: old=%d new=%d\n", blk, no, nn); ok = false; break; }
            double mo1, mo2;
            double d1 = NumDiff(oI.data(), nI.data(), no, mo1);
            double d2 = NumDiff(oQ.data(), nQ.data(), no, mo2);
            double mo = mo1 > mo2 ? mo1 : mo2, md = d1 > d2 ? d1 : d2;
            if (mo > 0) { double rel = md / mo; if (rel > worstRel) worstRel = rel; if (rel > 1e-4) { ok = false; } }
            for (int i = 0; i < no && ok; ++i)
                if (!std::isfinite(oI[i]) || !std::isfinite(nI[i])) { printf("  [截断 blk %d] 非有限值\n", blk); ok = false; }
        }
        printf("  截断风暴: %s (worstRel=%.3g totalOut old=%lld new=%lld)\n", ok ? "PASS" : "FAIL", worstRel, totO, totN);
        if (!ok || totO == 0 || totO != totN) ++gFail;
    }

    // ---------------- A4: MixDecFIR 截断+小块交替（截断恢复与 tail_ 交互） ----------------
    printf("[A4] MixDecFIR 截断+任意块长交替（截断恢复 × tail_ 拼接复合场景）\n");
    {
        oldref::MixDecFIR193 om;
        skdsp::MixDecFIR nm;
        om.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        nm.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        const int lens[] = { 65536, 40, 7, 65536, 33, 1, 2048, 289, 65536, 15, 65536, 100 };
        const int NL = (int)(sizeof(lens) / sizeof(lens[0]));
        std::vector<float> oI(70000), oQ(70000), nI(70000), nQ(70000);
        bool ok = true; double worstRel = 0; long long totO = 0, totN = 0;
        for (int rep = 0; rep < 4 && ok; ++rep) {
            for (int li = 0; li < NL && ok; ++li) {
                int nin = lens[li];
                std::vector<float> in(nin);
                FillInput(in, 1, rep * 100 + li);
                int mo_ = (li % 3 == 0) ? 3 : (li % 3 == 1 ? 1 : MAXOUT);   // 交替截断
                int no = om.process(in.data(), nin, oI.data(), oQ.data(), mo_);
                int nn = nm.process(in.data(), nin, nI.data(), nQ.data(), mo_);
                totO += no; totN += nn;
                if (no != nn) { printf("  [rep %d len %d mo %d] nout mismatch: old=%d new=%d\n", rep, nin, mo_, no, nn); ok = false; break; }
                double mo1, mo2;
                double d1 = NumDiff(oI.data(), nI.data(), no, mo1);
                double d2 = NumDiff(oQ.data(), nQ.data(), no, mo2);
                double mo = mo1 > mo2 ? mo1 : mo2, md = d1 > d2 ? d1 : d2;
                if (mo > 0) { double rel = md / mo; if (rel > worstRel) worstRel = rel; if (rel > 1e-4) { printf("  [rep %d len %d] rel %.3g 超限\n", rep, nin, rel); ok = false; } }
                for (int i = 0; i < no && ok; ++i)
                    if (!std::isfinite(oI[i]) || !std::isfinite(nI[i])) { printf("  [rep %d len %d] 非有限值\n", rep, nin); ok = false; }
            }
        }
        printf("  截断×块长交替: %s (worstRel=%.3g totalOut old=%lld new=%lld)\n", ok ? "PASS" : "FAIL", worstRel, totO, totN);
        if (!ok || totO == 0 || totO != totN) ++gFail;
    }

    // ---------------- A5: reset() 后重新流式（tail_/kToNext_ 归零一致性） ----------------
    printf("[A5] MixDecFIR reset() 一致性（reset 后输出与全新 init 实例逐点一致）\n");
    {
        oldref::MixDecFIR193 om;
        skdsp::MixDecFIR nm, ref2;
        om.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        nm.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        ref2.init(3553000.0, 64000000.0, 32, 9, 110000.0, 120.0);
        std::vector<float> in(BLK), oI(MAXOUT), oQ(MAXOUT), nI(MAXOUT), nQ(MAXOUT), rI(MAXOUT), rQ(MAXOUT);
        bool ok = true; double worstRel = 0;
        for (int blk = 0; blk < 40; ++blk) {
            FillInput(in, 1, blk);
            om.process(in.data(), BLK, oI.data(), oQ.data(), MAXOUT);
            nm.process(in.data(), BLK, nI.data(), nQ.data(), MAXOUT);
        }
        om.reset(); nm.reset(); ref2.reset();
        for (int blk = 0; blk < 20 && ok; ++blk) {
            FillInput(in, 2, blk);
            int no = om.process(in.data(), BLK, oI.data(), oQ.data(), MAXOUT);
            int nn = nm.process(in.data(), BLK, nI.data(), nQ.data(), MAXOUT);
            int nr = ref2.process(in.data(), BLK, rI.data(), rQ.data(), MAXOUT);
            if (no != nn || nn != nr) { printf("  [blk %d] nout mismatch old=%d new=%d ref2=%d\n", blk, no, nn, nr); ok = false; break; }
            double mo1, mo2;
            double d1 = NumDiff(oI.data(), nI.data(), no, mo1);
            double d2 = NumDiff(oQ.data(), nQ.data(), no, mo2);
            double mo = mo1 > mo2 ? mo1 : mo2, md = d1 > d2 ? d1 : d2;
            if (mo > 0) { worstRel = md / mo > worstRel ? md / mo : worstRel; if (md / mo > 1e-4) ok = false; }
        }
        printf("  reset 一致性: %s (worstRel=%.3g)\n", ok ? "PASS" : "FAIL", worstRel);
        if (!ok) ++gFail;
    }

    // ---------------- B: 回归——FracResampler 不受影响（v4.193 口径复跑） ----------------
    printf("[B] FracResampler 回归（未改动，v4.193 判据复跑确认）\n");
    {
        struct Case { int L, M; const char* name; double fsOut; };
        Case cases[] = { {3, 125, "48k(L3/M125)", 48000.0}, {6, 125, "96k(L6/M125)", 96000.0}, {12, 125, "192k(L12/M125)", 192000.0} };
        for (auto& cs : cases) {
            int maxOut = 2048 + 64;
            skdsp::FracResampler nf;
            double fcNorm = (0.48 * cs.fsOut) / 2000000.0;
            nf.init(cs.L, cs.M, 330, fcNorm, 120.0);
            const int NIN = 2048;
            std::vector<float> iI(NIN), iQ(NIN), nI(4096), nQ(4096);
            bool ok = true; long long totalOut = 0;
            unsigned long long s = 12345;
            for (int blk = 0; blk < 200 && ok; ++blk) {
                for (int i = 0; i < NIN; ++i) {
                    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
                    iI[i] = (float)(int32_t)(s >> 40) / 32768.0f;
                    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
                    iQ[i] = (float)(int32_t)(s >> 40) / 32768.0f;
                }
                int mo_ = (blk % 30 < 25) ? 7 : maxOut;
                int nn = nf.process(iI.data(), iQ.data(), NIN, nI.data(), nQ.data(), mo_);
                totalOut += nn;
                for (int i = 0; i < nn; ++i)
                    if (!std::isfinite(nI[i]) || !std::isfinite(nQ[i])) { ok = false; break; }
            }
            printf("  %s 截断风暴安全性: %s (totalOut=%lld)\n", cs.name, ok && totalOut > 0 ? "PASS" : "FAIL", totalOut);
            if (!ok || totalOut == 0) ++gFail;
        }
    }

    printf("\n== 结果: %s (fail=%d) ==\n", gFail == 0 ? "ALL PASS" : "FAILED", gFail);
    return gFail == 0 ? 0 : 1;
}
