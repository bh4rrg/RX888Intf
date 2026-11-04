---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: 'cf1c2bf9-1e40-445c-aac6-9d38fd78e505'
  PropagateID: 'cf1c2bf9-1e40-445c-aac6-9d38fd78e505'
  ReservedCode1: 'fc0c8061-1fda-47ec-b1d0-6b5ad8b13b42'
  ReservedCode2: 'fc0c8061-1fda-47ec-b1d0-6b5ad8b13b42'
---

# RX888Intf — CW Skimmer Server 的 RX888/RX666 驱动 DLL

面向 RX888 / RX666（BBRF103 家族）宽带 SDR 接收机的 SkimSrv 开源设备
驱动，聚焦多波段 CW Skimming。编译为 32 位 DLL，由 CW Skimmer Server
（Afreet，1.6.x）加载。

当前版本：**v4.195**（2026-11-04）

## 特性

- 从零重写的直接射频采样流水线：64M IQ → 分通道 DDC（NCO 混频 +
  抽取 FIR 64M→2M + 分数重采样 2M→96k），无 r2iq、无 DC blocker
- AVX2/FMA 向量化热路径（`-O3 -march=x86-64-v3 -ffast-math`），
  Ryzen 7 3700X 上约 198µs/8 通道块——实时有余量
- 构造即正确的流控：跨块 FIR 历史（`tail_` + 拍计数器），环形缓冲
  过载绝不无声丢数据
- RX666 支持：每次上电自动下载固件；拒绝把内置 RX888 固件刷入
  RX666 硬件（防止进入无响应的 'SDR' 假态）
- 生产级日志（RX888New.log），基于 QPC 的计时遥测（每 1000 块的
  wall/wait/prefetch）
- v4.195：USB 链路半死故障下自动恢复（最多 3 次）；仍失败则主动
  停止接收并在日志末尾写醒目报警块（"USB RECOVERY EXHAUSTED"），
  不再无声卡死

## 快速上手（用户）

下载 Release 包（RX888Intf_vX.Y.zip），解压后把全部文件复制到
SkimSrv.exe 所在目录，无需编译。

- RX888：开箱即用（RX888 固件已内置 DLL）
- RX666：另从官方 RX666 驱动包复制 `RX666.img` 到同目录（每次上电
  都需要；DLL 拒绝把内置 RX888 固件刷入 RX666 硬件——收不到时查
  RX888New.log）

前置条件：CW Skimmer Server 1.6.x（32 位）、Cypress FX3 USB 驱动
（CyUSB3.sys）、USB 3.0 口（RX888）。

## 从源码构建（开发者）

工具链：MinGW-w64 GCC，32 位目标（DLL 必须与 32 位 SkimSrv 进程
匹配）。

```
build.bat              （使用 C:\msys64\mingw32\bin\g++.exe）
```

编译参数为强制项，不得修改：

- `-ffast-math` —— GCC 严格浮点模式禁止归约向量化；失去它会导致
  8 倍性能退化
- `-march=x86-64-v3` —— AVX2+FMA 基线（不要用 `-march=native`）

唯一的外部 DLL 依赖是 libwinpthread-1.dll（MinGW 运行库，已随
Release 包分发）。

## 仓库结构

```
src/          驱动源码（skimsrv_intf.cpp + DSP 流水线 + CyAPI）
tools/        验证与基准测试工具
LICENSE       MIT
NOTICE        第三方来源声明（ExtIO_sddc、Cypress CyAPI、固件）
```

## 配置（RX888Intf.ini）

驱动只读取三个键：

- `OutputScale` —— 输出给 SkimSrv 的 IQ 幅度，直接影响解码门限。
  RX666：6000 / RX888：10.0
- `HfAttenuation` —— HF 前端衰减（dB），仅 0/10/20 三档
- `UnpauseMode` —— 解码解除暂停策略：0 = 温和（默认），1 = 强制

其余旧键（ADCFrequencyMHz、SRateIdx、AdcDither 等）一律忽略，
对应行为已在代码中固定。

## 常见问题

- 驱动日志：DLL 同目录的 RX888New.log（纯文本，累积追加）
- 日志出现 "RX666.img MISSING" → 从官方 RX666 驱动包复制该文件，
  重新插拔 USB，再 Start
- 日志出现 "USB RECOVERY EXHAUSTED" → USB 链路已硬性故障，驱动
  已停止接收。拔插 USB 线重新上电，再重启 SkimSrv；若频繁出现，
  检查 USB 线缆、供电或集线器
- 完全没有 "Enumerate" 行 → Cypress FX3 驱动（CyUSB3.sys）未装好
  或设备不在 USB 3.0 口上
- 不要往 SkimSrv 目录放任何无关 DLL：SkimSrv 会扫描并加载目录内
  找到的 DLL

## 致谢

- Oscar Steila ik1xpv —— ExtIO_sddc / BBRF103 项目，MIT 许可
  （FX3handler、SDDC_FX3 固件镜像）
- Cypress Semiconductor —— CyAPI USB 库

---

# RX888Intf — RX888/RX666 driver DLL for CW Skimmer Server

Open-source SkimSrv device driver for the RX888 / RX666 (BBRF103-family)
wideband SDR receivers, focused on multi-band CW skimming. Built as a
32-bit DLL loaded by CW Skimmer Server (Afreet, 1.6.x).

Current release: **v4.195** (2026-11-04)

## Highlights

- Direct RF sampling pipeline rewritten from scratch: 64M IQ → per-channel
  DDC (NCO mixer + decimating FIR 64M→2M + fractional resampler 2M→96k),
  no r2iq, no DC blocker
- AVX2/FMA vectorized hot path (`-O3 -march=x86-64-v3 -ffast-math`),
  ~198µs/8-channel block on a Ryzen 7 3700X — real-time with margin
- Correct-by-construction flow control: cross-block FIR history
  (`tail_` + beat counter), ring-buffer overrun never silently drops data
- RX666 support: firmware auto-download on each power-up, refusal to
  flash the built-in RX888 firmware into RX666 hardware (prevents the
  unresponsive 'SDR' zombie state)
- Production-grade logging (RX888New.log), QPC-based timing telemetry
  (wall/wait/prefetch per 1000 blocks)
- v4.195: automatic recovery from a half-dead USB link (up to 3
  attempts); if recovery fails, reception stops cleanly with a loud
  alarm block ("USB RECOVERY EXHAUSTED") at the end of the log —
  no more silent hangs

## Quick start (users)

Download the release package (RX888Intf_vX.Y.zip), unzip and copy all
files into your SkimSrv.exe directory. No build required.

- RX888: works out of the box (RX888 firmware is embedded in the DLL)
- RX666: additionally copy `RX666.img` from the official RX666 driver
  package into the same directory (required at every power-up; the DLL
  refuses to flash its built-in RX888 firmware into RX666 hardware —
  check RX888New.log if reception does not start)

Requirements: CW Skimmer Server 1.6.x (32-bit), Cypress FX3 USB driver
(CyUSB3.sys), USB 3.0 port (RX888).

## Build from source (developers)

Toolchain: MinGW-w64 GCC, 32-bit target (the DLL must match the 32-bit
SkimSrv process).

```
build.bat              (uses C:\msys64\mingw32\bin\g++.exe)
```

Compiler flags are mandatory and must not be changed:

- `-ffast-math` — GCC strict FP forbids reduction vectorization; losing
  it causes an 8x performance regression
- `-march=x86-64-v3` — AVX2+FMA baseline (do NOT use `-march=native`)

The only external DLL dependency is libwinpthread-1.dll (MinGW runtime,
shipped in the release package).

## Repository layout

```
src/          driver source (skimsrv_intf.cpp + DSP pipeline + CyAPI)
tools/        verification & benchmark harnesses
LICENSE       MIT
NOTICE        third-party attribution (ExtIO_sddc, Cypress CyAPI, firmware)
```

## Configuration (RX888Intf.ini)

Only three keys are read by the driver:

- `OutputScale` — IQ amplitude delivered to SkimSrv, directly affects
  decode threshold. RX666: 6000 / RX888: 10.0
- `HfAttenuation` — HF front-end attenuation in dB, steps 0/10/20 only
- `UnpauseMode` — decoder unpause strategy: 0 = gentle (default), 1 = force

All other legacy keys (ADCFrequencyMHz, SRateIdx, AdcDither, ...) are
ignored; the corresponding behavior is fixed in code.

## Troubleshooting

- Driver log: RX888New.log next to the DLL (plain text, appended)
- "RX666.img MISSING" in the log → copy the file from the official RX666
  driver package, replug USB, Start again
- "USB RECOVERY EXHAUSTED" in the log → the USB link is hard-dead and
  reception has stopped. Unplug/replug the USB cable, then restart
  SkimSrv; if it happens often, check cable, power or hub
- No "Enumerate" lines at all → Cypress FX3 driver (CyUSB3.sys) not
  installed or device not on a USB 3.0 port
- Never put unrelated DLLs into the SkimSrv directory: SkimSrv scans and
  loads DLLs found there

## Acknowledgements

- Oscar Steila ik1xpv — ExtIO_sddc / BBRF103 project, MIT licensed
  (FX3handler, SDDC_FX3 firmware image)
- Cypress Semiconductor — CyAPI USB library

> AI生成