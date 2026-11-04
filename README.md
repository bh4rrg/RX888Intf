---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: 'c215c6d4-e959-423e-b460-f3fa3ab2e816'
  PropagateID: 'c215c6d4-e959-423e-b460-f3fa3ab2e816'
  ReservedCode1: 'af0f1804-7ed9-4978-ad88-8de10544ae86'
  ReservedCode2: 'af0f1804-7ed9-4978-ad88-8de10544ae86'
---

# RX888Intf — CW Skimmer Server 的 RX888/RX666 驱动 DLL

面向 RX888 / RX666（BBRF103 家族）宽带 SDR 接收机的 SkimSrv 开源设备
驱动，聚焦多波段 CW Skimming。编译为 32 位 DLL，由 CW Skimmer Server
（Afreet，1.6.x）加载。

当前版本：**v4.195**（2026-11-04）

## 特性

- 从零重写的直接射频采样流水线：64M IQ → 分通道 DDC，无 r2iq、无 DC blocker
- AVX2/FMA 向量化热路径
- RX666 支持

## 快速上手（用户）

下载 Release 包（RX888Intf_vX.Y.zip），解压后把全部文件复制到
SkimSrv.exe 所在目录，无需编译。

- RX888：开箱即用（RX888 固件已内置 DLL）
- RX666：另从官方 RX666 驱动包复制 `RX666.img` 到同目录

前置条件：CW Skimmer Server 1.6.x（32 位）、Cypress FX3 USB 驱动
（CyUSB3.sys）、USB 3.0 口。

## 从源码构建（开发者）

工具链：MinGW-w64 GCC，32 位目标（DLL 必须与 32 位 SkimSrv 进程
匹配）。

```
build.bat              （使用 C:\msys64\mingw32\bin\g++.exe）
```

编译参数为强制项，不得修改
唯一的外部 DLL 依赖是 libwinpthread-1.dll。

## 仓库结构

```
src/          驱动源码（skimsrv_intf.cpp + DSP 流水线 + CyAPI）
tools/        验证与基准测试工具
LICENSE       MIT
NOTICE        第三方来源声明（ExtIO_sddc、Cypress CyAPI、固件）
```


## 常见问题

- 日志出现 "RX666.img MISSING" → 从官方 RX666 驱动包复制该文件，
  重新插拔 USB，再 Start
- 日志出现 "USB RECOVERY EXHAUSTED" → USB 链路已硬性故障，驱动
  已停止接收。拔插 USB 线重新上电，再重启 SkimSrv；若频繁出现，
  检查 USB 线缆、供电或集线器
- 完全没有 "Enumerate" 行 → Cypress FX3 驱动（CyUSB3.sys）未装好
  或设备不在 USB 3.0 口上

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
  DDC, no r2iq, no DC blocker
- AVX2/FMA vectorized hot path  
- Correct-by-construction flow control: cross-block FIR history, ring-buffer overrun never silently drops data
- RX666 support
- v4.195: automatic recovery from a half-dead USB link (up to 3
  attempts); if recovery fails, reception stops cleanly with a loud
  alarm block ("USB RECOVERY EXHAUSTED") at the end of the log —
  no more silent hangs

## Quick start (users)

Download the release package (RX888Intf_vX.Y.zip), unzip and copy all
files into your SkimSrv.exe directory. No build required.

- RX888: works out of the box (RX888 firmware is embedded in the DLL)
- RX666: additionally copy `RX666.img` from the official RX666 driver
  package into the same directory
  
Requirements: CW Skimmer Server 1.6.x (32-bit), Cypress FX3 USB driver
(CyUSB3.sys), USB 3.0 port .


## Repository layout

```
src/          driver source (skimsrv_intf.cpp + DSP pipeline + CyAPI)
tools/        verification & benchmark harnesses
LICENSE       MIT
NOTICE        third-party attribution (ExtIO_sddc, Cypress CyAPI, firmware)
```


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