# RX888Intf — CW Skimmer Server 的 RX888/RX666 驱动 DLL

面向 RX888 / RX666（BBRF103 家族）宽带 SDR 接收机的 SkimSrv 开源设备
驱动，聚焦多波段 CW Skimming。编译为 32 位 DLL，由 CW Skimmer Server
（Afreet，1.6.x）加载。

当前版本：**v4.208**（2026-11-24）

## 特性

- 从零重写的直接射频采样流水线：64M IQ → 分通道 DDC，无 r2iq、无 DC blocker
- AVX2/FMA 向量化热路径
- RX888 / RX666 双机型支持：硬件型号由 ini 的 `HwModel` 显式指定
- 精确速率契约：192k 通道按 192000 Hz 交付（12/125 重采样）
- 半死 USB 链路的自动恢复（最多 3 次）；失败则干净停机并打醒目告警

## 快速上手（用户）

下载 Release 包（`RX888Intf_vX.Y.zip`），解压后把全部文件复制到
SkimSrv.exe 所在目录，无需编译。

- RX888：开箱即用（固件已内置 DLL），ini 保留默认 `HwModel=1`
- RX666：ini 设 `[RX888] HwModel=2`，并从官方 RX666 驱动包复制
  `RX666.img` 到同目录

前置条件：CW Skimmer Server 1.6.x（32 位）、Cypress FX3 USB 驱动
（CyUSB3.sys）、USB 3.0 口。

## 从源码构建（开发者）

工具链：MinGW-w64 GCC，32 位目标（DLL 必须与 32 位 SkimSrv 进程
匹配）。

编译参数为强制项，不得修改：

```
g++ -shared -O3 -march=x86-64-v3 -ffast-math -o RX888Intf.dll \
    skimsrv_intf.cpp usb_shim.cpp FX3handler.cpp CyAPI.cpp RX888New.def \
    -static-libgcc -static-libstdc++ \
    -lsetupapi -lcfgmgr32 -lwinmm -lpsapi
```

唯一的外部 DLL 依赖是 `libwinpthread-1.dll`。

## 仓库结构

```
src/          驱动源码（skimsrv_intf.cpp + DSP 流水线 + CyAPI）
LICENSE       MIT
NOTICE        第三方来源声明（ExtIO_sddc、Cypress CyAPI、固件）
```

## 常见问题

- 日志出现 `RX666.img MISSING` → 仅当 `HwModel=2`（RX666 硬件）时出现。
  从官方 RX666 驱动包复制该文件到同目录，重新插拔 USB，再 Start
- 日志出现 `WARNING: HwModel=RX888 but firmware is RX666 build` →
  设备里的固件与配置不符。断电重插 USB，让驱动重新下载内置的 RX888 固件
- 日志出现 `USB RECOVERY EXHAUSTED` → USB 链路已硬性故障，驱动已停止
  接收。拔插 USB 线重新上电，再重启 SkimSrv；若频繁出现，检查 USB 线缆、
  供电或集线器
- 完全没有 `Enumerate` 行 → Cypress FX3 驱动（CyUSB3.sys）未装好，或
  设备不在 USB 3.0 口上
- SkimSrv 目录只保留 `RX888Intf.dll` 这一个设备 DLL，其它设备驱动
  （如 `Qs1rIntf.dll`）应移出该目录

## 致谢

- Oscar Steila ik1xpv —— ExtIO_sddc / BBRF103 项目，MIT 许可
  （FX3handler、SDDC_FX3 固件镜像）
- Cypress Semiconductor —— CyAPI USB 库

---

# RX888Intf — RX888/RX666 driver DLL for CW Skimmer Server

Open-source SkimSrv device driver for the RX888 / RX666 (BBRF103-family)
wideband SDR receivers, focused on multi-band CW skimming. Built as a
32-bit DLL loaded by CW Skimmer Server (Afreet, 1.6.x).

Current release: **v4.208** (2026-11-24)

## Highlights

- Direct RF sampling pipeline rewritten from scratch: 64M IQ → per-channel
  DDC, no r2iq, no DC blocker
- AVX2/FMA vectorized hot path
- Correct-by-construction flow control: cross-block FIR history, ring-buffer
  overrun never silently drops data
- RX888 / RX666 support: the hardware model is set explicitly via `HwModel`
  in the ini
- Exact rate contract: the 192k channel delivers 192000 Hz (12/125 resampler)
- Automatic recovery from a half-dead USB link (up to 3 attempts); if
  recovery fails, reception stops cleanly with a loud alarm block
  ("USB RECOVERY EXHAUSTED") at the end of the log

## Quick start (users)

Download the release package (`RX888Intf_vX.Y.zip`), unzip and copy all
files into your SkimSrv.exe directory. No build required.

- RX888: works out of the box (RX888 firmware is embedded in the DLL);
  keep `HwModel=1` (the default)
- RX666: set `[RX888] HwModel=2` in the ini and additionally copy
  `RX666.img` from the official RX666 driver package into the same directory

Requirements: CW Skimmer Server 1.6.x (32-bit), Cypress FX3 USB driver
(CyUSB3.sys), USB 3.0 port.

## Repository layout

```
src/          driver source (skimsrv_intf.cpp + DSP pipeline + CyAPI)
LICENSE       MIT
NOTICE        third-party attribution (ExtIO_sddc, Cypress CyAPI, firmware)
```

## Troubleshooting

- Driver log: RX888New.log next to the DLL (plain text, appended)
- `RX666.img MISSING` → only when `HwModel=2` (RX666 hardware). Copy the
  file from the official RX666 driver package, replug USB, Start again
- `WARNING: HwModel=RX888 but firmware is RX666 build` → the firmware in
  the device does not match the configuration. Power-cycle the USB cable
  so the driver can download the built-in RX888 firmware
- `USB RECOVERY EXHAUSTED` → the USB link is hard-dead and reception has
  stopped. Unplug/replug the USB cable, then restart SkimSrv; if it happens
  often, check cable, power or hub
- No `Enumerate` lines at all → Cypress FX3 driver (CyUSB3.sys) not
  installed or device not on a USB 3.0 port
- Keep `RX888Intf.dll` as the only device DLL in the SkimSrv directory;
  move other device drivers (e.g. `Qs1rIntf.dll`) out of that directory

## Acknowledgements

- Oscar Steila ik1xpv — ExtIO_sddc / BBRF103 project, MIT licensed
  (FX3handler, SDDC_FX3 firmware image)
- Cypress Semiconductor — CyAPI USB library

> AI生成
