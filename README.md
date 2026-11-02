---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: 'c33577a0-e117-4acf-a37f-96c5d1c11d25'
  PropagateID: 'c33577a0-e117-4acf-a37f-96c5d1c11d25'
  ReservedCode1: 'b4a0b822-a7e4-4362-b54e-20d563296ee3'
  ReservedCode2: 'b4a0b822-a7e4-4362-b54e-20d563296ee3'
---

# RX888Intf — RX888/RX666 driver DLL for CW Skimmer Server

Open-source SkimSrv device driver for the RX888 / RX666 (BBRF103-family)
wideband SDR receivers, focused on multi-band CW skimming. Built as a
32-bit DLL loaded by CW Skimmer Server (Afreet, 1.6.x).

Current release: **v4.194** (2026-10-30)

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
- No "Enumerate" lines at all → Cypress FX3 driver (CyUSB3.sys) not
  installed or device not on a USB 3.0 port
- Never put unrelated DLLs into the SkimSrv directory: SkimSrv scans and
  loads DLLs found there

## Acknowledgements

- Oscar Steila ik1xpv — ExtIO_sddc / BBRF103 project, MIT licensed
  (FX3handler, SDDC_FX3 firmware image)
- Cypress Semiconductor — CyAPI USB library

> AI生成