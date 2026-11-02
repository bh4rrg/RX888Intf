---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: 'a7905fa2-24c8-4dcd-a070-29a0cb2fd0ba'
  PropagateID: 'a7905fa2-24c8-4dcd-a070-29a0cb2fd0ba'
  ReservedCode1: 'e5d2a834-608f-4c59-9b05-6773fa195c93'
  ReservedCode2: 'e5d2a834-608f-4c59-9b05-6773fa195c93'
---

# Verification & benchmark tools

These harnesses were used during development to verify the DSP pipeline
numerically (bit-exact against the reference implementation), stress the
allocator and threading, and benchmark each pipeline stage.

They are development-time tools; end users do not need them. Compile with
the same MinGW-w64 32-bit toolchain as the driver (see ../src/build.bat).
Note: these sources include headers from ../src (e.g. polyphase.h,
ddc_engine.h); adjust include paths accordingly, e.g.
`g++ -I../src tools/verify_v4194.cpp ...`.

- `verify_v4194.cpp` — numerical verification of the v4.194 MixDecFIR /
  FracResampler rewrite against the v4.193 reference: full-block
  streaming, mixed block lengths (incl. 287/288/289 window-depth
  boundaries), truncation storms, reset consistency. Expect ALL PASS.
- `stress_ddc.cpp` — long-run stress (hundreds of thousands of blocks,
  200+ Start/Stop cycles, tens of thousands of hot re-inits) with heap
  verification; must finish with zero errors.
- `pipeline_bench_v4194.cpp` — per-stage micro-benchmarks
  (full / mixdec / frac / fir / rot / memcpy / split) for 1 and 8
  channels, for regression tracking of DSP performance.
- `dll_load_test.cpp` — minimal load test: LoadLibrary + 6-export
  resolution + FreeLibrary, for dependency-chain verification in an
  isolated directory (no USB access, no log writes).

> AI生成