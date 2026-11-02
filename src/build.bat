@echo off
REM ============================================================
REM  RX888Intf.dll build script (v4.191+)
REM  IRON RULE (v4.193): -march=x86-64-v3 -ffast-math are MANDATORY.
REM  -ffast-math: GCC strict FP forbids reduction vectorization;
REM    losing it causes 8x regression (vfmadd/AVX2 -> x87 fmul/faddp).
REM  -march=x86-64-v3: AVX2+FMA standard baseline (Ryzen 3700X and mobile
REM    Core i7-8559U both support). Replaces -march=native (v4.192): znver2 tuning was
REM    34% SLOWER than v3 on 3700X in firbench, and native tied the DLL
REM    to one microarch (fragile cross-vendor deployment).
REM  Usage: build.bat [output.dll]   default: RX888New_o3_fast.dll
REM  Deploy: stop SkimSrv first (DLL is locked while running),
REM          backup old DLL with timestamp, copy, restart, check log.
REM ============================================================
setlocal
set PATH=C:\msys64\mingw32\bin;%PATH%
set OUT=%~1
if "%OUT%"=="" set OUT=RX888New_o3_fast.dll

where g++.exe >nul 2>&1
if errorlevel 1 (
  echo [ERROR] g++.exe not found in C:\msys64\mingw32\bin
  exit /b 1
)

echo [BUILD] %OUT%  (-O3 -march=x86-64-v3 -ffast-math)
g++.exe -shared -O3 -march=x86-64-v3 -ffast-math -o %OUT% skimsrv_intf.cpp usb_shim.cpp FX3handler.cpp CyAPI.cpp RX888New.def -static-libgcc -static-libstdc++ -lsetupapi -lwinmm -lpsapi
if errorlevel 1 (
  echo [ERROR] build FAILED
  exit /b 1
)

echo [OK] built %OUT%
for %%F in (%OUT%) do echo [SIZE] %%~zF bytes
echo.
echo [HINT] verify SIMD: objdump -d -M intel %OUT% ^| findstr /c:"vfmadd"  (must be ^>0 in WorkerTramp; x87 fmul/faddp should be ~0)
endlocal
