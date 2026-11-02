// dll_load_test.cpp - 验证 RX888New.dll 可加载且导出符号齐全
// 无硬件模式：GetSdrInfo 应返回 offline 名称，其他导出函数不崩溃
// 编译: g++ -O2 -std=c++17 dll_load_test.cpp -o dll_load_test.exe
#include <windows.h>
#include <cstdio>
#include <cstring>

// SkimSrv 接口类型（与 skimsrv_intf.cpp 一致）
typedef struct { float Re, Im; } Cmplx;
typedef Cmplx* CmplxA;
typedef CmplxA* CmplxAA;
typedef struct {
    char* DeviceName;
    int   MaxRecvCount;
    float ExactRates[3];
} SdrInfo, *PSdrInfo;
typedef void(__stdcall* tIQProc)(int, CmplxAA);
typedef void(__stdcall* tAudioProc)(int, CmplxA, CmplxA, int);
typedef void(__stdcall* tErrorProc)(int, char*);
typedef void(__stdcall* tStatusBitsProc)(int, unsigned char);
typedef void(__stdcall* tLoadProgressProc)(int, int, int);
typedef struct {
    int  THandle;
    int  RecvCount;
    int  RateID;
    BOOL LowLatency;
    tIQProc pIQProc;
    tAudioProc pAudioProc;
    tStatusBitsProc pStatusBitProc;
    tLoadProgressProc pLoadProgressProc;
    tErrorProc pErrorProc;
} SdrSettings, *PSdrSettings;

int main() {
    typedef void(__stdcall* pGetSdrInfo)(PSdrInfo);
    typedef void(__stdcall* pStartRx)(PSdrSettings);
    typedef void(__stdcall* pStopRx)();
    typedef void(__stdcall* pSetRxFrequency)(int, int);
    typedef void(__stdcall* pSetCtrlBits)(unsigned char);
    typedef int(__stdcall* pReadPort)(int);

    HMODULE h = LoadLibraryA("RX888New.dll");
    if (!h) {
        printf("LoadLibrary FAILED, error=%lu\n", GetLastError());
        return 1;
    }
    printf("LoadLibrary OK\n");

    pGetSdrInfo GetSdrInfo = (pGetSdrInfo)GetProcAddress(h, "GetSdrInfo");
    pStartRx    StartRx    = (pStartRx)GetProcAddress(h, "StartRx");
    pStopRx     StopRx     = (pStopRx)GetProcAddress(h, "StopRx");
    pSetRxFrequency SetRxFreq = (pSetRxFrequency)GetProcAddress(h, "SetRxFrequency");
    pSetCtrlBits SetCtrl  = (pSetCtrlBits)GetProcAddress(h, "SetCtrlBits");
    pReadPort    ReadPort  = (pReadPort)GetProcAddress(h, "ReadPort");
    if (!GetSdrInfo || !StartRx || !StopRx || !SetRxFreq || !SetCtrl || !ReadPort) {
        printf("GetProcAddress missing: %p %p %p %p %p %p\n",
               (void*)GetSdrInfo, (void*)StartRx, (void*)StopRx,
               (void*)SetRxFreq, (void*)SetCtrl, (void*)ReadPort);
        FreeLibrary(h);
        return 1;
    }
    printf("All 6 exports resolved OK\n");

    // GetSdrInfo（无硬件 → offline 名）
    SdrInfo info;
    memset(&info, 0, sizeof(info));
    GetSdrInfo(&info);
    printf("GetSdrInfo: name='%s' MaxRecv=%d rates=%.0f/%.0f/%.0f\n",
           info.DeviceName ? info.DeviceName : "(null)",
           info.MaxRecvCount, info.ExactRates[0], info.ExactRates[1], info.ExactRates[2]);

    // 空操作导出（不崩溃即可）
    SetCtrl(0);
    int rp = ReadPort(0);
    printf("SetCtrlBits/ReadPort no-crash OK (ReadPort=%d)\n", rp);

    // StartRx（无硬件：应走 UsbOpen 失败路径，不应崩溃）
    SdrSettings set;
    memset(&set, 0, sizeof(set));
    set.THandle = 1;
    set.RecvCount = 1;
    set.RateID = 1;
    StartRx(&set);
    printf("StartRx (no hw) returned without crash\n");
    StopRx();

    FreeLibrary(h);
    printf("DLL unload OK\n");
    printf("=== DLL LOAD TEST PASS ===\n");
    return 0;
}