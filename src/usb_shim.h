// usb_shim.h - FX3handler USB 底座的 C 壳接口（供 skimsrv_intf.cpp 使用）
// 隔离 C++ 类（fx3handler/ringbuffer）与 DLL 导出层，简化编译单元依赖
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// 探测设备（可离线）：创建并打开 fx3handler，Enumerate 后保留对象
// 成功返回 true；无设备返回 false（对象保持有效可重试）
bool UsbTryOpen(void** dev, int32_t idx);

// 打开设备 + 下载固件 + STARTADC(adcFreq)。成功返回 true
bool UsbOpen(void** dev, uint32_t adcFreq);

// 关闭并释放设备
void UsbClose(void** dev);

// 创建 65536 样本 int16 环形缓冲（QUEUE_SIZE 块）并启动 USB 流线程
// ring 输出到 *ring（ringbuffer<int16_t>*）
bool UsbStartStream(void* dev, void** ring);

// 停止 USB 流（唤醒消费线程 + join 采集线程；ring 对象保留，不在此释放）
// v4.211: 返回 false 表示 ADC 线程 join 超时脱管——调用方不得再 UsbClose。
bool UsbStopStream(void* dev, void* ring);

// v4.211: 只置 ADC 线程的 run=false（不 join、不碰设备）。StopRx 第一动作。
void UsbRequestStop(void* dev);

// 释放环形缓冲（仅在消费线程退出后调用，见 skimsrv_intf StopRx 顺序）
void UsbFreeRing(void** ring);

// 阻塞取读指针（等价 ringbuffer::getReadPtr，空则等待新块/停止）
// 返回 int16_t*（65536 样本）
void* UsbPeekReadPtr(void* ring);

// 消费完成，推进读指针
void UsbReadDone(void* ring);

// v4.199: 环形缓冲"写端因满而等待"的累计次数——非零说明消费速度追不上
// USB 生产速度（实时率亏损），此时上游 FX3 缓冲溢出会静默丢样。
int UsbRingFullCount(void* ring);

// v4.204: 只发 STOPFX3，不 join 任何线程、不做阻塞等待。
// 仅供 DLL_PROCESS_DETACH 使用（进程已在退出，禁止长时间阻塞）。
void UsbEmergencyStop(void);

// v4.209: 在拆除主机端点之前先停 ADC（幂等、2 s 超时）。
// 现场实测：端点先拆再发 STOPFX3 会让控制传输卡满 10 s 并导致设备掉线。
void UsbStopAdc(void* dev);

// v4.209: 远程免插拔恢复（端口循环 → 父端口复位 → PnP 重新枚举）。
// 设备对象已释放或从未建立时仍会走 PnP 回退。成功返回 true。
bool UsbPortRecover(void* dev);

#ifdef __cplusplus
}
#endif