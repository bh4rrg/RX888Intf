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
void UsbStopStream(void* dev, void* ring);

// 释放环形缓冲（仅在消费线程退出后调用，见 skimsrv_intf StopRx 顺序）
void UsbFreeRing(void** ring);

// 阻塞取读指针（等价 ringbuffer::getReadPtr，空则等待新块/停止）
// 返回 int16_t*（65536 样本）
void* UsbPeekReadPtr(void* ring);

// 消费完成，推进读指针
void UsbReadDone(void* ring);

#ifdef __cplusplus
}
#endif