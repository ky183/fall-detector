#pragma once
// ============================================================
//  sensor_manager.h —— 宿主机回放用 **桩头文件**（不参与固件编译）
//
//  仅为让 algo/cnn/host_replay/replay.py 能在 PC 上用 g++ 编译真实的
//  firmware/waist_firmware/cnn_detector.cpp 而存在：
//  cnn_detector.h 里 #include "sensor_manager.h" 取的是同目录优先，
//  回放时把本文件与本目录的 main.cpp 一起复制进临时构建目录，
//  即可保证被编译的 cnn_detector.cpp 与上板的是**同一份源码**。
//
//  字段必须与 firmware/waist_firmware/sensor_manager.h 的
//  SensorData / FallInput 逐字段一致，否则回放结论无意义。
// ============================================================
#include <stdint.h>

struct SensorData {
    float ax, ay, az;      // 加速度 (m/s²)
    float gx, gy, gz;      // 角速度 (deg/s)
    float svm;             // 合加速度幅值 (m/s²)
    uint32_t ts;           // 采样时刻 millis()
};

struct FallInput {
    SensorData local;      // 腰端本地 MPU
    SensorData remote;     // 腕端远端数据（hasRemote=false 时无效）
    bool hasRemote;        // 是否收到腕端数据
    uint32_t remoteAgeMs;  // 最近一包远端数据距今（链路健康度）
};
