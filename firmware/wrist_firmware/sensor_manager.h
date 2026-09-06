#pragma once
// ============================================================
//  传感器管理（腕端）
//  职责：MPU6050 读取 → 计算特征(SVM) → 写入全局共享数据
//
//  ★ 全局共享数据 g_sensor_data 是数据枢纽：
//    - 发送任务（task_tx）从这里取数 → ESP-NOW 发给腰端
//    - 显示任务（task_display）从这里取数 → OLED
//    - 跨任务读写必须持有 g_sensor_mutex（互斥锁）
//
//  ★ 跌倒判定（算法成员负责）不在此文件内：
//    腰端的算法代码同样从共享数据取 SensorData，不碰硬件
// ============================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// 统一传感器数据快照（腕/腰两端同构，字段与 SensorPayload 对应）
struct SensorData {
    float ax, ay, az;      // 加速度 (m/s²)
    float gx, gy, gz;      // 角速度 (deg/s)
    float svm;             // 合加速度幅值 (m/s²)
    // TODO(算法)：pitch/roll 姿态角在此结构体扩展，由算法成员自行添加
    uint32_t ts;           // 采样时刻 millis()
};

// 全局共享数据（外部通过 sensor_lock()/sensor_unlock() 访问）
extern SensorData        g_sensor_data;
extern SemaphoreHandle_t g_sensor_mutex;

// 初始化：创建互斥锁 + 配置 MPU（量程/滤波）
// 返回 false = MPU 不在线（任务内会自动重试，接上线即恢复）
bool sensor_manager_init();

// 读取一次 MPU 并写入共享数据（task_sensor 以 SAMPLE_HZ 周期调用）
// ENABLE_MPU6050=0 时填充模拟数据（正弦摆动），无硬件也能调试下游模块
bool sensor_manager_read();
