#pragma once
// ============================================================
//  传感器管理（腰端）
//  职责：
//    1. 本地 MPU（#2）读取 → 写 g_local_data（裸寄存器驱动，兼容 6050/6500）
//    2. 存储远端（腕端）数据 g_remote_data（由 net_espnow 收包回调写入）
//
//  ★ 数据枢纽：检测任务（task_detect）从这里取两端数据快照，
//    组装 FallInput 交给算法模块 fall_detector
// ============================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// 统一传感器数据快照（与腕端 SensorData 同构）
struct SensorData {
    float ax, ay, az;      // 加速度 (m/s²)
    float gx, gy, gz;      // 角速度 (deg/s)
    float svm;             // 合加速度幅值 (m/s²)
    // TODO(算法)：pitch/roll 姿态角在此结构体扩展
    uint32_t ts;           // 采样时刻 millis()
};

// 检测算法的统一输入（本地 + 远端快照）
struct FallInput {
    SensorData local;      // 腰端本地 MPU
    SensorData remote;     // 腕端远端数据（hasRemote=false 时无效）
    bool hasRemote;        // 是否收到过腕端数据
    uint32_t remoteAgeMs;  // 最近一包远端数据距今（链路健康度）
};

// 全局共享数据（互斥锁保护，勿直接读写，用下面的函数/任务内加锁）
extern SensorData        g_local_data;
extern SensorData        g_remote_data;
extern SemaphoreHandle_t g_sensor_mutex;

// 初始化：创建互斥锁 + 配置本地 MPU（失败不阻塞，read 内自动重试）
bool sensor_manager_init(void);

// 读取一次本地 MPU → 写 g_local_data（task_sensor 周期调用）
bool sensor_manager_read(void);

// 写入远端数据（net_espnow 收包回调调用；锁被占时直接丢弃该包——20ms 后有下一包）
void sensor_manager_set_remote(const SensorData& d);

// 远端链路状态：是否收到过腕端数据 + 最近一包距今毫秒数
bool sensor_manager_has_remote(uint32_t* ageMs = nullptr);
