// ============================================================
//  传感器管理（腕端）实现 — 裸寄存器驱动
//  背景：实测模块 WHO_AM_I=0x70（MPU6500），Adafruit_MPU6050 库
//        严格校验 0x68 而拒绝。MPU6500 寄存器布局与 MPU6050 兼容，
//        故直接寄存器读写，兼容 6050/6500/6515 全系。
//  参考：MPU-6000/6050 Register Map v4.2，MPU-6500 v2.1
// ============================================================
#include "sensor_manager.h"
#include "logger.h"
#include "config.h"
#include <math.h>
#include <Wire.h>

// ---- MPU 寄存器地址（6050/6500 兼容） ----
#define REG_SMPLRT_DIV   0x19  // 采样率分频
#define REG_CONFIG       0x1A  // DLPF 数字低通滤波
#define REG_GYRO_CFG     0x1B  // 陀螺仪量程
#define REG_ACCEL_CFG    0x1C  // 加速度计量程
#define REG_DATA         0x3B  // 数据区起始：axH..azL, tH tL, gxH..gzL 共 14 字节
#define REG_PWR_MGMT_1   0x6B  // 电源管理（睡眠/时钟源）
#define REG_WHO_AM_I     0x75  // 芯片 ID

// ---- 量程换算系数（与下方配置对应：±8g / ±500 deg/s） ----
static const float ACCEL_LSB = (8.0f * 9.80665f) / 32768.0f;  // m/s²/LSB
static const float GYRO_LSB  = 500.0f / 32768.0f;             // deg/s/LSB

// ---- 模块状态 ----
static bool    s_mpu_ok   = false;
static uint8_t s_chip_id  = 0;

// ---- 全局共享数据（数据枢纽，见 sensor_manager.h 说明） ----
SensorData        g_sensor_data = {};
SemaphoreHandle_t g_sensor_mutex = nullptr;

// ---- 基础寄存器访问 ----
static void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t mpu_read(uint8_t reg) {
    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(reg);
    Wire.endTransmission(false);              // repeated start
    Wire.requestFrom(ADDR_MPU6050, 1);
    return Wire.available() ? Wire.read() : 0xFF;
}

bool sensor_manager_init(void) {
    if (g_sensor_mutex == nullptr) {
        g_sensor_mutex = xSemaphoreCreateMutex();
    }

#if ENABLE_MPU6050
    // 探测：宽容校验（0x00/0xFF = 总线无应答）
    s_chip_id = mpu_read(REG_WHO_AM_I);
    if (s_chip_id == 0x00 || s_chip_id == 0xFF) {
        LOG_E("SENS", "MPU not responding (id=0x%02X)", s_chip_id);
        return false;
    }
    LOG_I("SENS", "MPU chip ID=0x%02X (%s)", s_chip_id,
          s_chip_id == 0x68 ? "MPU6050" :
          s_chip_id == 0x70 ? "MPU6500" :
          s_chip_id == 0x71 ? "MPU6515" : "compatible chip");

    // 唤醒与配置（6050/6500 通用）
    mpu_write(REG_PWR_MGMT_1, 0x01);   // 退出睡眠，时钟=陀螺 PLL（比内部 RC 准）
    mpu_write(REG_SMPLRT_DIV, 0x04);   // 内部采样 1kHz/(1+4)=200Hz，固件 50Hz 读取留裕量
    mpu_write(REG_CONFIG, 0x04);       // DLPF≈20Hz，抑制高频抖动
    mpu_write(REG_ACCEL_CFG, 0x10);    // 加速度 ±8g（跌倒冲击可达数 g）
    mpu_write(REG_GYRO_CFG, 0x08);     // 陀螺 ±500 deg/s
    delay(10);                         // 等配置生效
    return true;
#else
    return true;
#endif
}

bool sensor_manager_read(void) {
#if ENABLE_MPU6050
    // ---- 真实传感器路径 ----
    if (!s_mpu_ok) {
        // 离线重连：不阻塞其他任务，每周期只尝试一次 init
        s_mpu_ok = sensor_manager_init();
        if (!s_mpu_ok) return false;
    }

    // 连续读 14 字节（ax ay az temp gx gy gz，各 16 位大端）
    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(REG_DATA);
    if (Wire.endTransmission(false) != 0) {       // 总线错误 → 标记离线待重连
        s_mpu_ok = false;
        return false;
    }
    uint8_t buf[14];
    if (Wire.requestFrom(ADDR_MPU6050, 14) != 14) {
        s_mpu_ok = false;
        return false;
    }
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();
    // buf[6..7] 是温度，本项目暂不使用，跳过

    SensorData d;
    d.ax = (int16_t)((buf[0] << 8) | buf[1])  * ACCEL_LSB;
    d.ay = (int16_t)((buf[2] << 8) | buf[3])  * ACCEL_LSB;
    d.az = (int16_t)((buf[4] << 8) | buf[5])  * ACCEL_LSB;
    d.gx = (int16_t)((buf[8] << 8) | buf[9])  * GYRO_LSB;
    d.gy = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_LSB;
    d.gz = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_LSB;
    d.svm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
    d.ts  = millis();

    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10))) {
        g_sensor_data = d;
        xSemaphoreGive(g_sensor_mutex);
        return true;
    }
    LOG_E("SENS", "mutex timeout");   // 拿不到锁说明下游卡死，优先排查显示任务
    return false;

#else
    // ---- 模拟数据路径（ENABLE_MPU6050=0）----
    // 静止 + 缓慢正弦摆动，用于无硬件时调试 OLED / 通信链路
    uint32_t now = millis();
    float s = sinf(now * 0.001f);
    SensorData d;
    d.ax = 0.3f * s;  d.ay = -0.5f * s;  d.az = 9.8f;
    d.gx = 15.0f * s; d.gy = -8.0f * s;  d.gz = 3.0f * s;
    d.svm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
    d.ts  = now;
    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10))) {
        g_sensor_data = d;
        xSemaphoreGive(g_sensor_mutex);
    }
    return true;
#endif
}
