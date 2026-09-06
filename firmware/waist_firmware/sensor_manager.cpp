// ============================================================
//  传感器管理（腰端）实现 — 裸寄存器驱动
//  与腕端 sensor_manager.cpp 同款驱动（实测模块可能是 MPU6500，
//  WHO_AM_I=0x70，寄存器与 MPU6050 兼容），另含远端数据存储
//  踩坑记录见 docs/notes/mpu6500_driver_note.md
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
#define REG_DATA         0x3B  // 数据区起始（14 字节）
#define REG_PWR_MGMT_1   0x6B  // 电源管理
#define REG_WHO_AM_I     0x75  // 芯片 ID

// ---- 量程换算系数（±8g / ±500 deg/s） ----
static const float ACCEL_LSB = (8.0f * 9.80665f) / 32768.0f;  // m/s²/LSB
static const float GYRO_LSB  = 500.0f / 32768.0f;             // deg/s/LSB

// ---- 全局共享数据（local=腰端 MPU，remote=腕端无线数据） ----
SensorData        g_local_data  = {};
SensorData        g_remote_data = {};
SemaphoreHandle_t g_sensor_mutex = nullptr;

// ---- 远端数据时效（供 FallInput.remoteAgeMs） ----
static volatile uint32_t s_remote_last_ms = 0;
static volatile bool     s_has_remote     = false;

static bool s_mpu_ok = false;

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
    uint8_t id = mpu_read(REG_WHO_AM_I);
    if (id == 0x00 || id == 0xFF) {
        LOG_E("SENS", "MPU not responding (id=0x%02X)", id);
        return false;
    }
    LOG_I("SENS", "MPU chip ID=0x%02X (%s)", id,
          id == 0x68 ? "MPU6050" :
          id == 0x70 ? "MPU6500" :
          id == 0x71 ? "MPU6515" : "compatible chip");

    mpu_write(REG_PWR_MGMT_1, 0x01);   // 唤醒，时钟=陀螺 PLL
    mpu_write(REG_SMPLRT_DIV, 0x04);   // 内部采样 200Hz
    mpu_write(REG_CONFIG, 0x04);       // DLPF≈20Hz
    mpu_write(REG_ACCEL_CFG, 0x10);    // ±8g
    mpu_write(REG_GYRO_CFG, 0x08);     // ±500 deg/s
    delay(10);
    return true;
#else
    return true;
#endif
}

bool sensor_manager_read(void) {
#if ENABLE_MPU6050
    if (!s_mpu_ok) {
        s_mpu_ok = sensor_manager_init();
        if (!s_mpu_ok) return false;
    }

    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(REG_DATA);
    if (Wire.endTransmission(false) != 0) {
        s_mpu_ok = false;
        return false;
    }
    uint8_t buf[14];
    if (Wire.requestFrom(ADDR_MPU6050, 14) != 14) {
        s_mpu_ok = false;
        return false;
    }
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();
    // buf[6..7] 是温度，暂不使用

    SensorData d;
    d.ax = (int16_t)((buf[0] << 8) | buf[1])   * ACCEL_LSB;
    d.ay = (int16_t)((buf[2] << 8) | buf[3])   * ACCEL_LSB;
    d.az = (int16_t)((buf[4] << 8) | buf[5])   * ACCEL_LSB;
    d.gx = (int16_t)((buf[8] << 8) | buf[9])   * GYRO_LSB;
    d.gy = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_LSB;
    d.gz = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_LSB;
    d.svm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
    d.ts  = millis();

    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10))) {
        g_local_data = d;
        xSemaphoreGive(g_sensor_mutex);
        return true;
    }
    return false;
#else
    return true;   // 本地 MPU 关闭：检测自动降级为纯远端模式
#endif
}

void sensor_manager_set_remote(const SensorData& d) {
    if (xSemaphoreTake(g_sensor_mutex, 0)) {   // 不等待：回调上下文禁止阻塞
        g_remote_data = d;
        xSemaphoreGive(g_sensor_mutex);
        s_remote_last_ms = millis();
        s_has_remote = true;
    }
    // 拿不到锁则丢弃本包：50Hz 流，丢一帧无碍
}

bool sensor_manager_has_remote(uint32_t* ageMs) {
    if (ageMs) *ageMs = s_has_remote ? (millis() - s_remote_last_ms) : 0;
    return s_has_remote;
}
