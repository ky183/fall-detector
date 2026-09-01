#pragma once

// ================= 腕端硬件引脚（XIAO ESP32C3） =================
// XIAO C3：D4 = GPIO6(SDA)，D5 = GPIO7(SCL)
#define PIN_I2C_SDA         6
#define PIN_I2C_SCL         7

// ================= 设备 I2C 地址 =================
#define ADDR_MPU6050        0x68

// ================= 分级调试开关 =================
#define ENABLE_MPU6050      1   // Step 1：传感器读取
#define ENABLE_ESPNOW       1   // Step 3：无线互联
#define ENABLE_LOW_POWER    0   // Step 6：低功耗

// ================= 系统参数 =================
#define SERIAL_BAUD         115200
