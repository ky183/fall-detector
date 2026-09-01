#pragma once

// ================= 腰端硬件引脚（ESP32-S3） =================
#define PIN_I2C_SDA         8   // MPU6050#2 + OLED 共用 I2C
#define PIN_I2C_SCL         9
#define PIN_BUZZER          4   // 无源蜂鸣器（经 S8050 驱动）
#define PIN_BTN_CANCEL      5   // 取消报警
#define PIN_BTN_WAKE        6   // 唤醒/自检

// ================= 设备 I2C 地址 =================
#define ADDR_MPU6050        0x68
#define ADDR_OLED           0x3C

// ================= 分级调试开关 =================
// 按 Step 逐步打开，减少定位问题的范围
#define ENABLE_MPU6050      1   // Step 1：传感器读取
#define ENABLE_OLED         1   // Step 4：显示
#define ENABLE_ESPNOW       1   // Step 3：无线互联
#define ENABLE_WIFI_PUSH    0   // Step 5：微信推送
#define ENABLE_LOW_POWER    0   // Step 6：低功耗

// ================= 系统参数 =================
#define SERIAL_BAUD         115200
