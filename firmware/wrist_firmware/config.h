#pragma once
// ============================================================
//  腕端全局配置 — XIAO ESP32S3
//  所有引脚、参数、功能开关集中于此，改一处全局生效
//  分段调试原则：哪个模块没接好/没到货，把对应 ENABLE 置 0 即可
// ============================================================

// ================= 硬件引脚 =================
// XIAO S3：D4 = GPIO5(SDA)，D5 = GPIO6(SCL)
// 依据官方板卡包 variants/XIAO_ESP32S3/pins_arduino.h
#define PIN_I2C_SDA         5       // OLED + MPU6050 共用 I2C 总线
#define PIN_I2C_SCL         6
#define PIN_BTN_1           2       // 交互按钮（丝印 D1），另一端接 GND，内部上拉
                                    // 短按 = OLED 翻页/交互；长按 = 模拟跌倒（调试用）
                                    // 取消报警按钮在腰端（与蜂鸣器同端，不经无线，最可靠）
// 电池直接焊 BAT+/BAT- 焊盘，不占 GPIO

// ================= 设备 I2C 地址 =================
#define ADDR_MPU6050        0x68    // GY-521 默认（AD0 悬空）
#define ADDR_OLED           0x3C    // SSD1306 默认

// ================= OLED 参数 =================
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_RESET          -1      // 无独立复位脚

// ================= 采样与任务参数 =================
#define SAMPLE_HZ           50      // MPU 采样率（50Hz = 20ms 一次）
#define BUTTON_POLL_MS      50      // 按钮扫描周期（去抖由 ui_button 内部处理）
#define DISPLAY_PERIOD_MS   500     // OLED 刷新周期（慢速，省总线）
#define BTN_LONG_PRESS_MS   1000    // 长按判定阈值

// ================= ESP-NOW 通信 =================
#define ESPNOW_CHANNEL      1       // WiFi 信道（有效范围 1~13），两端必须一致
// 腰端 MAC 地址：占位为广播地址（腰端上线后，用其串口打印的实际 MAC 替换）
// 广播模式下腰端也能收到，仅少一层地址过滤，Step 3 联调时改为单播
#define WAIST_MAC           {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// ================= 分级调试开关 =================
#define LOG_LEVEL           3       // 0=关 1=错误 2=信息 3=调试（含心跳日志）

#define ENABLE_MPU6050      1       // 0 = 不读真实 MPU，填充模拟数据（无硬件也能调显示/通信）
#define ENABLE_OLED         1       // 0 = 不初始化 OLED（屏没接/坏时调试其他模块）
#define ENABLE_BUTTON       1       // 0 = 不扫描按钮
#define ENABLE_ESPNOW       1       // 0 = 不初始化无线（未烧腰端/无天线时调试本地功能）
                                    // 注意：XIAO ESP32S3 需外接 U.FL 天线，不接也能收发但距离骤降

#define ENABLE_FALL_DETECT  0       // 跌倒判定算法（组内算法成员负责，预留）
                                    // 0 = 按钮长按可模拟跌倒事件，用于调通报警链路

// ================= 系统参数 =================
#define SERIAL_BAUD         115200
