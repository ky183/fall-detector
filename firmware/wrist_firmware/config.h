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
#define PIN_BTN_1           3       // 交互按钮（丝印 D2/GPIO3），另一端接 GND，内部上拉
                                    // 注：原定 D1(GPIO2)，实测该脚虚焊无响应，2026-09-07 迁移到 D2
                                    // 短按 = OLED 翻页/交互；长按 = 模拟跌倒（调试用）
#define PIN_LED             8       // 状态指示 LED（PCB 上 LED1，丝印 D9/GPIO8，经 1kΩ 限流，高电平点亮）
                                    // 闪烁含义见 hw_led.h：心跳=正常 慢闪=链路故障 快闪=MPU离线
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
#define BTN_LONG_PRESS_MS   1000    // 长按判定阈值（演示方便触发，维持 1s）

// ================= ESP-NOW 通信 =================
// ★必须与腰端 ESPNOW_CHANNEL 一致；腰端常连WiFi模式下该值还须等于热点信道
//   （2026-09-10 改 6 以匹配 vivo 热点实测信道，腰端串口有失配告警）
#define ESPNOW_CHANNEL      6       // WiFi 信道（有效范围 1~13），两端必须一致
// 腰端 MAC 地址：占位为广播地址（腰端上线后，用其串口打印的实际 MAC 替换）
// 广播模式下腰端也能收到，仅少一层地址过滤，Step 3 联调时改为单播
#define WAIST_MAC           {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// ================= 分级调试开关 =================
#define LOG_LEVEL           3       // 0=关 1=错误 2=信息 3=调试（含心跳日志）

#define ENABLE_MPU6050      1       // 0 = 不读真实 MPU，填充模拟数据（无硬件也能调显示/通信）
#define ENABLE_OLED         1       // 0 = 不初始化 OLED（屏没接/坏时调试其他模块）
                                    // 屏不在时 init 自动失败但不阻塞，其余功能照常
                                    // 正常页布局由 DEBUG_DISPLAY 切换（见下）
#define DEBUG_DISPLAY       0       // 1 = OLED 正常页用调试布局（SVM 大字+事件计数）
                                    // 0 = 产品布局（时长大字+状态/链路）
                                    // 报警弹窗/已推送/已取消页不受此开关影响（始终显示）
#define ENABLE_LED          1       // 0 = 不控制状态 LED（LED 未焊/调其他模块时）
#define ENABLE_BUTTON       1       // 0 = 不扫描按钮
#define ENABLE_ESPNOW       1       // 0 = 不初始化无线（未烧腰端/无天线时调试本地功能）
                                    // 注意：XIAO ESP32S3 需外接 U.FL 天线，不接也能收发但距离骤降

#define ENABLE_FALL_DETECT  0       // 跌倒判定算法（组内算法成员负责，预留）
                                    // 0 = 按钮长按可模拟跌倒事件，用于调通报警链路

// ================= 系统参数 =================
#define SERIAL_BAUD         115200
