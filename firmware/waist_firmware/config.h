#pragma once
// ============================================================
//  腰端全局配置 — ESP32-S3 大板（ESP32-S3-WROOM-1-N16R8 课程板）
//  所有引脚、参数、功能开关集中于此，改一处全局生效
//  分段调试原则：哪个模块没接好/没到货，把对应 ENABLE 置 0 即可
// ============================================================

// ================= 硬件引脚（大板排针 H4） =================
#define PIN_I2C_SDA         8       // MPU6050#2，排针 H4 第12脚，开漏需上拉
#define PIN_I2C_SCL         9       // 排针 H4 第15脚
#define PIN_BUZZER          4       // 无源蜂鸣器（经 S8050 驱动，基极串 1kΩ），H4 第4脚
#define PIN_BTN_CANCEL      5       // 取消报警按钮，H4 第5脚，另一端接 GND

// ================= 大板注意事项（烧录/供电） =================
// 烧录口：Micro USB（CH340 串口），不是 Type-C
// 供电：Type-C/DC5521，输入必须 ≥6.65V，5V 充电宝直连 Type-C 无法启动
// 烧录选项：板选 ESP32S3 Dev Module，USB CDC in Boot = Disable

// ================= 设备 I2C 地址 =================
#define ADDR_MPU6050        0x68    // 腰端 MPU（实测可能是 MPU6500，驱动兼容）

// ================= ESP-NOW 通信 =================
#define ESPNOW_CHANNEL      1       // 与腕端一致（有效范围 1~13）
// 腕端 MAC：当前不做过滤（按包内 devId 过滤），联调稳定后可改单播

// ================= 报警参数 =================
#define ALARM_CANCEL_WINDOW_MS   10000   // 触发报警后的取消窗口（毫秒）
#define BUZZER_FREQ_HZ           2700    // 无源蜂鸣器谐振频率（响度最大）
#define ALARM_BEEP_ON_MS         300     // 哔声时长（间歇模式）
#define ALARM_BEEP_PERIOD_MS     500     // 哔声周期（300 响 + 200 停）

// ================= 采样与任务参数 =================
#define SAMPLE_HZ           50      // 本地 MPU 采样率
#define BUTTON_POLL_MS      50      // 按钮扫描周期
#define ALARM_TICK_MS       100     // 报警状态机步进周期
#define STAT_PERIOD_MS      1000    // 接收统计打印周期

// ================= 分级调试开关 =================
#define LOG_LEVEL           3       // 0=关 1=错误 2=信息 3=调试

#define ENABLE_MPU6050      1       // 0 = 不读本地 MPU（只收腕端数据调试链路）
#define ENABLE_ESPNOW       1       // 0 = 不初始化无线（单板调试蜂鸣器/按钮）
#define ENABLE_BUZZER       1       // 0 = 不响（调试时防止吵）
#define ENABLE_BUTTON       1       // 0 = 不扫描按钮

#define ENABLE_FALL_DETECT  0       // ★跌倒判定算法（算法组负责）
                                    // 0 = 算法未启用，仅腕端"长按模拟跌倒"可触发报警
#define ENABLE_WIFI_PUSH    0       // 微信推送（Step 5 实现，当前为占位）

// ================= 系统参数 =================
#define SERIAL_BAUD         115200
