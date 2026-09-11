#pragma once
// ============================================================
//  Arduino.h —— 宿主机回放用 **桩头文件**（不参与固件编译）
//
//  只为让 algo/cnn/host_replay/ 下的驱动能在 PC 上编译真实的固件模块：
//    fusion_main.cpp  -> fall_detector.cpp + cnn_detector.cpp
//                     -> alarm_manager.cpp + hw_button.cpp + hw_buzzer.cpp
//                        + net_pusher.cpp（无 secrets.h 时走占位实现）
//  只补这些模块真正用到的那几个符号。
//  时间由测试驱动显式推进（g_millis），保证回放可复现。
// ============================================================
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

// 被测代码只读 millis()，由驱动在每次 50Hz 调用前推进。
// 用 C++17 inline 变量：多个被测 .cpp 都 include 本头文件，仍只有一个实例，
// 不用每个驱动各自写一份定义（以前的 extern + 驱动定义方式很容易漏）
inline uint32_t g_millis = 0;
static inline uint32_t millis() { return g_millis; }
static inline void delay(uint32_t ms) { g_millis += ms; }

struct SerialShim {
    void printf(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
    }
    void println() { printf("\n"); }
};
inline SerialShim Serial;

#define degrees(rad)  ((rad) * 57.29577951308232f)
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// ---- GPIO / 蜂鸣器（hw_button.cpp / hw_buzzer.cpp 用） ----
// 回放时不接硬件，只记录调用次数与最后电平，供断言检查
#define INPUT_PULLUP  0x05
#define OUTPUT        0x03
#define LOW           0x0
#define HIGH          0x1

inline int g_pin_mode_calls    = 0;
inline int g_tone_calls        = 0;
inline int g_notone_calls      = 0;
inline int g_last_digital_read = 1;   // 1=松开，0=按下；由驱动设置

static inline void pinMode(int, int) { g_pin_mode_calls++; }
static inline int  digitalRead(int) { return g_last_digital_read; }
static inline void digitalWrite(int, int) {}
static inline void tone(int, unsigned int) { g_tone_calls++; }
static inline void noTone(int) { g_notone_calls++; }
