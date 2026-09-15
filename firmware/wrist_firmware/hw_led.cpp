// ============================================================
//  状态指示 LED 实现 — 闪烁含义见 hw_led.h 头注释
//  ENABLE_LED=0 时全部为占位空实现（编译通过、不控制引脚）
// ============================================================
#include "hw_led.h"
#include "config.h"
#include <Arduino.h>

#if ENABLE_LED
// ---- 内部状态 ----
// 仅被 task_sensor/task_tx（上报）和 task_led（读取）访问；
// 单字节读写原子性由硬件保证，50Hz 上报频率下无需加锁
static volatile bool s_mpu_ok      = true;   // 上电默认正常，避免首包前误报
static volatile int  s_tx_fail_cnt = 0;      // 连续发送失败计数
static const int     TX_FAIL_LIMIT = 10;     // 连续失败 10 次（约 0.2s）判链路故障
#endif

void led_init(void) {
#if ENABLE_LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);    // 默认熄灭
#endif
}

void led_report_mpu(bool ok) {
#if ENABLE_LED
    s_mpu_ok = ok;
#else
    (void)ok;
#endif
}

void led_report_tx(bool ok) {
#if ENABLE_LED
    if (ok) {
        s_tx_fail_cnt = 0;
    } else if (s_tx_fail_cnt < TX_FAIL_LIMIT) {
        s_tx_fail_cnt++;
    }
#else
    (void)ok;
#endif
}

void led_tick(void) {
#if ENABLE_LED
    // 优先级：MPU 离线（快闪）> 链路故障（慢闪）> 正常（心跳）
    uint32_t period, onMs;
    if (!s_mpu_ok) {
        period = 200;  onMs = 100;    // 快闪 5Hz：传感器故障
    } else if (s_tx_fail_cnt >= TX_FAIL_LIMIT) {
        period = 1000; onMs = 100;    // 慢闪 1Hz：链路故障
    } else {
        period = 2000; onMs = 100;    // 心跳：每 2s 短闪一次（醒目且省电）
    }
    digitalWrite(PIN_LED, (millis() % period) < onMs ? HIGH : LOW);
#endif
}
