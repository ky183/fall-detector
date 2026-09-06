// ============================================================
//  按钮输入（腕端）实现
//  去抖策略：连续 2 个采样周期（约 100ms）读到相同新电平才认状态变化
//  长按在按住满 BTN_LONG_PRESS_MS 时立即触发（不等松手）
// ============================================================
#include "ui_button.h"
#include "config.h"

#define DEBOUNCE_COUNT 2   // 连续 N 次采样一致才确认（50ms 周期 x 2 = 100ms 容忍）

void button_init(void) {
#if ENABLE_BUTTON
    pinMode(PIN_BTN_1, INPUT_PULLUP);   // 按下 = LOW
#endif
}

ButtonEvent button_poll(void) {
#if !ENABLE_BUTTON
    return BTN_NONE;
#else
    // —— 状态机变量（static：跨调用保持） ——
    static bool     stable    = false;   // 稳定电平状态：true = 按下
    static bool     lastRaw   = false;   // 上次原始采样
    static uint8_t  sameCnt   = 0;       // 连续相同采样计数
    static uint32_t pressStart = 0;      // 本次按下时刻
    static bool     longFired = false;   // 本次按压是否已触发长按

    bool raw = (digitalRead(PIN_BTN_1) == LOW);

    // 去抖：电平变化时清零计数，连续稳定 DEBOUNCE_COUNT 次才确认
    if (raw != lastRaw) {
        lastRaw = raw;
        sameCnt = 0;
    } else if (sameCnt < 255) {
        sameCnt++;
    }

    // 确认状态变化（稳定的新电平）
    if (sameCnt >= DEBOUNCE_COUNT && raw != stable) {
        stable = raw;
        if (stable) {                    // 按下沿
            pressStart = millis();
            longFired  = false;
        } else {                         // 松开沿
            if (!longFired) return BTN_SHORT;   // 未触发过长按 → 短按
        }
    }

    // 长按判定（按住期间轮询）
    if (stable && !longFired &&
        millis() - pressStart >= BTN_LONG_PRESS_MS) {
        longFired = true;
        return BTN_LONG;
    }

    return BTN_NONE;
#endif
}
