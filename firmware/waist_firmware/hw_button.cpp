// ============================================================
//  取消报警按钮（腰端）实现
// ============================================================
#include "hw_button.h"
#include "config.h"

#define DEBOUNCE_COUNT 2   // 连续 N 次采样一致才确认

void button_init(void) {
#if ENABLE_BUTTON
    pinMode(PIN_BTN_CANCEL, INPUT_PULLUP);   // 按下 = LOW
#endif
}

CancelBtnEvent button_poll(void) {
#if !ENABLE_BUTTON
    return CBTN_NONE;
#else
    static bool    lastRaw = false;
    static bool    stable  = false;    // true = 按下
    static uint8_t sameCnt = 0;

    bool raw = (digitalRead(PIN_BTN_CANCEL) == LOW);

    if (raw != lastRaw) {         // 电平变化，重新计数
        lastRaw = raw;
        sameCnt = 0;
    } else if (sameCnt < 255) {
        sameCnt++;
    }

    if (sameCnt >= DEBOUNCE_COUNT && raw != stable) {
        stable = raw;
        if (stable) return CBTN_PRESSED;   // 只报"按下"事件
    }
    return CBTN_NONE;
#endif
}
