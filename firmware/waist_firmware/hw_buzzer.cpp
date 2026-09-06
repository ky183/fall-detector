// ============================================================
//  蜂鸣器（腰端）实现
//  无源蜂鸣器需要方波驱动：tone(pin, freq) 输出 50% 方波
//  声响模式由 alarm_manager 控制（哔哔间歇），本模块只管发声
// ============================================================
#include "hw_buzzer.h"
#include "config.h"

void buzzer_init(void) {
#if ENABLE_BUZZER
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);   // 确保上电不响
#endif
}

void buzzer_beep_on(void) {
#if ENABLE_BUZZER
    tone(PIN_BUZZER, BUZZER_FREQ_HZ);
#endif
}

void buzzer_beep_off(void) {
#if ENABLE_BUZZER
    noTone(PIN_BUZZER);
#endif
}

// 连续模式（备用，当前报警用间歇模式由 alarm_manager 驱动）
void buzzer_start(void) { buzzer_beep_on(); }
void buzzer_stop(void)  { buzzer_beep_off(); }
