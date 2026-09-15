// ============================================================
//  蜂鸣器（腰端）实现 — 有源蜂鸣器模块（高电平触发）
//  有源 = 内置振荡源，通电即响（固定音调），无需 tone()/PWM。
//  声响节奏（两段式间歇哔哔）由 alarm_manager 的 tick 驱动，
//  本模块只管"响/不响"两个原语。
//  历史：无源蜂鸣器 + tone(2700Hz) 方案已于 2026-09-13 随有源
//  模块到货作废（BUZZER_FREQ_HZ 一并移除）。
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
    digitalWrite(PIN_BUZZER, HIGH);  // 高电平 = 响
#endif
}

void buzzer_beep_off(void) {
#if ENABLE_BUZZER
    digitalWrite(PIN_BUZZER, LOW);   // 低电平 = 停
#endif
}

// 连续模式（备用，当前报警用间歇模式由 alarm_manager 驱动）
void buzzer_start(void) { buzzer_beep_on(); }
void buzzer_stop(void)  { buzzer_beep_off(); }
