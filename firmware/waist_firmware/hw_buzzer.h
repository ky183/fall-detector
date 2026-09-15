#pragma once
// ============================================================
//  蜂鸣器（腰端）— 有源蜂鸣器模块（高电平响，板载 S8050）
//  报警音：间歇哔哔（节奏/两段式参数在 config.h，由 alarm_manager 驱动）
//  驱动方式：digitalWrite 高/低电平开关（有源内置振荡源，无需 PWM）
// ============================================================
#include <Arduino.h>

void buzzer_init();      // 引脚初始化（ENABLE_BUZZER=0 时静默）
void buzzer_start();     // 开始报警音（连续模式，备用）
void buzzer_stop();      // 停止发声
void buzzer_beep_on();   // 单个哔声开始（供间歇模式外部驱动）
void buzzer_beep_off();  // 单个哔声结束
