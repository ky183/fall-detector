#pragma once
// ============================================================
//  蜂鸣器（腰端）— 无源蜂鸣器，S8050 三极管驱动
//  报警音：间歇哔哔（周期/占空由 config.h 配置）
//  驱动方式：tone()/noTone()（ESP32 LEDC 封装，自动分配通道）
// ============================================================
#include <Arduino.h>

void buzzer_init();      // 引脚初始化（ENABLE_BUZZER=0 时静默）
void buzzer_start();     // 开始报警音（间歇模式由 alarm_manager 的 tick 驱动或内部定时）
void buzzer_stop();      // 停止发声
void buzzer_beep_on();   // 单个哔声开始（供间歇模式外部驱动）
void buzzer_beep_off();  // 单个哔声结束
