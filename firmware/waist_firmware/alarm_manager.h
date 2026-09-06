#pragma once
// ============================================================
//  报警管理（腰端）— 核心状态机
//
//    NORMAL ──触发(算法/模拟)──▶ WAIT_CANCEL ──10s超时──▶ SENT
//       ▲                          │                          │
//       └──────────── 按钮按下 ────────────────────────────────┘
//
//    WAIT_CANCEL：哔哔报警中，等待按钮取消
//    SENT：已过取消窗口，微信推送已发（占位），继续响直到按钮
// ============================================================
#include <Arduino.h>

enum AlarmState {
    ST_NORMAL      = 0,   // 正常监测
    ST_WAIT_CANCEL = 1,   // 报警中，取消窗口内
    ST_SENT        = 2,   // 报警中，已推送（窗口已过）
};

void alarm_init(void);               // 蜂鸣器静音初始化
void alarm_trigger(const char* src); // 触发报警（src: "algo"=算法 / "sim"=模拟命令）
void alarm_cancel(void);             // 取消报警（按钮）
void alarm_tick(void);               // 周期调用（ALARM_TICK_MS）：哔哔节拍 + 窗口超时推送
AlarmState alarm_state(void);
uint32_t alarm_elapsed_ms(void);     // 当前报警已持续毫秒数（非报警态为 0）
