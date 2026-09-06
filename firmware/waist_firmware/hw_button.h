#pragma once
// ============================================================
//  取消报警按钮（腰端）
//  去抖：连续 2 次采样确认电平变化（约 100ms @50ms 周期）
//  任何时刻按下都视为"确认按下"事件（报警中=取消报警）
// ============================================================
#include <Arduino.h>

enum CancelBtnEvent {
    CBTN_NONE      = 0,   // 无事件
    CBTN_PRESSED   = 1,   // 确认按下（去抖后的下降沿）
};

void button_init(void);            // 配置引脚上拉
CancelBtnEvent button_poll(void);  // 周期调用（BUTTON_POLL_MS）
