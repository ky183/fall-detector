#pragma once
// ============================================================
//  OLED 显示（腕端）— SSD1306 128x64，I2C 地址 0x3C
//  v0.6 页面优先级（高→低）：
//    1. 报警弹窗（腰端 ACK_ALARM_WAIT）：倒计时大字 + 取消提示
//    2. 已推送页（ACK_ALARM_SENT）：FAMILY ALERTED
//    3. 已取消页（3 秒）：CANCELLED
//    4. 正常页：标题 + 运行时长 + 链路
//       DEBUG_DISPLAY=1 时切换为调试布局（SVM 大字/事件计数）
//  全英文显示（Adafruit GFX 内置字库）；中文需换 U8g2，只改 .cpp
// ============================================================
#include <Arduino.h>

// 显示所需的运行信息（task_display 每次刷新前组装）
struct UiInfo {
    // —— 腰端报警状态回显（PKT_ACK，v0.6；.ino 负责老化判断）——
    uint8_t alarmState;   // ACK_ALARM_*（0=正常）
    uint8_t remainSec;    // 取消窗剩余秒（仅 WAIT 态有效）
    bool    cancelled;    // 报警被取消（3 秒提示窗内）
    bool    simActive;    // 模拟跌倒激活中（长按触发，本地调试提示）
    float   svm;          // 最新 SVM（DEBUG_DISPLAY=1 显示）
    uint32_t uptime;      // 运行秒数
    uint32_t evtCnt;      // 按钮事件累计（DEBUG_DISPLAY=1 显示）
    bool    linkOk;       // ESP-NOW 链路最近一次发送是否成功
};

bool oled_init();                     // 返回 false = 屏不在线（不影响其他模块）
void oled_show_boot();                // 开机画面（版本信息）
void oled_update(const UiInfo& info); // 刷新状态页（task_display 周期调用）
