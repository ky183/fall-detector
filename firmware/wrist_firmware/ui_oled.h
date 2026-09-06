#pragma once
// ============================================================
//  OLED 显示（腕端）— SSD1306 128x64，I2C 地址 0x3C
//  状态页布局（骨架版本）：
//    ┌──────────────┐
//    │ FALL DETECTOR│  标题
//    │ STATE:NORMAL │  系统状态（ALARM 时反色闪烁）
//    │  SVM 9.81    │  实时合加速度（大字）
//    │ T 00:03:12   │  运行时长（时间同步后改为真实时钟）
//    │ E:3  L:OK    │  按钮事件计数 / 链路状态
//    └──────────────┘
// ============================================================
#include <Arduino.h>

// 显示所需的运行信息（task_display 每次刷新前组装）
struct UiInfo {
    bool    alarm;      // 报警中（腕端正常为 false；取消倒计时逻辑在腰端）
    bool    simActive;  // 模拟跌倒激活中（长按触发，显示提示）
    float   svm;        // 最新 SVM
    uint32_t uptime;    // 运行秒数
    uint32_t evtCnt;    // 按钮事件累计
    bool    linkOk;     // ESP-NOW 链路最近一次发送是否成功
};

bool oled_init();                     // 返回 false = 屏不在线（不影响其他模块）
void oled_show_boot();                // 开机画面（版本信息）
void oled_update(const UiInfo& info); // 刷新状态页（task_display 周期调用）
