#pragma once
// ============================================================
//  微信推送（WxPusher）— 腰端
//  当前为占位实现（ENABLE_WIFI_PUSH=0）：
//    只打日志，返回 true，用于调通报警链路
//  Step 5 实现：连 WiFi → HTTP POST WxPusher → 家属微信
//  凭据放 secrets.h（已被 .gitignore 忽略，禁止提交）
// ============================================================
#include <stdint.h>

// 推送跌倒警报（delaySec = 从报警触发到现在的秒数，供消息展示）
// 返回 true = 推送成功（占位实现恒 true）
bool pusher_send_fall_alert(uint32_t delaySec);
