#pragma once
// ============================================================
//  ESP-NOW 无线通信（腰端）— 接收侧
//  职责：初始化射频 + 接收腕端数据包/命令包 + 链路统计
//  依赖：esp_now.h（ESP32 Arduino 自带）
// ============================================================
#include <Arduino.h>
#include "protocol.h"

bool espnow_init(void);   // WiFi STA + ESP-NOW + 注册接收回调

// 链路统计（task_stat 周期读取打印）
void espnow_get_stats(uint32_t* rxCount, uint32_t* lostCount, uint32_t* lastRxAgeMs);
