#pragma once
// ============================================================
//  ESP-NOW 无线通信（腰端）— 接收侧 + 报警状态回显
//  职责：初始化射频 + 接收腕端数据包/命令包 + 链路统计
//        + v0.6：报警状态回发腕端（OLED 弹窗用，ENABLE_ALARM_ECHO）
//  依赖：esp_now.h（ESP32 Arduino 自带）
// ============================================================
#include <Arduino.h>
#include "protocol.h"

bool espnow_init(void);   // WiFi STA + ESP-NOW + 注册接收回调

// 链路统计（task_stat 周期读取打印）
void espnow_get_stats(uint32_t* rxCount, uint32_t* lostCount, uint32_t* lastRxAgeMs);

// 报警状态回发（task_alarm 调用；腕端 MAC 取自首个收到的数据包，之前调用直接失败）
// alarmState: ACK_ALARM_*；remainSec: 取消窗剩余秒（非 WAIT 态传 0）
// epochSec:   当前 UTC 秒（net_time_epoch()，未同步传 0，腕端自动忽略）
bool espnow_send_ack(uint8_t alarmState, uint8_t remainSec, uint32_t epochSec);
