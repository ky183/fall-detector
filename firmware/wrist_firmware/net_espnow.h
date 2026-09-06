#pragma once
// ============================================================
//  ESP-NOW 无线通信（腕端）— 发送侧
//  职责：初始化射频 + 两个发送接口（数据包 / 命令包）
//  依赖：esp_now.h（ESP32 Arduino 自带，无需安装）
//
//  信道说明：ESP-NOW 不经路由器，但要求收发两端在同一 WiFi 信道
//  （config.h 的 ESPNOW_CHANNEL，两端必须一致）
// ============================================================
#include <Arduino.h>
#include "sensor_manager.h"   // SensorData
#include "protocol.h"         // WristPacket / CmdCode

bool espnow_init();                          // WiFi STA + ESP-NOW + 添加对端
bool espnow_send_data(const SensorData& d);  // 发 PKT_DATA（task_tx 周期调用）
bool espnow_send_cmd(uint8_t cmd, uint8_t arg = 0);  // 发 PKT_CMD（事件触发）

// 最近一次发送是否成功（OLED 链路指示用；单次失败不代表链路断，
// 连续失败才需要排查——接收端用 seq 可进一步统计丢包率）
extern volatile bool g_espnow_last_ok;
