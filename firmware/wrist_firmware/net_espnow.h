#pragma once
// ============================================================
//  ESP-NOW 无线通信（腕端）— 发送侧 + ACK 接收
//  职责：初始化射频 + 两个发送接口（数据包 / 命令包）
//        + v0.6：接收腰端 PKT_ACK 报警状态回显（OLED 弹窗用）
//  依赖：esp_now.h（ESP32 Arduino 自带，无需安装）
//
//  信道说明：ESP-NOW 不经路由器，但要求收发两端在同一 WiFi 信道
//  （config.h 的 ESPNOW_CHANNEL，两端必须一致）
// ============================================================
#include <Arduino.h>
#include "sensor_manager.h"   // SensorData
#include "protocol.h"         // WristPacket / CmdCode

bool espnow_init();                          // WiFi STA + ESP-NOW + 添加对端 + 注册ACK接收
bool espnow_send_data(const SensorData& d);  // 发 PKT_DATA（task_tx 周期调用）
bool espnow_send_cmd(uint8_t cmd, uint8_t arg = 0);  // 发 PKT_CMD（事件触发）

// 最近一次发送是否成功（OLED 链路指示用；单次失败不代表链路断，
// 连续失败才需要排查——接收端用 seq 可进一步统计丢包率）
extern volatile bool g_espnow_last_ok;

// —— 腰端报警状态回显（PKT_ACK，v0.6；接收回调只写这几个 volatile，别处只读）——
extern volatile uint8_t  g_alarm_state;      // 最近收到的 ACK_ALARM_*（0=NORMAL）
extern volatile uint8_t  g_alarm_remain;     // 取消窗剩余秒（仅 WAIT 态有效）
extern volatile uint32_t g_alarm_rx_ms;      // 最近一包 ACK 到达时刻（显示层做老化）
extern volatile uint32_t g_cancel_until_ms;  // "已取消"提示截止时刻（回调置 3 秒）
