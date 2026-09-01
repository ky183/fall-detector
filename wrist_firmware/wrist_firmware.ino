// ============================================================
//  腕端固件 — XIAO ESP32C3
//  智能跌倒检测报警器 · 清华第29届硬件设计大赛
//  职责：采集姿态 → ESP-NOW 发送
// ============================================================
#include "config.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("[wrist] 腕端固件启动");
  Serial.println("[wrist] Step 0 环境搭建完成，等待 Step 1 代码");
}

void loop() {
  delay(1000);
}
