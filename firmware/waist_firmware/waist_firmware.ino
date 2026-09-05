// ============================================================
//  腰端固件（主控）— ESP32-S3
//  智能跌倒检测报警器 · 清华第29届硬件设计大赛
//  职责：融合判断 + 报警 + OLED 显示 + WiFi 微信推送
// ============================================================
#include "config.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("[waist] 腰端固件启动");
  Serial.println("[waist] Step 0 环境搭建完成，等待 Step 1 代码");
}

void loop() {
  delay(1000);
}
