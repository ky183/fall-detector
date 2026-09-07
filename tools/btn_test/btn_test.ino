// ============================================================
//  按钮引脚诊断工具 — 腕端（不属于正式固件）
//  原理：把 XIAO 全部可用 GPIO 设为上拉输入，实时打印电平变化。
//        按下按钮时看哪个脚变 0：
//          D1(GPIO2) 变 0   → 硬件通路正常（问题在主固件侧，反馈组长）
//          其他脚变 0       → 线插错脚了（工具会告诉你实际插在哪）
//          全程无任何变化   → 开关坏 / 线断 / 焊点虚焊（需万用表/补焊）
//
//  使用：板选 XIAO_ESP32S3 烧录 → 串口 115200 → 反复按/松按钮
//  诊断完烧回 firmware/wrist_firmware/wrist_firmware.ino
// ============================================================
#include <Arduino.h>

// XIAO ESP32S3 可安全读取的 GPIO（不含 D6/D7=串口脚）
static const int PINS[]  = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
static const char* NAMES[] = { "D0", "D1", "D2", "D3", "D4", "D5", "D8", "D9", "D10" };
static const int PIN_CNT = sizeof(PINS) / sizeof(PINS[0]);
static int s_last[9];

void setup() {
    Serial.begin(115200);
    delay(300);
    for (int i = 0; i < PIN_CNT; i++) {
        pinMode(PINS[i], INPUT_PULLUP);   // 上拉：松开=1，按下(接GND)=0
        s_last[i] = 1;
    }
    Serial.println("[btnt] === button pin scanner ===");
    Serial.println("[btnt] 现在反复按下/松开按钮，观察输出");
    Serial.println("[btnt] 预期：D1(GPIO2) 在 0/1 之间变化");
}

void loop() {
    for (int i = 0; i < PIN_CNT; i++) {
        int v = digitalRead(PINS[i]);
        if (v != s_last[i]) {
            Serial.printf("[btnt] %s(GPIO%d): %d -> %d   %s\n",
                          NAMES[i], PINS[i], s_last[i], v,
                          (v == 0) ? "<<< 检测到按下!" : "松开");
            s_last[i] = v;
        }
    }
    delay(20);
}
