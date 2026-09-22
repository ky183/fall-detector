// ============================================================
//  OLED 显示（腕端）实现
//  依赖：Adafruit SSD1306 + GFX（已安装）
//  当前显示英文（字库内置）；若后期需要中文，更换为 U8g2 库，
//  只需改本文件，接口 ui_oled.h 不变
//
//  v0.6 页面优先级：报警倒计时 > 已推送 > 已取消(3s) > 正常页
//  正常页布局由 config.h 的 DEBUG_DISPLAY 切换：
//    0 = 产品布局（时长大字 + 状态/链路）
//    1 = 调试布局（SVM 大字 + 事件计数，原骨架版）
// ============================================================
#include "ui_oled.h"
#include "logger.h"
#include "config.h"
#include "protocol.h"      // ACK_ALARM_*（报警状态取值）

#if ENABLE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
static bool s_oled_ok = false;
#endif

bool oled_init(void) {
#if ENABLE_OLED
    // SSD1306_SWITCHCAPVCC = 使用内部升压电荷泵（模块默认接法）
    if (!display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) {
        return false;   // I2C 上找不到 0x3C
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    s_oled_ok = true;
    return true;
#else
    return false;   // 编译期关闭，视作不在线
#endif
}

void oled_show_boot(void) {
#if ENABLE_OLED
    if (!s_oled_ok) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("FALL DETECTOR");
    display.println("wrist v0.6");
    display.println("booting...");
    display.display();
#endif
}

void oled_update(const UiInfo& info) {
#if ENABLE_OLED
    if (!s_oled_ok) return;

    display.clearDisplay();

    if (info.alarmState == ACK_ALARM_WAIT) {
        // ===== 1. 报警弹窗：取消窗倒计时（最高优先级）=====
        display.fillRect(0, 0, 128, 14, SSD1306_WHITE);   // 反色顶条
        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(1);
        display.setCursor(5, 3);
        display.print("! FALL DETECTED !");
        display.setTextColor(SSD1306_WHITE);

        // 剩余秒大字（size3 ≈ 18px/字符，居中）
        char num[4];
        snprintf(num, sizeof(num), "%u", (unsigned)info.remainSec);
        display.setTextSize(3);
        display.setCursor((128 - (int16_t)strlen(num) * 18) / 2, 20);
        display.print(num);

        display.setTextSize(1);
        display.setCursor(10, 48);
        display.print("SECONDS TO NOTIFY");
        display.setCursor(10, 56);
        display.print("WAIST KNOB=CANCEL");
    } else if (info.alarmState == ACK_ALARM_SENT) {
        // ===== 2. 已推送页：家人已被通知 =====
        display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(1);
        display.setCursor(5, 3);
        display.print("! FAMILY ALERTED !");
        display.setTextColor(SSD1306_WHITE);

        display.setTextSize(2);
        display.setCursor(22, 24);
        display.print("ALERT!");
        display.setTextSize(1);
        display.setCursor(4, 48);
        display.print("NOTIFYING VIA WECHAT");
        display.setCursor(16, 56);
        display.print("WAIST KNOB=STOP");
    } else if (info.cancelled) {
        // ===== 3. 已取消页（报警被腰端按钮取消，显示 3 秒）=====
        display.setTextSize(2);
        display.setCursor(10, 12);
        display.print("CANCELLED");
        display.setTextSize(1);
        display.setCursor(34, 36);
        display.print("FALSE ALARM");
        display.setCursor(22, 48);
        display.print("RESUMING WATCH...");
    } else {
        // ===== 4. 正常页 =====
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("FALL DETECTOR [W]");

        // 模拟跌倒提示（长按触发，本地调试反馈）
        if (info.simActive) {
            display.fillRect(0, 10, 128, 10, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
            display.setCursor(2, 12);
            display.print("! SIM FALL SENT !");
            display.setTextColor(SSD1306_WHITE);
        }

        uint32_t s = info.uptime;
        char line[24];
#if DEBUG_DISPLAY
        // —— 调试布局：SVM 大字 + 事件计数（原骨架版，联调/调参用）——
        if (!info.simActive) {
            display.setCursor(0, 12);
            display.print("STATE: NORMAL");
        }
        display.setTextSize(2);
        display.setCursor(0, 24);
        display.print("SVM ");
        display.println(info.svm, 2);
        snprintf(line, sizeof(line), "T %02u:%02u:%02u  E:%lu",
                 (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60),
                 (unsigned long)info.evtCnt);
        display.setTextSize(1);
        display.setCursor(0, 48);
        display.print(line);
        display.setCursor(0, 56);
        display.println(info.linkOk ? "LINK: OK" : "LINK: --");
#else
        // —— 产品布局：运行时长大字 + 步数预留行 + 链路 ——
        snprintf(line, sizeof(line), "%02u:%02u:%02u",
                 (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
        display.setTextSize(2);
        display.setCursor((128 - (int16_t)strlen(line) * 12) / 2, 20);
        display.print(line);
        display.setTextSize(1);
        display.setCursor(0, 44);
        snprintf(line, sizeof(line), "STEPS %lu", (unsigned long)info.steps);
        display.print(line);
        display.setCursor(0, 56);
        display.println(info.linkOk ? "LINK: OK" : "LINK: --");
#endif
    }

    display.display();
#endif
}
