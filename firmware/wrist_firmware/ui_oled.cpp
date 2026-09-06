// ============================================================
//  OLED 显示（腕端）实现
//  依赖：Adafruit SSD1306 + GFX（已安装）
//  当前显示英文（字库内置）；若后期需要中文，更换为 U8g2 库，
//  只需改本文件，接口 ui_oled.h 不变
// ============================================================
#include "ui_oled.h"
#include "logger.h"
#include "config.h"

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
    display.println("wrist v0.1");
    display.println("booting...");
    display.display();
#endif
}

void oled_update(const UiInfo& info) {
#if ENABLE_OLED
    if (!s_oled_ok) return;

    // —— 组装各显示行 ——
    char line[24];

    // 运行时长 hh:mm:ss（TODO：腰端 NTP 时间同步后改为真实时钟）
    uint32_t s = info.uptime;
    snprintf(line, sizeof(line), "T %02u:%02u:%02u  E:%lu",
             (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60),
             (unsigned long)info.evtCnt);

    display.clearDisplay();
    display.setTextSize(1);

    // 标题 + 状态（报警/模拟时用反色块强调）
    display.setCursor(0, 0);
    display.println("FALL DETECTOR [W]");
    if (info.simActive) {
        display.fillRect(0, 10, 128, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, 12);
        display.println("! SIM FALL SENT !");
        display.setTextColor(SSD1306_WHITE);
    } else {
        display.setCursor(0, 12);
        display.println(info.alarm ? "STATE: ALARM!" : "STATE: NORMAL");
    }

    // SVM 大字（判定核心特征，方便肉眼对照）
    display.setTextSize(2);
    display.setCursor(0, 24);
    display.print("SVM ");
    display.println(info.svm, 2);

    // 底部：时间 + 事件计数 / 链路
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.println(line);
    display.setCursor(0, 56);
    display.println(info.linkOk ? "LINK: OK" : "LINK: --");

    display.display();
#endif
}
