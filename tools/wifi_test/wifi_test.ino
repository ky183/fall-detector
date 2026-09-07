// ============================================================
//  WiFi 连接诊断工具 v2（腰端大板）
//  升级点：抓取底层 disconnect reason 码，精确指认失败原因：
//    reason 201 = 找不到 AP（5GHz/没开/名字错）
//    reason 15  = 四次握手超时（密码错误）
//    reason 2   = 认证超时（多为 WPA3/加密方式不兼容）
//    reason 134 = 握手失败（密码/加密）
//
//  使用：改好 SSID/PASS → 烧录 → 串口 115200
//  输出发组长时【从扫描列表开始完整复制】
// ============================================================
#include <WiFi.h>

// ↓↓↓ 改成你的热点名和密码（热点名连空格都要一致）↓↓↓
const char* SSID = "your_hotspot";   // 改成你的热点名（连空格都要一致）
const char* PASS = "your_password";  // 改成你的热点密码

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[wifi] === wifi diagnostic v2 ===");

    // ---- 断开原因监听（关键升级） ----
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.printf("[wifi] >>> DISCONNECT reason=%d (201=没找到AP 15=密码错 2=认证/WPA3)\n",
                      info.wifi_sta_disconnected.reason);
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    // ---- 第一步：扫描（判断热点是否 2.4G 可见） ----
    Serial.println("[wifi] scanning (5s)...");
    int n = WiFi.scanNetworks();
    Serial.printf("[wifi] found %d networks:\n", n);
    for (int i = 0; i < n; i++) {
        Serial.printf("[wifi]   %2d | %-24s | ch%-3d | %ddBm | %s\n",
                      i + 1, WiFi.SSID(i).c_str(),
                      WiFi.channel(i),          // 1~13 = 2.4G；36+ = 5GHz
                      WiFi.RSSI(i),
                      WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "ENC");
    }

    // ---- 第二步：连接测试 ----
    Serial.printf("[wifi] connecting to \"%s\" ...\n", SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    uint32_t t0 = millis();
    int lastSt = -1;
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        int st = WiFi.status();
        if (st != lastSt) {                     // 只打印状态变化，不刷屏
            Serial.printf("[wifi]   status: %d -> %d (%.1fs)\n", lastSt, st, (millis() - t0) / 1000.0);
            lastSt = st;
        }
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[wifi] CONNECTED! ip=%s ch=%d\n",
                      WiFi.localIP().toString().c_str(), WiFi.channel());
        Serial.println("[wifi] >>> WiFi 正常，问题在主固件共存逻辑，反馈组长");
    } else {
        Serial.println("[wifi] FAILED（看上方最后一条 DISCONNECT reason）");
    }
}

void loop() {}
