// ============================================================
//  微信推送（WxPusher）实现 — 当前为占位
// ============================================================
#include "net_pusher.h"
#include "logger.h"
#include "config.h"

#if ENABLE_WIFI_PUSH
#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"   // WXPUSHER_APP_TOKEN / WXPUSHER_UID / WIFI_SSID / WIFI_PASS
// secrets.h 模板（自行创建，禁止提交 Git）：
//   #define WIFI_SSID          "你家WiFi名"
//   #define WIFI_PASS          "你家WiFi密码"
//   #define WXPUSHER_APP_TOKEN "xxxx"
//   #define WXPUSHER_UID       "xxxx"
#endif

bool pusher_send_fall_alert(uint32_t delaySec) {
#if ENABLE_WIFI_PUSH
    // TODO(Step 5)：WiFi 连接（带 5s 超时）→ HTTP POST WxPusher → 断开
    // 放最低优先级任务调用，绝不阻塞检测
    (void)delaySec;
    return false;
#else
    LOG_I("PUSH", "[placeholder] fall alert, delay=%us (Step 5 接入 WxPusher)", delaySec);
    return true;
#endif
}
