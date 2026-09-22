// ============================================================
//  微信推送（WxPusher）实现
//  ENABLE_WIFI_PUSH=1：真实推送（WiFi + HTTPS POST）
//  ENABLE_WIFI_PUSH=0：占位（只打日志），链路调试用
//
//  两种工作模式（config.h 的 PUSH_ALWAYS_ON）：
//    0 = 按需连接：报警时才连 WiFi，推完立刻断开、信道还给
//        ESP-NOW。对任意信道的热点都鲁棒，但推送窗口只有 ~20s。
//    1 = 常连模式：开机即连 WiFi 并保持（断线 60s 静默重连），
//        推送即时。★前提：热点信道 == ESPNOW_CHANNEL（单射频
//        只能停在一个信道），连接后会自动校验，不匹配打告警。
//  两种模式共用：推送失败不再丢弃 —— 最多 PUSH_MAX_ATTEMPTS 次
//  重试（间隔 15s），覆盖约 90 秒的"热点补救窗口"。
// ============================================================
#include "net_pusher.h"
#include "logger.h"
#include "config.h"

// secrets.h 里有 WiFi 密码与 WxPusher Token，已被 .gitignore 排除：
//   **新克隆的仓库里没有这个文件**。若无条件 #include，腰端固件会直接编译
//   不过（而且报错信息只是"secrets.h: No such file"，很难定位）。
//   所以这里先探测再决定：有则启用真实推送，没有则自动降级为占位实现，
//   保证仓库一拉下来就能编译、能烧录调试其余模块。
#ifndef __has_include
#define __has_include(x) 0     // 老工具链兜底：一律视作没有 secrets.h
#endif

#if ENABLE_WIFI_PUSH && __has_include("secrets.h")
#define PUSH_REAL 1
#else
#define PUSH_REAL 0
#if ENABLE_WIFI_PUSH
#warning "ENABLE_WIFI_PUSH=1 但未找到 secrets.h（模板见 secrets.h.example）：微信推送降级为占位实现，填好凭据后重编译即恢复"
#endif
#endif

#if PUSH_REAL
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "secrets.h"

// ---- 队列与任务 ----
static QueueHandle_t s_queue = nullptr;

typedef struct {
    uint32_t delaySec;   // 报警触发后经过的秒数
    // TODO(扩展)：推送类型枚举、自定义消息等在此结构体加字段
} PushReq;

#define PUSH_QUEUE_LEN   2      // 队列深度：防重复刷屏
#define PUSH_TASK_STACK  8192   // TLS+HTTP 需要大栈
#define PUSH_URL "https://wxpusher.zjiecode.com/api/send/message"
#define POST_TIMEOUT_MS  8000

// ---- WiFi 连接辅助 ----
static bool wifi_ok(void) { return WiFi.status() == WL_CONNECTED; }

// 阻塞等待连接（最长 timeout），成功返回 true
static bool connect_wait(uint32_t timeout) {
    if (wifi_ok()) return true;
    // ★修复：清除上一次未完成的连接状态（否则 STA 卡在 connecting，
    //   后续 begin 报 "sta is connecting, cannot set config"，6 次重试全废）
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (!wifi_ok()) {
        if (millis() - t0 > timeout) {
            LOG_E("PUSH", "WiFi connect timeout (ssid=%s)", WIFI_SSID);
            // ★修复：超时必须终止后台连接尝试，给下一个 attempt 留干净状态
            WiFi.disconnect();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // ★共存校验：单射频约束 —— STA 连接后射频跟随热点信道，
    //   若与 ESPNOW_CHANNEL 不一致，腕端链路会静默断流，必须暴露出来
    int ch = WiFi.channel();
    if (ch != ESPNOW_CHANNEL) {
        LOG_E("PUSH", "!! hotspot ch=%d != ESPNOW ch=%d, wrist link BROKEN", ch, ESPNOW_CHANNEL);
    }
    LOG_I("PUSH", "WiFi connected, ip=%s ch=%d",
          WiFi.localIP().toString().c_str(), ch);
    return true;
}

// ---- 单次 POST（假定已连接；在 task_push 上下文，可阻塞） ----
static bool post_once(uint32_t delaySec) {
    // 文案在 config.h 的 PUSH_* 宏，可直接改中文（保持 UTF-8）
    // 多 UID：secrets.h 的 WXPUSHER_UIDS 数组逐个拼进 JSON
    static const char* kUids[] = { WXPUSHER_UIDS };
    const int uidCnt = sizeof(kUids) / sizeof(kUids[0]);
    char uidsJson[256] = "";
    for (int i = 0; i < uidCnt; i++) {
        if (i > 0) strcat(uidsJson, ",");
        strcat(uidsJson, "\""); strcat(uidsJson, kUids[i]);
        strcat(uidsJson, "\"");
    }

    char body[512];
    snprintf(body, sizeof(body),
        "{\"appToken\":\"%s\",\"content\":\"%s%lu%s\",\"summary\":\"%s\",\"contentType\":1,\"uids\":[%s]}",
        WXPUSHER_APP_TOKEN,
        PUSH_CONTENT_HEAD, (unsigned long)delaySec, PUSH_CONTENT_TAIL,
        PUSH_SUMMARY, uidsJson);

    WiFiClientSecure client;
    client.setInsecure();               // 跳过证书校验（演示级；严格化可内置根证书）
    HTTPClient http;
    if (!http.begin(client, PUSH_URL)) {
        LOG_E("PUSH", "http begin failed");
        return false;
    }
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(POST_TIMEOUT_MS);
    int code = http.POST(body);
    String resp = http.getString();
    http.end();

    if (code != 200) {
        LOG_E("PUSH", "HTTP %d, resp=%s", code, resp.c_str());
        return false;
    }
    LOG_I("PUSH", "sent ok, resp=%s", resp.c_str());
    return true;
}

// ---- 推送任务：最低优先级。无请求时兼做 WiFi 保活监督 ----
static void task_push(void* pv) {
#if PUSH_ALWAYS_ON
    connect_wait(WIFI_TIMEOUT_MS);          // 开机先连一次（连不上不阻塞，后面监督）
    uint32_t lastKeepalive = millis();
#endif
    for (;;) {
        // 等请求，最多 15s 醒来一次做保活检查
        PushReq r;
        bool has = (xQueueReceive(s_queue, &r, pdMS_TO_TICKS(15000)) == pdTRUE);

        if (has) {
            LOG_I("PUSH", "processing alert (delay=%lus)", (unsigned long)r.delaySec);
            bool ok = false;
            for (int attempt = 1; attempt <= PUSH_MAX_ATTEMPTS && !ok; attempt++) {
#if PUSH_ALWAYS_ON
                ok = wifi_ok() || connect_wait(WIFI_TIMEOUT_MS);
                if (ok) ok = post_once(r.delaySec);
#else
                ok = connect_wait(WIFI_TIMEOUT_MS) && post_once(r.delaySec);
#endif
                if (!ok && attempt < PUSH_MAX_ATTEMPTS) {
                    LOG_I("PUSH", "attempt %d/%d failed, retry in 15s...",
                          attempt, PUSH_MAX_ATTEMPTS);
                    vTaskDelay(pdMS_TO_TICKS(15000));
                }
            }
#if !PUSH_ALWAYS_ON
            // 按需模式：推完立刻释放射频，恢复 ESP-NOW 工作信道（关键！）
            WiFi.disconnect();
            esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
#endif
            LOG_I("PUSH", "alert done (ok=%d)", ok);
        }
#if PUSH_ALWAYS_ON
        else if (!wifi_ok() && millis() - lastKeepalive > PUSH_RECONNECT_MS) {
            // 常连模式保活：掉线后低频静默重连（高频重连会扫频，
            // 周期性打断 ESP-NOW，得不偿失）
            lastKeepalive = millis();
            connect_wait(WIFI_TIMEOUT_MS);
        }
#endif
    }
}

bool pusher_init(void) {
    if (s_queue == nullptr) {
        s_queue = xQueueCreate(PUSH_QUEUE_LEN, sizeof(PushReq));
    }
    if (!xTaskCreatePinnedToCore(task_push, "push", PUSH_TASK_STACK,
                                 nullptr, 1, nullptr, 1)) {   // 最低优先级
        LOG_E("PUSH", "task create failed");
        return false;
    }
    LOG_I("PUSH", "pusher ready (mode=%s, queue=%d)",
          PUSH_ALWAYS_ON ? "always-on" : "on-demand", PUSH_QUEUE_LEN);
    return true;
}

bool pusher_queue_fall_alert(uint32_t delaySec) {
    if (s_queue == nullptr) return false;
    PushReq r = { .delaySec = delaySec };
    if (xQueueSend(s_queue, &r, 0) != pdTRUE) {   // 不等待
        LOG_E("PUSH", "queue full, alert dropped");
        return false;
    }
    LOG_I("PUSH", "alert queued");
    return true;
}

#else   // ---------- 占位实现（未启用推送，或缺少 secrets.h）----------

bool pusher_init(void) { return true; }

bool pusher_queue_fall_alert(uint32_t delaySec) {
    LOG_I("PUSH", "[placeholder] fall alert, delay=%us (PUSH_REAL=0)", delaySec);
    return true;
}

#endif
