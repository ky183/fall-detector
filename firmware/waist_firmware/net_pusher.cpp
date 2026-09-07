// ============================================================
//  微信推送（WxPusher）实现
//  ENABLE_WIFI_PUSH=1：真实推送（WiFi + HTTPS POST）
//  ENABLE_WIFI_PUSH=0：占位（只打日志），链路调试用
// ============================================================
#include "net_pusher.h"
#include "logger.h"
#include "config.h"

#if ENABLE_WIFI_PUSH
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

// ---- 单次推送执行（在 task_push 上下文，可慢可阻塞） ----
static bool do_push_once(uint32_t delaySec) {
    // 1) 连接 WiFi（STA 模式已由 espnow_init 设置）
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > WIFI_TIMEOUT_MS) {
            LOG_E("PUSH", "WiFi connect timeout (ssid=%s)", WIFI_SSID);
            WiFi.disconnect();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    LOG_I("PUSH", "WiFi connected, ip=%s", WiFi.localIP().toString().c_str());

    // 2) 组包 + HTTPS POST（WxPusher 消息接口）
    //    文案在 config.h 的 PUSH_* 宏，可直接改中文（保持 UTF-8）
    //    多 UID：secrets.h 的 WXPUSHER_UIDS 数组逐个拼进 JSON
    static const char* kUids[] = { WXPUSHER_UIDS };
    const int uidCnt = sizeof(kUids) / sizeof(kUids[0]);
    char uidsJson[256] = "";
    for (int i = 0; i < uidCnt; i++) {
        if (i > 0) strcat(uidsJson, ",");
        strcat(uidsJson, "\"");
        strcat(uidsJson, kUids[i]);
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
        WiFi.disconnect();
        return false;
    }
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(8000);
    int code = http.POST(body);
    String resp = http.getString();
    http.end();

    // 3) 清理：断开 WiFi，恢复 ESP-NOW 工作信道（关键！否则远端数据断流）
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (code != 200) {
        LOG_E("PUSH", "HTTP %d, resp=%s", code, resp.c_str());
        return false;
    }
    LOG_I("PUSH", "sent ok, resp=%s", resp.c_str());
    return true;
}

// ---- 推送任务：最低优先级，阻塞等队列 ----
static void task_push(void* pv) {
    for (;;) {
        PushReq r;
        if (xQueueReceive(s_queue, &r, portMAX_DELAY) != pdTRUE) continue;
        LOG_I("PUSH", "processing alert (delay=%lus)", (unsigned long)r.delaySec);
        bool ok = do_push_once(r.delaySec);
        if (!ok) {
            LOG_I("PUSH", "retry once...");
            vTaskDelay(pdMS_TO_TICKS(2000));   // 稍等再试（网络抖动）
            ok = do_push_once(r.delaySec);
        }
        LOG_I("PUSH", "alert done (ok=%d)", ok);
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
    LOG_I("PUSH", "pusher ready (queue=%d)", PUSH_QUEUE_LEN);
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

#else   // ---------- 占位实现 ----------

bool pusher_init(void) { return true; }

bool pusher_queue_fall_alert(uint32_t delaySec) {
    LOG_I("PUSH", "[placeholder] fall alert, delay=%us (ENABLE_WIFI_PUSH=0)", delaySec);
    return true;
}

#endif
