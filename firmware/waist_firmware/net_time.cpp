// ============================================================
//  时间同步（腰端）实现
//  ENABLE_NTP_BOOT_SYNC=1 且本地有 secrets.h 才启用真实同步；
//  否则两个接口安全降级（sync 恒 false / epoch 恒 0，腕端只显示运行时长）。
// ============================================================
#include "net_time.h"
#include "logger.h"
#include "config.h"

// —— 走时锚点（同步成功后设置；epoch() 用 millis 外推，无需再联网）——
static uint32_t s_epoch_anchor    = 0;   // 同步时刻的 UTC 秒（0=未同步）
static uint32_t s_anchor_local_ms = 0;   // 同步时刻的本地 millis()

#ifndef __has_include
#define __has_include(x) 0
#endif

#if ENABLE_NTP_BOOT_SYNC && __has_include("secrets.h")
#define TIME_REAL 1
#include <WiFi.h>
#include <esp_wifi.h>
#include <time.h>
#include "secrets.h"

// 射频释放：断 WiFi 并把信道还给 ESP-NOW（与按需推送的恢复动作一致）
static void wifi_release(void) {
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

bool net_time_sync(void) {
    if (s_epoch_anchor != 0) return true;   // 已同步过：幂等

    // 1) 连 WiFi（限时；热点没开就快速失败，不纠缠）
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > NTP_WIFI_TIMEOUT_MS) {
            LOG_I("TIME", "wifi timeout, time sync skipped (hotspot off?)");
            wifi_release();
            return false;
        }
        delay(200);
    }

    // 2) NTP：configTime 异步同步（UTC，不设时区；显示端自行 +8h）
    //    国内源优先，墙内响应快且稳
    configTime(0, 0, "ntp.aliyun.com", "ntp.tencent.com");
    t0 = millis();
    time_t now = 0;
    while ((now = time(nullptr)) < 1700000000) {   // 2023-11 之后才算有效时间
        if (millis() - t0 > NTP_WAIT_TIMEOUT_MS) {
            LOG_I("TIME", "ntp response timeout");
            wifi_release();
            return false;
        }
        delay(200);
    }

    // 3) 记锚点 + 释放射频（回 ESP-NOW 工作信道，腕端链路自动恢复）
    s_epoch_anchor    = (uint32_t)now;
    s_anchor_local_ms = millis();
    wifi_release();

    LOG_I("TIME", "ntp synced, epoch=%lu", (unsigned long)s_epoch_anchor);
    return true;
}

uint32_t net_time_epoch(void) {
    if (s_epoch_anchor == 0) return 0;
    return s_epoch_anchor + (millis() - s_anchor_local_ms) / 1000;
}

#else   // ---------- 降级实现（开关关闭或缺 secrets.h）----------

#if ENABLE_NTP_BOOT_SYNC
#warning "ENABLE_NTP_BOOT_SYNC=1 但未找到 secrets.h：时间同步不可用（腕端将只显示运行时长）"
#endif

bool net_time_sync(void) { return false; }
uint32_t net_time_epoch(void) { return 0; }

#endif
