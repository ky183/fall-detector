// ============================================================
//  ESP-NOW 无线通信（腰端）实现 — 接收侧
// ============================================================
#include "net_espnow.h"
#include "logger.h"
#include "config.h"
#include "sensor_manager.h"
#include "alarm_manager.h"   // CMD_SIM_FALL 直接触发报警（链路调试）

#if ENABLE_ESPNOW
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- 接收统计 ----
static volatile uint32_t s_rx_count   = 0;   // 累计收包
static volatile uint32_t s_lost_count = 0;   // 按 seq 差估算的丢包
static volatile uint32_t s_last_rx_ms = 0;   // 最近一包时刻
static volatile bool     s_last_seq_valid = false;
static volatile uint8_t  s_last_seq = 0;

// 首包来源 MAC（打印供单播配置参考）
static bool s_src_logged = false;

// 接收回调（WiFi 任务上下文执行，只做轻量处理，禁止阻塞/延时）
static void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len != sizeof(WristPacket)) return;              // 长度不符，丢弃
    const WristPacket* pkt = reinterpret_cast<const WristPacket*>(data);
    if (!proto_valid(pkt))  return;                       // 魔数/校验和不过，丢弃
    if (pkt->h.devId != DEV_ID_WRIST) return;             // 只收腕端的包

    if (!s_src_logged && info) {
        const uint8_t* m = info->src_addr;
        LOG_I("NET", "wrist MAC: %02X:%02X:%02X:%02X:%02X:%02X (recorded)",
              m[0], m[1], m[2], m[3], m[4], m[5]);
        s_src_logged = true;
    }

    // 丢包估算：seq 应比上次大 1（uint8 回绕）
    if (s_last_seq_valid) {
        s_lost_count += (uint8_t)(pkt->h.seq - s_last_seq - 1);
    }
    s_last_seq = pkt->h.seq;
    s_last_seq_valid = true;
    s_rx_count++;
    s_last_rx_ms = millis();

    switch (pkt->h.type) {
    case PKT_DATA: {
        // 写入远端共享数据（锁忙则丢帧，下包 20ms 后到）
        SensorData d;
        d.ax = pkt->u.data.ax;  d.ay = pkt->u.data.ay;  d.az = pkt->u.data.az;
        d.gx = pkt->u.data.gx;  d.gy = pkt->u.data.gy;  d.gz = pkt->u.data.gz;
        d.svm = pkt->u.data.svm;
        d.ts  = millis();   // 用本地时间戳（对端 millis 与本机无对齐意义）
        sensor_manager_set_remote(d);
        break;
    }
    case PKT_CMD:
        if (pkt->u.cmd.cmd == CMD_SIM_FALL) {
            LOG_I("NET", "rx CMD_SIM_FALL -> trigger alarm");
            alarm_trigger("sim");
        }
        break;
    default:
        break;   // PKT_ACK 预留
    }
}
#endif

bool espnow_init(void) {
#if ENABLE_ESPNOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        LOG_E("NET", "esp_now_init failed");
        return false;
    }
    esp_now_register_recv_cb(on_recv);

    LOG_I("NET", "ESP-NOW ready, ch=%d (rx mode)", ESPNOW_CHANNEL);
    LOG_I("NET", "my MAC: %s", WiFi.macAddress().c_str());
    return true;
#else
    return false;
#endif
}

void espnow_get_stats(uint32_t* rxCount, uint32_t* lostCount, uint32_t* lastRxAgeMs) {
    if (rxCount)    *rxCount    = s_rx_count;
    if (lostCount)  *lostCount  = s_lost_count;
    if (lastRxAgeMs) {
        *lastRxAgeMs = s_last_seq_valid ? (millis() - s_last_rx_ms) : 0;
    }
}
