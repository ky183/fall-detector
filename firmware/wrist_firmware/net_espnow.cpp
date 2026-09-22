// ============================================================
//  ESP-NOW 无线通信（腕端）实现 — 发送侧
// ============================================================
#include "net_espnow.h"
#include "logger.h"
#include "config.h"

#if ENABLE_ESPNOW
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>   // esp_wifi_set_channel

// 对端（腰端）MAC，来自 config.h（当前为广播地址占位）
static uint8_t s_waist_mac[6] = WAIST_MAC;
static uint8_t s_seq = 0;                 // 发送序号
volatile bool g_espnow_last_ok = false;   // 最近一次发送结果

// —— 腰端报警状态回显（PKT_ACK，v0.6）——
volatile uint8_t  g_alarm_state     = ACK_ALARM_NORMAL;
volatile uint8_t  g_alarm_remain    = 0;
volatile uint32_t g_alarm_rx_ms     = 0;
volatile uint32_t g_cancel_until_ms = 0;

// —— 时钟锚点（PKT_ACK.epochSec 捎带下发，60s 周期刷新+报警包也带）——
volatile uint32_t g_epoch_anchor    = 0;   // 0=从未同步（显示层回退运行时长）
volatile uint32_t g_epoch_anchor_ms = 0;

// 发送结果回调（ESP-NOW 异步返回，此函数在 WiFi 任务上下文执行，
// 只做置位和日志，不做耗时操作）
// 注意：板卡包 3.3.x 的签名为 (wifi_tx_info_t*, status)；
// 若升级板卡包后此处报错，按编译器提示调整签名即可
static void on_sent(const wifi_tx_info_t* info, esp_now_send_status_t st) {
    (void)info;
    g_espnow_last_ok = (st == ESP_NOW_SEND_SUCCESS);
}

// 接收回调：腰端 PKT_ACK（报警状态回显）。WiFi 任务上下文，只写 volatile
static void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    (void)info;
    if (len != sizeof(WristPacket)) return;               // 长度不符，丢弃
    const WristPacket* pkt = reinterpret_cast<const WristPacket*>(data);
    if (!proto_valid(pkt)) return;                        // 魔数/校验和不过，丢弃
    if (pkt->h.devId != DEV_ID_WAIST) return;             // 只收腰端的包
    if (pkt->h.type != PKT_ACK) return;                   // 腕端只处理 ACK

    uint8_t next = pkt->u.ack.alarmState;
    // 报警中收到 NORMAL = 腰端按钮取消成功 → "已取消"提示 3 秒
    if (next == ACK_ALARM_NORMAL && g_alarm_state != ACK_ALARM_NORMAL) {
        g_cancel_until_ms = millis() + 3000;
    }
    g_alarm_state  = next;
    g_alarm_remain = pkt->u.ack.remainSec;
    g_alarm_rx_ms  = millis();

    // 时钟锚点（epoch>0 才有效；每包刷新，兼顾漂移校准）
    if (pkt->u.ack.epochSec != 0) {
        g_epoch_anchor    = pkt->u.ack.epochSec;
        g_epoch_anchor_ms = millis();
    }
}
#endif

bool espnow_init(void) {
#if ENABLE_ESPNOW
    // ESP-NOW 基于 WiFi 射频：STA 模式 + 不连接任何路由器
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // 信道两端一致（config.h），固定后不再漂移
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        LOG_E("NET", "esp_now_init failed");
        return false;
    }
    esp_now_register_send_cb(on_sent);
    esp_now_register_recv_cb(on_recv);   // v0.6：接收腰端 PKT_ACK（报警状态回显）

    // 注册对端
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, s_waist_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;   // 广播模式不支持加密；演示环境可接受
    if (esp_now_add_peer(&peer) != ESP_OK) {
        LOG_E("NET", "add_peer failed");
        return false;
    }

    LOG_I("NET", "ESP-NOW ready, ch=%d, dst=%02X:%02X:%02X:%02X:%02X:%02X",
          ESPNOW_CHANNEL, s_waist_mac[0], s_waist_mac[1], s_waist_mac[2],
          s_waist_mac[3], s_waist_mac[4], s_waist_mac[5]);
    LOG_I("NET", "my MAC: %s", WiFi.macAddress().c_str());  // 供腰端配置对端用
    return true;
#else
    return false;
#endif
}

// 统一的打包发送：填头 → 填载荷 → 校验和 → 发送
static bool send_packet(WristPacket& pkt) {
#if ENABLE_ESPNOW
    pkt.h.devId = DEV_ID_WRIST;
    pkt.h.seq   = s_seq++;
    pkt.checksum = proto_checksum(&pkt);

    bool ok = (esp_now_send(s_waist_mac, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK);
    if (!ok) LOG_E("NET", "send failed (seq=%u)", pkt.h.seq);
    return ok;
#else
    (void)pkt;
    return false;
#endif
}

bool espnow_send_data(const SensorData& d) {
#if ENABLE_ESPNOW
    WristPacket pkt = {};
    pkt.h.magic = 0xA5;
    pkt.h.type  = PKT_DATA;
    pkt.u.data.ts  = d.ts;
    pkt.u.data.ax  = d.ax;  pkt.u.data.ay = d.ay;  pkt.u.data.az = d.az;
    pkt.u.data.gx  = d.gx;  pkt.u.data.gy = d.gy;  pkt.u.data.gz = d.gz;
    pkt.u.data.svm = d.svm;
    pkt.u.data.battery = 0xFF;   // 未接电量检测，占位"未知"
    return send_packet(pkt);
#else
    (void)d;
    return false;
#endif
}

bool espnow_send_cmd(uint8_t cmd, uint8_t arg) {
#if ENABLE_ESPNOW
    WristPacket pkt = {};
    pkt.h.magic = 0xA5;
    pkt.h.type  = PKT_CMD;
    pkt.u.cmd.cmd = cmd;
    pkt.u.cmd.arg = arg;
    LOG_I("NET", "send CMD 0x%02X (arg=%u)", cmd, arg);
    return send_packet(pkt);
#else
    (void)cmd; (void)arg;
    return false;
#endif
}
