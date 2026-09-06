#pragma once
// ============================================================
//  板间通信协议（腕端 ↔ 腰端，ESP-NOW 载荷）
//  ★ 两端共用：腰端固件需要复制本文件，修改必须同步两端
//
//  包结构（共 38 字节，ESP-NOW 上限 250 字节，余量充足）：
//    PacketHeader(4B) + payload union(33B) + checksum(1B)
// ============================================================
#include <stdint.h>

// ---------- 设备标识 ----------
#define DEV_ID_WRIST     0x01
#define DEV_ID_WAIST     0x02

// ---------- 包类型 ----------
enum PacketType : uint8_t {
    PKT_DATA = 0x01,   // 腕 → 腰：传感器数据（50Hz 周期发送）
    PKT_CMD  = 0x02,   // 腕 → 腰：控制命令（事件触发）
    PKT_ACK  = 0x03,   // 腰 → 腕：应答/状态下发（预留，时间同步等）
};

// ---------- 命令码（PKT_CMD 的 payload） ----------
enum CmdCode : uint8_t {
    CMD_SIM_FALL = 0x02,   // 模拟跌倒（调试：ENABLE_FALL_DETECT=0 时腕端长按触发）
    // 取消报警由腰端本地按钮实现（不经无线）；后续如需腕端远程取消再扩展命令码
};

// ---------- 包体定义 ----------
#pragma pack(push, 1)

struct PacketHeader {
    uint8_t magic;    // 固定 0xA5，用于快速过滤非法包
    uint8_t devId;    // 发送者设备 ID（DEV_ID_xxx）
    uint8_t type;     // PacketType
    uint8_t seq;      // 发送序号，递增；接收端可统计丢包
};

struct SensorPayload {          // PKT_DATA 的载荷
    uint32_t ts;                // 腕端毫秒时间戳（开机计时的 millis()）
    float    ax, ay, az;        // 加速度 (m/s²)
    float    gx, gy, gz;        // 角速度 (deg/s)
    float    svm;               // 合加速度幅值 SVM = √(ax²+ay²+az²) (m/s²)
    uint8_t  battery;           // 电量百分比，0xFF = 未知（未接电量检测）
};

struct CmdPayload {             // PKT_CMD 的载荷
    uint8_t cmd;                // CmdCode
    uint8_t arg;                // 命令参数，暂未用，置 0
};

struct WristPacket {
    PacketHeader h;
    union {                     // 按 h.type 解释
        SensorPayload data;
        CmdPayload    cmd;
    } u;
    uint8_t checksum;           // 头 + payload 的字节累加和（低 8 位）
};

#pragma pack(pop)

// ---------- 校验工具 ----------
inline uint8_t proto_checksum(const WristPacket* p) {
    const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(WristPacket) - 1; i++) sum += b[i];
    return sum;
}

// 简单合法性检查：魔数 + 校验和
inline bool proto_valid(const WristPacket* p) {
    return p && p->h.magic == 0xA5 && proto_checksum(p) == p->checksum;
}
