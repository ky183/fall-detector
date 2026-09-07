#pragma once
// ============================================================
//  微信推送（WxPusher）— 腰端
//
//  架构（可扩展性设计）：
//    报警/检测任务 --入队(非阻塞)--> 队列 --> task_push(最低优先级)
//                                                    │
//                                        连WiFi → HTTPS POST → 断开
//                                        → 恢复 ESP-NOW 信道
//
//  要点：
//    1. 报警任务绝不因 HTTP 而阻塞（队列入队即返回）
//    2. 推送期间 WiFi 连接会切换信道，ESP-NOW 短暂停收（约10~20s），
//       推完自动恢复信道——报警已在进行，数据断流无碍
//    3. 凭据在 secrets.h（git 忽略），模板见 secrets.h.example
//    4. 新增推送类型：扩展 PushType + queue 函数即可
// ============================================================
#include <stdint.h>

// 初始化：创建队列 + 推送任务（setup 中 #if ENABLE_WIFI_PUSH 时调用一次）
bool pusher_init(void);

// 入队一条跌倒警报（非阻塞，队列满则丢弃并记日志）
// delaySec = 从报警触发至今的秒数（写入消息供家属判断）
bool pusher_queue_fall_alert(uint32_t delaySec);
