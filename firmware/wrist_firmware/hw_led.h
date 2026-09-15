#pragma once
// ============================================================
//  状态指示 LED（腕端）— 板载 LED1（D9/GPIO8，1kΩ 限流，高电平点亮）
//
//  设计目的：不用盯串口，扫一眼就知道板子状态：
//    心跳短闪（每 2s 闪 100ms） ：一切正常
//    慢闪（每 1s 闪 100ms）     ：MPU 正常但 ESP-NOW 连续发送失败（链路故障）
//    快闪（每 200ms 闪 100ms）  ：MPU 离线（最底层故障，优先查传感器）
//    常灭                       ：未调 led_init / ENABLE_LED=0
//
//  使用：
//    setup() 里调 led_init()；创建 50ms 周期任务循环调 led_tick()
//    task_sensor 每次读数后调 led_report_mpu(ok)
//    task_tx    每次发送后调 led_report_tx(ok)
// ============================================================

void led_init(void);            // 配置引脚为输出，默认熄灭
void led_tick(void);            // 闪烁状态机（50ms 周期调用）

void led_report_mpu(bool ok);   // 传感器健康上报（50Hz 调用）
void led_report_tx(bool ok);    // 发送结果上报（连续失败 10 次才判链路故障，防单包抖动误报）
