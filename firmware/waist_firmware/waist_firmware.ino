// ============================================================
//  腰端固件入口 — ESP32-S3 大板（主控）
//  智能跌倒检测报警器 · 清华第29届硬件设计大赛
//  职责：本地采集 + 接收腕端数据 + 融合判断（算法组）+ 报警 + 推送
//
//  任务一览（FreeRTOS，全部跑在 core1，WiFi 协议栈在 core0）：
//    task_sensor  50Hz  高   读本地 MPU → g_local_data
//    task_detect  50Hz  高   组装两端数据 → fall_detector（算法组）
//    task_button  20Hz  中   取消按钮扫描（去抖）
//    task_alarm   10Hz  高   报警状态机（哔哔节拍 + 推送窗口）
//    task_stat     1Hz  低   链路统计打印（收包/丢包）
//
//  烧录（重要）：
//    板选 ESP32S3 Dev Module，USB CDC in Boot = Disable
//    烧录口 = Micro USB（CH340 串口）；供电 Type-C/DC5521 需 ≥6.65V
// ============================================================
#include "config.h"
#include "logger.h"
#include "protocol.h"
#include "sensor_manager.h"
#include "hw_buzzer.h"
#include "hw_button.h"
#include "fall_detector.h"
#include "alarm_manager.h"
#include "net_espnow.h"
#include "net_pusher.h"
#include "net_time.h"
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ================= 任务实现 =================

// 本地传感器采样：严格 50Hz
static void task_sensor(void* pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
    for (;;) {
        vTaskDelayUntil(&last, period);
        if (!sensor_manager_read()) {
            static uint32_t lastErr = 0;
            if (millis() - lastErr > 1000) {
                LOG_E("SENS", "local MPU offline, retrying...");
                lastErr = millis();
            }
        }
    }
}

// 检测任务：取两端快照 → 算法判定 → 触发报警
static void task_detect(void* pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
    for (;;) {
        vTaskDelayUntil(&last, period);
#if ENABLE_FALL_DETECT
        FallInput in = {};
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10))) {
            in.local  = g_local_data;
            in.remote = g_remote_data;
            xSemaphoreGive(g_sensor_mutex);
        }
        in.hasRemote = sensor_manager_has_remote(&in.remoteAgeMs);

        if (fall_detector_update(in) == FALL_CONFIRMED) {
            alarm_trigger("algo");
        }
#else
        // 算法未启用：本任务空转（链路由腕端 SIM_FALL 命令触发）
#endif
    }
}

// 取消按钮：按下即取消报警（含已推送状态）
static void task_button(void* pv) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        if (button_poll() == CBTN_PRESSED) {
            LOG_I("BTN", "cancel pressed");
            alarm_cancel();
        }
    }
}

// 报警状态机步进：哔哔节拍 + 取消窗口超时推送
// v0.6：报警状态变化/报警期间低频重发 PKT_ACK 给腕端（OLED 弹窗显示用）
static void task_alarm(void* pv) {
    static AlarmState s_echo_last = ST_NORMAL;
    static uint32_t   s_echo_last_ms = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ALARM_TICK_MS));
        alarm_tick();
#if ENABLE_ALARM_ECHO
        AlarmState st  = alarm_state();
        uint32_t   now = millis();
        // 发送时机：状态变化立即发；取消窗内 2Hz 重发；推送后 1Hz 重发（丢包兜底）
        bool echo = (st != s_echo_last)
                 || (st == ST_WAIT_CANCEL && now - s_echo_last_ms >= 500)
                 || (st == ST_SENT       && now - s_echo_last_ms >= 1000);
        if (echo) {
            uint8_t remain = 0;
            if (st == ST_WAIT_CANCEL) {
                uint32_t leftMs = ALARM_CANCEL_WINDOW_MS - alarm_elapsed_ms();
                remain = (uint8_t)((leftMs + 999) / 1000);   // 向上取整（OLED 大字显示）
            }
            espnow_send_ack((uint8_t)st, remain, net_time_epoch());
            s_echo_last_ms = now;
        }
        // NORMAL 态周期时间包：epoch 捎带给腕端时钟（未同步时 epoch=0，腕端忽略）
        else if (st == ST_NORMAL && now - s_echo_last_ms >= TIME_SYNC_ECHO_MS) {
            espnow_send_ack(ACK_ALARM_NORMAL, 0, net_time_epoch());
            s_echo_last_ms = now;
        }
        s_echo_last = st;
#endif
    }
}

#if ENABLE_NTP_BOOT_SYNC
// 开机延时一次性 NTP 时间同步（独立低优先级任务，绝不阻塞报警/判定链路）
// 同步窗口内射频被 WiFi 占用（腕端 LED 慢闪），完成后自动回 ESP-NOW 信道
static void task_timesync(void* pv) {
    vTaskDelay(pdMS_TO_TICKS(NTP_BOOT_DELAY_MS));
    if (net_time_sync()) {
        LOG_I("SYS", "time ready (wrist clock will follow)");
    }
    vTaskDelete(nullptr);   // 一次性任务，结束自删
}
#endif

// 链路统计：1Hz 打印收包/丢包（LOG_LEVEL>=3 时输出）
static void task_stat(void* pv) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STAT_PERIOD_MS));
        uint32_t rx = 0, lost = 0, age = 0;
        espnow_get_stats(&rx, &lost, &age);
        LOG_D("STAT", "rx=%lu lost=%lu lastAge=%lums state=%d",
              (unsigned long)rx, (unsigned long)lost, (unsigned long)age, alarm_state());
    }
}

// ================= 入口 =================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    LOG_I("SYS", "=== waist firmware v0.1 (skeleton) ===");

    // I2C：大板 GPIO8/9（默认 100kHz，稳定性优先）
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // —— 各模块初始化：失败不阻塞，分级开关见 config.h ——
    alarm_init();   // 蜂鸣器静音初始化（确保上电不响）

#if ENABLE_MPU6050
    LOG_I("SYS", "local MPU init ... %s", sensor_manager_init() ? "ok" : "FAIL(0x68 not found)");
#else
    sensor_manager_init();   // 仅创建互斥锁
    LOG_I("SYS", "local MPU disabled (rx-only mode)");
#endif

#if ENABLE_BUTTON
    button_init();
    LOG_I("SYS", "cancel button init ok (GPIO%d)", PIN_BTN_CANCEL);
#endif

#if ENABLE_ESPNOW
    espnow_init();
#else
    LOG_I("SYS", "ESP-NOW disabled");
#endif

#if ENABLE_WIFI_PUSH
    pusher_init();   // 推送队列 + 最低优先级推送任务
    LOG_I("SYS", "wifi push enabled");
#endif

    // —— 创建任务（core1）——
    xTaskCreatePinnedToCore(task_sensor, "sensor", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(task_detect, "detect", 4096, nullptr, 3, nullptr, 1);
#if ENABLE_BUTTON
    xTaskCreatePinnedToCore(task_button, "button", 2048, nullptr, 2, nullptr, 1);
#endif
    xTaskCreatePinnedToCore(task_alarm,  "alarm",  2048, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(task_stat,   "stat",   2048, nullptr, 1, nullptr, 1);
#if ENABLE_NTP_BOOT_SYNC
    xTaskCreatePinnedToCore(task_timesync, "timesync", 4096, nullptr, 1, nullptr, 1);
#endif

    LOG_I("SYS", "all tasks started");
}

// 全部工作在 FreeRTOS 任务中，loop 空转
void loop() {
    vTaskDelay(portMAX_DELAY);
}
