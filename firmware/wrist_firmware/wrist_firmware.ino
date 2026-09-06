// ============================================================
//  腕端固件入口 — XIAO ESP32S3
//  智能跌倒检测报警器 · 清华第29届硬件设计大赛
//  职责：采集姿态 → OLED 显示 → ESP-NOW 发送 → 腰端
//
//  任务一览（FreeRTOS，全部跑在 core1，WiFi 协议栈在 core0）：
//    task_sensor   50Hz  高   读 MPU → 写共享数据 g_sensor_data
//    task_tx       50Hz  高   读共享数据 → ESP-NOW 发送
//    task_button   20Hz  中   按钮扫描（去抖+短/长按）
//    task_display   2Hz  低   OLED 刷新
//
//  跌倒判定不在腕端：ENABLE_FALL_DETECT=0 时长按按钮发"模拟跌倒"
//  命令给腰端，用于算法到位前调通整条报警链路
// ============================================================
#include "config.h"
#include "logger.h"
#include "protocol.h"
#include "sensor_manager.h"
#include "ui_oled.h"
#include "ui_button.h"
#include "net_espnow.h"
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ================= 运行状态（display/日志共用） =================
static volatile uint32_t g_evt_cnt   = 0;   // 按钮事件累计
static volatile uint32_t g_sim_until = 0;   // 模拟跌倒提示截止时刻（OLED 显示用）

// ================= 任务实现 =================

// 传感器采样：严格 50Hz（vTaskDelayUntil 保证周期稳定，不累积误差）
static void task_sensor(void* pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
    for (;;) {
        vTaskDelayUntil(&last, period);
        if (!sensor_manager_read()) {
            // MPU 离线（read 内部已重试），降频打印避免刷屏
            static uint32_t lastErr = 0;
            if (millis() - lastErr > 1000) {
                LOG_E("SENS", "MPU offline, retrying...");
                lastErr = millis();
            }
        }
    }
}

// 无线发送：复制一份共享数据快照再打包（持锁时间最短化）
static void task_tx(void* pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
    for (;;) {
        vTaskDelayUntil(&last, period);
        SensorData snap;
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10))) {
            snap = g_sensor_data;
            xSemaphoreGive(g_sensor_mutex);
        } else {
            continue;   // 偶发拿不到锁：跳过本帧，不阻塞任务
        }
        espnow_send_data(snap);
    }
}

// 按钮扫描：短按=交互（翻页 TODO），长按=模拟跌倒（调试）
static void task_button(void* pv) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        ButtonEvent e = button_poll();
        if (e == BTN_NONE) continue;

        g_evt_cnt++;
        if (e == BTN_SHORT) {
            LOG_I("BTN", "short press (evt=%lu)", (unsigned long)g_evt_cnt);
            // TODO(交互)：OLED 多页显示时在此切换页面
        } else {  // BTN_LONG
            LOG_I("BTN", "long press -> SIM FALL");
#if ENABLE_FALL_DETECT
            // 算法启用后，模拟功能关闭（避免干扰真实判定）
            LOG_I("BTN", "fall detect enabled, sim ignored");
#else
            espnow_send_cmd(CMD_SIM_FALL);
            g_sim_until = millis() + 3000;   // OLED 提示 3 秒
#endif
        }
    }
}

// OLED 刷新：组装 UiInfo → 刷新（慢速低优先级，绝不干扰采样）
static void task_display(void* pv) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));

        UiInfo info = {};
        info.alarm     = false;                       // 报警状态在腰端，腕端骨架恒 NORMAL
        info.simActive = (millis() < g_sim_until);
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10))) {
            info.svm = g_sensor_data.svm;
            xSemaphoreGive(g_sensor_mutex);
        }
        info.uptime = millis() / 1000;
        info.evtCnt = g_evt_cnt;
        info.linkOk = g_espnow_last_ok;
        oled_update(info);
    }
}

// ================= 入口 =================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    LOG_I("SYS", "=== wrist firmware v0.1 (skeleton) ===");

    // I2C 总线先行（OLED 与 MPU 共用）
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    // 保持默认 100kHz：补焊总线信号质量有限，400kHz 实测不稳（MPU 离线）
    // 50Hz 采样每帧约 14 字节，100kHz 下仅占 1.3ms，带宽余量充足

    // —— 各模块初始化：失败不阻塞（对应开关关掉/硬件后补，其余照常跑）——
#if ENABLE_MPU6050
    LOG_I("SYS", "MPU init ... %s", sensor_manager_init() ? "ok" : "FAIL(0x68 not found)");
#else
    sensor_manager_init();   // 仅创建互斥锁，填充模拟数据
    LOG_I("SYS", "MPU disabled, mock data mode");
#endif

#if ENABLE_OLED
    LOG_I("SYS", "OLED init ... %s", oled_init() ? "ok" : "FAIL(0x3C not found)");
    oled_show_boot();
#else
    LOG_I("SYS", "OLED disabled");
#endif

#if ENABLE_BUTTON
    button_init();
    LOG_I("SYS", "button init ok (GPIO%d)", PIN_BTN_1);
#endif

#if ENABLE_ESPNOW
    espnow_init();
#else
    LOG_I("SYS", "ESP-NOW disabled");
#endif

    // —— 创建任务（core1；栈大小按 printf 浮点需求留足）——
    xTaskCreatePinnedToCore(task_sensor,  "sensor",  4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(task_tx,      "tx",      4096, nullptr, 3, nullptr, 1);
#if ENABLE_BUTTON
    xTaskCreatePinnedToCore(task_button,  "button",  2048, nullptr, 2, nullptr, 1);
#endif
#if ENABLE_OLED
    xTaskCreatePinnedToCore(task_display, "display", 4096, nullptr, 1, nullptr, 1);
#endif

    LOG_I("SYS", "all tasks started");
}

// 全部工作在 FreeRTOS 任务中，loop 空转（永久让出）
void loop() {
    vTaskDelay(portMAX_DELAY);
}
