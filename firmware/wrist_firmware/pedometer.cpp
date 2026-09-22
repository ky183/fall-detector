// ============================================================
//  步数统计（腕端）实现
//  两级防误报（v0.6 调参版）：
//    1. 阈值+迟滞：|SVM-g| 平滑后过 2.5 m/s² 才算候选峰
//    2. 连续步确认：孤立晃动（1~3 峰）不计数；连续走出第 4 步
//       时把 pending 一起补记；停走 >2s 重新确认
//  参数集中在 config.h 的 PEDO_*，可现场调灵敏度。
// ============================================================
#include "pedometer.h"
#include "config.h"

#if ENABLE_PEDOMETER

static uint32_t s_steps = 0;          // 已确认累计步数（RAM，重启清零）
static float    s_ema   = 0.0f;       // 动态幅值的 EMA 平滑值
static bool     s_above = false;      // 当前是否处于"过阈"段（迟滞防抖）
static uint32_t s_last_step_ms = 0;   // 最近一次候选峰时刻（最小间隔窗判据）
static uint8_t  s_pending = 0;        // 待确认的连续候选步数
static bool     s_active = false;     // 行走已确认（持续记步中）
static uint32_t s_last_peak_ms = 0;   // 最近一次任意候选峰（中断判据）

void pedometer_feed(float svm) {
    // 动态幅值：SVM 围绕重力 g 波动，去重力即得运动强度
    float d = fabsf(svm - 9.81f);
    s_ema += PEDO_EMA_ALPHA * (d - s_ema);          // 一阶低通，去高频毛刺

    uint32_t now = millis();

    // —— 上升沿 = 一个候选步（最小间隔窗内才算，防一次摆臂记多步）——
    if (!s_above && s_ema >= PEDO_THRESHOLD_MS2 &&
        now - s_last_step_ms >= PEDO_MIN_GAP_MS) {
        s_above = true;

        if (now - s_last_peak_ms <= PEDO_MAX_GAP_MS) {
            // 与上一步间隔正常：连续行走中
            if (s_active) {
                s_steps++;                       // 已确认：直接记
            } else {
                s_pending++;
                if (s_pending >= PEDO_CONFIRM_STEPS) {
                    s_steps += s_pending;        // 连续步确认成立：补记全部
                    s_pending = 0;
                    s_active = true;
                }
            }
        } else {
            // 间隔太久：之前的 pending 是孤立动作，丢弃，从本步重新数
            s_pending = 1;
            s_active = false;
        }
        s_last_step_ms  = now;
        s_last_peak_ms  = now;
    }

    // 迟滞下沿（阈值一半）才回落，避免临界抖动
    if (s_above && s_ema < PEDO_THRESHOLD_MS2 * 0.5f) {
        s_above = false;
    }

    // 行走中断（悬空的 pending 丢弃，重新等连续步）
    if (s_pending > 0 && now - s_last_peak_ms > PEDO_MAX_GAP_MS) {
        s_pending = 0;
        s_active  = false;
    }
}

uint32_t pedometer_get_steps(void) { return s_steps; }

void pedometer_reset(void) {
    s_steps = 0;
    s_ema = 0.0f;
    s_above = false;
    s_last_step_ms = 0;
    s_pending = 0;
    s_active = false;
    s_last_peak_ms = 0;
}

#else   // ---------- 关闭时的空实现 ----------

void     pedometer_feed(float svm) { (void)svm; }
uint32_t pedometer_get_steps(void) { return 0; }
void     pedometer_reset(void) {}

#endif
