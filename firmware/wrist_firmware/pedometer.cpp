// ============================================================
//  步数统计（腕端）实现
//  峰值检测状态机：
//    静止(below) --EMA超阈--> 记一步 + 进入 above（上升沿+间隔窗）
//    above  --EMA低于阈值的迟滞线下沿--> 回 below（防一次摆动记多步）
//  参数集中在 config.h 的 PEDO_*，可现场调灵敏度。
// ============================================================
#include "pedometer.h"
#include "config.h"

#if ENABLE_PEDOMETER

static uint32_t s_steps = 0;          // 累计步数（RAM，重启清零）
static float    s_ema   = 0.0f;       // 动态幅值的 EMA 平滑值
static bool     s_above = false;      // 当前是否处于"过阈"段
static uint32_t s_last_step_ms = 0;   // 最近一次记步时刻（间隔窗判据）

void pedometer_feed(float svm) {
    // 动态幅值：SVM 围绕重力 g 波动，去重力即得运动强度
    float d = fabsf(svm - 9.81f);
    s_ema += PEDO_EMA_ALPHA * (d - s_ema);          // 一阶低通，去高频毛刺

    uint32_t now = millis();
    if (!s_above) {
        if (s_ema >= PEDO_THRESHOLD_MS2) {
            // 上升沿 + 最小间隔窗（防一次摆臂记多步）
            if (now - s_last_step_ms >= PEDO_MIN_GAP_MS) {
                s_steps++;
                s_last_step_ms = now;
            }
            s_above = true;
        }
    } else {
        // 迟滞下沿（阈值的一半）才回落，避免临界抖动
        if (s_ema < PEDO_THRESHOLD_MS2 * 0.5f) {
            s_above = false;
        }
        // 超时强制回落：长时间过阈（如持续甩手）不无限累积状态
        if (now - s_last_step_ms > PEDO_MAX_GAP_MS) {
            s_above = false;
        }
    }
}

uint32_t pedometer_get_steps(void) { return s_steps; }

void pedometer_reset(void) {
    s_steps = 0;
    s_ema = 0.0f;
    s_above = false;
    s_last_step_ms = 0;
}

#else   // ---------- 关闭时的空实现 ----------

void     pedometer_feed(float svm) { (void)svm; }
uint32_t pedometer_get_steps(void) { return 0; }
void     pedometer_reset(void) {}

#endif
