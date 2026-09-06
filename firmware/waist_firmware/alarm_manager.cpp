// ============================================================
//  报警管理（腰端）实现
// ============================================================
#include "alarm_manager.h"
#include "logger.h"
#include "config.h"
#include "hw_buzzer.h"
#include "net_pusher.h"
#include "fall_detector.h"

static AlarmState s_state = ST_NORMAL;
static uint32_t   s_start_ms = 0;
static bool       s_beeping = false;   // 当前哔声相位（避免重复调 tone）

void alarm_init(void) {
    buzzer_init();
    buzzer_beep_off();
    s_state = ST_NORMAL;
}

void alarm_trigger(const char* src) {
    if (s_state != ST_NORMAL) return;   // 已在报警中，不重复触发
    s_state    = ST_WAIT_CANCEL;
    s_start_ms = millis();
    LOG_I("ALRM", "ALARM triggered (src=%s), cancel window %ums", src, ALARM_CANCEL_WINDOW_MS);
}

void alarm_cancel(void) {
    if (s_state == ST_NORMAL) return;
    bool pushed = (s_state == ST_SENT);
    s_state = ST_NORMAL;
    buzzer_beep_off();
    s_beeping = false;
    fall_detector_reset();   // 复位算法状态，防止连环触发
    LOG_I("ALRM", "alarm canceled by button (pushed=%d)", pushed);
}

void alarm_tick(void) {
    if (s_state == ST_NORMAL) return;

    uint32_t elapsed = millis() - s_start_ms;

    // —— 取消窗口超时：推送（只推一次），进入 SENT 继续响 ——
    if (s_state == ST_WAIT_CANCEL && elapsed >= ALARM_CANCEL_WINDOW_MS) {
        s_state = ST_SENT;
        bool ok = pusher_send_fall_alert(elapsed / 1000);
        LOG_I("ALRM", "cancel window over -> push alert (ok=%d), keep buzzing", ok);
    }

    // —— 哔哔节拍：按绝对时间取相位，不依赖 tick 对齐 ——
    bool on = (elapsed % ALARM_BEEP_PERIOD_MS) < ALARM_BEEP_ON_MS;
    if (on != s_beeping) {
        s_beeping = on;
        on ? buzzer_beep_on() : buzzer_beep_off();
    }
}

AlarmState alarm_state(void)     { return s_state; }
uint32_t   alarm_elapsed_ms(void) {
    return (s_state == ST_NORMAL) ? 0 : (millis() - s_start_ms);
}
