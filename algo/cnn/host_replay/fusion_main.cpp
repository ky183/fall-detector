// ============================================================
//  fusion_main.cpp —— 两级判决（RF 主判决 + CNN 二级仲裁）集成回放
//
//  由 algo/cnn/host_replay/replay.py 构建（Test C）。
//  目的：在 PC 上跑**真实的** fall_detector.cpp + cnn_detector.cpp，
//  验证三件事（都不能只靠离线指标覆盖）：
//    1. 状态机仍能走完 基线 -> 触发 -> 采集 -> 判定，且能重新复位
//    2. 仲裁门控正确：腕端在线才跑 CNN，离线/覆盖率不足则完全不跑
//       （行为回退到纯 RF，即 v2 已验证的那条路）
//    3. 判定结果通过 FallDebug 可观测（串口回放/答辩取证要用）
//
//  时间由 g_millis 显式推进（20ms/拍 = 50Hz），保证回放可复现。
// ============================================================
#include "Arduino.h"      // 桩：Serial / millis / GPIO / tone
#include "config.h"
#include "fall_detector.h"
#include "alarm_manager.h"   // 报警状态机（端到端回放）
#include "hw_button.h"
#include "hw_buzzer.h"
#include "net_pusher.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// 桩的全局（g_millis / Serial / GPIO 计数器）定义在 Arduino.h 里
// （C++17 inline 变量，多个 TU 共一份），这里不再重复定义

static SensorData mk(float ax, float ay, float az,
                     float gx, float gy, float gz) {
    SensorData d = {};
    d.ax = ax; d.ay = ay; d.az = az;
    d.gx = gx; d.gy = gy; d.gz = gz;
    d.svm = sqrtf(ax * ax + ay * ay + az * az);
    d.ts  = g_millis;
    return d;
}

// 静止站立姿态；跌后躺倒姿态（均含角速度 0）
#define STAND  0.0f, 0.0f, 9.81f,  0.0f, 0.0f, 0.0f
#define LYING  0.0f, 9.81f, 0.0f,  0.0f, 0.0f, 0.0f

struct Result {
    bool      decided;
    bool      fired;
    FallDebug dbg;
};

// wristOnline=false 模拟腕端掉线（hasRemote=0）
// wristMoves=false 模拟"装置被单独磕碰"（腕端在线但几乎没动）
static Result run(const char* name, bool wristOnline, bool wristMoves) {
    fall_detector_reset();
    g_millis = 0;

    Result r = {};
    memset(&r.dbg, 0, sizeof(r.dbg));

    for (int phase = 0; phase < 3; ++phase) {
        int steps = (phase == 0) ? 260 : (phase == 1 ? 20 : 190);
        for (int i = 0; i < steps; ++i) {
            g_millis += 20;                       // 50Hz

            FallInput in = {};
            switch (phase) {
            case 0:  in.local = mk(STAND);   break;                  // 开机站姿基线
            case 1:  in.local = mk(18, -6, 26, 0, 0, 150); break;    // 冲击（svm≈32>30 触发）
            default: in.local = mk(LYING);   break;                  // 跌后躺倒不动
            }

            in.hasRemote = wristOnline;
            if (!wristOnline) {
                in.remote = SensorData{};
            } else if (phase == 1 && wristMoves) {
                in.remote = mk(10, 5, 14, 0, 0, 180);                // 手臂联动
            } else if (phase >= 1 && wristMoves) {
                in.remote = mk(LYING);                               // 随身体躺倒
            } else {
                in.remote = mk(STAND);                               // 腕端纹丝不动
            }
            in.remoteAgeMs = 20;                                     // 链路新鲜

            if (fall_detector_update(in) == FALL_CONFIRMED) r.fired = true;

            const FallDebug& d = fall_detector_debug();
            if (d.valid && !r.decided) { r.decided = true; r.dbg = d; }
        }
    }

    printf("SCENARIO %-22s decided=%d fired=%d rf=%.4f cnnRan=%d cnnP=%.4f "
           "cnnCov=%.3f vetoed=%d rescued=%d\n",
           name, r.decided ? 1 : 0, r.fired ? 1 : 0, (double)r.dbg.proba,
           r.dbg.cnnRan ? 1 : 0, (double)r.dbg.cnnProba,
           (double)r.dbg.cnnCoverage, r.dbg.cnnVetoed ? 1 : 0,
           r.dbg.cnnRescued ? 1 : 0);
    return r;
}

// ---- 报警全流程：触发 -> 取消窗口 -> 推送入队 -> 按钮取消 -> 复位 ----
static void check(bool ok, const char* what, int* fail) {
    printf("  %s %s\n", ok ? "[ok]" : "[!!]", what);
    if (!ok) (*fail)++;
}

static void test_alarm_flow(int* fail) {
    printf("SCENARIO %-22s -> 报警全流程\n", "alarm_flow");
    g_millis = 0;
    g_tone_calls = g_notone_calls = 0;
    g_last_digital_read = HIGH;
    fall_detector_reset();
    alarm_init();

    alarm_trigger("algo");
    check(alarm_state() == ST_WAIT_CANCEL, "触发 -> WAIT_CANCEL", fail);

    alarm_trigger("sim");
    check(alarm_state() == ST_WAIT_CANCEL, "报警中重复触发被忽略（不重入）", fail);

    // 走完取消窗口：应转入 SENT 并让推送入队（队列满/未启用不影响状态机）
    g_millis += ALARM_CANCEL_WINDOW_MS + 100;
    alarm_tick();
    check(alarm_state() == ST_SENT, "取消窗口超时 -> SENT（推送入队）", fail);

    // 蜂窝节拍：连续 tick 观察蜂鸣器调用
    for (int i = 0; i < 20; ++i) { g_millis += ALARM_TICK_MS; alarm_tick(); }
#if ENABLE_BUZZER
    check(g_tone_calls > 0, "报警期间蜂鸣器被驱动过", fail);
#else
    printf("  [--] ENABLE_BUZZER=0，跳过蜂鸣器断言（tone 调用 %d 次）\n", g_tone_calls);
#endif

    // 按钮取消：模拟按下 + 去抖确认
    g_last_digital_read = LOW;
    bool pressed = false;
    for (int i = 0; i < 10; ++i) {
        g_millis += BUTTON_POLL_MS;
        if (button_poll() == CBTN_PRESSED) { pressed = true; break; }
    }
    check(pressed, "按钮去抖后报出按下事件", fail);

    alarm_cancel();
    check(alarm_state() == ST_NORMAL, "按钮取消 -> NORMAL", fail);
    check(!fall_detector_debug().valid, "取消同时复位了算法（防连环触发）", fail);
    check(alarm_elapsed_ms() == 0, "非报警态 elapsed 归零", fail);

    alarm_cancel();
    check(alarm_state() == ST_NORMAL, "NORMAL 下重复取消幂等", fail);

    // 取消后必须能重新走一遍（否则一次性设备）
    Result r = run("after_cancel", true, true);
    check(r.decided, "取消后能重新基线并再次判定", fail);
}

int main(void) {
    int fail = 0;

    // 1) 跌倒 + 腕端在线：状态机必须判定，且 CNN 必须真的跑了
    Result a = run("fall+wrist_online", true, true);
    if (!a.decided) { printf("  !! 未走到判定\n"); fail++; }
    if (!a.dbg.cnnRan) { printf("  !! 腕端在线却未跑 CNN\n"); fail++; }
    else if (!(a.dbg.cnnProba >= 0.0f && a.dbg.cnnProba <= 1.0f)) {
        printf("  !! CNN 概率越界 %.6f\n", (double)a.dbg.cnnProba); fail++;
    }
    if (a.dbg.cnnCoverage < CNN_MIN_COVERAGE) {
        printf("  !! 腕端全程在线但覆盖率 %.3f 不达标\n", (double)a.dbg.cnnCoverage); fail++;
    }

    // 2) 跌倒 + 腕端离线：必须仍然判定，但 CNN 一次都不能跑（失效开放）
    Result b = run("fall+wrist_offline", false, false);
    if (!b.decided) { printf("  !! 腕端离线时未走到判定（应回退纯 RF）\n"); fail++; }
    if (b.dbg.cnnRan) { printf("  !! 腕端离线却跑了 CNN\n"); fail++; }
    if (b.dbg.cnnProba != -1.0f) {
        printf("  !! 未跑 CNN 时 cnnProba 应为 -1，实为 %.4f\n", (double)b.dbg.cnnProba); fail++;
    }

    // 3) 装置被磕碰（腰端强冲击、腕端纹丝不动）：CNN 应参与，供其否决
    Result c = run("knock+wrist_still", true, false);
    if (!c.decided) { printf("  !! 未走到判定\n"); fail++; }
    if (!c.dbg.cnnRan) { printf("  !! 腕端在线却未跑 CNN\n"); fail++; }

    // 4) 复位后必须能重新走一遍（报警取消 -> 重新基线）
    Result d = run("after_reset", true, true);
    if (!d.decided) { printf("  !! 复位后无法再次判定\n"); fail++; }

    // 5) 报警全流程（状态机 + 蜂鸣器 + 推送入队 + 按钮取消）
    test_alarm_flow(&fail);

    if (fail) printf("FUSION FAIL (%d)\n", fail);
    else      printf("FUSION PASS\n");
    return fail ? 1 : 0;
}
