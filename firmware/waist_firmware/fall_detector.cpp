// ============================================================
//  ★ 跌倒判定算法 — 占位实现（算法组从这里开始）★
//
//  当前行为：恒返回 FALL_NONE（不判定）。
//  ENABLE_FALL_DETECT=0 时本模块不会被调用（task_detect 直接跳过），
//  报警链路由腕端"长按模拟跌倒"命令触发，用于联调。
//
//  实现建议（算法组可自由推翻）：
//    阶段1：单腰端特征——SVM 超阈值(冲击) + 姿态/静止判据 + 时间窗
//    阶段2：融合腕端数据（in.hasRemote && in.remoteAgeMs < 200）
// ============================================================
#include "fall_detector.h"
#include "logger.h"
#include "config.h"

FallEvent fall_detector_update(const FallInput& in) {
    // TODO(算法组)：在此实现判定逻辑，参考签名：
    //   输入 in.local / in.remote（SensorData，单位见 fall_detector.h）
    //   返回 FALL_CONFIRMED 即触发报警（alarm_manager 会接手）
    (void)in;
    return FALL_NONE;
}

void fall_detector_reset(void) {
    // TODO(算法组)：复位内部状态机/滑动窗口
}
