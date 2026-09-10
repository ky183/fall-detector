#pragma once
// ============================================================
//  ★ 跌倒判定算法接口（已实现：三阶段触发 + 小随机森林）★
//
//  调用方式：task_detect 以 50Hz 调用 fall_detector_update()，
//            传入两端传感器数据快照，返回本次判定结果。
//
//  实现说明（v2，训练/导出全流程见仓库 algo/ 目录）：
//    - 开机先采集站姿基线（FD_BASELINE_MS），期间不判定
//    - 冲击触发（SVM/角速度越限）-> 采集确认窗 -> 特征 -> 随机森林
//    - 判跌倒后锁存，只返回一次 FALL_CONFIRMED，报警取消后由
//      alarm_manager 调 fall_detector_reset() 重新基线
//    - 腕端数据（in.remote）当前未参与判定，预留给融合否决票
//
//  运行约束（FreeRTOS 任务内）：
//    1. 请勿使用 delay()；常态单次 <0.1ms，判定瞬间 <1ms
//    2. 内部状态全部 static，无需外部加锁（仅本任务调用）
//    3. 输入单位：加速度 m/s²（静止约 9.81）、角速度 deg/s
// ============================================================
#include "sensor_manager.h"   // FallInput / SensorData

enum FallEvent {
    FALL_NONE     = 0,   // 无事件
    FALL_CONFIRMED = 1,  // 确认跌倒（触发报警）
};

// 判定过程快照 —— 上板验证/数据回放用：
// 串口可打印 feats/proba 与电脑端回放对照，定位域差异
struct FallDebug {
    bool     valid;          // 是否已完成过一次完整判定
    float    proba;          // 随机森林输出 P(跌倒)，>= FD_RF_THRESHOLD 判跌倒
    float    feats[12];      // 特征向量（顺序见 rf_model.h 头注释）
    uint32_t decideMs;       // 触发 -> 判定的耗时（毫秒）
    float    wristPeak;      // 腕端冲击窗 SVM 峰值（-1=无有效腕端数据）
    float    wristCov;       // 腕端冲击窗数据覆盖率 0~1
    bool     vetoed;         // 本次判定是否被腕端否决票拦截
};

// 50Hz 调用：输入两端数据快照，输出判定事件
FallEvent fall_detector_update(const FallInput& in);

// 复位内部状态并重新采集基线（报警取消后调用，防止重复触发）
void fall_detector_reset(void);

// 取最近一次判定快照（不判定时 valid=false）
const FallDebug& fall_detector_debug(void);
