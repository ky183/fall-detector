#pragma once
// ============================================================
//  ★ 跌倒判定算法接口（算法组只改 fall_detector.cpp）★
//
//  调用方式：task_detect 以 50Hz 调用 fall_detector_update()，
//            传入两端传感器数据快照，返回本次判定结果。
//
//  算法组注意事项：
//    1. 只需实现 fall_detector.cpp 里的 fall_detector_update()
//       和 fall_detector_reset()，不需要碰任何其他文件
//    2. 输入数据已换算好单位：加速度 m/s²（静止约 9.8）、角速度 deg/s
//    3. hasRemote=false 表示腕端离线，算法需降级为单传感器判定
//    4. 本函数运行在 FreeRTOS 任务中，请勿使用 delay()，
//       单次执行时间需 <10ms（不影响 50Hz 节拍）
//    5. 状态机/滑动窗口等内部状态用 static 或类成员保存即可
//    6. 完成后把 config.h 的 ENABLE_FALL_DETECT 置 1 启用
//
//  参考特征（民生赛道常见做法，可自由发挥）：
//    SVM 突变（冲击检测）+ 姿态角变化（跌倒后近水平）+ 时间窗口
// ============================================================
#include "sensor_manager.h"   // FallInput / SensorData

enum FallEvent {
    FALL_NONE     = 0,   // 无事件
    FALL_CONFIRMED = 1,  // 确认跌倒（触发报警）
};

// 50Hz 调用：输入两端数据快照，输出判定事件
FallEvent fall_detector_update(const FallInput& in);

// 复位内部状态（报警取消后调用，防止重复触发）
void fall_detector_reset(void);
