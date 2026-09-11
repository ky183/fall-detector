#pragma once
// ============================================================
//  ★ CNN 二级仲裁 — 1D-CNN 前向推理接口（纯原生 C++，零库依赖）★
//
//  角色：**第二级**判决，不是主判决。
//    主判决 = fall_detector.cpp（三阶段冲击触发 + 小随机森林）。
//    CNN 只在"腰端已触发冲击、且判定时腕端链路在线"时补跑一次，
//    每次事件最多 1 次推理；腕端离线或覆盖不足则完全不跑。
//    好处：拿到 RF 用不到的腕端 6 通道信息，而 CPU 开销可忽略。
//
//  数据流（都由 fall_detector.cpp 驱动，外部无需直接调用）：
//    fall_detector_update() 每拍 50Hz 调 cnn_detector_push() 写历史
//    fall_detector 的 decide() 判定时调 cnn_detector_prob() 取一次结果
//
//  与训练侧（algo/cnn/cnn_train.py）必须一致的四项，见 cnn_detector.cpp 头注释
// ============================================================
#include "sensor_manager.h"   // FallInput / SensorData

// 50Hz 历史缓存长度，由"仲裁窗最远能落到哪里"倒推：
//   判定时事件缓冲最长 EV_N=320 拍（见 fall_detector.cpp），锚点又可能
//   落在缓冲最前端，故 ageSamples <= 319；取窗需 age+49 拍历史
//   -> 368 拍，向上取 384。这样任何一次判定都能取到完整窗，
//   不会静默降级为"只用 RF"。
//   （开机后 5s 基线 + 3.5s 采集 = 8.5s > 384 拍 = 7.68s，足够填满）
// 内存：12 通道 × 384 拍 × 4B ≈ 18.4KB（BSS，不在栈上）
#define CNN_HIST_N   384

// 每次 50Hz 采样调用：把两端数据按训练侧口径归一化后写入内部历史
void cnn_detector_push(const FallInput& in);

// 取"中心位于 ageSamples 个 50Hz 样本之前"的 2s 窗做一次推理。
//   ageSamples : 锚点距今多少个 50Hz 样本（＝触发判定用的冲击峰值位置）
//   minCoverage: 该窗内腕端数据覆盖率下限，低于此值视为"腕端不可信"
//   返回 false = 历史未填满或腕端覆盖不足，本次未推理（*outProb 不写出）
bool cnn_detector_prob(int ageSamples, float minCoverage,
                       float* outProb, float* outCoverage);

// 清空历史（由 fall_detector_reset() 调用）
void cnn_detector_reset(void);
