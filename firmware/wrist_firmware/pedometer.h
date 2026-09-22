#pragma once
// ============================================================
//  步数统计（腕端）— v0.6，纯本地：不通信、不占协议、腰端零依赖
//
//  数据源：本机 50Hz SVM（task_sensor 每帧喂入）
//  算法：动态幅值 |SVM-g| → EMA 低通 → 阈值+迟滞+间隔窗记步
//        （规则法，不训练模型；手环级精度足够，演示/健康指标用）
//  存储：RAM 计数，重启清零（调试期方案；日后要持久化加 NVS 落盘即可）
// ============================================================
#include <Arduino.h>

void     pedometer_feed(float svm);     // 每帧喂入最新 SVM（task_sensor 50Hz 调用）
uint32_t pedometer_get_steps(void);     // 当前累计步数（task_display 读取）
void     pedometer_reset(void);         // 清零（预留：跨日/按钮复位用）
