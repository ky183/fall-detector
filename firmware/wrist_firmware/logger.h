#pragma once
// ============================================================
//  日志工具（腕端）
//  统一格式：[wrist][模块标签] 消息
//  级别由 config.h 的 LOG_LEVEL 控制：
//    1=仅错误  2=+关键信息  3=+调试心跳（默认）
//  用法示例：
//    LOG_E("SENS", "MPU init failed");
//    LOG_I("NET",  "ESP-NOW started, ch=%d", ESPNOW_CHANNEL);
//    LOG_D("SENS", "svm=%.2f", svm);
// ============================================================
#include <Arduino.h>
#include "config.h"

#define LOG_E(tag, ...) do { if (LOG_LEVEL >= 1) { \
    Serial.printf("[wrist][" tag "] E: " __VA_ARGS__); Serial.println(); } } while (0)

#define LOG_I(tag, ...) do { if (LOG_LEVEL >= 2) { \
    Serial.printf("[wrist][" tag "] " __VA_ARGS__); Serial.println(); } } while (0)

#define LOG_D(tag, ...) do { if (LOG_LEVEL >= 3) { \
    Serial.printf("[wrist][" tag "] " __VA_ARGS__); Serial.println(); } } while (0)
