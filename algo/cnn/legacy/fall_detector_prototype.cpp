// ============================================================
//  ⚠ 已废弃（仅存档，不参与编译）⚠
//  这是 CNN 接入固件前的原型：它把整个 1D-CNN 当成主判决，
//  直接实现了 fall_detector_update/reset，与固件的 RF 版同名冲突。
//  正式实现已改为二级仲裁，见：
//    firmware/waist_firmware/cnn_detector.cpp   （前向推理，静态缓冲）
//    firmware/waist_firmware/fall_detector.cpp  （RF 主判决 + CNN 仲裁）
//  保留本文件只为追溯原始写法；两处差异：
//    1. 旧版工作数组在栈上（单帧峰值 ~18KB），会撑爆 task_detect 的 4KB 栈
//    2. 旧版陀螺归一化写成 deg/s ÷ 2000，与训练侧口径不符（差 ~65 倍）
// ============================================================
//  ★ 跌倒判定算法 — 纯原生 C++ 1D-CNN 前向推理引擎 (零库依赖) ★
//  无需任何 TensorFlow/TFLite 库，任意 ESP32 编译器均可直接编译通过
// ============================================================
#include "fall_detector.h"
#include "logger.h"
#include "config.h"
#include "cnn_weights.h"  // 纯权重数组
#include <math.h>
#include <string.h>

// 1. 传感器字段提取宏
#define GET_ACC_X(s) ((s).ax)
#define GET_ACC_Y(s) ((s).ay)
#define GET_ACC_Z(s) ((s).az)
#define GET_GYR_X(s) ((s).gx)
#define GET_GYR_Y(s) ((s).gy)
#define GET_GYR_Z(s) ((s).gz)

constexpr float FALL_PROB_THRESHOLD = 0.80f;
constexpr int WINDOW_LEN = 80;
constexpr int CHANNELS   = 12;

// 滑动窗口环形缓冲区
static float ring_buffer[WINDOW_LEN][CHANNELS] = {0};
static int buffer_head   = 0;
static int total_samples = 0;
static uint8_t downsample_cnt = 0;
static uint8_t step_counter   = 0;

// ================= 原生 1D-CNN 前向传播函数 =================
static float run_native_1dcnn(const float input_data[80][12]) {
    // ---- Layer 1: Conv1D (kernel=5, pad=same, filters=16, relu) ----
    float c1[80][16] = {0};
    for (int t = 0; t < 80; ++t) {
        for (int f = 0; f < 16; ++f) {
            float sum = g_conv1_b[f];
            for (int k = 0; k < 5; ++k) {
                int in_t = t + k - 2; // same padding (center 2)
                if (in_t >= 0 && in_t < 80) {
                    for (int c = 0; c < 12; ++c) {
                        sum += input_data[in_t][c] * g_conv1_w[(k * 12 + c) * 16 + f];
                    }
                }
            }
            c1[t][f] = (sum > 0.0f) ? sum : 0.0f; // ReLU
        }
    }

    // ---- Layer 2: MaxPool1D (pool_size=2) -> shape: (40, 16) ----
    float p1[40][16];
    for (int t = 0; t < 40; ++t) {
        for (int f = 0; f < 16; ++f) {
            float v1 = c1[2 * t][f];
            float v2 = c1[2 * t + 1][f];
            p1[t][f] = (v1 > v2) ? v1 : v2;
        }
    }

    // ---- Layer 3: Conv1D (kernel=3, pad=same, filters=32, relu) ----
    float c2[40][32] = {0};
    for (int t = 0; t < 40; ++t) {
        for (int f = 0; f < 32; ++f) {
            float sum = g_conv2_b[f];
            for (int k = 0; k < 3; ++k) {
                int in_t = t + k - 1; // same padding (center 1)
                if (in_t >= 0 && in_t < 40) {
                    for (int c = 0; c < 16; ++c) {
                        sum += p1[in_t][c] * g_conv2_w[(k * 16 + c) * 32 + f];
                    }
                }
            }
            c2[t][f] = (sum > 0.0f) ? sum : 0.0f; // ReLU
        }
    }

    // ---- Layer 4: MaxPool1D (pool_size=2) -> shape: (20, 32) ----
    float p2[20][32];
    for (int t = 0; t < 20; ++t) {
        for (int f = 0; f < 32; ++f) {
            float v1 = c2[2 * t][f];
            float v2 = c2[2 * t + 1][f];
            p2[t][f] = (v1 > v2) ? v1 : v2;
        }
    }

    // ---- Layer 5: Conv1D (kernel=3, pad=same, filters=32, relu) ----
    float c3[20][32] = {0};
    for (int t = 0; t < 20; ++t) {
        for (int f = 0; f < 32; ++f) {
            float sum = g_conv3_b[f];
            for (int k = 0; k < 3; ++k) {
                int in_t = t + k - 1;
                if (in_t >= 0 && in_t < 20) {
                    for (int c = 0; c < 32; ++c) {
                        sum += p2[in_t][c] * g_conv3_w[(k * 32 + c) * 32 + f];
                    }
                }
            }
            c3[t][f] = (sum > 0.0f) ? sum : 0.0f; // ReLU
        }
    }

    // ---- Layer 6: GlobalAveragePooling1D -> shape: (32) ----
    float gap[32] = {0};
    for (int f = 0; f < 32; ++f) {
        float sum = 0.0f;
        for (int t = 0; t < 20; ++t) {
            sum += c3[t][f];
        }
        gap[f] = sum / 20.0f;
    }

    // ---- Layer 7: Dense (units=16, relu) ----
    float d1[16] = {0};
    for (int i = 0; i < 16; ++i) {
        float sum = g_dense1_b[i];
        for (int j = 0; j < 32; ++j) {
            sum += gap[j] * g_dense1_w[j * 16 + i];
        }
        d1[i] = (sum > 0.0f) ? sum : 0.0f; // ReLU
    }

    // ---- Layer 8: Dense (units=2, softmax) ----
    float logits[2] = {g_dense2_b[0], g_dense2_b[1]};
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 16; ++j) {
            logits[i] += d1[j] * g_dense2_w[j * 2 + i];
        }
    }

    // Softmax 提取跌倒类别概率 (Index 1)
    float max_l = (logits[0] > logits[1]) ? logits[0] : logits[1];
    float exp0 = expf(logits[0] - max_l);
    float exp1 = expf(logits[1] - max_l);
    float fall_prob = exp1 / (exp0 + exp1);

    return fall_prob;
}

// ================= 对外接口实现 =================

FallEvent fall_detector_update(const FallInput& in) {
    // 采样率转换 (50Hz -> 40Hz: 每 5 拍丢 1 拍)
    downsample_cnt++;
    if (downsample_cnt >= 5) {
        downsample_cnt = 0;
        return FALL_NONE;
    }

    // 单位换算与归一化
    constexpr float G_INV   = 1.0f / 9.80665f;
    constexpr float GYR_INV = 1.0f / 2000.0f;

    float w_ax = GET_ACC_X(in.local) * G_INV;
    float w_ay = GET_ACC_Y(in.local) * G_INV;
    float w_az = GET_ACC_Z(in.local) * G_INV;
    float w_gx = GET_GYR_X(in.local) * GYR_INV;
    float w_gy = GET_GYR_Y(in.local) * GYR_INV;
    float w_gz = GET_GYR_Z(in.local) * GYR_INV;

    float r_ax = 0.0f, r_ay = 0.0f, r_az = 0.0f;
    float r_gx = 0.0f, r_gy = 0.0f, r_gz = 0.0f;
    if (in.hasRemote) {
        r_ax = GET_ACC_X(in.remote) * G_INV;
        r_ay = GET_ACC_Y(in.remote) * G_INV;
        r_az = GET_ACC_Z(in.remote) * G_INV;
        r_gx = GET_GYR_X(in.remote) * GYR_INV;
        r_gy = GET_GYR_Y(in.remote) * GYR_INV;
        r_gz = GET_GYR_Z(in.remote) * GYR_INV;
    }

    // 写入环形缓冲区
    ring_buffer[buffer_head][0]  = w_ax;
    ring_buffer[buffer_head][1]  = w_ay;
    ring_buffer[buffer_head][2]  = w_az;
    ring_buffer[buffer_head][3]  = w_gx;
    ring_buffer[buffer_head][4]  = w_gy;
    ring_buffer[buffer_head][5]  = w_gz;
    ring_buffer[buffer_head][6]  = r_ax;
    ring_buffer[buffer_head][7]  = r_ay;
    ring_buffer[buffer_head][8]  = r_az;
    ring_buffer[buffer_head][9]  = r_gx;
    ring_buffer[buffer_head][10] = r_gy;
    ring_buffer[buffer_head][11] = r_gz;

    buffer_head = (buffer_head + 1) % WINDOW_LEN;
    if (total_samples < WINDOW_LEN) {
        total_samples++;
        return FALL_NONE;
    }

    // 触发判断: 每 200ms 或检测到冲击时推理
    step_counter++;
    float waist_acc_mag = sqrtf(w_ax * w_ax + w_ay * w_ay + w_az * w_az);
    bool impact_triggered = (waist_acc_mag > 2.2f);

    if (step_counter >= 8 || impact_triggered) {
        step_counter = 0;

        // 展开环形缓冲区至连续输入
        static float ordered_input[80][12];
        for (int i = 0; i < WINDOW_LEN; ++i) {
            int idx = (buffer_head + i) % WINDOW_LEN;
            memcpy(ordered_input[i], ring_buffer[idx], sizeof(float) * 12);
        }

        // 纯 C++ 原生前向推理
        float fall_prob = run_native_1dcnn(ordered_input);

        if (fall_prob >= FALL_PROB_THRESHOLD) {
            LOG_I("ALGO", "Fall Confirmed! Prob: %.2f%% (Impact: %.2fg)", 
                  fall_prob * 100.0f, waist_acc_mag);
            return FALL_CONFIRMED;
        }
    }

    return FALL_NONE;
}

void fall_detector_reset(void) {
    buffer_head   = 0;
    total_samples = 0;
    downsample_cnt= 0;
    step_counter  = 0;
    memset(ring_buffer, 0, sizeof(ring_buffer));
    LOG_I("ALGO", "Fall detector reset.");
}