// ============================================================
//  ★ CNN 二级仲裁 — 纯原生 C++ 1D-CNN 前向推理引擎（零库依赖）★
//
//  模型来源：algo/cnn/cnn_train.py（FallAllD 数据集，40Hz，腰+腕 各 6 通道）
//            训练后导出 firmware/waist_firmware/cnn_weights.h（自动生成，勿手改）
//
//  角色：**第二级**判决。主判决是 fall_detector.cpp 的
//        三阶段冲击触发 + 随机森林；本模块只在"已触发 + 腕端在线"时
//        由 decide() 调用一次，每次事件最多 1 次推理。
//
//  ★ 与训练侧必须逐项一致的四项（改任何一项都要重训并回归）★
//   1. 输入窗 (80,12) @40Hz = 2s
//      通道序 = [腰acc×3, 腰gyr×3, 腕acc×3, 腕gyr×3]
//   2. 归一化（训练侧是"数据集原始 LSB ÷ 除数"）：
//        acc: 原始 LSB / 4096.0    （4096 LSB/g，除完即 g 值）
//        gyr: 原始 LSB / 2000.0
//      板端拿到的是 m/s² 与 deg/s，必须**先换算回数据集原始 LSB**再除。
//      ⚠ 移植最易错的一步：训练侧的 /2000 作用在原始 LSB 上，若板端
//        直接把 deg/s 除以 2000，陀螺 6 通道会整体缩小约 65 倍。
//        实测（FallAllD 全量 22140 窗）该错误会让部署门限 0.80 下的
//        灵敏度从 76.9% 掉到 66.8%，并有 4.7% 的窗判决翻转。
//   3. 采样率 50Hz -> 40Hz：每 5 拍丢 1 拍（实测判决翻转 0.4%，无需抗混叠）
//   4. 权重排布：Keras Conv1D kernel (kernel, in_ch, filters) 行主序
//
//  资源占用：
//    权重 6210 float = 24.3KB flash（const）
//    工作缓冲全部 static（BSS ≈ 36KB）——task_detect 栈只有 4KB，
//    绝不能放栈上（这些数组同帧峰值曾达 ~18KB，会直接栈溢出复位）
//
//  一致性验证：算法侧有一份按本文件同款逻辑写的 Python 参考实现 +
//    g++ 宿主编译回放（见 algo/cnn/README.md），与 RF 的
//    05_export.py 双语言回放同性质，是移植正确性的硬验收。
// ============================================================
#include "cnn_detector.h"
#include "cnn_weights.h"     // 纯权重数组
#include "config.h"
#include <math.h>
#include <string.h>

// ================= 归一化常量（对应训练侧除数，改动必须重训） =================
static constexpr float GRAV_MS2      = 9.80665f;  // 板端 m/s² -> g
static constexpr float ACC_LSB_PER_G = 4096.0f;   // FallAllD 加速度标度（LSB/g）
static constexpr float ACC_DIV       = 4096.0f;   // 训练侧 acc 除数
// FallAllD 数据源的陀螺量程标度：±500 dps 满量程 16 位 -> 32768/500 = 65.536 LSB/dps。
// 与腰端 sensor_manager.cpp 的 GYRO_LSB(=500/32768 deg/s per LSB) 是同一个量程，
// 所以板上值乘 65.536 就能一比一还原成数据集原始 LSB。
// ★ 这个数必须与数据集实际量程一致；改它等于换掉输入分布，必须重训。
static constexpr float GYR_LSB_PER_DPS = 65.536f;
static constexpr float GYR_DIV         = 2000.0f; // 训练侧 gyr 除数

// 统一的换算口径：板端物理量 -> 数据集原始 LSB -> 训练侧归一化值
//   norm = 物理量 × (LSB_PER_UNIT ÷ DIV)
//   acc: (m/s² ÷ 9.80665) × 4096 ÷ 4096 ≡ m/s² ÷ 9.80665
//   gyr: deg/s × 65.536 ÷ 2000          （≠ deg/s ÷ 2000，见文件头 ⚠）
static inline float norm_acc(float ms2) {
    return (ms2 / GRAV_MS2) * (ACC_LSB_PER_G / ACC_DIV);
}
static inline float norm_gyr(float dps) {
    return dps * (GYR_LSB_PER_DPS / GYR_DIV);
}

// ================= 历史环形缓冲（50Hz，已归一化） =================
static float   s_hist[CNN_HIST_N][12];
static uint8_t s_wristOk[CNN_HIST_N];      // 该拍腕端数据是否有效
static int     s_head = 0;                 // 下一个写入位置
static int     s_cnt  = 0;                 // 已累计的有效样本数（封顶 CNN_HIST_N）

// ================= 原生 1D-CNN 前向传播 =================
// 全部工作缓冲 static：见文件头"资源占用"。每个元素都是先写后读，
// 无需每次清零（逐层赋值覆盖，无累加残留）。
static float cnn_forward(const float in[80][12]) {
    static float c1[80][16];
    static float p1[40][16];
    static float c2[40][32];
    static float p2[20][32];
    static float c3[20][32];
    static float gap[32];
    static float d1[16];

    // ---- Layer 1: Conv1D (kernel=5, pad=same, filters=16, relu) ----
    for (int t = 0; t < 80; ++t) {
        for (int f = 0; f < 16; ++f) {
            float sum = g_conv1_b[f];
            for (int k = 0; k < 5; ++k) {
                int in_t = t + k - 2;                 // same padding (kernel 5 -> 左右各 2)
                if (in_t >= 0 && in_t < 80) {
                    for (int c = 0; c < 12; ++c) {
                        sum += in[in_t][c] * g_conv1_w[(k * 12 + c) * 16 + f];
                    }
                }
            }
            c1[t][f] = (sum > 0.0f) ? sum : 0.0f;     // ReLU
        }
    }

    // ---- Layer 2: MaxPool1D (pool=2) -> (40,16) ----
    for (int t = 0; t < 40; ++t) {
        for (int f = 0; f < 16; ++f) {
            float v1 = c1[2 * t][f];
            float v2 = c1[2 * t + 1][f];
            p1[t][f] = (v1 > v2) ? v1 : v2;
        }
    }

    // ---- Layer 3: Conv1D (kernel=3, pad=same, filters=32, relu) ----
    for (int t = 0; t < 40; ++t) {
        for (int f = 0; f < 32; ++f) {
            float sum = g_conv2_b[f];
            for (int k = 0; k < 3; ++k) {
                int in_t = t + k - 1;                 // same padding (kernel 3)
                if (in_t >= 0 && in_t < 40) {
                    for (int c = 0; c < 16; ++c) {
                        sum += p1[in_t][c] * g_conv2_w[(k * 16 + c) * 32 + f];
                    }
                }
            }
            c2[t][f] = (sum > 0.0f) ? sum : 0.0f;
        }
    }

    // ---- Layer 4: MaxPool1D (pool=2) -> (20,32) ----
    for (int t = 0; t < 20; ++t) {
        for (int f = 0; f < 32; ++f) {
            float v1 = c2[2 * t][f];
            float v2 = c2[2 * t + 1][f];
            p2[t][f] = (v1 > v2) ? v1 : v2;
        }
    }

    // ---- Layer 5: Conv1D (kernel=3, pad=same, filters=32, relu) ----
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
            c3[t][f] = (sum > 0.0f) ? sum : 0.0f;
        }
    }

    // ---- Layer 6: GlobalAveragePooling1D -> (32) ----
    for (int f = 0; f < 32; ++f) {
        float sum = 0.0f;
        for (int t = 0; t < 20; ++t) sum += c3[t][f];
        gap[f] = sum / 20.0f;
    }

    // ---- Layer 7: Dense (units=16, relu) ----
    for (int i = 0; i < 16; ++i) {
        float sum = g_dense1_b[i];
        for (int j = 0; j < 32; ++j) sum += gap[j] * g_dense1_w[j * 16 + i];
        d1[i] = (sum > 0.0f) ? sum : 0.0f;
    }

    // ---- Layer 8: Dense (units=2, softmax) -> 取 index 1 = P(跌倒) ----
    float lo0 = g_dense2_b[0], lo1 = g_dense2_b[1];
    for (int j = 0; j < 16; ++j) {
        lo0 += d1[j] * g_dense2_w[j * 2 + 0];
        lo1 += d1[j] * g_dense2_w[j * 2 + 1];
    }
    float mx   = (lo0 > lo1) ? lo0 : lo1;          // 减最大值防 exp 溢出
    float e0   = expf(lo0 - mx);
    float e1   = expf(lo1 - mx);
    return e1 / (e0 + e1);
}

// ================= 对外接口 =================

void cnn_detector_push(const FallInput& in) {
    const SensorData& s = in.local;
    bool wristOk = in.hasRemote && in.remoteAgeMs < WRIST_FRESH_MS;

    float* h = s_hist[s_head];
    h[0] = norm_acc(s.ax);
    h[1] = norm_acc(s.ay);
    h[2] = norm_acc(s.az);
    h[3] = norm_gyr(s.gx);
    h[4] = norm_gyr(s.gy);
    h[5] = norm_gyr(s.gz);
    if (wristOk) {
        const SensorData& r = in.remote;
        h[6]  = norm_acc(r.ax);
        h[7]  = norm_acc(r.ay);
        h[8]  = norm_acc(r.az);
        h[9]  = norm_gyr(r.gx);
        h[10] = norm_gyr(r.gy);
        h[11] = norm_gyr(r.gz);
    } else {
        // 腕端离线：填 0 并标记无效。仲裁入口会用覆盖率把这类窗挡掉
        // （训练数据里腕端从不为 0，喂 0 属于域外输入，不可作为仲裁依据）
        for (int c = 6; c < 12; ++c) h[c] = 0.0f;
    }
    s_wristOk[s_head] = wristOk ? 1 : 0;

    s_head = (s_head + 1) % CNN_HIST_N;
    if (s_cnt < CNN_HIST_N) s_cnt++;
}

bool cnn_detector_prob(int ageSamples, float minCoverage,
                       float* outProb, float* outCoverage) {
    constexpr int WIN50 = 100;              // 仲裁窗 2s @50Hz
    constexpr int WIN40 = 80;               // 抽取后 2s @40Hz
    constexpr int HALF  = WIN50 / 2 - 1;    // 49：锚点前后各覆盖的样本数

    // 窗覆盖 50Hz 的 [age-49, age+49]：两端都必须落在已采数据内，
    // 否则会读到环形缓冲里"还没写到的将来位置"（脏数据）
    if (ageSamples < HALF) return false;             // 锚点离当前太近，窗尾越过最新样本
    int oldestAgo = ageSamples + HALF;
    if (oldestAgo >= s_cnt) return false;            // 历史还没填满（开机初期）

    static float win[WIN40][12];
    int okCnt = 0;
    for (int k = 0; k < WIN40; ++k) {
        int j   = k + k / 4;                          // 每 5 拍取 4 拍：50Hz -> 40Hz
        int ago = oldestAgo - j;                      // 恒 >= 0（上面已保证）
        int idx = (s_head - 1 - ago) % CNN_HIST_N;
        if (idx < 0) idx += CNN_HIST_N;

        memcpy(win[k], s_hist[idx], sizeof(float) * 12);
        if (s_wristOk[idx]) okCnt++;
    }

    float cov = (float)okCnt / (float)WIN40;
    if (outCoverage) *outCoverage = cov;
    if (cov < minCoverage) return false;             // 腕端不可信 -> 不仲裁

    if (outProb) *outProb = cnn_forward(win);
    return true;
}

void cnn_detector_reset(void) {
    s_head = 0;
    s_cnt  = 0;
    memset(s_hist, 0, sizeof(s_hist));
    memset(s_wristOk, 0, sizeof(s_wristOk));
}
