# -*- coding: utf-8 -*-
"""
CNN 二级仲裁 —— 训练 + 导出 C 权重

定位（★ 重要）：
    产物 cnn_weights.h 是**第二级**仲裁，不是主判决。
    主判决 = firmware/waist_firmware/fall_detector.cpp 的三阶段触发 + 随机森林
             （训练见 algo/01..05，权重 rf_model.h）
    本模型只在"腰端已触发冲击 + 判定时腕端在线"时被调用一次，
    用 RF 用不到的腕端 6 通道信息做补判/否决。

与随机森林管线（algo/03_train.py）的方法论差距 —— 已知问题，重训时再修：
    1. 这里按**窗口**随机划分训练/测试：同一个人的同一次 trial 会同时出现
       在两边，且步长 40 / 窗长 80 使相邻窗口有 50% 重叠 -> 指标虚高，
       不能与 03_train.py 的"按人留出"结果直接对比。
    2. 无老年组压力测试；腕端通道也没做 dropout 增广
       （腕端离线时输入全 0，属域外输入）。
    3. 未做双语言一致性回放（05_export.py 有，是移植正确性的硬验收）。
    在补齐之前，本模型只作为二级仲裁，不改主判决。

数据：algo/data/FallAllD_40SamplesPerSec_ActivityIdsFiltered.pkl
      FallAllD（15 人，40Hz，腰/腕/颈；此处只用腰+腕）
      ActivityID >= 100 记为跌倒，其余记为日常动作
      原始标度：acc 4096 LSB/g，gyr 65.536 LSB/dps（=32768/500，±500 dps 量程）

导出：firmware/waist_firmware/cnn_weights.h（自动生成，勿手改）
      板端实现 firmware/waist_firmware/cnn_detector.cpp
"""

from pathlib import Path

import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras import layers, models

# 固定随机种子：权重初始化与 shuffle 都要可复现，否则导出的 .h 无法复现
SEED = 42
np.random.seed(SEED)
tf.random.set_seed(SEED)

# 路径约定与 algo/05_export.py 一致：数据在 algo/data/，产物直写固件目录
ALGO     = Path(__file__).resolve().parent   # algo/cnn
ALGO_DIR = ALGO.parent                       # algo
FW_DIR   = ALGO_DIR.parent / "firmware" / "waist_firmware"
PKL      = ALGO_DIR / "data" / "FallAllD_40SamplesPerSec_ActivityIdsFiltered.pkl"

print("1. 正在加载数据...")
df = pd.read_pickle(PKL)

# 只保留腰部和手腕传感器
df = df[df["Device"].isin(["Waist", "Wrist"])].copy()
df["Acc"] = df["Acc"].apply(lambda x: np.array(x, dtype=np.float32))
df["Gyr"] = df["Gyr"].apply(lambda x: np.array(x, dtype=np.float32))

print("2. 正在对齐 Waist(腰) 和 Wrist(腕) 数据...")
trials = df.groupby(["SubjectID", "ActivityID", "TrialNo"])

window_size = 80  # 40Hz * 2s = 80 点
step_size = 40  # 50% 重叠

X_windows = []
y_windows = []

for (subject_id, act_id, trial_no), group in trials:
    waist_data = group[group["Device"] == "Waist"]
    wrist_data = group[group["Device"] == "Wrist"]

    if len(waist_data) == 0 or len(wrist_data) == 0:
        continue

    w_acc = waist_data["Acc"].values[0]
    w_gyr = waist_data["Gyr"].values[0]
    r_acc = wrist_data["Acc"].values[0]
    r_gyr = wrist_data["Gyr"].values[0]

    min_len = min(len(w_acc), len(w_gyr), len(r_acc), len(r_gyr))
    if min_len < window_size:
        continue

    # 归一化量纲
    w_acc = w_acc[:min_len] / 4096.0  # 归一化为 g
    w_gyr = w_gyr[:min_len] / 2000.0  # 归一化为 dps 比例
    r_acc = r_acc[:min_len] / 4096.0
    r_gyr = r_gyr[:min_len] / 2000.0

    features = np.hstack([w_acc, w_gyr, r_acc, r_gyr])
    label = 1 if act_id >= 100 else 0

    for i in range(0, min_len - window_size, step_size):
        X_windows.append(features[i : i + window_size])
        y_windows.append(label)

X_windows = np.array(X_windows, dtype=np.float32)
y_windows = np.array(y_windows, dtype=np.int32)

print(f"-> 数据集提取完成！窗口数: {X_windows.shape[0]}")

# 3. 用纯 Numpy 随机划分 80% 训练集, 20% 测试集 (不再依赖 sklearn)
np.random.seed(SEED)
indices = np.arange(len(X_windows))
np.random.shuffle(indices)

split_idx = int(len(indices) * 0.8)
train_indices, test_indices = indices[:split_idx], indices[split_idx:]

X_train, X_test = X_windows[train_indices], X_windows[test_indices]
y_train, y_test = y_windows[train_indices], y_windows[test_indices]

y_train_cat = tf.keras.utils.to_categorical(y_train, num_classes=2)
y_test_cat = tf.keras.utils.to_categorical(y_test, num_classes=2)

print("3. 构建轻量 1D-CNN 模型...")
model = models.Sequential(
    [
        layers.Input(shape=(80, 12)),
        layers.Conv1D(16, kernel_size=5, padding="same", activation="relu"),
        layers.MaxPooling1D(pool_size=2),
        layers.Conv1D(32, kernel_size=3, padding="same", activation="relu"),
        layers.MaxPooling1D(pool_size=2),
        layers.Conv1D(32, kernel_size=3, padding="same", activation="relu"),
        layers.GlobalAveragePooling1D(),
        layers.Dense(16, activation="relu"),
        layers.Dropout(0.3),
        layers.Dense(2, activation="softmax"),
    ]
)

model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])

print("4. 开始快速训练 (30 轮)...")
model.fit(
    X_train,
    y_train_cat,
    epochs=30,
    batch_size=64,
    validation_data=(X_test, y_test_cat),
    verbose=1,
)

# 保存一份以备后用（归档在 algo/out/，与 rf_model.json 同处）
OUT_DIR = ALGO_DIR / "out"
OUT_DIR.mkdir(exist_ok=True)
model.save(OUT_DIR / "cnn_model.h5")
print(f"-> 模型已保存至 {OUT_DIR / 'cnn_model.h5'}")

# 5. 直接在内存中提取权重并生成 cnn_weights.h
print("5. 正在生成纯 C++ 权重头文件 (cnn_weights.h)...")


def array_to_c_string(arr, name):
    flattened = arr.flatten()
    elements = ", ".join([f"{x:.7f}f" for x in flattened])
    formatted = ""
    for i, el in enumerate(elements.split(", ")):
        formatted += el + ", "
        if (i + 1) % 8 == 0:
            formatted += "\n    "
    return f"const float {name}[{len(flattened)}] = {{\n    {formatted.rstrip(', ')}\n}};\n\n"


header_content = """// ============================================================
//  cnn_weights.h — 1D-CNN 二级仲裁权重（自动生成，勿手改）
//  生成: algo/cnn/cnn_train.py | FallAllD 数据集（15 人, 40Hz, 腰+腕）
//  网络: Conv1D(16,k5) -> MaxPool2 -> Conv1D(32,k3) -> MaxPool2
//        -> Conv1D(32,k3) -> GAP -> Dense(16) -> Dense(2,softmax)
//  输入: (80,12) @40Hz = 2s
//        通道序 [0..2]腰acc [3..5]腰gyr [6..8]腕acc [9..11]腕gyr
//  权重排布（Keras Conv1D kernel = (kernel, in_ch, filters) 行主序）:
//        g_conv1_w[(k*12+c)*16+f]   g_conv2_w[(k*16+c)*32+f]
//        g_conv3_w[(k*32+c)*32+f]   g_dense1_w[j*16+i]  g_dense2_w[j*2+i]
//  归一化（★ 板端必须按同一口径复现，见 cnn_detector.cpp 文件头）:
//        acc = 数据集原始 LSB / 4096   (= g 值)
//        gyr = 数据集原始 LSB / 2000
//  用法: 见 cnn_detector.cpp；本权重不参与常态判决，
//        只在"冲击触发后 + 腕端在线"时被调用一次。
// ============================================================
#ifndef CNN_WEIGHTS_H_
#define CNN_WEIGHTS_H_

"""

# 对应 model 各层提取参数
conv1_w, conv1_b = model.layers[0].get_weights()
conv2_w, conv2_b = model.layers[2].get_weights()
conv3_w, conv3_b = model.layers[4].get_weights()
dense1_w, dense1_b = model.layers[6].get_weights()
dense2_w, dense2_b = model.layers[8].get_weights()

header_content += array_to_c_string(conv1_w, "g_conv1_w")
header_content += array_to_c_string(conv1_b, "g_conv1_b")
header_content += array_to_c_string(conv2_w, "g_conv2_w")
header_content += array_to_c_string(conv2_b, "g_conv2_b")
header_content += array_to_c_string(conv3_w, "g_conv3_w")
header_content += array_to_c_string(conv3_b, "g_conv3_b")
header_content += array_to_c_string(dense1_w, "g_dense1_w")
header_content += array_to_c_string(dense1_b, "g_dense1_b")
header_content += array_to_c_string(dense2_w, "g_dense2_w")
header_content += array_to_c_string(dense2_b, "g_dense2_b")

header_content += "#endif // CNN_WEIGHTS_H_\n"

OUT = FW_DIR / "cnn_weights.h"
# ★ encoding 必须显式指定 utf-8：Windows 下默认 cp936 会写出 GBK 字节，
#   与仓库其余文件（UTF-8）不一致
OUT.write_text(header_content, encoding="utf-8")

print("=" * 45)
# 不用 emoji：Windows 控制台默认 GBK，print 非 GBK 字符会抛
# UnicodeEncodeError（原来是 🎉，训练跑完最后一行才崩）
print("[OK] 大功告成！已生成 cnn_weights.h")
print(f"   -> {OUT}")
print("   固件侧由 cnn_detector.cpp 直接 #include，无需手动拷贝")
print("=" * 45)
