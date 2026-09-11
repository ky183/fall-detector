# -*- coding: utf-8 -*-
"""
CNN 二级仲裁 —— 训练 + 导出 C 权重（v2）

定位（★ 重要，未变）：
    产物 cnn_weights.h 是**第二级**仲裁，不是主判决。
    主判决 = firmware/waist_firmware/fall_detector.cpp 的三阶段触发 + 随机森林
             （训练见 algo/01..05，权重 rf_model.h）
    本模型只在"腰端已触发冲击 + 判定时腕端在线"时被调用一次，
    用 RF 用不到的腕端 6 通道信息做补判/否决。
    当前固件 config.h 处于**影子模式**（CNN_ROLE_RESCUE=0）：
    CNN 只计算记录，不改变判决。

v2 相对 v1 的修复（评估口径 + 部署域对齐）：
    1. 按受试者划分训练/测试（v1 是随机窗口划分：50% 重叠的相邻窗口同时
       出现在两侧 -> 指标虚高。修复后测试集全部是训练没见过的人）。
    2. 触发锚定标签（v1 把整个 trial 的所有窗口都标 1，包括摔前站立和
       摔后躺卧 -> 教了"站立=跌倒"的错误语义）。v2 以腰部 SVM 峰值为锚点：
       正样本 = 峰值中心的 2s 窗（与板端 cnn_detector_prob 的取窗方式一致）；
       摔前/摔后段补负样本（同 algo/02_features.py 的 NEG_SEG 思路）。
    3. 部署域增广（对齐两个真实存在的域差）：
       a. 腕端 dropout：整窗置 0（链路离线）/ 随机拍置 0（丢包）。
          训练数据腕端从不缺失，而板端 cov<1 的窗里有 0 值，属域外输入。
       b. 随机旋转：腰/腕各自独立的随机 SO(3) 旋转，同一端 acc 与 gyr 用
          同一旋转矩阵（刚体旋转下两矢量同样变换，物理自洽、保手性）。
          动机：腕端 PCB 佩戴方向不可控 + 数据集轴向约定无从核对，
          旋转增广迫使模型只学"与佩戴朝向无关"的特征，一次解决两个域差。
    4. ablation：旋转增广开/关各训一版做对比 —— 规矩是不让性能无谓变差，
       用数据决定取舍（默认导出旋转版，除非对比明显更差）。

仍遗留（下次重训再修，或待外部信息）：
    - 老年组压力测试：取决于 pkl 中受试者年龄构成，待向数据提供方确认
    - 腕端陀螺通道来源未核实（若为复制/填充的伪通道，应考虑丢弃）
    - "RF 漏掉子集"上的专项评估（RESCUE 的直接依据，需 RF 侧数据联动）

数据：algo/data/FallAllD_40SamplesPerSec_ActivityIdsFiltered.pkl
      FallAllD（15 人，40Hz，腰/腕/颈；此处只用腰+腕）
      ActivityID >= 100 记为跌倒，其余记为日常动作
      原始标度：acc 4096 LSB/g，gyr 65.536 LSB/dps（=32768/500，±500 dps 量程）
      ★ 本机无 pkl 时可用 --smoke 用合成数据验证流程（不产指标）

用法：
    python algo/cnn/cnn_train.py            # 真数据 + 双版 ablation
    python algo/cnn/cnn_train.py --smoke    # 合成数据，只验流程跑通
    python algo/cnn/cnn_train.py --no-rot   # 只训"无旋转增广"单版

导出：firmware/waist_firmware/cnn_weights.h（自动生成，勿手改）
      板端实现 firmware/waist_firmware/cnn_detector.cpp
      ★ 归一化口径与 v1 完全一致（acc LSB/4096、gyr LSB/2000），
        板端 norm_acc/norm_gyr 无需任何改动。
"""

import argparse
import sys
from pathlib import Path

import numpy as np

# Windows 控制台默认 GBK，print 中文/非 GBK 字符会抛 UnicodeEncodeError
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# 固定随机种子：权重初始化与增广抽样都要可复现，否则导出的 .h 无法复现
SEED = 42

# 路径约定与 algo/05_export.py 一致：数据在 algo/data/，产物直写固件目录
ALGO     = Path(__file__).resolve().parent   # algo/cnn
ALGO_DIR = ALGO.parent                       # algo
FW_DIR   = ALGO_DIR.parent / "firmware" / "waist_firmware"
PKL      = ALGO_DIR / "data" / "FallAllD_40SamplesPerSec_ActivityIdsFiltered.pkl"

# ---------------- 窗口与标签（秒，40Hz；与板端仲裁窗同构） ----------------
FS        = 40
WINDOW    = 80          # 2s @40Hz，与 cnn_detector_prob 的 WIN40 一致
HALF_WIN  = 40
JITTER    = 10          # 正窗中心随机抖动 ±0.25s（仅训练集扩充副本用）
NEG_SEG   = (-6.0, -4.0)   # 摔前负样本段（相对峰值，同 02_features.py）
POST_SEG  = (4.0, 6.0)     # 摔后负样本段（静躺 ≠ 正在跌倒）
MAX_NEG_PER_ADL = 3     # 每 ADL trial 负窗上限（控制类不平衡）

# ---------------- 划分与增广（只作用于训练集） ----------------
TEST_STRIDE   = 5       # 每 5 人取 1 人留出（15 人 -> 3 人测试）
P_ROT         = 0.5     # 每端独立随机旋转的概率
P_WRIST_DROP  = 0.25    # 腕 6 通道整窗置 0（模拟链路离线）
P_WRIST_MASK  = 0.25    # 腕 6 通道随机 30% 拍置 0（模拟丢包）
MASK_FRAC     = 0.30

# ---------------- 归一化（与板端 cnn_detector.cpp 严格一致，勿改） ----------------
ACC_LSB_PER_G  = 4096.0
ACC_DIV        = 4096.0
GYR_LSB_PER_DPS = 65.536
GYR_DIV        = 2000.0

EPOCHS = 50


# ================================================================ 数据构建
def _norm_window(wa, wg, ra, rg):
    """原始 LSB -> 训练/部署同款归一化。输入均为 (N,3) 数组。"""
    out = np.empty((len(wa), 12), dtype=np.float32)
    out[:, 0:3] = (wa / ACC_LSB_PER_G) * (ACC_LSB_PER_G / ACC_DIV)
    out[:, 3:6] = wg * (GYR_LSB_PER_DPS / GYR_DIV)
    out[:, 6:9] = (ra / ACC_LSB_PER_G) * (ACC_LSB_PER_G / ACC_DIV)
    out[:, 9:12] = rg * (GYR_LSB_PER_DPS / GYR_DIV)
    return out


def build_windows(df, rng):
    """触发锚定切窗。返回 windows(N,80,12)、labels、subjects、is_jitter。"""
    X, y, subj, is_jit = [], [], [], []

    for (sid, aid, tno), g in df.groupby(["SubjectID", "ActivityID", "TrialNo"]):
        wd = g[g["Device"] == "Waist"]
        rd = g[g["Device"] == "Wrist"]
        if len(wd) == 0 or len(rd) == 0:
            continue
        wa, wg = np.asarray(wd["Acc"].values[0]), np.asarray(wd["Gyr"].values[0])
        ra, rg = np.asarray(rd["Acc"].values[0]), np.asarray(rd["Gyr"].values[0])
        n = min(len(wa), len(wg), len(ra), len(rg))
        if n < WINDOW:
            continue

        def take(s):
            s = max(0, min(s, n - WINDOW))
            X.append(_norm_window(wa[s:s + WINDOW], wg[s:s + WINDOW],
                                  ra[s:s + WINDOW], rg[s:s + WINDOW]))
            y.append(0)
            subj.append(sid)
            is_jit.append(False)

        svm = np.linalg.norm(wa[:n] / ACC_LSB_PER_G, axis=1)   # 腰端 g 域幅值

        if int(aid) >= 100:                                    # 跌倒 trial
            i_pk = int(np.argmax(svm))
            c = max(HALF_WIN, min(i_pk, n - HALF_WIN))
            # 正窗：峰值居中（与板端取窗一致）
            X.append(_norm_window(wa[c - HALF_WIN:c + HALF_WIN],
                                  wg[c - HALF_WIN:c + HALF_WIN],
                                  ra[c - HALF_WIN:c + HALF_WIN],
                                  rg[c - HALF_WIN:c + HALF_WIN]))
            y.append(1); subj.append(sid); is_jit.append(False)
            # 训练集扩充副本：中心抖动（评估不参与，见 split）
            cj = int(np.clip(c + rng.integers(-JITTER, JITTER + 1),
                             HALF_WIN, n - HALF_WIN))
            X.append(_norm_window(wa[cj - HALF_WIN:cj + HALF_WIN],
                                  wg[cj - HALF_WIN:cj + HALF_WIN],
                                  ra[cj - HALF_WIN:cj + HALF_WIN],
                                  rg[cj - HALF_WIN:cj + HALF_WIN]))
            y.append(1); subj.append(sid); is_jit.append(True)
            # 摔前 / 摔后负样本段
            for t0, t1 in (NEG_SEG, POST_SEG):
                lo, hi = i_pk + int(t0 * FS), i_pk + int(t1 * FS)
                lo, hi = max(0, lo), min(n, hi)
                if hi - lo >= WINDOW:
                    take(lo)
        else:                                                  # ADL trial
            starts = list(range(0, n - WINDOW + 1, WINDOW))     # 非重叠
            if starts:
                rng.shuffle(starts)
                for s in starts[:MAX_NEG_PER_ADL]:
                    take(s)

    return (np.array(X, dtype=np.float32), np.array(y, dtype=np.int32),
            np.array(subj), np.array(is_jit))


# ================================================================ 增广
def _random_rotation(rng):
    """随机 SO(3) 旋转矩阵（Rodrigues 公式）。det=+1，保手性。"""
    v = rng.normal(size=3)
    v /= np.linalg.norm(v)
    th = rng.uniform(0.0, 2.0 * np.pi)
    x, y_, z = v
    K = np.array([[0.0, -z, y_], [z, 0.0, -x], [-y_, x, 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def augment_batch(X, rng, with_rot=True):
    """对 (N,80,12) 逐窗做部署域增广，返回增广副本（不改原数据）。
    腰(0:6)与腕(6:12)各自独立旋转；同一端 acc/gyr 用同一矩阵。"""
    out = X.copy()
    for i in range(len(out)):
        w = out[i]
        if with_rot and rng.random() < P_ROT:
            R = _random_rotation(rng)
            w[:, 0:3] = w[:, 0:3] @ R
            w[:, 3:6] = w[:, 3:6] @ R
        if with_rot and rng.random() < P_ROT:
            R = _random_rotation(rng)
            w[:, 6:9] = w[:, 6:9] @ R
            w[:, 9:12] = w[:, 9:12] @ R
        r = rng.random()
        if r < P_WRIST_DROP:
            w[:, 6:12] = 0.0
        elif r < P_WRIST_DROP + P_WRIST_MASK:
            w[rng.random(WINDOW) < MASK_FRAC, 6:12] = 0.0
    return out


# ================================================================ 训练 / 评估
def build_model():
    import tensorflow as tf
    from tensorflow.keras import layers, models
    model = models.Sequential(
        [
            layers.Input(shape=(WINDOW, 12)),
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
    model.compile(optimizer="adam", loss="categorical_crossentropy",
                  metrics=["accuracy"])
    return model


def train_one(tag, Xtr, ytr, Xte, yte):
    import tensorflow as tf
    tf.random.set_seed(SEED)
    model = build_model()
    # 用逐样本 sample_weight 而非 class_weight：后者在 Keras 3 下与
    # one-hot 标签的组合存在兼容坑，sample_weight 语义等价且无歧义
    n_pos, n_neg = int(ytr.sum()), int((ytr == 0).sum())
    sw = np.ones(len(ytr), dtype=np.float32)
    if 0 < n_pos < n_neg:
        sw[ytr == 1] = min(5.0, n_neg / n_pos)   # 类权重，限幅防过矫正
    model.fit(Xtr, tf.keras.utils.to_categorical(ytr, 2),
              epochs=EPOCHS, batch_size=64, verbose=0, sample_weight=sw)
    print(f"   [{tag}] 训练完成（正 {n_pos} / 负 {n_neg}，"
          f"w_pos={float(sw[ytr == 1][0]) if n_pos else 1.0:.2f}）")
    return model


def predict_p(model, X):
    return model.predict(X, verbose=0)[:, 1]


def sweep_table(model, Xte, yte):
    rows = []
    p = predict_p(model, Xte)
    for thr in (0.5, 0.6, 0.7, 0.8, 0.9):
        tp = int(((p >= thr) & (yte == 1)).sum())
        fn = int(((p < thr) & (yte == 1)).sum())
        fp = int(((p >= thr) & (yte == 0)).sum())
        tn = int(((p < thr) & (yte == 0)).sum())
        sens = tp / max(1, tp + fn)
        spec = tn / max(1, tn + fp)
        rows.append((thr, tp, fn, fp, tn, sens, spec))
    return rows


def per_subject(model, Xte, yte, subj_te):
    out = []
    p = predict_p(model, Xte)
    for sid in sorted(set(subj_te), key=str):
        m = subj_te == sid
        tp = int(((p[m] >= 0.8) & (yte[m] == 1)).sum())
        fn = int(((p[m] < 0.8) & (yte[m] == 1)).sum())
        fp = int(((p[m] >= 0.8) & (yte[m] == 0)).sum())
        tn = int(((p[m] < 0.8) & (yte[m] == 0)).sum())
        out.append((str(sid), tp, fn, fp, tn,
                    tp / max(1, tp + fn), tn / max(1, tn + fp)))
    return out


def masked_eval(model, Xte, yte, rng):
    """腕端 30% 拍置 0 后再评（部署链路丢包的鲁棒性证据）。"""
    Xm = Xte.copy()
    mask = rng.random(Xm.shape[0:2]) < MASK_FRAC
    Xm[mask, 6:12] = 0.0
    p = predict_p(model, Xm)
    tp = int(((p >= 0.8) & (yte == 1)).sum()); fn = int(((p < 0.8) & (yte == 1)).sum())
    fp = int(((p >= 0.8) & (yte == 0)).sum()); tn = int(((p < 0.8) & (yte == 0)).sum())
    return tp / max(1, tp + fn), tn / max(1, tn + fp)


# ================================================================ 导出
def export_header(model, meta):
    def array_to_c_string(arr, name):
        flattened = arr.flatten()
        elements = ", ".join([f"{x:.7f}f" for x in flattened])
        formatted = ""
        for i, el in enumerate(elements.split(", ")):
            formatted += el + ", "
            if (i + 1) % 8 == 0:
                formatted += "\n    "
        return (f"const float {name}[{len(flattened)}] = {{\n    "
                f"{formatted.rstrip(', ')}\n}};\n\n")

    header = f"""// ============================================================
//  cnn_weights.h — 1D-CNN 二级仲裁权重（自动生成，勿手改）
//  生成: algo/cnn/cnn_train.py (v2) | FallAllD 数据集（15 人, 40Hz, 腰+腕）
//  模型: {meta}
//  网络: Conv1D(16,k5) -> MaxPool2 -> Conv1D(32,k3) -> MaxPool2
//        -> Conv1D(32,k3) -> GAP -> Dense(16) -> Dense(2,softmax)
//  输入: (80,12) @40Hz = 2s
//        通道序 [0..2]腰acc [3..5]腰gyr [6..8]腕acc [9..11]腕gyr
//  训练口径(v2): 按受试者划分 + 峰值锚定标签 + 部署域增广
//        (腕端 dropout/mask + 腰腕独立随机旋转 -> 对佩戴朝向鲁棒)
//  权重排布（Keras Conv1D kernel = (kernel, in_ch, filters) 行主序）:
//        g_conv1_w[(k*12+c)*16+f]   g_conv2_w[(k*16+c)*32+f]
//        g_conv3_w[(k*32+c)*32+f]   g_dense1_w[j*16+i]  g_dense2_w[j*2+i]
//  归一化（★ 板端必须按同一口径复现，见 cnn_detector.cpp 文件头）:
//        acc = 数据集原始 LSB / 4096   (= g 值)
//        gyr = 数据集原始 LSB / 2000
//  用法: 见 cnn_detector.cpp；当前固件为影子模式（CNN 不改变判决），
//        只在"冲击触发后 + 腕端在线"时被调用一次。
// ============================================================
#ifndef CNN_WEIGHTS_H_
#define CNN_WEIGHTS_H_

"""
    for layer, stem in ((0, "conv1"), (2, "conv2"), (4, "conv3"),
                        (6, "dense1"), (8, "dense2")):
        w, b = model.layers[layer].get_weights()
        header += array_to_c_string(w, f"g_{stem}_w")
        header += array_to_c_string(b, f"g_{stem}_b")
    header += "#endif // CNN_WEIGHTS_H_\n"

    out = FW_DIR / "cnn_weights.h"
    # ★ encoding 必须显式指定 utf-8：Windows 下默认 cp936 会写出 GBK 字节
    out.write_text(header, encoding="utf-8")
    print(f"   -> 已导出 {out}")
    return out


# ================================================================ 冒烟数据
def make_smoke_df(rng):
    """合成迷你数据集（结构与真 pkl 同构），只验流程，不产指标。"""
    import pandas as pd
    rows = []
    for sid in [f"S{i:02d}" for i in range(1, 6)]:
        for k in range(10):
            fall = (k % 2 == 0)
            aid = 101 + (k % 3) if fall else 10 + (k % 5)
            n = FS * 12
            t = np.arange(n)
            acc = np.zeros((n, 3)); gyr = np.zeros((n, 3))
            acc[:, 2] = 1.0                                  # 站立 1g
            if fall:
                i0 = n // 2
                acc[i0:i0 + 5] += 4.0                        # 冲击尖峰
                acc[i0 + 160:, :] = 0.0
                acc[i0 + 160:, 1] = 1.0                      # 躺平
                gyr[i0:i0 + 30, 0] = 3.0
                racc = acc * 0.5 + 0.05 * rng.normal(size=(n, 3))
                rgyr = gyr * 0.6
            else:
                acc[:, 0] += 0.2 * np.sin(2 * np.pi * 1.0 * t / FS)
                racc = 0.1 * rng.normal(size=(n, 3)); racc[:, 2] += 1.0
                rgyr = 0.2 * rng.normal(size=(n, 3))
            rows.append(dict(SubjectID=sid, ActivityID=aid, TrialNo=k + 1,
                             Device="Waist",
                             Acc=(acc * ACC_LSB_PER_G),
                             Gyr=(gyr * GYR_LSB_PER_DPS)))
            rows.append(dict(SubjectID=sid, ActivityID=aid, TrialNo=k + 1,
                             Device="Wrist",
                             Acc=(racc * ACC_LSB_PER_G),
                             Gyr=(rgyr * GYR_LSB_PER_DPS)))
    return pd.DataFrame(rows)


# ================================================================ 主流程
def report_lines(title, model, Xte, yte, subj_te, rng):
    lines = [f"---- {title} ----"]
    lines.append(f"{'thr':>4} {'TP':>4} {'FN':>4} {'FP':>4} {'TN':>4} "
                 f"{'sens':>7} {'spec':>7}")
    for thr, tp, fn, fp, tn, se, sp in sweep_table(model, Xte, yte):
        lines.append(f"{thr:>4.1f} {tp:>4} {fn:>4} {fp:>4} {tn:>4} "
                     f"{se:>7.3f} {sp:>7.3f}")
    lines.append("逐人（thr=0.8）:")
    for sid, tp, fn, fp, tn, se, sp in per_subject(model, Xte, yte, subj_te):
        lines.append(f"  {sid}: sens={se:.3f} spec={sp:.3f} "
                     f"(TP{tp} FN{fn} FP{fp} TN{tn})")
    se, sp = masked_eval(model, Xte, yte, rng)
    lines.append(f"腕端30%丢包鲁棒性(thr=0.8): sens={se:.3f} spec={sp:.3f}")
    return lines


def main():
    global EPOCHS
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true",
                    help="无真数据时用合成数据验证流程（不导出权重）")
    ap.add_argument("--no-rot", action="store_true",
                    help="跳过旋转增广 ablation，只训单版")
    ap.add_argument("--epochs", type=int, default=EPOCHS)
    args = ap.parse_args()
    EPOCHS = args.epochs

    rng = np.random.default_rng(SEED)

    if args.smoke:
        print("1. [SMOKE] 生成合成数据（结构与真 pkl 同构）...")
        df = make_smoke_df(rng)
    else:
        if not PKL.exists():
            sys.exit(f"找不到 {PKL}\n"
                     "（数据集不入库。拿到 pkl 后放入该路径，或先用 --smoke 验证流程）")
        import pandas as pd
        print("1. 正在加载 FallAllD pkl ...")
        df = pd.read_pickle(PKL)
        df = df[df["Device"].isin(["Waist", "Wrist"])].copy()
        df["Acc"] = df["Acc"].apply(lambda x: np.asarray(x))
        df["Gyr"] = df["Gyr"].apply(lambda x: np.asarray(x))

    print("2. 触发锚定切窗 ...")
    X, y, subj, is_jit = build_windows(df, rng)
    print(f"-> 共 {len(X)} 窗（正 {int(y.sum())} / 负 {int((y == 0).sum())}）")

    # 按受试者划分：jitter 扩充副本只在训练侧出现（评估独立性）
    subjects = sorted(set(subj), key=str)
    test_set = set(subjects[::TEST_STRIDE])
    print(f"-> 留出测试受试者: {sorted(map(str, test_set))}")
    te_m = np.isin(subj, list(test_set)) & ~is_jit
    tr_m = ~np.isin(subj, list(test_set))
    Xte, yte, subj_te = X[te_m], y[te_m], subj[te_m]
    Xtr, ytr = X[tr_m], y[tr_m]
    print(f"-> 训练 {len(Xtr)} 窗 / 测试 {len(Xte)} 窗"
          f"（测试正 {int(yte.sum())} / 负 {int((yte == 0).sum())}）")
    if len(Xte) == 0 or yte.sum() == 0 or (yte == 0).sum() == 0:
        if not args.smoke:
            sys.exit("测试集为空或只有单类，检查划分与数据构成")

    print("3. 训练（含部署域增广）...")
    Xtr_aug = np.concatenate([Xtr, augment_batch(Xtr, rng, with_rot=True)])
    ytr_aug = np.concatenate([ytr, ytr])
    model_rot = train_one("rot", Xtr_aug, ytr_aug, Xte, yte)

    model_plain = None
    if not args.no_rot:
        Xtr_aug0 = np.concatenate([Xtr, augment_batch(Xtr, rng, with_rot=False)])
        ytr_aug0 = np.concatenate([ytr, ytr])
        model_plain = train_one("no-rot", Xtr_aug0, ytr_aug0, Xte, yte)

    print("4. 评估 ...")
    lines = ["=" * 64,
             f"CNN 二级仲裁 v2 训练报告（smoke={args.smoke}, seed={SEED}, "
             f"epochs={EPOCHS}）",
             f"划分: 按受试者留出 {sorted(map(str, test_set))}；"
             f"标签: 峰值锚定；增广: rot={P_ROT} drop={P_WRIST_DROP} "
             f"mask={P_WRIST_MASK}", "=" * 64, ""]
    lines += report_lines("模型 A：随机旋转增广（默认导出版）",
                          model_rot, Xte, yte, subj_te, rng)
    lines.append("")
    if model_plain is not None:
        lines += report_lines("模型 B：无旋转增广（ablation 对照）",
                              model_plain, Xte, yte, subj_te, rng)
        lines.append("")
        lines.append("判读：A 对 B 若 sens/spec 均无明显下降，则旋转增广"
                     "换来佩戴朝向鲁棒性是纯收益；若明显下降，重新评估"
                     "旋转策略（如限幅小角度）后再定导出版。")

    text = "\n".join(lines)
    print(text)
    if args.smoke:
        # ★ 冒烟不落盘任何产物：报告与 h5 只在真数据训练时生成，
        #   防止误覆盖上一次真训练的结果
        print("[SMOKE] 流程跑通。合成数据不导出权重、指标无参考价值。")
        return 0

    OUT_DIR = ALGO_DIR / "out"
    OUT_DIR.mkdir(exist_ok=True)
    (OUT_DIR / "cnn_train_report.txt").write_text(text + "\n", encoding="utf-8")
    model_rot.save(OUT_DIR / "cnn_model.h5")

    print("5. 导出 cnn_weights.h ...")
    export_header(model_rot, "v2 旋转增广版（报告见 algo/out/cnn_train_report.txt）")
    print("=" * 45)
    print("[OK] 完成。烧录前请重跑: python tools/check_consistency.py 和"
          " python algo/cnn/host_replay/replay.py")
    print("=" * 45)
    return 0


if __name__ == "__main__":
    sys.exit(main())
