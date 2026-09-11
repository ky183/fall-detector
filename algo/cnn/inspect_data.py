# -*- coding: utf-8 -*-
"""
FallAllD pkl 数据体检 —— 训练前的数据可信度检查

背景：处理版 pkl（FallAllD_40SamplesPerSec_ActivityIdsFiltered）来自 Kaggle
公开数据集 "Derived FallAllD Dataset"，其数据卡片声明做过 SMOTE 合成采样
与活动过滤。SMOTE 合成样本是特征空间近邻插值，若存在于本 pkl：
  - 跨受试者插值会污染"按人划分"（变相数据泄漏）
  - 插值会把跌倒冲击尖峰磨平（时序失真）
体检结论直接决定：用处理版直接训练，还是用原始版自己重做预处理。

检查项：
  1. 结构：列、设备分布、受试者/活动构成
  2. SMOTE 痕迹：trial 等长性（切窗版痕迹）、(受试者,活动,次数) 键重复、索引异常
  3. 标度核对：静止段 |acc| 应 ≈4096 LSB（=1g）、角速度 ≈0
     —— 与 cnn_detector.cpp 的 ACC_LSB_PER_G=4096 / GYR_LSB_PER_DPS=65.536 对齐
  4. 腕端陀螺通道来源：恒 0（填充）或与腰部逐点高相关（复制）都是伪通道

用法：python algo/cnn/inspect_data.py <pkl路径>
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ACC_LSB_PER_G = 4096.0     # 与 cnn_detector.cpp / cnn_train.py 同一口径
GYR_LSB_PER_DPS = 65.536


def main(path):
    print(f"读取 {path} ...")
    df = pd.read_pickle(path)
    print(f"\n== 1. 结构 ==")
    print(f"总行数 {len(df)}，列: {list(df.columns)}")
    print(f"Device 分布: {df['Device'].value_counts().to_dict()}")
    subjects = sorted(df['SubjectID'].unique(), key=str)
    print(f"受试者 {len(subjects)} 人: {subjects}")
    acts = sorted(df['ActivityID'].unique())
    falls = [a for a in acts if int(a) >= 100]
    print(f"活动 ID 范围 {acts[0]}..{acts[-1]}，跌倒类 {len(falls)} 种，"
          f"ADL 类 {len(acts) - len(falls)} 种")

    print(f"\n== 2. SMOTE / 切窗痕迹 ==")
    lens = df["Acc"].apply(len)
    print(f"trial 时序长度：min={lens.min()} max={lens.max()} "
          f"unique={lens.nunique()}（unique 越少越像固定切窗版；"
          f"原始不等长 trial 应有几十种长度）")
    key = df.groupby(["SubjectID", "ActivityID", "TrialNo"]).size()
    print(f"(受试者,活动,次数) 组合 {len(key)} 个，每组合行数分布: "
          f"{key.value_counts().to_dict()}（每 key 应恰好 1 行/设备；"
          f"出现 >2 说明有重复行）")
    idx = df.index
    contig = (idx.to_series().diff().dropna() == 1).mean()
    print(f"行索引连续比例 {contig:.3f}（SMOTE 追加行常造成索引断裂/浮点）")

    print(f"\n== 3. 标度核对（假设 acc 4096 LSB/g, gyr 65.536 LSB/dps）==")
    rng = np.random.default_rng(0)
    sample = df[df["Device"] == "Waist"].sample(min(200, len(df)), random_state=1)
    mags, gmag = [], []
    for _, r in sample.iterrows():
        a = np.asarray(r["Acc"]); g = np.asarray(r["Gyr"])
        m = int(len(a) * 0.6)
        seg = a[m:m + 40] if len(a) - m >= 40 else a[m:]
        if len(seg) == 0:
            continue
        mags.append(np.median(np.linalg.norm(seg, axis=1)))
        gmag.append(np.median(np.linalg.norm(g[m:m + len(seg)], axis=1)))
    mags = np.array(mags) / ACC_LSB_PER_G          # -> g 域
    gmag = np.array(gmag)
    print(f"腰端 trial 中段 |acc| 中位数（g 域）分位数: "
          f"10%={np.quantile(mags, .1):.2f} 50%={np.quantile(mags, .5):.2f} "
          f"90%={np.quantile(mags, .9):.2f}")
    print(f"  -> 静止/低动态活动应贴近 1.0；若整体 ≈4096 倍/≈0.002 说明已是 g 域")
    print(f"腰端 |gyr| 中位数（dps）分位数: "
          f"10%={np.quantile(gmag, .1):.1f} 50%={np.quantile(gmag, .5):.1f}")

    print(f"\n== 4. 腕端陀螺通道来源 ==")
    pairs = df.groupby(["SubjectID", "ActivityID", "TrialNo"])
    cors, zeros = [], 0
    checked = 0
    for (sid, aid, tno), g in pairs:
        if checked >= 40:
            break
        wd = g[g["Device"] == "Waist"]; rd = g[g["Device"] == "Wrist"]
        if len(wd) == 0 or len(rd) == 0:
            continue
        wg = np.asarray(wd["Gyr"].values[0])[:, 0]
        rg = np.asarray(rd["Gyr"].values[0])[:, 0]
        n = min(len(wg), len(rg))
        if n < 50:
            continue
        wg, rg = wg[:n], rg[:n]
        if np.allclose(rg, 0):
            zeros += 1
        elif np.std(wg) > 1e-6 and np.std(rg) > 1e-6:
            cors.append(abs(np.corrcoef(wg, rg)[0, 1]))
        checked += 1
    print(f"抽查 {checked} 对腰/腕 trial：腕陀螺全 0 的 {zeros} 对；"
          f"其余 X 轴 |相关系数| 中位数 "
          f"{np.median(cors):.3f}（>0.95 高度怀疑复制；正常应为中等相关）")

    print("\n== 结论判读 ==")
    print("干净（可直接训练）的条件：长度多样化、无重复键、索引连续、"
          "|acc|中位数分布合理（低分位≈1g）、腕陀螺非全0且非复制。")
    print("任一条件不满足 -> 建议用原始版 FallAllD.pkl 自行重做预处理"
          "（40Hz 降采样 + 腰腕过滤），全程可控。")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("用法: python inspect_data.py <pkl路径>")
    if not Path(sys.argv[1]).exists():
        sys.exit(f"文件不存在: {sys.argv[1]}")
    main(sys.argv[1])
