# -*- coding: utf-8 -*-
"""
SisFall 数据集加载器（跌倒检测算法 · 第①步）

职责（与后续脚本分工）：
    1. 解析原始 .txt 文件 -> 物理单位数组（加速度 m/s²、角速度 deg/s）
    2. 200Hz -> 50Hz 降采样（与腰端固件采样率一致，保证"训练域=部署域"）
    3. 计算基础派生量：SVM 合加速度、躯干倾角（低通重力方向）
    4. 提供全量索引扫描（给 02_features/03_train 用）

数据格式备忘（来自数据集 Readme.txt，写代码前必读）：
    - 每个文件一次动作，200Hz，9 列 = 3 个传感器各 3 轴：
        col0-2  ADXL345 加速度  ±16g   13bit   <- 我们用这个（量程最大，跌倒冲击不削顶）
        col3-5  ITG3200 陀螺仪  ±2000°/s 16bit  <- 我们用这个
        col6-8  MMA8451Q 加速度 ±8g    14bit   <- 弃用（与我们 MPU 同量程，留作对照也行）
    - 原始值是"位"，换算公式：物理值 = (2*量程/2^分辨率) * 原始值
        加速度:   raw / 256.0  -> g   （32/8192）
        角速度:   raw / 16.384 -> °/s （4000/65536）
    - 行格式坑：一行内多个采样用 ';' 分隔，采样内列用 ',' 分隔
    - 文件名: <动作码>_<受试者>_<次数>.txt，D01-D19=日常活动，F01-F15=跌倒
      SA=青年(19-30岁) SE=老年(60-75岁，只做日常活动，没摔）

单位约定：与腰端固件一致 —— 加速度 m/s²（静止约 9.81），角速度 deg/s。
"""

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import lfilter

# 数据集根目录（algo/data/ 已被 .gitignore 排除，组员需自行下载）
DATA_DIR = Path(__file__).resolve().parent / "data" / "sisfall" / "SisFall_dataset"
INDEX_CACHE = Path(__file__).resolve().parent / "data" / "sisfall_index.csv"

FS_RAW = 200       # 数据集原始采样率
FS_WORK = 50       # 工作采样率（= 腰端固件 SAMPLE_HZ）
DECIM = FS_RAW // FS_WORK

G = 9.81           # m/s²

# 传感器换算系数（见文件头备忘）
K_ACC = 32.0 / 8192.0      # ADXL345: bit -> g
K_GYR = 4000.0 / 65536.0   # ITG3200: bit -> deg/s

CLIP_G = 8.0       # 我们 MPU 的量程 ±8g：统计样本中有多少峰值会削顶（域差距量化）


# ---------------------------------------------------------------- 文件名解析
def parse_filename(fname: str) -> dict:
    """'F01_SA01_R04.txt' -> {act, subject, trial, is_fall, group}"""
    stem = Path(fname).stem
    act, subject, trial = stem.split("_")
    return {
        "act": act,                       # D01-D19 / F01-F15
        "subject": subject,               # SA01-SA23 青年 / SE01-SE15 老年
        "trial": trial,                   # R01-R05
        "is_fall": act.startswith("F"),
        "group": "SE" if subject.startswith("SE") else "SA",
    }


# ---------------------------------------------------------------- 单文件读取
def load_file(path, fs=FS_WORK) -> dict:
    """
    读取一个 SisFall 文件 -> 物理单位字典

    返回: ax,ay,az (m/s²), gx,gy,gz (deg/s), svm (m/s²), n, fs
    默认降采样到 50Hz（块平均，见 _decimate 注释）
    """
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # 行内多采样 ';' 分隔；兼容纯逗号 CSV 的情况
            samples = line.split(";") if ";" in line else [line]
            for s in samples:
                parts = s.split(",")
                if len(parts) < 6:
                    continue  # 尾部残缺片段，丢弃
                try:
                    rows.append([int(v) for v in parts[:6]])
                except ValueError:
                    continue  # 个别行有杂质字符，跳过该采样

    raw = np.asarray(rows, dtype=np.float64)
    acc = raw[:, 0:3] * K_ACC * G          # bit -> g -> m/s²
    gyr = raw[:, 3:6] * K_GYR              # bit -> deg/s

    if fs < FS_RAW:
        acc = _decimate(acc)
        gyr = _decimate(gyr)

    svm = np.sqrt((acc ** 2).sum(axis=1))  # 合加速度幅值（固件同名概念）

    return {
        "ax": acc[:, 0], "ay": acc[:, 1], "az": acc[:, 2],
        "gx": gyr[:, 0], "gy": gyr[:, 1], "gz": gyr[:, 2],
        "svm": svm, "n": len(svm), "fs": fs,
    }


def _decimate(x: np.ndarray) -> np.ndarray:
    """
    200Hz -> 50Hz 降采样：4 点块平均。

    为什么用块平均而不是隔 4 取 1：
      隔点抽取会把 >25Hz 的高频噪声折叠进有效频段（混叠）。
      块平均是一个简易抗混叠滤波，也近似模拟固件 DLPF(20Hz) 的效果，
      让训练数据的频率特性更接近板上真实读数。
    """
    n = (len(x) // DECIM) * DECIM
    return x[:n].reshape(-1, DECIM, x.shape[1]).mean(axis=1)


# ---------------------------------------------------------------- 派生特征
def tilt_angle(d: dict) -> np.ndarray:
    """
    躯干倾角（度）：低通取出重力方向后，与传感器 Z 轴的夹角。

    原理：加速度 = 重力 + 运动加速度。对加速度做低通（截止 ~0.8Hz）
    可近似滤掉运动分量，剩下的就是重力方向 —— 即"身体此刻斜不斜"。
    直立 ≈ 0°，躺倒 ≈ 90°。这是三阶段判决第二阶段的核心量，
    也是区分"快速坐下"与"跌倒"的关键（坐下后躯干仍接近直立？取决于佩戴
    位置——所以 02_features 里会用"窗口内最小倾角"等更稳的特征）。

    实现：scipy.lfilter 向量化计算一阶 IIR，与逐点循环
        y[i] = a*x[i] + (1-a)*y[i-1]
    数学上完全等价（部署时固件用逐点写法，两者结果一致），
    用 lfilter 是因为 4505 个文件逐点 Python 循环太慢。
    与固件对应：上板时用同结构的一阶 IIR，保证训练/部署滤波一致。
    """
    alpha = 0.03   # 一阶 IIR 系数 @50Hz，截止约 0.8Hz
    acc = np.stack([d["ax"], d["ay"], d["az"]], axis=1)
    b = [alpha]
    a = [1.0, -(1.0 - alpha)]
    # zi 初始化为首样本，等价于"首点直接初始化"，避免引入假角度
    zi = (acc[0] * (1.0 - alpha)).reshape(1, 3)
    lp, _ = lfilter(b, a, acc, axis=0, zi=zi)
    g_mag = np.sqrt((lp ** 2).sum(axis=1))
    g_mag[g_mag < 1e-6] = 1e-6
    cosv = np.clip(lp[:, 2] / g_mag, -1.0, 1.0)
    return np.degrees(np.arccos(cosv))


def clip_fraction(d: dict) -> float:
    """
    峰值超出 ±8g 的采样占比 —— 量化"数据集 ±16g vs 我们硬件 ±8g"的域差距。
    若跌倒样本普遍不超 8g，说明 ±8g 部署量程够用；否则特征设计需避开绝对峰值。
    """
    return float((d["svm"] > CLIP_G * G).mean())


# ---------------------------------------------------------------- 索引扫描
def build_index(force: bool = False) -> pd.DataFrame:
    """
    扫描全部 4505 个文件，生成摘要索引（后续特征/训练脚本的入口）。
    结果缓存为 CSV（algo/data/sisfall_index.csv，已被 gitignore），
    二次运行秒级加载。force=True 强制重扫。
    """
    if INDEX_CACHE.exists() and not force:
        return pd.read_csv(INDEX_CACHE)

    recs = []
    files = sorted(DATA_DIR.rglob("*.txt"))
    for i, p in enumerate(files):
        if p.name.lower().startswith("readme"):
            continue
        meta = parse_filename(p.name)
        d = load_file(p)
        tilt = tilt_angle(d)
        recs.append({
            **meta,
            "path": str(p.relative_to(DATA_DIR)),
            "n": d["n"],
            "dur_s": round(d["n"] / d["fs"], 1),
            "svm_mean": round(float(d["svm"].mean()), 2),
            "svm_peak": round(float(d["svm"].max()), 2),
            "clip_pct": round(clip_fraction(d) * 100, 3),
            "tilt_max": round(float(tilt.max()), 1),
        })
        if (i + 1) % 500 == 0:
            print(f"  ... {i+1}/{len(files)}")

    df = pd.DataFrame(recs)
    df.to_csv(INDEX_CACHE, index=False)
    print(f"index -> {INDEX_CACHE} ({len(df)} files)")
    return df


# ---------------------------------------------------------------- 自检
def selfcheck():
    """跑 3 个代表文件，人工核对物理量是否合理（分段调试原则）"""
    cases = [
        ("D01_SA01_R01.txt", "慢走（100s）:  SVM 应 ≈ 9.8±小幅波动，倾角小"),
        ("D08_SA01_R01.txt", "快坐半高凳（12s）: 有冲击、坐下后倾角中等 —— 最难负样本"),
        ("F01_SA01_R01.txt", "前向滑倒（15s）: 冲击大、跌后倾角接近 90° 且持续"),
    ]
    for fname, expect in cases:
        p = DATA_DIR / fname.split("_")[1] / fname
        d = load_file(p)
        tilt = tilt_angle(d)
        # 跌倒/坐下后的倾角取后半段（动作发生在文件中段，后半段最能反映"结果姿态"）
        half = d["n"] // 2
        print(f"\n== {fname}  ({expect})")
        print(f"   采样点 {d['n']} @ {d['fs']}Hz = {d['n']/d['fs']:.1f}s")
        print(f"   SVM   mean={d['svm'].mean():6.2f}  peak={d['svm'].max():6.2f} m/s²"
              f"   (静止应≈9.81)")
        print(f"   倾角  后半段 mean={tilt[half:].mean():5.1f}°  max={tilt[half:].max():5.1f}°"
              f"   (直立≈0° 躺倒≈90°)")
        print(f"   超±8g 占比 = {clip_fraction(d)*100:.2f}%   (域差距量化)")
        # 前几个原始采样换算抽查：静止段三轴合应 ≈ 1g
        head = np.sqrt(d["ax"][:25]**2 + d["ay"][:25]**2 + d["az"][:25]**2)
        print(f"   开头0.5s SVM均值 = {head.mean():.2f} m/s² (受试者静止待命，应≈9.81)")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "check":
        selfcheck()
    elif cmd == "index":
        build_index(force="--force" in sys.argv)
    else:
        print("用法: python sisfall.py [check | index [--force]]")
