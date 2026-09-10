# -*- coding: utf-8 -*-
"""
第③步：特征提取 —— 把每个数据文件变成一个"事件样本"特征向量

★ 核心设计决策：触发锚定窗口（trigger-anchored），不是全文件滑窗

    常见做法（网上很多仓库）：整个文件滑窗切分，跌倒文件的所有窗口都标 1。
    这是数据泄漏（rhmt80 仓库 MODEL_CARD 记录的同款坑）：
      跌倒文件里"摔之前的走路"也被标成跌倒 -> 模型学到错误概念。
    后果在离线指标上看不出来，上板后误报率暴涨。

    我们的做法：模拟部署时的真实工作流 ——
      1. 找 SVM 峰值时刻 t_pk（= 板上"冲击触发"事件）
      2. 特征从 t_pk 附近固定偏移的窗口提取（冲击前/冲击时刻/冲击后）
      3. ADL 文件同样处理（峰值小也照做）——模型学的是
         "冲击触发之后，什么样的后续模式是跌倒"
    训练分布 = 部署分布，这是小数据上泛化的最有效手段。

    另一个技巧：跌倒文件里 t_pk 前 4~6s 的"摔前走路"段，额外提取一个
    负样本（标签 0）。既不浪费数据，又提供了最难的负样本（紧邻真摔的
    日常动作），直接对冲上面说的错误标注问题。

窗口布局（秒，相对 t_pk）：
    dip 窗   [-1.5, -0.2)   自由落体段：跌倒前 SVM 会先跌到 ~0g（失重）
    pre 窗   [-1.0,  0.0)   冲击前的基线活动
    imp 窗   [-0.5, +0.5]   冲击瞬间
    post 窗  [+0.5, +3.0]   确认窗（= 固件里"等一下再看"的那一段）

特征表（全部可在固件里用几行 C 复算，这是选择特征的第一原则）：
    svm_peak_imp   imp 窗 SVM 峰值          —— 冲击强度
    svm_dip_pre    dip 窗 SVM 最小值        —— 失重前兆（跌倒独有，坐下没有）
    svm_mean_pre   pre 窗 SVM 均值          —— 触发前活动水平
    svm_std_post   post 窗 SVM 标准差       —— 冲击后活动量（"人还在动吗"）
    svm_mean_post  post 窗 SVM 均值
    dtilt_med_post post 窗 |ΔTilt| 中位数    —— 姿态：躯干倒没倒
    dtilt_max_post post 窗 |ΔTilt| 最大值
    gmag_peak_imp  imp 窗角速度模峰值        —— 翻转剧烈程度
    gmag_std_post  post 窗角速度模标准差      —— 冲击后是否还在翻身

输出: algo/data/features.csv（缓存，gitignore），供 03_train 使用。
v2 变更（对照 v1 备份 features_v1.csv 评审）：
    1. 域对齐裁剪 CLIP_ACC/CLIP_GYR —— 修复"数据集量程 > 硬件量程"的上板失效隐患
    2. 新增 svm_impulse_imp / dip_dur_pre / dtilt_chg（砸坐误报与坐姿跌倒专项）
用法: python 02_features.py check     # 4 个已知样本自检（分段调试）
      python 02_features.py all       # 全量提取（首次约 1~2 分钟）
"""

from pathlib import Path

import numpy as np
import pandas as pd

from sisfall import DATA_DIR, INDEX_CACHE, load_file, tilt_angle

FEAT_CACHE = Path(__file__).resolve().parent / "data" / "features.csv"

FS = 50  # 与 sisfall.load_file 默认输出一致

# ---- 窗口定义（秒，相对冲击时刻 t_pk）——与固件三阶段时序一一对应 ----
WIN_DIP = (-1.5, -0.2)
WIN_PRE = (-1.0, 0.0)
WIN_IMP = (-0.5, 0.5)
WIN_POST = (0.5, 3.0)
WIN_ENERGY = (-0.4, 0.2)  # v2: 冲击能量积分窗（略偏冲击前，覆盖下落减速全程）
NEG_SEG = (-6.0, -4.0)   # 跌倒文件的"摔前走路"负样本段

# ---- 硬件域对齐（v2 关键修正）----
# 我们的 MPU6500: 加速度 ±8g、陀螺仪 ±500°/s；数据集传感器: ±16g、±2000°/s。
# 跌倒的角速度常达 1000~2000°/s，在硬件上会饱和在 500 —— 若用原始值训练，
# 模型会学到"角速度>800 才算摔"这类硬件永远达不到的规则，上板即失效。
# 对策：特征计算前裁剪到硬件量程，训练分布 = 部署分布。
CLIP_ACC = 8.0 * 9.81     # SVM 上限（±8g，m/s²）
CLIP_GYR = 500.0          # 角速度模上限（±500°/s）

FEATURES = [
    "svm_peak_imp", "svm_dip_pre", "svm_mean_pre",
    "svm_std_post", "svm_mean_post",
    "dtilt_med_post", "dtilt_max_post",
    "gmag_peak_imp", "gmag_std_post",
    # ---- v2 新增（"砸坐"误报与坐姿跌倒漏报专项）----
    "svm_impulse_imp",   # 冲击能量：|SVM-g| 积分。摔地(落差大) >> 坐凳(落差小)
    "dip_dur_pre",       # 失重时长：dip 窗内 SVM<0.8g 占比。摔前自由落体更久
    "dtilt_chg",         # 事件前后姿态变化：|post中位 - pre中位|，朝向无关
]


def _seg(x, t0, t1):
    """按秒取窗口切片（自动夹到数据边界；空窗返回 None）"""
    a, b = max(int(t0 * FS), 0), min(int(t1 * FS), len(x))
    return x[a:b] if b - a >= FS // 4 else None    # 至少 0.25s 才算有效


def extract_one(d: dict, tilt: np.ndarray, i_pk: int) -> dict | None:
    """从锚点 i_pk 提取一个样本的特征。窗口不完整（数据太短）返回 None。

    注意：所有特征都在"裁剪到硬件量程后"的信号上计算（见 CLIP_ACC/CLIP_GYR），
    保证模型学到的规则在 MPU6500 上真实可达。
    """
    svm = np.minimum(d["svm"], CLIP_ACC)                       # 域对齐：±8g
    gmag = np.minimum(np.sqrt(d["gx"]**2 + d["gy"]**2 + d["gz"]**2),
                      CLIP_GYR)                                # 域对齐：±500°/s
    dt = np.abs(tilt - np.median(tilt[: min(2 * FS, len(tilt))]))  # ΔTilt 相对前2s基线

    t0 = i_pk / FS
    imp_s = _seg(svm, t0 + WIN_IMP[0], t0 + WIN_IMP[1])
    dip_s = _seg(svm, t0 + WIN_DIP[0], t0 + WIN_DIP[1])
    pre_s = _seg(svm, t0 + WIN_PRE[0], t0 + WIN_PRE[1])
    post_s = _seg(svm, t0 + WIN_POST[0], t0 + WIN_POST[1])
    post_dt = _seg(dt, t0 + WIN_POST[0], t0 + WIN_POST[1])
    pre_dt = _seg(dt, t0 + WIN_PRE[0], t0 + WIN_PRE[1])
    imp_g = _seg(gmag, t0 + WIN_IMP[0], t0 + WIN_IMP[1])
    post_g = _seg(gmag, t0 + WIN_POST[0], t0 + WIN_POST[1])
    ene_s = _seg(svm, t0 + WIN_ENERGY[0], t0 + WIN_ENERGY[1])

    if any(v is None for v in (imp_s, post_s, post_dt, imp_g, pre_dt)):
        return None    # 冲击太靠文件头尾，窗口不完整 -> 弃样（数量极少）

    return {
        "svm_peak_imp": float(imp_s.max()),
        "svm_dip_pre": float(dip_s.min()) if dip_s is not None else float(svm.mean()),
        "svm_mean_pre": float(pre_s.mean()) if pre_s is not None else float(svm.mean()),
        "svm_std_post": float(post_s.std()),
        "svm_mean_post": float(post_s.mean()),
        "dtilt_med_post": float(np.median(post_dt)),
        "dtilt_max_post": float(post_dt.max()),
        "gmag_peak_imp": float(imp_g.max()),
        "gmag_std_post": float(post_g.std()) if post_g is not None else 0.0,
        # ---- v2 新增 ----
        # 冲击能量(m/s)：|SVM-g| 在能量窗的均值 —— 速度变化量的代理，
        # 摔地落差 ≈ 坐凳落差的两倍以上，能量按落差线性放大（v=√(2gh)）
        "svm_impulse_imp": float(np.abs(ene_s - 9.81).mean()),
        # 失重时长(0~1)：dip 窗内低于 0.8g 的占比 —— 自由落体持续的相对时长
        "dip_dur_pre": float((dip_s < 0.8 * 9.81).mean()) if dip_s is not None else 0.0,
        # 姿态净变化(°)：事件前/后中位 |ΔTilt| 之差 —— 与绝对朝向无关的纯变化量
        "dtilt_chg": float(abs(np.median(post_dt) - np.median(pre_dt))),
    }


def sample_from_file(path: Path, meta: dict) -> list[dict]:
    """
    一个文件 -> 样本列表（1~2 个）
      跌倒文件: [主样本(标签1), 摔前走路负样本(标签0)]
      ADL 文件: [主样本(标签0)]
    """
    d = load_file(path)
    if d["n"] < 5 * FS:            # 短于 5s 的残缺文件直接跳过
        return []
    tilt = tilt_angle(d)

    i_pk = int(np.argmax(d["svm"]))
    f = extract_one(d, tilt, i_pk)
    out = []
    if f is not None:
        out.append({**meta, **f, "label": int(meta["is_fall"]), "anchor": "peak"})

    # 跌倒文件补充"摔前走路"负样本（i_pk 前 4~6s 段内再找局部锚点）
    if meta["is_fall"]:
        lo = max(i_pk + int(NEG_SEG[0] * FS), 0)
        hi = min(i_pk + int(NEG_SEG[1] * FS), d["n"])
        if hi - lo >= FS:                      # 段内至少 1s 数据才有效
            j = lo + int(np.argmax(d["svm"][lo:hi]))
            g = extract_one(d, tilt, j)
            if g is not None:
                out.append({**meta, **g, "label": 0, "anchor": "prefall"})
    return out


def build_all(force: bool = False) -> pd.DataFrame:
    """全量提取 + 缓存。依赖 01 步生成的索引缓存快速遍历文件清单。"""
    if FEAT_CACHE.exists() and not force:
        return pd.read_csv(FEAT_CACHE)

    from sisfall import parse_filename
    files = sorted(p for p in DATA_DIR.rglob("*.txt")
                   if not p.name.lower().startswith("readme"))
    recs = []
    for i, p in enumerate(files):
        meta = parse_filename(p.name)
        meta["file"] = p.name
        recs.extend(sample_from_file(p, meta))
        if (i + 1) % 500 == 0:
            print(f"  ... {i+1}/{len(files)}")

    df = pd.DataFrame(recs)
    df.to_csv(FEAT_CACHE, index=False)
    print(f"features -> {FEAT_CACHE}  (rows={len(df)}, "
          f"pos={int(df.label.sum())}, neg={int((df.label == 0).sum())})")
    return df


def check():
    """自检：4 个已知文件的特征值，与 01_explore 的结论人工对照"""
    cases = ["D01_SA01_R01.txt", "D19_SA01_R01.txt",
             "D08_SA01_R01.txt", "F01_SA01_R01.txt"]
    from sisfall import parse_filename
    rows = []
    for fname in cases:
        p = DATA_DIR / fname.split("_")[1] / fname
        meta = parse_filename(fname)
        for s in sample_from_file(p, meta):
            rows.append({"file": fname, "anchor": s["anchor"],
                         **{k: round(s[k], 1) for k in FEATURES}})
    df = pd.DataFrame(rows)
    pd.set_option("display.width", 200)
    print(df.to_string(index=False))
    print("""
对照 01_explore 结论自检：
  F01 主样本: svm_peak_imp≈89, dtilt_med_post≈56   (冲击+姿态双高)
  F01 prefall: 各特征应与 D01 走路行接近           (摔前走路=负样本)
  D08: svm_peak_imp≈35 但 dtilt_med_post 应 <10    (坐下的判别证据)
  D19: svm_std_post / gmag_std_post 应明显大于 F01  (跳完继续动)
""")


if __name__ == "__main__":
    import sys
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "check":
        check()
    elif cmd == "all":
        build_all(force="--force" in sys.argv)
    else:
        print("用法: python 02_features.py [check | all [--force]]")
