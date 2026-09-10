# -*- coding: utf-8 -*-
"""
第②步：可视化对比 —— "为什么需要三阶段判决"

从 4 类代表动作各选 1 例，双行子图对比：
    上行 SVM（冲击维度）  下行 Δtilt 倾角变化（姿态维度）
目的：直观看清
    - 只看冲击：快坐/跳/绊 的峰值和跌倒同量级 -> 必然误报
    - 加上姿态变化：只有真跌倒 Δtilt 大且持续
    - 加上冲击后活动量：跳/绊 之后 SVM 恢复波动，跌倒之后归于平静
输出: algo/out/explore.png + 终端摘要表

图内文字用英文：matplotlib 默认字体不含中文，避免配置字体栈的麻烦。
"""

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")          # 无显示环境也能出图（CI/服务器友好）
import matplotlib.pyplot as plt

from sisfall import DATA_DIR, load_file, tilt_angle, G

OUT_DIR = Path(__file__).resolve().parent / "out"
OUT_DIR.mkdir(exist_ok=True)

# 4 类代表动作（case 说明写英文，直接可用于答辩 PPT）
CASES = [
    ("D01_SA01_R01.txt", "Walk slowly (ADL)"),
    ("D19_SA01_R01.txt", "Gentle jump (hard ADL)"),      # 跳：有冲击，但人马上继续动
    ("D08_SA01_R01.txt", "Quick sit, half-height (hard ADL)"),  # 快坐：冲击≈跌倒，但姿态没变
    ("F01_SA01_R01.txt", "Forward slip fall (FALL)"),    # 滑倒：冲击大 + 姿态倒 + 之后不动
]

IMPACT_TH = 25.0   # 参考冲击阈值 2.5g m/s²（仅画图参考线，正式标定在 04_threshold）


def analyze(fname: str) -> dict:
    """加载 + 计算 Δtilt（相对文件前 2s 站立基线的倾角变化）+ 冲击后活动量"""
    p = DATA_DIR / fname.split("_")[1] / fname
    d = load_file(p)
    tilt = tilt_angle(d)
    base_n = min(int(2 * d["fs"]), d["n"])            # 前 2 秒站立基线
    base = np.median(tilt[:base_n])
    dtilt = tilt - base                                # Δtilt：正=倒向一侧，绝对值=偏离直立
    t = np.arange(d["n"]) / d["fs"]

    # 冲击后 1~3s 的 SVM 标准差（"人还在动吗"）：取冲击峰值时刻往后看
    i_pk = int(np.argmax(d["svm"]))
    a, b = min(i_pk + d["fs"], d["n"]), min(i_pk + 3 * d["fs"], d["n"])
    post_std = float(d["svm"][a:b].std()) if b > a else float("nan")

    return dict(d=d, t=t, dtilt=dtilt, i_pk=i_pk,
                svm_pk=float(d["svm"].max()),
                dtilt_rest=float(np.median(dtilt[d["n"] // 2:])),
                post_std=post_std)


def main():
    fig, axes = plt.subplots(len(CASES), 2, figsize=(13, 3.0 * len(CASES)),
                             sharex="col")
    fig.suptitle("Impact (SVM) vs Posture (dTilt) — why 3-phase logic is needed",
                 fontsize=13, y=0.995)

    rows = []
    for r, (fname, label) in enumerate(CASES):
        r_ = analyze(fname)
        t, d, dt = r_["t"], r_["d"], r_["dtilt"]

        ax1, ax2 = axes[r][0], axes[r][1]
        ax1.plot(t, d["svm"], lw=0.8, color="tab:blue")
        ax1.axhline(IMPACT_TH, color="r", ls="--", lw=1, label=f"impact th {IMPACT_TH/G:.1f}g")
        ax1.axvline(t[r_["i_pk"]], color="gray", ls=":", lw=1)
        ax1.set_ylabel(f"SVM m/s2\n({label.split(' (')[0]})", fontsize=8)
        ax1.set_title(label if r == 0 else "", fontsize=9)
        ax1.legend(fontsize=7, loc="upper right")

        ax2.plot(t, np.abs(dt), lw=0.9, color="tab:green", label="|dTilt| deg")
        ax2.axvline(t[r_["i_pk"]], color="gray", ls=":", lw=1)
        ax2.set_ylabel("|dTilt| deg", fontsize=8)
        ax2.set_xlabel("time (s)", fontsize=8)
        ax2.legend(fontsize=7, loc="upper left")

        rows.append((label, r_["svm_pk"] / 9.81, abs(r_["dtilt_rest"]), r_["post_std"]))

    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = OUT_DIR / "explore.png"
    fig.savefig(out, dpi=130)
    print(f"figure -> {out}\n")

    # 终端摘要表：三个维度并列，三阶段判决的证据表
    print(f"{'case':<38}{'SVM peak(g)':>12}{'rest |dTilt|(deg)':>20}{'post 1-3s SVM std':>20}")
    print("-" * 90)
    for label, pk, rest, pstd in rows:
        print(f"{label:<38}{pk:>12.1f}{rest:>20.1f}{pstd:>20.1f}")
    print("""
读表方法（答辩讲稿）:
  - Quick sit 的冲击(3.5g)接近跌倒 -> 只用冲击必误报
  - 跳/坐的 rest |dTilt| 很小, 跌倒 >40deg -> 姿态是关键判别维度
  - 跌倒后 post SVM std 低(静止), 日常动作后恢复波动 -> 活动量兜底
""")


if __name__ == "__main__":
    main()
