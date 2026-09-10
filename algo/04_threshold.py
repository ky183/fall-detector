# -*- coding: utf-8 -*-
"""
第⑤步：三阶段阈值法基线 + 四种方法对比 + 上板参数导出

★ 方法论（防泄漏的最后关卡）：
    阈值参数只允许在训练集（18 个青年）上网格搜索标定，
    冻结后在留出集（5 个全新人）+ 老年组上各评估一次。
    若在留出集上挑参数 = 对着考卷改答案（rhmt80 仓库记录的同款坑），
    指标虚高，上板露馅。

对比的四种方法：
    A. 阈值法    三阶段 AND 逻辑（固件最易移植、完全可解释）
    B. 随机森林  300 树（03_train 已训好的 rf.joblib）
    C. 小随机森林 15 树×深度6（C++ 数组化移植的可行性探针）
    D. 混合门控  冲击门限粗筛(T1/T2) 通过后交给 RF(0.3) 精判
                  —— 即固件真实工作流：粗筛跑在 50Hz 循环里，ML 只判疑似段

输出:
    对比表（留出集 + 老年组）
    algo/data/threshold_params.json   阈值法最终参数（第⑥步 C 移植直接引用）
    algo/out/train_report.txt 追加阈值章节
"""

import json
from pathlib import Path

import numpy as np
import pandas as pd
import joblib

ALGO = Path(__file__).resolve().parent
DATA = ALGO / "data"
OUT_DIR = ALGO / "out"
OUT_DIR.mkdir(exist_ok=True)

TEST_SUBJECTS = ["SA04", "SA09", "SA14", "SA19", "SA23"]   # 与 03_train 完全一致
SEED = 42
RF_DEPLOY_TH = 0.3   # 03_train 门限 sweep 的结论：灵敏度优先

import importlib.util as _ilu
_spec = _ilu.spec_from_file_location("feat", ALGO / "02_features.py")
_feat = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_feat)
FEATURES = _feat.FEATURES

# ---- 阈值网格（粗粒度、物理可解释；在训练集上搜索） ----
# T1 冲击峰值(m/s²) T2 角速度峰值(°/s) T3 姿态变化(°) T4 冲击后活动量(m/s²)
GRID = {
    "T1": [15, 20, 25, 30],        # 1.5g ~ 3g
    "T2": [100, 200, 300, 400],    # 硬件量程内(±500)
    "T3": [20, 30, 40],            # 躯干偏离直立的角度
    "T4": [2, 4, 6],               # 静止判据
}


def thresh_pred(df: pd.DataFrame, p: dict) -> np.ndarray:
    """三阶段 AND：冲击(T1 或 T2 任一) + 姿态(T3) + 静止(T4)"""
    impact = (df.svm_peak_imp > p["T1"]) | (df.gmag_peak_imp > p["T2"])
    posture = df.dtilt_med_post > p["T3"]
    still = df.svm_std_post < p["T4"]
    return (impact & posture & still).astype(int).values


def sens_spec(y: np.ndarray, pred: np.ndarray) -> tuple:
    tp = int(((y == 1) & (pred == 1)).sum())
    fn = int(((y == 1) & (pred == 0)).sum())
    tn = int(((y == 0) & (pred == 0)).sum())
    fp = int(((y == 0) & (pred == 1)).sum())
    sens = tp / (tp + fn) if tp + fn else float("nan")
    spec = tn / (tn + fp) if tn + fp else float("nan")
    return sens, spec, fp


def main():
    df = pd.read_csv(DATA / "features.csv")
    train = df[(df.group == "SA") & (~df.subject.isin(TEST_SUBJECTS))]
    test = df[df.subject.isin(TEST_SUBJECTS)]
    elderly = df[df.group == "SE"]
    se_adl, se_falls = elderly[elderly.label == 0], elderly[elderly.label == 1]

    lines, print_ = [], print

    def say(s=""):
        print_(s)
        lines.append(s)

    # ========== 1) 阈值参数：训练集网格搜索 ==========
    y_tr = train.label.values
    best, best_score = None, -1
    for T1 in GRID["T1"]:
        for T2 in GRID["T2"]:
            for T3 in GRID["T3"]:
                for T4 in GRID["T4"]:
                    p = dict(T1=T1, T2=T2, T3=T3, T4=T4)
                    s, sp, _ = sens_spec(y_tr, thresh_pred(train, p))
                    # 目标：特异度≥97% 约束下最大化 (灵敏度+特异度)/2
                    #（SisFall 论文的标准指标），约束防止"全报警"类作弊解
                    if sp >= 0.97 and (s + sp) / 2 > best_score:
                        best_score, best = (s + sp) / 2, p

    say("== 阈值参数（仅用训练集标定，已冻结） ==")
    say(f"   T1 冲击峰值    > {best['T1']} m/s²  ({best['T1']/9.81:.1f}g)")
    say(f"   T2 角速度峰值  > {best['T2']} °/s")
    say(f"   T3 姿态变化    > {best['T3']} °")
    say(f"   T4 冲击后活动  < {best['T4']} m/s²")
    say(f"   训练集得分 (SE+SP)/2 = {best_score*100:.1f}%")
    json.dump(best, open(DATA / "threshold_params.json", "w"), indent=2)

    # ========== 2) 四种方法在相同评估集上的对比 ==========
    rf = joblib.load(DATA / "rf.joblib")
    rf_small = __import__("sklearn.ensemble", fromlist=["RandomForestClassifier"])\
        .RandomForestClassifier(n_estimators=15, max_depth=6,
                                random_state=SEED, n_jobs=-1)
    rf_small.fit(train[FEATURES], train.label)

    def eval_all(name, pred_fn):
        """pred_fn(df)->0/1 数组；统一评估三套数据"""
        s_te, sp_te, fp_te = sens_spec(test.label.values, pred_fn(test))
        s_se6, _, _ = sens_spec(se_falls.label.values, pred_fn(se_falls))
        _, _, fp_adl = sens_spec(se_adl.label.values, pred_fn(se_adl))
        rows.append((name, s_te * 100, sp_te * 100, fp_te,
                     fp_adl / len(se_adl) * 100, s_se6 * 100))

    rows = []
    eval_all("A 阈值法", lambda d: thresh_pred(d, best))
    eval_all("B 随机森林300", lambda d: (rf.predict_proba(d[FEATURES])[:, 1] >= RF_DEPLOY_TH).astype(int))
    eval_all("C 小随机森林15x6", lambda d: (rf_small.predict_proba(d[FEATURES])[:, 1] >= RF_DEPLOY_TH).astype(int))
    # 混合门控：粗筛=冲击阶段(T1/T2)，通过者交给 RF300 精判
    eval_all("D 混合门控", lambda d: (((d.svm_peak_imp > best["T1"]) | (d.gmag_peak_imp > best["T2"])) &
                                       (rf.predict_proba(d[FEATURES])[:, 1] >= RF_DEPLOY_TH)).astype(int))

    say("\n== 四种方法对比（留出集=5 个全新青年受试者） ==")
    say(f"   {'方法':<14}{'灵敏%':>7}{'特异%':>7}{'误报':>5}{'老年FP%':>9}{'SE06检%':>9}")
    for r in rows:
        say(f"   {r[0]:<14}{r[1]:>7.1f}{r[2]:>7.1f}{r[3]:>5}{r[4]:>9.1f}{r[5]:>9.1f}")
    say("""
决策依据（第⑥步 C++ 移植选型）：
    - 若 C 小森林 ≈ B 大森林 -> 用 C（数组化移植体积可控）
    - 若 C 明显掉点且 D 占优 -> 上板走 D（阈值粗筛 + 精简 ML）
    - 阈值法 A 是保底与参照系，也是"两代方法对比"报告章节的数据来源
""")

    # 追加写入总报告
    with open(OUT_DIR / "train_report.txt", "a", encoding="utf-8") as f:
        f.write("\n\n========== 04_threshold 追加 ==========\n" + "\n".join(lines) + "\n")
    print_(f"params -> {DATA/'threshold_params.json'}")
    print_(f"report -> {OUT_DIR/'train_report.txt'}")


if __name__ == "__main__":
    main()
