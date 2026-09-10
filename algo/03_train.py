# -*- coding: utf-8 -*-
"""
第④步：随机森林训练 + 严格评估

★ 三个方法论决策（答辩重点，也是指标可信的根基）：

1. 按"人"划分，不按样本随机划分
     测试集是 5 个完整受试者（SA04/09/14/19/23，固定列表，可复现），
     训练集完全没见过这些人。
     原因：同一个人的动作风格相似，若他的样本同时出现在训练/测试集，
     指标会虚高 10 个点以上（"记住这个人"而非"学会跌倒"）。
     上板面对的是全新的佩戴者，人不相交的划分才模拟真实泛化。

2. 老年组（SE，只做日常活动）不进训练集，专门做"泛化压力测试"
     训练只见青年 -> 拿老人的日常动作测误报率。
     SisFall 论文结论：这会让性能显著下降。我们的指标如果也降，
     是诚实的发现（答辩讲分析），不是失败。

3. 报告灵敏度(Sensitivity)优先于准确率(accuracy)
     产品逻辑：漏报(老人摔了没报)的代价 >> 误报(白响一次可取消)。
     类不平衡下 accuracy 无意义（全预测"没摔"也有 70% 准确率）。

输出:
    终端打印完整报告 + algo/out/train_report.txt（留档）
    algo/data/rf.joblib（模型，给 05 混合门控复用）
"""

from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import GroupKFold
import joblib

ALGO = Path(__file__).resolve().parent
OUT_DIR = ALGO / "out"
OUT_DIR.mkdir(exist_ok=True)

# 固定测试受试者（人为指定并固定下来：可复现、可审查、覆盖不同体型性别）
TEST_SUBJECTS = ["SA04", "SA09", "SA14", "SA19", "SA23"]
SEED = 42

# 特征列从 02_features.py 导入（模块名以数字开头不能直接 import，用 importlib）
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location("feat", ALGO / "02_features.py")
_feat = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_feat)
FEATURES = _feat.FEATURES


def metrics(y_true, proba, th=0.5):
    """灵敏度/特异度/精确率 —— 跌倒检测的标准三指标"""
    y_pred = (proba >= th).astype(int)
    tp = int(((y_true == 1) & (y_pred == 1)).sum())
    fn = int(((y_true == 1) & (y_pred == 0)).sum())
    tn = int(((y_true == 0) & (y_pred == 0)).sum())
    fp = int(((y_true == 0) & (y_pred == 1)).sum())
    sens = tp / (tp + fn) if tp + fn else float("nan")
    spec = tn / (tn + fp) if tn + fp else float("nan")
    prec = tp / (tp + fp) if tp + fp else float("nan")
    return dict(tp=tp, fn=fn, tn=tn, fp=fp, sens=sens, spec=spec, prec=prec)


def main():
    df = pd.read_csv(ALGO / "data" / "features.csv")

    # ---- 数据划分（详见文件头决策 1、2）----
    train = df[(df.group == "SA") & (~df.subject.isin(TEST_SUBJECTS))]
    test = df[df.subject.isin(TEST_SUBJECTS)]          # 青年留出：跌倒+日常都有
    elderly = df[df.group == "SE"]                     # 老年：全负样本，压力测试用
    print(f"样本划分: 训练 {len(train)} (青年{train.subject.nunique()}人) | "
          f"测试 {len(test)} (留出{len(TEST_SUBJECTS)}人) | "
          f"老年压力 {len(elderly)} ({elderly.subject.nunique()}人)\n")

    model = RandomForestClassifier(
        n_estimators=300,      # 树数：小数据集上 300 足够稳定
        random_state=SEED,     # 固定随机种子：结果可复现（工程规范）
        n_jobs=-1,
    )
    model.fit(train[FEATURES], train.label)

    lines = []

    def say(s=""):
        print(s)
        lines.append(s)

    # ---- 1) 留出集（全新受试者）核心指标 ----
    p_te = model.predict_proba(test[FEATURES])[:, 1]
    m = metrics(test.label.values, p_te)
    say("== 留出集（5 个全新青年受试者） ==")
    say(f"   灵敏度 Sensitivity = {m['sens']*100:5.1f}%  ({m['tp']}/{m['tp']+m['fn']} 跌倒被检出)")
    say(f"   特异度 Specificity = {m['spec']*100:5.1f}%  (误报 {m['fp']}/{m['tn']+m['fp']})")
    say(f"   精确率 Precision   = {m['prec']*100:5.1f}%")

    # ---- 2) 逐动作误报分析：哪些日常动作被误判 ----
    fp = test[(test.label == 0) & (p_te >= 0.5)]
    say("\n== 误报的动作明细（top） ==")
    if len(fp):
        for act, n in fp.act.value_counts().head(8).items():
            total = int(((test.label == 0) & (test.act == act)).sum())
            say(f"   {act}: {n}/{total} 次误判")
    else:
        say("   无误报")

    # ---- 3) 漏报分析：哪些跌倒没检出 ----
    fn = test[(test.label == 1) & (p_te < 0.5)]
    say("\n== 漏报的跌倒明细 ==")
    if len(fn):
        say(fn[["act", "subject", "trial"]].to_string(index=False))
    else:
        say("   无漏报")

    # ---- 4) 特征重要度（物理直觉审查点）----
    say("\n== 特征重要度（应与物理直觉一致） ==")
    imp = sorted(zip(FEATURES, model.feature_importances_),
                 key=lambda x: -x[1])
    for name, v in imp:
        bar = "#" * int(v * 100)
        say(f"   {name:<16} {v:.3f} {bar}")

    # ---- 5) 老年组泛化压力测试 ----
    # 注意口径：SE06（柔道专家）是唯一做跌倒动作的老人，他的跌倒是真阳性，
    # 不能算误报 —— 日常误报与 SE06 检出率分开统计
    se_adl = elderly[elderly.label == 0]
    se_falls = elderly[elderly.label == 1]
    p_adl = model.predict_proba(se_adl[FEATURES])[:, 1]
    fp_se = int((p_adl >= 0.5).sum())
    say("\n== 老年组压力测试（训练只见青年，测老人日常动作） ==")
    say(f"   老年日常误报 {fp_se}/{len(se_adl)} = {fp_se/len(se_adl)*100:.1f}%"
        f"（SisFall 论文预期：明显差于青年，属正常现象）")
    if fp_se:
        for act, n in se_adl[(p_adl >= 0.5)].act.value_counts().head(5).items():
            say(f"   {act}: {n} 次")
    if len(se_falls):
        p_sef = model.predict_proba(se_falls[FEATURES])[:, 1]
        det = int((p_sef >= 0.5).sum())
        say(f"   SE06(柔道专家)真实跌倒检出 {det}/{len(se_falls)}"
            f" —— 老人跌倒能否检出，比误报更关键")

    # ---- 6) 交叉验证稳定性（GroupKFold 按人分折）----
    say("\n== 5 折按人交叉验证（训练集内部，看稳定性） ==")
    gkf = GroupKFold(n_splits=5)
    ses, sps = [], []
    for tr_i, va_i in gkf.split(train[FEATURES], train.label, groups=train.subject):
        cv = RandomForestClassifier(n_estimators=300, random_state=SEED, n_jobs=-1)
        cv.fit(train[FEATURES].iloc[tr_i], train.label.iloc[tr_i])
        pv = cv.predict_proba(train[FEATURES].iloc[va_i])[:, 1]
        mm = metrics(train.label.iloc[va_i].values, pv)
        ses.append(mm["sens"])
        sps.append(mm["spec"])
    say(f"   Sensitivity {np.mean(ses)*100:.1f}% ± {np.std(ses)*100:.1f}")
    say(f"   Specificity {np.mean(sps)*100:.1f}% ± {np.std(sps)*100:.1f}")

    # ---- 7) 判决门限敏感性（产品调参入口）----
    say("\n== 判决门限 sweep（灵敏度 vs 特异度 trade-off） ==")
    say(f"   {'th':>5}{'sens%':>8}{'spec%':>8}")
    for th in (0.3, 0.4, 0.5, 0.6, 0.7):
        mm = metrics(test.label.values, p_te, th)
        say(f"   {th:>5}{mm['sens']*100:>8.1f}{mm['spec']*100:>8.1f}")
    say("   （部署原则：宁可灵敏，误报有人工取消窗口兜底）")

    joblib.dump(model, ALGO / "data" / "rf.joblib")
    report = OUT_DIR / "train_report.txt"
    report.write_text("\n".join(lines), encoding="utf-8")
    print(f"\nreport -> {report}\nmodel  -> {ALGO/'data'/'rf.joblib'}")


if __name__ == "__main__":
    main()
