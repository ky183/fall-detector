# -*- coding: utf-8 -*-
"""
第⑥步-A：小随机森林导出 C 常量 + 双语言一致性回放

流程：
    1. 用与 03/04 完全一致的数据划分/种子重训 15树×深6 小森林
    2. 导出为 C 头文件 firmware/waist_firmware/rf_model.h（自动生成，勿手改）
       + JSON 副本 algo/out/rf_model.json（回放用）
    3. ★ 一致性回放（移植正确性的硬验收）：
       用 Python 按 C 代码同款逻辑（float32 阈值 + 逐树遍历取均值）重算
       全部 5920 个样本的判决，与 sklearn 原模型逐条对比。
       判决不一致数必须 = 0（允许极少数位于门限边界的浮点翻转，
       如出现会逐条列出人工审查）。

C 端约定（与 fall_detector.cpp 的遍历代码一一对应）：
    节点: feat>=0 为内部节点（x[feat] <= thr 走 left），feat=-1 为叶子
    概率: 全部树叶子 pfall 的平均，>= FD_RF_THRESHOLD(0.4) 判跌倒
    特征顺序: 与 02_features.FEATURES 完全一致（写死在头文件注释里）
"""

import json
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier

ALGO = Path(__file__).resolve().parent
FW = ALGO.parent / "firmware" / "waist_firmware"
TEST_SUBJECTS = ["SA04", "SA09", "SA14", "SA19", "SA23"]
SEED = 42
DEPLOY_TH = 0.4

import importlib.util as _ilu
_spec = _ilu.spec_from_file_location("feat", ALGO / "02_features.py")
_feat = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_feat)
FEATURES = _feat.FEATURES


def train_small_rf():
    df = pd.read_csv(ALGO / "data" / "features.csv")
    train = df[(df.group == "SA") & (~df.subject.isin(TEST_SUBJECTS))]
    rf = RandomForestClassifier(n_estimators=15, max_depth=6,
                                random_state=SEED, n_jobs=-1)
    rf.fit(train[FEATURES], train.label)
    return rf, df


def extract_trees(rf):
    """sklearn 森林 -> 扁平节点数组（顺序遍历每棵树的所有节点）"""
    nodes, starts = [], []
    for est in rf.estimators_:
        t = est.tree_
        starts.append(len(nodes))
        for i in range(t.node_count):
            if t.children_left[i] == -1:      # 叶子
                n0, n1 = t.value[i][0]
                p = float(n1 / (n0 + n1)) if (n0 + n1) > 0 else 0.5
                nodes.append(dict(feat=-1, thr=0.0, left=0, right=0, pfall=p))
            else:
                # float32：与 C 端 float 精度一致，回放才可比
                nodes.append(dict(feat=int(t.feature[i]),
                                  thr=float(np.float32(t.threshold[i])),
                                  left=int(t.children_left[i]) + starts[-1],
                                  right=int(t.children_right[i]) + starts[-1],
                                  pfall=0.0))
    return nodes, starts


def c_mirror_predict(nodes, starts, x):
    """C 遍历逻辑的 Python 镜像（fall_detector.cpp 里 rf_predict 的逐行对照）"""
    acc = 0.0
    for s in starts:
        i = s
        while nodes[i]["feat"] >= 0:
            i = nodes[i]["left"] if x[nodes[i]["feat"]] <= nodes[i]["thr"] \
                else nodes[i]["right"]
        acc += nodes[i]["pfall"]
    return acc / len(starts)


def flit(v: float) -> str:
    """浮点 -> 合法 C 字面量。两个坑：

    1. %.9g 会把 0.0 输出成 '0'，拼成 '0f' 非法 -> 必须保证小数点存在
    2. ★ C 端存的是 float(32 位)，必须**先降到 float32 再取 9 位有效数字**。
       直接对 float64 取 9 位会丢 1 ULP：例如 24/99 的 float64 取 9 位得
       '0.242424242'，解析回 float32 是 0.24242423，而 C 端期望的是
       float32(24/99) = 0.24242425。全表 2166 个值里有 2 个这种情况，
       会让 rf_model.h 与 rf_model.json 的判决在门限附近不再逐位一致
       （tools/check_consistency.py 会报出来）。
    """
    s = f"{float(np.float32(v)):.9g}"
    if not any(c in s for c in ".eE"):
        s += ".0"
    return s + "f"


def emit_c_header(nodes, starts):
    feat_comment = "\n".join(f"//   [{i}] {f}" for i, f in enumerate(FEATURES))
    lines = [
        "// ============================================================",
        "//  rf_model.h — 小随机森林跌倒分类器（自动生成，勿手改）",
        f"//  生成: algo/05_export.py | 15树 x 深度6 | 训练集=18个青年受试者",
        "//  特征顺序（与 02_features.py FEATURES 一致，缺一不可）:",
        feat_comment,
        "//  用法: float p = rf_predict(feats); p >= FD_RF_THRESHOLD 判跌倒",
        "// ============================================================",
        "#pragma once",
        "",
        "typedef struct {",
        "    int8_t   feat;   // >=0: 分裂特征下标; -1: 叶子",
        "    float    thr;    // x[feat] <= thr 走 left，否则走 right",
        "    int16_t  left;   // 左子节点下标（全局）",
        "    int16_t  right;  // 右子节点下标（全局）",
        "    float    pfall;  // 叶子: P(跌倒)；内部节点无效",
        "} RfNode_t;",
        "",
        f"#define RF_NUM_TREES   {len(starts)}",
        f"#define RF_TOTAL_NODES {len(nodes)}",
        "",
        f"static const RfNode_t RF_NODES[RF_TOTAL_NODES] = {{",
    ]
    for n in nodes:
        lines.append(f"    {{{n['feat']}, {flit(n['thr'])}, "
                     f"{n['left']}, {n['right']}, {flit(n['pfall'])}}},")
    lines.append("};")
    lines.append(f"static const int16_t RF_TREE_STARTS[RF_NUM_TREES] = {{"
                 + ", ".join(map(str, starts)) + "};")
    lines.append("")
    (FW / "rf_model.h").write_text("\n".join(lines), encoding="utf-8")
    return len(nodes)


def main():
    rf, df = train_small_rf()
    nodes, starts = extract_trees(rf)
    n = emit_c_header(nodes, starts)
    json.dump(dict(features=FEATURES, nodes=nodes, starts=starts,
                   deploy_th=DEPLOY_TH),
              open(ALGO / "out" / "rf_model.json", "w"))
    print(f"C header -> {FW / 'rf_model.h'}  ({n} nodes, "
          f"约 {n * 16 / 1024:.0f} KB 常量)")

    # ---- 一致性回放：C 镜像逻辑 vs sklearn，全量 5920 样本 ----
    X = df[FEATURES].values
    p_sk = rf.predict_proba(X)[:, 1]
    p_c = np.array([c_mirror_predict(nodes, starts, row) for row in X])
    dec_sk = (p_sk >= DEPLOY_TH).astype(int)
    dec_c = (p_c >= DEPLOY_TH).astype(int)
    mism = np.where(dec_sk != dec_c)[0]
    print(f"\n== 一致性回放（{len(df)} 样本，门限 {DEPLOY_TH}） ==")
    print(f"   概率最大偏差   = {np.abs(p_sk - p_c).max():.2e}")
    print(f"   判决不一致数量 = {len(mism)}")
    if len(mism):
        for i in mism[:10]:
            print(f"   [差异] row{i}: sklearn={p_sk[i]:.6f} c_mirror={p_c[i]:.6f} "
                  f"label={df.label.iloc[i]}")
        raise SystemExit("!! 存在判决不一致，禁止上板 —— 逐条排查")
    print("   PASS：双语言实现完全一致，可上板")


if __name__ == "__main__":
    main()
