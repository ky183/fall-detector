# -*- coding: utf-8 -*-
"""
全项目静态一致性自检（不需要硬件、不需要数据集）

验的是"跨文件/跨语言对不上就会静默出错"的那些接口，这类错误编译器和
离线指标都抓不到，但只要错一处，板上的行为就和离线验证过的不一样：

  1. 特征顺序：02_features.py(训练) / rf_model.h(注释) / rf_model.json /
     fall_detector.cpp(feats[] 赋值) 四处必须完全同序
  2. RF 导出保真：rf_model.h 的 1083 个节点与 starts 必须与
     algo/out/rf_model.json 在 float32 下逐字段完全相等
     （.h 只写 9 位有效数字，若位数不够会在门限附近改变判决）
  3. 板间协议：两端 protocol.h 的枚举/结构体/校验逻辑必须一致
  4. 两端 config.h：ESPNOW_CHANNEL、SAMPLE_HZ 必须一致
  5. README 引脚表 vs 两端 config.h：文档与代码不能各说各话
  6. CNN 侧常量：cnn_detector.cpp 的量程标度必须与 sensor_manager.cpp
     实际配置的 MPU 量程对得上（acc ±8g / gyr ±500dps）

用法：python tools/check_consistency.py
退出码 0 = 全部通过；非 0 = 有检查项失败（逐条打印，便于定位）
"""

import json
import re
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parent.parent
WAIST = ROOT / "firmware" / "waist_firmware"
WRIST = ROOT / "firmware" / "wrist_firmware"

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("  [OK] " if ok else "  [!!] ") + name + (("  " + detail) if detail else ""))


def read(p):
    return p.read_text(encoding="utf-8")


# ---------------------------------------------------------------- 1. 特征顺序
def check_feature_order():
    """四处特征顺序必须一致：训练 -> 导出 -> JSON -> 固件赋值"""
    feat_py = None
    blk = re.search(r"FEATURES\s*=\s*\[(.*?)\]",
                    read(ROOT / "algo" / "02_features.py"), re.S)
    if blk:
        # 必须先剔注释：FEATURES 块里的说明文字含中文引号内容（如 "砸坐"），
        # 直接抽引号会把注释文字当成特征名
        code = "\n".join(ln.split("#", 1)[0] for ln in blk.group(1).splitlines())
        feat_py = re.findall(r"[\"'](\w+)[\"']", code)

    h = read(WAIST / "rf_model.h")
    feat_h = [n for _, n in sorted(re.findall(r"//\s*\[(\d+)\]\s*(\w+)", h),
                                   key=lambda t: int(t[0]))]
    feat_json = json.loads(read(ROOT / "algo" / "out" / "rf_model.json"))["features"]

    # fall_detector.cpp：每个下标取**首次**出现（if/else 会重复赋值同一下标）
    lines = read(WAIST / "fall_detector.cpp").splitlines()
    seen = {}
    for i, ln in enumerate(lines):
        mo = re.search(r"feats\[(\d+)\]\s*=", ln)
        if not mo or int(mo.group(1)) in seen:
            continue
        for j in range(i, min(i + 6, len(lines))):
            cm = re.search(r"//\s*(\w+)\s*$", lines[j])
            if cm:
                seen[int(mo.group(1))] = cm.group(1)
                break
    feat_cpp = [v for _, v in sorted(seen.items())]

    base = feat_py
    check("特征顺序 · 02_features.py 可解析", base is not None and len(base) == 12)
    for label, lst in [("rf_model.h 注释", feat_h),
                       ("algo/out/rf_model.json", feat_json),
                       ("fall_detector.cpp feats[]", feat_cpp)]:
        same = (lst == base)
        check("特征顺序 · " + label, same,
              "" if same else f"{lst} != {base}")


# ------------------------------------------------------- 2. RF 导出保真（float32）
def check_rf_fidelity():
    import numpy as np

    h = read(WAIST / "rf_model.h")
    j = json.loads(read(ROOT / "algo" / "out" / "rf_model.json"))

    body = re.search(r"RF_NODES\[RF_TOTAL_NODES\]\s*=\s*\{(.*?)\n\};", h, re.S)
    nodes_h = re.findall(r"\{\s*(-?\d+),\s*([-0-9.eE+]+)f,\s*(-?\d+),\s*(-?\d+),\s*([-0-9.eE+]+)f\s*\}",
                         body.group(1))
    starts_h = re.findall(r"RF_TREE_STARTS\[RF_NUM_TREES\]\s*=\s*\{([^}]*)\}", h)
    starts_h = [int(x) for x in re.findall(r"-?\d+", starts_h[0])]

    nodes_j = j["nodes"]
    starts_j = j["starts"]

    check("RF 节点数一致", len(nodes_h) == len(nodes_j) == 1083,
          f"h={len(nodes_h)} json={len(nodes_j)}")
    check("RF 树数/起始下标一致", starts_h == starts_j,
          "" if starts_h == starts_j else f"{starts_h} != {starts_j}")

    bad = []
    for i, (n, jn) in enumerate(zip(nodes_h, nodes_j)):
        f_h, thr_h, l_h, r_h, p_h = n
        cmp32 = lambda a, b: np.float32(float(a)) == np.float32(float(b))
        if (int(f_h) != jn["feat"] or int(l_h) != jn["left"] or int(r_h) != jn["right"]
                or not cmp32(thr_h, jn["thr"]) or not cmp32(p_h, jn["pfall"])):
            bad.append((i, n, jn))
    check("RF 节点逐字段 float32 相等", not bad,
          f"{len(bad)} 个不符" + (f"，首个 {bad[0]}" if bad else ""))
    for i, n, jn in bad[:6]:
        print(f"        #{i}: h={n}  json=feat{jn['feat']} thr{jn['thr']!r} "
              f"l{jn['left']} r{jn['right']} pfall{jn['pfall']!r}")
        print(f"            float32: thr {np.float32(float(n[1]))!r} vs "
              f"{np.float32(jn['thr'])!r} | pfall "
              f"{np.float32(float(n[4]))!r} vs {np.float32(jn['pfall'])!r}")


# ---------------------------------------------------------------- 3. 板间协议
def check_protocol():
    """两端 protocol.h 除注释外必须逐字节一致（文件自己就是这么约定的）"""
    a = read(WAIST / "protocol.h")
    b = read(WRIST / "protocol.h")
    strip = lambda s: "\n".join(ln for ln in s.splitlines()
                                if not ln.strip().startswith("//")).strip()
    check("protocol.h 两端代码部分一致", strip(a) == strip(b))
    if strip(a) != strip(b):
        import difflib
        print("\n".join(list(difflib.unified_diff(
            strip(a).splitlines(), strip(b).splitlines(),
            "waist/protocol.h", "wrist/protocol.h", lineterm=""))[:20]))


# ------------------------------------------------- 4/5. 配置常量与文档引脚
def grab_define(text, name):
    m = re.search(rf"#define\s+{name}\s+([^\s/]+)", text)
    return m.group(1) if m else None


def check_configs():
    cw = read(WAIST / "config.h")
    cr = read(WRIST / "config.h")
    for name in ("ESPNOW_CHANNEL", "SAMPLE_HZ"):
        a, b = grab_define(cw, name), grab_define(cr, name)
        check(f"两端 {name} 一致", a == b, f"waist={a} wrist={b}")

    # README 引脚表 vs config.h（README 写 "GPIO8"，config 写 8，需归一化）
    norm = lambda s: re.sub(r"[^0-9]", "", (s or ""))
    pairs = [("腰端 I2C SDA", "PIN_I2C_SDA", cw, "GPIO8"),
             ("腰端 I2C SCL", "PIN_I2C_SCL", cw, "GPIO9"),
             ("腰端蜂鸣器", "PIN_BUZZER", cw, "GPIO4"),
             ("腰端取消按钮", "PIN_BTN_CANCEL", cw, "GPIO5"),
             ("腕端 I2C SDA", "PIN_I2C_SDA", cr, "GPIO5"),
             ("腕端 I2C SCL", "PIN_I2C_SCL", cr, "GPIO6"),
             ("腕端按钮", "PIN_BTN_1", cr, "GPIO3")]
    readme = read(ROOT / "README.md")
    readme_pins = set(norm(x) for x in re.findall(r"GPIO(\d+)", readme))
    for label, macro, cfg, expect in pairs:
        v = grab_define(cfg, macro)
        check(f"引脚 {label} ({macro}={v})", norm(v) == norm(expect),
              "" if norm(v) == norm(expect)
              else f"config.h={v}，README 写的是 {expect}")
        readme_pins.discard(norm(expect))

    # README 提了但两端 config.h 都没定义的引脚（文档与代码各说各话）
    defined = set(norm(grab_define(cw, m)) for m in
                  re.findall(r"#define\s+(PIN_\w+)", cw))
    defined |= set(norm(grab_define(cr, m)) for m in
                   re.findall(r"#define\s+(PIN_\w+)", cr))
    orphan = sorted(p for p in readme_pins if p not in defined)
    check("README 引脚表每一条都有对应宏", not orphan,
          "" if not orphan else f"README 提到但代码未定义: GPIO{', GPIO'.join(orphan)}"
          "（若为预留功能，请在 README 行末标注“预留”）")

    # 按钮数量：README 列出的按钮都要有对应宏（行内标注“预留”的不算）
    btn_rows = len([1 for ln in readme.splitlines()
                    if re.search(r"按钮\d", ln) and "预留" not in ln])
    btn_macros = len(re.findall(r"#define\s+PIN_BTN_\w+", cw))
    check("README 按钮数与腰端宏数一致", btn_rows <= btn_macros,
          f"README 列了 {btn_rows} 个已实现腰端按钮，config.h 只有 {btn_macros} 个 PIN_BTN_* 宏")


# ------------------------------------------------- 6. CNN 量程标度 vs MPU 配置
def check_cnn_scale():
    cnn = read(WAIST / "cnn_detector.cpp")
    sm = read(WAIST / "sensor_manager.cpp")

    lsb = re.search(r"GYR_LSB_PER_DPS\s*=\s*([0-9.]+)f", cnn)
    acc_lsb = re.search(r"ACC_LSB_PER_G\s*=\s*([0-9.]+)f", cnn)
    check("CNN 常量可解析", lsb and acc_lsb)

    # sensor_manager.cpp 里由寄存器配置推出的实际标度
    gyr_full = re.search(r"GYRO_LSB\s*=\s*([0-9.]+)f\s*/\s*([0-9.]+)f", sm)
    acc_full = re.search(r"ACCEL_LSB\s*=\s*\(\s*([0-9.]+)f\s*\*\s*([0-9.]+)f\s*\)\s*/\s*([0-9.]+)f", sm)
    gyr_range, gyr_n = float(gyr_full.group(1)), float(gyr_full.group(2))
    acc_g, _, acc_n = (float(acc_full.group(i)) for i in (1, 2, 3))

    lsb_real = gyr_n / gyr_range                 # LSB per dps
    lsb_cfg_g = acc_n / acc_g                    # LSB per g
    check(f"陀螺量程标度一致 (±{gyr_range:.0f}dps -> {lsb_real:.3f} LSB/dps)",
          abs(lsb_real - float(lsb.group(1))) < 0.05,
          f"cnn={lsb.group(1)} 实际={lsb_real:.3f}")
    check(f"加速度量程标度一致 (±{acc_g:.0f}g -> {lsb_cfg_g:.0f} LSB/g)",
          abs(lsb_cfg_g - float(acc_lsb.group(1))) < 0.5,
          f"cnn={acc_lsb.group(1)} 实际={lsb_cfg_g:.0f}")


def main():
    print("=" * 64)
    print("全项目静态一致性自检")
    print("=" * 64)
    check_feature_order()
    print("-" * 64)
    check_rf_fidelity()
    print("-" * 64)
    check_protocol()
    print("-" * 64)
    check_configs()
    print("-" * 64)
    check_cnn_scale()

    bad = [r for r in results if not r[1]]
    print("=" * 64)
    if bad:
        print(f"[FAIL] {len(bad)}/{len(results)} 项不通过：")
        for n, _, d in bad:
            print(f"   - {n} {d}")
    else:
        print(f"[PASS] {len(results)} 项全部通过")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
