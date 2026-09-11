# -*- coding: utf-8 -*-
"""
CNN 二级仲裁 —— 双语言一致性回放（宿主机硬验收）

做两件事：
  A. 算法保真度（硬门槛）：把真实 firmware/waist_firmware/cnn_detector.cpp
     用 g++ 编到 PC 上跑，与下面这份独立写的 Python 参考实现逐条对比。
     判别不一致数必须 = 0（允许浮点末位误差 <= 1e-5）。
     作用：证明"从 C++ 到板子"的这段移植没有引入行为差异。
  B. 部署保真度（仅统计）：拿 FallAllD 真实窗，按板端单位(m/s², deg/s)
     转回去、上采样成 50Hz 再走板端那条"每5拍丢1拍"的路，与直接对
     原始 40Hz 窗做推理的结果比较，量化 50->40Hz 抽取带来的偏差。

与 algorithm/05_export.py 的双语言回放同性质：这是移植正确性的硬验收。

参考实现刻意 **不** 复写常量：GRAV/ACC_DIV/GYR_DIV/GYR_LSB_PER_DPS/
CNN_HIST_N 全部从 cnn_detector.cpp 与 config.h 里正则抓取，只独立重写
逻辑（环形缓冲 / 取窗 / 抽取 / 前向）。这样才检验得出"逻辑有没有搬错"，
而不是"常量选得对不对"（后者是建模决策，另见 cnn_train.py 文档）。

用法（需要 g++ 与 numpy；Test B 需要 algo/data/ 下的 pkl，缺失则自动跳过）：
    python algo/cnn/host_replay/replay.py
"""

import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

# Windows 控制台默认 GBK，直接 print 中文/非 GBK 字符会抛 UnicodeEncodeError
# （回放结尾的结论行曾因此挂掉）。统一按 UTF-8 输出，编码失败降级为替换符。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE    = Path(__file__).resolve().parent          # algo/cnn/host_replay
ALGO    = HERE.parent.parent                       # algo
ROOT    = ALGO.parent                              # 仓库根
FW      = ROOT / "firmware" / "waist_firmware"
BUILD   = HERE / ".build"
PKL     = ALGO / "data" / "FallAllD_40SamplesPerSec_ActivityIdsFiltered.pkl"

COPY = ["cnn_detector.cpp", "cnn_detector.h", "cnn_weights.h", "config.h",
        "fall_detector.cpp", "fall_detector.h", "rf_model.h", "logger.h",
        # 报警全流程（Test C）用到的模块；net_pusher 无 secrets.h 时自动走占位实现
        "alarm_manager.cpp", "alarm_manager.h",
        "hw_button.cpp", "hw_button.h", "hw_buzzer.cpp", "hw_buzzer.h",
        "net_pusher.cpp", "net_pusher.h"]
STUBS = ["sensor_manager.h", "Arduino.h"]

# 被编译的 .cpp（全部取自 firmware/，不另存副本）
SRCS = ["cnn_detector.cpp", "fall_detector.cpp",
        "alarm_manager.cpp", "hw_button.cpp", "hw_buzzer.cpp", "net_pusher.cpp"]


# Windows 下子进程输出默认按 GBK 解码，g++ 报错里带非 GBK 字节会把
# _readerthread 直接打挂（r.stdout 变 None）。统一强制 UTF-8 + 容错替换。
def run_proc(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", **kw)


# ---------------------------------------------------------------- 构建
def build():
    if shutil.which("g++") is None:
        sys.exit("找不到 g++，无法回放（Windows 下可装 MSYS2/MinGW-w64）")
    BUILD.mkdir(exist_ok=True)
    for name in COPY:
        shutil.copyfile(FW / name, BUILD / name)
    for name in STUBS + ["main.cpp", "fusion_main.cpp"]:
        shutil.copyfile(HERE / name, BUILD / name)

    # 被编译的 .cpp 全部是 firmware/ 下的真实源码，只替换两个桩头文件
    targets = [("replay", "main.cpp"), ("fusion", "fusion_main.cpp")]
    built = {}
    for stem, driver in targets:
        exe = BUILD / (stem + (".exe" if sys.platform == "win32" else ""))
        cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-I.",
               "-o", str(exe), driver] + SRCS
        r = run_proc(cmd, cwd=BUILD)
        if r.returncode != 0:
            sys.exit(f"编译失败（{driver}）：\n" + r.stdout + r.stderr)
        if r.stderr.strip():
            print("g++ 警告：\n" + r.stderr.strip())
        print(f"[构建] OK  {exe.relative_to(ROOT)}")
        built[stem] = exe
    return built


# ------------------------------------------------- 常量：从真实源码抓取
def grab_constants():
    cpp = (FW / "cnn_detector.cpp").read_text(encoding="utf-8")
    cfg = (FW / "config.h").read_text(encoding="utf-8")
    hdr = (FW / "cnn_detector.h").read_text(encoding="utf-8")

    def f(src, name):
        m = re.search(rf"\b{name}\s*=\s*([0-9.]+)f", src) or \
            re.search(rf"#define\s+{name}\s+([0-9.]+)f", src)
        if not m:
            sys.exit(f"在源码里找不到常量 {name}")
        return float(m.group(1))

    def i(src, name):
        m = re.search(rf"#define\s+{name}\s+(\d+)", src)
        if not m:
            sys.exit(f"在源码里找不到常量 {name}")
        return int(m.group(1))

    return dict(
        grav=f(cpp, "GRAV_MS2"),
        acc_lsb=f(cpp, "ACC_LSB_PER_G"),
        acc_div=f(cpp, "ACC_DIV"),
        gyr_lsb=f(cpp, "GYR_LSB_PER_DPS"),
        gyr_div=f(cpp, "GYR_DIV"),
        hist_n=i(hdr, "CNN_HIST_N"),
        wrist_fresh=i(cfg, "WRIST_FRESH_MS"),
        min_cov=f(cfg, "CNN_MIN_COVERAGE"),
    )


# ------------------------------------------------- 权重：从生成的 .h 解析
def load_weights():
    s = (FW / "cnn_weights.h").read_text(encoding="utf-8")

    def W(name):
        m = re.search(rf"const float {name}\[(\d+)\] = \{{(.*?)\}};", s, re.S)
        if not m:
            sys.exit(f"权重组 {name} 未找到")
        v = np.array([float(x) for x in re.findall(r"([-0-9.eE+]+)f", m.group(2))],
                     dtype=np.float32)
        assert v.size == int(m.group(1)), name
        return v

    return dict(
        c1w=W("g_conv1_w").reshape(5, 12, 16),  c1b=W("g_conv1_b"),
        c2w=W("g_conv2_w").reshape(3, 16, 32),  c2b=W("g_conv2_b"),
        c3w=W("g_conv3_w").reshape(3, 32, 32),  c3b=W("g_conv3_b"),
        d1w=W("g_dense1_w").reshape(32, 16),    d1b=W("g_dense1_b"),
        d2w=W("g_dense2_w").reshape(16, 2),     d2b=W("g_dense2_b"),
    )


def conv_same(x, w, b):
    """Keras Conv1D padding='same'（核对过 k=5/3 时 C++ 的 in_t 偏移）"""
    k = w.shape[0]
    p = k // 2
    xp = np.pad(x, ((p, p), (0, 0)))
    sw = np.lib.stride_tricks.sliding_window_view(xp, k, axis=0)   # (T,k,Cin)
    return np.maximum(np.einsum("tkc,kcf->tf", sw, w) + b, 0.0)


def forward(win, Wt):
    """与 cnn_detector.cpp 的 cnn_forward 逐步对应，全程 float32"""
    x = forward_windows(win[np.newaxis, ...], Wt)
    return float(x[0])


def forward_windows(batch, Wt):
    c1 = conv_same_batch(batch, Wt["c1w"], Wt["c1b"])
    p1 = c1.reshape(len(c1), 40, 2, 16).max(axis=2)
    c2 = conv_same_batch(p1, Wt["c2w"], Wt["c2b"])
    p2 = c2.reshape(len(c2), 20, 2, 32).max(axis=2)
    c3 = conv_same_batch(p2, Wt["c3w"], Wt["c3b"])
    gap = c3.mean(axis=1)
    d1 = np.maximum(gap @ Wt["d1w"] + Wt["d1b"], 0.0)
    lo = d1 @ Wt["d2w"] + Wt["d2b"]
    lo = lo - lo.max(axis=1, keepdims=True)
    e = np.exp(lo)
    return (e / e.sum(axis=1, keepdims=True))[:, 1]


def conv_same_batch(x, w, b):
    k = w.shape[0]
    p = k // 2
    xp = np.pad(x, ((0, 0), (p, p), (0, 0)))
    sw = np.lib.stride_tricks.sliding_window_view(xp, k, axis=1)   # (N,T,Cin,k)
    sw = np.transpose(sw, (0, 1, 3, 2))
    return np.maximum(np.einsum("ntkc,kcf->ntf", sw, w) + b, 0.0)


# ------------------------------------------------- Python 参考实现（镜像 .cpp）
class Ref:
    def __init__(self, C, Wt):
        self.C, self.Wt = C, Wt
        self.reset()

    def reset(self):
        self.hist = np.zeros((self.C["hist_n"], 12), dtype=np.float32)
        self.ok = np.zeros(self.C["hist_n"], dtype=bool)
        self.head, self.cnt = 0, 0

    def push(self, acc, gyr, racc, rgyr, has_remote, age_ms):
        C = self.C
        h = np.empty(12, dtype=np.float32)
        h[0:3] = (acc / C["grav"]) * (C["acc_lsb"] / C["acc_div"])
        h[3:6] = gyr * (C["gyr_lsb"] / C["gyr_div"])
        ok = bool(has_remote) and age_ms < C["wrist_fresh"]
        if ok:
            h[6:9] = (racc / C["grav"]) * (C["acc_lsb"] / C["acc_div"])
            h[9:12] = rgyr * (C["gyr_lsb"] / C["gyr_div"])
        else:
            h[6:12] = 0.0
        self.hist[self.head] = h
        self.ok[self.head] = ok
        self.head = (self.head + 1) % C["hist_n"]
        self.cnt = min(self.cnt + 1, C["hist_n"])

    def prob(self, age):
        C = self.C
        half = 100 // 2 - 1
        if age < half:
            return False, -1.0, 0.0
        oldest = age + half
        if oldest >= self.cnt:
            return False, -1.0, 0.0
        win = np.empty((80, 12), dtype=np.float32)
        nok = 0
        for k in range(80):
            j = k + k // 4
            ago = oldest - j
            idx = (self.head - 1 - ago) % C["hist_n"]
            win[k] = self.hist[idx]
            nok += int(self.ok[idx])
        cov = nok / 80.0
        if cov < C["min_cov"]:
            return False, -1.0, cov
        return True, forward(win, self.Wt), cov


# ------------------------------------------------- 驱动
def run_cpp(exe, cases):
    """cases: list of (age, samples[])；samples: (12 floats, hasRemote, ageMs)"""
    lines = []
    for age, samples in cases:
        lines.append(f"{len(samples)} {age}")
        for s in samples:
            lines.append(" ".join(f"{v:.9g}" for v in s[:12]) +
                         f" {1 if s[12] else 0} {int(s[13])}")
    r = run_proc([str(exe)], input="\n".join(lines) + "\n")
    if r.returncode != 0:
        sys.exit("回放进程出错：\n" + r.stderr)
    out = []
    for ln in r.stdout.strip().splitlines():
        a, b, c = ln.split()
        out.append((a == "1", float(b), float(c)))
    return out


def make_case(rng, n, age, has_remote, n_samples_ok=True):
    s = []
    for _ in range(n):
        acc = rng.normal(0, 4.0, 3)          # 腰端 m/s²
        gyr = rng.normal(0, 120.0, 3)        # 腰端 deg/s
        racc = rng.normal(0, 5.0, 3)
        rgyr = rng.normal(0, 150.0, 3)
        a = 20 if has_remote else 10**6      # 新鲜 / 过期
        s.append((*acc, *gyr, *racc, *rgyr, has_remote, a))
    return age, s


def test_a(exe, C, Wt, n_case=120):
    rng = np.random.default_rng(20260911)
    cases = []
    for k in range(n_case):
        n = int(rng.integers(60, 340))
        # 七成落在"能真正跑前向"的区间（HALF=49 -> 49 <= age <= n-50），
        # 三成故意越界，专门覆盖两条早退分支（历史不足 / 窗尾越界）
        if k % 10 < 7 and n >= 110:
            age = int(rng.integers(49, n - 49))
        else:
            age = int(rng.integers(0, n + 60))
        cases.append(make_case(rng, n, age, bool(k % 3)))   # 含腕端离线的用例

    got = run_cpp(exe, cases)
    ref = Ref(C, Wt)
    bad, maxd, nrun = [], 0.0, 0
    for (age, samples), (ok, p, cov) in zip(cases, got):
        ref.reset()
        for s in samples:
            ref.push(np.array(s[0:3]), np.array(s[3:6]),
                     np.array(s[6:9]), np.array(s[9:12]), s[12], s[13])
        rok, rp, rcov = ref.prob(age)
        assert ok == rok, f"ran 标志不一致: C++={ok} ref={rok} (age={age}, n={len(samples)})"
        if not ok:
            continue
        nrun += 1
        d = abs(p - rp)
        maxd = max(maxd, d)
        if d > 1e-5:
            bad.append((age, p, rp, d))

    print(f"[A] 用例 {len(cases)} 个，其中实际推理 {nrun} 个；"
          f"判别不一致 {len(bad)} 个，最大概率偏差 {maxd:.3e}")
    for b in bad[:5]:
        print("    不一致:", b)
    return len(bad)


def test_b(exe, C, Wt):
    if not PKL.exists():
        print("[B] 跳过：找不到 algo/data/ 下的 FallAllD pkl（数据集不入库）")
        return 0
    import pandas as pd
    df = pd.read_pickle(PKL)
    df = df[df["Device"].isin(["Waist", "Wrist"])].copy()
    df["Acc"] = df["Acc"].apply(lambda x: np.asarray(x, dtype=np.float64))
    df["Gyr"] = df["Gyr"].apply(lambda x: np.asarray(x, dtype=np.float64))

    groups = list(df.groupby(["SubjectID", "ActivityID", "TrialNo"]))
    falls = [g for k, g in groups if k[1] >= 100]
    adls = [g for k, g in groups if k[1] < 100]
    rng = np.random.default_rng(7)
    picked = (list(rng.choice(len(falls), 40, replace=False)),
              list(rng.choice(len(adls), 40, replace=False)))

    cases, direct = [], []
    for pool, idxs in ((falls, picked[0]), (adls, picked[1])):
        for i in idxs:
            g = pool[i]
            wd = g[g.Device == "Waist"]
            rd = g[g.Device == "Wrist"]
            if len(wd) == 0 or len(rd) == 0:
                continue
            L = min(len(wd["Acc"].values[0]), len(rd["Acc"].values[0]))
            if L < 80:
                continue
            # 数据集原始 LSB -> 板端物理量（m/s², deg/s）
            wa = wd["Acc"].values[0][:80] / C["acc_lsb"] * C["grav"]
            wg = wd["Gyr"].values[0][:80] / C["gyr_lsb"]
            ra = rd["Acc"].values[0][:80] / C["acc_lsb"] * C["grav"]
            rg = rd["Gyr"].values[0][:80] / C["gyr_lsb"]
            F40 = np.hstack([wa, wg, ra, rg])
            direct.append(np.hstack([wa / C["grav"], wg * C["gyr_lsb"] / C["gyr_div"],
                                     ra / C["grav"], rg * C["gyr_lsb"] / C["gyr_div"]])
                          .astype(np.float32))
            # 上采样成 50Hz（板端实际采样率），交给板端那条抽取路径
            idx40 = np.arange(80)
            idx50 = np.arange(100) / 1.25
            F50 = np.stack([np.interp(idx50, idx40, F40[:, c]) for c in range(12)], axis=1)
            samples = [(*F50[i], True, 0) for i in range(100)]
            cases.append((49, samples))        # age=49 -> 窗恰好覆盖这 100 拍

    got = run_cpp(exe, cases)
    ref = Ref(C, Wt)
    bad, maxd = 0, 0.0
    cpp_p, ref_p = [], []
    for (age, samples), (ok, p, cov) in zip(cases, got):
        ref.reset()
        for s in samples:
            ref.push(np.array(s[0:3]), np.array(s[3:6]),
                     np.array(s[6:9]), np.array(s[9:12]), s[12], s[13])
        rok, rp, _ = ref.prob(age)
        if ok != rok:
            bad += 1
            continue
        if not ok:
            continue
        cpp_p.append(p)
        ref_p.append(rp)
        maxd = max(maxd, abs(p - rp))

    cpp_p = np.array(cpp_p, dtype=np.float32)
    direct_p = forward_windows(np.array(direct[:len(cpp_p)], dtype=np.float32), Wt)
    dev = np.abs(cpp_p - direct_p)
    print(f"[B] 真实窗 {len(cpp_p)} 个（跌到/日常各半）；"
          f"与参考实现不一致 {bad} 个，最大偏差 {maxd:.3e}")
    print(f"    50->40Hz 抽取 + 单位往返造成的概率偏差："
          f"均值 {dev.mean():.4f}，最大 {dev.max():.4f}，"
          f"跨 0.80 门限翻转 {(np.sign(cpp_p - 0.8) != np.sign(direct_p - 0.8)).sum()} 个")
    return bad


def test_c(exe):
    """固件集成：编译并跑真实的 fall_detector.cpp（状态机 + 仲裁门控 + 复位）。

    验的是 A/B 两测盖不到的东西：门控是否真的按"腕端在线"生效、
    腕端离线时是否确实回退到纯 RF、复位后能不能重新判定。
    """
    r = run_proc([str(exe)])
    for ln in r.stdout.strip().splitlines():
        print("    " + ln)
    if r.returncode != 0:
        print("[C] 集成回放失败（见上面 !! 行）")
        return 1
    print("[C] 两级判决集成回放通过（状态机 + 仲裁门控 + 复位）")
    return 0


def main():
    built = build()
    C = grab_constants()
    Wt = load_weights()
    print("[常量] " + ", ".join(f"{k}={v:g}" for k, v in C.items()))

    bad = test_a(built["replay"], C, Wt)
    bad += test_b(built["replay"], C, Wt)
    bad += test_c(built["fusion"])

    print("=" * 60)
    if bad == 0:
        print("[PASS] 一致性回放通过：C++ 与 Python 参考实现完全一致")
    else:
        print(f"[FAIL] 回放失败：{bad} 处不一致，必须逐条人工审查")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
