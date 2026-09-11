// ============================================================
//  ★ 跌倒判定算法 — 三阶段触发 + 小随机森林精判（v2）+ CNN 二级仲裁 ★
//  训练/评估/导出全流程见仓库 algo/ 目录（SisFall 数据集，38 人）
//  离线指标（留出集=5 个全新受试者）：灵敏度 96.0% 特异度 98.4%
//  老年组（SE）日常误报 4.6%，SE06 老人真摔检出 85.3%
//
//  状态机（与离线训练管线严格同构 —— 这是离线指标能迁移到板上的前提）：
//    ST_BASELINE  开机站姿基线采集（FD_BASELINE_MS）。假设开机时人站立，
//                  基线 = 该时段倾角中位数。ΔTilt 全部相对此基线。
//    ST_IDLE      50Hz 滚动维护：姿态 IIR、SVM/角速度/ΔTilt 环形缓冲；
//                  SVM 或角速度越限 -> 触发（对应训练里的"冲击锚点"）
//    ST_COLLECT   触发后继续采集 FD_COLLECT_MS（覆盖峰值 + 3s 确认窗），
//                  然后在缓冲内找 SVM 峰值为锚点，按训练同款窗口算特征，
//                  送 rf_predict()，>= FD_RF_THRESHOLD 判跌倒
//    ST_LATCHED   判跌倒后锁存：只返回一次 FALL_CONFIRMED，
//                  直到 fall_detector_reset()（报警取消时由 alarm 调用）
//
//  与训练管线的对应关系（改任何一处必须同步改 algo/ 侧并重训）：
//    姿态 IIR   alpha=0.03 @50Hz      <-> algo/sisfall.py tilt_angle()
//    窗口/特征  见下方 WINDOW 注释     <-> algo/02_features.py extract_one()
//    量程裁剪   CLIP_ACC/CLIP_GYR      <-> 同名常量（训练时已按硬件量程）
//    森林       rf_model.h（自动生成） <-> algo/05_export.py
//
//  ★ 二级仲裁：CNN 只在"触发后 + 腕端在线"时跑一次 ★
//    主判决仍是上面的 RF；CNN 不参与常态判决，只用 RF 用不到的
//    腕端 6 通道信息做补判/否决（角色开关见 config.h 的 CNN_ROLE_*）。
//    训练/评估见 algo/cnn/，权重 cnn_weights.h，实现 cnn_detector.cpp
// ============================================================
#include "fall_detector.h"
#include "cnn_detector.h"    // CNN 二级仲裁（仅在 decide() 内调用）
#include "rf_model.h"
#include "logger.h"
#include "config.h"
#include <math.h>

// ================= 常量（与 algo/ 侧一致） =================
static const int   FS        = SAMPLE_HZ;      // 50Hz
static const int   RING_N    = 96;             // 预触发环形缓冲 1.92s（dip 窗需 1.5s）
static const int   EV_N      = 320;            // 事件缓冲：1.92s 历史 + 2.56s 采集
static const int   WIN_MIN   = FS / 4;         // 窗口最少 0.25s 样本（同训练 _seg）
static const float GRAV      = 9.81f;
static const float CLIP_ACC  = 8.0f * GRAV;    // 域对齐：MPU ±8g
static const float CLIP_GYR  = 500.0f;         // 域对齐：MPU ±500°/s
static const float TILT_ALPHA = 0.03f;         // 姿态 IIR 系数（同训练）
static const float DIP_TH    = 0.8f * GRAV;    // 失重判据 0.8g（同训练 dip_dur_pre）

// ================= 状态 =================
enum State { ST_BASELINE, ST_IDLE, ST_COLLECT, ST_LATCHED };
static State st = ST_BASELINE;

static float  lp[3];            // 加速度低通（重力方向）
static bool   lpInit = false;

static float  baseBuf[FS * 12]; // 基线采集缓冲（6s @50Hz 上限）
static int    baseN = 0;
static float  baselineTilt = 90.0f;

static float  rSvm[RING_N], rDt[RING_N], rGm[RING_N];   // 预触发环形
static float  rWsvm[RING_N];     // 腕端 SVM 预触发环形（-1 = 该时刻无有效腕端数据）
static int    rHead = 0, rCnt = 0;

static float  eSvm[EV_N], eDt[EV_N], eGm[EV_N];         // 事件缓冲（时间升序）
static float  eWsvm[EV_N];       // 腕端 SVM 事件缓冲（-1 = 缺样）
static int    eN = 0;
static uint32_t trigMs = 0;
static bool   pendingConfirm = false;

static FallDebug dbg = {};

// ================= 小工具 =================
static float medianOf(const float* a, int i0, int i1);
static float seg_max(const float* a, int i0, int i1) {
    float m = a[i0];
    for (int i = i0 + 1; i < i1; i++) if (a[i] > m) m = a[i];
    return m;
}
static float seg_min(const float* a, int i0, int i1) {
    float m = a[i0];
    for (int i = i0 + 1; i < i1; i++) if (a[i] < m) m = a[i];
    return m;
}
static float seg_mean(const float* a, int i0, int i1) {
    float s = 0; for (int i = i0; i < i1; i++) s += a[i];
    return s / (i1 - i0);
}
static float seg_std(const float* a, int i0, int i1) {
    float mu = seg_mean(a, i0, i1), s = 0;
    for (int i = i0; i < i1; i++) s += (a[i] - mu) * (a[i] - mu);
    return sqrtf(s / (i1 - i0));
}
// 窗口取值：越界自动夹紧；不足 WIN_MIN 返回 false（同训练的弃窗逻辑）
static bool seg_range(int anchor, float t0s, float t1s,
                      int* a, int* b) {
    *a = anchor + (int)(t0s * FS);
    *b = anchor + (int)(t1s * FS);
    if (*a < 0) *a = 0;
    if (*b > eN) *b = eN;
    return (*b - *a) >= WIN_MIN;
}

// ================= 随机森林推理（与 05_export 回放逐行对照） =================
static float rf_predict(const float x[12]) {
    float acc = 0;
    for (int t = 0; t < RF_NUM_TREES; t++) {
        int i = RF_TREE_STARTS[t];
        while (RF_NODES[i].feat >= 0)
            i = (x[RF_NODES[i].feat] <= RF_NODES[i].thr)
                ? RF_NODES[i].left : RF_NODES[i].right;
        acc += RF_NODES[i].pfall;
    }
    return acc / RF_NUM_TREES;
}

// ================= 事件判定（触发后调用一次） =================
static void decide() {
    // 锚点 = 事件缓冲内的 SVM 峰值（同训练 argmax）
    int iP = 0;
    for (int i = 1; i < eN; i++) if (eSvm[i] > eSvm[iP]) iP = i;

    // ---- 窗口定义（秒，相对锚点；与 02_features.py 完全一致） ----
    int a, b; bool ok;
    float feats[12];

    ok = seg_range(iP, -0.5f, 0.5f, &a, &b);                 // imp 冲击窗
    feats[0] = ok ? seg_max(eSvm, a, b) : seg_max(eSvm, 0, eN);        // svm_peak_imp
    ok = seg_range(iP, -1.5f, -0.2f, &a, &b);                // dip 失重窗
    feats[1] = ok ? seg_min(eSvm, a, b) : seg_mean(eSvm, 0, eN);       // svm_dip_pre
    int dipLo = a, dipHi = b; bool dipOk = ok;
    ok = seg_range(iP, -1.0f, 0.0f, &a, &b);                 // pre 冲击前窗
    feats[2] = ok ? seg_mean(eSvm, a, b) : seg_mean(eSvm, 0, eN);      // svm_mean_pre
    int preA = a, preB = b; bool preOk = ok;
    ok = seg_range(iP, 0.5f, 3.0f, &a, &b);                  // post 确认窗
    int postA = a, postB = b; bool postOk = ok;
    feats[3] = postOk ? seg_std(eSvm, postA, postB)
                      : seg_std(eSvm, eN - FS, eN);                     // svm_std_post
    feats[4] = postOk ? seg_mean(eSvm, postA, postB)
                      : seg_mean(eSvm, eN - FS, eN);                    // svm_mean_post
    feats[5] = postOk ? medianOf(eDt, postA, postB) : 0.0f;             // dtilt_med_post
    feats[6] = postOk ? seg_max(eDt, postA, postB) : 0.0f;              // dtilt_max_post
    ok = seg_range(iP, -0.5f, 0.5f, &a, &b);
    feats[7] = ok ? seg_max(eGm, a, b) : seg_max(eGm, 0, eN);           // gmag_peak_imp
    feats[8] = postOk ? seg_std(eGm, postA, postB) : 0.0f;              // gmag_std_post
    ok = seg_range(iP, -0.4f, 0.2f, &a, &b);                 // energy 能量窗
    if (ok) { float s = 0; for (int i = a; i < b; i++)
                  s += fabsf(eSvm[i] - GRAV);
              feats[9] = s / (b - a); }                                   // svm_impulse_imp
    else feats[9] = 0.0f;
    if (dipOk) { int c = 0; for (int i = dipLo; i < dipHi; i++)
                     if (eSvm[i] < DIP_TH) c++;
                 feats[10] = (float)c / (dipHi - dipLo); }                // dip_dur_pre
    else feats[10] = 0.0f;
    feats[11] = (postOk && preOk)                                          // dtilt_chg
        ? fabsf(medianOf(eDt, postA, postB) - medianOf(eDt, preA, preB)) : 0.0f;

    float p = rf_predict(feats);
    dbg.valid = true;
    dbg.proba = p;
    for (int i = 0; i < 12; i++) dbg.feats[i] = feats[i];
    dbg.decideMs = millis() - trigMs;

    // ---- 腕端融合 v1：否决票（保守 + 失效开放）----
    // 在与腰端相同的冲击窗 ±0.5s 内统计腕端 SVM 峰值与覆盖率。
    // 否决条件（三条同时满足才否决）：
    //   ①覆盖率达标（链路健康，数据可信）
    //   ②腰端冲击很强（≥FUSION_VETO_WAIST_STRONG，弱冲击本就难报警）
    //   ③腕端几乎没动（<FUSION_VETO_WRIST_QUIET ≈1.2g 静止水平）
    // 物理依据：人整体跌倒时手臂几乎必有联动震动；只有"装置单独被
    // 磕碰/摆弄"才会腰端巨震而腕端纹丝不动。
    // 无论开关状态都统计并记录（自采数据标定用）。
    int wa = iP - FS / 2, wb = iP + FS / 2;
    if (wa < 0) wa = 0;
    if (wb > eN) wb = eN;
    float wPeak = -1.0f;
    int wCnt = 0;
    for (int i = wa; i < wb; i++) {
        if (eWsvm[i] >= 0.0f) { wCnt++; if (eWsvm[i] > wPeak) wPeak = eWsvm[i]; }
    }
    float wCov = (wb > wa) ? (float)wCnt / (wb - wa) : 0.0f;
    dbg.wristPeak = wPeak;
    dbg.wristCov = wCov;
    dbg.vetoed = false;
#if ENABLE_WRIST_FUSION
    if (wCov >= FUSION_MIN_COVERAGE && feats[0] >= FUSION_VETO_WAIST_STRONG &&
        wPeak >= 0.0f && wPeak < FUSION_VETO_WRIST_QUIET) {
        dbg.vetoed = true;
        LOG_I("DETC", "wrist VETO: waist peak=%.1f but wrist=%.1f cov=%.0f%% "
              "(device knocked, not a body fall)", feats[0], wPeak, wCov * 100);
    }
#endif

    // ---- CNN 二级仲裁（第二级判决，见 cnn_detector.h）----
    // decide() 本身只会在冲击触发后被调用且只调一次，所以这里天然满足
    // "只在触发后跑"；再用覆盖率卡住"腕端在线"这一条。
    // 锚点复用同一个 iP，仲裁窗 = 以冲击峰值为中心的 2s。
    dbg.cnnRan      = false;
    dbg.cnnProba    = -1.0f;
    dbg.cnnCoverage = 0.0f;
    dbg.cnnVetoed   = false;
    dbg.cnnRescued  = false;
#if ENABLE_CNN_ARBITER
    {
        int   anchorAgo = (eN - 1) - iP;   // 锚点距今多少个 50Hz 样本
        float cnnP = -1.0f, cnnCov = 0.0f;
        dbg.cnnRan = cnn_detector_prob(anchorAgo, CNN_MIN_COVERAGE,
                                       &cnnP, &cnnCov);
        if (dbg.cnnRan) dbg.cnnProba = cnnP;
        dbg.cnnCoverage = cnnCov;
    }
#endif

    // ---- 两级判决合成 ----
    // 第一级：RF 主判决（含旧的腕端否决票），行为与 v2 一致
    bool fire = !dbg.vetoed && (p >= FD_RF_THRESHOLD);
#if ENABLE_CNN_ARBITER && (CNN_ROLE_VETO || CNN_ROLE_RESCUE)
    if (dbg.cnnRan) {
#if CNN_ROLE_VETO
        // 否决：RF 判跌倒、CNN 高置信否认 -> 压掉这次报警（会降灵敏度）
        if (fire && dbg.cnnProba <= CNN_CALM_THRESHOLD) {
            fire = false;
            dbg.cnnVetoed = true;
        }
#endif
#if CNN_ROLE_RESCUE
        // 补判：RF 未判跌倒、CNN 高置信确认 -> 补一次报警（只增不减）
        if (!fire && dbg.cnnProba >= CNN_FALL_THRESHOLD) {
            fire = true;
            dbg.cnnRescued = true;
        }
#endif
    }
#endif

    LOG_I("DETC", "decision proba=%.2f peak=%.1fm/s2 tilt=%.0fdeg "
          "gyr=%.0fdps wrist=%.1f/cov%.0f%% proba_th=%.2f (%lums)",
          p, feats[0], feats[5], feats[7], wPeak, wCov * 100,
          FD_RF_THRESHOLD, (unsigned long)dbg.decideMs);
    LOG_I("DETC", "cnn %-4s p=%.2f cov=%.0f%% th=%.2f%s%s",
          dbg.cnnRan ? "ran" : "skip", dbg.cnnProba, dbg.cnnCoverage * 100,
          CNN_FALL_THRESHOLD,
          dbg.cnnVetoed ? " VETO" : "", dbg.cnnRescued ? " RESCUE" : "");

    if (fire) {
        st = ST_LATCHED;
        pendingConfirm = true;
    } else {
        st = ST_IDLE;
        rCnt = 0;   // 清空预触发缓冲，避免同一事件的尾巴再次触发
    }
}

// ================= 主入口（50Hz） =================
FallEvent fall_detector_update(const FallInput& in) {
    // CNN 二级仲裁的 50Hz 历史：每拍都要喂（基线期/锁存期也喂），
    // 否则判定时锚点窗口里会出现空洞
    cnn_detector_push(in);

    const SensorData& s = in.local;
    float svm = fminf(s.svm, CLIP_ACC);
    float gm  = fminf(sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz), CLIP_GYR);

    // 姿态 IIR（同训练 alpha=0.03）
    if (!lpInit) { lp[0] = s.ax; lp[1] = s.ay; lp[2] = s.az; lpInit = true; }
    else {
        lp[0] += TILT_ALPHA * (s.ax - lp[0]);
        lp[1] += TILT_ALPHA * (s.ay - lp[1]);
        lp[2] += TILT_ALPHA * (s.az - lp[2]);
    }
    float gmag_lp = sqrtf(lp[0] * lp[0] + lp[1] * lp[1] + lp[2] * lp[2]);
    if (gmag_lp < 1e-6f) gmag_lp = 1e-6f;
    float tilt = degrees(acosf(constrain(lp[2] / gmag_lp, -1.0f, 1.0f)));
    float dtilt = fabsf(tilt - baselineTilt);

    // 腕端样本：链路新鲜才有效，否则记 -1（丢包/离线的洞）。
    // 注意用 remoteAgeMs（板间时钟不同步，不能用对端 ts）
    float wsvm = (in.hasRemote && in.remoteAgeMs < WRIST_FRESH_MS)
                 ? fminf(in.remote.svm, CLIP_ACC) : -1.0f;

    switch (st) {
    case ST_BASELINE: {
        if (baseN < (int)(sizeof(baseBuf) / sizeof(baseBuf[0])))
            baseBuf[baseN++] = tilt;
        if (baseN * 1000 / FS >= FD_BASELINE_MS) {
            baselineTilt = medianOf(baseBuf, 0, baseN);
            st = ST_IDLE;
            LOG_I("DETC", "baseline tilt=%.1fdeg, detector armed", baselineTilt);
        }
        break;
    }
    case ST_IDLE: {
        rSvm[rHead] = svm; rDt[rHead] = dtilt; rGm[rHead] = gm;
        rWsvm[rHead] = wsvm;
        rHead = (rHead + 1) % RING_N;
        if (rCnt < RING_N) rCnt++;
        if (svm > FD_TRIGGER_SVM || gm > FD_TRIGGER_GYR) {
            // 环形 -> 事件缓冲（按时间顺序展开）
            eN = 0;
            for (int k = rCnt; k > 0; k--) {
                int idx = (rHead - k + RING_N) % RING_N;
                eSvm[eN] = rSvm[idx]; eDt[eN] = rDt[idx]; eGm[eN] = rGm[idx];
                eWsvm[eN] = rWsvm[idx];
                eN++;
            }
            trigMs = millis();
            st = ST_COLLECT;
            LOG_I("DETC", "impact trigger svm=%.1f gyr=%.1f", svm, gm);
        }
        break;
    }
    case ST_COLLECT: {
        if (eN < EV_N) { eSvm[eN] = svm; eDt[eN] = dtilt; eGm[eN] = gm;
                         eWsvm[eN] = wsvm; eN++; }
        if (millis() - trigMs >= FD_COLLECT_MS || eN >= EV_N) decide();
        break;
    }
    case ST_LATCHED:
        break;   // 等 fall_detector_reset()
    }

    if (pendingConfirm) {
        pendingConfirm = false;
        return FALL_CONFIRMED;
    }
    return FALL_NONE;
}

void fall_detector_reset(void) {
    st = ST_BASELINE;
    baseN = 0;
    rCnt = 0; rHead = 0;
    eN = 0;
    pendingConfirm = false;
    dbg.valid = false;
    cnn_detector_reset();   // 历史清空；随后 5s 基线期足以重新填满仲裁窗
    LOG_I("DETC", "reset -> re-baseline");
}

const FallDebug& fall_detector_debug(void) { return dbg; }

// ================= 中位数（插入排序，窗口 <=150 点足够快） =================
static float medianOf(const float* a, int i0, int i1) {
    // static 缓冲：避免 2.4KB 压栈（task_detect 栈仅 4KB）；
    // 本函数仅被 task_detect 单线程调用（判定/基线），无重入风险
    static float tmp[600];
    int n = i1 - i0;
    if (n > 600) n = 600;
    for (int i = 0; i < n; i++) tmp[i] = a[i0 + i];
    for (int i = 1; i < n; i++) {          // 插入排序
        float v = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    return (n & 1) ? tmp[n / 2] : (tmp[n / 2 - 1] + tmp[n / 2]) / 2;
}
