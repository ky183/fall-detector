# 交接文档 — 2026-09-10 · 算法上板 v0.3

> 写给后续开发者（人或 AI）：本文档记录截至 2026-09-10 晚的项目全貌。
> 阅读顺序建议：§1 现状 → §3 已验证/未验证 → §4 当前阻塞 → §6 待办。
> 代码内的权威注释：`fall_detector.cpp` 头部有"训练↔固件对应关系表"，改任何一侧先看它。

## 0. 一句话状态

跌倒判定算法已完成"离线训练 → C++ 移植 → 上板运行 → 微信推送"全链路；
腰端已在真机上跑通判定与推送；**当前阻塞在硬件层（供电 ≥6.65V 未落实）**，
佩戴测试与腕端融合标定未开始。

## 1. 系统当前形态

```
腕端 XIAO ESP32S3 ──ESP-NOW 50Hz ch6──> 腰端 ESP32-S3 大板
  MPU6500(±8g/±500°/s)                   MPU6500#2 + 蜂鸣器 + 取消按钮 + WiFi
  OLED + 按钮(长按=模拟跌倒)              ├ 三阶段触发 + 15树随机森林判定
  （腕端只发原始数据，不做判定）           ├ 报警状态机(10s取消窗) → WxPusher微信
                                         └ WiFi常连模式(与ESP-NOW同信道共存)
```

| 模块 | 状态 | 说明 |
|---|---|---|
| 双板采集 + ESP-NOW | ✅ 稳定 | 常连共存下丢包 ~7%，lastAge 稳定个位数 ms |
| 腰端判定算法 | ✅ 已上板 | 详见 §2 |
| 报警/取消/推送 | ✅ 实测通过 | 摔倒→微信 ~17s（3.5s判定+10s取消窗+4s POST）|
| 腕端融合 v1 | ⚠️ 代码就绪未验证 | `ENABLE_WRIST_FUSION=0`，等待腕端链路+自采数据标定 |
| 佩戴测试 | ❌ 未开始 | 弹力腰带未到货 |
| 供电 | ❌ **阻塞** | 大板需 ≥6.65V，详见 §4 |

## 2. 算法部分（algo/ 目录）

### 2.1 数据集与管线

- 数据集：**SisFall**（38 人 / 4505 文件 / 1798 次跌倒，腰部佩戴，200Hz）
  - 下载位置：`algo/data/sisfall/`（gitignore，组员自行下载）
  - 下载方式：HuggingFace 镜像 `hf-mirror.com/datasets/Trupal7/Sisfall_Dataset`
    （直连 hf.co 被墙；zip 232MB，解压后即 SisFall_dataset 目录）
- 管线脚本（按序号执行，各带 `check` 自检模式）：
  1. `sisfall.py` — 加载器：位值→物理单位、200→50Hz 块平均降采样（=固件采样率，
     训练域=部署域）、姿态角（低通 IIR α=0.03，与固件同参）
  2. `01_explore.py` — 可视化证明"三阶段判决缺一不可"（答辩素材 `out/explore.png`）
  3. `02_features.py` — **触发锚定**特征提取（不是全文件滑窗，防泄漏），12 特征，
     5920 样本。v2 关键点：**量程裁剪 CLIP_ACC=±8g / CLIP_GYR=±500°/s**（数据集
     传感器 ±16g/±2000°/s，不裁剪则模型学到硬件达不到的规则，上板失效）
  4. `03_train.py` — 训练+严格评估（见 2.2）
  5. `04_threshold.py` — 阈值基线 + 四方法对比 + 参数标定（只在训练集标定）
  6. `05_export.py` — 训练 15树小森林 → 生成 `firmware/waist_firmware/rf_model.h`
     （自动生成勿手改）+ **双语言一致性回放**（C 遍历逻辑的 Python 镜像 vs sklearn，
     5920 样本判决 0 不一致才允许上板）

### 2.2 关键指标（答辩数字，全部来自"模型没见过的人"）

| 方法 | 留出集灵敏 | 留出集特异 | 老年日常误报 | SE06老人真摔检出 |
|---|---|---|---|---|
| 阈值法（三阶段AND） | 72.8% | 99.3% | 3.8% | 76.0% |
| 随机森林 300 树 | 97.9% | 97.9% | 6.7% | 88.0% |
| **小随机森林 15树×深6（上板方案）** | **97.1%** | **97.5%** | 6.9% | 89.3% |
| 小森林@门限0.4（部署点） | 96.0% | 98.4% | 4.6% | 85.3% |

- 评估方法论（答辩重点）：按"人"划分（SA04/09/14/19/23 留出）；老年组不进训练、
  专门做泛化压力测试；阈值参数只在训练集标定后冻结
- 已知弱点（诚实记录）：坐姿晕倒型跌倒（F13/F14/F15）漏报集中；体重最轻受试者
  的"软摔"漏报；改进思路是自适应基线，评估为高风险暂缓

### 2.3 上板实现（腰端固件）

- `fall_detector.cpp`：状态机 BASELINE(开机站5s采基线)→IDLE(触发: SVM>30 或
  |ω|>200)→COLLECT(3.5s确认窗)→判决→LATCHED(锁存到报警取消)。特征窗口定义
  与训练侧逐条对应（注释里有对照表）
- `rf_model.h`：1083 节点 ≈17KB 常量，15 树遍历
- 判决日志格式：
  `[DETC] decision proba=0.47 peak=78.5 tilt=1deg gyr=500 wrist=11.2/cov98% (3501ms)`
  （wrist 字段无论融合开关都输出，是标定融合阈值的数据来源）

## 3. 已验证 vs 未验证（接手必读）

**已在真机验证：**
- 判定→报警→取消→微信推送全链路（多次，含 code=1000 成功回执）
- WiFi 常连 + ESP-NOW 同信道共存（POST 期间 rx 仍增长）
- 判定管线"正确拒绝"能力（强冲击+无姿态变化 → proba=0.07 拒绝）

**未验证（下次继续的点）：**
1. **腕端数据进判决日志**：实测时腕端链路已死，`wrist=-1.0/cov0%`。
   验证标准：腕端开机+链路正常时，decision 行出现 `wrist=9~12/cov>90%`
2. 佩戴状态下的完整测试协议（A 阴性/B 阳性，见下）
3. 融合否决票的实际效果（阈值是工程初值，需自采数据标定）

## 4. 当前头号问题：供电（未解决，其他问题的疑似总根源）

**大板输入必须 ≥6.65V**（板载 DC-DC 最小输入；5V 充电宝实测带不动 WiFi）。
2026-09-10 晚出现的一组症状统一指向供电不稳：
- 无人触碰时 SVM 恒 ~41 m/s²（4.2g 假读数）反复触发
- MPU 反复 offline（时间点与 WiFi 活动窗重合）
- WiFi 连接失败、腕端链路大量丢包

**待办**：PD 诱骗线（充电宝→9V→DC5521，10~20元）或 9V/12V DC 适配器。
换正规供电后第一步：静置观察 SVM 是否回到 ~9.8、trigger 风暴是否消失。
（腕端 XIAO 无此问题，5V USB 正常。）

## 5. 已知设计内行为与使用纪律（不是 bug，答辩可能被问）

1. **摘戴/摆弄装置 = 伪跌倒信号**（姿态大变+静止）。纪律：先佩戴→再上电；
   先关机→再摘下。误触发有 10s 取消窗兜底。腕端否决票正式启用后可拦大部分
2. **MPU 朝向**：绝对朝向无所谓（基线相对），但推荐板面平贴腰前（与 SisFall
   训练一致）。上电后改变朝向会被视为跌倒
3. **基线假设开机站立**：报警取消后会重新采基线，需站直 5 秒等 `detector armed`
4. 推送时间线 ~17s，可调 `ALARM_CANCEL_WINDOW_MS`（不建议低于 6s）

## 6. 待办清单（优先级）

1. 【阻塞】解决供电（PD 诱骗线/适配器）→ 复测 SVM 稳定性
2. MPU 四焊点补焊（VCC/GND/SDA/SCL）+ 杜邦线固定（佩戴装备到货前的常规加固）
3. 腕端链路验证（§3.1 的标准）
4. 佩戴装备到货 → 测试协议：
   - A 阴性：走30s/快坐×3/跳×3/弯腰×3/下蹲×3，合格=零报警（decision proba<0.4 允许）
   - B 阳性：垫上侧摔/前扑/后倒 ×3，合格=≥7/9 报警；每轮取消后站直5s
   - 记录所有 decision 行（proba + wrist 值）= 第一批自采标定数据
5. 用自采数据标定融合阈值 → `ENABLE_WRIST_FUSION=1`
6. 数据采集工具（腕腰两端 DEBUG_DATA CSV 输出）→ 域差距量化
7. （可选）"自愈检测"创新点、第二数据集交叉验证、起跳类特征增强

## 7. 新环境复现指南（组员从零跑通）

```powershell
git clone https://github.com/ky183/fall-detector.git
cd fall-detector
# 数据集（232MB，需 Python: numpy/pandas/scipy/matplotlib/scikit-learn）
#   从 hf-mirror.com/datasets/Trupal7/Sisfall_Dataset 下载 zip
#   解压到 algo/data/sisfall/SisFall_dataset
python algo\02_features.py check     # 自检
python algo\02_features.py all       # 全量特征（~2分钟）
python algo\03_train.py              # 训练+评估报告
python algo\05_export.py             # 重新生成 rf_model.h + 一致性回放
# 固件：Arduino IDE 打开 firmware/waist_firmware（板: ESP32S3 Dev Module,
#   USB CDC on Boot=Disabled, 烧录口 Micro USB），secrets.h 照
#   secrets.h.example 填。腰端供电必须 ≥6.65V！
```

## 8. 关键参数速查（全部在腰端 config.h，有注释）

| 参数 | 值 | 含义 |
|---|---|---|
| ESPNOW_CHANNEL | 6 | 两端一致；常连模式下还须=热点信道（失配有串口告警）|
| FD_TRIGGER_SVM / GYR | 30 m/s² / 200°/s | 冲击触发（满足其一）|
| FD_RF_THRESHOLD | 0.40 | 森林判决门限（部署点指标见 §2.2）|
| FD_BASELINE_MS / COLLECT_MS | 5000 / 3500 | 基线时长 / 确认窗时长 |
| ENABLE_WRIST_FUSION | 0 | 腕端否决票（标定后开）|
| FUSION_VETO_* | 0.6 / 30 / 12 | 覆盖率/腰端强冲击/腕端安静阈值 |
| PUSH_ALWAYS_ON | 1 | WiFi 常连模式（0=按需连接）|
| PUSH_MAX_ATTEMPTS | 6 | 推送重试 6 次×15s，不再丢弃 |
| ALARM_CANCEL_WINDOW_MS | 10000 | 报警取消窗 |

## 9. 文件索引

| 路径 | 内容 |
|---|---|
| `algo/` | 离线管线（§2.1），`data/`和`reference/`已gitignore |
| `firmware/waist_firmware/rf_model.h` | 森林常量表（05_export.py 生成）|
| `firmware/waist_firmware/fall_detector.*` | 判定算法（头注释=对应关系表）|
| `firmware/waist_firmware/net_pusher.cpp` | 推送（常连/按需双模式）|
| `algo/out/train_report.txt` | 训练评估报告留档 |
| `tools/` | i2c_scanner / btn_test / wifi_test 排障工具 |
| `docs/notes/mpu6500_driver_note.md` | MPU6500 驱动踩坑记录 |

---
*v0.3 tag：`git tag -a v0.3`（若提交时遗漏）。文档随代码一起 commit。*
