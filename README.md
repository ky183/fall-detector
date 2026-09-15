# 智能跌倒检测报警器

清华大学第29届硬件设计大赛 · 民生赛道

面向**独居老人**的可穿戴跌倒检测报警器：手腕端采集姿态，腰部融合判断，跌倒时蜂鸣器报警 + OLED 显示 + 按钮取消 + WxPusher 微信推送。

## 当前状态（v0.4 · 2026-09-13）

- ✅ 已闭环（真机实测）：采集 → ESP-NOW(50Hz, ch6) → 三阶段触发 + 随机森林判决 → 报警/取消 → WiFi 常连 + WxPusher 推送（摔倒→微信全程 ~17s）
- ✅ 算法已上板：`ENABLE_FALL_DETECT=1`；CNN 二级仲裁处于**影子模式**（只记录不生效，见下文两级判决）
- ⛔ **当前阻塞：腰端供电**——大板需 ≥6.65V（PD 诱骗线/9V 适配器待购）；供电不稳会造成 SVM 假读数反复误触发
- ⚠️ 腕端需重烧 v0.4：ESP-NOW 信道 1→6，旧固件与新腰端完全失联
- 待办顺序：供电落实复测 → 腕端重烧+链路验证 → 腕端 PCB 焊接 bring-up → 佩戴测试协议 A/B（见 `docs/handover_2026-09-10_algo_v0.3.md` §6）

## 系统架构（双板 ESP-NOW）

| 端 | 硬件 | 职责 |
|----|------|------|
| 手腕 | XIAO ESP32S3 + MPU6050#1 + 电池 | 采集姿态 → ESP-NOW 发送 |
| 腰部 | ESP32-S3 + MPU6050#2 + OLED + 蜂鸣器 + 按钮 | 融合判断 + 报警 + 显示 + WiFi 推送 |

## 目录结构

```
├── firmware/               # 固件源码
│   ├── waist_firmware/     # 腰端固件（ESP32-S3 主控）
│   └── wrist_firmware/     # 腕端固件（XIAO ESP32S3）
├── algo/                   # 算法训练/评估/导出管线（离线，PC 上跑）
│   ├── 01_explore.py       #   随机森林主判决：探索→特征→训练→门限→导出
│   │   ... 05_export.py    #   产物 -> firmware/waist_firmware/rf_model.h
│   ├── sisfall.py          #   SisFall 数据集读取/预处理公共库
│   ├── cnn/                #   CNN 二级仲裁（FallAllD 数据集）
│   │   ├── cnn_train.py    #     训练 + 导出 firmware/.../cnn_weights.h
│   │   ├── host_replay/    #     双语言一致性回放（移植正确性的硬验收）
│   │   └── legacy/         #     接入前的原型（存档，不参与编译）
│   ├── data/               #   数据集（体积大，不入库，各自下载）
│   └── out/                #   训练报告 / 导出副本（留档）
├── docs/                   # 文档
│   ├── plan/               # 规划书、开发计划
│   ├── notes/              # 开发笔记、踩坑记录
│   └── defense/            # 答辩材料
├── hardware/               # 硬件资料
│   ├── schematic/          # 原理图
│   ├── pcb/                # PCB 文件
│   ├── enclosure/          # 3D 打印外壳
│   └── bom/                # 物料清单
├── bin/                    # 烧录文件（编译好的 .bin，交付物）
│   ├── waist/              # 腰端固件烧录文件
│   └── wrist/              # 腕端固件烧录文件
├── tools/                  # 工具脚本
│   └── check_consistency.py#   全项目静态一致性自检（不需硬件/数据集）
├── assets/                 # 演示素材（视频/图片）
└── test/                   # 测试记录
    └── logs/               # 测试日志
```

## 跌倒判定算法（两级判决）

| 级 | 模块 | 训练数据 | 何时跑 | 作用 |
|----|------|----------|--------|------|
| 第一级 · 主判决 | `fall_detector.cpp` + `rf_model.h` | SisFall 38 人 | 常态 50Hz | 冲击触发 → 特征 → 小随机森林 |
| 第二级 · 二级仲裁 | `cnn_detector.cpp` + `cnn_weights.h` | FallAllD 15 人 | **仅触发后 1 次** | 用腕端 6 通道补判 / 否决 |

- 第一级按"人"留出评估（5 个全新受试者，门限 0.40 部署点）：灵敏度 96.0% / 特异度 98.4%；老年组日常误报 4.6%。四方法完整对比（阈值法/大森林/小森林/混合门控）见 `docs/handover_2026-09-10_algo_v0.3.md` §2.2。
- 第二级只在“已触发 **且** 判定时腕端在线（仲裁窗覆盖率 ≥ `CNN_MIN_COVERAGE`）”时才跑一次；
  腕端离线或覆盖不足则视同不可信，**完全不跑**，行为回退到纯 RF（即已验收的 v2 路径）。
- 角色由 `config.h` 控制：`CNN_ROLE_RESCUE`（补漏报）/ `CNN_ROLE_VETO`（压误报）。
  **两者当前均为 0 —— 影子模式（v0.4，2026-09-11）**：CNN 照常计算并记录
  （decision 日志的 `cnn p=` 行），但无权改变判决结果；等佩戴测试攒够
  "RF 对/错 × CNN 高/低"对照数据后再决定授权（理由：v2 重训后真实灵敏度
  70~78%，低于 RF 主判决 96%，不应有改变判决的权力）。
- 腕端链路一断，CNN 直接不跑；RF 主判决不依赖腕端 —— 失效开放（fail-open）设计。

### 改动算法后的验收

改完必跑，三道关都不需要硬件：

```bash
python tools/check_consistency.py       # ① 静态一致性：特征顺序/RF导出保真/板间协议/引脚
python algo/cnn/host_replay/replay.py   # ② 回放：需 g++，含算法保真/部署保真/固件集成（含报警全流程）
algo/05_export.py                       # ③ 随机森林离线指标（需 algo/data/ 数据集）
```

- ① 修的是"跨文件对不上但编译通过、离线指标也看不出"的那类接口（如特征顺序、
  量程标度、两端 protocol.h 漂移），共 22 项，全过退出码 0。
- ② 把 `firmware/` 下**真实的** `cnn_detector.cpp` / `fall_detector.cpp` /
  `alarm_manager.cpp` 等用 g++ 编到 PC 上跑，与一份独立写的 Python 参考实现
  逐条对比（判别不一致数必须为 0），并验证状态机、仲裁门控与报警全流程。
  数据集不在库时自动跳过「真实窗」项，不影响其余验收。


## 引脚分配

**腰端（ESP32-S3）**

| 功能 | 引脚 | 备注 |
|------|------|------|
| I2C SDA（MPU6050#2） | GPIO8 | 开漏，需上拉 |
| I2C SCL | GPIO9 | |
| 蜂鸣器（有源模块） | GPIO4 | 高电平=响；模块 VCC→5V；两段式节奏（取消窗内急促哔/推送后长鸣） |
| 按钮1（取消报警） | GPIO5 | 已实现 `PIN_BTN_CANCEL` |
| 按钮2（唤醒/自检） | GPIO6 | **预留**：config.h 尚未定义宏，功能未实现 |

**腕端（XIAO ESP32S3）**

| 功能 | 引脚 | 备注 |
|------|------|------|
| I2C SDA（MPU6050#1 + OLED） | D4（GPIO5） | 两者共用总线，100kHz |
| I2C SCL | D5（GPIO6） | |
| 交互按钮 | D2（GPIO3） | 短按=交互，长按=模拟跌倒；`PIN_BTN_1` |
| 电池 | BAT+ / BAT- 焊盘 | 不占 GPIO |

## 环境要求

- Arduino IDE 2.x
- ESP32 board package **3.x**（自带 XIAO_ESP32S3 板型，无需另装 Seeed 包）
- 库：Adafruit_GFX、Adafruit_SSD1306、Adafruit_BusIO、Adafruit_Sensor（MPU 为裸寄存器驱动，无需 MPU 库）

## 快速开始

1. 打开 `firmware/waist_firmware/waist_firmware.ino`，板子选 **ESP32S3 Dev Module**，烧录到腰端。
2. 打开 `firmware/wrist_firmware/wrist_firmware.ino`，板子选 **XIAO_ESP32S3**，烧录到腕端。
3. 串口波特率 `115200`。

## 开发计划

见 `docs/plan/` 与项目规划书（Step 0~7，两周）。
