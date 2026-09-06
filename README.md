# 智能跌倒检测报警器

清华大学第29届硬件设计大赛 · 民生赛道

面向**独居老人**的可穿戴跌倒检测报警器：手腕端采集姿态，腰部融合判断，跌倒时蜂鸣器报警 + OLED 显示 + 按钮取消 + WxPusher 微信推送。

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
├── tools/                  # 工具脚本（烧录脚本等）
├── assets/                 # 演示素材（视频/图片）
└── test/                   # 测试记录
    └── logs/               # 测试日志
```

## 引脚分配

**腰端（ESP32-S3）**

| 功能 | 引脚 |
|------|------|
| I2C SDA（MPU6050#2 + OLED） | GPIO8 |
| I2C SCL | GPIO9 |
| 蜂鸣器（S8050 驱动） | GPIO4 |
| 按钮1（取消报警） | GPIO5 |
| 按钮2（唤醒/自检） | GPIO6 |

**腕端（XIAO ESP32S3）**

| 功能 | 引脚 |
|------|------|
| I2C SDA（MPU6050#1） | D4（GPIO5） |
| I2C SCL | D5（GPIO6） |
| 电池 | BAT+ / BAT- 焊盘 |

## 环境要求

- Arduino IDE 2.x
- ESP32 board package **3.x**（自带 XIAO_ESP32S3 板型，无需另装 Seeed 包）
- 库：Adafruit_GFX、Adafruit_SSD1306、Adafruit_MPU6050、Adafruit_BusIO、Adafruit_Sensor

## 快速开始

1. 打开 `firmware/waist_firmware/waist_firmware.ino`，板子选 **ESP32S3 Dev Module**，烧录到腰端。
2. 打开 `firmware/wrist_firmware/wrist_firmware.ino`，板子选 **XIAO_ESP32S3**，烧录到腕端。
3. 串口波特率 `115200`。

## 开发计划

见 `docs/plan/` 与项目规划书（Step 0~7，两周）。
