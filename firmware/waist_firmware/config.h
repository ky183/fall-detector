#pragma once
// ============================================================
//  腰端全局配置 — ESP32-S3 大板（ESP32-S3-WROOM-1-N16R8 课程板）
//  所有引脚、参数、功能开关集中于此，改一处全局生效
//  分段调试原则：哪个模块没接好/没到货，把对应 ENABLE 置 0 即可
// ============================================================

// ================= 硬件引脚（大板排针 H4） =================
#define PIN_I2C_SDA         8       // MPU6050#2，排针 H4 第12脚，开漏需上拉
#define PIN_I2C_SCL         9       // 排针 H4 第15脚
#define PIN_BUZZER          4       // 无源蜂鸣器（经 S8050 驱动，基极串 1kΩ），H4 第4脚
#define PIN_BTN_CANCEL      5       // 取消报警按钮，H4 第5脚，另一端接 GND

// ================= 大板注意事项（烧录/供电） =================
// 烧录口：Micro USB（CH340 串口），不是 Type-C
// 供电：Type-C/DC5521，输入必须 ≥6.65V，5V 充电宝直连 Type-C 无法启动
// 烧录选项：板选 ESP32S3 Dev Module，USB CDC in Boot = Disable

// ================= 设备 I2C 地址 =================
#define ADDR_MPU6050        0x68    // 腰端 MPU（实测可能是 MPU6500，驱动兼容）

// ================= ESP-NOW 通信 =================
// ★常连模式(PUSH_ALWAYS_ON=1)约束：必须与热点信道一致（单射频只有一个信道）！
//   上次实测 vivo 热点在 ch6。串口看 "WiFi connected ... ch=X"，
//   若 X != 本值会有 [PUSH] !! 告警 —— 按实际信道改这里和腕端 config.h
#define ESPNOW_CHANNEL      6       // 与腕端一致（有效范围 1~13）
// 腕端 MAC：当前不做过滤（按包内 devId 过滤），联调稳定后可改单播

// ================= 报警参数 =================
#define ALARM_CANCEL_WINDOW_MS   10000   // 触发报警后的取消窗口（毫秒）
#define BUZZER_FREQ_HZ           2700    // 无源蜂鸣器谐振频率（响度最大）
#define ALARM_BEEP_ON_MS         300     // 哔声时长（间歇模式）
#define ALARM_BEEP_PERIOD_MS     500     // 哔声周期（300 响 + 200 停）

// ================= 采样与任务参数 =================
#define SAMPLE_HZ           50      // 本地 MPU 采样率
#define BUTTON_POLL_MS      50      // 按钮扫描周期
#define ALARM_TICK_MS       100     // 报警状态机步进周期
#define STAT_PERIOD_MS      10000   // 链路统计打印周期：联调阶段可改 1000 看实时丢包，
                                    // 平时 10 秒一条即可；想完全关闭把 LOG_LEVEL 改 2

// ================= 分级调试开关 =================
#define LOG_LEVEL           3       // 0=关 1=错误 2=信息 3=调试

#define ENABLE_MPU6050      1       // 0 = 不读本地 MPU（只收腕端数据调试链路）
#define ENABLE_ESPNOW       1       // 0 = 不初始化无线（单板调试蜂鸣器/按钮）
#define ENABLE_BUZZER       0       // 0 = 不响（调试时防止吵）
#define ENABLE_BUTTON       1       // 0 = 不扫描按钮

#define ENABLE_FALL_DETECT  1       // ★跌倒判定算法（已实现：三阶段触发+小随机森林）
                                    // 0 = 未启用，仅腕端"长按模拟跌倒"可触发报警（联调用）
                                    // 1 = 启用板上算法（上板验证通过后打开）
// ---- 算法参数（由 algo/04_threshold.py 在训练集上标定，改动需重评）----
#define FD_TRIGGER_SVM      30.0f   // 冲击触发阈值 m/s²（≈3.1g）
#define FD_TRIGGER_GYR      200.0f  // 角速度触发阈值 °/s（与 SVM 满足其一即触发）
#define FD_RF_THRESHOLD     0.40f   // 随机森林判决门限：灵敏96% 特异98.4% 老年FP4.6%
                                    // （调低更灵敏误报升；自采数据验证后可微调）
#define FD_BASELINE_MS      5000    // 开机站姿基线采集时长（期间不判定，需保持站立）
#define FD_COLLECT_MS       3500    // 触发后确认窗采集时长（峰值+3s 确认窗）

// ---- 腕端融合 v1：否决票（自采数据验证标定后再启用）----
// 逻辑：腕端链路健康 && 腰端强冲击 && 腕端几乎没动 -> 判为"装置被磕碰"否决报警。
// 三个阈值均为工程初值，待自采数据回放标定（wrist 数据无论开关都记录在串口）
#define ENABLE_WRIST_FUSION      0       // 1=启用腕端否决票（腕端需开机且链路正常）
#define FUSION_MIN_COVERAGE      0.6f    // 腕端冲击窗数据覆盖率下限（低于=数据不可信，不否决）
#define FUSION_VETO_WAIST_STRONG 30.0f   // 腰端冲击峰值下限 m/s²（约3g）
#define FUSION_VETO_WRIST_QUIET  12.0f   // 腕端峰值上限 m/s²（约1.2g，静止水平）
#define WRIST_FRESH_MS           200     // 腕端数据新鲜度判据（remoteAge 小于此=有效样本）

// ---- CNN 二级仲裁（第二级判决，实现见 cnn_detector.cpp / cnn_weights.h）----
// 架构：随机森林仍是主判决（三阶段触发 -> 特征 -> RF）。
//       CNN 只在"腰端已触发冲击 + 判定时腕端链路在线"时补跑一次推理
//       （每次事件最多 1 次），不参与常态判决，CPU 开销可忽略。
//       腕端不在线 / 仲裁窗覆盖率不足 -> 直接不跑，行为完全回退到纯 RF。
// 训练/导出链路：algo/cnn/cnn_train.py -> firmware/waist_firmware/cnn_weights.h
#define ENABLE_CNN_ARBITER  1       // 1=启用 CNN 二级仲裁（0=纯 RF，与 v2 行为一致）
#define CNN_ROLE_RESCUE     0       // 1=允许 CNN 补判 RF 漏掉的跌倒（补漏报）
                                    //   ★ 2026-09-11 影子模式：当前 CNN 尚未经过
                                    //   按人划分的真实评估，不允许它改变判决结果。
                                    //   影子期只看日志（decision 后的 "cnn p=..." 行），
                                    //   攒够"RF 对/错 × CNN 高/低"对照数据后再开
#define CNN_ROLE_VETO       0       // 1=允许 CNN 否决 RF 的"跌倒"判决（压误报）
                                    //   ★ 会压低灵敏度，须用自采数据标定后再开
#define CNN_FALL_THRESHOLD  0.80f   // CNN 判"跌倒"门限（＝训练侧部署门限）
#define CNN_CALM_THRESHOLD  0.20f   // CNN 判"明确非跌倒"门限（仅否决票用）
#define CNN_MIN_COVERAGE    0.6f    // 仲裁窗腕端覆盖率下限（同 FUSION_MIN_COVERAGE 口径）

#define ENABLE_WIFI_PUSH    1       // 微信推送（已实现：队列+独立任务+WxPusher HTTPS）
                                    // 需要 secrets.h（复制 secrets.h.example 填真实值）
                                    // ★ 该文件不入库；缺失时 net_pusher 自动降级为
                                    //   占位实现并给编译告警，不会让工程编译不过
#define WIFI_TIMEOUT_MS     10000   // 推送时 WiFi 连接超时（毫秒）

// ---- 推送工作模式 ----
#define PUSH_ALWAYS_ON      1       // 1=常连模式：开机连WiFi并保持，推送即时（演示用）
                                    // 0=按需连接：报警才连，推完释放信道给ESP-NOW
                                    // 常连模式前提：热点信道 == ESPNOW_CHANNEL（见上）
#define PUSH_RECONNECT_MS   60000   // 常连模式掉线后的静默重连间隔
#define PUSH_MAX_ATTEMPTS   6       // 推送失败重试次数（间隔15s，补救窗口约90s+）

// ================= 推送文案（改这里即可，保持 UTF-8 编码） =================
// %lu 位置会替换成"报警后经过的秒数"
#define PUSH_SUMMARY        "跌倒报警"                              // 微信消息列表里的标题
#define PUSH_CONTENT_HEAD   "警报：检测到佩戴者跌倒，已持续 "          // 正文前半
#define PUSH_CONTENT_TAIL   " 秒未取消！请立即联系老人确认情况。"      // 正事后半

// ================= 系统参数 =================
#define SERIAL_BAUD         115200
