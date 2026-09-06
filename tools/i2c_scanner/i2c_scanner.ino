// ============================================================
//  I2C 多引脚扫描器 v2 — 诊断 XIAO 焊点/插针位置问题
//  原理：依次在各候选引脚组上初始化 I2C 并扫描，
//        你的线插在哪组、哪组通，一轮全知道
//
//  使用：烧录后串口 115200，每轮约 4 秒，输出形如：
//    [scan] SDA=GPIO5(D4) SCL=GPIO6(D5): (no device)
//    [scan] SDA=GPIO7(D8) SCL=GPIO8(D9): found 0x68 <- MPU6050
//  判读：
//    某组 found 0x68 → 线插在那组脚上且通路正常！
//      → 把这个结果告诉组长，改固件引脚或挪线到 D4/D5
//    全部 (no device) → 物理断路：XIAO 焊点虚焊或线没插进孔，
//      → 用万用表通断档量 XIAO 焊盘到 MPU 针脚，或补焊
// ============================================================
#include <Wire.h>

// 候选引脚组：{SDA, SCL}（正反接都覆盖；不含 D6/D7 串口脚）
// GPIO→丝印：1=D0 2=D1 3=D2 4=D3 5=D4 6=D5 7=D8 8=D9 9=D10
static const int PAIRS[][2] = {
    {5, 6},   // D4/D5 官方 I2C 位
    {6, 5},   // D5/D4 反接
    {7, 8},   // D8/D9
    {8, 7},   // D9/D8
    {1, 2},   // D0/D1
    {2, 1},   // D1/D0
    {3, 4},   // D2/D3
    {4, 3},   // D3/D2
};
static const char* TAGS[][2] = {
    {"D4", "D5"}, {"D5", "D4"}, {"D8", "D9"}, {"D9", "D8"},
    {"D0", "D1"}, {"D1", "D0"}, {"D2", "D3"}, {"D3", "D2"},
};

void scanOne(int sda, int scl, const char* tSda, const char* tScl) {
    Wire.end();
    Wire.begin(sda, scl);
    Serial.printf("[scan] SDA=GPIO%d(%s) SCL=GPIO%d(%s): ", sda, tSda, scl, tScl);

    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            if (found == 0) Serial.println();   // 首个器件换行对齐
            Serial.printf("       found 0x%02X", addr);
            if (addr == 0x68) Serial.print(" <- MPU6050");
            if (addr == 0x69) Serial.print(" <- MPU6050(AD0高)");
            if (addr == 0x3C || addr == 0x3D) Serial.print(" <- OLED");
            Serial.println();
            found++;
            delay(5);
        }
    }
    if (found == 0) Serial.println("(no device)");
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("[scan] === multi-pin I2C scanner v2 ===");
    Serial.println("[scan] 把 MPU 接好再烧录，每 4 秒出一轮全引脚扫描");
}

void loop() {
    Serial.println("[scan] ---- round ----");
    for (uint8_t i = 0; i < sizeof(PAIRS) / sizeof(PAIRS[0]); i++) {
        scanOne(PAIRS[i][0], PAIRS[i][1], TAGS[i][0], TAGS[i][1]);
    }
    delay(3000);
}
