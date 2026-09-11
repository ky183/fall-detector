// ============================================================
//  main.cpp —— CNN 二级仲裁 宿主机回放驱动（不参与固件编译）
//
//  由 algo/cnn/host_replay/replay.py 编译并驱动：
//  把一串 50Hz 采样喂给**真实的** cnn_detector.cpp，再要一次仲裁结果。
//  用途：与 Python 参考实现逐条对比，作为移植正确性的硬验收
//        （与随机森林侧 algo/05_export.py 的双语言回放同性质）。
//
//  输入格式（stdin，可连续多个用例，读到 EOF 为止）：
//      <n> <ageSamples>
//      <12 floats: 腰acc3 腰gyr3 腕acc3 腕gyr3> <hasRemote 0/1> <remoteAgeMs>
//      ... 共 n 行
//  输出格式（stdout，每个用例一行）：
//      <ran 0/1> <prob> <coverage>
// ============================================================
#include "cnn_detector.h"
#include "config.h"          // 用真实配置里的 CNN_MIN_COVERAGE 等参数
#include "Arduino.h"         // 桩：Serial / millis / GPIO（inline 变量，无需再定义）

#include <stdio.h>
#include <stdlib.h>

int main(void) {
    int n = 0, age = 0;
    while (scanf("%d %d", &n, &age) == 2) {
        cnn_detector_reset();              // 每个用例相互独立（同 fall_detector_reset 语义）
        for (int i = 0; i < n; ++i) {
            FallInput in = {};
            int hasRemote = 0;
            unsigned ageMs = 0;
            if (scanf("%f %f %f %f %f %f %f %f %f %f %f %f %d %u",
                      &in.local.ax, &in.local.ay, &in.local.az,
                      &in.local.gx, &in.local.gy, &in.local.gz,
                      &in.remote.ax, &in.remote.ay, &in.remote.az,
                      &in.remote.gx, &in.remote.gy, &in.remote.gz,
                      &hasRemote, &ageMs) != 14) {
                fprintf(stderr, "bad input at sample %d\n", i);
                return 2;
            }
            in.hasRemote   = (hasRemote != 0);
            in.remoteAgeMs = ageMs;
            cnn_detector_push(in);
        }
        float p = -1.0f, cov = 0.0f;
        bool ok = cnn_detector_prob(age, CNN_MIN_COVERAGE, &p, &cov);
        printf("%d %.9g %.9g\n", ok ? 1 : 0, (double)p, (double)cov);
        fflush(stdout);
    }
    return 0;
}
