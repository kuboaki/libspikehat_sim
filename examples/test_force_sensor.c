/**
 * test_force_sensor.c — フォースセンサーテスト（C版）
 *
 * 実行方法:
 *   SPIKEHAT_SIM_XML=examples/test_force_sensor.xml ./build/test_force_sensor
 */
#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) { fprintf(stderr, "spikehat_open failed\n"); return 1; }

    printf("フォースセンサーテスト開始（Ctrl+C で終了）\n");
    printf("press_ctrl スライダーを操作してください\n\n");

    int prev_force = -1;
    for (int i = 0; i < 5000; i++) {
        int force = 0, pressed = 0;
        spikehat_force_read(hat, 0, &force, &pressed);
        if (force != prev_force) {
            printf("force=%4d N  pressed=%d\n", force, pressed);
            prev_force = force;
        }
        usleep(10000);
    }

    spikehat_close(hat);
    return 0;
}
