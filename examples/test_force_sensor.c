/**
 * test_force_sensor.c — フォースセンサーテスト（C版）
 *
 * ポート構成:
 *   B(1): フォースセンサー
 *
 * 実機での実行方法:
 *   ./build/test_force_sensor
 *
 * シムでの実行方法:
 *   SPIKEHAT_SIM_XML=examples/test_force_sensor.xml ./build/test_force_sensor
 *
 * テスト内容:
 *   1. force_read       : force[N] と pressed を同時に取得
 *   2. force_is_pressed : タッチ判定のみ取得
 *   3. force_get_force  : 力[N]のみ取得
 */
#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

#define PORT_FORCE   1        /* ポートB */
#define LOOP_COUNT   5        /* 各テストの繰り返し回数 */
#define INTERVAL_US  500000   /* 0.5秒 */

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) { fprintf(stderr, "spikehat_open failed\n"); return 1; }

    spikehat_port_config(hat, PORT_FORCE, SPIKEHAT_DEVICE_FORCE);
    sleep(1);

    /* ── テスト1: force_read ────────────────────────────── */
    printf("=== テスト1: force_read (force と pressed を同時取得) ===\n");
    printf("センサーを押してみてください\n\n");
    for (int i = 0; i < LOOP_COUNT; i++) {
        int force = 0, pressed = 0;
        if (spikehat_force_read(hat, PORT_FORCE, &force, &pressed) == 0)
            printf("[%d] force=%3d N  pressed=%d  %s\n",
                   i + 1, force, pressed, pressed ? "[押下]" : "");
        else
            printf("[%d] 読み取り失敗\n", i + 1);
        usleep(INTERVAL_US);
    }

    /* ── テスト2: force_is_pressed ──────────────────────── */
    printf("\n=== テスト2: force_is_pressed (タッチ判定のみ) ===\n");
    printf("センサーを押してみてください\n\n");
    for (int i = 0; i < LOOP_COUNT; i++) {
        int pressed = 0;
        if (spikehat_force_is_pressed(hat, PORT_FORCE, &pressed) == 0)
            printf("[%d] pressed=%d  %s\n",
                   i + 1, pressed, pressed ? "[押下]" : "");
        else
            printf("[%d] 読み取り失敗\n", i + 1);
        usleep(INTERVAL_US);
    }

    /* ── テスト3: force_get_force ───────────────────────── */
    printf("\n=== テスト3: force_get_force (力[N]のみ) ===\n");
    printf("センサーを押してみてください\n\n");
    for (int i = 0; i < LOOP_COUNT; i++) {
        int force = 0;
        if (spikehat_force_get_force(hat, PORT_FORCE, &force) == 0)
            printf("[%d] force=%3d N\n", i + 1, force);
        else
            printf("[%d] 読み取り失敗\n", i + 1);
        usleep(INTERVAL_US);
    }

    printf("\n完了\n");
    spikehat_close(hat);
    return 0;
}
