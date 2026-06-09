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
 * フォースセンサーの戻り値:
 *   force   : 力 [N]（0〜10）
 *   pressed : 押下状態（0=未押下, 1=押下）
 */
#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

#define PORT_FORCE 1   /* ポートB */

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) { fprintf(stderr, "spikehat_open failed\n"); return 1; }

    spikehat_port_config(hat, PORT_FORCE, SPIKEHAT_DEVICE_FORCE);
    sleep(1);

    printf("=== フォースセンサーテスト (20回) ===\n");
    printf("センサーを押して離してみてください\n\n");

    int prev_force = -1, prev_pressed = -1;
    int count = 0;
    while (count < 2000) {
        int force = 0, pressed = 0;
        if (spikehat_force_read(hat, PORT_FORCE, &force, &pressed) == 0) {
            if (force != prev_force || pressed != prev_pressed) {
                printf("force=%3d N  pressed=%d  %s\n",
                       force, pressed, pressed ? "[押下]" : "");
                prev_force   = force;
                prev_pressed = pressed;
                count++;
            }
        }
        usleep(10000);  /* 10ms */
    }

    spikehat_close(hat);
    return 0;
}
