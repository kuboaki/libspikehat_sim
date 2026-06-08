#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

/* ポート構成:
 *   A(0): (未使用 or モーター)
 *   B(1): フォースセンサー
 *   C(2): カラーセンサー
 *   D(3): 距離センサー
 */
int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) return 1;

    spikehat_port_config(hat, 1, SPIKEHAT_DEVICE_FORCE);
    spikehat_port_config(hat, 2, SPIKEHAT_DEVICE_COLOR);
    spikehat_port_config(hat, 3, SPIKEHAT_DEVICE_DISTANCE);
    sleep(2);

    printf("=== センサーテスト (10回) ===\n");
    for (int i = 0; i < 10; i++) {
        int mm = -1, hue = 0, sat = 0, val = 0, force = 0, pressed = 0;

        if (spikehat_distance_read(hat, 3, &mm) == 0)
            printf("距離: %4d mm  ", mm);
        else
            printf("距離: ----    ");

        if (spikehat_color_read_hsv(hat, 2, &hue, &sat, &val) == 0)
            printf("色(HSV): %3d/%3d/%3d  ", hue, sat, val);
        else
            printf("色: --------    ");

        if (spikehat_force_read(hat, 1, &force, &pressed) == 0)
            printf("力: %2d N  %s", force, pressed ? "[押下]" : "      ");
        else
            printf("力: ----         ");

        printf("\n");
        sleep(1);
    }

    spikehat_close(hat);
    return 0;
}
