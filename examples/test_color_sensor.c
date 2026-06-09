#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

/*
 * test_color_sensor.c — カラーセンサーテスト
 *
 * ポート構成:
 *   C(2): カラーセンサー
 *
 * MuJoCo ビューアで sensor_slide_ctrl スライダーを動かすと
 * センサーが左右に移動し、各色ブロックの上でHSV値が変わる。
 *
 * 色ブロックの配置（左から右）:
 *   赤 → 青 → 黄 → 白 → 黒 → 緑
 *
 * 実行方法:
 *   SPIKEHAT_SIM_XML=examples/test_color_sensor.xml ./build/test_color_sensor
 */

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) return 1;

    spikehat_port_config(hat, 2, SPIKEHAT_DEVICE_COLOR);
    spikehat_sleep(hat, 1.0f);

    printf("=== カラーセンサーテスト (20回) ===\n");
    printf("MuJoCoビューアで sensor_slide_ctrl を動かしてセンサーを移動してください\n");
    printf("色ブロック配置（左→右）: 赤 / 青 / 黄 / 白 / 黒 / 緑\n\n");

    for (int i = 0; i < 20; i++) {
        int hue = 0, sat = 0, val = 0;

        if (spikehat_color_read_hsv(hat, 2, &hue, &sat, &val) == 0) {
            printf("[%2d] 色(HSV): hue=%3d  sat=%3d  val=%3d\n",
                   i + 1, hue, sat, val);
        } else {
            printf("[%2d] 色: 読み取り失敗\n", i + 1);
        }

        spikehat_sleep(hat, 1.0f);
    }

    spikehat_close(hat);
    return 0;
}
