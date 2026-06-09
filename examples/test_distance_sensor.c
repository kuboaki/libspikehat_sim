#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

/*
 * test_distance_sensor.c — 距離センサーテスト
 *
 * ポート構成:
 *   D(3): 距離センサー
 *
 * MuJoCo ビューアで wall_slide_ctrl スライダーを動かすと
 * 壁が前後に移動し、距離センサーの値が変わる。
 *
 * 有効距離: 50mm〜300mm
 * 範囲外または測定不能: 2000mm (DIST_INVALID)
 *
 * 実行方法:
 *   SPIKEHAT_SIM_XML=examples/test_distance_sensor.xml \
 *     ./build/test_distance_sensor
 */

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) return 1;

    spikehat_port_config(hat, 3, SPIKEHAT_DEVICE_DISTANCE);
    spikehat_sleep(hat, 1.0f);

    printf("=== 距離センサーテスト (20回) ===\n");
    printf("MuJoCoビューアで wall_slide_ctrl を動かして壁を移動してください\n");
    printf("有効距離: 50mm〜300mm\n\n");

    for (int i = 0; i < 20; i++) {
        int mm = -1;

        if (spikehat_distance_read(hat, 3, &mm) == 0) {
            if (mm == 2000)
                printf("[%2d] 距離: ---- mm (範囲外または測定不能)\n", i + 1);
            else
                printf("[%2d] 距離: %4d mm\n", i + 1, mm);
        } else {
            printf("[%2d] 距離: 読み取り失敗\n", i + 1);
        }

        spikehat_sleep(hat, 1.0f);
    }

    spikehat_close(hat);
    return 0;
}
