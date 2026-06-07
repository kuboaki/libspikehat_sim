#include <stdio.h>
#include <unistd.h>
#include "spikehat.h"

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) return 1;

    /* ポートA(0)にMアンギュラーモーターを設定 */
    spikehat_port_config(hat, 0, SPIKEHAT_DEVICE_MOTOR_M);
    sleep(1);

    printf("=== モーターテスト ===\n");

    /* 速度5で2秒間回転 */
    printf("速度5で2秒間回転...\n");
    spikehat_motor_run_for_seconds(hat, 0, 2.0f, 5);
    sleep(4);

    /* 現在の速度・位置を表示 */
    int speed = 0, pos = 0;
    if (spikehat_motor_get_speed(hat, 0, &speed) == 0)
        printf("速度: %d\n", speed);
    if (spikehat_motor_get_position(hat, 0, &pos) == 0)
        printf("位置: %d 度\n", pos);

    /* 逆方向に速度3で2秒間 */
    printf("速度-3で2秒間回転...\n");
    spikehat_motor_run_for_seconds(hat, 0, 2.0f, -3);
    sleep(3);

    spikehat_motor_coast(hat, 0);
    printf("完了\n");

    spikehat_close(hat);
    return 0;
}
