#include <stdio.h>
#include <math.h>
#include "spikehat.h"

#define PORT_MOTOR   0   /* ポートA: Mアンギュラーモーター */
#define ALIGN_SPEED 10   /* 初期位置合わせの速度 */
#define RUN_SPEED    5   /* motor_start テストの速度 */

/* 現在位置から原点(0度)へ戻す（初期位置合わせ） */
static void return_to_origin(spikehat_t *hat, int port) {
    int cur_pos = 0;
    spikehat_motor_get_position(hat, port, &cur_pos);
    if (cur_pos == 0) return;

    printf("初期位置合わせ: 現在位置 %d 度 -> 0 度\n", cur_pos);
    spikehat_motor_run_for_degrees(hat, port, -cur_pos, ALIGN_SPEED);

    double dur = (fabs((double)cur_pos) / 360.0) / (ALIGN_SPEED * 0.05);
    if (dur < 0.5) dur = 0.5;
    spikehat_sleep(hat, (float)(dur + 0.5));
}

static void print_status(spikehat_t *hat, int port) {
    int speed = 0, pos = 0;
    if (spikehat_motor_get_speed(hat, port, &speed) == 0)
        printf("速度: %d\n", speed);
    if (spikehat_motor_get_position(hat, port, &pos) == 0)
        printf("位置: %d 度\n", pos);
}

int main(void) {
    spikehat_t *hat = spikehat_open(SPIKEHAT_SERIAL_DEFAULT);
    if (!hat) return 1;

    /* ポートA(0)にMアンギュラーモーターを設定 */
    spikehat_port_config(hat, PORT_MOTOR, SPIKEHAT_DEVICE_MOTOR_M);
    spikehat_sleep(hat, 1.0f);

    /* ── run_for_seconds (PID速度制御 + 時間指定) ──────────── */
    printf("=== モーターテスト (run_for_seconds) ===\n");
    return_to_origin(hat, PORT_MOTOR);

    printf("速度5で2秒間回転...\n");
    spikehat_motor_run_for_seconds(hat, PORT_MOTOR, 2.0f, 5);
    spikehat_sleep(hat, 10.0f);
    print_status(hat, PORT_MOTOR);

    printf("速度-3で2秒間回転...\n");
    spikehat_motor_run_for_seconds(hat, PORT_MOTOR, 2.0f, -3);
    spikehat_sleep(hat, 10.0f);
    print_status(hat, PORT_MOTOR);

    spikehat_motor_coast(hat, PORT_MOTOR);

    /* ── motor_start + motor_stop ──────────────────────────── */
    printf("\n=== モーターテスト (motor_start -> motor_stop) ===\n");
    return_to_origin(hat, PORT_MOTOR);

    printf("速度%dでstart, 5秒後にstop...\n", RUN_SPEED);
    spikehat_motor_start(hat, PORT_MOTOR, RUN_SPEED);
    spikehat_sleep(hat, 5.0f);
    spikehat_motor_stop(hat, PORT_MOTOR);
    print_status(hat, PORT_MOTOR);

    /* ── motor_start + motor_coast ─────────────────────────── */
    printf("\n=== モーターテスト (motor_start -> motor_coast) ===\n");
    return_to_origin(hat, PORT_MOTOR);

    printf("速度%dでstart, 5秒後にcoast...\n", RUN_SPEED);
    spikehat_motor_start(hat, PORT_MOTOR, RUN_SPEED);
    spikehat_sleep(hat, 5.0f);
    spikehat_motor_coast(hat, PORT_MOTOR);
    print_status(hat, PORT_MOTOR);

    /* ── motor_pwm + motor_stop ─────────────────────────────── */
    printf("\n=== モーターテスト (motor_pwm -> motor_stop) ===\n");
    return_to_origin(hat, PORT_MOTOR);

    printf("pwm 0.1で回転, 5秒後にstop...\n");
    spikehat_motor_pwm(hat, PORT_MOTOR, 0.1f);
    spikehat_sleep(hat, 5.0f);
    spikehat_motor_stop(hat, PORT_MOTOR);
    print_status(hat, PORT_MOTOR);

    printf("\n完了\n");
    spikehat_close(hat);
    return 0;
}
