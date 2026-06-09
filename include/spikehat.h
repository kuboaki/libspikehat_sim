#ifndef SPIKEHAT_H
#define SPIKEHAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>

#define SPIKEHAT_MAX_PORTS      4
#define SPIKEHAT_SERIAL_DEFAULT "/dev/serial0"

typedef enum {
    SPIKEHAT_DEVICE_NONE     = 0,
    SPIKEHAT_DEVICE_MOTOR_M  = 1,  /* SPIKE Prime Medium Angular Motor */
    SPIKEHAT_DEVICE_MOTOR_L  = 2,  /* SPIKE Prime Large Angular Motor  */
    SPIKEHAT_DEVICE_COLOR    = 3,  /* SPIKE Prime Color Sensor (HSV)   */
    SPIKEHAT_DEVICE_DISTANCE = 4,  /* SPIKE Prime Distance Sensor      */
    SPIKEHAT_DEVICE_FORCE    = 5,  /* SPIKE Prime Force Sensor         */
} spikehat_device_t;

typedef struct {
    spikehat_device_t device;
    float             values[8];
    int               nvalues;
    int               valid;
} spikehat_port_data_t;

typedef struct spikehat {
    int                  fd;
    pthread_t            reader;
    pthread_mutex_t      lock;
    spikehat_port_data_t ports[SPIKEHAT_MAX_PORTS];
    volatile int         running;
} spikehat_t;

/* 初期化 / 終了 */
spikehat_t *spikehat_open(const char *device);
void        spikehat_close(spikehat_t *hat);

/* スリープ
 * 実機版: OS の usleep() と同等
 * シム版: MuJoCo のシミュレーションステップを進める
 * 注意: time.sleep() や usleep() を直接使うとシム版では
 *       センサー値が更新されないため、必ずこの関数を使うこと */
void spikehat_sleep(spikehat_t *hat, float seconds);

/* ポート設定 - open後、利用前に呼ぶ */
int spikehat_port_config(spikehat_t *hat, int port, spikehat_device_t type);

/* モーター制御 */
int spikehat_motor_pwm(spikehat_t *hat, int port, float power);       /* -1.0〜+1.0 直接PWM */
int spikehat_motor_start(spikehat_t *hat, int port, int speed);       /* -100〜+100 速度制御 */
int spikehat_motor_stop(spikehat_t *hat, int port);
int spikehat_motor_coast(spikehat_t *hat, int port);
int spikehat_motor_run_for_seconds(spikehat_t *hat, int port, float seconds, int speed);
int spikehat_motor_get_speed(spikehat_t *hat, int port, int *speed);
int spikehat_motor_get_position(spikehat_t *hat, int port, int *degrees);

/* センサー読み取り */
int spikehat_distance_read(spikehat_t *hat, int port, int *mm);
int spikehat_color_read_hsv(spikehat_t *hat, int port, int *hue, int *sat, int *val);
int spikehat_color_read_rgb(spikehat_t *hat, int port, int *r, int *g, int *b);
int spikehat_force_read(spikehat_t *hat, int port, int *force, int *pressed);
int spikehat_force_is_pressed(spikehat_t *hat, int port, int *pressed);
int spikehat_force_get_force(spikehat_t *hat, int port, int *force);

#ifdef __cplusplus
}
#endif

#endif /* SPIKEHAT_H */
