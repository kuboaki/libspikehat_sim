/**
 * sim_spikehat.c — MuJoCo シミュレーター上で spikehat API を模倣する実装
 *
 * spikehat.h と同じ API を提供し、実機の libspikehat と差し替えて使える。
 * 実機との違いはシリアル通信の代わりに MuJoCo の C API を使う点のみ。
 *
 * 【設計方針】
 *   - spikehat_t 構造体を拡張して MuJoCo の model/data を保持する
 *   - motor_get_position は累積角度（度数）を返す（実機エンコーダと同様）
 *   - time.sleep 相当は spikehat_motor_run_for_seconds 内でステップ実行
 *   - speed (-100〜+100) を ctrl (-1〜+1) に変換するスケール: 1/100
 *
 * 【ビルド方法】
 *   MuJoCo のヘッダーとライブラリをリンクする:
 *   cc -shared -fPIC -o libspikehat_sim.so sim_spikehat.c \
 *      -I/path/to/mujoco/include \
 *      -L/path/to/mujoco/lib -lmujoco \
 *      -lpthread -lm
 */

#include "../include/spikehat.h"

#include <mujoco/mujoco.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

/* ── シミュレーション定数 ──────────────────────────────────────────── */

/** speed (-100〜+100) → ctrl (-1〜+1) の変換スケール */
#define SPEED_TO_CTRL  (1.0 / 100.0)

/** デフォルトの速度スケール（実時間1秒 = シム何秒分） */
#define DEFAULT_SPEED_SCALE  10.0

/** モータージョイント名 */
#define MOTOR_JOINT_NAME  "motor_joint"

/** 最大検出距離 [m] */
#define DIST_MAX_M  0.300

/** 距離測定不能値 [mm] */
#define DIST_INVALID  2000


/* ── spikehat_t の拡張フィールド ────────────────────────────────────
 *
 * spikehat.h の spikehat_t はシリアル通信用のフィールドを持つ。
 * シム版では fd / reader / lock / running / ports を流用しつつ、
 * MuJoCo 関連データを追加フィールドとして持つ。
 *
 * ただし spikehat_t の構造体を変更するとヘッダーが変わってしまうため、
 * 「シム専用の拡張構造体」を定義して spikehat_t * にキャストして使う。
 */

typedef struct sim_spikehat {
    /* spikehat_t と同じレイアウト（先頭に配置して互換性を保つ） */
    int                  fd;          /* 未使用（0固定） */
    pthread_t            reader;      /* 未使用 */
    pthread_mutex_t      lock;        /* 未使用 */
    spikehat_port_data_t ports[SPIKEHAT_MAX_PORTS];
    volatile int         running;

    /* MuJoCo 拡張フィールド */
    mjModel *model;
    mjData  *data;
    double   speed_scale;    /* 速度スケール */
    double   ctrl;           /* 現在のctrl値 */

    /* 累積角度管理（実機エンコーダ相当） */
    double   position_deg;   /* 累積角度 [度] */
    double   prev_qpos;      /* 前回のqpos値 [rad] */

    /* MuJoCo インデックスキャッシュ */
    int      joint_id;       /* motor_joint の ID */
    int      qpos_adr;       /* motor_joint の qpos アドレス */
    int      ctrl_id;        /* actuator の ctrl インデックス */
    int      color_site_id;  /* カラーセンサーsite（-1=未検出） */
} sim_spikehat_t;


/* ── ヘルパー関数 ────────────────────────────────────────────────── */

static void _update_position(sim_spikehat_t *sim) {
    if (sim->qpos_adr < 0) return;  /* モーターなし */
    double curr = sim->data->qpos[sim->qpos_adr];
    double delta = curr - sim->prev_qpos;
    if (delta >  M_PI) delta -= 2.0 * M_PI;
    if (delta < -M_PI) delta += 2.0 * M_PI;
    sim->position_deg -= delta * (180.0 / M_PI);
    sim->prev_qpos = curr;
}

/** 1ステップ進めて累積角度を更新する */
static void sim_step(sim_spikehat_t *sim) {
    sim->data->ctrl[sim->ctrl_id] = sim->ctrl;
    mj_step(sim->model, sim->data);
    _update_position(sim);
}

/** seconds 秒分のシミュレーションステップを実行する */
static void sim_sleep(sim_spikehat_t *sim, double seconds) {
    int steps = (int)(seconds * sim->speed_scale / sim->model->opt.timestep);
    if (steps < 1) steps = 1;
    for (int i = 0; i < steps; i++) {
        sim_step(sim);
    }
}


/* ── 初期化 / 終了 ──────────────────────────────────────────────── */

/**
 * spikehat_open — MuJoCo XMLからシムを初期化する
 *
 * @param device  MuJoCo XML ファイルのパスを渡す（実機のシリアルパスの代わり）
 * @return        sim_spikehat_t * を spikehat_t * にキャストして返す
 */
spikehat_t *spikehat_open(const char *device) {
    /* 環境変数 SPIKEHAT_SIM_XML が設定されていればXMLパスとして使う
     * 未設定の場合は device 引数をそのまま使う（通常は /dev/serial0 が渡される） */
    const char *xml_path = getenv("SPIKEHAT_SIM_XML");
    if (!xml_path || xml_path[0] == '\0') {
        xml_path = device;
    }

    sim_spikehat_t *sim = calloc(1, sizeof(sim_spikehat_t));
    if (!sim) return NULL;

    /* MuJoCo モデルのロード */
    char error[1024] = "";
    sim->model = mj_loadXML(xml_path, NULL, error, sizeof(error));
    if (!sim->model) {
        fprintf(stderr, "sim_spikehat: mj_loadXML failed: %s\n", error);
        free(sim);
        return NULL;
    }
    sim->data = mj_makeData(sim->model);
    if (!sim->data) {
        fprintf(stderr, "sim_spikehat: mj_makeData failed\n");
        mj_deleteModel(sim->model);
        free(sim);
        return NULL;
    }

    /* 速度スケールの設定 */
    sim->speed_scale = DEFAULT_SPEED_SCALE;

    /* motor_joint のインデックスを検索（なければ -1 でモーター機能無効） */
    sim->joint_id = mj_name2id(sim->model, mjOBJ_JOINT, "motor_joint");
    if (sim->joint_id < 0) {
        fprintf(stderr, "[sim] motor_joint not found: motor functions disabled\n");
        sim->qpos_adr = -1;
    } else {
        sim->qpos_adr = sim->model->jnt_qposadr[sim->joint_id];
    }
    sim->ctrl_id = 0;

    /* 初期状態を計算 */
    mj_forward(sim->model, sim->data);
    sim->prev_qpos    = (sim->qpos_adr >= 0) ? sim->data->qpos[sim->qpos_adr] : 0.0;
    sim->position_deg = 0.0;
    sim->ctrl         = 0.0;
    sim->running      = 1;
    sim->color_site_id = mj_name2id(sim->model, mjOBJ_SITE, "color_site");
    fprintf(stderr, "[sim] color_site id=%d\n", sim->color_site_id);

    fprintf(stderr, "[sim] spikehat_open: %s (timestep=%.4f, speed_scale=%.1f)\n",
            xml_path, sim->model->opt.timestep, sim->speed_scale);

    return (spikehat_t *)sim;
}

void spikehat_close(spikehat_t *hat) {
    if (!hat) return;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;

    /* 全モーターを停止 */
    sim->ctrl = 0.0;
    sim_step(sim);

    mj_deleteData(sim->data);
    mj_deleteModel(sim->model);
    free(sim);

    fprintf(stderr, "[sim] spikehat_close: 終了\n");
}


/* ── ポート設定 ─────────────────────────────────────────────────── */

int spikehat_port_config(spikehat_t *hat, int port, spikehat_device_t type) {
    if (!hat || port < 0 || port >= SPIKEHAT_MAX_PORTS) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ports[port].device  = type;
    sim->ports[port].valid   = 0;
    sim->ports[port].nvalues = 0;
    fprintf(stderr, "[sim] port_config(port=%d, device=%d)\n", port, type);
    return 0;
}


/* ── モーター制御 ───────────────────────────────────────────────── */

int spikehat_motor_pwm(spikehat_t *hat, int port, float power) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (power >  1.0f) power =  1.0f;
    if (power < -1.0f) power = -1.0f;
    sim->ctrl = (double)power;
    sim_step(sim);
    return 0;
}

int spikehat_motor_start(spikehat_t *hat, int port, int speed) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ctrl = speed * SPEED_TO_CTRL;
    sim_step(sim);
    return 0;
}

int spikehat_motor_stop(spikehat_t *hat, int port) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ctrl = 0.0;
    sim_step(sim);
    return 0;
}

int spikehat_motor_coast(spikehat_t *hat, int port) {
    return spikehat_motor_stop(hat, port);
}

int spikehat_motor_run_for_seconds(spikehat_t *hat, int port,
                                   float seconds, int speed) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ctrl = speed * SPEED_TO_CTRL;
    sim_sleep(sim, (double)seconds);
    sim->ctrl = 0.0;
    sim_step(sim);
    return 0;
}

int spikehat_motor_run_for_degrees(spikehat_t *hat, int port,
                                   int degrees, int speed) {
    if (!hat || speed == 0) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->qpos_adr < 0) return -1;

    /* 目標角度を計算
     * 正の速度 → position_deg が減少するため、degrees の符号を反転 */
    double cur    = sim->position_deg;
    double target = cur - (double)degrees;

    /* 速度の符号で方向を決める */
    int actual_speed = (degrees >= 0) ? abs(speed) : -abs(speed);
    sim->ctrl = actual_speed * SPEED_TO_CTRL;

    /* 慣性補正: 目標の手前で止める（速度に比例した補正量） */
    double early = 0.0;
    double stop_at = target;

    /* stop_at に達するまでステップ実行（speed_scale を考慮） */
    int max_steps = (int)(10.0 * sim->speed_scale / sim->model->opt.timestep);
    for (int i = 0; i < max_steps; i++) {
        sim_step(sim);
        double pos = sim->position_deg;
        if (degrees >= 0 && pos <= stop_at) break;
        if (degrees <  0 && pos >= stop_at) break;
    }

    /* ブレーキ: ctrl=0 にして慣性が収まるまで少し待つ */
    sim->ctrl = 0.0;
    int settle = (int)(0.5 / sim->model->opt.timestep);
    for (int i = 0; i < settle; i++) {
        sim_step(sim);
    }
    return 0;
}

int spikehat_motor_get_speed(spikehat_t *hat, int port, int *speed) {
    if (!hat || !speed) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    *speed = (int)round(sim->ctrl / SPEED_TO_CTRL);
    return 0;
}

int spikehat_motor_get_position(spikehat_t *hat, int port, int *degrees) {
    if (!hat || !degrees) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->qpos_adr < 0) return -1;  /* モーターなし */
    *degrees = (int)round(sim->position_deg);
    return 0;
}


/* ── センサー読み取り ────────────────────────────────────────────── */

int spikehat_distance_read(spikehat_t *hat, int port, int *mm) {
    if (!hat || !mm) return -1;
    /* TODO: MuJoCo のレイキャストで実装 */
    *mm = DIST_INVALID;
    return 0;
}

static void rgb_to_hsv(double r, double g, double b,
                       int *hue, int *sat, int *val) {
    double max = r > g ? (r > b ? r : b) : (g > b ? g : b);
    double min = r < g ? (r < b ? r : b) : (g < b ? g : b);
    double delta = max - min;
    *val = (int)round(max * 1000.0);
    *sat = (max > 0.0) ? (int)round((delta / max) * 1000.0) : 0;
    if (delta < 1e-6) { *hue = 0; return; }
    double h;
    if (max == r)      h = 60.0 * fmod((g - b) / delta, 6.0);
    else if (max == g) h = 60.0 * ((b - r) / delta + 2.0);
    else               h = 60.0 * ((r - g) / delta + 4.0);
    if (h < 0.0) h += 360.0;
    *hue = (int)round(h);
}

int spikehat_color_read_hsv(spikehat_t *hat, int port,
                             int *hue, int *sat, int *val) {
    if (!hat || !hue || !sat || !val) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->color_site_id < 0) { *hue = *sat = *val = 0; return 0; }

    /* color_site の現在位置 */
    double *sp = &sim->data->site_xpos[sim->color_site_id * 3];

    /* 真下方向 */
    double down[3] = { 0.0, 0.0, -1.0 };

    /* color_site が属する body を除外してレイキャスト */
    int site_bodyid = sim->model->site_bodyid[sim->color_site_id];
    int geomid_out[1] = { -1 };
    mjtNum normal[3];
    mjtNum dist = mj_ray(sim->model, sim->data,
                         sp, down, NULL, 1, site_bodyid, geomid_out, normal);

    if (dist < 0 || geomid_out[0] < 0) {
        *hue = *sat = *val = 0;
        return 0;
    }

    /* ヒットしたgeomのRGBAからHSVを計算 */
    int gid = geomid_out[0];
    float *rgba = &sim->model->geom_rgba[gid * 4];
    rgb_to_hsv((double)rgba[0], (double)rgba[1], (double)rgba[2],
               hue, sat, val);
    return 0;
}

int spikehat_color_read_rgb(spikehat_t *hat, int port,
                             int *r, int *g, int *b) {
    if (!hat || !r || !g || !b) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->color_site_id < 0) { *r = *g = *b = 0; return 0; }

    /* color_site の現在位置 */
    double *sp = &sim->data->site_xpos[sim->color_site_id * 3];

    /* 真下方向 */
    double down[3] = { 0.0, 0.0, -1.0 };

    /* レイキャスト */
    int site_bodyid = sim->model->site_bodyid[sim->color_site_id];
    int geomid_out[1] = { -1 };
    mjtNum normal[3];
    mjtNum dist = mj_ray(sim->model, sim->data,
                         sp, down, NULL, 1, site_bodyid,
                         geomid_out, normal);

    if (dist < 0 || geomid_out[0] < 0) {
        *r = *g = *b = 0;
        return 0;
    }

    /* geomのRGBAから RGB (0〜255) に変換 */
    float *rgba = &sim->model->geom_rgba[geomid_out[0] * 4];
    *r = (int)round(rgba[0] * 255.0f);
    *g = (int)round(rgba[1] * 255.0f);
    *b = (int)round(rgba[2] * 255.0f);
    return 0;
}

int spikehat_force_read(spikehat_t *hat, int port,
                        int *force, int *pressed) {
    if (!hat || !force || !pressed) return -1;
    *force = 0; *pressed = 0;
    return 0;
}
