#!/usr/bin/env python3
"""
test_distance_sensor_viewer.py — 距離センサーリアルタイムテスト

MuJoCoビューアと連携してセンサー値をリアルタイム表示する。
wall_slide_ctrl スライダーを動かすと距離が変わる。

実行方法:
  cd libspikehat_sim
  uv run mjpython examples/test_distance_sensor_viewer.py

距離の計測について:
  distance_site（センサーボディ内側5mm）から壁表面までの距離を返す。
  wall_body pos Y と計測距離の関係:
    計測距離 ≈ wall_body_Y - 0.0243m
      （0.0243 = site_Y(0.0108) + 壁の半厚(0.0271/2=0.0135)）
  例: wall_body pos Y=0.175m → 計測距離 ≈ 150.7mm
"""
import mujoco
import mujoco.viewer
import numpy as np
import os

XML_PATH = os.path.join(os.path.dirname(__file__), "test_distance_sensor.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

site_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "distance_site")
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wall_slide")
qpos_adr = model.jnt_qposadr[joint_id]

WALL_BODY_INITIAL_Y = 0.1173  # 壁の初期Y座標[m]（IOファイルから算出: 117.3mm）
SITE_OFFSET_M = 0.0243        # site_Y(0.0108) + 壁半厚(0.0135) [m]
DIST_MAX_M   = 0.300
DIST_MIN_M   = 0.050
DIST_INVALID = 2000

def read_distance():
    sp      = data.site_xpos[site_id].copy()
    xmat    = data.site_xmat[site_id].reshape(3, 3)
    forward = xmat[:, 1]   # site_xmat のY軸正方向 = センサー前方
    bodyid  = model.site_bodyid[site_id]
    geomid  = np.array([-1], dtype=np.int32)
    normal  = np.zeros(3)
    dist    = mujoco.mj_ray(model, data, sp, forward,
                             None, 1, bodyid, geomid, normal)
    if dist < 0 or dist > DIST_MAX_M:
        return DIST_INVALID
    if dist < DIST_MIN_M:
        return DIST_INVALID
    return int(round(dist * 1000))

# 起動時に distance_site の初期ワールド座標を表示
mujoco.mj_kinematics(model, data)
sp = data.site_xpos[site_id]
print(f"distance_site id={site_id}  wall_slide qpos_adr={qpos_adr}")
print(f"distance_site 初期ワールド座標: x={sp[0]:.4f} y={sp[1]:.4f} z={sp[2]:.4f}")
print("ビューアを起動します。wall_slide_ctrl スライダーを動かしてください。")
print(f"有効距離: {DIST_MIN_M*1000:.0f}mm〜{DIST_MAX_M*1000:.0f}mm\n")

prev_mm = -1

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 初期カメラ設定
    # azimuth=310: 現在(130)の反対側 → 壁が左、センサーが右に見える
    # extent=0.125: 2倍の表示サイズ
    model.stat.center[:] = [0.0, 0.06, 0.03]
    model.stat.extent    = 0.125
    viewer.cam.lookat[:] = model.stat.center
    viewer.cam.distance  = model.stat.extent * 3.0
    viewer.cam.azimuth   = 310.0
    viewer.cam.elevation = -25.0

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        mm = read_distance()

        if mm != prev_mm:
            wall_slide = data.qpos[qpos_adr]
            # axis="0 -1 0"のため正の値で壁が離れる: Y = initial - slide
            wall_y = (WALL_BODY_INITIAL_Y - wall_slide) * 1000
            if mm == DIST_INVALID:
                print(f"wall_slide={wall_slide*1000:+6.1f}mm  "
                      f"wall_y={wall_y:+7.1f}mm  距離: ---- mm (範囲外)")
            else:
                print(f"wall_slide={wall_slide*1000:+6.1f}mm  "
                      f"wall_y={wall_y:+7.1f}mm  距離: {mm:4d} mm")
            prev_mm = mm
