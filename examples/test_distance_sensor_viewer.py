#!/usr/bin/env python3
"""
test_distance_sensor_viewer.py — 距離センサーリアルタイムテスト

MuJoCoビューアと連携してセンサー値をリアルタイム表示する。
wall_slide_ctrl スライダーを動かすと距離が変わる。

実行方法:
  cd libspikehat_sim
  mjpython examples/test_distance_sensor_viewer.py

距離の計測について:
  distance_site（センサーデバイス面）から壁表面までの距離を返す。
  wall_y（壁bodyの中心）との差 = siteオフセット(15mm) + 壁半厚(5mm) = 20mm
  例: wall_y=150mm → 距離=130mm
"""
import mujoco
import mujoco.viewer
import numpy as np
import os

XML_PATH = os.path.join(os.path.dirname(__file__), "test_distance_sensor.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

# distance_site のIDを取得
site_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "distance_site")
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wall_slide")
qpos_adr = model.jnt_qposadr[joint_id]

DIST_MAX_M   = 0.300
DIST_MIN_M   = 0.050
DIST_INVALID = 2000

def read_distance():
    sp       = data.site_xpos[site_id].copy()
    # site_xmat の Y軸正方向が前方
    xmat     = data.site_xmat[site_id].reshape(3, 3)
    forward  = xmat[:, 1]
    bodyid   = model.site_bodyid[site_id]
    geomid   = np.array([-1], dtype=np.int32)
    normal   = np.zeros(3)
    dist     = mujoco.mj_ray(model, data, sp, forward,
                              None, 1, bodyid, geomid, normal)
    if dist < 0 or dist > DIST_MAX_M:
        return DIST_INVALID
    if dist < DIST_MIN_M:
        return DIST_INVALID
    return int(round(dist * 1000))

print(f"distance_site id={site_id}  wall_slide qpos_adr={qpos_adr}")
print("ビューアを起動します。wall_slide_ctrl スライダーを動かしてください。")
print("有効距離: 50mm〜300mm\n")

prev_mm = -1

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        mm = read_distance()

        if mm != prev_mm:
            wall_pos = data.qpos[qpos_adr]
            if mm == DIST_INVALID:
                print(f"wall_slide={wall_pos*1000:+6.1f}mm  距離: ---- mm (範囲外)")
            else:
                wall_actual = (0.150 + wall_pos) * 1000  # 壁の実際のY座標[mm]
                print(f"wall_slide={wall_pos*1000:+6.1f}mm  "
                      f"wall_y={wall_actual:+6.1f}mm  距離: {mm:4d} mm")
            prev_mm = mm
