#!/usr/bin/env python3
"""
test_force_sensor_viewer.py — フォースセンサーリアルタイムテスト

MuJoCoビューアと連携してセンサー値をリアルタイム表示する。
press_ctrl スライダーを動かすと押下ブロックが下降してセンサーに接触する。

実行方法:
  cd libspikehat_sim
  mjpython examples/test_force_sensor_viewer.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os

XML_PATH = os.path.join(os.path.dirname(__file__), "test_force_sensor.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "force_touch")
joint_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,  "press_slide")
qpos_adr  = model.jnt_qposadr[joint_id]
sens_adr  = model.sensor_adr[sensor_id]

PRESS_THRESHOLD = 1.0  # N

print(f"force_touch sensor id={sensor_id}  adr={sens_adr}")
print("ビューアを起動します。press_ctrl スライダーを下げてください。")
print("1N以上で pressed=1 になります。\n")

prev_state = (-1, -1)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        f       = data.sensordata[sens_adr]
        force   = int(round(f))
        pressed = 1 if f > PRESS_THRESHOLD else 0
        press_pos = data.qpos[qpos_adr] * 1000  # mm

        if (force, pressed) != prev_state:
            print(f"press={press_pos:+5.1f}mm  "
                  f"force={force:3d} N  "
                  f"pressed={'[押下]' if pressed else '      '}")
            prev_state = (force, pressed)
