#!/usr/bin/env python3
"""
test_motor_viewer.py — モーターリアルタイムテスト

MuJoCoビューアと連携してモーターの角度・角速度をリアルタイム表示する。
Control タブの motor_ctrl スライダーを動かすとモーターが回転する。

実行方法:
  cd libspikehat_sim
  mjpython examples/test_motor_viewer.py
"""
import mujoco
import mujoco.viewer
import os

XML_PATH = os.path.join(os.path.dirname(__file__), "test_motor.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "motor_joint")
qpos_adr = model.jnt_qposadr[joint_id]
qvel_adr = model.jnt_dofadr[joint_id]

print(f"motor_joint id={joint_id}  qpos_adr={qpos_adr}")
print("ビューアを起動します。Control タブの motor_ctrl スライダーを動かしてください。\n")

prev_deg = None

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        deg = data.qpos[qpos_adr] * 180.0 / 3.141592653589793
        vel = data.qvel[qvel_adr] * 180.0 / 3.141592653589793

        if prev_deg is None or abs(deg - prev_deg) >= 1.0:
            print(f"角度: {deg:+7.1f} deg  角速度: {vel:+7.1f} deg/s")
            prev_deg = deg
