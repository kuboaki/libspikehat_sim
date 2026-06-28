#!/usr/bin/env python3
"""
test_force_sensor_viewer.py

button + press_block = 1つの剛体（シャフト接合）。
button_slide を直接制御。
ctrl=0 でスプリングが自動復元。

実行方法:
  cd libspikehat_sim
  uv run mjpython examples/test_force_sensor_viewer.py
"""
import mujoco
import mujoco.viewer
import os

XML_PATH = os.path.join(os.path.dirname(__file__), "test_force_sensor.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

button_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "button_slide")
press_ctrl_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "press_ctrl")
btn_qpos_adr    = model.jnt_qposadr[button_joint_id]

STIFFNESS   = 1190.0  # 実機確認: 10N at 8.4mm → 10/0.0084=1190 N/m
PRESSED_ON  = 0.001
PRESSED_OFF = 0.0005

print("press_ctrl スライダー: 目標位置 [m]")
print("  ctrl=0     → 初期位置（スプリングで自動保持）")
print("  ctrl=0.001 → [押下] 閾値(1mm, 1N)")
print("  ctrl=0.010 → 最大(10mm, 10N)")
print("  ctrl=0 に戻す → スプリング自動復元")
print("  Space キー → ctrl=0 リセット\n")

def key_callback(keycode):
    if keycode == 32:
        data.ctrl[press_ctrl_id] = 0.0

prev_state = None

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    model.stat.center[:] = [0.0, 0.0, 0.06]
    model.stat.extent    = 0.12
    viewer.cam.lookat[:] = model.stat.center
    viewer.cam.distance  = model.stat.extent * 3.0
    viewer.cam.azimuth   = 200.0
    viewer.cam.elevation = -25.0

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        btn_pos_m    = data.qpos[btn_qpos_adr]
        spring_force = STIFFNESS * btn_pos_m

        was_pressed = prev_state[1] if prev_state else False
        if was_pressed:
            pressed = btn_pos_m > PRESSED_OFF
        else:
            pressed = btn_pos_m > PRESSED_ON

        state = (round(btn_pos_m * 10000), pressed)
        if state != prev_state:
            print(f"button={btn_pos_m*1000:5.2f}mm  "
                  f"force={spring_force:5.2f}N  "
                  f"{'[押下]' if pressed else '      '}")
            prev_state = state
