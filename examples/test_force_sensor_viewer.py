#!/usr/bin/env python3
"""
test_force_sensor_viewer.py — フォースセンサーテスト（力検出付き）

button と press_block が1つの剛体として動く。
力の読み取り: spring_force = stiffness × joint_pos [N]
  1mm → 1N (pressed 閾値)
 10mm → 10N (最大)

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

button_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "button_slide")
btn_qpos_adr    = model.jnt_qposadr[button_joint_id]

STIFFNESS         = 1000.0  # N/m (1N/mm)
PRESSED_ON_M      = 0.001   # 1.0mm: pressed 開始（仕様: 1 ± 0.5mm）
PRESSED_OFF_M     = 0.0005  # 0.5mm: pressed 解除（ヒステリシス）

print(f"button_slide id={button_joint_id}")
print("press_ctrl スライダーを動かしてください")
print("  1mm(0.001) → pressed(≈1N)、15mm(0.015) → 完全埋没(15N)\n")

prev_state = None

with mujoco.viewer.launch_passive(model, data) as viewer:
    model.stat.center[:] = [0.0, 0.0, 0.05]
    model.stat.extent    = 0.10
    viewer.cam.lookat[:] = model.stat.center
    viewer.cam.distance  = model.stat.extent * 3.0
    viewer.cam.azimuth   = 200.0
    viewer.cam.elevation = -25.0

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        btn_pos_m    = data.qpos[btn_qpos_adr]
        spring_force = STIFFNESS * btn_pos_m

        # ヒステリシス: ON=1mm、OFF=0.5mm（実機仕様: しきい値 1 ± 0.5mm）
        was_pressed  = prev_state[1] if prev_state else False
        if was_pressed:
            pressed = btn_pos_m > PRESSED_OFF_M
        else:
            pressed = btn_pos_m > PRESSED_ON_M

        state = (round(btn_pos_m * 10000), pressed)
        if state != prev_state:
            print(f"button={btn_pos_m*1000:5.2f}mm  "
                  f"force={spring_force:5.2f}N  "
                  f"{'[押下]' if pressed else '      '}")
            prev_state = state
