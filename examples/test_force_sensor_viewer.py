#!/usr/bin/env python3
"""
test_force_sensor_viewer.py — フォースセンサーテスト（press_block あり）

press_block と button は equality 制約でシャフト接合（1:1 連動）。
motor アクチュエーターで press_block に力を印加。

  ctrl=0  → 力ゼロ → button 内蔵スプリングで press_block も自動復元（「離す」）
  ctrl>0  → press_block が下降 → button が一緒に押し込まれる（「押す」）
  ctrl=10N → 最大（実機確認値: 10N at 8.4mm）
  Space キー → ctrl=0 リセット（即座に「離す」）

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
press_joint_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "press_slide")
press_ctrl_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "press_ctrl")
btn_qpos_adr    = model.jnt_qposadr[button_joint_id]
press_qpos_adr  = model.jnt_qposadr[press_joint_id]

STIFFNESS   = 1253.0  # 実機再校正: slider=9.5N が実機10N相当 → 1190×(10/9.5)=1253 N/m
GRAVITY_PRELOAD = (0.010 + 0.020) * 9.81  # button+press_block の重力分 ≈ 0.294N
PRESSED_ON  = 0.001   # 1.0mm: pressed 開始
PRESSED_OFF = 0.0005  # 0.5mm: pressed 解除（ヒステリシス）

print("press_ctrl スライダー: 印加力 [N]")
print("  ctrl=0   → 「離す」: 力ゼロ → スプリングで press_block も自動復元")
print("  ctrl≈1N  → 「押す」: [押下] 開始")
print("  ctrl=10N → 最大（実機確認値）")
print("  Space キー → ctrl=0（即座に「離す」）\n")

def key_callback(keycode):
    if keycode == 32:
        data.ctrl[press_ctrl_id] = 0.0

prev_state = None

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    model.stat.center[:] = [0.0, 0.0, 0.07]
    model.stat.extent    = 0.12
    viewer.cam.lookat[:] = model.stat.center
    viewer.cam.distance  = model.stat.extent * 3.0
    viewer.cam.azimuth   = 200.0
    viewer.cam.elevation = -25.0

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        btn_pos_m    = data.qpos[btn_qpos_adr]
        press_pos_m  = data.qpos[press_qpos_adr]
        spring_force = STIFFNESS * btn_pos_m
        # 外力 = スプリング力 - 重力プリロード（実機キャリブレーション相当）
        ext_force    = max(0.0, spring_force - GRAVITY_PRELOAD)
        applied_n    = data.ctrl[press_ctrl_id]

        was_pressed = prev_state[3] if prev_state else False
        if was_pressed:
            pressed = btn_pos_m > PRESSED_OFF
        else:
            pressed = btn_pos_m > PRESSED_ON

        state = (round(btn_pos_m * 10000),
                 round(press_pos_m * 10000),
                 round(applied_n * 10),
                 pressed)
        if state != prev_state:
            print(f"apply={applied_n:5.1f}N  "
                  f"button={btn_pos_m*1000:5.2f}mm  "
                  f"press={press_pos_m*1000:5.2f}mm  "
                  f"force={ext_force:5.2f}N  "
                  f"{'[押下]' if pressed else '      '}")
            prev_state = state
