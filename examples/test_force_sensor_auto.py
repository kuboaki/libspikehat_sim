#!/usr/bin/env python3
"""
test_force_sensor_auto.py — フォースセンサー自動テスト（シム専用）

spikehat_sim_set_ctrl でスライダーを段階的に操作して
force値とpressed判定の変化を確認する。

実行方法:
  cd libspikehat_sim
  SPIKEHAT_SIM_XML=examples/test_force_sensor.xml \
    python3 examples/test_force_sensor_auto.py
"""
import sys
import ctypes
import mujoco
import os

sys.path.insert(0, 'python')
from spikehat import SpikeHat, DEVICE_FORCE, _lib, _hat_p

# spikehat_sim_set_ctrl のシグネチャを登録
_lib.spikehat_sim_set_ctrl.restype  = ctypes.c_int
_lib.spikehat_sim_set_ctrl.argtypes = [_hat_p, ctypes.c_int, ctypes.c_double]

XML_PATH   = os.environ.get('SPIKEHAT_SIM_XML', 'examples/test_force_sensor.xml')
PORT_FORCE = 1

# アクチュエーターIDを取得
_m = mujoco.MjModel.from_xml_path(XML_PATH)
ctrl_id = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_ACTUATOR, "press_ctrl")
print(f"press_ctrl actuator_id={ctrl_id}\n")

def set_slider(hat, val):
    _lib.spikehat_sim_set_ctrl(hat._hat, ctrl_id, val)
    hat.sleep(0.3)

with SpikeHat(XML_PATH) as hat:
    hat.port_config(PORT_FORCE, DEVICE_FORCE)
    hat.sleep(1.0)

    # ── テスト1: force_read ──────────────────────────────
    print("=== テスト1: force_read (スライダーを段階的に変化) ===")
    print(f"{'slider':>8}  {'force':>6}  {'pressed':>8}")
    print("-" * 30)
    for val in [0.0, 0.020, 0.040, 0.055, 0.060, 0.065, 0.070, 0.065, 0.040, 0.0]:
        set_slider(hat, val)
        force, pressed = hat.force_read(PORT_FORCE)
        print(f"{val*1000:>6.1f}mm  {force:>6}N  {pressed:>8}  "
              f"{'[押下]' if pressed else ''}")

    # ── テスト2: force_is_pressed ────────────────────────
    print("\n=== テスト2: force_is_pressed ===")
    print(f"{'slider':>8}  {'pressed':>8}")
    print("-" * 20)
    for val in [0.0, 0.040, 0.060, 0.070, 0.040, 0.0]:
        set_slider(hat, val)
        pressed = hat.force_is_pressed(PORT_FORCE)
        print(f"{val*1000:>6.1f}mm  {int(pressed):>8}  "
              f"{'[押下]' if pressed else ''}")

    # ── テスト3: force_get_force ─────────────────────────
    print("\n=== テスト3: force_get_force ===")
    print(f"{'slider':>8}  {'force':>6}")
    print("-" * 18)
    for val in [0.0, 0.020, 0.040, 0.055, 0.060, 0.065, 0.070, 0.040, 0.0]:
        set_slider(hat, val)
        force = hat.force_get_force(PORT_FORCE)
        print(f"{val*1000:>6.1f}mm  {force:>6}N")

print("\n完了")
