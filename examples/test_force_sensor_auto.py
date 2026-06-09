#!/usr/bin/env python3
"""
test_force_sensor_auto.py — フォースセンサー自動テスト

spikehat_sim_set_ctrl でスライダーを自動操作して3つのAPIをテストする。
シム専用テスト（実機では動作しない）。

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
_lib.spikehat_sim_get_model.restype  = ctypes.c_void_p
_lib.spikehat_sim_get_model.argtypes = [_hat_p]

XML_PATH   = os.environ.get('SPIKEHAT_SIM_XML', 'examples/test_force_sensor.xml')
PORT_FORCE = 1

print("=== フォースセンサー自動テスト ===\n")

with SpikeHat(XML_PATH) as hat:
    hat.port_config(PORT_FORCE, DEVICE_FORCE)
    hat.sleep(1.0)

    # アクチュエーターIDを取得
    model_ptr = _lib.spikehat_sim_get_model(hat._hat)
    model = mujoco.MjModel.from_raw_pointer(model_ptr) if hasattr(mujoco.MjModel, 'from_raw_pointer') else None

    # 直接mj_name2idで取得
    import mujoco._structs
    # モデルをXMLから再ロードしてIDを調べる
    _m = mujoco.MjModel.from_xml_path(XML_PATH)
    ctrl_id = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_ACTUATOR, "press_ctrl")
    print(f"press_ctrl actuator_id={ctrl_id}")

    def set_slider(val):
        _lib.spikehat_sim_set_ctrl(hat._hat, ctrl_id, val)
        hat.sleep(0.5)

    for test_name, api_func in [
        ("テスト1: force_read",
         lambda: hat.force_read(PORT_FORCE)),
        ("テスト2: force_is_pressed",
         lambda: (None, hat.force_is_pressed(PORT_FORCE))),
        ("テスト3: force_get_force",
         lambda: (hat.force_get_force(PORT_FORCE), None)),
    ]:
        print(f"=== {test_name} ===")

        set_slider(0.0)
        force, pressed = api_func()
        print(f"  未押下: force={force}  pressed={pressed}")

        set_slider(0.065)
        force, pressed = api_func()
        print(f"  押下中: force={force}  pressed={pressed}")

        set_slider(0.0)
        force, pressed = api_func()
        print(f"  解放後: force={force}  pressed={pressed}")
        print()

print("完了")
