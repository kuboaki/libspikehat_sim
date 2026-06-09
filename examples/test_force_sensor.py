#!/usr/bin/env python3
"""
test_force_sensor.py — フォースセンサーテスト（Python版）

ポート構成:
  B(1): フォースセンサー

実機での実行方法:
  python3 examples/test_force_sensor.py

シムでの実行方法:
  SPIKEHAT_SIM_XML=examples/test_force_sensor.xml \\
    python3 examples/test_force_sensor.py

フォースセンサーの戻り値:
  force   : 力 [N]（0〜10）
  pressed : 押下状態（False=未押下, True=押下）
"""
import sys
import time
sys.path.insert(0, 'python')

from spikehat import SpikeHat, DEVICE_FORCE

PORT_FORCE = 1   # ポートB

with SpikeHat() as hat:
    hat.port_config(PORT_FORCE, DEVICE_FORCE)
    time.sleep(1)

    print("=== フォースセンサーテスト (20回) ===")
    print("センサーを押して離してみてください")
    print()

    prev_force   = -1
    prev_pressed = -1
    count = 0
    while count < 2000:
        try:
            force, pressed = hat.force_read(PORT_FORCE)
            if force != prev_force or pressed != prev_pressed:
                print(f"force={force:3d} N  pressed={pressed}  "
                      f"{'[押下]' if pressed else ''}")
                prev_force   = force
                prev_pressed = pressed
                count += 1
        except RuntimeError:
            pass
        time.sleep(0.01)
