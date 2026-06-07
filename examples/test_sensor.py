#!/usr/bin/env python3
"""センサーテスト
ポート構成:
  A(0): (未使用 or モーター)
  B(1): フォースセンサー
  C(2): カラーセンサー
  D(3): 距離センサー
"""
import sys, time
sys.path.insert(0, '../python')
from spikehat import SpikeHat, DEVICE_FORCE, DEVICE_COLOR, DEVICE_DISTANCE

with SpikeHat() as hat:
    hat.port_config(1, DEVICE_FORCE)
    hat.port_config(2, DEVICE_COLOR)
    hat.port_config(3, DEVICE_DISTANCE)
    time.sleep(2)

    print("=== センサーテスト (10回) ===")
    for _ in range(10):
        try:
            mm = hat.distance_read(3)
            print(f"距離: {mm:4d} mm", end="  ")
        except RuntimeError:
            print("距離: ----   ", end="  ")

        try:
            h, s, v = hat.color_read_hsv(2)
            print(f"色(HSV): {h:3d}/{s:3d}/{v:3d}", end="  ")
        except RuntimeError:
            print("色: --------  ", end="  ")

        try:
            force, pressed = hat.force_read(1)
            print(f"力: {force:2d} N  {'[押下]' if pressed else '      '}")
        except RuntimeError:
            print("力: ----")

        time.sleep(1)
