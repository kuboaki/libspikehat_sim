#!/usr/bin/env python3
"""
test_distance_sensor.py — 距離センサーテスト

ポート構成:
  D(3): 距離センサー

MuJoCo ビューアで wall_slide_ctrl スライダーを動かすと
壁が前後に移動し、距離センサーの値が変わる。

有効距離: 50mm〜300mm
範囲外または測定不能: 2000mm (DIST_INVALID)

実行方法:
  SPIKEHAT_SIM_XML=examples/test_distance_sensor.xml \\
    python3 examples/test_distance_sensor.py
"""
import sys
sys.path.insert(0, 'python')

from spikehat import SpikeHat, DEVICE_DISTANCE

DIST_INVALID = 2000

with SpikeHat() as hat:
    hat.port_config(3, DEVICE_DISTANCE)
    hat.sleep(1)

    print("=== 距離センサーテスト (20回) ===")
    print("MuJoCoビューアで wall_slide_ctrl を動かして壁を移動してください")
    print("有効距離: 50mm〜300mm")
    print()

    for i in range(20):
        try:
            mm = hat.distance_read(3)
            if mm == DIST_INVALID:
                print(f"[{i+1:2d}] 距離: ---- mm (範囲外または測定不能)")
            else:
                print(f"[{i+1:2d}] 距離: {mm:4d} mm")
        except RuntimeError:
            print(f"[{i+1:2d}] 距離: 読み取り失敗")

        hat.sleep(1)
