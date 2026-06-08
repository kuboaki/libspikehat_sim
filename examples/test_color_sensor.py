#!/usr/bin/env python3
"""
test_color_sensor.py — カラーセンサーテスト

ポート構成:
  C(2): カラーセンサー

MuJoCo ビューアで sensor_slide_ctrl スライダーを動かすと
センサーが左右に移動し、各色ブロックの上でHSV値が変わる。

色ブロックの配置（左から右）:
  赤 → 青 → 黄 → 白 → 黒 → 緑

実行方法:
  SPIKEHAT_SIM_XML=examples/test_color_sensor.xml \\
    python3 examples/test_color_sensor.py
"""
import sys
import time
sys.path.insert(0, 'python')

from spikehat import SpikeHat, DEVICE_COLOR

with SpikeHat() as hat:
    hat.port_config(2, DEVICE_COLOR)
    time.sleep(1)

    print("=== カラーセンサーテスト (20回) ===")
    print("MuJoCoビューアで sensor_slide_ctrl を動かしてセンサーを移動してください")
    print("色ブロック配置（左→右）: 赤 / 青 / 黄 / 白 / 黒 / 緑")
    print()

    for i in range(20):
        try:
            h, s, v = hat.color_read_hsv(2)
            print(f"[{i+1:2d}] 色(HSV): hue={h:3d}  sat={s:3d}  val={v:3d}")
        except RuntimeError:
            print(f"[{i+1:2d}] 色: 読み取り失敗")

        time.sleep(1)
