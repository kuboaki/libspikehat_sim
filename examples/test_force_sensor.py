#!/usr/bin/env python3
"""
test_force_sensor.py — フォースセンサーテスト（Python版）

ポート構成:
  B(1): フォースセンサー

実機での実行方法:
  cd examples && python3 test_force_sensor.py

シムでの実行方法:
  cd examples && SPIKEHAT_SIM_XML=test_force_sensor.xml \\
    python3 test_force_sensor.py

テスト内容:
  1. force_read       : force[N] と pressed を同時に取得
  2. force_is_pressed : タッチ判定のみ取得
  3. force_get_force  : 力[N]のみ取得
"""
import sys
import time
sys.path.insert(0, '../python')

from spikehat import SpikeHat, DEVICE_FORCE

PORT_FORCE  = 1    # ポートB
LOOP_COUNT  = 5    # 各テストの繰り返し回数
INTERVAL    = 0.5  # 秒

with SpikeHat() as hat:
    hat.port_config(PORT_FORCE, DEVICE_FORCE)
    time.sleep(1)

    # ── テスト1: force_read ──────────────────────────────
    print("=== テスト1: force_read (force と pressed を同時取得) ===")
    print("センサーを押してみてください\n")
    for i in range(LOOP_COUNT):
        try:
            force, pressed = hat.force_read(PORT_FORCE)
            print(f"[{i+1}] force={force:3d} N  pressed={int(pressed)}  "
                  f"{'[押下]' if pressed else ''}")
        except RuntimeError:
            print(f"[{i+1}] 読み取り失敗")
        time.sleep(INTERVAL)

    # ── テスト2: force_is_pressed ────────────────────────
    print("\n=== テスト2: force_is_pressed (タッチ判定のみ) ===")
    print("センサーを押してみてください\n")
    for i in range(LOOP_COUNT):
        try:
            pressed = hat.force_is_pressed(PORT_FORCE)
            print(f"[{i+1}] pressed={int(pressed)}  "
                  f"{'[押下]' if pressed else ''}")
        except RuntimeError:
            print(f"[{i+1}] 読み取り失敗")
        time.sleep(INTERVAL)

    # ── テスト3: force_get_force ─────────────────────────
    print("\n=== テスト3: force_get_force (力[N]のみ) ===")
    print("センサーを押してみてください\n")
    for i in range(LOOP_COUNT):
        try:
            force = hat.force_get_force(PORT_FORCE)
            print(f"[{i+1}] force={force:3d} N")
        except RuntimeError:
            print(f"[{i+1}] 読み取り失敗")
        time.sleep(INTERVAL)

    print("\n完了")
