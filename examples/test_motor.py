#!/usr/bin/env python3
"""モーターテスト (Python版)"""
import sys, time
sys.path.insert(0, '../python')
from spikehat import SpikeHat, DEVICE_MOTOR_M

with SpikeHat() as hat:
    hat.port_config(0, DEVICE_MOTOR_M)
    time.sleep(1)

    print("=== モーターテスト ===")

    print("速度5で2秒間回転...")
    hat.motor_run_for_seconds(0, 2.0, 5)
    time.sleep(4)

    try:
        print(f"速度: {hat.motor_get_speed(0)}")
        print(f"位置: {hat.motor_get_position(0)} 度")
    except RuntimeError as e:
        print(f"フィードバックなし: {e}")

    print("速度-3で2秒間回転...")
    hat.motor_run_for_seconds(0, 2.0, -3)
    time.sleep(3)

    hat.motor_coast(0)
    print("完了")
