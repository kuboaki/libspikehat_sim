#!/usr/bin/env python3
"""モーターテスト (Python版)"""
import sys
sys.path.insert(0, '../python')
from spikehat import SpikeHat, DEVICE_MOTOR_M

PORT_MOTOR  = 0   # ポートA: Mアンギュラーモーター
ALIGN_SPEED = 10  # 初期位置合わせの速度
RUN_SPEED   = 5   # motor_start テストの速度


def return_to_origin(hat, port):
    """現在位置から原点(0度)へ戻す（初期位置合わせ）"""
    cur_pos = hat.motor_get_position(port)
    if cur_pos == 0:
        return

    print(f"初期位置合わせ: 現在位置 {cur_pos} 度 -> 0 度")
    hat.motor_run_for_degrees(port, -cur_pos, ALIGN_SPEED)

    dur = (abs(cur_pos) / 360.0) / (ALIGN_SPEED * 0.05)
    if dur < 0.5:
        dur = 0.5
    hat.sleep(dur + 0.5)


def print_status(hat, port):
    try:
        print(f"速度: {hat.motor_get_speed(port)}")
        print(f"位置: {hat.motor_get_position(port)} 度")
    except RuntimeError as e:
        print(f"フィードバックなし: {e}")


with SpikeHat() as hat:
    hat.port_config(PORT_MOTOR, DEVICE_MOTOR_M)
    hat.sleep(1.0)

    # ── run_for_seconds (PID速度制御 + 時間指定) ──────────────
    print("=== モーターテスト (run_for_seconds) ===")
    return_to_origin(hat, PORT_MOTOR)
    
    print("速度5で2秒間回転...")
    hat.motor_run_for_seconds(PORT_MOTOR, 2.0, 5)
    hat.sleep(10.0)
    print_status(hat, PORT_MOTOR)

    print("速度-3で2秒間回転...")
    hat.motor_run_for_seconds(PORT_MOTOR, 2.0, -3)
    hat.sleep(10.0)
    print_status(hat, PORT_MOTOR)
    
    hat.motor_coast(PORT_MOTOR)

    # ── motor_start + motor_stop ──────────────────────────────
    print("\n=== モーターテスト (motor_start -> motor_stop) ===")
    return_to_origin(hat, PORT_MOTOR)

    print(f"速度{RUN_SPEED}でstart, 5秒後にstop...")
    hat.motor_start(PORT_MOTOR, RUN_SPEED)
    hat.sleep(5.0)
    hat.motor_stop(PORT_MOTOR)
    print_status(hat, PORT_MOTOR)

    # ── motor_start + motor_coast ─────────────────────────────
    print("\n=== モーターテスト (motor_start -> motor_coast) ===")
    return_to_origin(hat, PORT_MOTOR)

    print(f"速度{RUN_SPEED}でstart, 5秒後にcoast...")
    hat.motor_start(PORT_MOTOR, RUN_SPEED)
    hat.sleep(5.0)
    hat.motor_coast(PORT_MOTOR)
    print_status(hat, PORT_MOTOR)

    # ── motor_pwm + motor_stop ─────────────────────────────
    print("\n=== モーターテスト (motor_pwm -> motor_stop) ===")
    return_to_origin(hat, PORT_MOTOR)

    print(f"pwm 0.1で回転, 2秒後にstop...")
    hat.motor_pwm(PORT_MOTOR, 0.1)
    hat.sleep(2.0)
    hat.motor_stop(PORT_MOTOR)
    print_status(hat, PORT_MOTOR)

    print("\n完了")
