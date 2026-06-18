#!/usr/bin/env python3
"""
test_motor_viewer2.py — SPIKE互換APIを経由したモーター回転方向テスト

SpikeHat API (motor_start) を経由してモーターを制御し、
MuJoCo ビューアで回転方向を目視確認する。

  正のspeed → 時計回り（CW）なら実機と一致 ✓
  正のspeed → 反時計回り（CCW）なら修正が必要 ✗

実行方法:
  cd /Users/kuboaki/Projects/libspikehat_sim
  PYTHONPATH=python uv run mjpython examples/test_motor_viewer2.py
"""
import sys
import os
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
from spikehat import SpikeHat, DEVICE_MOTOR_L

XML_PATH = os.path.join(os.path.dirname(__file__), "test_motor.xml")

# ── ビューア用のモデル/データ（表示専用） ──
viewer_model = mujoco.MjModel.from_xml_path(XML_PATH)
viewer_data  = mujoco.MjData(viewer_model)

joint_id = mujoco.mj_name2id(viewer_model, mujoco.mjtObj.mjOBJ_JOINT, "motor_joint")
qpos_adr = viewer_model.jnt_qposadr[joint_id]
qvel_adr = viewer_model.jnt_dofadr[joint_id]

# ── SpikeHat（実際の制御・物理計算） ──
hat = SpikeHat(XML_PATH)
hat.port_config(0, DEVICE_MOTOR_L)

# ── テストシーケンス ──
# Phase 0: speed=+50 で5秒（CWを期待）
# Phase 1: speed=-50 で5秒（CCWを期待）
# Phase 2: 停止
SPEED_POS  =  50
SPEED_NEG  = -50
PHASE_STEPS = int(5.0 / viewer_model.opt.timestep)

phase      = 0
step_count = 0
prev_deg   = None

print("=== モーター回転方向テスト ===")
print(f"  Phase 0 (0〜5s) : speed={SPEED_POS:+d}  → 時計回り(CW)が正解")
print(f"  Phase 1 (5〜10s): speed={SPEED_NEG:+d} → 反時計回り(CCW)が正解")
print(f"  Phase 2 (10s〜) : 停止")
print()

with mujoco.viewer.launch_passive(viewer_model, viewer_data) as viewer:
    while viewer.is_running():

        # SpikeHat API でモーターを1ステップ進める
        if phase == 0:
            hat.motor_start(0, SPEED_POS)
        elif phase == 1:
            hat.motor_start(0, SPEED_NEG)
        else:
            hat.motor_stop(0)

        # SpikeHat の qpos をビューア用データにコピーして表示を更新
        viewer_data.qpos[qpos_adr] = hat.sim_get_qpos(qpos_adr)
        mujoco.mj_forward(viewer_model, viewer_data)
        viewer.sync()

        step_count += 1
        if step_count >= PHASE_STEPS:
            phase += 1
            step_count = 0
            if phase == 1:
                print(f"--- Phase 1: speed={SPEED_NEG:+d} (CCW方向へ) ---")
            elif phase == 2:
                print("--- Phase 2: 停止 ---")

        # 1度以上変化したら位置・速度を表示
        deg = viewer_data.qpos[qpos_adr] * 180.0 / 3.141592653589793
        vel = viewer_data.qvel[qvel_adr] * 180.0 / 3.141592653589793
        pos = hat.motor_get_position(0)
        if prev_deg is None or abs(deg - prev_deg) >= 1.0:
            print(f"qpos: {deg:+7.1f} deg  vel: {vel:+7.1f} deg/s  "
                  f"position_deg: {pos:+5d} deg")
            prev_deg = deg

hat.close()
