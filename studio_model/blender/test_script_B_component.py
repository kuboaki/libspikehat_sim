"""
Script B: コンポーネント方式（include相当）によるエクスポート

distance_sensor_test2.io をBlenderにインポートした状態で実行する。
Sensor A の形状を bottom_z centering で標準コンポーネントSTLとして1つだけ
エクスポートし、Sensor A・Sensor B それぞれの配置に必要な mount pos と
回転行列（Blender座標系）を計算する。

MuJoCo eulerへの変換は別途Python(numpy)で正確に行う
（Blenderのto_euler()は軸変換を考慮しないため、ここでは行列のみ出力する）。

実行手順:
  1. Blenderを起動し、デフォルトオブジェクトを削除
  2. File > Import > LDraw で distance_sensor_test2.ldr をインポート（Scale=1.0）
  3. 本スクリプトを実行
"""
import bpy
import os
import sys
import numpy as np
import mathutils
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/test_script_B_log.txt"
SCALE = 0.0004


class _Tee:
    def __init__(self, filepath):
        self._stdout = sys.stdout
        self._file = open(filepath, "w", encoding="utf-8")
    def write(self, text):
        self._stdout.write(text); self._file.write(text)
    def flush(self):
        self._stdout.flush(); self._file.flush()
    def close(self):
        sys.stdout = self._stdout
        self._file.close()

_tee = _Tee(LOG_PATH)
sys.stdout = _tee
print("# test_script_B_log (コンポーネント方式)")


def combined_bbox(meshes):
    xs, ys, zs = [], [], []
    for obj in meshes:
        mat = obj.matrix_world
        for v in obj.data.vertices:
            wv = mat @ v.co
            xs.append(wv.x); ys.append(wv.y); zs.append(wv.z)
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


all_37316 = [o for o in bpy.data.objects if o.name.startswith('37316c01.dat')]

def by_x(objs):
    return sorted(objs, key=lambda o: o.matrix_world.translation.x)

sensors = by_x(all_37316)
print(f"検出センサー数: {len(sensors)}")

if len(sensors) >= 2:
    sensor_A, sensor_B = sensors[0], sensors[1]
    print(f"Sensor A: {sensor_A.name}  world_pos={tuple(sensor_A.matrix_world.translation)}")
    print(f"Sensor B: {sensor_B.name}  world_pos={tuple(sensor_B.matrix_world.translation)}")

    # === 1) Sensor A の形状を bottom_z centering で標準STLとしてエクスポート ===
    (x0, x1), (y0, y1), (z0, z1) = combined_bbox([sensor_A])
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    dx, dy, dz = -cx, -cy, -z0
    print(f"\nbbox: X[{x0:.1f},{x1:.1f}] Y[{y0:.1f},{y1:.1f}] Z[{z0:.1f},{z1:.1f}] LDU")
    print(f"center_mode=bottom_z offset=({dx:.3f},{dy:.3f},{dz:.3f}) LDU")

    all_verts = []
    triangles = []
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = sensor_A.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()
    mat = sensor_A.matrix_world
    wv_list = [mat @ v.co for v in mesh.vertices]
    for v in wv_list:
        x, y, z = v.x + dx, v.y + dy, v.z + dz
        all_verts.append([-x * SCALE, -y * SCALE, z * SCALE])
    for poly in mesh.polygons:
        vlist = list(poly.vertices)
        for i in range(1, len(vlist) - 1):
            triangles.append([vlist[0], vlist[i], vlist[i + 1]])
    eval_obj.to_mesh_clear()

    stl_data = stl_mesh.Mesh(np.zeros(len(triangles), dtype=stl_mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j, vi in enumerate(tri):
            stl_data.vectors[i][j] = all_verts[vi]
    stl_data.save(os.path.join(OUTPUT_DIR, "test_component_sensor.stl"))
    print(f"Saved test_component_sensor.stl: 頂点数={len(all_verts)}")

    # === 2) 各センサーの mount pos（ワールド、Sensor A基準）と回転行列を出力 ===
    # STL(0,0,0) のセンサーローカルオフセット（standalone座標、=sensor_A自身のローカル）
    stl_local_offset = (-dx, -dy, -dz)  # = (cx, cy, z0)

    def compute_world_mount(obj, stl_offset_local):
        R = obj.matrix_world.to_3x3()
        offset = mathutils.Vector(stl_offset_local)
        stl_world = R @ offset + obj.matrix_world.translation
        return stl_world, R

    ref = sensor_A.matrix_world.translation  # Sensor Aのワールド位置を原点とする

    print("\n" + "=" * 60)
    print("マウント座標・回転行列")
    print("=" * 60)
    for name, obj in [("A", sensor_A), ("B", sensor_B)]:
        stl_world, R = compute_world_mount(obj, stl_local_offset)
        dx_, dy_, dz_ = stl_world.x - ref.x, stl_world.y - ref.y, stl_world.z - ref.z
        mj_x, mj_y, mj_z = -dx_ * SCALE, -dy_ * SCALE, dz_ * SCALE
        print(f"\nSensor {name}:")
        print(f"  mount pos (Sensor A基準 MuJoCo) = ({mj_x:.4f}, {mj_y:.4f}, {mj_z:.4f})")
        print(f"  Blender回転行列 row0={[round(v,4) for v in R[0]]}")
        print(f"               row1={[round(v,4) for v in R[1]]}")
        print(f"               row2={[round(v,4) for v in R[2]]}")
else:
    print("ERROR: センサーが2個見つかりません")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
