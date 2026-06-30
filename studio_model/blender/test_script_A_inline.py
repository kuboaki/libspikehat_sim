"""
Script A: インライン展開によるエクスポート

distance_sensor_test2.io をBlenderにインポートした状態で実行する。
Sensor A(0°)・Sensor B(5°、Y軸回転)をそれぞれ独立したSTLとして、
ワールド座標の回転をメッシュに直接焼き込んでエクスポートする。
（sonar_radarのblender_export.pyと同じ「インライン」方式）

実行手順:
  1. Blenderを起動し、デフォルトオブジェクトを削除
  2. File > Import > LDraw で distance_sensor_test2.ldr をインポート（Scale=1.0）
  3. 本スクリプトを実行
"""
import bpy
import os
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/test_script_A_log.txt"
SCALE = 0.0004


class _Tee:
    def __init__(self, filepath):
        self._stdout = __import__('sys').stdout
        self._file = open(filepath, "w", encoding="utf-8")
    def write(self, text):
        self._stdout.write(text); self._file.write(text)
    def flush(self):
        self._stdout.flush(); self._file.flush()
    def close(self):
        __import__('sys').stdout = self._stdout
        self._file.close()

import sys
_tee = _Tee(LOG_PATH)
sys.stdout = _tee
print("# test_script_A_log (inline方式)")


def export_combined(meshes, out_filename, ref_pos):
    rx, ry, rz = ref_pos
    all_verts = []
    triangles = []
    v_offset = 0
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in meshes:
        eval_obj = obj.evaluated_get(depsgraph)
        mesh = eval_obj.to_mesh()
        mat = obj.matrix_world
        wv_list = [mat @ v.co for v in mesh.vertices]
        for v in wv_list:
            x, y, z = v.x - rx, v.y - ry, v.z - rz
            all_verts.append([-x * SCALE, -y * SCALE, z * SCALE])
        for poly in mesh.polygons:
            vlist = list(poly.vertices)
            for i in range(1, len(vlist) - 1):
                triangles.append([v_offset + vlist[0], v_offset + vlist[i], v_offset + vlist[i + 1]])
        v_offset += len(wv_list)
        eval_obj.to_mesh_clear()

    stl_data = stl_mesh.Mesh(np.zeros(len(triangles), dtype=stl_mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j, vi in enumerate(tri):
            stl_data.vectors[i][j] = all_verts[vi]
    stl_data.save(os.path.join(OUTPUT_DIR, out_filename))
    print(f"  Saved {out_filename}: 頂点数={len(all_verts)}")


print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    print(f"  [{o.type:5s}] {o.name:30s} pos={tuple(round(v,2) for v in o.matrix_world.translation)}")

all_37316 = [o for o in bpy.data.objects if o.name.startswith('37316c01.dat')]
all_3001  = [o for o in bpy.data.objects if o.name.startswith('3001.dat')]

def by_x(objs):
    return sorted(objs, key=lambda o: o.matrix_world.translation.x)

sensors = by_x(all_37316)
markers = by_x(all_3001)
print(f"\n検出センサー数: {len(sensors)}  マーカー数: {len(markers)}")

if len(sensors) >= 2 and len(markers) >= 2:
    sensor_A, sensor_B = sensors[0], sensors[1]
    marker_A, marker_B = markers[0], markers[1]

    print(f"\nSensor A: {sensor_A.name}  world_pos={tuple(sensor_A.matrix_world.translation)}")
    print(f"Sensor B: {sensor_B.name}  world_pos={tuple(sensor_B.matrix_world.translation)}")

    # 基準点: Sensor Aのワールド位置をMuJoCo原点とする
    ref = tuple(sensor_A.matrix_world.translation)

    print("\n" + "=" * 60)
    print("test_inline_A.stl エクスポート（Sensor A + Marker A）")
    print("=" * 60)
    export_combined([sensor_A, marker_A], "test_inline_A.stl", ref)

    print("\n" + "=" * 60)
    print("test_inline_B.stl エクスポート（Sensor B + Marker B）")
    print("=" * 60)
    export_combined([sensor_B, marker_B], "test_inline_B.stl", ref)

    # 参考: Sensor Bのワールド位置（Sensor A基準のdome-local MuJoCo座標）
    bx, by, bz = sensor_B.matrix_world.translation
    rx, ry, rz = ref
    dx, dy, dz = bx-rx, by-ry, bz-rz
    print(f"\nSensor B の Sensor A基準 dome-local MuJoCo pos: "
          f"({-dx*SCALE:.4f}, {-dy*SCALE:.4f}, {dz*SCALE:.4f})")
else:
    print("ERROR: センサーまたはマーカーが2個ずつ見つかりません")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
