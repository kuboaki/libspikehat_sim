"""
Blender用スクリプト: distance_sensor.io → distance_sensor.stl

対応モデル: distance_sensor.io
  Blenderシーン上のオブジェクト:
    （ルートEMPTY）
      └── 37316c01.dat 系MESHオブジェクト群

座標変換:
  mj_x = -blender_x * SCALE
  mj_y = -blender_y * SCALE
  mj_z =  blender_z * SCALE

センサー検出面:
  LDraw -Z（パーツ前面）→ Blender -Y → MuJoCo +Y
  standalone（identity回転）での body local 検出面方向 = +Y
  → euler="0 0 0" で検出面が世界+Y（前方）を向く

出力:
  distance_sensor.stl … センサー外形（XY中心・Z底面=0）

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で distance_sensor.io をインポート（Scale=1.0）
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import math
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_distance_sensor_log.txt"
SCALE = 0.0004  # LDU → m


# ── ログ ─────────────────────────────────────────────────

class _Tee:
    def __init__(self, filepath):
        self._stdout = sys.stdout
        self._file   = open(filepath, "w", encoding="utf-8")
    def write(self, text):
        self._stdout.write(text)
        self._file.write(text)
    def flush(self):
        self._stdout.flush()
        self._file.flush()
    def close(self):
        sys.stdout = self._stdout
        self._file.close()

_tee = _Tee(LOG_PATH)
sys.stdout = _tee
print("# blender_export_distance_sensor_log")
print(f"# LOG_PATH: {LOG_PATH}")


# ── メッシュ収集 ──────────────────────────────────────────

def collect_mesh_descendants(root):
    result = []
    for child in root.children:
        if child.type == 'MESH':
            result.append(child)
            result.extend(collect_mesh_descendants(child))
        elif child.type == 'EMPTY':
            result.extend(collect_mesh_descendants(child))
    return result


# ── バウンディングボックス ────────────────────────────────

def combined_bbox(meshes):
    xs, ys, zs = [], [], []
    for obj in meshes:
        mat = obj.matrix_world
        for v in obj.data.vertices:
            wv = mat @ v.co
            xs.append(wv.x); ys.append(wv.y); zs.append(wv.z)
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


# ── STL エクスポート ──────────────────────────────────────

def export_stl(meshes, out_filename, center_mode="bottom_z"):
    if not meshes:
        print("  ERROR: メッシュリストが空です")
        return None

    print(f"  MESHオブジェクト数: {len(meshes)}")

    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
    cx, cy, cz = (x0+x1)/2, (y0+y1)/2, (z0+z1)/2
    print(f"  統合bbox (変換前):")
    print(f"    X[{x0:.1f}, {x1:.1f}]  Y[{y0:.1f}, {y1:.1f}]  Z[{z0:.1f}, {z1:.1f}] LDU")

    if center_mode == "bottom_z":
        dx, dy, dz = -cx, -cy, -z0
    else:
        dx, dy, dz = -cx, -cy, -cz
    print(f"  オフセット: ({dx:.2f}, {dy:.2f}, {dz:.2f}) LDU")

    all_verts = []
    triangles = []
    v_offset  = 0

    depsgraph = bpy.context.evaluated_depsgraph_get()

    for obj in meshes:
        eval_obj = obj.evaluated_get(depsgraph)
        mesh     = eval_obj.to_mesh()
        mat      = obj.matrix_world
        wv_list  = [mat @ v.co for v in mesh.vertices]

        for v in wv_list:
            x, y, z = v.x + dx, v.y + dy, v.z + dz
            all_verts.append([-x * SCALE, -y * SCALE, z * SCALE])

        for poly in mesh.polygons:
            vlist = list(poly.vertices)
            for i in range(1, len(vlist) - 1):
                triangles.append([v_offset + vlist[0],
                                   v_offset + vlist[i],
                                   v_offset + vlist[i + 1]])
        v_offset += len(wv_list)
        eval_obj.to_mesh_clear()

    stl_data = stl_mesh.Mesh(np.zeros(len(triangles), dtype=stl_mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j, vi in enumerate(tri):
            stl_data.vectors[i][j] = all_verts[vi]

    filepath = os.path.join(OUTPUT_DIR, out_filename)
    stl_data.save(filepath)
    print(f"  Saved: {filepath}")
    print(f"  頂点数: {len(all_verts)}  三角面数: {len(triangles)}")

    w = (x1 - x0) * SCALE
    d = (y1 - y0) * SCALE
    h = (z1 - z0) * SCALE
    print(f"  MuJoCo上のサイズ(参考): W={w:.4f}m  D={d:.4f}m  H={h:.4f}m")

    # 検出面位置の計算
    # Blender -Y (y0, 最小値) = LDraw -Z（センサー前面＝検出面）→ MuJoCo +Y
    # bottom_z centering後: MuJoCo Y of detection face = -(y0 + dy) * SCALE
    # ── 検出面の3D重心からsite位置を計算 ──────────────────
    # 検出面 = Blender Y_min 付近の頂点群（LDraw -Z = センサー前面）
    FACE_TOL    = 2.0
    SITE_RADIUS = 0.005

    face_verts = []
    for obj in meshes:
        eval_obj2 = obj.evaluated_get(depsgraph)
        mesh2     = eval_obj2.to_mesh()
        mat2      = obj.matrix_world
        for v in mesh2.vertices:
            wv = mat2 @ v.co
            if wv.y <= y0 + FACE_TOL:
                face_verts.append((wv.x + dx, wv.y + dy, wv.z + dz))
        eval_obj2.to_mesh_clear()

    if face_verts:
        fc_x = sum(v[0] for v in face_verts) / len(face_verts)
        fc_y = sum(v[1] for v in face_verts) / len(face_verts)
        fc_z = sum(v[2] for v in face_verts) / len(face_verts)
        site_x     = -fc_x * SCALE
        site_y_face = -fc_y * SCALE
        site_z     =  fc_z * SCALE
        site_y     = site_y_face - SITE_RADIUS
        print(f"  検出面頂点数: {len(face_verts)}  重心(Blender): ({fc_x:.2f}, {fc_y:.2f}, {fc_z:.2f}) LDU")
        print(f"  site body local MuJoCo: X={site_x:.4f} Y_face={site_y_face:.4f} Z={site_z:.4f} m")
        print(f"  site Y (内側{SITE_RADIUS*1000:.0f}mm): {site_y:.4f} m")
    else:
        site_x, site_y, site_z = 0.0, -(y0+dy)*SCALE - SITE_RADIUS, (z0+z1)/2*SCALE
        print("  WARNING: 検出面頂点未検出、近似値を使用")

    print(f"  → distance_site pos=\"{site_x:.4f} {site_y:.4f} {site_z:.4f}\"")
    return site_x, site_y, site_z


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:40s} parent={pname}{extra}")


# ── メッシュ収集 ──────────────────────────────────────────

roots = [o for o in bpy.data.objects if o.parent is None and o.type == 'EMPTY']
print(f"\nルートEMPTYオブジェクト: {[r.name for r in roots]}")

all_meshes = []
for root in roots:
    all_meshes.extend(collect_mesh_descendants(root))

print(f"収集MESHオブジェクト数: {len(all_meshes)}")


# ── distance_sensor STL エクスポート ──────────────────────

print("\n" + "=" * 60)
print("distance_sensor 処理")
print("=" * 60)

result = export_stl(
    meshes       = all_meshes,
    out_filename = "distance_sensor.stl",
    center_mode  = "bottom_z",
)

if result:
    sx_str = f"{result[0]:.4f}"
    sy_str = f"{result[1]:.4f}"
    sz_str = f"{result[2]:.4f}"
else:
    sx_str, sy_str, sz_str = "0.0000", "0.0108", "0.0116"


# ── MuJoCo XMLスニペット出力 ──────────────────────────────

print("\n" + "=" * 60)
print("MuJoCo XML スニペット（参考）")
print("=" * 60)
print(f"""
<!-- distance_sensor_body.xml に記述するスニペット -->
<!-- センサーの向き:
     STLはStudio標準向き（検出面がbody local +Y方向）
     euler="0 0 0" で検出面が世界+Y（前方）を向く
     distance_site pos Y = 検出面から5mm内側（自動計算値: {site_y_str}m） -->
<body name="distance_sensor" euler="0 0 0">
  <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>
  <geom name="distance_sensor_geom" type="mesh" mesh="distance_sensor_mesh"
        contype="0" conaffinity="0" rgba="0.3 0.3 0.3 1"/>
  <!-- site位置は検出面3D重心から算出（部品形状に忠実・再利用可能） -->
  <site name="distance_site" pos="{sx_str} {sy_str} {sz_str}" size="0.005" rgba="1 0 0 1"/>
</body>
""")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
