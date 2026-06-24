"""
Blender用スクリプト: color_sensor.io → color_sensor.stl

対応モデル: color_sensor.io
  Blenderシーン上のオブジェクト:
    （ルートEMPTY）
      └── 37308c01.dat 系MESHオブジェクト群

座標系:
  LDraw座標系（Y下向き）から MuJoCo座標系（Z上向き）に変換する。
    mj_x = -ldu_x * SCALE
    mj_y = -ldu_y * SCALE
    mj_z =  ldu_z * SCALE

出力:
  color_sensor.stl … センサー外形（XY中心・Z底面=0）

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で color_sensor.io をインポート（Scale=1.0）
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import math
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_color_sensor_log.txt"
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
print("# blender_export_color_sensor_log")
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
    """
    center_mode: 'bottom_z' = XY中心 + Z底面を0
                 'center'   = XYZ中心を0
    """
    if not meshes:
        print("  ERROR: メッシュリストが空です")
        return

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
    site_y_face = -(y0 + dy) * SCALE
    # siteはセンサーボディ内側（検出面から球半径=0.005m分だけ内側）に配置する
    # → センサーを低い位置に配置しても、siteが対象物に干渉しない
    SITE_RADIUS = 0.005
    site_y = site_y_face - SITE_RADIUS
    print(f"  検出面 MuJoCo body local Y: {site_y_face:.4f} m  (Blender Y_min={y0:.1f} LDU)")
    print(f"  site Y (内側{SITE_RADIUS*1000:.0f}mm): {site_y:.4f} m")
    print(f"  → color_site pos=\"0 {site_y:.4f} Z_center\"  (euler=\"-90 0 0\"で世界-Z=下向き)")
    return site_y


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:40s} parent={pname}{extra}")


# ── メッシュ収集 ──────────────────────────────────────────

# インポートされたシーンのルートEMPTYを探す（.ioファイル名のオブジェクト）
roots = [o for o in bpy.data.objects if o.parent is None and o.type == 'EMPTY']
print(f"\nルートEMPTYオブジェクト: {[r.name for r in roots]}")

all_meshes = []
for root in roots:
    all_meshes.extend(collect_mesh_descendants(root))

print(f"収集MESHオブジェクト数: {len(all_meshes)}")

# ── color_sensor STL エクスポート ─────────────────────────

print("\n" + "=" * 60)
print("color_sensor 処理")
print("=" * 60)

site_y = export_stl(
    meshes       = all_meshes,
    out_filename = "color_sensor.stl",
    center_mode  = "bottom_z",
)

# ── MuJoCo XMLスニペット出力 ──────────────────────────────

print("\n" + "=" * 60)
print("MuJoCo XML スニペット（参考）")
print("=" * 60)
site_y_str = f"{site_y:.4f}" if site_y is not None else "-0.0118"
print(f"""
<!-- color_sensor_body.xml に記述するスニペット -->
<!-- センサーの向き:
     STLはStudio標準向き（検出面がbody local +Y方向）
     euler="-90 0 0" で検出面を世界-Z（下向き）に回転
     color_site pos Y = 検出面のbody local Y座標（自動計算値: {site_y_str}m）-->
<body name="color_sensor_body" pos="0 -0.012 0.034" euler="-90 0 0">
  <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>
  <geom name="color_sensor_geom" type="mesh" mesh="color_sensor_mesh"
        contype="0" conaffinity="0" rgba="0.3 0.3 0.3 1"/>
  <!-- Z_centerはセンサーSTLのZ範囲中心(≈half_Z)で補正: world Y=0に揃える -->
  <site name="color_site" pos="0 {site_y_str} Z_center" size="0.005" rgba="1 1 0 1"/>
</body>
""")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
