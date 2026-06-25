"""
Blender用スクリプト: distance_sensor_test.io → obstacle_wall_a.stl + 位置情報

対応モデル: distance_sensor_test.io
  Blenderシーン上のサブモデル:
    ground_plate    … 床（基準面）
    37316c01.dat    … 距離センサー（位置・高さの参照用）
    obstacle_wall_a … 壁（センサーの前方に配置）

エクスポート方針:
  - obstacle_wall_a → obstacle_wall_a.stl（bottom_z centering）
  - センサー(37316c01.dat)のMuJoCo座標を計算して XML pos 参考値を出力
  - ground_plate はMuJoCo側でfloor geomを使うため不要

出力:
  obstacle_wall_a.stl … 壁の外形メッシュ

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で distance_sensor_test.io をインポート（Scale=1.0）
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import math
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_distance_sensor_test_log.txt"
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
print("# blender_export_distance_sensor_test_log")
print(f"# LOG_PATH: {LOG_PATH}")


# ── メッシュ収集 ──────────────────────────────────────────

def collect_mesh_descendants(root, stop_at_empty=False):
    result = []
    for child in root.children:
        if child.type == 'MESH':
            result.append(child)
            result.extend(collect_mesh_descendants(child, stop_at_empty))
        elif child.type == 'EMPTY':
            if stop_at_empty:
                print(f"    SKIP EMPTY: {child.name}")
            else:
                result.extend(collect_mesh_descendants(child, stop_at_empty))
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

def export_stl(meshes, out_filename, center_mode="bottom_z", shared_offset=None):
    if not meshes:
        print(f"  SKIP: メッシュなし → {out_filename}")
        return None

    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
    cx, cy, cz = (x0+x1)/2, (y0+y1)/2, (z0+z1)/2
    print(f"  MESHオブジェクト数: {len(meshes)}")
    print(f"  統合bbox: X[{x0:.1f},{x1:.1f}] Y[{y0:.1f},{y1:.1f}] Z[{z0:.1f},{z1:.1f}] LDU")

    if shared_offset is not None:
        dx, dy, dz = shared_offset
        print(f"  共有オフセット: ({dx:.2f}, {dy:.2f}, {dz:.2f}) LDU")
    elif center_mode == "bottom_z":
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
    print(f"  サイズ: W={w:.4f}m  D={d:.4f}m  H={h:.4f}m")
    return (x0,x1),(y0,y1),(z0,z1)


# ── LDraw→MuJoCo 座標変換 ────────────────────────────────

def ldu_to_mujoco(blender_pos, offset):
    """Blenderワールド座標 + オフセット → MuJoCo座標"""
    x = -(blender_pos.x + offset[0]) * SCALE
    y = -(blender_pos.y + offset[1]) * SCALE
    z =  (blender_pos.z + offset[2]) * SCALE
    return x, y, z


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:40s} parent={pname}{extra}")


# ── サブモデルルート取得 ──────────────────────────────────

ground_root = bpy.data.objects.get("ground_plate")
wall_root   = bpy.data.objects.get("obstacle_wall_a")
# センサーは単体パーツとして直接存在する場合とEMPTY子の場合がある
sensor_obj  = bpy.data.objects.get("37316c01.dat")

print(f"\nサブモデル検索結果:")
print(f"  ground_plate    : {'OK' if ground_root  else 'NOT FOUND'}")
print(f"  obstacle_wall_a : {'OK' if wall_root    else 'NOT FOUND'}")
print(f"  37316c01.dat    : {'OK' if sensor_obj   else 'NOT FOUND'}")


# ── ground_plateを基準オフセット計算 ─────────────────────

print("\n" + "=" * 60)
print("基準オフセット計算（ground_plate の Z底面を0とする）")
print("=" * 60)

if ground_root:
    ground_meshes = collect_mesh_descendants(ground_root)
    if ground_meshes:
        (gx0,gx1),(gy0,gy1),(gz0,gz1) = combined_bbox(ground_meshes)
        # XY中心、Z底面=0 を基準とする
        ref_offset = (-(gx0+gx1)/2, -(gy0+gy1)/2, -gz0)
        print(f"  ground_plate bbox: X[{gx0:.1f},{gx1:.1f}] Y[{gy0:.1f},{gy1:.1f}] Z[{gz0:.1f},{gz1:.1f}]")
        print(f"  基準オフセット: ({ref_offset[0]:.2f}, {ref_offset[1]:.2f}, {ref_offset[2]:.2f}) LDU")
    else:
        ref_offset = (0, 0, 0)
        print("  WARNING: ground_plate メッシュなし、オフセット=(0,0,0)")
else:
    ref_offset = (0, 0, 0)
    print("  WARNING: ground_plate が見つかりません、オフセット=(0,0,0)")


# ── obstacle_wall_a エクスポート ──────────────────────────

print("\n" + "=" * 60)
print("obstacle_wall_a 処理")
print("=" * 60)

if wall_root:
    wall_meshes = collect_mesh_descendants(wall_root)
    wall_bbox   = export_stl(
        meshes       = wall_meshes,
        out_filename = "obstacle_wall_a.stl",
        center_mode  = "bottom_z",
    )
else:
    print("  NOT FOUND")
    wall_bbox = None


# ── センサー位置計算（メッシュbboxベース） ───────────────

print("\n" + "=" * 60)
print("センサー位置計算（メッシュbbox底面をsensor_mount pos Zとする）")
print("=" * 60)

if sensor_obj:
    # センサーのメッシュbboxを取得（LDrawパーツ原点ではなく物理的な底面を使う）
    sensor_meshes_all = [sensor_obj] + [c for c in sensor_obj.children_recursive
                                         if c.type == 'MESH']
    (sbx0,sbx1),(sby0,sby1),(sbz0,sbz1) = combined_bbox(sensor_meshes_all)

    # センサー底面 MuJoCo Z = sensor_mount pos Z（bottom_z STLのZ=0が底面に対応）
    sensor_mount_z = (sbz0 + ref_offset[2]) * SCALE
    # センサー高さ中心 → site Z body local（distance_sensor_body.xmlのsite_z）
    sensor_h = (sbz1 - sbz0) * SCALE
    site_z_body = sensor_h / 2

    # テストシーンではX=0,Y=0に配置（センサーが+Y方向を向く）
    print(f"  センサー bbox: X[{sbx0:.1f},{sbx1:.1f}] Y[{sby0:.1f},{sby1:.1f}] Z[{sbz0:.1f},{sbz1:.1f}] LDU")
    print(f"  センサー底面 MuJoCo Z: {sensor_mount_z:.4f} m  （= sensor_mount pos Z）")
    print(f"  センサー高さ: {sensor_h*1000:.1f} mm  → site Z body local: {site_z_body:.4f} m")

    # センサー原点のY座標（参考）
    s_pos = sensor_obj.matrix_world.translation
    s_origin_y = -(s_pos.y + ref_offset[1]) * SCALE
else:
    sensor_mount_z = 0.050
    sensor_h = 0.023
    site_z_body = 0.012
    s_origin_y = 0.047
    print("  WARNING: センサーオブジェクト未検出、デフォルト値を使用")


# ── 壁の初期位置計算 ─────────────────────────────────────

print("\n" + "=" * 60)
print("壁の初期位置計算（センサー→壁の相対距離をIOファイルから算出）")
print("=" * 60)

if wall_root and sensor_obj:
    w_pos  = wall_root.matrix_world.translation
    s_pos  = sensor_obj.matrix_world.translation
    # IOファイルでのセンサーと壁のY方向相対距離（MuJoCo）
    # MuJoCo Y = -Blender Y なので差の絶対値を使う
    sensor_blender_y = s_pos.y
    wall_blender_y   = w_pos.y
    # センサーが検出する方向の壁の距離（正の値）
    initial_dist = abs(wall_blender_y - sensor_blender_y) * SCALE
    print(f"  センサー Blender Y: {sensor_blender_y:.2f} LDU")
    print(f"  壁     Blender Y: {wall_blender_y:.2f} LDU")
    print(f"  IOファイルでのセンサー→壁 距離: {initial_dist*1000:.1f} mm")
    # テストシーン: センサーY=0, 壁をinitial_dist前方(+Y)に配置
    wall_initial_y = initial_dist
else:
    initial_dist   = 0.175
    wall_initial_y = initial_dist
    print("  WARNING: 壁またはセンサーが未検出、デフォルト値を使用")

wall_initial_y_offset = wall_initial_y  # スライダー基準となる壁body初期Y


# ── MuJoCo XMLスニペット出力 ──────────────────────────────

print("\n" + "=" * 60)
print("MuJoCo XML スニペット（参考）")
print("=" * 60)

print(f"""
<!-- asset セクション -->
<mesh name="distance_sensor_mesh" file="meshes/distance_sensor.stl"/>
<mesh name="obstacle_wall_a_mesh" file="meshes/obstacle_wall_a.stl"/>

<!-- worldbody セクション -->

<!-- 距離センサー: sensor_mount pos Z = IOファイルから算出したセンサー底面高さ -->
<!-- site Z body local: {site_z_body:.4f} m (センサー高さ中心) -->
<body name="sensor_mount" pos="0 0 {sensor_mount_z:.4f}">
  <include file="components/distance_sensor_body.xml"/>
</body>

<!-- 壁: IOファイルでのセンサー→壁距離({initial_dist*1000:.1f}mm)を前方(+Y)に配置 -->
<!-- euler="0 0 180": LDraw→MuJoCo X軸反転による向き反転を補正 -->
<body name="wall_body" pos="0 {wall_initial_y:.4f} 0" euler="0 0 180">
  <joint name="wall_slide" type="slide" axis="0 1 0"
         range="-0.125 0.225" damping="10.0"/>
  <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
  <geom name="wall_geom" type="mesh" mesh="obstacle_wall_a_mesh"
        contype="1" conaffinity="1" rgba="0.8 0.8 0.2 0.8"/>
</body>

<!-- 注意: sensor_mount pos Zはセンサー底面高さ。
     MuJoCo floor(Z=0) = ground_plate構造体の底面。
     distance_sensor_body.xmlのsite_z を {site_z_body:.4f} に更新すること -->
""")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
