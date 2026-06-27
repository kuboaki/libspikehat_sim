"""
Blender用スクリプト: force_sensor.io → force_sensor_body.stl + button.stl

対応モデル: force_sensor.io
  Blenderシーン上のオブジェクト:
    （ルートEMPTY）
      ├── カスタムパーツA（センサー本体 = 低いZ位置）
      └── カスタムパーツB（ボタン = 高いZ位置）

エクスポート方針:
  - 全パーツの統合bboxからXY中心・Z底面=0の共有オフセットを計算
  - force_sensor_body.stl: センサー本体メッシュ（共有オフセット適用）
  - button.stl: ボタンメッシュ（共有オフセット適用）
  → 両STLは同じ座標原点を持ち、MuJoCo上で正しい相対位置に配置される

押し込み方向:
  LDraw Z → Blender Y → 無関係（高さ方向は LDraw Y → Blender Z）
  ボタン押し込み = Blender -Z方向 = MuJoCo -Z方向（下向き）
  joint axis="0 0 -1"

出力:
  force_sensor_body.stl ... センサー本体外形
  button.stl            ... ボタン外形

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で force_sensor.io をインポート（Scale=1.0）
     Additional Library Path にカスタムパーツのパスを設定すること
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_force_sensor_log.txt"
SCALE = 0.0004  # LDU → m
FACE_TOL    = 2.0   # 検出面とみなすZ方向許容幅 [LDU]（ボタン上面）
SITE_RADIUS = 0.005  # site ボディ内側距離 [m]


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
print("# blender_export_force_sensor_log")


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

def mesh_z_center(meshes):
    (_, _), (_, _), (z0, z1) = combined_bbox(meshes)
    return (z0 + z1) / 2


# ── STL エクスポート ──────────────────────────────────────

def export_stl(meshes, out_filename, shared_offset):
    dx, dy, dz = shared_offset
    print(f"  MESHオブジェクト数: {len(meshes)}")

    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
    print(f"  bbox: X[{x0:.1f},{x1:.1f}] Y[{y0:.1f},{y1:.1f}] Z[{z0:.1f},{z1:.1f}] LDU")

    all_verts, triangles, v_offset = [], [], 0
    depsgraph = bpy.context.evaluated_depsgraph_get()

    for obj in meshes:
        eval_obj = obj.evaluated_get(depsgraph)
        mesh     = eval_obj.to_mesh()
        mat      = obj.matrix_world
        wv_list  = [mat @ v.co for v in mesh.vertices]
        for v in wv_list:
            all_verts.append([-(v.x+dx)*SCALE, -(v.y+dy)*SCALE, (v.z+dz)*SCALE])
        for poly in mesh.polygons:
            vlist = list(poly.vertices)
            for i in range(1, len(vlist)-1):
                triangles.append([v_offset+vlist[0], v_offset+vlist[i], v_offset+vlist[i+1]])
        v_offset += len(wv_list)
        eval_obj.to_mesh_clear()

    stl_data = stl_mesh.Mesh(np.zeros(len(triangles), dtype=stl_mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j, vi in enumerate(tri):
            stl_data.vectors[i][j] = all_verts[vi]

    filepath = os.path.join(OUTPUT_DIR, out_filename)
    stl_data.save(filepath)
    print(f"  Saved: {filepath}  (頂点:{len(all_verts)} 面:{len(triangles)})")
    w = (x1-x0)*SCALE; d = (y1-y0)*SCALE; h = (z1-z0)*SCALE
    print(f"  サイズ: W={w:.4f}m D={d:.4f}m H={h:.4f}m")
    return (x0,x1),(y0,y1),(z0,z1)


# ── ボタン上面の3D重心計算（force_site位置） ─────────────

def compute_button_top_site(button_meshes, shared_offset, z1_button):
    """ボタン上面（Z_max付近）頂点の3D重心からforce_site位置を計算"""
    dx, dy, dz = shared_offset
    face_verts = []
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in button_meshes:
        eval_obj = obj.evaluated_get(depsgraph)
        mesh     = eval_obj.to_mesh()
        mat      = obj.matrix_world
        for v in mesh.vertices:
            wv = mat @ v.co
            if wv.z >= z1_button - FACE_TOL:  # ボタン上面付近
                face_verts.append((wv.x+dx, wv.y+dy, wv.z+dz))
        eval_obj.to_mesh_clear()

    if not face_verts:
        return None
    fc_x = sum(v[0] for v in face_verts) / len(face_verts)
    fc_y = sum(v[1] for v in face_verts) / len(face_verts)
    fc_z = sum(v[2] for v in face_verts) / len(face_verts)
    # MuJoCo body local座標（shared_offset適用後）
    mj_x = -fc_x * SCALE
    mj_y = -fc_y * SCALE
    mj_z =  fc_z * SCALE
    print(f"  ボタン上面頂点数: {len(face_verts)}  重心(Blender): ({fc_x:.2f},{fc_y:.2f},{fc_z:.2f}) LDU")
    print(f"  force_site body local MuJoCo: ({mj_x:.4f}, {mj_y:.4f}, {mj_z:.4f}) m")
    # site はボタン上面から SITE_RADIUS 内側（MuJoCo Z を少し下げる）
    site_z = mj_z - SITE_RADIUS
    print(f"  force_site Z (内側{SITE_RADIUS*1000:.0f}mm): {site_z:.4f} m")
    return mj_x, mj_y, site_z


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "="*60)
print("シーン内オブジェクト一覧")
print("="*60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:50s} parent={pname}{extra}")


# ── メッシュ収集と body/button の分類 ─────────────────────

roots = [o for o in bpy.data.objects if o.parent is None and o.type == 'EMPTY']
all_mesh_objs = []
for root in roots:
    all_mesh_objs.extend(collect_mesh_descendants(root))

print(f"\n収集MESHオブジェクト数: {len(all_mesh_objs)}")

# Z中心でソート: 低い方 = センサー本体、高い方 = ボタン
sorted_by_z = sorted(all_mesh_objs, key=lambda o: mesh_z_center([o]))
if len(sorted_by_z) < 2:
    print("ERROR: メッシュが2つ未満です")
    _tee.close()
    raise SystemExit("メッシュ不足")

body_meshes   = [sorted_by_z[0]]   # 最もZ中心が低い = センサー本体
button_meshes = sorted_by_z[1:]    # それより高い = ボタン（1つのはず）

print(f"  センサー本体: {[m.name for m in body_meshes]}")
print(f"  ボタン      : {[m.name for m in button_meshes]}")


# ── 共有オフセット計算（全パーツのZ底面=0） ──────────────

print("\n" + "="*60)
print("共有オフセット計算（全パーツ統合bbox）")
print("="*60)

all_meshes = body_meshes + button_meshes
(ax0,ax1),(ay0,ay1),(az0,az1) = combined_bbox(all_meshes)
cx, cy = (ax0+ax1)/2, (ay0+ay1)/2
shared_offset = (-cx, -cy, -az0)
print(f"  全体bbox: X[{ax0:.1f},{ax1:.1f}] Y[{ay0:.1f},{ay1:.1f}] Z[{az0:.1f},{az1:.1f}] LDU")
print(f"  共有オフセット: ({shared_offset[0]:.2f},{shared_offset[1]:.2f},{shared_offset[2]:.2f}) LDU")


# ── STL エクスポート ──────────────────────────────────────

print("\n" + "="*60)
print("force_sensor_body 処理")
print("="*60)
body_bbox = export_stl(body_meshes, "force_sensor_body.stl", shared_offset)

print("\n" + "="*60)
print("button 処理")
print("="*60)
button_bbox = export_stl(button_meshes, "button.stl", shared_offset)

# ボタン上面Z（force_site計算に使用）
btn_z1 = button_bbox[2][1]  # Blender世界座標でのボタン上面Z

# ボタン位置（MuJoCo座標）
btn_cx_bl = (button_bbox[0][0]+button_bbox[0][1])/2 + shared_offset[0]
btn_cy_bl = (button_bbox[1][0]+button_bbox[1][1])/2 + shared_offset[1]
btn_cz_bl = (button_bbox[2][0]+button_bbox[2][1])/2 + shared_offset[2]
btn_mx = -btn_cx_bl * SCALE
btn_my = -btn_cy_bl * SCALE
btn_mz =  button_bbox[2][0]*SCALE + shared_offset[2]*SCALE  # ボタン底面Z（MuJoCo）

# センサー本体高さ（MuJoCo）
body_h = (body_bbox[2][1] - body_bbox[2][0]) * SCALE
sensor_mount_z = 0.0  # sensor_mount pos Z はblender_export_force_sensor_test.pyで計算

print(f"\n  ボタン底面 MuJoCo: Z={btn_mz:.4f} m  （= button_body pos Z）")
print(f"  センサー本体高さ: {body_h:.4f} m")


# ── force_site 位置計算 ───────────────────────────────────

print("\n" + "="*60)
print("force_site 位置計算（ボタン上面3D重心 → button body local座標）")
print("="*60)

site_result = compute_button_top_site(button_meshes, shared_offset, btn_z1)
if site_result:
    sx_world, sy_world, sz_world = site_result
    # button_body は pos="0 0 0"（sensor_mount原点）に配置するため、
    # force_site の座標は sensor_mount local = button body local と同じ
    sx = sx_world
    sy = sy_world
    sz = sz_world
    print(f"  force_site = sensor_mount local = button body local:")
    print(f"    ({sx:.4f}, {sy:.4f}, {sz:.4f}) m  ← XML に使う値")
    print(f"  （button_body は pos=0,0,0 のため座標変換不要）")
else:
    sx, sy = 0.0, 0.0
    sz = (btn_z1 + shared_offset[2]) * SCALE - SITE_RADIUS - btn_mz

# ボタンストローク（仕様: 2mm = 0.002m）
BUTTON_STROKE_M = 0.002


# ── MuJoCo XMLスニペット出力 ──────────────────────────────

print("\n" + "="*60)
print("MuJoCo XML スニペット（参考）")
print("="*60)
print(f"""
<!-- asset セクション -->
<mesh name="force_sensor_body_mesh" file="meshes/force_sensor_body.stl"/>
<mesh name="button_mesh"            file="meshes/button.stl"/>

<!-- components/force_sensor_body.xml のスニペット -->
<!-- MuJoCoの<include>はルート要素を除去して子要素を挿入するため
     センサー本体geom・衝突box・buttonボディが sensor_mount に展開される -->
<body name="force_sensor_body" euler="0 0 0">
  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0005 0.0005 0.0005"/>
  <geom name="force_sensor_geom" type="mesh" mesh="force_sensor_body_mesh"
        contype="0" conaffinity="0" rgba="0.3 0.3 0.8 1"/>

  <!-- ボタン（スプリング付きスライダー）
       axis="0 0 -1": 押し込み方向 = 世界-Z（下向き）
       range: 0（初期）〜 STROKE（最大押し込み）
       springref="0": スプリングがjoint_pos=0（初期位置）に復元
       stiffness: 10N / {BUTTON_STROKE_M}m = {10/BUTTON_STROKE_M:.0f} N/m -->
  <!-- button_body: pos="0 0 0" → STL（共有オフセット済み）が正しい位置に表示される -->
  <body name="button" pos="0 0 0">
    <joint name="button_slide" type="slide" axis="0 0 -1"
           range="0 {BUTTON_STROKE_M:.3f}"
           springref="0"
           stiffness="{10/BUTTON_STROKE_M:.0f}"
           damping="10"/>
    <inertial pos="0 0 0" mass="0.005" diaginertia="0.00001 0.00001 0.00001"/>
    <geom name="button_geom" type="mesh" mesh="button_mesh"
          contype="1" conaffinity="1" rgba="0.8 0.8 0.2 1"/>
    <!-- force_site: ボタン上面3D重心から算出（押し込み検出点） -->
    <site name="force_site" pos="{sx:.4f} {sy:.4f} {sz:.4f}" size="0.010" rgba="0 1 0 1"/>
  </body>
</body>

<!-- sensor セクション -->
<touch name="force_touch" site="force_site"/>

<!-- タッチ判定（libspikehat_sim 参考）:
     joint_pos > {BUTTON_STROKE_M/2*1000:.0f}mm → pressed=1
     force = stiffness × joint_pos [N] -->
""")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
