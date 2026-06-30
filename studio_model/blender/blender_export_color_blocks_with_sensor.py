"""
Blender用スクリプト: color_blocks_with_color_sensor.io → センサー配置情報

対応モデル: color_blocks_with_color_sensor.io
  Blenderシーン上の構造:
    （ルートEMPTY）
      ├── base_plate     … ベースプレートサブモデル
      ├── gap_cover      … 隙間カバーサブモデル
      ├── tile_black ... tile_brown … 各色タイルサブモデル
      ├── test_harness   … テスト治具骨格（シャフト付き）
      ├── 37308.dat      … カラーセンサー本体 MESH
      └── 48989.dat      … ピンコネクター MESH（MuJoCoでは省略）

処理内容:
  1. カラーブロック群（base_plate + gap_cover + tile_*）の全体bboxから
     shared_offset を計算（blender_export_color_blocks.py と同一手法）
  2. センサー（37308.dat）の Blender world transform を取得
  3. shared_offset を適用してカラーブロック原点基準の MuJoCo 座標に変換
  4. test_color_sensor.xml 用の body pos / euler を出力

座標変換:
  mj_x = -blender_x * SCALE
  mj_y = -blender_y * SCALE
  mj_z =  blender_z * SCALE

  回転: T_conj = diag(-1,-1,1) を用いて
  R_mj = T_conj @ R_blender @ T_conj

注意:
  test_harness と 48989.dat は MuJoCo では slider joint で抽象化するため
  このスクリプトでは STL エクスポートを行わない。

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で color_blocks_with_color_sensor.io をインポート
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import math
import mathutils
import os
import sys

LOG_PATH = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_color_blocks_with_sensor_log.txt"
SCALE    = 0.0004  # LDU → m


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
print("# blender_export_color_blocks_with_sensor_log")
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


# ── シーン確認 ────────────────────────────────────────────

COLOR_NAMES = [
    "tile_black", "tile_blue", "tile_green", "tile_yellow",
    "tile_red",   "tile_white", "tile_brown",
]

print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:50s} parent={pname}{extra}")


# ── カラーブロックの shared_offset 計算 ──────────────────

print("\n" + "=" * 60)
print("カラーブロック shared_offset 計算")
print("=" * 60)

block_root_names = ["base_plate", "gap_cover"] + COLOR_NAMES
block_meshes = []
for name in block_root_names:
    obj = bpy.data.objects.get(name)
    if obj:
        meshes = collect_mesh_descendants(obj)
        block_meshes.extend(meshes)
        print(f"  {name:12s}: {len(meshes)} meshes")
    else:
        print(f"  {name:12s}: NOT FOUND")

if not block_meshes:
    print("ERROR: カラーブロックメッシュが見つかりません")
    _tee.close()
    raise SystemExit("block meshes not found")

(x0,x1),(y0,y1),(z0,z1) = combined_bbox(block_meshes)
cx, cy = (x0+x1)/2, (y0+y1)/2
shared_offset = (-cx, -cy, -z0)
dx, dy, dz = shared_offset

print(f"\n  全ブロック統合bbox:")
print(f"    X[{x0:.2f}, {x1:.2f}]  Y[{y0:.2f}, {y1:.2f}]  Z[{z0:.2f}, {z1:.2f}] Blender")
print(f"  shared_offset: ({dx:.3f}, {dy:.3f}, {dz:.3f}) Blender LDU相当")


# ── センサーオブジェクト取得 ──────────────────────────────

print("\n" + "=" * 60)
print("センサー（37308.dat）位置・回転取得")
print("=" * 60)

# 37308.dat または 37308c01.dat など名前が揺れる場合があるため前方一致で検索
sensor_obj = None
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.name.startswith("37308"):
        sensor_obj = obj
        break

if sensor_obj is None:
    print("ERROR: センサーオブジェクト（37308*.dat）が見つかりません")
    _tee.close()
    raise SystemExit("sensor not found")

print(f"  センサーオブジェクト: {sensor_obj.name}  parent={sensor_obj.parent.name if sensor_obj.parent else '(root)'}")

mat_world = sensor_obj.matrix_world

# Blender world 位置
t = mat_world.translation
print(f"  Blender world 位置: ({t.x:.4f}, {t.y:.4f}, {t.z:.4f})")

# shared_offset 適用後の位置（カラーブロック原点基準）
bx = t.x + dx
by = t.y + dy
bz = t.z + dz
print(f"  shared_offset 適用後: ({bx:.4f}, {by:.4f}, {bz:.4f})")

# MuJoCo 座標に変換
mj_x = -bx * SCALE
mj_y = -by * SCALE
mj_z =  bz * SCALE
print(f"  MuJoCo pos: ({mj_x:.4f}, {mj_y:.4f}, {mj_z:.4f}) m")


# ── センサー回転の変換 ────────────────────────────────────

print("\n" + "=" * 60)
print("センサー回転 → MuJoCo euler")
print("=" * 60)

R_bl = mat_world.to_3x3().normalized()
print(f"  Blender 回転行列:")
for row in R_bl:
    print(f"    [{row[0]:7.4f} {row[1]:7.4f} {row[2]:7.4f}]")

# T_conj = diag(-1,-1,1): Blender → MuJoCo 回転変換
# R_mj = T_conj @ R_bl @ T_conj  (T_conj は自己逆行列)
T_conj = mathutils.Matrix(((-1,0,0),(0,-1,0),(0,0,1)))
R_mj   = T_conj @ R_bl @ T_conj

print(f"  MuJoCo 回転行列:")
for row in R_mj:
    print(f"    [{row[0]:7.4f} {row[1]:7.4f} {row[2]:7.4f}]")

euler_rad = R_mj.to_euler('XYZ')
euler_deg = [math.degrees(a) for a in euler_rad]
print(f"  MuJoCo euler (XYZ度): ({euler_deg[0]:.2f}, {euler_deg[1]:.2f}, {euler_deg[2]:.2f})")


# ── センサーのSTL bbox確認（bottom_zオフセット参照用） ────

print("\n" + "=" * 60)
print("センサーSTLのbbox（body pos Y補正の参考）")
print("=" * 60)
print("  color_sensor.stl は bottom_z 方式（XY中心・Z底面=0）")
print("  → STL の Z範囲中心（half_H）が euler後に world Y 方向にズレる")
print("  → body pos Y = -half_H で補正（センサーをY方向に中央揃え）")
stl_h = 0.0231  # color_sensor.stl の高さ（blender_export_color_sensor_log.txtより）
half_h = stl_h / 2
print(f"  STL高さ: {stl_h:.4f}m  half_H: {half_h:.4f}m")
print(f"  → body pos の Y を {-half_h:.4f}m 補正することを検討")


# ── MuJoCo XML スニペット出力 ─────────────────────────────

px_str = f"{mj_x:.4f}"
py_str = f"{mj_y:.4f}"
pz_str = f"{mj_z:.4f}"
ex_str = f"{euler_deg[0]:.1f}"
ey_str = f"{euler_deg[1]:.1f}"
ez_str = f"{euler_deg[2]:.1f}"

print("\n" + "=" * 60)
print("MuJoCo XML スニペット（test_color_sensor.xml 参考）")
print("=" * 60)
print(f"""
<!-- カラーセンサー body（スライダー付き） -->
<!-- pos: color_blocks_with_color_sensor.io のセンサー位置をブロック原点基準に変換 -->
<!-- euler: Studioで配置した向きをMuJoCo回転に変換 -->
<body name="color_sensor_body" pos="{px_str} {py_str} {pz_str}" euler="{ex_str} {ey_str} {ez_str}">
  <joint name="sensor_slide" type="slide" axis="1 0 0"
         range="-0.090 0.090" damping="10.0"/>
  <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>
  <include file="components/color_sensor_body.xml"/>
</body>
""")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
