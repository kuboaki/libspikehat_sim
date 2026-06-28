"""
Blender用スクリプト: force_sensor_test.io → press_block.stl + 位置情報

対応モデル: force_sensor_test.io
  Blenderシーン上のサブモデル:
    （ルートEMPTY）
      ├── カスタムパーツA（センサー本体）
      ├── カスタムパーツB（ボタン）
      └── press_block   … 押下機構サブモデル（ボタン直上に配置）

エクスポート方針:
  - press_block → press_block.stl（bottom_z centering）
  - センサー本体・ボタンは blender_export_force_sensor.py で処理済み
  - センサーの sensor_mount pos Z をメッシュbbox底面から計算
  - press_block の初期位置をIOファイルから計算

出力:
  press_block.stl ... 押下機構の外形メッシュ

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で force_sensor_test.io をインポート（Scale=1.0）
     Additional Library Path にカスタムパーツのパスを設定すること
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_force_sensor_test_log.txt"
SCALE = 0.0004
BUTTON_STROKE_M = 0.002  # ボタンストローク [m]


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
print("# blender_export_force_sensor_test_log")


# ── メッシュ収集 ──────────────────────────────────────────

def collect_mesh_descendants(root, stop_at_empty=False):
    result = []
    for child in root.children:
        if child.type == 'MESH':
            result.append(child)
            result.extend(collect_mesh_descendants(child, stop_at_empty))
        elif child.type == 'EMPTY':
            if stop_at_empty:
                pass
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

def export_stl(meshes, out_filename, center_mode="bottom_z", shared_offset=None, rotor_axis_obj=None):
    if not meshes:
        print(f"  SKIP: メッシュなし → {out_filename}")
        return None

    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
    cx, cy = (x0+x1)/2, (y0+y1)/2
    if shared_offset is not None:
        dx, dy, dz = shared_offset
    elif center_mode == "rotor_axis" and rotor_axis_obj is not None:
        ax = rotor_axis_obj.matrix_world.translation
        dx, dy, dz = -ax.x, -ax.y, -ax.z
        print(f"  rotor_axis_obj: {rotor_axis_obj.name}  origin=({ax.x:.2f},{ax.y:.2f},{ax.z:.2f}) LDU")
    elif center_mode == "bottom_z":
        dx, dy, dz = -cx, -cy, -z0
    else:
        dz = -(z0+z1)/2
        dx, dy = -cx, -cy

    print(f"  MESHオブジェクト数: {len(meshes)}")
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
    print(f"  Saved: {filepath}")
    w = (x1-x0)*SCALE; d = (y1-y0)*SCALE; h = (z1-z0)*SCALE
    print(f"  サイズ: W={w:.4f}m D={d:.4f}m H={h:.4f}m")
    return (x0,x1),(y0,y1),(z0,z1)


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "="*60)
print("シーン内オブジェクト一覧")
print("="*60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:50s} parent={pname}{extra}")


# ── サブモデルとメッシュの取得 ───────────────────────────

press_root = bpy.data.objects.get("press_block")
print(f"\npress_block : {'OK' if press_root else 'NOT FOUND'}")

# press_block 以外の全メッシュ = センサー本体 + ボタン
all_non_press = []
for o in bpy.data.objects:
    if o.type == 'MESH':
        # press_block の子孫でないもの
        parent = o.parent
        is_press = False
        while parent:
            if parent == press_root:
                is_press = True
                break
            parent = parent.parent
        if not is_press:
            all_non_press.append(o)

print(f"センサー本体+ボタン メッシュ数: {len(all_non_press)}")

if press_root:
    press_meshes = collect_mesh_descendants(press_root)
    print(f"press_block メッシュ数: {len(press_meshes)}")
else:
    press_meshes = []


# ── 基準オフセット計算（全パーツのZ底面=0） ──────────────

print("\n" + "="*60)
print("基準オフセット計算（センサー本体+ボタン+press_block の統合bbox）")
print("="*60)

all_meshes = all_non_press + press_meshes
(ax0,ax1),(ay0,ay1),(az0,az1) = combined_bbox(all_meshes)
cx, cy = (ax0+ax1)/2, (ay0+ay1)/2
ref_offset = (-cx, -cy, -az0)
print(f"  全体bbox: X[{ax0:.1f},{ax1:.1f}] Y[{ay0:.1f},{ay1:.1f}] Z[{az0:.1f},{az1:.1f}] LDU")
print(f"  基準オフセット: ({ref_offset[0]:.2f},{ref_offset[1]:.2f},{ref_offset[2]:.2f}) LDU")


# ── センサー本体/ボタン の位置計算 ───────────────────────

print("\n" + "="*60)
print("センサー本体・ボタン位置計算")
print("="*60)

if len(all_non_press) >= 2:
    # Z中心でソート: 低い方 = センサー本体、高い方 = ボタン
    sorted_np = sorted(all_non_press, key=lambda o: combined_bbox([o])[2][0])
    body_meshes   = [sorted_np[0]]
    button_meshes = sorted_np[1:]

    (bx0,bx1),(by0,by1),(bz0,bz1) = combined_bbox(body_meshes)
    (btx0,btx1),(bty0,bty1),(btz0,btz1) = combined_bbox(button_meshes)

    # sensor_mount pos Z = センサー本体底面（from ref_offset）
    sensor_mount_z = (bz0 + ref_offset[2]) * SCALE
    # ボタン底面 MuJoCo Z（共有オフセット基準）
    button_bottom_z = (btz0 + ref_offset[2]) * SCALE
    # ボタン上面 MuJoCo Z
    button_top_z    = (btz1 + ref_offset[2]) * SCALE
    # ボタンストローク（仕様値を使用: 2mm）

    print(f"  センサー本体: {[m.name for m in body_meshes]}")
    print(f"    bbox Z: [{bz0:.1f}, {bz1:.1f}] LDU")
    print(f"    sensor_mount pos Z: {sensor_mount_z:.4f} m")
    print(f"  ボタン: {[m.name for m in button_meshes]}")
    print(f"    bbox Z: [{btz0:.1f}, {btz1:.1f}] LDU")
    print(f"    button_body pos Z: {button_bottom_z:.4f} m")
    print(f"    ボタン上面 Z: {button_top_z:.4f} m")
    print(f"    ストローク: {BUTTON_STROKE_M*1000:.1f} mm（仕様値）")
else:
    sensor_mount_z = 0.050
    button_bottom_z = 0.060
    button_top_z    = 0.070
    print("  WARNING: センサーパーツが不足、デフォルト値を使用")


# ── press_block STL エクスポートと位置計算 ───────────────

print("\n" + "="*60)
print("press_block 処理")
print("="*60)

press_bbox = None
if press_meshes:
    # rotor_axis: press_block EMPTYの原点を基準にSTLを配置
    # → STL local Z=0 = press_block LDraw原点（= シャフト底面）
    # これにより MuJoCo での body pos Z が LDR と一致する
    press_bbox = export_stl(press_meshes, "press_block.stl",
                            center_mode="rotor_axis",
                            rotor_axis_obj=press_root)

print("\n" + "="*60)
print("press_block 位置計算（MuJoCo座標）")
print("="*60)

if press_bbox and press_root:
    (px0,px1),(py0,py1),(pz0,pz1) = press_bbox
    # press_block XY中心（MuJoCo）
    pb_cx_bl = (px0+px1)/2 + ref_offset[0]
    pb_cy_bl = (py0+py1)/2 + ref_offset[1]
    pb_mx = -pb_cx_bl * SCALE
    pb_my = -pb_cy_bl * SCALE
    # rotor_axis モード: STL local Z=0 = press_block LDraw原点（シャフト底面）
    # → body pos Z = press_block EMPTY の world Blender Z を ref_offset で変換
    ax = press_root.matrix_world.translation
    pb_body_z = (ax.z + ref_offset[2]) * SCALE
    print(f"  press_block EMPTY world Z: {ax.z:.3f} LDU")
    print(f"  press_block body pos Z (MuJoCo): {pb_body_z:.4f} m ({pb_body_z*1000:.2f}mm)")
    print(f"  ボタン上面  MuJoCo Z:      {button_top_z:.4f} m")
    shaft_depth = button_top_z - pb_body_z
    print(f"  シャフトがbuttonに差し込まれた深さ: {shaft_depth*1000:.2f}mm")
    pb_bottom_z = pb_body_z  # 旧変数名との互換性
    initial_gap = 0.0  # rotor_axisモードでは press_block がbutton内に差し込まれた状態が初期位置

    # press_block のスライダー範囲
    # ctrl=0: press_block は初期位置（ボタン非接触）
    # ctrl>0: press_block が下降（-Z方向）してボタンを押す
    press_range_max = initial_gap + BUTTON_STROKE_M + 0.002  # ギャップ+ストローク+余裕
else:
    pb_mx, pb_my, pb_bottom_z = 0.0, 0.0, 0.080
    initial_gap = 0.002
    press_range_max = 0.010
    print("  WARNING: press_block が見つかりません、デフォルト値を使用")


# ── MuJoCo XMLスニペット出力 ──────────────────────────────

print("\n" + "="*60)
print("MuJoCo XML スニペット（参考）")
print("="*60)

press_range_str = f"{press_range_max:.4f}"
print(f"""
<!-- asset セクション -->
<mesh name="force_sensor_body_mesh" file="meshes/force_sensor_body.stl"/>
<mesh name="button_mesh"            file="meshes/button.stl"/>
<mesh name="press_block_mesh"       file="meshes/press_block.stl"/>

<!-- worldbody セクション -->

<!-- フォースセンサー（button_bodyをinclud経由で取り込む） -->
<body name="sensor_mount" pos="0 0 {sensor_mount_z:.4f}">
  <include file="components/force_sensor_body.xml"/>
</body>

<!-- 押下ブロック（Z方向スライダー: 正の値で下降=ボタン押し込み） -->
<body name="press_block" pos="{pb_mx:.4f} {pb_my:.4f} {pb_bottom_z:.4f}">
  <joint name="press_slide" type="slide" axis="0 0 -1"
         range="0 {press_range_str}" damping="5.0"/>
  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0005 0.0005 0.0005"/>
  <geom name="press_geom" type="mesh" mesh="press_block_mesh"
        contype="1" conaffinity="1" rgba="0.7 0.7 0.7 1"/>
</body>

<!-- アクチュエーター: ctrl=0→初期位置、ctrl正→下降（ボタン押し込み） -->
<actuator>
  <position name="press_ctrl" joint="press_slide"
            kp="500" ctrllimited="true" ctrlrange="0 {press_range_str}"/>
</actuator>

<!-- 初期ギャップ: {initial_gap*1000:.2f} mm（ボタン上面〜press_block底面）
     ctrl ≈ {initial_gap:.4f}m でボタンに接触開始
     ctrl ≈ {initial_gap + BUTTON_STROKE_M:.4f}m でフルストローク（{BUTTON_STROKE_M*1000:.0f}mm押し込み） -->
""")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
