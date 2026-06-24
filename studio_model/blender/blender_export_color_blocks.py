"""
Blender用スクリプト: color_blocks.io → STLファイル群

対応モデル: color_blocks.io
  Blenderシーン上のサブモデル構造:
    （ルートEMPTY）
      ├── base_plate   … 白ベースプレート（Step 1）
      ├── tile_black   … 黒タイル+プレート（Step 2）
      ├── tile_blue    … 青タイル+プレート
      ├── tile_green   … 緑タイル+プレート
      ├── tile_yellow  … 黄タイル+プレート
      ├── tile_red     … 赤タイル+プレート
      ├── tile_white   … 白タイル+プレート
      ├── tile_brown   … 茶タイル+プレート
      └── gap_cover    … 隙間カバー白タイル（Step 3、base_plateと統合）

エクスポート方針:
  - base_plate + gap_cover → base_plate.stl（1ファイルに統合）
  - tile_* → 各色ごとに個別STL（7ファイル）
  - 全STLは共有オフセット（全体bboxのXY中心・Z底面=0）を使用
    → MuJoCo XML上で各geomのposを揃えた配置が可能

出力:
  base_plate.stl   … ベースプレート+隙間カバー（白）
  tile_black.stl   … 黒ブロック
  tile_blue.stl    … 青ブロック
  tile_green.stl   … 緑ブロック
  tile_yellow.stl  … 黄ブロック
  tile_red.stl     … 赤ブロック
  tile_white.stl   … 白ブロック
  tile_brown.stl   … 茶ブロック

使い方:
  1. Blenderを起動してデフォルトオブジェクトを削除
  2. File → Import → LDraw で color_blocks.io をインポート（Scale=1.0）
  3. このスクリプトをBlenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import math
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/libspikehat_sim/examples/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/libspikehat_sim/studio_model/blender/blender_export_color_blocks_log.txt"
SCALE = 0.0004  # LDU → m

COLOR_NAMES = [
    "tile_black",
    "tile_blue",
    "tile_green",
    "tile_yellow",
    "tile_red",
    "tile_white",
    "tile_brown",
]


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
print("# blender_export_color_blocks_log")
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

def export_stl(meshes, out_filename, shared_offset):
    """
    shared_offset を使って全STLの座標原点を統一する。
    """
    if not meshes:
        print(f"  SKIP: メッシュなし → {out_filename}")
        return

    dx, dy, dz = shared_offset
    print(f"  MESHオブジェクト数: {len(meshes)}")
    print(f"  共有オフセット: ({dx:.2f}, {dy:.2f}, {dz:.2f}) LDU")

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


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:40s} parent={pname}{extra}")


# ── サブモデルルート取得 ──────────────────────────────────

base_root     = bpy.data.objects.get("base_plate")
gap_root      = bpy.data.objects.get("gap_cover")
color_roots   = {name: bpy.data.objects.get(name) for name in COLOR_NAMES}

print(f"\nサブモデル検索結果:")
print(f"  base_plate : {'OK' if base_root else 'NOT FOUND'}")
print(f"  gap_cover  : {'OK' if gap_root  else 'NOT FOUND'}")
for name, obj in color_roots.items():
    print(f"  {name:12s}: {'OK' if obj else 'NOT FOUND'}")


# ── メッシュ収集 ──────────────────────────────────────────

base_meshes = collect_mesh_descendants(base_root) if base_root else []
gap_meshes  = collect_mesh_descendants(gap_root)  if gap_root  else []
color_meshes = {
    name: collect_mesh_descendants(obj) if obj else []
    for name, obj in color_roots.items()
}


# ── 共有オフセット計算（全パーツのbboxから） ─────────────

print("\n" + "=" * 60)
print("共有オフセット計算（全パーツ統合bbox）")
print("=" * 60)

all_meshes = base_meshes + gap_meshes
for meshes in color_meshes.values():
    all_meshes += meshes

if all_meshes:
    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(all_meshes)
    cx, cy = (x0+x1)/2, (y0+y1)/2
    shared_offset = (-cx, -cy, -z0)
    print(f"  全パーツ統合bbox:")
    print(f"    X[{x0:.1f}, {x1:.1f}]  Y[{y0:.1f}, {y1:.1f}]  Z[{z0:.1f}, {z1:.1f}] LDU")
    print(f"  共有オフセット: ({shared_offset[0]:.2f}, {shared_offset[1]:.2f}, {shared_offset[2]:.2f}) LDU")
else:
    print("  ERROR: メッシュが見つかりません")
    _tee.close()
    raise SystemExit("メッシュなし")


# ── base_plate + gap_cover → base_plate.stl ──────────────

print("\n" + "=" * 60)
print("base_plate 処理（base_plate + gap_cover）")
print("=" * 60)

export_stl(
    meshes       = base_meshes + gap_meshes,
    out_filename = "base_plate.stl",
    shared_offset = shared_offset,
)


# ── 各色タイル → tile_*.stl ───────────────────────────────

for name, meshes in color_meshes.items():
    print("\n" + "=" * 60)
    print(f"{name} 処理")
    print("=" * 60)
    export_stl(
        meshes        = meshes,
        out_filename  = f"{name}.stl",
        shared_offset = shared_offset,
    )


# ── 各タイルの中心pos計算（MuJoCo XML参考値） ────────────

print("\n" + "=" * 60)
print("各タイルの中心pos（MuJoCo XML 参考値）")
print("=" * 60)
print("  ※ shared_offsetを適用したMuJoCo座標系でのXY中心・Z底面")
print()

for name, meshes in color_meshes.items():
    if not meshes:
        continue
    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
    dx, dy, dz = shared_offset
    cx = (x0+x1)/2 + dx
    cy = (y0+y1)/2 + dy
    cz_bot = z0 + dz
    mj_x = -cx * SCALE
    mj_y = -cy * SCALE
    mj_z =  cz_bot * SCALE
    h = (z1 - z0) * SCALE / 2
    print(f"  {name:12s}: pos=\"{mj_x:.4f} {mj_y:.4f} {mj_z:.4f}\"  half_h={h:.4f}m")


# ── MuJoCo XMLスニペット出力 ──────────────────────────────

print("\n" + "=" * 60)
print("MuJoCo XML スニペット（参考）")
print("=" * 60)

# LEGO color rgba values
RGBA = {
    "tile_black":  "0.027 0.027 0.027 1",
    "tile_blue":   "0.000 0.089 0.515 1",
    "tile_green":  "0.000 0.373 0.165 1",
    "tile_yellow": "0.992 0.843 0.000 1",
    "tile_red":    "0.578 0.010 0.002 1",
    "tile_white":  "0.991 0.991 0.991 1",
    "tile_brown":  "0.294 0.157 0.071 1",
}

print("\n<!-- asset セクション -->")
print('<mesh name="base_plate_mesh" file="meshes/base_plate.stl"/>')
for name in COLOR_NAMES:
    print(f'<mesh name="{name}_mesh" file="meshes/{name}.stl"/>')

print("\n<!-- worldbody セクション -->")
print('<body name="color_blocks" pos="0 0 0">')
print('  <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>')
# LDraw color 8 = Dark Gray (検出対象色と混同しない背景色)
BASE_PLATE_RGBA = "0.388 0.373 0.322 1"
print('  <geom name="base_plate_geom" type="mesh" mesh="base_plate_mesh"')
print(f'        contype="0" conaffinity="0" rgba="{BASE_PLATE_RGBA}"/>')
for name in COLOR_NAMES:
    rgba = RGBA.get(name, "0.5 0.5 0.5 1")
    meshes = color_meshes[name]
    if meshes:
        (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
        dx, dy, dz = shared_offset
        cx = (x0+x1)/2 + dx
        cy = (y0+y1)/2 + dy
        cz_bot = z0 + dz
        mj_x = -cx * SCALE
        mj_y = -cy * SCALE
        mj_z =  cz_bot * SCALE
        print(f'  <!-- {name} -->')
        print(f'  <geom name="{name}_geom" type="mesh" mesh="{name}_mesh"')
        print(f'        pos="{mj_x:.4f} {mj_y:.4f} {mj_z:.4f}"')
        print(f'        contype="0" conaffinity="0" rgba="{rgba}"/>')
print('</body>')

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
