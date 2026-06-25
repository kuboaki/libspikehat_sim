# 距離センサーコンポーネントの使い方

## 概要

`examples/components/distance_sensor_body.xml` は、SPIKE Prime 距離センサー（`37316c01.dat`）の
MuJoCo コンポーネントです。他のロボットモデルに `<include>` で組み込んで再利用できます。

```
examples/
├── components/
│   └── distance_sensor_body.xml  ← このコンポーネント
├── meshes/
│   ├── distance_sensor.stl       ← センサー外形メッシュ
│   └── obstacle_wall_a.stl       ← テスト用壁メッシュ
└── test_distance_sensor.xml      ← 使用例（壁テストシーン）
```

---

## コンポーネントの構成

| 要素 | 内容 |
|---|---|
| `distance_sensor` body | `euler="0 0 0"` でセンサー検出面を前方（+Y）に配置 |
| `distance_sensor_geom` | `distance_sensor.stl` を参照するメッシュジオメトリ（衝突なし） |
| `distance_sensor_col` | 衝突判定用 box（センサー外形に近似）。障害物がセンサーを通り抜けるのを防ぐ |
| `distance_site` | レイキャスト発射点。検出面3D重心から算出し、センサーボディ内側に固定配置 |

**設計原則:**
- `distance_site` の位置は `blender_export_distance_sensor.py` が検出面の3D重心から
  自動計算します。部品が同じであれば常に同じ相対位置になります。
- `distance_site` はセンサーボディの内側に配置されているため、センサーが
  どの位置・姿勢に搭載されても対象物と干渉しません。
- MuJoCo の `<include>` はルート要素を除去して子要素を挿入するため、
  `sensor_mount` 側に `<inertial>` を別途書かないこと（重複エラーになります）。

---

## 組み込み手順

### Step 1: Studioで搭載位置を決める

自分のロボットの Studio モデルに `37316c01.dat`（距離センサー）を配置します。
センサーの検出面が **前方（ロボットが検出したい方向）** を向くよう回転させてください。

次の2つの Studio ファイルを作成します：

```
distance_sensor.io          ← センサー単体モデル（STL生成用）
my_robot_with_sensor.io     ← ロボット + センサーの組み合わせモデル（位置確認用）
├── my_robot_body
├── ...
└── 37316c01.dat            ← 距離センサー（検出方向に向けて配置）
```

### Step 2: Blenderでセンサーのpos/euler を確認する

**センサーSTLの生成:**

1. Blenderを起動してデフォルトオブジェクトを削除
2. `File → Import → LDraw` で `distance_sensor.io` をインポート（Scale=1.0）
3. `studio_model/blender/blender_export_distance_sensor.py` を実行
4. `examples/meshes/distance_sensor.stl` が生成され、ログに以下が出力される：

```
センサー底面 MuJoCo Z: X.XXXX m  （= sensor_mount pos Z）
site body local MuJoCo: X=0.XXXX Y_face=0.XXXX Z=0.XXXX m
```

**センサー位置と初期距離の計算:**

1. Blenderを起動してデフォルトオブジェクトを削除
2. `File → Import → LDraw` で組み合わせモデルの `.io` をインポート（Scale=1.0）
3. `studio_model/blender/blender_export_distance_sensor_test.py` を実行
4. ログに以下が出力される：

```
センサー底面 MuJoCo Z: X.XXXX m  （= sensor_mount pos Z）
IOファイルでのセンサー→壁 距離: XXX.X mm
```

### Step 3: MuJoCo XMLに組み込む

#### asset セクション

```xml
<asset>
  <!-- distance_sensor.stl のパスは自分のプロジェクトに合わせて調整 -->
  <mesh name="distance_sensor_mesh"
        file="path/to/libspikehat_sim/examples/meshes/distance_sensor.stl"/>
  <!-- 自分のモデルのアセット -->
  ...
</asset>
```

#### worldbody セクション

```xml
<!-- sensor_mount の pos: Step 2 で求めたセンサー底面高さ -->
<!-- sensor_mount の euler: センサーの検出方向に応じて設定 -->
<!--   前方(+Y): euler="0 0 0"（デフォルト） -->
<!--   下向き(-Z): euler="-90 0 0"           -->
<!--   上向き(+Z): euler="90 0 0"            -->
<body name="sensor_mount" pos="X Y Z">
  <include file="path/to/libspikehat_sim/examples/components/distance_sensor_body.xml"/>
</body>
```

#### sensor セクション（距離計測に必要）

```xml
<sensor>
  <framepos  name="distance_site_pos"  objtype="site" objname="distance_site"/>
  <framequat name="distance_site_quat" objtype="site" objname="distance_site"/>
</sensor>
```

### Step 4: レイキャストで距離を読む

`libspikehat_sim` の `distance_read()` API が `distance_site` からの前方レイキャストで
距離を返します。

```python
# Python (libspikehat_sim API)
hat = SpikeHat(xml_path)
dist_mm = hat.distance_read(port=3)   # port番号はモデルの設定による
```

C API の場合は `spikehat_distance_read()` を使用します。

---

## センサーの向きと euler

距離センサーは検出面の向きによって `sensor_mount` の euler を変えます。

| 検出方向 | sensor_mount euler | 用途例 |
|---|---|---|
| 前方（+Y） | `euler="0 0 0"` | 前方の障害物検出 |
| 下向き（-Z） | `euler="-90 0 0"` | 床までの距離計測 |
| 上向き（+Z） | `euler="90 0 0"` | 天井までの距離計測 |
| 後方（-Y） | `euler="0 0 180"` | 後方の障害物検出 |
| 右向き（+X） | `euler="0 0 -90"` | 右側面の障害物検出 |
| 左向き（-X） | `euler="0 0 90"` | 左側面の障害物検出 |

レイキャストは常に `distance_site` の体前方向（body local +Y）に発射されます。
`sensor_mount` の euler でその方向が変わります。

---

## distance_site の位置

`distance_sensor_body.xml` の `distance_site` は、`blender_export_distance_sensor.py` が
検出面（LDraw -Z面）の3D重心を計算して自動設定します。

```xml
<!-- 検出面3D重心から算出（部品形状に忠実・再利用可能） -->
<site name="distance_site" pos="X Y Z" size="0.005" rgba="1 0 0 1"/>
```

この値は部品 `37316c01.dat` の形状から決まるため、**どのモデルに搭載しても変わりません**。
STLを再生成した場合にのみ再計算が必要です。

---

## STLの再生成が必要な場合

センサー形状を変更した場合は以下の手順で再生成します：

1. Studio で `distance_sensor.io` を更新・保存
2. Blender で `distance_sensor.io` をインポート（Scale=1.0）
3. `studio_model/blender/blender_export_distance_sensor.py` を実行
4. `examples/meshes/distance_sensor.stl` が更新される
5. ログに出力される `site body local MuJoCo` 値を
   `distance_sensor_body.xml` の `distance_site pos` に反映

---

## テストシーンの実行

```bash
cd libspikehat_sim
uv run mjpython examples/test_distance_sensor_viewer.py
```

`wall_slide_ctrl` スライダーで壁を前後に移動して距離値を確認できます。
- 正の値：壁が遠ざかる（距離増加）
- 負の値：壁が近づく（距離減少）
- 有効距離：50mm〜300mm

---

## カラーセンサーとの対比

| 項目 | カラーセンサー | 距離センサー |
|---|---|---|
| 部品番号 | `37308c01.dat` | `37316c01.dat` |
| 検出方向（標準） | 下向き（-Z） | 前方（+Y） |
| sensor_mount euler | `"-90 0 0"` | `"0 0 0"` |
| site 名 | `color_site` | `distance_site` |
| site rgba | 黄色（1 1 0 1） | 赤（1 0 0 1） |
| 衝突判定 box | なし | あり |
| ドキュメント | `using_color_sensor_component.md` | このファイル |
