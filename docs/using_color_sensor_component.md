# カラーセンサーコンポーネントの使い方

## 概要

`examples/components/color_sensor_body.xml` は、SPIKE Prime カラーセンサー（`37308c01.dat`）の
MuJoCo コンポーネントです。他のロボットモデルに `<include>` で組み込んで再利用できます。

```
examples/
├── components/
│   └── color_sensor_body.xml   ← このコンポーネント
├── meshes/
│   └── color_sensor.stl        ← センサー外形メッシュ
└── test_color_sensor.xml       ← 使用例（カラーブロックテストシーン）

studio_model/
├── color_sensor.io / .ldr               ← センサー単体Studioモデル
├── color_blocks.io / .ldr               ← カラーブロック群Studioモデル
├── color_blocks_with_color_sensor.io    ← テスト配置参照モデル
└── blender/
    ├── blender_export_color_sensor.py              ← color_sensor.stl 生成
    ├── blender_export_color_blocks.py              ← tile_*.stl / base_plate.stl 生成
    └── blender_export_color_blocks_with_sensor.py  ← センサー搭載位置 pos/euler 算出
```

---

## コンポーネントの構成

| 要素 | 内容 |
|---|---|
| `color_sensor` body | `euler="-90 0 0"` でセンサー検出面を下向きに配置 |
| `color_sensor_geom` | `color_sensor.stl` を参照するメッシュジオメトリ |
| `color_site` | レイキャスト発射点。センサーボディ内側（検出面から5mm内側）に配置 |

```xml
<!-- color_sensor_body.xml の内容 -->
<body name="color_sensor" euler="-90 0 0">
  <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>
  <geom name="color_sensor_geom" type="mesh" mesh="color_sensor_mesh"
        contype="0" conaffinity="0" rgba="0.3 0.3 0.3 1"/>
  <site name="color_site" pos="0.0000 0.0063 0.0116" size="0.005" rgba="1 1 0 1"/>
</body>
```

**設計原則:** `color_site` はセンサーボディの内側に配置されているため、センサーが
どの位置・姿勢に搭載されても対象物と干渉しません。

---

## MuJoCo `<include>` の動作について

MuJoCo の `<include>` は **ルート要素（`<body>` タグ）を除去** して子要素のみを挿入します。

```xml
<!-- 組み込み側 -->
<body name="sensor_mount" pos="X Y Z" euler="-90 0 0">
  <include file=".../color_sensor_body.xml"/>
  <!-- 展開後: inertial / geom / site が挿入される。body タグは除去 -->
</body>
```

このため:
- `euler="-90 0 0"` は **呼び出し側の body** に指定する必要があります
- `color_sensor_body.xml` 内の `euler="-90 0 0"` はファイル構造の記述であり、include 時には適用されません
- `inertial` は `color_sensor_body.xml` から供給されるため、呼び出し側には書かないこと

---

## 組み込み手順

### Step 1: Studioで搭載位置を決める

自分のロボットの Studio モデルに `37308c01.dat`（カラーセンサー）を配置します。
センサーの検出面が **下向き（床方向）** になるよう回転させてください。

ロボット＋センサーを1つの `.io` ファイルにまとめておくと Step 2 で pos を自動計算できます。

```
my_robot_with_sensor.io
├── my_robot_body    ← ロボット本体（基準となる body）
├── ...
└── 37308c01.dat     ← カラーセンサー（検出面下向きで配置）
```

> **注意（Studio の床スナップ）:**
> `37308c01.dat` は Studio の床スナップにより Z=19.5 LDU が自動付与されます。
> これは `blender_export_color_sensor.py` の `center_mode="bottom_z"` で吸収されるため、
> Studio 上の Z 値は修正しなくてかまいません。

### Step 2: Blenderでセンサーのpos/eulerを算出する

組み合わせモデルを Blender にインポートし、専用スクリプトを実行します。

```
1. Blender を起動してデフォルトオブジェクトを削除
2. File → Import → LDraw で my_robot_with_sensor.io をインポート（Scale=1.0）
3. blender_export_color_blocks_with_sensor.py を参考にスクリプトを作成・実行
   （基準 body の shared_offset を使い、センサーの相対 pos/euler を算出）
```

テスト環境（カラーブロック上のセンサー）の場合は
`studio_model/blender/blender_export_color_blocks_with_sensor.py` がそのまま使えます。

スクリプト出力の MuJoCo XML スニペットを Step 3 に使います。

### Step 3: MuJoCo XMLに組み込む

#### asset セクション

```xml
<asset>
  <mesh name="color_sensor_mesh"
        file="path/to/libspikehat_sim/examples/meshes/color_sensor.stl"/>
  <!-- 自分のモデルのアセット -->
</asset>
```

#### worldbody セクション

センサーを搭載する body の **子として** `<include>` します。
`euler="-90 0 0"` を呼び出し側の body に指定し、検出面を下向きにします。

```xml
<body name="sensor_mount" pos="X Y Z" euler="-90 0 0">
  <!--
    pos  : Step 2 で算出した搭載位置（基準 body 相対座標）
    euler: "-90 0 0" 固定（検出面 body local +Y → 世界 -Z 下向き）
    ※ inertial は color_sensor_body.xml から供給されるため不要
  -->
  <include file="path/to/libspikehat_sim/examples/components/color_sensor_body.xml"/>
</body>
```

> **pos Y の補正について:**
> `color_sensor.stl` は `bottom_z` 方式（高さ H=0.0231m、Z底面=0）のため、
> `euler="-90 0 0"` 適用後に STL が world Y=[0, H] に展開されます。
> センサーを Y=0 に中央揃えするには `pos Y -= H/2 = -0.0116m` の補正が必要です。
> `blender_export_color_blocks_with_sensor.py` のログに補正値の参考が出力されます。

#### sensor セクション（色検出に必要）

```xml
<sensor>
  <framepos  name="color_site_pos"  objtype="site" objname="color_site"/>
  <framequat name="color_site_quat" objtype="site" objname="color_site"/>
</sensor>
```

### Step 4: レイキャストで色を読む

```python
# Python (libspikehat_sim API)
hat = SpikeHat(xml_path)
hsv = hat.color_read_hsv(port=2)   # port番号はモデルの設定による
```

C API の場合は `spikehat_color_read_hsv()` を使用します。

---

## color_site の位置（参考値）

`color_sensor_body.xml` の `color_site` 現在値（`blender_export_color_sensor.py` による算出）:

```xml
<site name="color_site" pos="0.0000 0.0063 0.0116" size="0.005" rgba="1 1 0 1"/>
```

| パラメータ | 値 | 意味 |
|---|---|---|
| X=0.0000 | センサー幅方向中心 | 検出面3D重心から算出 |
| Y=0.0063 | 検出面から5mm内側 | センサー外に出ないよう内側に配置 |
| Z=0.0116 | センサー奥行き方向中心 | 検出面3D重心から算出 |

ビューアで `color_site`（黄色い球）がセンサーボディ内に収まっているか確認してください。

---

## 使用例

`examples/test_color_sensor.xml` が完全な使用例です。
カラーブロックテスト治具（`color_blocks_with_color_sensor.io`）を再現したシーンで
センサーをスライダーで動かしながら7色のタイルを検出します。

```bash
cd libspikehat_sim
uv run mjpython examples/test_color_sensor_viewer.py
```

### test_harness について

`color_blocks_with_color_sensor.io` には `test_harness` サブモデルが含まれています。
これは実際の物理テスト治具（シャフトにピンコネクターでセンサーを保持する構造）を
Studio 上で確認するために作成したものです。

`test_harness` は MuJoCo シミュレーション（`test_color_sensor.xml`）には含まれていません。
シャフト上のスライド機構は MuJoCo の `<joint type="slide">` で抽象化されており、
`test_color_sensor_viewer.py` の `sensor_slide_ctrl` スライダーがこれに対応します。

```
物理構造                         MuJoCo 抽象化
─────────────────────────        ──────────────────────────
test_harness（シャフト）    →    slider joint (axis="1 0 0")
48989 ピンコネクター       →    省略（センサー本体のみモデル化）
37308c01.dat センサー      →    color_sensor_body（<include>）
```

---

## STLの再生成が必要な場合

センサー形状を変更した場合は以下の手順で再生成します：

1. Studio で `color_sensor.io` を更新・保存（Z=19.5 LDU の床スナップは無視してよい）
2. Blender で `color_sensor.io` をインポート（Scale=1.0）
3. `studio_model/blender/blender_export_color_sensor.py` を実行
4. `examples/meshes/color_sensor.stl` が更新される
5. ログに出力される `color_site pos` 値を `color_sensor_body.xml` に反映
