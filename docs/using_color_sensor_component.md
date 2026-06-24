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
```

---

## コンポーネントの構成

| 要素 | 内容 |
|---|---|
| `color_sensor` body | `euler="-90 0 0"` でセンサー検出面を下向きに配置 |
| `color_sensor_geom` | `color_sensor.stl` を参照するメッシュジオメトリ |
| `color_site` | レイキャスト発射点。センサーボディ内側（検出面から5mm内側）に固定配置 |

**設計原則:** `color_site` はセンサーボディの内側に配置されているため、センサーが
どの位置・姿勢に搭載されても対象物と干渉しません。センサーメッシュと `color_site` は
常に一体として移動・回転します。

---

## 組み込み手順

### Step 1: Studioで搭載位置を決める

自分のロボットの Studio モデルに `37308c01.dat`（カラーセンサー）を配置します。
センサーの検出面が **下向き（床方向）** になるよう回転させてください。

組み合わせ確認用に、ロボット＋センサーを1つの `.io` ファイルにまとめておくと
次の Step で pos 値を自動計算できます。

```
my_robot_with_sensor.io
├── my_robot_body    ← ロボット本体
├── ...
└── 37308c01.dat     ← カラーセンサー（検出面下向きで配置）
```

### Step 2: Blenderでセンサーのpos/eulerを確認する

`studio_model/blender/blender_export_color_sensor.py` を参考に、
組み合わせモデルをBlenderにインポートして `37308c01.dat` の
ワールド座標（pos）と回転行列（euler）を取得します。

センサーの検出面が下向きなら MuJoCo での euler は `"-90 0 0"` が標準です。
センサーが水平・斜め等の場合は euler を適切に調整してください。

### Step 3: MuJoCo XMLに組み込む

#### asset セクション

```xml
<asset>
  <!-- color_sensor.stl のパスは自分のプロジェクトに合わせて調整 -->
  <mesh name="color_sensor_mesh"
        file="path/to/libspikehat_sim/examples/meshes/color_sensor.stl"/>
  <!-- 自分のモデルのアセット -->
  ...
</asset>
```

#### worldbody セクション

センサーを搭載する body の **子として** `<include>` します。

```xml
<body name="sensor_mount" pos="X Y Z">
  <!--
    pos: Step 2 で求めたセンサーのワールド座標（またはロボット本体基準の相対座標）
    eulerは color_sensor_body.xml 側に定義済み（"-90 0 0"）
  -->
  <include file="path/to/libspikehat_sim/examples/components/color_sensor_body.xml"/>
</body>
```

#### sensor セクション（色検出に必要）

```xml
<sensor>
  <framepos  name="color_site_pos"  objtype="site" objname="color_site"/>
  <framequat name="color_site_quat" objtype="site" objname="color_site"/>
</sensor>
```

### Step 4: レイキャストで色を読む

`libspikehat_sim` の `color_read_hsv()` / `color_read_rgb()` API が
`color_site` からの下向きレイキャストで色を返します。

```python
# Python (libspikehat_sim API)
hat = SpikeHat(xml_path)
hsv = hat.color_read_hsv(port=2)   # port番号はモデルの設定による
```

C API の場合は `spikehat_color_read_hsv()` を使用します。

---

## color_site の位置調整

`color_sensor_body.xml` の `color_site` デフォルト値:

```xml
<site name="color_site" pos="0 0.007 0.012" size="0.005" rgba="1 1 0 1"/>
```

| パラメータ | 値 | 意味 |
|---|---|---|
| Y=0.007 | 検出面から5mm内側 | センサー外に出ないよう内側に配置 |
| Z=0.012 | センサー奥行き中心 | 親bodyのY補正が必要な場合に調整 |

センサーの搭載姿勢や `euler` が変わる場合は、ビューアで `color_site`（黄色い球）が
センサーボディ内に収まっているか確認してから調整してください。

---

## 使用例

`examples/test_color_sensor.xml` が完全な使用例です。
カラーセンサーをスライダーで動かしながら7色のタイルを検出します。

```bash
cd libspikehat_sim
uv run mjpython examples/test_color_sensor_viewer.py
```

---

## STLの再生成が必要な場合

センサー形状を変更した場合は以下の手順で再生成します：

1. Studio で `color_sensor.io` を更新・保存
2. Blender で `color_sensor.io` をインポート（Scale=1.0）
3. `studio_model/blender/blender_export_color_sensor.py` を実行
4. `examples/meshes/color_sensor.stl` が更新される
5. ログに出力される `color_site pos` 値を `color_sensor_body.xml` に反映
