# フォースセンサーコンポーネントの使い方

## 概要

`examples/components/force_sensor_body.xml` は、SPIKE Prime フォースセンサー（カスタムパーツ）の
MuJoCo コンポーネントです。他のロボットモデルに `<include>` で組み込んで再利用できます。

```
examples/
├── components/
│   └── force_sensor_body.xml   ← このコンポーネント
├── meshes/
│   ├── force_sensor_body.stl   ← センサー本体外形メッシュ
│   └── button.stl              ← ボタンパーツメッシュ
└── test_force_sensor.xml       ← 使用例（press_block テストシーン）
```

---

## コンポーネントの構成

| 要素 | 内容 |
|---|---|
| `force_sensor_body` geom | センサー本体ハウジング（視覚のみ、contype=0） |
| `force_site` site | センサー本体内部の検出点（視覚目的、Z=0.030m） |
| `button` body | 黄色いボタンパーツ（button_slide joint + 内蔵スプリング） |

**設計原則：**
- **内蔵スプリング**（stiffness=1253 N/m）がパーツ固有の復元力を提供
- 外力がなくなると自動でボタンが初期位置に戻る
- `force_touch` センサーで接触力を検出、またはスプリング変位から力を計算

---

## press_block の位置関係（重要な知見）

### LDRファイルからの位置算出

button と press_block の位置は **LDR ファイルから Blender スクリプトで自動算出**します。
手動で数値を入力すると、STL とのズレが生じます。

```
studio_model/blender/
├── blender_export_force_sensor.py       ← button.stl + force_sensor_body.stl 生成
└── blender_export_force_sensor_test.py  ← press_block.stl + press_block 位置算出
```

### rotor_axis モードを使う

press_block の STL エクスポートには `center_mode="rotor_axis"` を使用します。
これにより STL local Z=0 が press_block の LDraw 原点（= シャフト底面）に対応し、
MuJoCo での body pos Z が LDR 配置と一致します。

```python
# blender_export_force_sensor_test.py での press_block エクスポート
export_stl(press_meshes, "press_block.stl",
           center_mode="rotor_axis",
           rotor_axis_obj=press_root)  # press_block EMPTY の原点を基準
```

### スクリプトが算出する値

| 値 | 算出方法 |
|---|---|
| `press_block body pos Z` | press_block EMPTY の Blender world Z → MuJoCo Z 変換 |
| `press_block body pos XY` | press_block EMPTY の XY 中心 |
| `シャフト差し込み深さ` | button top Z - shaft tip Z |

---

## press_block との接合（シャフト接合）

### 機構の説明

```
press_block（グレー、外部押下体）
    │ シャフトが button 内部に差し込まれている
    │ LDRから: press_block LDraw Y=-100.875 → MuJoCo Z=0.0411m
    │ シャフト差し込み深さ: ≈6.77mm（ボタン内部ほぼいっぱい）
    ↓
button（黄色、force_sensor_button）
    │ button_slide joint（内蔵スプリング stiffness=1253）
    ↓
force_sensor_body（固定）
```

### MuJoCo での実装（equality 制約）

```xml
<!-- シャフト接合: press_slide が button_slide に追従 -->
<equality>
  <joint joint1="press_slide" joint2="button_slide"
         polycoef="0 1 0 0 0"/>
</equality>
```

**ポイント：**
- motor アクチュエーターを `button_slide` に直接かける
- `press_slide` は equality で受動的に追従
- これにより button と press_block が **1:1 で完全連動**

### press_block geom の衝突設定

```xml
<!-- contype=0 必須: シャフトが button 内部に入るため衝突なし -->
<geom name="press_geom" type="mesh" mesh="press_block_mesh"
      contype="0" conaffinity="0" rgba="0.7 0.7 0.7 1"/>
```

シャフトが button 内部に入るため、press_block_geom と button_geom が初期状態で重なります。
`contype=1` のままにすると衝突が発生してボタンが強制的に押し下げられます。

---

## 力の読み取り

### スプリング変位からの計算（推奨）

```python
STIFFNESS = 1253.0  # N/m（実機校正値）
spring_force = STIFFNESS × button_slide_joint_pos  # [N]
pressed = button_slide_joint_pos > 0.001  # 1mm でタッチ判定
```

### 重力補正

press_block 込みの場合、button+press_block の重力がスプリングに伝わります。
外力のみを表示する場合は重力分を引きます：

```python
GRAVITY_PRELOAD = (button_mass + press_block_mass) * 9.81  # ≈ 0.294N
ext_force = max(0.0, spring_force - GRAVITY_PRELOAD)
```

**実機との比較：** 実機では内蔵スプリングが press_block 装着程度の重力を十分に上回るため、
垂直配置でも沈み込みは発生しません（MuJoCo も同様の動作）。

---

## 実機仕様（SPIKE Prime フォースセンサー 型番 45606）

| 仕様 | 値 |
|---|---|
| 圧力測定範囲 | 0〜10N |
| タッチ検出範囲 | 0〜2mm |
| タッチ感応力 | 0.5〜1.0N（しきい値 1±0.5mm） |
| サンプリングレート | 100Hz（通常）|

### MuJoCo パラメーター（実機校正値）

| パラメーター | 値 | 算出方法 |
|---|---|---|
| stiffness | 1253 N/m | apply=9.5N が実機 10N 相当 → 1190×(10/9.5) |
| pressed ON 閾値 | 1.0mm | 仕様の中央値 |
| pressed OFF 閾値 | 0.5mm | ヒステリシス（安定動作のため） |
| range | 0〜9.5mm | 実機確認（最大押し込みで button top = sensor body top） |

---

## 組み込み手順

### Step 1: Studioでモデル作成

```
force_sensor.io  ← センサー単体（button のみ、press_block なし）
force_sensor_test.io  ← センサー + press_block（位置確認用）
```

**重要：** `force_sensor.io` に press_block を含めないこと。
press_block はテスト用ツールであり、コンポーネント外のパーツです。

### Step 2: Blenderでエクスポート

```bash
# 1. force_sensor.io をインポートしてスクリプト実行
#    → button.stl, force_sensor_body.stl 生成
blender_export_force_sensor.py

# 2. force_sensor_test.io をインポートしてスクリプト実行
#    → press_block.stl 生成、press_block body pos Z を算出
blender_export_force_sensor_test.py
```

ログに出力された `press_block body pos Z` の値を XML に反映します。

### Step 3: MuJoCo XML に組み込む

**sensor_mount のみ（press_block なし、荷物シナリオ等）：**

```xml
<body name="sensor_mount" pos="X Y Z">
  <include file="components/force_sensor_body.xml"/>
</body>
```

**press_block あり（テストシーン）：**

```xml
<!-- equality: press_slide が button_slide に追従（シャフト接合） -->
<equality>
  <joint joint1="press_slide" joint2="button_slide"
         polycoef="0 1 0 0 0"/>
</equality>

<body name="sensor_mount" pos="0 0 0">
  <include file="components/force_sensor_body.xml"/>
</body>

<!-- press_block: LDR から算出した位置に配置 -->
<body name="press_block" pos="-0.0004 0.0002 0.0411">
  <joint name="press_slide" type="slide" axis="0 0 -1"
         range="0 0.0095" damping="5"/>
  <inertial pos="0 0 0.014" mass="0.020" .../>
  <geom name="press_geom" type="mesh" mesh="press_block_mesh"
        contype="0" conaffinity="0" rgba="0.7 0.7 0.7 1"/>
</body>

<!-- motor on button_slide: press_block も equality で連動 -->
<actuator>
  <motor name="press_ctrl" joint="button_slide"
         gear="1" ctrllimited="true" ctrlrange="0 10"/>
</actuator>
```

---

## 「離す」動作の実現

```
ctrl=0  → 力ゼロ → 内蔵スプリングで button も press_block も自動復元
ctrl>0  → button + press_block が一緒に押し込まれる
```

**press_block の自動復元：** equality 制約を通じて button のスプリングが
press_block の復元力も担います。press_block 自身にスプリングは不要です。

**Space キー（テスト用リセット）：**

```python
def key_callback(keycode):
    if keycode == 32:  # Space
        data.ctrl[press_ctrl_id] = 0.0
```

---

## カラー・距離センサーとの対比

| 項目 | カラーセンサー | 距離センサー | フォースセンサー |
|---|---|---|---|
| 検出方向 | 下向き（-Z） | 前方（+Y） | 押し込み（-Z） |
| 検出方法 | レイキャスト | レイキャスト | 接触力・スプリング変位 |
| 内蔵スプリング | なし | なし | **あり（stiffness=1253）** |
| press_block | なし | なし | あり（equality で接合） |
| 自動復元 | なし | なし | **あり（スプリング）** |
| ドキュメント | using_color_sensor_component.md | using_distance_sensor_component.md | このファイル |
