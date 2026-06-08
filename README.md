# libspikehat_sim

MuJoCo シミュレーター上で [libspikehat](https://github.com/kuboaki/libspikehat) の API を模倣するライブラリ。

実機（Raspberry Pi）と同じコードをMac/Linuxのシミュレーション環境で動かすことができる。

## 対応環境

| 環境 | OS | 用途 |
|------|----|------|
| 実機 | Raspberry Pi (Linux) | libspikehat を使用 |
| シム | macOS / Linux | libspikehat_sim を使用 |

## 必要なもの

- CMake 3.16以上
- Python 3.x + MuJoCo (`pip install mujoco`)
- C コンパイラ（AppleClang / GCC）

## ビルド方法

```bash
cmake -S . -B build
cmake --build build
```

macOS では cmake 時に `mujoco.framework` のシンボリックリンクが自動的に作成される。

## libspikehat との関係

libspikehat_sim は libspikehat と同じ API（`spikehat.h`）を持ち、
リンクするライブラリを切り替えるだけで実機とシムを使い分けられます。

```
┌───────────────┐       ┌─────────────────────┐
│   C app        │       │   Python app         │
└──────┬────────┘       └──────────┬───────────┘
       │                           │ ctypes
       └────────────┬──────────────┘
            ┌───────▼────────────────────┐
            │  libspikehat_sim.dylib/so  │
            │  (MuJoCo simulation impl)  │
            └───────┬────────────────────┘
                    │ MuJoCo C API
            ┌───────▼────────────────────┐
            │   MuJoCo Physics Engine    │
            └────────────────────────────┘
```

### C アプリのリンク方法

```bash
# 実機用（Raspberry Pi）
cc myapp.c -I path/to/libspikehat/include \
           -L path/to/libspikehat/build -lspikehat

# シム用（Mac / Linux）
cc myapp.c -I path/to/libspikehat_sim/include \
           -L path/to/libspikehat_sim/build -lspikehat_sim
```

ヘッダー（`spikehat.h`）は共通なので、アプリコードの変更は不要です。

## 使い方

### C版

```bash
SPIKEHAT_SIM_XML=examples/test_motor.xml ./build/test_motor
```

### Python版

```bash
cd /path/to/libspikehat_sim
SPIKEHAT_SIM_XML=examples/test_motor.xml \
  python3 -c "
import sys
sys.path.insert(0, 'python')
exec(open('examples/test_motor.py').read())
"
```

### 環境変数

| 変数 | 説明 | デフォルト |
|------|------|-----------|
| `SPIKEHAT_SIM_XML` | MuJoCo XMLファイルのパス | `/dev/serial0`（エラーになる） |

## XMLファイルのパラメータ

`examples/test_motor.xml` のパラメータはSPIKE Prime Lモーターの特性に合わせて調整済み：

| パラメータ | 値 | 意味 |
|-----------|-----|------|
| `damping` | 0.55 | 粘性抵抗（実機の回転特性に合わせて調整） |
| `gear` | 3 | トルクゲイン |
| `timestep` | 0.0002 | シミュレーションの時間刻み |

速度5・2秒で約-312度（実機の-315度に対して誤差1%以内）。

## ディレクトリ構成

```
libspikehat_sim/
├── CMakeLists.txt
├── README.md
├── include/
│   └── spikehat.h          # libspikehat と共通のヘッダー
├── src/
│   └── sim_spikehat.c      # MuJoCo ベースの実装
├── python/
│   └── spikehat.py         # Python バインディング
├── examples/
│   ├── test_motor.c        # C版テスト
│   ├── test_motor.py       # Python版テスト
│   ├── test_motor.xml      # MuJoCo XMLモデル
│   └── meshes/             # STLファイル（シンボリックリンク）
└── build/                  # ビルド成果物（gitignore対象）
```

## 関連リポジトリ

- [libspikehat](https://github.com/kuboaki/libspikehat) — 実機用ライブラリ
