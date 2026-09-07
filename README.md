# StampFly Ecosystem

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 自分の手で、ドローンの飛行制御を作りたいあなたへ

**StampFly Ecosystem** は、ドローン制御を**学び、実装し、実験する**ための
教育・研究プラットフォームです。

「PID制御を教科書で学んだけど、実際に動くものを作りたい」
「姿勢推定アルゴリズムを自分で実装して試したい」
「研究用の飛行実験プラットフォームが欲しい」

そんなあなたのために、このエコシステムは存在します。

---

## 何ができるのか？

| できること | 内容 |
|-----------|------|
| **シミュレータで練習** | 実機がなくても、PC 上の 3D シミュレータで送信機を使った操縦を体験できる |
| **実機での飛行体験** | ファームウェアを書き込んだ StampFly を送信機で飛ばす。4 つの飛行モード（ACRO／STABILIZE／高度維持／位置保持）を搭載 |
| **センサー値の取得** | IMU・気圧・ToF・オプティカルフローの値をリアルタイムに読み出し、表示・記録できる |
| **コントローラからの指令の受信** | 送信機のスティック・ボタンの値を機体側で受け取り、自分のプログラムから使える |
| **独自の飛行プログラムの作成** | `sf app new` で自分のプロジェクトを作り、制御則や飛行の振る舞いを自由に書ける |
| **飛行プログラムの SILS での検証** | 作成した飛行プログラムを SILS（Software In the Loop Simulation: ファームウェアそのものを PC 上で飛ばす試験）で、実機に書き込む前に確認できる |

---

## 📦 インストール

**コマンドライン（CLI）で導入してください。** GUI 版インストーラ「StampFly Setup」もありますが、
まだ動作の安定性を確認できていないため、以下の CLI 手順で進めてください。
GUI 版の説明は **[GUI インストーラガイド](docs/guides/gui-installer.md)** にあります。

手順はどの OS でも同じ 3 段階です。コードブロックは 1 つずつコピーして端末に貼り付け、
実行が終わるのを待ってから次のブロックに進んでください。

| 段階 | 内容 |
|------|------|
| ① 前提ツール | Git と Python 3.10〜3.12（推奨 3.12）を入れる |
| ② 取得と導入 | リポジトリを取得し、インストーラを実行する（ESP-IDF v5.5.2 も導入される） |
| ③ 有効化と診断 | 開発環境を有効化し、`sf doctor` で確認する |

インストーラ（②）の途中で ESP-IDF の導入を尋ねられたら **1（Install ESP-IDF v5.5.2）** を選びます。
ESP-IDF のダウンロードを含むため、時間に余裕のあるときに実行してください。

### Windows

コマンドプロンプト（CMD）で実行します。WSL は不要です。

**① Git と Python 3.12 を導入**（すでにあれば省略）。末尾の 2 つのオプションは、初回に出る利用規約への同意の質問を省くためのものです。
終わったら CMD を一度閉じて開き直します。

```cmd
winget install Git.Git --accept-source-agreements --accept-package-agreements
winget install Python.Python.3.12 --accept-source-agreements --accept-package-agreements
```

**② リポジトリを取得し、インストーラを実行**。途中の質問には画面の指示に従って答えます。

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
install.bat
```

**③ 開発環境を有効化して診断**。新しい CMD を開くたびに `setup_env.bat` を実行します。

```cmd
setup_env.bat
sf doctor
```

**→ [Windows セットアップ（詳細・トラブル対応）](docs/setup/windows.md)**

### macOS

ターミナルで実行します。

**① Homebrew を導入**（すでにあれば省略）。Enter を押し、Mac のログインパスワードを入力します。
Xcode Command Line Tools が無い場合はここで一緒に導入されます。
終了時に「Next steps」として表示される、brew を PATH に追加するコマンドをそのままコピーして実行してください。

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

続けて、ビルドに必要なツールと Python 3.12 を入れます。

```bash
brew install cmake ninja dfu-util ccache python@3.12
```

**② リポジトリを取得し、インストーラを実行**。途中の質問には画面の指示に従って答えます。

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**③ 開発環境を有効化して診断**。新しいターミナルを開くたびに `source setup_env.sh` を実行します。

```bash
source setup_env.sh
sf doctor
```

**→ [macOS セットアップ（詳細・トラブル対応）](docs/setup/macos.md)**

### Ubuntu

ターミナルで実行します（Ubuntu 22.04 LTS 以降）。

**① Git・Python・ビルドツールを導入**。パスワードを求められたら入力します。

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

**② リポジトリを取得し、インストーラを実行**。途中の質問には画面の指示に従って答えます。

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**③ 開発環境を有効化して診断**。新しいターミナルを開くたびに `source setup_env.sh` を実行します。

```bash
source setup_env.sh
sf doctor
```

実機や送信機を USB で使うときは、シリアルポートの権限を一度だけ設定します（反映には再ログインが必要）。

```bash
sudo usermod -a -G dialout $USER
```

**→ [Linux セットアップ（詳細・トラブル対応）](docs/setup/linux.md)**

`sf doctor` が問題なしと表示すれば導入完了です。
後で最新版に更新するときは `sf upgrade` を実行します（**→ [アップグレードガイド](docs/guides/upgrading.md)**）。

---

## 🎮 まずはシミュレータで飛ばしてみよう！

**実機がなくても大丈夫。** コントローラとPCがあれば、今すぐドローン操縦を体験できます。

### VPython版（軽量・ブラウザ表示）

```bash
sf sim run vpython
```

### Genesis版（高精度物理エンジン）

```bash
sf sim run genesis
```

コントローラをUSB HIDモードに切り替えてPCに接続すれば、
3Dビューでドローンを自由に飛ばせます。

| シミュレータ | 特徴 |
|-------------|------|
| VPython版 | 軽量、センサモデル充実、SILS/HILS対応 |
| Genesis版 | 2000Hz物理演算、物理量ベース制御 |

> **注:** HILS（Hardware In the Loop Simulation）は `simulator/vpython/interfaces/` にPython側インターフェースのみ実装済みで、ファームウェア側（実機と接続する受信処理）は未実装のため現状は接続できません。詳細は [`simulator/README.md`](simulator/README.md) を参照。

**→ [シミュレータで遊ぶ（詳細手順）](docs/getting-started.md#0-まずはシミュレータで遊んでみよう)**

---

## 🔗 すぐに使えるリソース

| リソース | 説明 |
|---------|------|
| [🌐 プロジェクト紹介](https://m5fly-kanazawa.github.io/stampfly_ecosystem/) | 実機3Dモデル付きランディングページ |
| [📚 ドキュメント](https://m5fly-kanazawa.github.io/stampfly_ecosystem/docs/) | 全ドキュメントを検索・閲覧 |
| [🔌 Web書き込み](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | ビルド不要、ブラウザから実機に書き込み |
| [🖥️ StampFly Flasher](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) | デスクトップ書き込みアプリ（Windows/macOS・Python不要） |
| [📦 ビルド済みファームウェア](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) | GitHub Releases（vehicle / controller） |

---

## 🚀 sf CLI クイックスタート

**sf CLI** は、ビルド、書き込み、モニタ、ログ取得などをシンプルに実行できる統合コマンドラインツールです。

```bash
# 環境をアクティブ化
source setup_env.sh

# 環境診断
sf doctor

# ビルド → 書き込み → モニタ
sf build vehicle && sf flash vehicle -m
```

### よく使うコマンド

| コマンド | 説明 |
|---------|------|
| `sf build vehicle` | 機体ファームウェアをビルド |
| `sf flash vehicle -m` | 書き込み後にモニタを開く |
| `sf monitor` | シリアルモニタを開く |
| `sf log wifi` | WiFiテレメトリをキャプチャ |
| `sf cal gyro` | ジャイロキャリブレーション |
| `sf sim run vpython` | シミュレータを起動 |
| `sf app new my_ctrl` | 自分の制御則プロジェクトを作成（研究・実験用） |

**→ [コマンドリファレンス](docs/commands/README.md)** | **[セットアップガイド](docs/setup/README.md)**

---

## 工場出荷状態に戻す

ワークショップやカスタムファームウェアの書き込みにより、機体・送信機のファームウェアは上書きされます。
工場出荷状態に戻したい場合は、以下のコマンドを実行してください：

```bash
# 機体を工場出荷状態に戻す
sf flash vehicle --legacy

# 送信機を工場出荷状態に戻す
sf flash controller --legacy
```

> **注意:** 事前に `source setup_env.sh` で開発環境をセットアップしてください。デバイスを USB 接続した状態で実行してください。

---

## ディレクトリ構成

```
stampfly_ecosystem/
├── docs/           # ドキュメント
├── firmware/       # 組込みファームウェア
│   ├── vehicle/     # 機体ファームウェア（主力）
│   ├── vehicle_old/ # レガシー機体ファームウェア（凍結）
│   ├── controller/  # 送信機ファームウェア
│   └── common/      # 共有コード（ESP-NOW プロトコル構造体）
├── protocol/       # 通信プロトコル仕様（構築中）
├── control/        # 制御設計資産（構築中）
├── analysis/       # 実験データ解析（構築中）
├── tools/          # 補助ツール（構築中）
├── simulator/      # 3Dフライトシミュレータ
├── ros/            # ROS連携（構築中）
├── examples/       # 学習用サンプル
└── third_party/    # 外部依存
```

---

## 始めよう

**→ [Getting Started（環境構築〜初フライト）](docs/getting-started.md)**

ESP-IDFのセットアップから、ファームウェアのビルド、
機体と送信機のペアリング、そして初フライトまで。
すべての手順をステップバイステップで解説しています。

---

## ワークショップで本格的に学ぶ

Getting Started で初フライトができたら、次は**ワークショップ**で制御の仕組みをじっくり学びましょう。

| レッスン | テーマ | 学べること |
|---------|--------|-----------|
| Lesson 1 | 環境構築 | ESP-IDF セットアップ、ビルド＆書き込み |
| Lesson 2 | コントローラ入力 | ESP-NOW 通信、スティック値の読み取り |
| Lesson 3 | LED 制御 | システム状態の可視化 |
| Lesson 4 | IMU センサー | 加速度・ジャイロデータの取得と理解 |
| Lesson 5 | モータ制御 | PWM によるモータ個別制御 |
| Lesson 6 | 姿勢推定 | 相補フィルタによる姿勢角の算出 |
| Lesson 7 | レート P 制御 | 角速度フィードバックによる安定化 |
| Lesson 8 | PID 制御 | 姿勢角の PID 制御と飛行 |

**→ [ワークショップスライド](docs/events/README.md)** | **[ワークショップガイド](docs/events/stampfly_workshop/)**

---

## 技術仕様

| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3（M5Stamp S3） |
| フレームワーク | ESP-IDF v5.5.2 + FreeRTOS |
| 姿勢推定 | ESKF（Error-State Kalman Filter） |
| 通信 | ESP-NOW + WiFi（テレメトリ） |
| センサー | BMI270, BMM150, BMP280, VL53L3CX, PMW3901 |

---

## 詳細ドキュメント

| ドキュメント | 説明 |
|-------------|------|
| [ドキュメント目録](docs/DOCUMENT_INDEX.md) | リポジトリ内の全ドキュメントの目録 |
| [物理パラメータリファレンス](docs/architecture/stampfly-parameters.md) | 機体の物理パラメータ（質量・慣性・モータ特性等）の確定値 |
| [stampfly_physical.yaml](control/models/stampfly_physical.yaml) | 物理パラメータの機械可読 SSOT（Single Source of Truth） |

---

## ライセンス

MIT License

---

---

<a id="english"></a>

# StampFly Ecosystem

## For those who want to build their own drone control

**StampFly Ecosystem** is an educational and research platform for
**learning, implementing, and experimenting** with drone control.

"I learned PID control from textbooks, but I want to build something that actually flies."
"I want to implement my own attitude estimation algorithm and test it."
"I need a flight experiment platform for my research."

This ecosystem exists for you.

---

## What can you do?

| Capability | Description |
|-----------|-------------|
| **Practice in the simulator** | Fly a 3D simulator on your PC with the transmitter, no real drone needed |
| **Fly the real drone** | Flash the firmware and fly StampFly with the transmitter. Four flight modes included (ACRO / STABILIZE / Altitude Hold / Position Hold) |
| **Read sensor values** | Read IMU, barometer, ToF and optical-flow values in real time, display and record them |
| **Receive transmitter commands** | Receive stick and button values on the vehicle and use them from your own program |
| **Write your own flight program** | Create your own project with `sf app new` and write the control law and flight behavior yourself |
| **Verify your flight program in SILS** | Check your flight program in SILS (Software In the Loop Simulation: the firmware itself flying on your PC) before flashing it to the real drone |

---

## 📦 Installation

**Install from the command line (CLI).** A GUI installer, "StampFly Setup", also exists,
but its stability is not yet confirmed, so please follow the CLI steps below.
The GUI installer is described in the **[GUI Installer Guide](docs/guides/gui-installer.md)**.

The procedure is the same three stages on every OS. Copy one code block at a time into
your terminal, wait for it to finish, then move on to the next block.

| Stage | What you do |
|-------|-------------|
| 1. Prerequisites | Install Git and Python 3.10–3.12 (3.12 recommended) |
| 2. Get and install | Clone the repository and run the installer (it also installs ESP-IDF v5.5.2) |
| 3. Activate and check | Activate the development environment and run `sf doctor` |

When the installer (stage 2) asks about ESP-IDF, choose **1 (Install ESP-IDF v5.5.2)**.
It downloads ESP-IDF, so run it when you have some time.

### Windows

Run in Command Prompt (CMD). WSL is not required.

**1. Install Git and Python 3.12** (skip if already installed). The two trailing options skip the
license-agreement questions that appear on first use. Close and reopen CMD when done.

```cmd
winget install Git.Git --accept-source-agreements --accept-package-agreements
winget install Python.Python.3.12 --accept-source-agreements --accept-package-agreements
```

**2. Clone the repository and run the installer.** Answer its questions as prompted on screen.

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
install.bat
```

**3. Activate the environment and check.** Run `setup_env.bat` in every new CMD window.

```cmd
setup_env.bat
sf doctor
```

**→ [Windows Setup (details and troubleshooting)](docs/setup/windows.md)**

### macOS

Run in Terminal.

**1. Install Homebrew** (skip if already installed). Press Enter and type your Mac login password.
If the Xcode Command Line Tools are missing, they are installed here as well.
When it finishes, copy and run the commands shown under "Next steps" to add brew to your PATH.

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Then install the build tools and Python 3.12.

```bash
brew install cmake ninja dfu-util ccache python@3.12
```

**2. Clone the repository and run the installer.** Answer its questions as prompted on screen.

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**3. Activate the environment and check.** Run `source setup_env.sh` in every new terminal.

```bash
source setup_env.sh
sf doctor
```

**→ [macOS Setup (details and troubleshooting)](docs/setup/macos.md)**

### Ubuntu

Run in a terminal (Ubuntu 22.04 LTS or later).

**1. Install Git, Python and the build tools.** Type your password when asked.

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

**2. Clone the repository and run the installer.** Answer its questions as prompted on screen.

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**3. Activate the environment and check.** Run `source setup_env.sh` in every new terminal.

```bash
source setup_env.sh
sf doctor
```

To use the drone or transmitter over USB, grant serial-port permission once (log out and back in to apply).

```bash
sudo usermod -a -G dialout $USER
```

**→ [Linux Setup (details and troubleshooting)](docs/setup/linux.md)**

Installation is complete when `sf doctor` reports no problems.
To update later, run `sf upgrade` (**→ [Upgrading Guide](docs/guides/upgrading.md)**).

---

## 🎮 Try the Simulator First!

**No drone needed.** With just a controller and PC, you can experience drone piloting right now.

### VPython Version (Lightweight, Browser)

```bash
sf sim run vpython
```

### Genesis Version (High-Precision Physics)

```bash
sf sim run genesis
```

Switch the controller to USB HID mode and connect to your PC.
You can fly a drone freely in the 3D view.

| Simulator | Features |
|-----------|----------|
| VPython | Lightweight, rich sensor models, SILS/HILS |
| Genesis | 2000Hz physics, physical-unit control |

> **Note:** HILS (Hardware In the Loop Simulation) has a Python-side interface only, in `simulator/vpython/interfaces/`. The firmware-side receiver is not implemented, so it cannot currently be connected. See [`simulator/README.md`](simulator/README.md) for details.

**→ [Play with the Simulator (Detailed Steps)](docs/getting-started.md#0-try-the-simulator-first)**

---

## 🔗 Quick Links

| Resource | Description |
|----------|-------------|
| [🌐 Project Overview](https://m5fly-kanazawa.github.io/stampfly_ecosystem/) | Landing page with interactive 3D drone model |
| [📚 Documentation](https://m5fly-kanazawa.github.io/stampfly_ecosystem/docs/) | Browse and search all documentation |
| [🔌 Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | Flash real hardware from the browser, no build required |
| [🖥️ StampFly Flasher](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) | Desktop flashing app (Windows/macOS, no Python required) |
| [📦 Pre-built Firmware](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) | GitHub Releases (vehicle / controller binaries) |

---

## 🚀 sf CLI Quick Start

**sf CLI** is an integrated command-line tool for build, flash, monitor, log capture, and more.

```bash
# Activate environment
source setup_env.sh

# Run diagnostics
sf doctor

# Build → Flash → Monitor
sf build vehicle && sf flash vehicle -m
```

### Common Commands

| Command | Description |
|---------|-------------|
| `sf build vehicle` | Build vehicle firmware |
| `sf flash vehicle -m` | Flash and open monitor |
| `sf monitor` | Open serial monitor |
| `sf log wifi` | Capture WiFi telemetry |
| `sf cal gyro` | Gyro calibration |
| `sf sim run vpython` | Run simulator |
| `sf app new my_ctrl` | Create your own control-law project (research/experiments) |

**→ [Command Reference](docs/commands/README.md)** | **[Setup Guide](docs/setup/README.md)**

---

## Restore Factory Firmware

Workshop lessons and custom firmware will overwrite the factory firmware on your vehicle and controller.
To restore the factory state, run:

```bash
# Restore vehicle to factory firmware
sf flash vehicle --legacy

# Restore controller to factory firmware
sf flash controller --legacy
```

> **Note:** Run `source setup_env.sh` to set up the development environment first. Connect the device via USB before running.

---

## Directory Structure

```
stampfly_ecosystem/
├── docs/           # Documentation
├── firmware/       # Embedded firmware
│   ├── vehicle/     # Vehicle firmware (primary)
│   ├── vehicle_old/ # Legacy vehicle firmware (frozen)
│   ├── controller/  # Transmitter firmware
│   └── common/      # Shared code (ESP-NOW protocol structs)
├── protocol/       # Communication protocol spec (WIP)
├── control/        # Control design assets (WIP)
├── analysis/       # Experiment data analysis (WIP)
├── tools/          # Utility tools (WIP)
├── simulator/      # 3D flight simulator
├── ros/            # ROS integration (WIP)
├── examples/       # Learning examples
└── third_party/    # External dependencies
```

---

## Get Started

**→ [Getting Started (Setup to First Flight)](docs/getting-started.md)**

From ESP-IDF setup to firmware build,
pairing vehicle and controller, and your first flight.
All steps explained step-by-step.

---

## Learn Through the Workshop

Once you've completed your first flight with Getting Started, dive deeper with the **Workshop** to learn how drone control really works.

| Lesson | Topic | What You'll Learn |
|--------|-------|-------------------|
| Lesson 1 | Environment Setup | ESP-IDF setup, build & flash |
| Lesson 2 | Controller Input | ESP-NOW communication, reading stick values |
| Lesson 3 | LED Control | Visualizing system state |
| Lesson 4 | IMU Sensor | Reading accelerometer & gyroscope data |
| Lesson 5 | Motor Control | Individual motor control via PWM |
| Lesson 6 | Attitude Estimation | Computing attitude angles with complementary filter |
| Lesson 7 | Rate P Control | Stabilization via angular rate feedback |
| Lesson 8 | PID Control | Attitude PID control and flight |

**→ [Workshop Slides](docs/events/README.md)** | **[Workshop Guide](docs/events/stampfly_workshop/)**

---

## Technical Specifications

| Item | Specification |
|------|---------------|
| MCU | ESP32-S3 (M5Stamp S3) |
| Framework | ESP-IDF v5.5.2 + FreeRTOS |
| Pose Estimation | ESKF (Error-State Kalman Filter) |
| Communication | ESP-NOW + WiFi (telemetry) |
| Sensors | BMI270, BMM150, BMP280, VL53L3CX, PMW3901 |

---

## Further Documentation

| Document | Description |
|----------|-------------|
| [Document Index](docs/DOCUMENT_INDEX.md) | Index of all documentation in this repository |
| [Physical Parameters Reference](docs/architecture/stampfly-parameters.md) | Confirmed values for the vehicle's physical parameters (mass, inertia, motor characteristics, etc.) |
| [stampfly_physical.yaml](control/models/stampfly_physical.yaml) | Machine-readable SSOT (Single Source of Truth) for physical parameters |

---

## License

MIT License
