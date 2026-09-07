# 独自プログラム開発入門

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本書は、機体（StampFly）とシミュレータで既に飛ばせるようになった人が、次に **自分の制御則・推定器・判断ロジック** を書くための入口ガイドである。前提として、リポジトリ直下の `README.md` の「実際に飛ばしてみよう」まで済んでいること（ファームウェアの書き込みと送信機での飛行を一度経験していること）を想定する。

対象は `firmware/vehicle`（vehicle 本体。実機・SILS 双方の主力ファームウェア）の **L1 Topic API**（`sf::api::` 名前空間。トピックの最新値を読む関数群と、`IController`/`IEstimator` という差し替え可能なインターフェース）である。HAL（Hardware Abstraction Layer: センサ・アクチュエータのドライバ層、L2）や BSP（Board Support Package: I2C/SPI 等の共有ハードウェア資源を初期化・所有する層、L3）を自分で書く話は対象外。将来の文書で扱う。

### 現在地の正直な説明（2026-09-07 時点）

`sf app` コマンドで作った自作プロジェクトは、現状 vehicle 本体とは別の「**単独ベンチ**」（HAL・トピック・実ファームを使わない、または実機センサを読むだけの独立した ESP-IDF プロジェクト）である。自作コントローラを実機や SILS（Software In the Loop Simulation: ファームウェアそのものを PC 上で動かす試験）で飛ばすには、vehicle 本体に**手作業で組み込む**必要がある。その手順は本書 §5 に書く。

`sf app` を L1 の正式な入口にし、自作プログラムを vehicle 本体に組み込んで実機と SILS の両方で動かせるようにする計画が [`docs/plans/sf-app-sils-plan.md`](../plans/sf-app-sils-plan.md) として進行中である（本書 §6）。実装されるまでは、本書 §5 の手作業が L1 のコントローラ・推定器を飛ばす経路である。

## 2. 4 つの層と、この文書が扱う層

vehicle は、学習者がレベルに応じて入口を選べる4つの階層を提供する（`firmware/vehicle/docs/architecture.md` の「学習者の入口（4 階層アクセス）」節）。

| 層 | 名前空間 | 典型ユーザー | できること |
|----|---------|------------|----------|
| L0: Workshop API | `ws::*` | 初心者 | `setup()`/`loop_400Hz(dt)` と `ws::motor_set_duty()` 等の関数だけでフライト制御そのものを組み立てる |
| **L1: Topic API（本書が扱う層）** | `sf::api::*` | 推定・制御・ガイダンス学習者 | トピック（後述）を読み書きし、`IController`（制御則インターフェース）/`IEstimator`（状態推定インターフェース）を自分の実装に差し替える |
| L2: HAL Direct | `stampfly::*Wrapper` | ハードウェア学習者 | センサドライバを直接呼ぶ |
| L3: BSP Internal | `sf::internal::board` | ファーム実装者 | 起動順序・共有ハードウェア資源の管理そのものを変更する |

L0 は「最下層の飛行制御そのものを自分で書く」層で、`ws::gyro_x()` のような関数を直接組み合わせて姿勢制御まで作る。これに対して L1 は「vehicle 本体が既に持っている推定・制御の仕組み（トピック、`IController`、`IEstimator`）を使って、その一部品を差し替える」層である。すでに動いているカスケード PID 制御器やセンサ融合の仕組みをそのまま使い、自分が変えたい一点（例えばヨー軸のトルク配分や、ある観測の使い方）だけを書き換える。

各層は並列に共存する。L1 学習者が L0 の骨格を経由する必要はない。

## 3. vehicle 本体の中で自分のコードが動く場所

### Pub-Sub（発行/購読）の考え方

vehicle 本体のコンポーネント間通信は、直接関数を呼び合うのではなく **Topic**（データの入れ物）を介した **Pub-Sub**（Publish=発行 / Subscribe=購読）方式で行う。発行側（Publisher）は「このデータが最新です」と `publish()` するだけで、誰が読むかを知らない。購読側（Subscriber）は `latest()` で「今ある最新値」を取得するだけで、誰が発行したかを知らない。センサタスクがトピックに値を書き込み、推定・制御タスクがそれを読んで計算し、また別のトピックに書き込む——あなたが書くコードは、この流れのどこかの部品（推定器・制御器・トピックを読むだけの監視タスク等）になる。

### 主要トピック

トピックの正本は `firmware/vehicle/docs/topic_reference.md` §3。以下は L1 学習者が主に触れるものの抜粋。

| トピック名 | データ型 | 発行元 | 購読者 | レート |
|-----------|---------|-------|-------|-------|
| `sensor_imu` | `ImuData` | ImuTask | ImuTask 内の推定器 | 400Hz |
| `estimate_state` | `StateEstimate` | ImuTask | ControlTask, TelemetryTask | 400Hz |
| `command_setpoint` | `CommandSetpoint` | CommTask（ESP-NOW 受信） | ControlTask | 50Hz |
| `control_output` | `ControlOutput` | ControlTask | TelemetryTask | 400Hz |
| `actuator_motor` | `MotorOutput` | ControlTask | モータドライバ | 400Hz |
| `system_mode` | `SystemMode` | StateTask | ControlTask, NotifyTask | イベント駆動 |
| `sensor_power` | `PowerData` | PowerTask | TelemetryTask, FailsafeTask | 10Hz |

### 差し替えられる部品: `IController` と `IEstimator`

vehicle 本体は制御則・状態推定をそれぞれインターフェース（実装の中身を問わず「この関数群さえ実装すればよい」という規約）越しに扱う。

`IController`（`firmware/vehicle/components/sf_controller/include/controller.hpp`）は12個のメソッドを持つ。中心は次の1つ。

```cpp
virtual ControlOutput compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt
) = 0;
```

`compute()` は400Hzで、IMU（慣性計測装置）に同期して呼ばれる。現在の推定状態とパイロット指令を受け取り、機体座標系での推力・トルクを返す。このほか `reset()`（内部状態の初期化）と `onModeChange()`（飛行モード変更の通知）は必ず実装する。残り 9 個（`onLanding()`、`onTakeoff()`、`onTakeoffComplete()`、`isTakeoffComplete()`、`setGuidanceTarget()`、`isGuidanceActive()`、`startExcitation()`、`fetchSysidResult()`、`reloadParams()`）は自動離着陸・ガイダンス目標・システム同定・パラメータ再読込みへの対応で、何もしない既定実装を持つため、必要なものだけ上書きすればよい。現行の唯一の実装は `PidController`（カスケードPID: 姿勢→レートの2段構成）。

`IEstimator`（`firmware/vehicle/components/sf_estimator/include/estimator.hpp`）は状態推定側の同じ考え方。中心は次の2つ。

```cpp
virtual void predict(const ImuData& imu, float dt) = 0;
virtual StateEstimate getState() const = 0;
```

`predict()` はIMUレート（400Hz）でジャイロ・加速度から状態を前方に伝搬する予測ステップ、`getState()` は現在の推定値を返す。他に `updateTof()`/`updateFlow()`/`updateMag()`/`updateBaro()`（各センサの観測更新）、`reset()`、`resetPositionVelocity()` 等がある。現行の実装は `EskfEstimator`（ESKF: Error-State Kalman Filter、15状態）と `ComplementaryEstimator`（相補フィルタ、姿勢のみ）の2つ。

### 読み取り専用の Topic API（8関数）

`sf::api::`（`firmware/vehicle/components/sf_api/include/sf_api.hpp`）は、L1 学習者向けの単一公開ヘッダで、以下の8関数を提供する。全てトピックの `latest()`（最新値のコピーを返す）を薄くラップしたものである。

| 関数 | 戻り値の型 | 内容 |
|------|-----------|------|
| `imu_latest()` | `ImuData` | 最新のIMU値 |
| `estimate_latest()` | `StateEstimate` | 最新の状態推定値（姿勢・位置・速度） |
| `command_latest()` | `CommandSetpoint` | 最新のパイロット指令（スロットル/ロール/ピッチ/ヨー） |
| `control_latest()` | `ControlOutput` | 最新の制御出力（推力・トルク） |
| `motor_latest()` | `MotorOutput` | 最新のモータ duty 値 |
| `power_latest()` | `PowerData` | 最新の電源値（電圧・電流） |
| `current_mode()` | `SystemMode` | 現在のシステムモード（ARM状態・フライトモード） |
| `is_armed()` | `bool` | 現在ARM状態か |

**書き込み系のAPI（アクチュエータ操作、モード要求、ガイダンス目標の外部設定等）はまだ無い。** `sf_api.hpp` のコメントには、これらが将来のマイルストーンに先送りされていると明記されている。現時点でパイロット指令やモードを外から書き換える経路は L1 には用意されていない。

## 4. 今すぐ試せること（単独ベンチ）

vehicle 本体を再ビルドせずにすぐ試せるのが、`sf app` コマンドが作る単独ベンチである。

```bash
source setup_env.sh
sf app new my_ctrl
```

これで `firmware/apps/my_ctrl` に、既定の複製元 `10_custom_controller` のコピーが作られる。編集・ビルド・書き込みは以下。

```bash
sf app edit my_ctrl
sf app build my_ctrl
sf app flash my_ctrl -m
```

`10_custom_controller` は `IController` を実装した薄いラッパークラス `LearnerController` を、**合成信号（正弦波）に対して `compute()` を呼ぶだけ**で動かすベンチである。実際のセンサにもモータにも一切触れず、実機を飛ばさない。ロール/ピッチ/ヨーのトルク出力がベンチ内の合成外乱にどう応答するかを、USBシリアルモニタで確認できる。

トピックの読み取りだけを学びたい場合は、`--from 09_topic_api_hello` から作ると良い。

```bash
sf app new my_hello --from 09_topic_api_hello
```

こちらは実際のBMI270（IMUセンサ）をSPIで読み、相補フィルタで `estimate_state` トピックに publish するので、`sf::api::estimate_latest()` が本物のセンサ値を返す様子を確認できる（ただし起動校正・フェイルセーフ・離着陸ロジックは無く、これも飛行はしない）。

## 5. 自作コントローラを vehicle 本体で飛ばす（現行の手順）

自作した `IController` 実装を実際の飛行制御パイプラインで使うには、vehicle 本体を再ビルドする必要がある。`10_custom_controller` の README §8「実機で飛ばすレシピ」に基づく手順を、迷わない順番でまとめる。

### 5.1 コンポーネント化

`learner_controller.hpp`/`.cpp` を新しいコンポーネントとしてコピーする。

```bash
mkdir -p firmware/vehicle/components/sf_controller_learner
```

新コンポーネントの `CMakeLists.txt` に `REQUIRES sf_controller sf_controller_pid` を設定する。

### 5.2 main コンポーネントの REQUIRES に追加

`firmware/vehicle/main/CMakeLists.txt` の `REQUIRES` に、新コンポーネント名（例: `sf_controller_learner`）を追加する。

### 5.3 include の追加

`firmware/vehicle/tasks/control_task.cpp` の既存include群の下に、自作クラスのヘッダを追加する。

```cpp
#include "learner_controller.hpp"
```

### 5.4 コントローラ選択の1行を差し替える

`control_task.cpp` にある、コントローラを決めている唯一の1行を書き換える。

```cpp
static sf::LearnerController controller;
```

### 5.5 SILS で必ず先に確認する

実機に書き込む前に、SILSで確認する。この順番を必ず守る。

```bash
sf sils build --target vehicle
```

続けて、実在するシナリオ（例: `alt_flight.scn` = 高度維持モードでの離陸→ホバー→着陸、`acro_flight.scn` = ACRO モードでの姿勢制御）に対して実行する。

```bash
sf sils scenario simulator/sils/scenarios/alt_flight.scn --target vehicle
```

終了コード0（PASS）であることを確認する。合否基準（`.expect` ファイル）を満たさない場合は、実機に進む前にコントローラの実装を見直す。

### 5.6 実機ビルド・書き込み

SILSで合否判定がPASSしたら、実機用にビルドし直して書き込む。

```bash
sf build vehicle
sf flash vehicle -m
```

### 5.7 ログで確認する

飛行後、WiFi経由でテレメトリを取得し、解析する。

```bash
sf log wifi -d 30
sf log analyze
```

`sf log analyze` はジャイロ統計・入力-応答相関・振動周波数解析・PIDチューニングの推奨事項を表示する。グラフで確認したい場合は `sf log viz` を使う。

### 5.8 安全上の注意

`docs/guides/safety.md` に記載の飛行前チェックリスト（プロペラガードの損傷確認、飛行エリア2m×2m以上の確保、バッテリー30%以上、緊急停止方法の事前確認）に必ず従う。自作コントローラの初回飛行は、想定外の挙動が起きやすい。安定して飛ぶことを確認できるまでは、緊急停止（`sf emergency` またはPython SDKの `drone.emergency()`）をすぐ実行できる態勢で臨む。

## 6. これからの形（計画）

`sf app` を L1 の入口として完成させる計画が [`docs/plans/sf-app-sils-plan.md`](../plans/sf-app-sils-plan.md) として進行中である。要旨は次の通り。

| 項目 | 内容 |
|------|------|
| アプリフック | vehicle 本体に、ユーザープログラムが実装を提供できる関数を 1 組定義する（仮称 `sf::app::controller()`、`sf::app::estimator()`、`sf::app::start()`）。`control_task.cpp` の 1 行固定と `imu_task.cpp` の推定器選択を、このフックを呼ぶ形に置き換える |
| 取り込み方式 | `firmware/apps/<name>/*.cpp` を vehicle の main コンポーネントに直接コンパイルする。実機ビルド（ESP-IDF）と SILS ビルド（`emu_vehicle`）の両方が同じ変数 `SF_APP_DIR` でそのディレクトリを取り込む |
| テンプレート | `11_app_controller`（`PidController` に委譲しつつ 1 軸だけ自分の式に置き換える `IController`）と `12_app_task_hello`（`estimate_latest()` を読んで記録するタスク）。`sf app new` の既定の複製元は 11 になる。09 / 10 はベンチとして残り、`sf app list` に「SILS 不可」と表示される |
| L0 との関係 | workshop 骨格（L0）は置き換えない。両者は並列に共存する別の入口 |

実装後の使い方は次のようになる見込みである。

```bash
sf app new my_ctrl
```

```bash
sf app build my_ctrl
```

```bash
sf app sils my_ctrl
```

`sf app build` が実機ビルド、`sf app sils` が SILS ビルドで、どちらも `firmware/apps/my_ctrl/` の同じソースを vehicle 本体に組み込んで動かす（Code Identity: 実機と SILS で同じコードが動く）。**実装されたら、本章の流れが §5 の手作業手順を置き換える。** トピックへの書き込み（ガイダンス目標の設定やモード要求）を L1 から行える API は、その後の段階で別途設計される。

## 7. 書き方の指針

| # | 指針 | 理由 |
|---|------|------|
| 1 | `IController::compute()` は400Hzで呼ばれる。1周期2.5ミリ秒以内に収める | 動的メモリ確保（`new`/`malloc`等、実行中に都度メモリを確保する処理）・`printf`によるログ出力・ブロッキング呼び出し（応答を待って処理が止まる呼び出し）を `compute()` 内に入れると、制御周期を超過し飛行が不安定化する |
| 2 | ゲイン等のパラメータは既存のparam仕組み（NVS: Non-Volatile Storage、電源を切っても消えない保存領域に値が残るパラメータ管理の仕組み）を使う | マジックナンバーを埋め込まず、飛行中の`param set`によるライブチューニングを可能にするため |
| 3 | まず既存の `PidController` に委譲する薄いラッパーを作り、1軸だけ自分の式に置き換える | `10_custom_controller` の設計方針。既存の実績あるカスケードPIDを再利用しつつ、変更点を最小化してデバッグを容易にする |
| 4 | SILS→実機の順を必ず守る | 制御則やパラメータの変更はシミュレーションで裏付けてから実機に載せる、という本プロジェクトの方針。実機での予期せぬ挙動を減らす |
| 5 | 飛行のたびに `sf log wifi` でログを取得し比較する | 変更が実際に効いたか、定性的な印象ではなく数値で確認するため |

## 8. 困ったとき

| 症状 | 確認すること |
|------|-------------|
| ビルドが通らない（未定義シンボル等） | `firmware/vehicle/main/CMakeLists.txt` の `REQUIRES` に新コンポーネント名を追加したか（§5.2） |
| ビルド時に `IController`/`IEstimator` の純粋仮想関数が未実装というエラー | インターフェースの全メソッドを実装したか。`compute()`/`predict()`/`getState()` 等の必須メソッド以外は、既定のno-op実装を持つものもあるため、継承元の宣言（`controller.hpp`/`estimator.hpp`）と照合する |
| SILSシナリオがPASSしない（離陸しない・姿勢が発散する等） | `.expect` ファイルの合否基準を確認し、`compute()`/`predict()` の符号・ゲイン・単位（度かラジアンか等）を見直す |
| 実機で振動が出る | ゲイン過大、または `compute()` 内の処理が重く400Hz（2.5ミリ秒）周期を超過していないか（§7の1） |
| `sf app new` が拒否される | プロジェクト名が予約名（`vehicle`, `vehicle_old`, `controller`, `workshop`, `common`, `apps`）と重複していないか、既に同名ディレクトリが存在していないか |

## 9. 関連文書

| 文書 | 内容 |
|------|------|
| [`docs/plans/sf-app-sils-plan.md`](../plans/sf-app-sils-plan.md) | 本書§6の計画の詳細（現状分析・設計判断・実装フェーズ） |
| [`firmware/vehicle/docs/architecture.md`](../../firmware/vehicle/docs/architecture.md) | 4階層アクセス（L0〜L3）の定義、Pub-Sub全体設計 |
| [`firmware/vehicle/docs/topic_reference.md`](../../firmware/vehicle/docs/topic_reference.md) | トピック一覧のSSOT、使用パターン |
| [`firmware/vehicle/examples/10_custom_controller/README.md`](../../firmware/vehicle/examples/10_custom_controller/README.md) | `IController` 差替え演習、実機で飛ばすレシピ |
| [`firmware/vehicle/examples/09_topic_api_hello/README.md`](../../firmware/vehicle/examples/09_topic_api_hello/README.md) | Topic API 読み取りの最小例 |
| [`firmware/apps/README.md`](../../firmware/apps/README.md) | `firmware/apps/` の使い方 |
| [`docs/commands/sf-app.md`](../commands/sf-app.md) | `sf app` コマンドリファレンス |
| [`docs/commands/sf-log.md`](../commands/sf-log.md) | ログ取得・解析コマンド |
| [`docs/guides/safety.md`](safety.md) | 飛行安全ガイド |

---

<a id="english"></a>

## 1. Overview

This guide is the entry point for readers who can already fly the vehicle (StampFly) both on real hardware and in the simulator, and who now want to write their **own controller, estimator, or decision logic**. It assumes you have completed the "Fly the Real Drone" section of the top-level `README.md` (you have flashed the firmware and flown it once with the transmitter).

This guide covers the **L1 Topic API** of `firmware/vehicle` (the vehicle firmware, the primary firmware for both real hardware and SILS) — the `sf::api::` namespace (functions that read the latest value of a topic) together with the swappable interfaces `IController` and `IEstimator`. Writing your own HAL (Hardware Abstraction Layer — the sensor/actuator driver layer, L2) or BSP (Board Support Package — the layer that initializes and owns shared hardware resources such as I2C/SPI, L3) is out of scope; a future document will cover it.

### An Honest Statement of Where Things Stand (as of 2026-09-07)

A project you create with the `sf app` command currently lives on a **standalone bench** separate from the vehicle firmware itself (an independent ESP-IDF project that uses neither HAL, Topics, nor the real firmware — or that only reads a real sensor). To fly your own controller on real hardware or in SILS (Software In the Loop Simulation — running the firmware itself on a PC), you must currently **wire it into the vehicle firmware by hand**. That procedure is section 5 below.

A plan to make `sf app` the official L1 entry point, so that your program is built into the vehicle firmware and runs on both real hardware and SILS, is in progress: [`docs/plans/sf-app-sils-plan.md`](../plans/sf-app-sils-plan.md) (see section 6). Until it ships, the manual steps in section 5 are the path to fly your own L1 controller or estimator.

## 2. The Four Layers and What This Guide Covers

The vehicle firmware provides four tiers of access so learners can pick an entry point matching their level (see the "Learner Access — 4-Tier Model" section of `firmware/vehicle/docs/architecture.md`).

| Tier | Namespace | Typical User | What You Can Do |
|------|-----------|--------------|------------------|
| L0: Workshop API | `ws::*` | Beginners | Build flight control itself using only `setup()`/`loop_400Hz(dt)` and functions like `ws::motor_set_duty()` |
| **L1: Topic API (covered here)** | `sf::api::*` | Estimation/control/guidance learners | Read and write Topics (below), and swap in your own implementation of `IController` (the controller interface) / `IEstimator` (the state-estimation interface) |
| L2: HAL Direct | `stampfly::*Wrapper` | Hardware learners | Call sensor drivers directly |
| L3: BSP Internal | `sf::internal::board` | Firmware implementers | Change boot ordering and shared hardware resource management itself |

L0 is the tier where you write flight control from scratch, wiring together functions like `ws::gyro_x()` yourself all the way up to attitude control. L1, by contrast, is the tier where you use the estimation/control machinery the vehicle firmware already has (Topics, `IController`, `IEstimator`) and swap out one part of it. You reuse the already-working cascade PID controller or sensor-fusion pipeline as-is, and rewrite only the one thing you want to change (e.g. yaw-axis torque allocation, or how one observation is used).

The tiers coexist in parallel — an L1 learner never has to go through the L0 skeleton.

## 3. Where Your Code Runs Inside the Vehicle Firmware

### The Pub-Sub (Publish/Subscribe) Idea

Components inside the vehicle firmware never call each other's functions directly; they communicate through **Topics** (typed data containers) using a **Pub-Sub** (Publish/Subscribe) pattern. A publisher just calls `publish()` — "here is the newest value" — without knowing who reads it. A subscriber just calls `latest()` — "give me whatever is newest right now" — without knowing who published it. Sensor tasks write into topics; estimation and control tasks read them, compute, and write into other topics. The code you write becomes one part of this flow — an estimator, a controller, or a task that merely watches topics.

### Key Topics

The authoritative topic catalog is `firmware/vehicle/docs/topic_reference.md` §3. Below is the subset an L1 learner mostly touches.

| Topic | Data Type | Publisher | Subscriber | Rate |
|-------|-----------|-----------|------------|------|
| `sensor_imu` | `ImuData` | ImuTask | the estimator inside ImuTask | 400 Hz |
| `estimate_state` | `StateEstimate` | ImuTask | ControlTask, TelemetryTask | 400 Hz |
| `command_setpoint` | `CommandSetpoint` | CommTask (ESP-NOW receive) | ControlTask | 50 Hz |
| `control_output` | `ControlOutput` | ControlTask | TelemetryTask | 400 Hz |
| `actuator_motor` | `MotorOutput` | ControlTask | motor driver | 400 Hz |
| `system_mode` | `SystemMode` | StateTask | ControlTask, NotifyTask | event-driven |
| `sensor_power` | `PowerData` | PowerTask | TelemetryTask, FailsafeTask | 10 Hz |

### Swappable Parts: `IController` and `IEstimator`

The vehicle firmware treats the control law and state estimation through interfaces (a contract specifying only which functions must exist, independent of the implementation).

`IController` (`firmware/vehicle/components/sf_controller/include/controller.hpp`) has 12 methods. The central one is:

```cpp
virtual ControlOutput compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt
) = 0;
```

`compute()` is called at 400 Hz, synchronized with the IMU (Inertial Measurement Unit). It receives the current state estimate and pilot setpoint and returns thrust/torque in the body frame. You must also implement `reset()` (re-initialize internal state) and `onModeChange()` (notification of a flight-mode change). The remaining 9 methods (`onLanding()`, `onTakeoff()`, `onTakeoffComplete()`, `isTakeoffComplete()`, `setGuidanceTarget()`, `isGuidanceActive()`, `startExcitation()`, `fetchSysidResult()`, `reloadParams()`) cover autonomous takeoff/landing, guidance targets, system identification and parameter reload; they have do-nothing default implementations, so override only the ones you need. The only current implementation is `PidController` (a two-stage cascade PID: attitude then rate).

`IEstimator` (`firmware/vehicle/components/sf_estimator/include/estimator.hpp`) follows the same idea on the estimation side. The central methods are:

```cpp
virtual void predict(const ImuData& imu, float dt) = 0;
virtual StateEstimate getState() const = 0;
```

`predict()` is the prediction step, called at IMU rate (400 Hz), that propagates the state forward from gyro/accelerometer data; `getState()` returns the current estimate. There are also `updateTof()`/`updateFlow()`/`updateMag()`/`updateBaro()` (observation updates for each sensor), `reset()`, `resetPositionVelocity()`, and more. The two current implementations are `EskfEstimator` (ESKF: Error-State Kalman Filter, 15 states) and `ComplementaryEstimator` (a complementary filter, attitude only).

### The Read-Only Topic API (8 Functions)

`sf::api::` (`firmware/vehicle/components/sf_api/include/sf_api.hpp`) is the single public header for L1 learners, providing these 8 functions. Every one is a thin wrapper around a topic's `latest()` (which returns a copy of the newest value).

| Function | Return Type | What It Reads |
|----------|-------------|---------------|
| `imu_latest()` | `ImuData` | latest IMU reading |
| `estimate_latest()` | `StateEstimate` | latest state estimate (attitude/position/velocity) |
| `command_latest()` | `CommandSetpoint` | latest pilot command (throttle/roll/pitch/yaw) |
| `control_latest()` | `ControlOutput` | latest control output (thrust/torque) |
| `motor_latest()` | `MotorOutput` | latest motor duty values |
| `power_latest()` | `PowerData` | latest power reading (voltage/current) |
| `current_mode()` | `SystemMode` | current system mode (ARM state / flight mode) |
| `is_armed()` | `bool` | whether the vehicle is currently armed |

**There is no write API yet** (actuator control, mode requests, or externally setting a guidance target). The comments in `sf_api.hpp` explicitly state these are deferred to a future milestone. As of now, L1 has no path to write pilot commands or the mode from the outside.

## 4. What You Can Try Now (Standalone Bench)

The `sf app` command builds a standalone bench you can try immediately, without rebuilding the vehicle firmware.

```bash
source setup_env.sh
sf app new my_ctrl
```

This creates `firmware/apps/my_ctrl` as a copy of the default source, `10_custom_controller`. Edit, build, and flash with:

```bash
sf app edit my_ctrl
sf app build my_ctrl
sf app flash my_ctrl -m
```

`10_custom_controller` runs a thin wrapper class, `LearnerController` (an `IController` implementation), by calling `compute()` **against a synthetic signal (a sine wave)** — it never touches a real sensor or motor and does not fly real hardware. You can watch how roll/pitch/yaw torque output responds to the bench's synthetic disturbance over the USB serial monitor.

If you only want to learn to read topics, start from `--from 09_topic_api_hello` instead:

```bash
sf app new my_hello --from 09_topic_api_hello
```

This one reads a real BMI270 (the IMU sensor) over SPI and publishes to the `estimate_state` topic using a complementary filter, so you can see `sf::api::estimate_latest()` returning real sensor-derived numbers (though it has no boot calibration, failsafe, or takeoff/landing logic, and also does not fly).

## 5. Flying Your Own Controller in the Vehicle Firmware (Current Procedure)

To use your own `IController` implementation in the actual flight-control pipeline, you must rebuild the vehicle firmware itself. The steps below follow section 8 ("Recipe to actually fly this") of the `10_custom_controller` README, reordered so nothing is skipped.

### 5.1 Turn It Into a Component

Copy `learner_controller.hpp`/`.cpp` into a new component.

```bash
mkdir -p firmware/vehicle/components/sf_controller_learner
```

Set `REQUIRES sf_controller sf_controller_pid` in the new component's `CMakeLists.txt`.

### 5.2 Add It to the main Component's REQUIRES

Add the new component's name (e.g. `sf_controller_learner`) to `REQUIRES` in `firmware/vehicle/main/CMakeLists.txt`.

### 5.3 Add the Include

Below the existing includes in `firmware/vehicle/tasks/control_task.cpp`, add your class's header.

```cpp
#include "learner_controller.hpp"
```

### 5.4 Swap the One Line That Selects the Controller

Rewrite the single line in `control_task.cpp` that decides which controller is used.

```cpp
static sf::LearnerController controller;
```

### 5.5 Always Confirm With SILS First

Before flashing real hardware, confirm with SILS. Never skip this order.

```bash
sf sils build --target vehicle
```

Then run it against a scenario that actually exists in the repository (e.g. `alt_flight.scn` — takeoff, hover, landing in altitude-hold mode; `acro_flight.scn` — attitude control in ACRO mode).

```bash
sf sils scenario simulator/sils/scenarios/alt_flight.scn --target vehicle
```

Confirm the exit code is 0 (PASS). If the pass/fail criteria (the `.expect` file) are not met, revisit your controller implementation before moving on to real hardware.

### 5.6 Build and Flash Real Hardware

Once SILS passes, rebuild for real hardware and flash it.

```bash
sf build vehicle
sf flash vehicle -m
```

### 5.7 Check With the Flight Log

After flying, capture telemetry over WiFi and analyze it.

```bash
sf log wifi -d 30
sf log analyze
```

`sf log analyze` prints gyro statistics, input-response correlation, oscillation-frequency analysis, and PID-tuning recommendations. Use `sf log viz` if you want to see it graphically.

### 5.8 Safety Notes

Always follow the pre-flight checklist in `docs/guides/safety.md` (check the propeller guards for damage, secure a flight area of at least 2m x 2m, keep the battery above 30%, and confirm the emergency-stop method beforehand). A first flight with your own controller is more likely to behave unexpectedly than a stock flight. Until you have confirmed stable flight, be ready to execute the emergency stop (`sf emergency`, or `drone.emergency()` from the Python SDK) immediately.

## 6. Where This Is Going (Plan)

A plan to complete `sf app` as the L1 entry point is in progress: [`docs/plans/sf-app-sils-plan.md`](../plans/sf-app-sils-plan.md). In summary:

| Item | Content |
|------|---------|
| App hooks | The vehicle firmware gains one set of functions that a user program can implement (working names `sf::app::controller()`, `sf::app::estimator()`, `sf::app::start()`). The single fixed line in `control_task.cpp` and the estimator selection in `imu_task.cpp` are replaced by calls to these hooks |
| How the code is pulled in | `firmware/apps/<name>/*.cpp` is compiled directly into the vehicle's main component. Both the hardware build (ESP-IDF) and the SILS build (`emu_vehicle`) take that directory through the same variable `SF_APP_DIR` |
| Templates | `11_app_controller` (an `IController` that delegates to `PidController` and lets you replace one axis with your own law) and `12_app_task_hello` (a task that reads `estimate_latest()` and logs it). `sf app new` will default to 11. Examples 09 / 10 remain as benches and `sf app list` marks them "no SILS" |
| Relation to L0 | The workshop skeleton (L0) is not replaced. The two remain parallel, separate entry points |

Once implemented, usage is expected to look like this.

```bash
sf app new my_ctrl
```

```bash
sf app build my_ctrl
```

```bash
sf app sils my_ctrl
```

`sf app build` builds for real hardware and `sf app sils` builds for SILS; both embed the same sources from `firmware/apps/my_ctrl/` into the vehicle firmware (Code Identity: the same code runs on hardware and in SILS). **When this ships, this chapter's flow replaces the manual procedure in section 5.** APIs that let L1 code write to topics (set a guidance target, request a mode) are designed separately in a later phase.

## 7. Writing Guidelines

| # | Guideline | Why |
|---|-----------|-----|
| 1 | `IController::compute()` is called at 400 Hz. Stay within 2.5 ms per cycle | Dynamic memory allocation (`new`/`malloc`, i.e. allocating memory during execution), `printf`-style logging, or blocking calls (calls that stall while waiting for a response) inside `compute()` will overrun the control period and destabilize flight |
| 2 | Use the existing param system (NVS: Non-Volatile Storage, a parameter store that survives power-off) for gains and other tunables | Avoids magic numbers and enables live tuning via `param set` while flying |
| 3 | Start with a thin wrapper that delegates to the existing `PidController`, and replace just one axis with your own formula | `10_custom_controller`'s design principle — reuse the proven cascade PID and minimize the change surface to keep debugging tractable |
| 4 | Always confirm with SILS before real hardware | Project policy: back every control-law or parameter change with simulation before it goes on the real drone. Reduces surprises in flight |
| 5 | Capture a log with `sf log wifi` after every flight and compare | Confirms whether a change actually worked with numbers, not a qualitative impression |

## 8. Troubleshooting

| Symptom | What to Check |
|---------|---------------|
| Build fails (undefined symbol, etc.) | Did you add the new component's name to `REQUIRES` in `firmware/vehicle/main/CMakeLists.txt` (section 5.2)? |
| Build error about unimplemented pure virtual methods of `IController`/`IEstimator` | Did you implement every required method? Check your declaration against `controller.hpp`/`estimator.hpp` — some methods beyond the required ones (`compute()`/`predict()`/`getState()`, etc.) have a default no-op |
| A SILS scenario does not pass (does not take off, attitude diverges, etc.) | Check the `.expect` file's pass criteria, and re-examine the sign, gain, and units (degrees vs. radians, etc.) in `compute()`/`predict()` |
| Real hardware vibrates | Gain too high, or `compute()` is doing too much work and overrunning the 400 Hz (2.5 ms) period (see guideline 1 in section 7) |
| `sf app new` is rejected | Check whether the project name collides with a reserved word (`vehicle`, `vehicle_old`, `controller`, `workshop`, `common`, `apps`), or whether a directory with that name already exists |

## 9. Related Documents

| Document | Content |
|----------|---------|
| [`docs/plans/sf-app-sils-plan.md`](../plans/sf-app-sils-plan.md) | Details of the plan described in section 6 (current-state analysis, design decisions, implementation phases) |
| [`firmware/vehicle/docs/architecture.md`](../../firmware/vehicle/docs/architecture.md) | Definition of the 4-tier access model (L0-L3), overall Pub-Sub design |
| [`firmware/vehicle/docs/topic_reference.md`](../../firmware/vehicle/docs/topic_reference.md) | SSOT for the topic catalog, usage patterns |
| [`firmware/vehicle/examples/10_custom_controller/README.md`](../../firmware/vehicle/examples/10_custom_controller/README.md) | `IController` swap-in exercise, recipe to actually fly it |
| [`firmware/vehicle/examples/09_topic_api_hello/README.md`](../../firmware/vehicle/examples/09_topic_api_hello/README.md) | Minimal Topic API read example |
| [`firmware/apps/README.md`](../../firmware/apps/README.md) | How to use `firmware/apps/` |
| [`docs/commands/sf-app.md`](../commands/sf-app.md) | `sf app` command reference |
| [`docs/commands/sf-log.md`](../commands/sf-log.md) | Log capture and analysis commands |
| [`docs/guides/safety.md`](safety.md) | Flight safety guide |
