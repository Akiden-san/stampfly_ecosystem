# Lesson 7: システム同定 / System Identification

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このレッスンについて

フライトデータからプラントモデル $G_p(s) = K / (s(\tau_m s + 1))$ のパラメータ $K$, $\tau_m$ を同定する。
L5 の P 制御で飛行し、WiFi テレメトリでデータを取得した後、`sf sysid fit` でモデルフィッティングを行う。

### 前提知識

- L05: レート P 制御と初フライト（Kp, rate_max の値を使う）
- L06: システムモデリング（伝達関数、プラントモデル）

## 2. システム同定の仕組み

### アルゴリズム概要

`sf sysid fit` は既定（`--input auto`）で **duty優先方式** を使う: 400Hz で
記録される4モータの `motor_duty_FR/RR/RL/FL` 列をミキサーの逆算式に通し、
レートループ PID がその周期に実際に出力した差動指令 $u(t)$ を直接復元する。
$K_p$ の値を知る必要も、フライト中に一定であることを仮定する必要もない。

```
Data Stream に記録されるデータ（sf log wifi -o *.csv）:
  motor_duty_FR/RR/RL/FL : 4モータ duty [0,1]（400Hz）
  gyro_x                  : ロール角速度実測 [rad/s]

プラント入出力の復元（ミキサー逆算、--kp 不要）:
  u_plant  = mixer_inverse(motor_duty_FR/RR/RL/FL)   ← プラントへの入力
  y_plant  = gyro_x                                   ← プラントの出力

開ループモデルをフィッティング:
  G_p(s) = K / (s·(τm·s + 1))
  minimize |y_simulated − y_plant|²   → K, τm を同定
```

400Hz duty 列の無い旧ログ向けのフォールバックとして、$K_p$ が既知であれば
`rate_ref_roll/pitch/yaw` 列（`ws::set_rate_target` の記録値）から
$u_{plant} = K_p \times (\text{rate\_ref} - \text{gyro})$ を計算する
"kp" 方式もある（`--input kp --kp 0.5` で明示指定）。`sf sysid fit` は
CSV のヘッダ列から自動的に方式を判別する（`--input auto`、既定）。

### なぜ開ループ同定が可能か

duty方式は、閉ループ制御の外側にあるモータ指令そのもの（4モータ duty）を
直接観測して逆算するため、$K_p$ の値にもフィードバック則の仮定にも依存
しない。kp方式（フォールバック）でも、閉ループデータで $K_p$ が既知なら
プラントへの入力 $u(t)$ を計算できるため、閉ループモデルを経由せずに
開ループモデルを直接同定できる。

> **重要（励振不足の落とし穴）:** どちらの方式でも、スティックをほとんど
> 動かさずに一定方向へ持ち続けた区間があると、閉ループの P 制御則
> $u = K_p(\text{target} - y)$ が $u \approx \text{定数} - K_p y$ に潰れ、
> $u$ と $y$ が「プラントの動特性やハードウェアの符号とは無関係に」強く
> 負相関して見える（閉ループ同定の典型的な落とし穴）。`sf sysid fit` は
> この状態を検出すると該当区間を除外し警告するが、そもそも起きないよう
> ステップ2の励振の指示に従うこと。

## 3. 手順

### ステップ 1: ファームウェア準備

1. `sf lesson switch 7` でテンプレートを `user_code.cpp` にコピー
2. `user_code.cpp` を開き、$K_p$ を設定（例: 0.5、L5 で使った値）
3. 角速度目標を計算した直後に `ws::set_rate_target(roll_target, pitch_target, yaw_target)` を呼ぶ（テンプレートに TODO ヒントあり）
4. WiFi チャンネルを設定
5. ビルド & 書き込み: `sf lesson build` → `sf lesson flash`

### ステップ 2: フライト & データ取得

1. PC でテレメトリ受信を開始（`.csv` を指定するとマージ済み Data Stream CSV を直接保存）: `sf log wifi -o flight.csv`
2. ARM → ホバリング → スティック操作でロール・ピッチ・ヨー入力
3. **フライト全体を通じて、各軸のスティックを大きめの振幅で連続的に、
   ランダムっぽく動かし続けること。** 一定方向に持ち続ける時間を作らない
   （2〜3回軽く動かすだけでは全く足りない — 目安として、記録全体での
   `rate_ref` の標準偏差が `rate_max` の 10% 未満だと `sf sysid fit` が
   励振不足として区間を除外し、同定に失敗する）
4. 着陸 → DISARM

`flight.csv` には `timestamp_us, gyro_x/y/z, rate_ref_roll/pitch/yaw, total_thrust` 等が1周期1行で入る（拡張子を `.jsonl` にすると従来通りセンサ種別ごとの JSON Lines で保存され、`sf sysid fit` の入力には使えない）。

### ステップ 3: 同定

```bash
# 全軸を同定（duty優先方式が自動選択される。--kp 不要）
sf sysid fit flight.csv --plot

# 特定軸のみ
sf sysid fit flight.csv --axis roll --plot

# 結果を YAML に保存
sf sysid fit flight.csv -o my_plant.yaml

# 400Hz duty 列の無い旧ログの場合のみ、kp方式にフォールバック
sf sysid fit flight.csv --input kp --kp 0.5 --plot
```

### ステップ 4: L6 理論値と比較

| 軸 | K (同定) | K (L6理論) | τm (同定) | τm (理論) |
|-----|---------|-----------|----------|----------|
| Roll | ? | 102.0 | ? | 0.020 |
| Pitch | ? | 70.0 | ? | 0.020 |
| Yaw | ? | 8.0 | ? | 0.020 |

同定した K, τm から設計 Kp を計算: $K_p = 1/(4\zeta^2 K \tau_m)$

## 4. API

| 関数 | 説明 | 値域 |
|------|------|------|
| `ws::gyro_x/y/z()` | 角速度 | rad/s |
| `ws::rc_roll/pitch/yaw()` | スティック入力 | -1.0 〜 +1.0 |
| `ws::rc_throttle()` | スロットル | 0.0 〜 1.0 |
| `ws::set_rate_target(r,p,y)` | 角速度目標を Data Stream に記録（ロギング専用、制御には無関係） | rad/s |
| `ws::motor_mixer(T,R,P,Y)` | モーターミキサー | --- |
| `ws::led_color(r,g,b)` | LED 色設定 | 0〜255 |
| `ws::set_channel(ch)` | WiFi チャンネル | 1, 6, 11 |

## 5. チャレンジ

- 異なる Kp（例: 0.3, 0.7）で飛行し、同定結果がどう変わるか比較する
- `--time-range` オプションで特定区間のみ分析する
- 同定した K, τm でシミュレーション応答と実測を重ねてプロットする

---

<a id="english"></a>

## 1. Overview

### About This Lesson

Identify plant model parameters $K$ and $\tau_m$ from flight data where
$G_p(s) = K / (s(\tau_m s + 1))$.
Fly with L5's P controller, capture WiFi telemetry, then run `sf sysid fit` for model fitting.

### Prerequisites

- L05: Rate P control and first flight (need Kp, rate_max values)
- L06: System Modeling (transfer function, plant model)

## 2. How System Identification Works

### Algorithm Overview

By default (`--input auto`), `sf sysid fit` uses the **duty-first method**:
it feeds the 400Hz `motor_duty_FR/RR/RL/FL` columns through the mixer's
inverse to directly recover the differential command $u(t)$ the rate-loop
PID actually issued that cycle. There is no need to know $K_p$, or to
assume it stayed constant during the flight.

```
Data Stream columns (sf log wifi -o *.csv):
  motor_duty_FR/RR/RL/FL : 4 motor duties [0,1] (400Hz)
  gyro_x                  : measured roll rate [rad/s]

Plant I/O reconstruction (mixer inverse, no --kp needed):
  u_plant  = mixer_inverse(motor_duty_FR/RR/RL/FL)   <- plant input
  y_plant  = gyro_x                                   <- plant output

Open-loop model fitting:
  G_p(s) = K / (s·(τm·s + 1))
  minimize |y_simulated − y_plant|²   → identify K, τm
```

As a fallback for older logs without the 400Hz duty columns, a "kp" method
computes $u_{plant} = K_p \times (\text{rate\_ref} - \text{gyro})$ from the
`rate_ref_roll/pitch/yaw` columns (recorded by `ws::set_rate_target`) when
$K_p$ is known (`--input kp --kp 0.5`). `sf sysid fit` auto-detects which
method to use from the CSV header (`--input auto`, the default).

### Why Open-Loop Identification Works

The duty method directly observes the motor command itself (4 motor
duties), outside the closed loop, so it needs neither $K_p$ nor any
assumption about the feedback law. The kp fallback also works on
closed-loop data: if $K_p$ is known, the plant input $u(t)$ can be computed
directly, allowing open-loop identification without going through the
closed-loop model.

> **Important (the insufficient-excitation pitfall):** with either method,
> a stretch where the stick barely moves and is held in one direction
> collapses the closed-loop P-control law $u = K_p(\text{target} - y)$ into
> $u \approx \text{const} - K_p y$, making $u$ and $y$ look strongly and
> misleadingly *negatively* correlated -- regardless of the true plant
> dynamics or hardware sign convention (a classic closed-loop
> identifiability pitfall). `sf sysid fit` detects and drops such segments
> with a warning, but follow Step 2's excitation guidance so it doesn't
> happen in the first place.

## 3. Procedure

### Step 1: Firmware Setup

1. Run `sf lesson switch 7` to copy the template to `user_code.cpp`
2. Open `user_code.cpp` and set $K_p$ (e.g., 0.5, same as L5)
3. Right after computing the rate targets, call `ws::set_rate_target(roll_target, pitch_target, yaw_target)` (a TODO hint is in the template)
4. Set WiFi channel
5. Build & flash: `sf lesson build` → `sf lesson flash`

### Step 2: Flight & Data Capture

1. Start telemetry on PC (a `.csv` extension saves the merged Data Stream CSV directly): `sf log wifi -o flight.csv`
2. ARM → hover → apply roll/pitch/yaw stick inputs
3. **Keep moving each axis's stick continuously, with large amplitude, in a
   quasi-random pattern throughout the whole flight.** Never hold one
   direction for long (a couple of light stick taps is nowhere near
   enough -- as a rule of thumb, `sf sysid fit` drops segments as
   insufficiently excited, and fitting fails, when the recorded
   `rate_ref`'s standard deviation is below 10% of `rate_max`)
4. Land → DISARM

`flight.csv` has one row per control cycle with `timestamp_us, gyro_x/y/z, rate_ref_roll/pitch/yaw, total_thrust`, etc. (a `.jsonl` extension instead saves the legacy per-sample JSON Lines format, which `sf sysid fit` cannot read).

### Step 3: Identification

```bash
# Identify all axes (duty-first method is auto-selected -- no --kp needed)
sf sysid fit flight.csv --plot

# Single axis only
sf sysid fit flight.csv --axis roll --plot

# Save results to YAML
sf sysid fit flight.csv -o my_plant.yaml

# Only for older logs without the 400Hz duty columns, fall back to kp method
sf sysid fit flight.csv --input kp --kp 0.5 --plot
```

### Step 4: Compare with L6 Theory

| Axis | K (identified) | K (L6 theory) | τm (identified) | τm (theory) |
|------|---------------|---------------|-----------------|-------------|
| Roll | ? | 102.0 | ? | 0.020 |
| Pitch | ? | 70.0 | ? | 0.020 |
| Yaw | ? | 8.0 | ? | 0.020 |

Compute design Kp from identified parameters: $K_p = 1/(4\zeta^2 K \tau_m)$

## 4. API

| Function | Description | Range |
|----------|-------------|-------|
| `ws::gyro_x/y/z()` | Angular rate | rad/s |
| `ws::rc_roll/pitch/yaw()` | Stick input | -1.0 to +1.0 |
| `ws::rc_throttle()` | Throttle | 0.0 to 1.0 |
| `ws::set_rate_target(r,p,y)` | Record the rate target into the Data Stream (logging only, no effect on control) | rad/s |
| `ws::motor_mixer(T,R,P,Y)` | Motor mixer | --- |
| `ws::led_color(r,g,b)` | LED color | 0-255 |
| `ws::set_channel(ch)` | WiFi channel | 1, 6, 11 |

## 5. Challenge

- Fly with different Kp values (e.g., 0.3, 0.7) and compare identification results
- Use `--time-range` option to analyze specific segments
- Overlay simulation response using identified K, τm with measured data
