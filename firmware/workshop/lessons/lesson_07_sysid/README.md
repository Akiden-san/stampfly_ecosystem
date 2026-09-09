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

400Hz duty 列の無い旧ログでは、$K_p$ を `--kp` で渡すと **間接閉ループ方式**
（`--input indirect`、`auto` はここに自動フォールバックする）が使われる。
こちらは $u=K_p(\text{target}-\text{gyro})$ を「外部入力」として直接
フィットするのではなく、target→gyro の閉ループ伝達関数そのものを
フィットし、既知の $K_p$ から代数的に $K$, $\tau_m$ を逆算する:

```
間接方式（--input indirect、--kp 必須）:
  target(t) は真に外部の信号（パイロットのスティック）、gyro(t) と
  代数的に絡み合っていない → target → gyro の閉ループ応答を直接
  シミュレーションし、実測 gyro と比較して K, τm をフィット
  （u=Kp*(target-gyro) を外部入力として直接使う旧来の "kp" 方式は
  参照する）
```

人間の操縦では持続的な高周波（〜8Hz）励振を安全に作れないため、
$u=K_p(\text{target}-\text{gyro})$ を直接フィットする旧来の "kp" 方式は
実飛行データで破綻しやすい（実測で R² < 0、K が理論値から1〜3桁ズレる例
あり）。間接方式はこの問題を回避し、実飛行データで K を理論値の数%〜
数十%程度まで復元できる（ただし $\tau_m$ は人間操縦データからは高周波
成分不足のため引き続き不確実になりやすい）。旧来の "kp" 方式は比較・
デバッグ用に `--input kp --kp 0.5` で明示指定した場合のみ残っている。
`sf sysid fit` は CSV とオプションから自動的に方式を判別する
（`--input auto`、既定: duty優先 → --kp があれば間接 → それ以外は
旧kp方式）。

### なぜ開ループ同定が可能か

duty方式は、閉ループ制御の外側にあるモータ指令そのもの（4モータ duty）を
直接観測して逆算するため、$K_p$ の値にもフィードバック則の仮定にも依存
しない。間接方式・旧kp方式は、閉ループデータでも $K_p$ が既知なら
プラントへの入力・応答の関係を復元できるという同じ原理に基づくが、
間接方式は target を外部信号として直接扱う分、実飛行データに対して
遥かに頑健である。

> **重要（励振不足の落とし穴、duty/旧kp方式）:** duty方式・旧来の "kp"
> 方式では、スティックをほとんど動かさずに一定方向へ持ち続けた区間が
> あると、閉ループの P 制御則 $u = K_p(\text{target} - y)$ が
> $u \approx \text{定数} - K_p y$ に潰れ、$u$ と $y$ が「プラントの
> 動特性やハードウェアの符号とは無関係に」強く負相関して見える
> （閉ループ同定の典型的な落とし穴）。`sf sysid fit` はこの状態を検出
> すると該当区間を除外し警告する。間接方式はこの罠を構造的に回避する
> （target を外部信号として直接フィットするため）が、良い $\tau_m$ を
> 得るにはやはり大きめ・高頻度な励振が要る。いずれにせよステップ2の
> 励振の指示に従うこと。

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

# 400Hz duty 列の無い旧ログの場合、--kp を渡すと間接閉ループ方式が
# 自動選択される（--input indirect で明示指定も可能）
sf sysid fit flight.csv --kp 0.5 --plot

# 比較・デバッグ用に旧来の直接 "kp" 方式を明示的に使いたい場合のみ
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

For older logs without the 400Hz duty columns, passing $K_p$ via `--kp`
selects the **indirect closed-loop method** (`--input indirect`, which
`auto` falls back to automatically). Instead of fitting
$u=K_p(\text{target}-\text{gyro})$ directly as an external input, it fits
the closed-loop target->gyro transfer function itself and backs out $K$,
$\tau_m$ algebraically from the known $K_p$:

```
Indirect method (--input indirect, --kp required):
  target(t) is a genuinely external signal (the pilot's stick), not
  algebraically entangled with gyro(t) -- so simulate the closed-loop
  target -> gyro response directly and fit K, tau_m against the measured
  gyro (contrast with the older "kp" method, which uses
  u=Kp*(target-gyro) as a direct external input)
```

A human pilot cannot safely sustain the persistent high-frequency (~8Hz)
excitation the direct "kp" method needs, so fitting
$u=K_p(\text{target}-\text{gyro})$ directly tends to fail on real flight
data (observed R² < 0, K off by 1-3 orders of magnitude in practice). The
indirect method avoids this and recovers K within a few percent to a few
tens of percent of theory on real flight data (though $\tau_m$ still tends
to stay uncertain from human-piloted data, which lacks enough
high-frequency content). The older "kp" method remains available for
comparison/debugging only, via the explicit `--input kp --kp 0.5`.
`sf sysid fit` auto-detects which method to use from the CSV and options
(`--input auto`, the default: duty-first -> indirect if `--kp` is given ->
otherwise the older kp method).

### Why Open-Loop Identification Works

The duty method directly observes the motor command itself (4 motor
duties), outside the closed loop, so it needs neither $K_p$ nor any
assumption about the feedback law. The indirect and older kp methods rest
on the same principle -- that closed-loop data still lets you recover the
plant input/response relationship once $K_p$ is known -- but the indirect
method is far more robust on real flight data because it treats target as
a genuinely external signal instead of folding it into a directly-fitted
$u(t)$.

> **Important (the insufficient-excitation pitfall, duty/older-kp
> methods):** with the duty method or the older "kp" method, a stretch
> where the stick barely moves and is held in one direction collapses the
> closed-loop P-control law $u = K_p(\text{target} - y)$ into
> $u \approx \text{const} - K_p y$, making $u$ and $y$ look strongly and
> misleadingly *negatively* correlated -- regardless of the true plant
> dynamics or hardware sign convention (a classic closed-loop
> identifiability pitfall). `sf sysid fit` detects and drops such segments
> with a warning. The indirect method structurally sidesteps this trap (it
> fits target directly as an external signal), but still needs large,
> frequent stick motion for a good $\tau_m$. Either way, follow Step 2's
> excitation guidance.

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

# For older logs without the 400Hz duty columns, passing --kp auto-selects
# the indirect closed-loop method (or pass --input indirect explicitly)
sf sysid fit flight.csv --kp 0.5 --plot

# Only to explicitly use the older direct "kp" method for comparison/debugging
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
