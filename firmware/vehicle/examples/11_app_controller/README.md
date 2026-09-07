# 11_app_controller — 独自 IController を vehicle 本体に組み込む

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded

## 1. 目的

`IController`（`components/sf_controller/include/controller.hpp`）を実装した
`AppController` は、既存の `PidController`（カスケードPID制御器）へ全メソッドを
委譲しながら、`compute()` の出力だけを 1 箇所（`adjust()`）で調整できる。
`examples/10_custom_controller` の `LearnerController` と考え方は同じだが、
こちらは合成入力に対する単独実行ではなく、**vehicle 本体にそのまま組み込んで**
実機・SILS の両方で動かす。

## 2. 組み込み型であること

このテンプレートは `type: embedded`（`app.yaml` 参照）— 単独ビルド可能な例題
09/10 とは異なり、`CMakeLists.txt` も `main/` も持たない。ここに置いた
`*.cpp`/`*.hpp` は `sf app` コマンドが `SF_APP_DIR` として vehicle 本体の
main コンポーネントへ直接コンパイルする（`firmware/vehicle/components/sf_app_hooks/`
参照）。つまり実行できるのは「vehicle 全体をビルドした結果」であり、この
ディレクトリ単体を `idf.py build` することはできない。

## 3. 使い方

まず自分のプロジェクトを作る（このテンプレートを複製する）:

```bash
sf app new my_ctrl
```

SILS（実機を使わない PC 上の検証）で動作を確認する:

```bash
sf app sils my_ctrl
```

実機向けにビルドする:

```bash
sf app build my_ctrl
```

実機に書き込む:

```bash
sf app flash my_ctrl
```

## 4. adjust() の書き換え例

書き換える箇所は `app_controller.cpp` の `adjust()` 1 関数だけ。既定は恒等
（`kPitchTorqueScale = 1.0f`）で、`PidController` の出力をそのまま通す。

```cpp
sf::ControlOutput AppController::adjust(
    sf::ControlOutput output,
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint)
{
    constexpr float kPitchTorqueScale = 1.0f;
    output.torque[1] *= kPitchTorqueScale;
    return output;
}
```

`kPitchTorqueScale` を変える、あるいはこの行を `state`/`setpoint` から計算した
独自の制御則（例: 自作のピッチレート P 項）に置き換えて再ビルドすると、挙動が
変わる。`ControlOutput::torque` は `float[3]`（R, P, Y の順、`data_types.hpp`
参照）。

## 5. 注意

| 項目 | 内容 |
|------|------|
| 呼び出し周期 | `compute()` は 400Hz で呼ばれる。重い処理を書くと制御周期を圧迫する |
| 転送メソッド | `reset()` と `onModeChange()` の転送を消さないこと。ARM/DISARM やモード切替の通知が届かなくなる |
| 鉛直フェーズ | 離陸・着陸・高度保持の鉛直チャネルは `PidController` に委譲したまま — `architecture.md` の不変条件 INV-1（全フェーズが単一の姿勢+レートパイプラインを共有する）を壊さないこと |

---

<a id="english"></a>

## 1. Purpose

`AppController` implements `IController`
(`components/sf_controller/include/controller.hpp`) and forwards every
method to the existing `PidController` (cascade PID controller), except that
the output of `compute()` passes through one insertion point, `adjust()`.
Same idea as `LearnerController` in `examples/10_custom_controller`, but this
one compiles straight into the vehicle body and runs on both real hardware
and SILS instead of standalone against synthetic input.

## 2. It is an embedded template

This template is `type: embedded` (see `app.yaml`) — unlike the standalone
examples 09/10, it has no `CMakeLists.txt` and no `main/`. The `*.cpp`/`*.hpp`
files here are compiled directly into the vehicle body's main component by
the `sf app` command via `SF_APP_DIR` (see
`firmware/vehicle/components/sf_app_hooks/`). What actually runs is the
result of building the whole vehicle — you cannot `idf.py build` this
directory on its own.

## 3. Usage

Create your own project (clone this template):

```bash
sf app new my_ctrl
```

Verify it in SILS (PC-side simulation, no hardware needed):

```bash
sf app sils my_ctrl
```

Build it for real hardware:

```bash
sf app build my_ctrl
```

Flash it to hardware:

```bash
sf app flash my_ctrl
```

## 4. Where to change adjust()

The only function you need to edit is `adjust()` in `app_controller.cpp`. The
default is the identity (`kPitchTorqueScale = 1.0f`) — `PidController`'s
output passes through unchanged.

```cpp
sf::ControlOutput AppController::adjust(
    sf::ControlOutput output,
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint)
{
    constexpr float kPitchTorqueScale = 1.0f;
    output.torque[1] *= kPitchTorqueScale;
    return output;
}
```

Change `kPitchTorqueScale`, or replace this line with your own control law
computed from `state`/`setpoint` (e.g. your own pitch-rate P term), then
rebuild to see the behavior change. `ControlOutput::torque` is a `float[3]`
(R, P, Y order — see `data_types.hpp`).

## 5. Notes

| Item | Detail |
|------|--------|
| Call rate | `compute()` runs at 400 Hz. Heavy work here squeezes the control loop's time budget |
| Forwarded methods | Do not remove the forwarding of `reset()` and `onModeChange()` — otherwise ARM/DISARM and mode-change notifications never reach the controller |
| Vertical phase | Takeoff, landing, and altitude-hold's vertical channel remain delegated to `PidController` — do not break `architecture.md`'s invariant INV-1 (every phase shares the ONE attitude+rate pipeline) |
