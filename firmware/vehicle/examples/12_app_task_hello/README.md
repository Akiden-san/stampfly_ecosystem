# 12_app_task_hello — Topic を読む最小の追加タスク

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded

## 1. 目的

`sf::app::start()`（`firmware/vehicle/components/sf_app_hooks/include/app_hooks.hpp`）
は、全標準タスク起動後に一度だけ呼ばれ、アプリ独自の追加タスクをここで起動できる。
本テンプレートはその最小形: `sf::api::estimate_latest()` と `sf::api::is_armed()`
を 1 秒ごとに読み、姿勢（度）と ARM 状態をログに出すだけの 1 タスクを起動する。

コントローラ・推定器はどちらも vehicle 標準（`stock::controller()` /
`stock::estimator()`）のまま — 変えているのは「追加タスクを1つ持つ」ことだけ。

## 2. 書き込み系 API はまだ無い

`sf::api::` は現状 **読み取り専用**（`sf_api.hpp` 参照）。パラメータの書き込みや
ガイダンス目標の設定など、Topic を「書く」API は将来の拡張（Phase 4、
`docs/plans/sf-app-sils-plan.md` §4 参照）で、本テンプレートの時点ではまだ実装
されていない。本タスクが行うのは「読んでログに出す」ことだけである。

## 3. 使い方

```bash
sf app new my_hello --from 12_app_task_hello
```

```bash
sf app sils my_hello
```

`sf app sils my_hello` のログ（`console.out` またはターミナル出力）に、
`AppHello` タグで 1 秒ごとに以下のような行が出れば動作確認完了:

```
I (xxxxx) AppHello: roll=0.1 pitch=-0.3 yaw=12.4 deg, armed=0
```

実機で確認する場合:

```bash
sf app build my_hello
```

```bash
sf app flash my_hello -m
```

## 4. どこを書き換えるか

タスク本体は `app.cpp` の `HelloTask()`。周期を変えたい場合は
`kHelloIntervalTicks`、優先度・スタックサイズ・コアを変えたい場合は
`kHelloTaskPriority` / `kHelloTaskStackBytes` / `kHelloTaskCore` を編集する。
別のトピック（`sf::api::imu_latest()`、`sf::api::control_latest()` 等、
`sf_api.hpp` 参照）を読むように差し替えることもできる。

---

<a id="english"></a>

## 1. Purpose

`sf::app::start()`
(`firmware/vehicle/components/sf_app_hooks/include/app_hooks.hpp`) is called
once, after every standard task is running, and is where an application
starts its own additional tasks. This template is the minimal shape: it
starts one task that reads `sf::api::estimate_latest()` and
`sf::api::is_armed()` once a second and logs attitude (degrees) and armed
state.

Both the controller and the estimator stay the vehicle's standard ones
(`stock::controller()` / `stock::estimator()`) — the only thing this
template changes is having one additional task.

## 2. There is no write-side API yet

`sf::api::` is currently **read-only** (see `sf_api.hpp`). A "write" Topic
API — writing parameters, setting a guidance target, etc. — is a future
extension (Phase 4, see `docs/plans/sf-app-sils-plan.md` §4) and is not
implemented as of this template. All this task does is read and log.

## 3. Usage

```bash
sf app new my_hello --from 12_app_task_hello
```

```bash
sf app sils my_hello
```

Confirm it works by checking that the `sf app sils my_hello` log
(`console.out`, or the terminal output) prints a line tagged `AppHello` once
a second, like:

```
I (xxxxx) AppHello: roll=0.1 pitch=-0.3 yaw=12.4 deg, armed=0
```

To check it on real hardware:

```bash
sf app build my_hello
```

```bash
sf app flash my_hello -m
```

## 4. Where to change things

The task body is `HelloTask()` in `app.cpp`. To change the period, edit
`kHelloIntervalTicks`; to change priority, stack size, or core, edit
`kHelloTaskPriority` / `kHelloTaskStackBytes` / `kHelloTaskCore`. You can
also swap in a different topic (`sf::api::imu_latest()`,
`sf::api::control_latest()`, etc. — see `sf_api.hpp`).
