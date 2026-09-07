# firmware/apps — 自分のプロジェクト

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. これは何のディレクトリか

`sf app new` で作成した、あなた自身のドローンファームウェアプロジェクトを置く場所。
`firmware/vehicle/examples/` の例題を複製したもので、`firmware/vehicle` 本体
（`sf lesson` のワークショップ実習が使う場所）とは別に、自分のコードとして
自由に書き換え・コミットできる。

`sf lesson switch` → `sf lesson edit` → `sf lesson build` → `sf lesson flash`
というワークショップ実習の流れと同じ形の

```
sf app new <name>  →  sf app edit <name>  →  sf app build <name>  →  sf app flash <name>
```

を、研究・自作制御則の入口として提供する。

## 2. `sf app new` からの流れ

```bash
source setup_env.sh                 # 開発環境をアクティブ化

sf app new my_ctrl                  # firmware/apps/my_ctrl を作成（既定: 10_custom_controller を複製）
sf app new my_hello --from 09_topic_api_hello   # 別の例題から作成する場合

sf app edit my_ctrl                 # 学習者が書くファイルをエディタで開く
                                     # （main/learner_controller.cpp があればそれ、無ければ main/main.cpp）
sf app build my_ctrl                # ビルド
sf app flash my_ctrl -m             # 書き込み、書き込み後にモニタを開く

sf app list                         # 自分のプロジェクト一覧と --from に指定できる例題一覧を表示
```

`--from` に指定できる例題は `firmware/vehicle/examples/` 配下のディレクトリ名
（例: `01_blink_led`、`09_topic_api_hello`、`10_custom_controller`）。
`--from` を省略すると `10_custom_controller`（`IController` 差替え演習）が使われる。
各例題の README（`firmware/vehicle/examples/<例題>/README.md`）に、その例題が
何を学べるか・どこまでの範囲か（実機を飛ばすか否か等）が書かれている。

## 3. ディレクトリ構成

```
firmware/apps/
├── README.md          ← このファイル（sf app new では複製されない）
├── my_ctrl/            ← sf app new my_ctrl で作成したプロジェクト
│   ├── CMakeLists.txt  ← EXTRA_COMPONENT_DIRS を vehicle/components に向け直し済み
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   ├── learner_controller.hpp / .cpp   ← 例題により無い場合もある
│   │   └── main.cpp
│   └── README.md       ← 複製元の例題の README（そのままコピーされる）
└── my_hello/           ← 別のプロジェクト
    └── ...
```

複製された `CMakeLists.txt` は `firmware/vehicle/components` の部品
（`sf_api`、`sf_controller`、`sf_estimator` 等）を `EXTRA_COMPONENT_DIRS` 経由で
そのまま使う。`firmware/vehicle` 本体をビルドし直す必要はない。

`build/`・`sdkconfig`・`managed_components/`・`dependencies.lock` は
ビルド時に生成される一時的な状態のため `sf app new` ではコピーされず、
`.gitignore` の既存パターンにより追跡対象外になる。それ以外
（ソースコード・`CMakeLists.txt`・`README.md` 等）は追跡対象のままなので、
自分のプロジェクトを好きなタイミングでコミットしてよい。

## 4. `sf build` / `sf flash` を直接使う場合

`sf app build` / `sf app flash` は、それぞれ次のコマンドに処理を委譲しているだけ:

```bash
sf build apps/<name>
sf flash apps/<name> [-m] [-p PORT]
```

なので、`sf build`/`sf flash` に直接 `apps/<name>` を対象として指定しても
全く同じ結果になる。

## 5. 研究・自作制御則を書くなら

- **既存の推定・制御則の一部だけ差し替えたい** → `10_custom_controller` を複製
  （既定）。`main/learner_controller.{hpp,cpp}` が薄いラッパーの雛形になっており、
  `IController::compute()` の1箇所を書き換えるだけで始められる。
- **状態推定値を読むだけでよい** → `09_topic_api_hello` を複製。`sf::api::*`
  （Topic API）で `estimate_state` 等を読み出す最小コード。
- **センサ・アクチュエータ単体から始めたい** → `01_blink_led`〜`08_battery_monitor`
  を複製。

いずれの例題も **StampFly 実機本体（`firmware/vehicle`）を飛ばすものではない**
（ベンチ実行、または実機センサを読むだけの構成）。実際の飛行制御パイプラインに
組み込むには `firmware/vehicle` 本体の再ビルドが必要 — 手順は
`firmware/vehicle/examples/10_custom_controller/README.md` の
「実機で飛ばすレシピ」を参照。

---

<a id="english"></a>

## 1. What This Directory Is

This is where your own drone-firmware projects created with `sf app new`
live. Each one is a copy of an example under `firmware/vehicle/examples/`,
separate from the `firmware/vehicle` firmware itself (which `sf lesson`'s
workshop exercises use), so you can freely edit and commit it as your own
code.

It provides the same flow as the workshop's
`sf lesson switch` -> `sf lesson edit` -> `sf lesson build` -> `sf lesson flash`,
as an entry point for research and custom control laws:

```
sf app new <name>  ->  sf app edit <name>  ->  sf app build <name>  ->  sf app flash <name>
```

## 2. The Flow From `sf app new`

```bash
source setup_env.sh                 # activate the dev environment

sf app new my_ctrl                  # creates firmware/apps/my_ctrl (default: clones 10_custom_controller)
sf app new my_hello --from 09_topic_api_hello   # or clone a different example

sf app edit my_ctrl                 # opens the file you're expected to write in your editor
                                     # (main/learner_controller.cpp if present, else main/main.cpp)
sf app build my_ctrl                # build
sf app flash my_ctrl -m             # flash, then open a monitor

sf app list                         # list your projects, and the examples available for --from
```

`--from` accepts any directory name under `firmware/vehicle/examples/`
(e.g. `01_blink_led`, `09_topic_api_hello`, `10_custom_controller`).
Omitting `--from` uses `10_custom_controller` (the `IController`
swap-in exercise). Each example's README
(`firmware/vehicle/examples/<example>/README.md`) explains what it teaches
and how far it goes (e.g. whether it flies real hardware).

## 3. Directory Layout

```
firmware/apps/
├── README.md          <- this file (not copied by sf app new)
├── my_ctrl/            <- project created by sf app new my_ctrl
│   ├── CMakeLists.txt  <- EXTRA_COMPONENT_DIRS repointed at vehicle/components
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   ├── learner_controller.hpp / .cpp   <- not present for every example
│   │   └── main.cpp
│   └── README.md       <- the source example's README, copied as-is
└── my_hello/           <- another project
    └── ...
```

The copied `CMakeLists.txt` uses `firmware/vehicle/components`'s parts
(`sf_api`, `sf_controller`, `sf_estimator`, etc.) directly via
`EXTRA_COMPONENT_DIRS`. No need to rebuild `firmware/vehicle` itself.

`build/`, `sdkconfig`, `managed_components/`, and `dependencies.lock` are
transient build state, so `sf app new` does not copy them, and the
repository's existing `.gitignore` patterns already exclude them. Everything
else (source code, `CMakeLists.txt`, `README.md`, etc.) stays tracked, so
you can commit your own project whenever you like.

## 4. Using `sf build` / `sf flash` Directly

`sf app build` / `sf app flash` simply delegate to:

```bash
sf build apps/<name>
sf flash apps/<name> [-m] [-p PORT]
```

so targeting `apps/<name>` directly with `sf build`/`sf flash` gives the
exact same result.

## 5. For Research and Custom Control Laws

- **Swap in one piece of an existing estimator/controller** -> clone
  `10_custom_controller` (the default). `main/learner_controller.{hpp,cpp}`
  is a thin-wrapper template — change the one line inside
  `IController::compute()` to get started.
- **Only need to read a state estimate** -> clone `09_topic_api_hello`, the
  smallest code that reads `estimate_state` etc. via `sf::api::*` (the Topic
  API).
- **Want to start from a single sensor/actuator** -> clone `01_blink_led`
  through `08_battery_monitor`.

None of these examples fly the real StampFly firmware
(`firmware/vehicle`) itself — they either run as a standalone bench or only
read a real sensor. Wiring one into the actual flight-control pipeline
requires rebuilding `firmware/vehicle` itself; see "Recipe to actually fly
this" in
`firmware/vehicle/examples/10_custom_controller/README.md` for the steps.
