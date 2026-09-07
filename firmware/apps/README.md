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
sf app new <name>  →  sf app edit <name>  →  sf app sils <name>  →  sf app build <name>  →  sf app flash <name>
```

を、研究・自作制御則の入口として提供する。

## 2. `sf app new` からの流れ

```bash
source setup_env.sh                 # 開発環境をアクティブ化
```

```bash
sf app new my_ctrl                  # firmware/apps/my_ctrl を作成（既定: 11_app_controller を複製、組み込み型）
```

```bash
sf app new my_hello --from 12_app_task_hello   # 別の例題（Topic を読む追加タスク）から作成する場合
```

```bash
sf app edit my_ctrl                 # 学習者が書くファイルをエディタで開く
```

```bash
sf app sils my_ctrl                 # SILS（実機を使わない PC 上の検証）で確認
```

```bash
sf app build my_ctrl                # 実機向けにビルド
```

```bash
sf app flash my_ctrl -m             # 書き込み、書き込み後にモニタを開く
```

```bash
sf app list                         # 自分のプロジェクト一覧と --from に指定できる例題一覧を表示
```

`--from` に指定できる例題は `firmware/vehicle/examples/` 配下のディレクトリ名
（例: `11_app_controller`、`12_app_task_hello`、`10_custom_controller`）。
`--from` を省略すると `11_app_controller`（組み込み型 `IController` テンプレート）が使われる。
各例題の README（`firmware/vehicle/examples/<例題>/README.md`）に、その例題が
何を学べるか・どこまでの範囲か（実機を飛ばすか否か等）が書かれている。

## 3. 2種類のプロジェクト: 組み込み型とベンチ型

`firmware/apps/<name>/app.yaml` の `type` によって、プロジェクトは2種類に分かれる。

| 種別 | 実体 | SILS | 実機 |
|------|------|------|------|
| `embedded`（組み込み型） | vehicle 本体の main コンポーネントに `SF_APP_DIR` 経由で直接コンパイルされる | 可（`sf app sils`） | 可 |
| `bench`（ベンチ型） | 独立した ESP-IDF プロジェクト | 不可（`sf app sils` は終了コード2で案内） | 可（独立プロジェクトとして） |

`sf app new` の既定の複製元 `11_app_controller` は組み込み型。`09_topic_api_hello`・
`10_custom_controller` はベンチ型として残っている（vehicle 全体をビルドせずに
API を学べる）。

## 4. ディレクトリ構成

組み込み型（既定）:

```
firmware/apps/
├── README.md               ← このファイル（sf app new では複製されない）
├── my_ctrl/                 ← sf app new my_ctrl（既定 --from 11_app_controller）で作成
│   ├── app.yaml              ← type: embedded / from / sils / description
│   ├── app.cpp                ← sf::app::controller()/estimator()/start() の実装
│   ├── app_controller.hpp/.cpp ← 独自 IController（AppController）
│   ├── README.md              ← 複製元の例題の README（そのままコピーされる）
│   └── build/                 ← sf app build/sils が生成するビルドツリー（.gitignore 済み）
└── my_hello/                 ← 別のプロジェクト（--from 12_app_task_hello 等）
    └── ...
```

ベンチ型（`--from 09_topic_api_hello`・`--from 10_custom_controller` 等）:

```
firmware/apps/
└── my_bench/
    ├── CMakeLists.txt      ← EXTRA_COMPONENT_DIRS を vehicle/components に向け直し済み
    ├── main/
    │   ├── CMakeLists.txt
    │   ├── learner_controller.hpp / .cpp   ← 例題により無い場合もある
    │   └── main.cpp
    └── README.md
```

複製された `CMakeLists.txt`（ベンチ型のみ）は `firmware/vehicle/components` の部品
（`sf_api`、`sf_controller`、`sf_estimator` 等）を `EXTRA_COMPONENT_DIRS` 経由で
そのまま使う。組み込み型は自身の `CMakeLists.txt` を持たず、vehicle 本体の
`main/CMakeLists.txt` が `SF_APP_DIR` 経由でこのディレクトリの `*.cpp` を取り込む。

`build/`・`sdkconfig`・`managed_components/`・`dependencies.lock` は
ビルド時に生成される一時的な状態のため `sf app new` ではコピーされず、
`.gitignore` の `**/build/` 等の既存パターンにより追跡対象外になる。それ以外
（ソースコード・`app.yaml`／`CMakeLists.txt`・`README.md` 等）は追跡対象のままなので、
自分のプロジェクトを好きなタイミングでコミットしてよい。

## 5. `sf build` / `sf flash` を直接使う場合

ベンチ型の `sf app build` / `sf app flash` は、それぞれ次のコマンドに処理を委譲しているだけ:

```bash
sf build apps/<name>
sf flash apps/<name> [-m] [-p PORT]
```

なので、`sf build`/`sf flash` に直接 `apps/<name>` を対象として指定しても
全く同じ結果になる（組み込み型は `idf.py -B firmware/apps/<name>/build -D
SF_APP_DIR=<abs path>` を `firmware/vehicle` で実行する専用の経路を使う）。

`sf app sils <name>` は `sf sils scenario <scenario> --target apps/<name>` の
薄いラッパーで、`sf sils scenario`/`sf sils build` に直接 `--target apps/<name>`
を指定しても同じ経路が使われる。

## 6. 研究・自作制御則を書くなら

- **既存の推定・制御則の一部だけ差し替え、SILS と実機の両方で確認したい**（既定） →
  `11_app_controller` を複製。`app_controller.cpp` の `adjust()` 1箇所を書き換えるだけで
  始められ、`sf app sils` → `sf app build` → `sf app flash` がそのまま動く。
- **Topic を読んで記録・判定するだけの追加タスクを書きたい** → `12_app_task_hello`
  を複製。`sf::app::start()` から起動する最小の FreeRTOS タスクの雛形。
- **vehicle 全体をビルドせずに `IController`/Topic API を手早く試したい** →
  `10_custom_controller`（`IController` 差替えの単独ベンチ）または
  `09_topic_api_hello`（Topic 読み取りの単独ベンチ）を複製。どちらもベンチ型
  のため `sf app sils` は使えない。
- **センサ・アクチュエータ単体から始めたい** → `01_blink_led`〜`08_battery_monitor`
  を複製。

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
sf app new <name>  ->  sf app edit <name>  ->  sf app sils <name>  ->  sf app build <name>  ->  sf app flash <name>
```

## 2. The Flow From `sf app new`

```bash
source setup_env.sh                 # activate the dev environment
```

```bash
sf app new my_ctrl                  # creates firmware/apps/my_ctrl (default: clones 11_app_controller, embedded-type)
```

```bash
sf app new my_hello --from 12_app_task_hello   # or clone a different example (a task that reads topics)
```

```bash
sf app edit my_ctrl                 # opens the file you're expected to write in your editor
```

```bash
sf app sils my_ctrl                 # verify in SILS (PC-side simulation, no hardware needed)
```

```bash
sf app build my_ctrl                # build for real hardware
```

```bash
sf app flash my_ctrl -m             # flash, then open a monitor
```

```bash
sf app list                         # list your projects, and the examples available for --from
```

`--from` accepts any directory name under `firmware/vehicle/examples/`
(e.g. `11_app_controller`, `12_app_task_hello`, `10_custom_controller`).
Omitting `--from` uses `11_app_controller` (the embedded-type `IController`
template). Each example's README
(`firmware/vehicle/examples/<example>/README.md`) explains what it teaches
and how far it goes (e.g. whether it flies real hardware).

## 3. Two Kinds of Project: Embedded-Type and Bench-Type

`firmware/apps/<name>/app.yaml`'s `type` splits a project into two kinds.

| Type | What It Is | SILS | Hardware |
|------|-----------|------|----------|
| `embedded` | Compiled directly into the vehicle firmware's main component via `SF_APP_DIR` | Yes (`sf app sils`) | Yes |
| `bench` | A standalone ESP-IDF project | No (`sf app sils` explains why and exits with code 2) | Yes (as a standalone project) |

`sf app new`'s default source, `11_app_controller`, is embedded-type.
`09_topic_api_hello` and `10_custom_controller` remain bench-type (learn the
API without building the whole vehicle firmware).

## 4. Directory Layout

Embedded-type (default):

```
firmware/apps/
├── README.md                <- this file (not copied by sf app new)
├── my_ctrl/                  <- created by sf app new my_ctrl (default --from 11_app_controller)
│   ├── app.yaml                <- type: embedded / from / sils / description
│   ├── app.cpp                  <- sf::app::controller()/estimator()/start() implementation
│   ├── app_controller.hpp/.cpp  <- your own IController (AppController)
│   ├── README.md                <- the source example's README, copied as-is
│   └── build/                   <- build tree created by sf app build/sils (gitignored)
└── my_hello/                  <- another project (e.g. --from 12_app_task_hello)
    └── ...
```

Bench-type (e.g. `--from 09_topic_api_hello`, `--from 10_custom_controller`):

```
firmware/apps/
└── my_bench/
    ├── CMakeLists.txt      <- EXTRA_COMPONENT_DIRS repointed at vehicle/components
    ├── main/
    │   ├── CMakeLists.txt
    │   ├── learner_controller.hpp / .cpp   <- not present for every example
    │   └── main.cpp
    └── README.md
```

The copied `CMakeLists.txt` (bench-type only) uses `firmware/vehicle/components`'s
parts (`sf_api`, `sf_controller`, `sf_estimator`, etc.) directly via
`EXTRA_COMPONENT_DIRS`. An embedded-type project has no `CMakeLists.txt` of
its own; the vehicle firmware's own `main/CMakeLists.txt` pulls in this
directory's `*.cpp` via `SF_APP_DIR` instead.

`build/`, `sdkconfig`, `managed_components/`, and `dependencies.lock` are
transient build state, so `sf app new` does not copy them, and the
repository's existing `.gitignore` patterns (`**/build/`, etc.) already
exclude them. Everything else (source code, `app.yaml`/`CMakeLists.txt`,
`README.md`, etc.) stays tracked, so you can commit your own project
whenever you like.

## 5. Using `sf build` / `sf flash` Directly

A bench-type project's `sf app build` / `sf app flash` simply delegate to:

```bash
sf build apps/<name>
sf flash apps/<name> [-m] [-p PORT]
```

so targeting `apps/<name>` directly with `sf build`/`sf flash` gives the
exact same result (an embedded-type project uses a dedicated path instead:
`idf.py -B firmware/apps/<name>/build -D SF_APP_DIR=<abs path>` run from
`firmware/vehicle`).

`sf app sils <name>` is a thin wrapper around `sf sils scenario <scenario>
--target apps/<name>`; passing `--target apps/<name>` directly to `sf sils
scenario`/`sf sils build` takes the same path.

## 6. For Research and Custom Control Laws

- **Swap in one piece of an existing estimator/controller, and verify it in
  both SILS and on real hardware** (default) -> clone `11_app_controller`.
  Change the one insertion point, `adjust()` in `app_controller.cpp`, to get
  started, and `sf app sils` -> `sf app build` -> `sf app flash` just work.
- **Write an additional task that only reads and logs/judges topics** ->
  clone `12_app_task_hello`, the minimal FreeRTOS task template started from
  `sf::app::start()`.
- **Quickly try `IController`/the Topic API without building the whole
  vehicle firmware** -> clone `10_custom_controller` (an `IController`
  swap-in standalone bench) or `09_topic_api_hello` (a Topic-read standalone
  bench). Both are bench-type, so `sf app sils` is unavailable for them.
- **Want to start from a single sensor/actuator** -> clone `01_blink_led`
  through `08_battery_monitor`.
