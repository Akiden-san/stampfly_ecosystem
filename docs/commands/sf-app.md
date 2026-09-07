# sf app

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

自分のドローンファームウェアプロジェクトを作成・編集・ビルド・書き込みする。
`firmware/vehicle/examples/<N>` の例題を `firmware/apps/<name>` に複製し、
`sf lesson`（ワークショップ実習）と同じ new → edit → build → flash の流れを
研究・自作制御則の入口として提供する。詳細は `firmware/apps/README.md` も参照。

## 2. 構文

```bash
sf app <subcommand> [args]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `new <name> [--from <example>]` | 例題を複製して新規プロジェクトを作成 |
| `edit <name> [--editor X] [--reuse-window]` | 学習者が書くファイルをエディタで開く |
| `build <name> [-c] [-v]` | プロジェクトをビルド（`sf build apps/<name>` と同義） |
| `flash <name> [-p PORT] [-b BAUD] [-m]` | プロジェクトを書き込む（`sf flash apps/<name>` と同義） |
| `list` | 自分のプロジェクト一覧と `--from` に指定できる例題一覧を表示 |

### `new` の引数

| 引数 | 説明 | デフォルト |
|------|------|-----------|
| `name` | プロジェクト名（C識別子として有効な文字列） | (必須) |
| `--from <example>` | 複製元の例題（`firmware/vehicle/examples/` 配下のディレクトリ名） | `10_custom_controller` |

`name` はC識別子であること（英数字とアンダースコアのみ、数字始まり不可）。
既に `firmware/apps/<name>` が存在する場合、および予約名
（`vehicle`, `vehicle_old`, `controller`, `workshop`, `common`, `apps`）は拒否される。
`--from` に存在しない例題を指定するとエラーになり、利用可能な例題一覧が表示される。

## 3. 使用例

```bash
# 既定の例題（10_custom_controller = IController 差替え演習）から作成
sf app new my_ctrl

# 別の例題（Topic API 読取専用）から作成
sf app new my_hello --from 09_topic_api_hello

# 学習者が書くファイルをエディタで開く
sf app edit my_ctrl

# ビルド・書き込み
sf app build my_ctrl
sf app flash my_ctrl -m

# 自分のプロジェクト一覧と、--from に指定できる例題一覧を表示
sf app list
```

## 4. 出力

`sf app new` 成功時:
```
[INFO] Creating new app project: my_ctrl
  From: /path/to/firmware/vehicle/examples/10_custom_controller
  To:   /path/to/firmware/apps/my_ctrl
[OK] Project created: /path/to/firmware/apps/my_ctrl

Next steps:
  sf app edit my_ctrl
  sf app build my_ctrl
  sf app flash my_ctrl -m
```

## 5. 補足

- `sf app edit` は `main/learner_controller.cpp` があればそれを、無ければ
  `main/main.cpp` を開く（エディタ検出は VSCode → vi → vim → Notepad の順、
  `sf lesson edit` と共通のロジック）。
- `sf app build`/`sf app flash` は内部で `sf build apps/<name>`/
  `sf flash apps/<name>` に処理を委譲しているだけなので、`sf build`/`sf flash`
  を直接使っても同じ結果になる。

---

<a id="english"></a>

## 1. Overview

Create, edit, build, and flash your own StampFly firmware project. A new
project is cloned from a `firmware/vehicle/examples/<N>` example into
`firmware/apps/<name>`, providing the same new -> edit -> build -> flash
flow as `sf lesson` (the workshop exercises) as an entry point for research
and custom control laws. See also `firmware/apps/README.md`.

## 2. Syntax

```bash
sf app <subcommand> [args]
```

### Subcommands

| Subcommand | Description |
|-----------|------|
| `new <name> [--from <example>]` | Create a new project cloned from an example |
| `edit <name> [--editor X] [--reuse-window]` | Open the learner-facing source file in an editor |
| `build <name> [-c] [-v]` | Build the project (equivalent to `sf build apps/<name>`) |
| `flash <name> [-p PORT] [-b BAUD] [-m]` | Flash the project (equivalent to `sf flash apps/<name>`) |
| `list` | List your projects and the examples available for `--from` |

### `new` Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `name` | Project name (must be a valid C identifier) | (required) |
| `--from <example>` | Example to clone (a directory name under `firmware/vehicle/examples/`) | `10_custom_controller` |

`name` must be a valid C identifier (letters, digits, underscore; cannot
start with a digit). Rejected if `firmware/apps/<name>` already exists, or
if `name` is a reserved word (`vehicle`, `vehicle_old`, `controller`,
`workshop`, `common`, `apps`). An unknown `--from` example fails with an
error listing the available examples.

## 3. Examples

```bash
# Create from the default example (10_custom_controller = IController swap-in exercise)
sf app new my_ctrl

# Create from a different example (Topic API, read-only)
sf app new my_hello --from 09_topic_api_hello

# Open the learner-facing file in your editor
sf app edit my_ctrl

# Build and flash
sf app build my_ctrl
sf app flash my_ctrl -m

# List your projects and the examples available for --from
sf app list
```

## 4. Output

On `sf app new` success:
```
[INFO] Creating new app project: my_ctrl
  From: /path/to/firmware/vehicle/examples/10_custom_controller
  To:   /path/to/firmware/apps/my_ctrl
[OK] Project created: /path/to/firmware/apps/my_ctrl

Next steps:
  sf app edit my_ctrl
  sf app build my_ctrl
  sf app flash my_ctrl -m
```

## 5. Notes

- `sf app edit` opens `main/learner_controller.cpp` if present, else
  `main/main.cpp` (editor detection order: VSCode -> vi -> vim -> Notepad,
  the same logic shared with `sf lesson edit`).
- `sf app build`/`sf app flash` simply delegate to `sf build apps/<name>`/
  `sf flash apps/<name>` internally, so using `sf build`/`sf flash` directly
  gives the same result.
