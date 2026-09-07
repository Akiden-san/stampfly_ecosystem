# `sf app` プロジェクトの SILS 対応計画

作成: 2026-09-07。発端: README「何ができるのか？」に「飛行プログラムの SILS（Software In the
Loop Simulation: ファームウェアそのものを PC 上で動かす試験）での検証」を掲げたが、`sf app` で
作った自作プロジェクトは現状 SILS で動かせないと判明したため、何が欠けているかを洗い出し、
実装計画を定める。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 現状 | `sf app` プロジェクトは例題（`firmware/vehicle/examples/<N>`）の複製で、独立した ESP-IDF プロジェクト。SILS は `vehicle` / `vehicle_old` / `workshop` の 3 ターゲットを CMake にベタ書きしており、`apps/<name>` を受け付ける口がどの層にも無い |
| 根本の問題 | 既定の複製元 `10_custom_controller` は合成信号で制御則を呼ぶだけの**ベンチ**で、機体のタスク基盤も実センサも使わない。仮に SILS でビルドできても閉ループにならず、飛行検証にはならない |
| 方針 | 「自作飛行プログラム」の受け皿を、すでに実機・SILS 両対応の **workshop 骨格**（vehicle のタスク基盤 + 学習者コード `setup()` / `loop_400Hz()`）に一本化する。`sf app` プロジェクトは「学習者コード一式を持つディレクトリ」となり、実機ビルドと SILS ビルドの両方が同じ骨格にそのコードを差し込む。並列の新機構は作らない |
| 成果 | `sf app new my_ctrl` → `sf app build my_ctrl`（実機）／ `sf app sils my_ctrl`（SILS）が**同一ソース**をコンパイルする。Code Identity（実機と SILS で同じコードが動くこと）を自作プロジェクトにも拡張する |
| 見積り | Phase 0〜3 で 4〜6 日。Phase 4（`sf lesson` の同方式への移行）は任意 |

## 1. 何ができていないか（事実）

調査は 2026-09-07 に実施し、以下の行番号はその時点のもの。

### sf CLI 層

| 項目 | 現状 | 根拠 |
|------|------|------|
| SILS ターゲットの固定 | `SILS_TARGETS = ("vehicle", "vehicle_old", "workshop")`、実行ファイル名は `emu_<name>` | `lib/sfcli/commands/sils.py:122-123` |
| `sf sils scenario --target` | `choices=list(SILS_TARGETS)` のため `apps/<name>` は引数解析の段階で拒否される | `sils.py:374` |
| `sf sils build --target` | 自由文字列だが `cmake --build --target <name>` にそのまま渡り、CMake 側に無いターゲットなので失敗する | `sils.py:325-328, 584, 654` |
| `sf app` の副コマンド | `new` / `edit` / `build` / `flash` / `list` のみ。`build` / `flash` は `sf build apps/<name>` / `sf flash apps/<name>` への委譲で、SILS には触れない | `lib/sfcli/commands/app.py:397-425` |
| `sf build` | `firmware/<target>` を `idf.py build` する実機ビルド専用。SILS（ホスト側 CMake）とは別系統 | `lib/sfcli/commands/build.py:87, 135`、`lib/sfcli/utils/paths.py:206-208` |

### SILS の CMake 層

| 項目 | 現状 | 根拠 |
|------|------|------|
| ターゲット定義 | `emu_vehicle` / `emu_vehicle_old` / `emu_workshop` の 3 つの `add_executable()` がベタ書き。ディレクトリを走査して動的にターゲットを生やす仕組みは無い | `simulator/sils/CMakeLists.txt:314-344, 358-404, 437-511` |
| `emu_workshop` の構成 | vehicle の全ソース（`EMU_FW_SRCS`）から `main/main.cpp`・`tasks/control_task.cpp`・`tasks/tasks.cpp` を正規表現で除き、`firmware/workshop/main/workshop_{main,tasks,control_task}.cpp` と `user_code.cpp` を足す | `CMakeLists.txt:468-475` |
| アプリ固有コードの注入点 | `emu_main.cpp` が `extern "C" void app_main()` を 1 つだけリンク時に解決する。「どの main をリンクするか」を CMake に書くことが唯一のフック | `simulator/sils/emu/emu_main.cpp:55, 387` |
| 学習者コードの初回生成 | `user_code.cpp` が無ければ Lesson 0 の `student.cpp` から `configure_file` で生成するロジックが、実機ビルドと SILS ビルドの 2 か所に重複している | `firmware/workshop/main/CMakeLists.txt:15-24`、`simulator/sils/CMakeLists.txt:450-455` |

### ファームウェア層（例題・apps の設計）

| 項目 | 現状 | 根拠 |
|------|------|------|
| 既定の複製元 | `sf app new` の既定 `--from` は `10_custom_controller` | `app.py`（`DEFAULT_EXAMPLE`） |
| 10_custom_controller の性質 | 独自 `app_main()` を持ち、合成サイン波でピッチ外乱を作って `LearnerController::compute()` を呼ぶだけ。プラントモデル無し・飛行シミュレータではないと README が明記 | `firmware/vehicle/examples/10_custom_controller/README.md:15-28` |
| 実機で飛ばす手順 | 「vehicle 本体を再ビルドする必要がある」とし、`tasks/control_task.cpp:63` の `static sf::PidController controller;` を**手で**書き換える手順を示す。自動化は無い | 同 README §8（159-175 行） |
| 09_topic_api_hello | 同様に独自 `app_main()` と簡易センサフィードを持つ独立プロジェクト | `firmware/vehicle/examples/09_topic_api_hello/main/` |
| apps の位置づけ | 「いずれの例題も StampFly 実機本体を飛ばすものではない」「飛行制御パイプラインに組み込むには `firmware/vehicle` 本体の再ビルドが必要」と明記。SILS への言及は無い | `firmware/apps/README.md` §5、`docs/commands/sf-app.md` |
| workshop 骨格の学習者 API | センサ値（gyro/accel/baro/mag/tof/flow）、送信機入力、モータ出力とミキサ、アーム、推定姿勢を関数で提供。学習者は `setup()` と `loop_400Hz(float dt)` を書く | `firmware/workshop/main/workshop_api.hpp` |
| `sf lesson switch` の実体 | `lessons/lesson_NN/{student,solution}.cpp` を `firmware/workshop/main/user_code.cpp` へ単純コピー。コピーが更新時刻を保つため、SILS 再ビルド前に `touch` が必要と講師ガイドに明記 | `lib/sfcli/commands/lesson.py:48-50, 758-793`、`docs/events/stampfly_workshop/workshop_guide.md:191-203` |

### テスト・CI・文書

| 項目 | 現状 | 根拠 |
|------|------|------|
| SILS シナリオ | workshop 向けは `workshop_acro.scn` / `.expect`（離陸すること・転倒しないことの緩い合格基準）が存在。apps 向けは無い | `simulator/sils/scenarios/workshop_acro.*` |
| CI | `sils-regression.yml` は 3 ターゲットの範囲で回る。apps を回すジョブは無い | `.github/workflows/sils-regression.yml` |
| 文書 | `docs/commands/sf-app.md`・`firmware/apps/README.md` に SILS 連携の記述は無い（意図的に「飛ばさない」設計と明記） | 同上 |

## 2. 設計判断

| 案 | 内容 | 判断 |
|----|------|------|
| A. 例題の `app_main` をそのまま SILS で動かす | `emu_apps_<name>` を追加し例題の main をリンク | **却下**。例題は機体のタスク基盤を使わず、プラントとの閉ループにならない。動いても「飛行プログラムの検証」にならない |
| B. vehicle 本体に新しい「コントローラ差し替えフック」を作り、apps がコントローラ実装を供給 | `control_task.cpp:63` の手作業を CMake 変数で自動化 | **保留**。workshop 骨格の `setup()`/`loop_400Hz()` と並ぶ第 2 の拡張点になり、二重構造を生む（アーキテクチャ不変条件「並列経路を作らない」に抵触）。Phase 4 以降で workshop 骨格と統合できる見通しが立ったときに再検討 |
| C. workshop 骨格を「ユーザーコード実行基盤」として一般化し、学習者コードの所在を CMake 変数で指定できるようにする | lessons も apps も同じ変数で差し込む | **採用**。実機・SILS 両対応と Code Identity が既に実証済み（CI で workshop ターゲットが回っている）。新機構は「変数化」だけ |

採用案の要点:

- 拡張点は 1 つ（`setup()` / `loop_400Hz()`）。`sf lesson`（学習）と `sf app`（研究・自作）は同じ骨格の利用者になる。
- 09 / 10 の例題は「API 学習・ベンチ」として残す。`sf app new --from 10_custom_controller` は引き続き可能だが、`sf app list` で **SILS 不可（ベンチ）** と表示し、既定の複製元は飛べるテンプレートに変える。
- 骨格ディレクトリ名 `firmware/workshop` を中立な名前に改める案は影響が大きいので Phase 4 で別途判断する。まず変数化で機能を成立させる。

## 3. 目標の使い方（受け入れ基準）

```bash
sf app new my_ctrl
```

```bash
sf app build my_ctrl
```

```bash
sf app sils my_ctrl
```

| 基準 | 内容 |
|------|------|
| 同一ソース | `sf app build` と `sf app sils` が `firmware/apps/my_ctrl/user_code.cpp` を同じ骨格にリンクする |
| 合格基準 | 既定テンプレートで `workshop_acro` 相当のシナリオ（離陸する・転倒しない）が PASS する |
| 退行なし | 既存の SILS 回帰（vehicle / vehicle_old / workshop）と実機ビルド（`sf build workshop`、`sf lesson build`）に変化がない |
| 分離 | app ごとにビルド成果物が分かれ、別 app の切替でキャッシュ汚染や `touch` が要らない |
| 3 OS | Windows（CMD）/ macOS / Ubuntu で同じコマンドが通る（パスに空白を含む場合を含む） |

## 4. 実装計画

### Phase 0: 仕様確定（0.5 日）

| 作業 | 内容 |
|------|------|
| プロジェクト構造の確定 | `firmware/apps/<name>/` に `user_code.cpp`（必須）、追加の `*.cpp` / `*.hpp`（任意）、`app.yaml`（複製元・説明・`sils: true/false`）、`README.md` |
| テンプレートの選定 | 既定 `--from` を、離陸できる学習者コード（`lessons/lesson_08_pid/solution.cpp` 相当）を骨格に持つ新例題（仮名 `11_user_code_flight`）にする。まず現行の lesson 8 解答が `workshop_acro` を PASS することを SILS で確認する |
| 変数名の確定 | CMake 変数 `SF_USER_CODE_DIR`（絶対パス）。未指定時は従来どおり `firmware/workshop/main/user_code.cpp`（初回は Lesson 0 から生成） |

### Phase 1: ビルド基盤の変数化（1〜2 日）

| 作業 | 対象 | 内容 |
|------|------|------|
| 実機ビルド | `firmware/workshop/main/CMakeLists.txt` | `SF_USER_CODE_DIR` が与えられたら、そのディレクトリの `user_code.cpp` と追加 `*.cpp` を `SRCS` に、ディレクトリを `INCLUDE_DIRS` に加える。未指定時のみ Lesson 0 からの初回生成を行う |
| SILS ビルド | `simulator/sils/CMakeLists.txt` | `emu_workshop` のソース集合を同じ変数で切り替える。実行ファイル名は `emu_workshop` のまま、**ビルドディレクトリを app ごとに分ける**（`simulator/sils/build/apps/<name>/`）ことで成果物を分離する |
| 重複の解消 | 上記 2 ファイル | Lesson 0 からの初回生成ロジックを 1 か所（`firmware/workshop/cmake/user_code.cmake` 等）にまとめ、両者から `include` する |
| 動作確認 | 手動 | `cmake -DSF_USER_CODE_DIR=<lessons/lesson_08_pid>` で SILS がビルドでき、`workshop_acro` が PASS する |

### Phase 2: sf CLI（1〜2 日）

| 作業 | 対象 | 内容 |
|------|------|------|
| `sf app new` | `app.py` | 既定 `--from` を新テンプレートに変更。複製後に `app.yaml` を書く。`--from 09/10` を選んだときは `sils: false` を記録 |
| `sf app list` | `app.py` | 各 app に「実機: 可 / SILS: 可・不可（ベンチ）」を表示 |
| `sf app build` / `flash` | `app.py`、`build.py` | `sils: true` の app は `sf build workshop` を `-DSF_USER_CODE_DIR=<abs path>` と app 専用ビルドディレクトリ（`idf.py -B`）で実行。`sils: false` の app は従来どおり独立プロジェクトとしてビルド |
| `sf app sils` | 新設（`app.py` → `sils.py` の関数呼び出し） | `sf app sils <name> [--scenario <scn>] [--expect <file>] [--noise ...]`。内部は `run_scenario(target="workshop", user_code_dir=..., build_dir=...)` の薄い包み |
| `sf sils scenario --target apps/<name>` | `sils.py` | `choices` を固定リストから「3 ターゲット + `apps/<存在するディレクトリ>`」を返す関数に変更。`sf sils build --target apps/<name>` も同様。実装は `sf app sils` と共有する |
| ヘルプ・エラー文 | 両ファイル | `sils: false` の app に `sf app sils` を打ったときは「この app はベンチ例題の複製で SILS では飛べません。`sf app new --from 11_user_code_flight` を使ってください」と案内する |

### Phase 3: テスト・CI・文書（1 日）

| 作業 | 対象 | 内容 |
|------|------|------|
| シナリオ | `simulator/sils/scenarios/app_flight.scn` / `.expect` | `workshop_acro` を基に、テンプレート app 用の合格基準を用意する |
| CI | `.github/workflows/sils-regression.yml` | `sf app new ci_app && sf app sils ci_app --scenario app_flight` のジョブを追加。`sf app new --from 10_custom_controller bench && sf app build bench` で従来経路の退行も確認 |
| CLI テスト | `lib/sfcli/tests/` | `new → list → sils` の流れと、`sils: false` の app に対する案内文のテスト |
| 文書 | `docs/commands/sf-app.md`、`firmware/apps/README.md`、`docs/next_step.md` §7・§8、`docs/events/stampfly_workshop/workshop_guide.md` | `sf app sils` の使い方、テンプレートの区分（飛ぶ / ベンチ）、`touch` 手順の廃止。README「何ができるのか？」の SILS 行の表現を最終確認 |

### Phase 4（任意）: `sf lesson` の同方式への移行

| 作業 | 内容 |
|------|------|
| コピー方式の廃止 | `sf lesson switch` を「`SF_USER_CODE_DIR=lessons/lesson_NN` を記録する」方式に変え、`user_code.cpp` へのコピーと `touch` を無くす。`sf lesson build` / `sf sils --target workshop` はその記録を読む |
| 骨格の改名 | `firmware/workshop` → 中立名（例: `firmware/user_code_runtime`）。参照箇所（CMake、sfcli、CI、文書、スライド）が多いため、単独の作業として影響範囲を洗い出してから判断 |

## 5. リスクと未確認事項

| 項目 | 内容 | 対処 |
|------|------|------|
| 骨格の飛行性能 | workshop 骨格は vehicle の `ControlTask` を `WorkshopControlTask` に置き換えており、高度ループは学習者コードが担う。テンプレートが実機で安全に離陸できるかは SILS の `workshop_acro` 相当の確認と、実機での確認が必要 | Phase 0 で lesson 8 解答の SILS 確認、Phase 3 後に実機確認 |
| `idf.py` への変数受け渡し | `-D` と `-B`（ビルドディレクトリ）の組み合わせで sdkconfig の扱いが変わらないか | Phase 1 で `sf build workshop` の既存経路と差分比較 |
| Windows のパス | CMake 変数に空白やバックスラッシュを含む絶対パスを渡す | 引用符付きで渡し、`windows-e2e.yml` に 1 ケース追加 |
| 既存回帰への影響 | 変数未指定時の挙動を厳密に維持する | Phase 1 完了時に SILS 回帰全件を A/B 比較（退行ゼロ） |
| 09 / 10 の位置づけ | 研究者が「制御則だけ差し替えたい」場合、`LearnerController` の形（角度制御の一段だけ）の方が書きやすい可能性 | Phase 2 でテンプレートに「`loop_400Hz` の中で `LearnerController` 相当を呼ぶ」書き方の例を含め、10 番からの移植手順を README に書く |

## 6. 関連文書

| 文書 | 関係 |
|------|------|
| `docs/architecture/simulation-policy.md` | SILS の位置づけと忠実度目標 |
| `firmware/vehicle/docs/development_roadmap.md` | Code / Param / Model Identity の 3 原則 |
| `firmware/vehicle/docs/architecture.md` | アーキテクチャ不変条件（並列経路を作らない） |
| `docs/events/stampfly_workshop/workshop_guide.md` | 「学習者コードを SILS で試す」の現行手順 |
| `firmware/vehicle/examples/10_custom_controller/README.md` | 現行の手作業による実機統合手順（§8） |
