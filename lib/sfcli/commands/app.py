"""
sf app - Manage your own drone-firmware projects

Create, edit, build, and flash your own StampFly firmware project. A new
project is cloned from a `firmware/vehicle/examples/<N>` example (e.g. the
Topic API read-only example, or the custom-controller example) into
`firmware/apps/<name>`, so a research user gets exactly the same
new -> edit -> build -> flash flow that workshop participants already know
from `sf lesson`.

自分のドローンファームウェアプロジェクトを作成・編集・ビルド・書き込みする。
新規プロジェクトは `firmware/vehicle/examples/<N>` の例題（Topic API 読取専用
例題、自作コントローラ例題 等）を `firmware/apps/<name>` に複製して作る。
研究利用者にも `sf lesson` でワークショップ参加者が既に知っている
new -> edit -> build -> flash と同じ流れを提供する。

Subcommands:
    new     - Create a new project cloned from an example
    edit    - Open the learner-facing source file in an editor
    build   - Build a project (= sf build apps/<name>)
    flash   - Flash a project (= sf flash apps/<name>)
    list    - List your projects and the examples available for --from
"""

import argparse
import re
import shutil
import subprocess
from pathlib import Path
from typing import List, Optional

from ..utils import console, editor, paths

COMMAND_NAME = "app"
COMMAND_HELP = "Manage your own drone-firmware projects"

# Reserved names that cannot be used for an app project. Projects actually
# live one level deeper (firmware/apps/<name>), so none of these would
# literally collide on disk -- they are rejected anyway because reusing a
# top-level firmware/ directory name here would be confusing to read.
# アプリのプロジェクト名として使えない予約名。実際には1階層下の
# firmware/apps/<name> に置かれるためディスク上で衝突はしないが、
# firmware/ 直下の既存ディレクトリ名を流用すると紛らわしいため禁止する。
RESERVED_NAMES = {"vehicle", "vehicle_old", "controller", "workshop", "common", "apps"}

# Default --from example: the smallest complete control-law exercise. See
# firmware/vehicle/examples/10_custom_controller/README.md for why this is
# a good default entry point (no hardware required, one exercise hook).
# --from 省略時の既定値: 最小の完結した制御則演習。実機不要・演習フックが
# 1箇所のみで、既定の入口として適切な理由は README.md 参照。
DEFAULT_EXAMPLE = "10_custom_controller"

# Never copy these from an example into a new project -- build output and
# dependency-manager state, which must be regenerated fresh for the new
# project (they cache paths/names tied to the example's own project name).
# 例題から新規プロジェクトへコピーしない物 -- ビルド成果物と依存関係
# マネージャの状態。例題自身のプロジェクト名に紐づくキャッシュのため、
# 新しいプロジェクト用に作り直させる。
_COPY_EXCLUDE = ("build", "sdkconfig", "managed_components", "dependencies.lock")

# The file a learner is expected to write, checked in this priority order
# inside a project's main/ directory. 10_custom_controller (and any project
# cloned from it) separates the exercise into learner_controller.cpp; older
# single-file examples (e.g. 09_topic_api_hello) only have main.cpp.
# 学習者が書くことを想定したファイル。プロジェクトの main/ 内でこの優先順位
# で確認する。10_custom_controller（から複製したプロジェクト）は演習部分を
# learner_controller.cpp に分離しているが、単一ファイル構成の例題
# （例: 09_topic_api_hello）は main.cpp のみを持つ。
_LEARNER_FILE_PRIORITY = ("learner_controller.cpp", "main.cpp")


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    app_subparsers = parser.add_subparsers(
        dest="app_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- new ---
    new_parser = app_subparsers.add_parser(
        "new",
        help="Create a new project cloned from an example",
        description="Copy a firmware/vehicle/examples/<N> example into firmware/apps/<name>.",
    )
    new_parser.add_argument("name", help="Project name (e.g. my_ctrl); must be a valid C identifier")
    new_parser.add_argument(
        "--from",
        dest="from_example",
        default=DEFAULT_EXAMPLE,
        metavar="<example>",
        help=f"Example directory under firmware/vehicle/examples/ to clone (default: {DEFAULT_EXAMPLE})",
    )
    new_parser.set_defaults(func=run_new)

    # --- edit ---
    edit_parser = app_subparsers.add_parser(
        "edit",
        help="Open the learner-facing source file in an editor",
        description="Open main/learner_controller.cpp (or main/main.cpp) in your editor (VSCode > vi).",
    )
    edit_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
    edit_parser.add_argument(
        "--editor",
        default=None,
        help="Override editor command (e.g., --editor nvim)",
    )
    edit_parser.add_argument(
        "--reuse-window",
        action="store_true",
        help="Open in existing VSCode window instead of a new one (default: new window)",
    )
    edit_parser.set_defaults(func=run_edit)

    # --- build ---
    build_parser = app_subparsers.add_parser(
        "build",
        help="Build a project",
        description="Build a project (equivalent to 'sf build apps/<name>').",
    )
    build_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
    build_parser.add_argument(
        "-c", "--clean",
        action="store_true",
        help="Clean build (fullclean before build)",
    )
    build_parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose build output",
    )
    build_parser.set_defaults(func=run_build)

    # --- flash ---
    flash_parser = app_subparsers.add_parser(
        "flash",
        help="Flash a project",
        description="Flash a project (equivalent to 'sf flash apps/<name>').",
    )
    flash_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
    flash_parser.add_argument(
        "-p", "--port",
        default=None,
        help="Serial port (auto-detect if not specified)",
    )
    flash_parser.add_argument(
        "-b", "--baud",
        type=int,
        default=460800,
        help="Baud rate (default: 460800)",
    )
    flash_parser.add_argument(
        "-m", "--monitor",
        action="store_true",
        help="Start monitor after flashing",
    )
    flash_parser.set_defaults(func=run_flash)

    # --- list ---
    list_parser = app_subparsers.add_parser(
        "list",
        help="List your projects and the examples available for --from",
        description="Show firmware/apps/ projects and firmware/vehicle/examples/ sources for --from.",
    )
    list_parser.set_defaults(func=run_list)

    # Default: show help
    parser.set_defaults(func=lambda args: (parser.print_help(), 0)[1])


# =============================================================================
# new
# =============================================================================

def _validate_new_name(name: str) -> Optional[str]:
    """Validate a new project name. Returns an error message, or None if OK."""
    if not name.isidentifier():
        return f"Invalid project name: '{name}' (must be a valid C identifier)"

    if name in RESERVED_NAMES:
        return f"Reserved name: '{name}' (cannot use {', '.join(sorted(RESERVED_NAMES))})"

    project_dir = paths.apps() / name
    if project_dir.exists():
        return f"Project already exists: {project_dir}"

    return None


def _examples_dir() -> Path:
    """Get firmware/vehicle/examples/ directory."""
    return paths.vehicle() / "examples"


def _example_summary(example_dir: Path) -> str:
    """First Markdown heading (`# ...`) of an example's README.md, or "" if
    there is no README or no such heading.
    例題の README.md にある最初の Markdown 見出し（`# ...`）。README が無い、
    または見出しが見つからない場合は ""。

    Every example's heading follows "# <dirname> — <description>"
    (verified above); that leading "<dirname> — " is stripped here since
    the caller already prints the directory name right before this summary.
    全例題の見出しは "# <dirname> — <description>" 形式（上記で確認済み）。
    呼び出し側は既にディレクトリ名をこの要約の直前に表示するため、先頭の
    "<dirname> — " はここで取り除く。
    """
    readme = example_dir / "README.md"
    if not readme.exists():
        return ""
    for line in readme.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("# "):
            heading = stripped[2:].strip()
            for separator in (" — ", " - "):
                prefix = f"{example_dir.name}{separator}"
                if heading.startswith(prefix):
                    return heading[len(prefix):]
            return heading
    return ""


def _print_available_examples(examples_dir: Path) -> None:
    """Print every example directory (with its README's first heading, if any)."""
    if not examples_dir.exists():
        console.print("  (no examples found)")
        return

    found = False
    for d in sorted(examples_dir.iterdir()):
        if d.is_dir() and (d / "CMakeLists.txt").exists():
            summary = _example_summary(d)
            suffix = f" -- {summary}" if summary else ""
            console.print(f"  {d.name}{suffix}")
            found = True

    if not found:
        console.print("  (no examples found)")


def _rewrite_cmake_project(project_dir: Path, name: str) -> None:
    """Point the copied top-level CMakeLists.txt at vehicle/components and
    rename its project() to the new app name.

    複製したトップレベル CMakeLists.txt の部品探索先を vehicle/components に
    向け直し、project() を新しいアプリ名に書き換える。

    Every firmware/vehicle/examples/<N>/CMakeLists.txt sets
    `EXTRA_COMPONENT_DIRS ../../components`, which resolves (relative to
    that file's own directory, firmware/vehicle/examples/<N>/) to
    firmware/vehicle/components. A copy living one level shallower, at
    firmware/apps/<name>/, needs `../../vehicle/components` to resolve to
    the same directory -- verified with `os.path.relpath`, not guessed.
    firmware/vehicle/examples/<N>/CMakeLists.txt はいずれも
    `EXTRA_COMPONENT_DIRS ../../components` を設定しており、これは
    （そのファイル自身のディレクトリ firmware/vehicle/examples/<N>/ から見て）
    firmware/vehicle/components に解決される。1階層浅い
    firmware/apps/<name>/ に置かれた複製が同じディレクトリに解決するには
    `../../vehicle/components` が必要 -- 推測ではなく `os.path.relpath` で
    検証済み。
    """
    cmake_file = project_dir / "CMakeLists.txt"
    content = cmake_file.read_text(encoding="utf-8")

    component_dirs = "../../vehicle/components"
    content, count = re.subn(
        r"set\(EXTRA_COMPONENT_DIRS\s+[^)]*\)",
        f"set(EXTRA_COMPONENT_DIRS {component_dirs})",
        content,
    )
    if count == 0:
        console.warning(
            f"Could not find EXTRA_COMPONENT_DIRS in {cmake_file} -- check it manually"
        )

    content, count = re.subn(r"project\([^)]*\)", f"project({name})", content)
    if count == 0:
        console.warning(f"Could not find project(...) in {cmake_file} -- check it manually")

    cmake_file.write_text(content, encoding="utf-8")


def run_new(args: argparse.Namespace) -> int:
    """Create a new project cloned from an example"""
    name = args.name
    from_example = args.from_example

    error = _validate_new_name(name)
    if error:
        console.error(error)
        return 1

    examples_dir = _examples_dir()
    example_dir = examples_dir / from_example
    if not example_dir.exists() or not (example_dir / "CMakeLists.txt").exists():
        console.error(f"Example not found: '{from_example}'")
        console.print()
        console.print("Available examples (firmware/vehicle/examples/):")
        _print_available_examples(examples_dir)
        return 1

    project_dir = paths.apps() / name

    console.info(f"Creating new app project: {name}")
    console.print(f"  From: {example_dir}")
    console.print(f"  To:   {project_dir}")

    paths.ensure_dir(paths.apps())
    shutil.copytree(example_dir, project_dir, ignore=shutil.ignore_patterns(*_COPY_EXCLUDE))

    _rewrite_cmake_project(project_dir, name)

    console.success(f"Project created: {project_dir}")
    console.print()
    console.print("Next steps:")
    console.print(f"  sf app edit {name}")
    console.print(f"  sf app build {name}")
    console.print(f"  sf app flash {name} -m")

    return 0


# =============================================================================
# edit
# =============================================================================

def _learner_file(project_dir: Path) -> Optional[Path]:
    """The file the learner is expected to edit, or None if project_dir/main
    has neither of the files in _LEARNER_FILE_PRIORITY.
    学習者が編集すべきファイル。project_dir/main に
    _LEARNER_FILE_PRIORITY のいずれも無ければ None。
    """
    main_dir = project_dir / "main"
    for filename in _LEARNER_FILE_PRIORITY:
        candidate = main_dir / filename
        if candidate.exists():
            return candidate
    return None


def run_edit(args: argparse.Namespace) -> int:
    """Open the learner-facing source file in the editor"""
    project_dir = paths.apps() / args.name
    if not project_dir.exists():
        console.error(f"Project not found: {project_dir}")
        console.print(f"  Create it first: sf app new {args.name}")
        return 1

    target_file = _learner_file(project_dir)
    if target_file is None:
        console.error(
            f"No editable source file found under {project_dir / 'main'} "
            f"(looked for {' or '.join(_LEARNER_FILE_PRIORITY)})"
        )
        return 1

    preferred = getattr(args, "editor", None)
    found = editor.find_editor(preferred)

    if found is None:
        if preferred:
            console.error(f"Editor not found: '{preferred}'")
        else:
            console.error("No editor found (tried: code, vi, vim)")
        console.print()
        for line in editor.install_hint(f"sf app edit {args.name} --editor <command>"):
            console.print(line)
        return 1

    editor_name, cmd = found
    reuse_window = bool(getattr(args, "reuse_window", False))

    # VSCode: open in a new window by default so this project does not
    # piggyback into an unrelated workspace already open in VSCode.
    # VSCode: 既に開いている別プロジェクトのウィンドウに紛れないよう、デフォルトで新規ウィンドウ
    extra_flags: List[str] = []
    if editor_name.startswith("VSCode") and not reuse_window:
        extra_flags = ["-n"]

    full_cmd = cmd + extra_flags + [str(target_file)]

    window_note = "" if not extra_flags else " (new window)"
    console.info(f"Opening {target_file.name} in {editor_name}{window_note}")
    console.print(f"  Path: {target_file}")

    try:
        # shell=False is safe: command + path are passed as a list
        # shell=False で安全: コマンドとパスをリストで渡す
        result = subprocess.run(full_cmd)
        return result.returncode
    except FileNotFoundError:
        console.error(f"Failed to launch editor: {' '.join(full_cmd)}")
        return 1


# =============================================================================
# build / flash
# =============================================================================

def run_build(args: argparse.Namespace) -> int:
    """Build a project (delegates to `sf build apps/<name>`)"""
    from . import build as build_cmd

    # Delegate through build.py's factory so newly added run() attributes
    # stay in sync automatically (see build.make_run_args docstring).
    # build.py のファクトリ経由で委譲する。run() に属性が増えても自動的に
    # 追従する（build.make_run_args の docstring 参照）。
    build_args = build_cmd.make_run_args(
        target=f"apps/{args.name}",
        clean=args.clean,
        verbose=args.verbose,
    )
    return build_cmd.run(build_args)


def run_flash(args: argparse.Namespace) -> int:
    """Flash a project (delegates to `sf flash apps/<name>`)"""
    from . import flash as flash_cmd

    # Delegate through flash.py's factory so newly added run() attributes
    # (e.g. --gui) stay in sync automatically (see flash.make_run_args
    # docstring).
    # flash.py のファクトリ経由で委譲する。run() に属性が増えても
    # （例: --gui）自動的に追従する（flash.make_run_args の docstring参照）。
    flash_args = flash_cmd.make_run_args(
        target=f"apps/{args.name}",
        port=args.port,
        baud=args.baud,
        monitor=args.monitor,
    )
    return flash_cmd.run(flash_args)


# =============================================================================
# list
# =============================================================================

def run_list(args: argparse.Namespace) -> int:
    """List firmware/apps/ projects and firmware/vehicle/examples/ sources"""
    apps_dir = paths.apps()

    console.header("Your Projects (firmware/apps/)")
    console.print()
    found = False
    if apps_dir.exists():
        for d in sorted(apps_dir.iterdir()):
            if d.is_dir() and (d / "CMakeLists.txt").exists():
                console.print(f"  {d.name}")
                found = True
    if not found:
        console.print("  (none yet -- create one: sf app new <name>)")
    console.print()

    console.header("Examples Available for --from (firmware/vehicle/examples/)")
    console.print()
    _print_available_examples(_examples_dir())
    console.print()
    console.print(f"  Default: {DEFAULT_EXAMPLE}")

    return 0
