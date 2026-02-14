"""
sf takeoff / land / hover / jump - Flight commands

Send flight commands to StampFly via WiFi CLI and monitor in real-time.
WiFi CLI経由でフライトコマンドを送信し、リアルタイムで監視します。

Commands:
    takeoff [alt]       - Take off to specified altitude (default: 0.5m)
    land                - Land the vehicle
    hover [alt] [dur]   - Hover at altitude for duration (default: 0.5m, 5.0s)
    jump [alt]          - Quick jump: climb then descend (default: 0.15m)
"""

import argparse
import asyncio
import sys
import time

from ..utils import console
from ..utils.vehicle_connection import VehicleConnection, DEFAULT_HOST


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register flight commands with CLI

    フライトコマンドをCLIに登録する
    """
    # --- takeoff ---
    takeoff_parser = subparsers.add_parser(
        "takeoff",
        help="Take off to specified altitude",
        description="Take off and hold altitude. WiFi CLI経由で離陸コマンドを送信。",
    )
    takeoff_parser.add_argument(
        "altitude",
        nargs="?",
        type=float,
        default=0.5,
        help="Target altitude in meters (0.1-2.0, default: 0.5)",
    )
    _add_common_args(takeoff_parser)
    takeoff_parser.set_defaults(func=run_takeoff)

    # --- land ---
    land_parser = subparsers.add_parser(
        "land",
        help="Land the vehicle",
        description="Land the vehicle. WiFi CLI経由で着陸コマンドを送信。",
    )
    _add_common_args(land_parser)
    land_parser.set_defaults(func=run_land)

    # --- hover ---
    hover_parser = subparsers.add_parser(
        "hover",
        help="Hover at altitude for duration",
        description="Hover at specified altitude for a duration then land. "
                    "WiFi CLI経由でホバリングコマンドを送信。",
    )
    hover_parser.add_argument(
        "altitude",
        nargs="?",
        type=float,
        default=0.5,
        help="Target altitude in meters (0.1-2.0, default: 0.5)",
    )
    hover_parser.add_argument(
        "duration",
        nargs="?",
        type=float,
        default=5.0,
        help="Hover duration in seconds (0.1-60.0, default: 5.0)",
    )
    _add_common_args(hover_parser)
    hover_parser.set_defaults(func=run_hover)

    # --- jump ---
    jump_parser = subparsers.add_parser(
        "jump",
        help="Quick jump: climb then descend",
        description="Quick jump: climb to altitude then descend immediately. "
                    "WiFi CLI経由でジャンプコマンドを送信。",
    )
    jump_parser.add_argument(
        "altitude",
        nargs="?",
        type=float,
        default=0.15,
        help="Target altitude in meters (0.1-2.0, default: 0.15)",
    )
    _add_common_args(jump_parser)
    jump_parser.set_defaults(func=run_jump)


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    """Add common options to a flight command parser.

    フライトコマンドパーサーに共通オプションを追加する
    """
    parser.add_argument(
        "--ip",
        default=DEFAULT_HOST,
        help=f"Vehicle IP address (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Monitor timeout in seconds (default: 120)",
    )
    parser.add_argument(
        "--no-monitor",
        action="store_true",
        help="Send command without monitoring (fire & forget)",
    )


# =============================================================================
# Command Runners
# =============================================================================

def run_takeoff(args: argparse.Namespace) -> int:
    """Execute takeoff command"""
    alt = args.altitude
    if alt < 0.1 or alt > 2.0:
        console.error("Altitude must be 0.1-2.0 [m]")
        return 1

    console.info(f"Takeoff to {alt:.2f} m", prefix="TAKEOFF")
    return _run_flight_command(
        cli_cmd=f"takeoff {alt}",
        label="TAKEOFF",
        target_alt=alt,
        args=args,
    )


def run_land(args: argparse.Namespace) -> int:
    """Execute land command"""
    console.info("Landing", prefix="LAND")
    return _run_flight_command(
        cli_cmd="land",
        label="LAND",
        target_alt=0.0,
        args=args,
    )


def run_hover(args: argparse.Namespace) -> int:
    """Execute hover command"""
    alt = args.altitude
    dur = args.duration
    if alt < 0.1 or alt > 2.0:
        console.error("Altitude must be 0.1-2.0 [m]")
        return 1
    if dur < 0.1 or dur > 60.0:
        console.error("Duration must be 0.1-60.0 [s]")
        return 1

    console.info(f"Hover at {alt:.2f} m for {dur:.1f} s", prefix="HOVER")
    return _run_flight_command(
        cli_cmd=f"hover {alt} {dur}",
        label="HOVER",
        target_alt=alt,
        args=args,
    )


def run_jump(args: argparse.Namespace) -> int:
    """Execute jump command"""
    alt = args.altitude
    if alt < 0.1 or alt > 2.0:
        console.error("Altitude must be 0.1-2.0 [m]")
        return 1

    console.info(f"Jump to {alt:.2f} m", prefix="JUMP")
    return _run_flight_command(
        cli_cmd=f"jump {alt}",
        label="JUMP",
        target_alt=alt,
        args=args,
    )


# =============================================================================
# Common Flight Execution
# =============================================================================

def _run_flight_command(
    cli_cmd: str,
    label: str,
    target_alt: float,
    args: argparse.Namespace,
) -> int:
    """Common flight command execution flow.

    共通フライトコマンド実行フロー
    """
    try:
        return asyncio.run(_async_flight(cli_cmd, label, target_alt, args))
    except KeyboardInterrupt:
        # Ctrl+C triggers cancel in the async handler
        # Ctrl+Cは非同期ハンドラ内でキャンセルを発動
        return 130


async def _async_flight(
    cli_cmd: str,
    label: str,
    target_alt: float,
    args: argparse.Namespace,
) -> int:
    """Async flight command execution.

    非同期フライトコマンド実行
    """
    conn = VehicleConnection()
    start_time = time.monotonic()

    try:
        # Connect to vehicle
        # 機体に接続
        console.print(f"  Connecting to {args.ip}...")
        try:
            await conn.connect(args.ip, timeout=5.0)
        except OSError:
            console.error(
                "Vehicle not reachable. Connect to StampFly WiFi AP first."
            )
            return 1
        except ImportError as e:
            console.error(str(e))
            return 1

        console.success("Connected (CLI + WebSocket)")

        # Send flight command
        # フライトコマンドを送信
        response = await conn.send_flight_command(cli_cmd)
        if response:
            console.print(f"  {response}")

        # Check for error in response
        # 応答にエラーがないかチェック
        if "Error" in response or "Failed" in response:
            console.error("Command rejected by vehicle")
            await conn.disconnect()
            return 1

        # Fire & forget mode
        # ファイア＆フォーゲットモード
        if args.no_monitor:
            console.success("Command sent (no-monitor mode)")
            await conn.disconnect()
            return 0

        # Monitor flight with real-time display
        # リアルタイム表示で飛行を監視
        console.print()

        monitor_state = {
            "label": label,
            "target_alt": target_alt,
            "start_time": start_time,
            "last_line_len": 0,
        }

        try:
            await conn.monitor_flight(
                callback=lambda telem, status: _display_callback(
                    telem, status, monitor_state
                ),
                poll_interval=0.5,
                timeout=args.timeout,
            )
        except KeyboardInterrupt:
            # Cancel flight on Ctrl+C
            # Ctrl+Cでフライトをキャンセル
            console.print()  # Clear the \r line
            console.warning("Cancelling flight...")
            try:
                cancel_resp = await conn.cancel_flight()
                if cancel_resp:
                    console.print(f"  {cancel_resp}")
            except Exception:
                pass
            await conn.disconnect()
            return 130

        # Final summary
        # 最終サマリー
        elapsed = time.monotonic() - start_time
        console.print()  # Newline after \r updates
        console.success(
            f"{label} completed ({elapsed:.1f}s)", prefix="OK"
        )

        await conn.disconnect()
        return 0

    except ConnectionError as e:
        console.print()
        console.error(f"Connection lost: {e}")
        return 1
    except Exception as e:
        console.print()
        console.error(f"Flight command failed: {e}")
        try:
            await conn.disconnect()
        except Exception:
            pass
        return 1


def _display_callback(
    telemetry: dict,
    flight_status: dict,
    state: dict,
) -> None:
    """Real-time flight status display callback (overwrites line with \\r).

    リアルタイムフライト状態表示コールバック（\\rで行を上書き）
    """
    label = state["label"]
    target_alt = state["target_alt"]
    elapsed = time.monotonic() - state["start_time"]

    # Extract values from telemetry
    # テレメトリから値を取得
    pos_z = telemetry.get("pos_z", 0.0) if telemetry else 0.0
    vel_z = telemetry.get("vel_z", 0.0) if telemetry else 0.0
    tof = telemetry.get("tof_bottom", 0.0) if telemetry else 0.0

    # Flight state from CLI polling
    # CLIポーリングからのフライト状態
    cmd_state = flight_status.get("state", "UNKNOWN")

    # Format velocity sign
    # 速度の符号をフォーマット
    vel_sign = "+" if vel_z >= 0 else ""

    # Build status line
    # ステータス行を構築
    if target_alt > 0:
        line = (
            f"  [{label}] {cmd_state} | "
            f"Alt: {pos_z:.2f}m -> {target_alt:.2f}m | "
            f"Vel: {vel_sign}{vel_z:.2f} m/s | "
            f"ToF: {tof:.2f}m | "
            f"{elapsed:.1f}s"
        )
    else:
        line = (
            f"  [{label}] {cmd_state} | "
            f"Alt: {pos_z:.2f}m | "
            f"Vel: {vel_sign}{vel_z:.2f} m/s | "
            f"ToF: {tof:.2f}m | "
            f"{elapsed:.1f}s"
        )

    # Pad with spaces to clear previous longer line
    # 前の長い行を消すためにスペースでパディング
    pad = max(0, state["last_line_len"] - len(line))
    print(f"\r{line}{' ' * pad}", end="", flush=True)
    state["last_line_len"] = len(line)
