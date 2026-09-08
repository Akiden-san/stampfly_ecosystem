#!/usr/bin/env python3
"""
visualize_stream.py - Data Stream CSV (sf log wifi -o *.csv) Visualization Tool
Data Stream CSV（sf log wifi -o *.csv）可視化ツール

Visualizes the merged 400Hz Data Stream CSV produced by
udp_capture.UDPTelemetryCapture.save_stream_csv() (invoked when
`sf log wifi -o FILE.csv` is used). Unlike the legacy "Extended telemetry"
CSV that visualize_extended.py handles, this format joins the 400Hz
IMU+ESKF block with the 400Hz rate_ref block index-for-index and
forward-fills the slower 50Hz CtrlRef block (angle_ref/total_thrust/
motor_duty/flight_mode) -- see save_stream_csv() in udp_capture.py for the
exact column list and merge logic this mirrors.
udp_capture.UDPTelemetryCapture.save_stream_csv()（`sf log wifi -o FILE.csv`
使用時に呼ばれる）が書き出す、400Hz マージ済み Data Stream CSV を可視化する。
visualize_extended.py が扱う旧「Extended telemetry」CSV とは異なり、この形式は
400Hz の IMU+ESKF ブロックと 400Hz の rate_ref ブロックをインデックスで結合し、
より低速な 50Hz の CtrlRef ブロック（angle_ref/total_thrust/motor_duty/
flight_mode）を前方補完している -- 正確な列一覧とマージ処理は udp_capture.py
の save_stream_csv() を参照（本モジュールはそれを写す）。

Usage:
    python visualize_stream.py <csv_file> [options]

Options:
    --mode {all,attitude,sensors}  Panel group to show (default: all)
    --save FILE                    Save figure to file
    --no-show                      Don't display window (use with --save)
    --time-range START END         Plot only this time range (seconds)

Examples:
    python visualize_stream.py flight.csv
    python visualize_stream.py flight.csv --mode attitude --save attitude.png
    python visualize_stream.py flight.csv --time-range 5 15
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# --- Constants (no magic numbers) / 定数（マジックナンバー禁止） ---

# Conversion factor: radians -> degrees, used for gyro/rate_ref/attitude
# panels so operators can read rates and angles in the units the tutorial
# and firmware logs are normally discussed in.
# rad -> deg 変換係数。gyro/rate_ref/姿勢パネルで使用し、チュートリアルや
# ファームのログで通常使われる単位（度）で読めるようにする。
RAD_TO_DEG = 180.0 / np.pi

# Column set that identifies a Data Stream CSV (see run_viz() in
# lib/sfcli/commands/log.py for the format-detection dispatch this backs).
# Checked BEFORE the legacy "Extended telemetry" test (timestamp_us +
# quat_w) because this format also carries those two columns.
# Data Stream CSV を識別する列集合（判定本体は lib/sfcli/commands/log.py の
# run_viz() 側にある）。この形式も timestamp_us と quat_w を持つため、旧
# 「Extended telemetry」判定より先にチェックする必要がある。
STREAM_REQUIRED_COLUMNS = frozenset({
    'timestamp_us', 'gyro_x', 'gyro_y', 'gyro_z',
    'rate_ref_roll', 'rate_ref_pitch', 'rate_ref_yaw',
    'total_thrust',
})

FIGURE_WIDTH_IN = 12
FIGURE_HEIGHT_PER_PANEL_IN = 1.9
GRID_HSPACE = 0.5
LINE_WIDTH_MEASURED = 1.0
LINE_WIDTH_REFERENCE = 1.0
LINE_WIDTH_THRUST = 1.8
LEGEND_FONT_SIZE = 8
PANEL_TITLE_FONT_SIZE = 10


def is_stream_csv(columns) -> bool:
    """True if `columns` looks like a Data Stream CSV (sf log wifi -o *.csv).
    `columns` が Data Stream CSV（sf log wifi -o *.csv）の列集合に見えれば True。
    """
    return STREAM_REQUIRED_COLUMNS.issubset(set(columns))


def _quat_to_euler_deg(qw, qx, qy, qz):
    """Convert a unit quaternion (w,x,y,z) to roll/pitch/yaw in degrees.
    単位クォータニオン(w,x,y,z)をroll/pitch/yaw[度]に変換する。

    Uses the same convention as visualize_extended.quat_to_euler() (body-to
    -world per the repo's quaternion convention) so attitude plotted from a
    Data Stream CSV agrees with the Extended telemetry CSV plot for the same
    flight.
    visualize_extended.quat_to_euler() と同じ規約（本リポジトリのクォータニオン
    規約でbody-to-world）を使う。同一フライトを Extended telemetry CSV で
    描いた場合と姿勢が食い違わないようにするため。
    """
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG


def load_stream_csv(filename):
    """Load a Data Stream CSV into a DataFrame with a zero-based time column.
    Data Stream CSV を読み込み、0始まりの time 列 `t` [s] を付与する。

    Columns (400Hz, see save_stream_csv() in udp_capture.py): timestamp_us
    [us]; gyro_*/accel_* raw IMU+ESKF outputs [rad/s]/[m/s^2]; quat_* unit
    quaternion; gyro_bias_*/accel_bias_* ESKF bias estimates [rad/s]/
    [m/s^2]; rate_ref_* 400Hz inner-loop rate reference [rad/s];
    angle_ref_roll/pitch 50Hz outer-loop angle reference [rad], forward
    -filled; total_thrust and motor_duty_* forward-filled from the 50Hz
    CtrlRef block; flight_mode integer enum, forward-filled.
    列（400Hz、udp_capture.py の save_stream_csv() 参照）: timestamp_us [us]、
    gyro_*/accel_* は生の IMU+ESKF 出力 [rad/s]/[m/s^2]、quat_* は単位
    クォータニオン、gyro_bias_*/accel_bias_* は ESKF のバイアス推定値
    [rad/s]/[m/s^2]、rate_ref_* は 400Hz 内側ループのレート指令 [rad/s]、
    angle_ref_roll/pitch は 50Hz 外側ループの角度指令 [rad]（前方補完済み）、
    total_thrust と motor_duty_* は 50Hz CtrlRef ブロックから前方補完、
    flight_mode は整数の列挙値（前方補完済み）。
    """
    df = pd.read_csv(filename)
    df['t'] = (df['timestamp_us'] - df['timestamp_us'].iloc[0]) / 1e6

    print(f"Loaded {len(df)} samples from {os.path.basename(filename)}")
    if len(df) > 1:
        duration = df['t'].iloc[-1]
        print(f"Duration: {duration:.1f}s")
        print(f"Sample rate: {len(df) / duration:.1f} Hz")

    return df


def _apply_time_range(df, time_range):
    """Return the subset of `df` whose `t` falls within time_range=(start,end).
    `t` が time_range=(開始,終了) に収まる `df` の部分集合を返す。"""
    if time_range is None:
        return df
    start, end = time_range
    return df[(df['t'] >= start) & (df['t'] <= end)]


def plot_rate_roll(ax, df):
    """Roll rate: gyro_x (measured) vs rate_ref_roll (commanded), both [deg/s].
    This is the panel used to read off a roll-rate step response.
    ロールレート: gyro_x（実測）と rate_ref_roll（指令）を [deg/s] で重ねる。
    ロールレートのステップ応答を読み取るための主パネル。"""
    t = df['t']
    ax.plot(t, df['gyro_x'] * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro')
    ax.plot(t, df['rate_ref_roll'] * RAD_TO_DEG, 'C1--', linewidth=LINE_WIDTH_REFERENCE, label='rate_ref')
    ax.set_ylabel('Roll rate [deg/s]')
    ax.set_title('Roll Rate', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)


def plot_rate_pitch(ax, df):
    """Pitch rate: gyro_y vs rate_ref_pitch, both [deg/s].
    ピッチレート: gyro_y と rate_ref_pitch を [deg/s] で重ねる。"""
    t = df['t']
    ax.plot(t, df['gyro_y'] * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro')
    ax.plot(t, df['rate_ref_pitch'] * RAD_TO_DEG, 'C1--', linewidth=LINE_WIDTH_REFERENCE, label='rate_ref')
    ax.set_ylabel('Pitch rate [deg/s]')
    ax.set_title('Pitch Rate', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)


def plot_rate_yaw(ax, df):
    """Yaw rate: gyro_z vs rate_ref_yaw, both [deg/s].
    ヨーレート: gyro_z と rate_ref_yaw を [deg/s] で重ねる。"""
    t = df['t']
    ax.plot(t, df['gyro_z'] * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro')
    ax.plot(t, df['rate_ref_yaw'] * RAD_TO_DEG, 'C1--', linewidth=LINE_WIDTH_REFERENCE, label='rate_ref')
    ax.set_ylabel('Yaw rate [deg/s]')
    ax.set_title('Yaw Rate', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)


def plot_attitude(ax, df):
    """Attitude from quaternion (roll/pitch/yaw [deg]) plus the 50Hz outer
    -loop angle_ref_roll/pitch [deg], dashed.
    クォータニオンから得た姿勢(roll/pitch/yaw[deg])と、50Hz 外側ループの
    angle_ref_roll/pitch[deg]（破線）を重ねる。"""
    t = df['t']
    roll, pitch, yaw = _quat_to_euler_deg(
        df['quat_w'].to_numpy(), df['quat_x'].to_numpy(),
        df['quat_y'].to_numpy(), df['quat_z'].to_numpy(),
    )
    ax.plot(t, roll, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='roll')
    ax.plot(t, pitch, 'C3-', linewidth=LINE_WIDTH_MEASURED, label='pitch')
    ax.plot(t, yaw, 'C2-', linewidth=LINE_WIDTH_MEASURED, label='yaw')
    ax.plot(t, df['angle_ref_roll'] * RAD_TO_DEG, 'C0--', linewidth=LINE_WIDTH_REFERENCE, label='roll_ref')
    ax.plot(t, df['angle_ref_pitch'] * RAD_TO_DEG, 'C3--', linewidth=LINE_WIDTH_REFERENCE, label='pitch_ref')
    ax.set_ylabel('Attitude [deg]')
    ax.set_title('Attitude (from quaternion)', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, ncol=3)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)


def plot_accel(ax, df):
    """Body-frame acceleration x/y/z [m/s^2].
    機体座標系の加速度 x/y/z [m/s^2]。"""
    t = df['t']
    ax.plot(t, df['accel_x'], 'C0-', linewidth=LINE_WIDTH_MEASURED, label='accel_x')
    ax.plot(t, df['accel_y'], 'C3-', linewidth=LINE_WIDTH_MEASURED, label='accel_y')
    ax.plot(t, df['accel_z'], 'C2-', linewidth=LINE_WIDTH_MEASURED, label='accel_z')
    ax.set_ylabel('Accel [m/s^2]')
    ax.set_title('Acceleration', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, ncol=3)
    ax.grid(True, alpha=0.3)


def plot_gyro_raw(ax, df):
    """Raw gyro rates x/y/z [deg/s] (no reference overlay -- for the
    "sensors" mode where only the raw signal is of interest).
    生のジャイロレート x/y/z [deg/s]（指令値は重ねない — 生信号だけを見たい
    "sensors" モード向け）。"""
    t = df['t']
    ax.plot(t, df['gyro_x'] * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro_x')
    ax.plot(t, df['gyro_y'] * RAD_TO_DEG, 'C3-', linewidth=LINE_WIDTH_MEASURED, label='gyro_y')
    ax.plot(t, df['gyro_z'] * RAD_TO_DEG, 'C2-', linewidth=LINE_WIDTH_MEASURED, label='gyro_z')
    ax.set_ylabel('Gyro [deg/s]')
    ax.set_title('Raw Gyro', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, ncol=3)
    ax.grid(True, alpha=0.3)


def plot_thrust_motors(ax, df):
    """total_thrust (thick black) plus the 4 motor_duty_* channels [0,1] on
    the same axis.
    total_thrust（太い黒線）と motor_duty_* の4チャンネル[0,1]を同じ軸に描く。"""
    t = df['t']
    ax.plot(t, df['total_thrust'], 'k-', linewidth=LINE_WIDTH_THRUST, label='total_thrust [N]')
    ax.plot(t, df['motor_duty_FR'], 'C0-', linewidth=LINE_WIDTH_MEASURED, label='FR')
    ax.plot(t, df['motor_duty_RR'], 'C1-', linewidth=LINE_WIDTH_MEASURED, label='RR')
    ax.plot(t, df['motor_duty_RL'], 'C2-', linewidth=LINE_WIDTH_MEASURED, label='RL')
    ax.plot(t, df['motor_duty_FL'], 'C3-', linewidth=LINE_WIDTH_MEASURED, label='FL')
    ax.set_ylabel('thrust [N] / duty [0-1]')
    ax.set_title('Thrust and Motors', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, ncol=5)
    ax.grid(True, alpha=0.3)


def plot_bias_and_mode(ax, df):
    """Gyro bias x/y/z [deg/s] on the left axis, flight_mode as a step line
    on a twin right axis.
    ジャイロバイアス x/y/z [deg/s] を左軸に、flight_mode をステップ線で
    右の双子軸に描く。"""
    t = df['t']
    ax.plot(t, df['gyro_bias_x'] * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='bias_x')
    ax.plot(t, df['gyro_bias_y'] * RAD_TO_DEG, 'C3-', linewidth=LINE_WIDTH_MEASURED, label='bias_y')
    ax.plot(t, df['gyro_bias_z'] * RAD_TO_DEG, 'C2-', linewidth=LINE_WIDTH_MEASURED, label='bias_z')
    ax.set_ylabel('Gyro bias [deg/s]')
    ax.set_title('Gyro Bias / Flight Mode', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.grid(True, alpha=0.3)

    ax_mode = ax.twinx()
    ax_mode.step(t, df['flight_mode'], 'k-', where='post', linewidth=1.2, label='flight_mode')
    ax_mode.set_ylabel('flight_mode')

    # Merge both axes' legends into a single box so the mode line isn't
    # missed.
    # 両軸の凡例を1つの箱にまとめ、モード線が見落とされないようにする。
    lines_left, labels_left = ax.get_legend_handles_labels()
    lines_right, labels_right = ax_mode.get_legend_handles_labels()
    ax.legend(lines_left + lines_right, labels_left + labels_right,
              loc='upper right', fontsize=LEGEND_FONT_SIZE, ncol=4)


# Panel groups selectable via --mode / mode 引数で選べるパネル群
_PANEL_GROUPS = {
    'all': [
        plot_rate_roll, plot_rate_pitch, plot_rate_yaw,
        plot_attitude, plot_accel, plot_thrust_motors, plot_bias_and_mode,
    ],
    'attitude': [plot_rate_roll, plot_rate_pitch, plot_rate_yaw, plot_attitude],
    'sensors': [plot_accel, plot_gyro_raw],
}


def visualize_all(df, filename, save_path=None, show=True, time_range=None, mode='all'):
    """Render the Data Stream CSV panel group selected by `mode`.
    `mode` で選んだ Data Stream CSV のパネル群を描画する。

    mode='all' (default): all 7 panels (rate x3, attitude, accel,
    thrust/motors, bias/flight_mode).
    mode='attitude': panels 1-4 only (rate x3, attitude).
    mode='sensors': accel + raw gyro.
    Any other value falls back to 'all' with an info message, since this
    format has no such panel group.
    mode='all'（既定）: 全7パネル（レートx3、姿勢、加速度、推力/モータ、
    バイアス/flight_mode）。
    mode='attitude': 1〜4番パネルのみ（レートx3、姿勢）。
    mode='sensors': 加速度 + 生ジャイロ。
    それ以外の値は、この形式には該当パネル群がない旨を表示した上で
    'all' にフォールバックする。
    """
    if mode not in _PANEL_GROUPS:
        print(f"Info: Data Stream CSV has no '{mode}' panel group; showing 'all' instead.")
        mode = 'all'

    df = _apply_time_range(df, time_range)
    panel_fns = _PANEL_GROUPS[mode]
    n_panels = len(panel_fns)

    fig = plt.figure(figsize=(FIGURE_WIDTH_IN, FIGURE_HEIGHT_PER_PANEL_IN * n_panels))
    fig.suptitle(
        f'{os.path.basename(filename)} - Data Stream CSV (sf log wifi -o *.csv, 400 Hz)',
        fontsize=14,
    )
    gs = GridSpec(n_panels, 1, figure=fig, hspace=GRID_HSPACE)

    ax_prev = None
    for i, plot_fn in enumerate(panel_fns):
        ax = fig.add_subplot(gs[i, 0], sharex=ax_prev)
        plot_fn(ax, df)
        ax_prev = ax

    # Only the bottom panel gets the shared "Time [s]" x-axis label -- panel
    # functions never set it themselves, since which panel is last depends
    # on `mode`.
    # 一番下のパネルにのみ共有の "Time [s]" x軸ラベルを付ける -- どのパネルが
    # 最後になるかは mode 次第なので、各パネル関数側では設定しない。
    ax_prev.set_xlabel('Time [s]')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved figure to {save_path}")

    if show:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize a Data Stream CSV (sf log wifi -o *.csv)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('file', help='Data Stream CSV path')
    parser.add_argument('--mode', choices=list(_PANEL_GROUPS.keys()), default='all',
                        help='Panel group to show (default: all)')
    parser.add_argument('--save', metavar='FILE', help='Save figure to file')
    parser.add_argument('--no-show', action='store_true', help="Don't display window")
    parser.add_argument('--time-range', nargs=2, type=float, metavar=('START', 'END'),
                        help='Time range to plot (seconds)')

    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: File not found: {args.file}")
        sys.exit(1)

    df = load_stream_csv(args.file)
    time_range = tuple(args.time_range) if args.time_range else None
    visualize_all(df, args.file, save_path=args.save, show=not args.no_show,
                  time_range=time_range, mode=args.mode)


if __name__ == '__main__':
    main()
