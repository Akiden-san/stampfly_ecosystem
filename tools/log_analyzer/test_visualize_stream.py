#!/usr/bin/env python3
"""
test_visualize_stream.py - Tests for the Data Stream CSV visualizer
Data Stream CSV 可視化ツールのテスト

Builds a synthetic capture with udp_capture.UDPTelemetryCapture (the same
way tools/log_analyzer's own scratch scripts build one) and writes it with
the REAL save_stream_csv(), so these tests exercise the exact CSV format
`sf log wifi -o FILE.csv` produces -- not a hand-rolled approximation of it.
udp_capture.UDPTelemetryCapture で合成キャプチャを組み立て、実物の
save_stream_csv() で書き出す。これにより、`sf log wifi -o FILE.csv` が
実際に出す CSV 形式そのものをテストする（手組みの近似ではなく）。

Usage:
    python3 test_visualize_stream.py
    pytest test_visualize_stream.py
"""

import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import visualize_extended  # noqa: E402
import visualize_stream  # noqa: E402

# Minimum PNG size used as a smoke check that matplotlib actually rendered
# panels (an empty/failed figure saves far smaller than this).
# matplotlib が実際にパネルを描画したことを確認する下限サイズ（空/失敗図は
# これより大幅に小さく保存される）。
MIN_PNG_BYTES = 10_000

SAMPLE_RATE_HZ = 400
DURATION_S = 5
N_SAMPLES = SAMPLE_RATE_HZ * DURATION_S
DT_US = 1_000_000 // SAMPLE_RATE_HZ
CTRL_REF_STRIDE = 8  # 50Hz CtrlRef among 400Hz IMU+ESKF samples / 400Hz中50Hz


def _build_stream_csv(tmp_path) -> Path:
    """Build a synthetic 5s/400Hz capture (roll rate_ref step) and save it
    with the real save_stream_csv(); return the CSV path.
    5秒/400Hzの合成キャプチャ（ロールrate_refのステップ）を組み立て、実物の
    save_stream_csv() で保存し、CSVパスを返す。"""
    cap = udp_capture.UDPTelemetryCapture()
    for i in range(N_SAMPLES):
        ts = 1_000_000 + i * DT_US
        cap.samples[udp_capture.PKT_IMU_ESKF].append({
            'timestamp_us': ts,
            'gyro_x': 0.1 * math.sin(i / 50), 'gyro_y': 0.0, 'gyro_z': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': -9.81,
            'gyro_raw_x': 0.0, 'gyro_raw_y': 0.0, 'gyro_raw_z': 0.0,
            'accel_raw_x': 0.0, 'accel_raw_y': 0.0, 'accel_raw_z': -9.81,
            'quat_w': 1.0, 'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0,
            'gyro_bias_x': 0.0, 'gyro_bias_y': 0.0, 'gyro_bias_z': 0.0,
            'accel_bias_x': 0.0, 'accel_bias_y': 0.0, 'accel_bias_z': 0.0,
        })
        # Roll rate_ref step at t=1s: 0 -> 100 (wire units, /1000.0 in
        # save_stream_csv() -> 0.1 rad/s).
        # t=1sでロールrate_refをステップ: 0 -> 100（電文単位、
        # save_stream_csv() で /1000.0 -> 0.1 rad/s）。
        rate_ref_roll = 100 if i >= N_SAMPLES // 2 else 0
        cap.samples[udp_capture.PKT_RATE_REF].append({
            'timestamp_us': ts, 'rate_ref_roll': rate_ref_roll,
            'rate_ref_pitch': 0, 'rate_ref_yaw': 0,
        })
        if i % CTRL_REF_STRIDE == 0:
            cap.samples[udp_capture.PKT_CTRL_REF].append({
                'timestamp_us': ts, 'flight_mode': 1, 'reserved': 0,
                'angle_ref_roll': 0, 'angle_ref_pitch': 0, 'total_thrust': 0.5,
                'motor_duty_FR': 0.5, 'motor_duty_RR': 0.5,
                'motor_duty_RL': 0.5, 'motor_duty_FL': 0.5,
            })

    csv_path = tmp_path / "stream.csv"
    cap.save_stream_csv(str(csv_path))
    return csv_path


def _read_header(csv_path):
    with open(csv_path, 'r') as f:
        return next(csv.reader(f))


def test_is_stream_csv_true_and_extended_branch_would_have_been_wrong(tmp_path):
    """The stream CSV header both (a) satisfies is_stream_csv(), and (b)
    contains timestamp_us + quat_w -- i.e. it would ALSO match the legacy
    "Extended telemetry" test in visualize_extended.load_csv(). Reproduces
    the ORIGINAL bug (KeyError 'gyro_corrected_x' is wrong -- the real
    failure is on the first column the extended branch reads that a stream
    CSV lacks) to document why run_viz() must check is_stream_csv() first.
    stream CSV のヘッダは (a) is_stream_csv() を満たし、かつ (b) timestamp_us +
    quat_w を含む -- つまり visualize_extended.load_csv() の旧
    「Extended telemetry」判定にも一致してしまう。元のバグを再現し、
    run_viz() が is_stream_csv() を先に判定すべき理由を示す。"""
    csv_path = _build_stream_csv(tmp_path)
    header = _read_header(csv_path)

    assert visualize_stream.is_stream_csv(header) is True
    assert 'timestamp_us' in header and 'quat_w' in header

    data, fmt = visualize_extended.load_csv(str(csv_path))
    assert fmt == 'extended'  # matches the legacy branch's own detection
    try:
        visualize_extended.plot_extended(data, output_file=None)
        raise AssertionError(
            "plot_extended() unexpectedly succeeded on a stream CSV -- the "
            "legacy branch should not be able to render this format")
    except KeyError:
        pass  # expected: stream CSV lacks the extended-format columns
    finally:
        import matplotlib.pyplot as plt
        plt.close('all')


def test_visualize_all_default_mode(tmp_path):
    df = visualize_stream.load_stream_csv(str(_build_stream_csv(tmp_path)))
    out_png = tmp_path / "viz_all.png"
    visualize_stream.visualize_all(df, "stream.csv", save_path=str(out_png), show=False)

    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_visualize_all_attitude_mode(tmp_path):
    df = visualize_stream.load_stream_csv(str(_build_stream_csv(tmp_path)))
    out_png = tmp_path / "viz_attitude.png"
    visualize_stream.visualize_all(df, "stream.csv", save_path=str(out_png),
                                   show=False, mode='attitude')

    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_visualize_all_time_range(tmp_path):
    df = visualize_stream.load_stream_csv(str(_build_stream_csv(tmp_path)))
    out_png = tmp_path / "viz_tr.png"
    visualize_stream.visualize_all(df, "stream.csv", save_path=str(out_png),
                                   show=False, time_range=(0.5, 1.5))

    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_legacy_extended_header_not_detected_as_stream():
    """A 45-column legacy "extended" header (gyro_raw_x, pos_x, ctrl_throttle,
    no rate_ref_roll) must NOT be detected as a Data Stream CSV.
    45列の旧「extended」ヘッダ（gyro_raw_x, pos_x, ctrl_throttle を含み
    rate_ref_roll を含まない）は Data Stream CSV として検出されてはならない。"""
    legacy_header = [
        'timestamp_us', 'timestamp_ms',
        'gyro_x', 'gyro_y', 'gyro_z',
        'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z',
        'accel_x', 'accel_y', 'accel_z',
        'accel_raw_x', 'accel_raw_y', 'accel_raw_z',
        'quat_w', 'quat_x', 'quat_y', 'quat_z',
        'pos_x', 'pos_y', 'pos_z',
        'vel_x', 'vel_y', 'vel_z',
        'baro_alt', 'tof_bottom', 'tof_front',
        'flow_x', 'flow_y',
        'mag_x', 'mag_y', 'mag_z',
        'ctrl_throttle', 'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
        'battery_voltage', 'battery_current',
        'motor_1', 'motor_2', 'motor_3', 'motor_4',
        'flight_state', 'eskf_status', 'loop_time_us',
    ]
    assert len(legacy_header) == 45
    assert 'rate_ref_roll' not in legacy_header
    assert visualize_stream.is_stream_csv(legacy_header) is False


def _run_all():
    """Standalone runner using a throwaway tmp dir (mirrors pytest's tmp_path
    fixture for `python3 test_visualize_stream.py` execution).
    使い捨ての一時ディレクトリで実行するスタンドアロンランナー
    （`python3 test_visualize_stream.py` 実行時、pytest の tmp_path を模す）。"""
    import shutil
    import tempfile

    tmp_dir = Path(tempfile.mkdtemp(prefix="test_visualize_stream_"))
    tests = [
        (test_is_stream_csv_true_and_extended_branch_would_have_been_wrong, (tmp_dir,)),
        (test_visualize_all_default_mode, (tmp_dir,)),
        (test_visualize_all_attitude_mode, (tmp_dir,)),
        (test_visualize_all_time_range, (tmp_dir,)),
        (test_legacy_extended_header_not_detected_as_stream, ()),
    ]
    failures = 0
    try:
        for fn, fn_args in tests:
            try:
                fn(*fn_args)
                print(f"  [TEST] {fn.__name__:<55} PASS")
            except AssertionError as e:
                failures += 1
                print(f"  [TEST] {fn.__name__:<55} FAIL: {e}")
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    total = len(tests)
    print(f"\n=== Results: {total - failures}/{total} passed, {failures} failed ===")
    return failures


if __name__ == '__main__':
    sys.exit(1 if _run_all() > 0 else 0)
