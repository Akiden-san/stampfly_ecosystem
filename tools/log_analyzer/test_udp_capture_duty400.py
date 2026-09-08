#!/usr/bin/env python3
"""
test_udp_capture_duty400.py - Host-side decode test for the 0x4A motor-duty
entry (kPktDuty400), the 400Hz plant-input record for `sf sysid fit`.

Builds unified packets (0x50) by hand -- mirrors UnifiedPacketBuilder in
data_stream_wire.hpp -- WITH and WITHOUT the 0x4A duty entry, and feeds them
to udp_capture.parse_packet(), so a firmware-side wire layout change is
caught here BEFORE it reaches real hardware. Also proves the forward/
backward compatibility contract (an unknown entry id is skipped via the
[id][size] framing alone) and save_stream_csv()'s source-selection: 400Hz
duty when present, 50Hz CtrlRef forward-fill when absent -- same 28-column
schema either way.
0x4A モータduty エントリ（kPktDuty400、`sf sysid fit` の400Hzプラント入力
記録）のホスト側デコードテスト。

統合パケット（0x50）を手組みし（data_stream_wire.hpp の
UnifiedPacketBuilder を模す）、0x4A エントリの有無それぞれで
udp_capture.parse_packet() に投入し、ファーム側の電文レイアウト変更を
実機の前にここで検出する。前方/後方互換の契約（未知のエントリIDは
[id][size] 枠組みだけでスキップされる）と、save_stream_csv() の
ソース選択（400Hz duty があればそちら、無ければ50Hz CtrlRef前方補完、
列構成は同じ28列）も検証する。

Usage:
    python3 test_udp_capture_duty400.py
    pytest test_udp_capture_duty400.py
"""

import csv
import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402

PKT_UNIFIED = 0x50
N = 8  # kSamplesPerPacket

FMT_IMU_ESKF = udp_capture.FMT_IMU_ESKF
FMT_POS_VEL = udp_capture.FMT_POS_VEL
FMT_RATE_REF = udp_capture.FMT_RATE_REF


def _xor_checksum(data: bytes) -> int:
    """Mirror xorChecksum() in data_stream_wire.hpp."""
    x = 0
    for b in data:
        x ^= b
    return x


def _imu_bytes(ts):
    """80B WireImuEskf; only the timestamp varies, rest is filler."""
    gyro = accel = gyro_raw = accel_raw = (0.0, 0.0, 0.0)
    quat = (1.0, 0.0, 0.0, 0.0)
    gyro_bias = accel_bias = (0, 0, 0)
    return struct.pack(FMT_IMU_ESKF, ts, *gyro, *accel, *gyro_raw, *accel_raw,
                        *quat, *gyro_bias, *accel_bias)


def _posvel_bytes(ts):
    """28B WirePosVel; all-zero pos/vel filler."""
    return struct.pack(FMT_POS_VEL, ts, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _rateref_bytes():
    """6B WireRateRef; zero rate_ref filler."""
    return struct.pack(FMT_RATE_REF, 0, 0, 0)


def build_unified(seq, base_ts_us=1_000_000, dt_us=2500, entries=b'', entry_count=0):
    """Build a raw 0x50 datagram: header + 8xImuEskf + 8xPosVel + 8xRateRef +
    entry_count + entries + checksum -- mirrors UnifiedPacketBuilder::begin()/
    addEntry()/finish() in data_stream_wire.hpp. Returns (datagram, imu_timestamps).
    """
    header = struct.pack('<BHB', PKT_UNIFIED, seq, N)
    imu_ts = [base_ts_us + i * dt_us for i in range(N)]
    body = header
    for ts in imu_ts:
        body += _imu_bytes(ts)
    for ts in imu_ts:
        body += _posvel_bytes(ts)
    for _ in range(N):
        body += _rateref_bytes()
    body += bytes([entry_count]) + entries
    return body + bytes([_xor_checksum(body)]), imu_ts


def _duty400_entry(duties):
    """[id][size][payload] for kPktDuty400. `duties` is 8 (fr,rr,rl,fl)
    tuples in duty units [0,1]."""
    payload = b''.join(
        struct.pack('<4H', *(round(v * 65535) for v in d)) for d in duties
    )
    assert len(payload) == 64
    return bytes([udp_capture.PKT_DUTY400, 64]) + payload


# =============================================================================
# parse_packet() decode tests
# =============================================================================

def test_duty400_entry_decodes_8_samples_paired_with_imu_timestamps():
    """8 sub-samples come back, each timestamped with the SAME-INDEX IMU
    sample's timestamp (like PKT_RATE_REF) -- the pairing `sf sysid fit`
    relies on to build a 400Hz-aligned CSV row."""
    duties = [(0.1 + 0.01 * i, 0.2, 0.3, 0.4) for i in range(N)]
    pkt, imu_ts = build_unified(1, entries=_duty400_entry(duties), entry_count=1)

    results = udp_capture.parse_packet(pkt)
    duty_samples = [s for pid, s in results if pid == udp_capture.PKT_DUTY400]
    assert len(duty_samples) == N

    for i, s in enumerate(duty_samples):
        assert s['timestamp_us'] == imu_ts[i]
        assert abs(s['duty_FR'] - duties[i][0]) < 1e-3
        assert abs(s['duty_RR'] - duties[i][1]) < 1e-3
        assert abs(s['duty_RL'] - duties[i][2]) < 1e-3
        assert abs(s['duty_FL'] - duties[i][3]) < 1e-3


def test_packet_without_duty400_entry_parses_fine_no_duty_samples():
    """A unified packet with NO 0x4A entry (old firmware) must still parse
    cleanly, with zero PKT_DUTY400 samples -- the fallback trigger for the
    50Hz CtrlRef forward-fill in save_stream_csv()."""
    pkt, _ = build_unified(2, entries=b'', entry_count=0)
    results = udp_capture.parse_packet(pkt)
    assert results  # IMU/PosVel/RateRef fixed blocks still present
    duty_samples = [s for pid, s in results if pid == udp_capture.PKT_DUTY400]
    assert duty_samples == []


def test_duty400_entry_does_not_corrupt_a_following_entry():
    """duty400, added FIRST by DataStream::appendEntries(), must not disturb
    offset tracking for entries appended after it."""
    control_entry = bytes([udp_capture.PKT_CONTROL, 20]) + \
        struct.pack('<I4f', 42, 0.5, 0.0, 0.0, 0.0)
    entries = _duty400_entry([(0.5, 0.5, 0.5, 0.5)] * N) + control_entry
    pkt, _ = build_unified(3, entries=entries, entry_count=2)

    results = udp_capture.parse_packet(pkt)
    ids = [pid for pid, _ in results]
    assert udp_capture.PKT_DUTY400 in ids
    assert udp_capture.PKT_CONTROL in ids
    ctrl = next(s for pid, s in results if pid == udp_capture.PKT_CONTROL)
    assert ctrl['timestamp_us'] == 42
    assert abs(ctrl['ctrl_throttle'] - 0.5) < 1e-6


def test_unknown_entry_id_is_skipped_without_corrupting_later_entries():
    """Compatibility note (a): a future/unknown entry id (not the 0x4A this
    parser knows, and not in SAMPLE_INFO) must be skipped by the [id][size]
    framing alone, without breaking a REAL entry that follows -- this is
    exactly the mechanism an OLD udp_capture.py relies on to safely ignore
    a NEW firmware's 0x4A duty entry."""
    fake_future_entry = bytes([0xEE, 4]) + b'\x01\x02\x03\x04'
    control_entry = bytes([udp_capture.PKT_CONTROL, 20]) + \
        struct.pack('<I4f', 99, 0.25, 0.0, 0.0, 0.0)
    entries = fake_future_entry + control_entry
    pkt, _ = build_unified(4, entries=entries, entry_count=2)

    results = udp_capture.parse_packet(pkt)
    ctrl = next(s for pid, s in results if pid == udp_capture.PKT_CONTROL)
    assert ctrl['timestamp_us'] == 99
    assert abs(ctrl['ctrl_throttle'] - 0.25) < 1e-6


# =============================================================================
# save_stream_csv() source-selection tests
# =============================================================================

def _seed_imu_and_rate_ref(cap, ts0, dt):
    for i in range(N):
        ts = ts0 + i * dt
        cap.samples[udp_capture.PKT_IMU_ESKF].append({
            'timestamp_us': ts,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': -9.81,
            'quat_w': 1.0, 'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0,
            'gyro_bias_x': 0.0, 'gyro_bias_y': 0.0, 'gyro_bias_z': 0.0,
            'accel_bias_x': 0.0, 'accel_bias_y': 0.0, 'accel_bias_z': 0.0,
        })


def test_save_stream_csv_uses_400hz_duty_when_present():
    """motor_duty_* must come from the 400Hz duty400 samples, NOT the 50Hz
    CtrlRef forward-fill, when both are present -- proven by giving them
    deliberately DIFFERENT values and checking which one wins."""
    cap = udp_capture.UDPTelemetryCapture()
    ts0, dt = 1_000_000, 2500
    _seed_imu_and_rate_ref(cap, ts0, dt)
    for i in range(N):
        ts = ts0 + i * dt
        cap.samples[udp_capture.PKT_DUTY400].append({
            'timestamp_us': ts,
            'duty_FR': 0.6, 'duty_RR': 0.6, 'duty_RL': 0.6, 'duty_FL': 0.6,
        })
    # 50Hz CtrlRef carries a DIFFERENT duty (0.1) -- must be overridden.
    cap.samples[udp_capture.PKT_CTRL_REF].append({
        'timestamp_us': ts0, 'flight_mode': 1, 'reserved': 0,
        'angle_ref_roll': 0, 'angle_ref_pitch': 0, 'total_thrust': 0.5,
        'motor_duty_FR': 0.1, 'motor_duty_RR': 0.1,
        'motor_duty_RL': 0.1, 'motor_duty_FL': 0.1,
    })

    with tempfile.TemporaryDirectory() as td:
        csv_path = Path(td) / "stream.csv"
        cap.save_stream_csv(str(csv_path))
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))

        assert len(rows) == N
        for row in rows:
            assert abs(float(row['motor_duty_FR']) - 0.6) < 1e-3
            # angle_ref/total_thrust/flight_mode are unaffected -- still 50Hz CtrlRef.
            assert abs(float(row['total_thrust']) - 0.5) < 1e-6
            # duty_rate_hz must say 400 -- this is what `sf sysid fit --input
            # auto` uses to trust the duty and skip the Kp reconstruction.
            assert int(row['duty_rate_hz']) == 400


def test_save_stream_csv_falls_back_to_50hz_ctrl_ref_when_no_duty400():
    """Without a duty400 entry (old firmware), motor_duty_* must come from
    the pre-existing 50Hz CtrlRef forward-fill, unchanged."""
    cap = udp_capture.UDPTelemetryCapture()
    ts0, dt = 1_000_000, 2500
    _seed_imu_and_rate_ref(cap, ts0, dt)
    cap.samples[udp_capture.PKT_CTRL_REF].append({
        'timestamp_us': ts0, 'flight_mode': 1, 'reserved': 0,
        'angle_ref_roll': 0, 'angle_ref_pitch': 0, 'total_thrust': 0.5,
        'motor_duty_FR': 0.3, 'motor_duty_RR': 0.3,
        'motor_duty_RL': 0.3, 'motor_duty_FL': 0.3,
    })

    with tempfile.TemporaryDirectory() as td:
        csv_path = Path(td) / "stream.csv"
        cap.save_stream_csv(str(csv_path))
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == N
        for row in rows:
            assert abs(float(row['motor_duty_FR']) - 0.3) < 1e-6
            # duty_rate_hz must say 50 -- `sf sysid fit --input auto` uses
            # this to refuse the (stale/quantized) duty and demand --kp.
            assert int(row['duty_rate_hz']) == 50


def _run_all():
    tests = [
        test_duty400_entry_decodes_8_samples_paired_with_imu_timestamps,
        test_packet_without_duty400_entry_parses_fine_no_duty_samples,
        test_duty400_entry_does_not_corrupt_a_following_entry,
        test_unknown_entry_id_is_skipped_without_corrupting_later_entries,
        test_save_stream_csv_uses_400hz_duty_when_present,
        test_save_stream_csv_falls_back_to_50hz_ctrl_ref_when_no_duty400,
    ]
    failures = 0
    for t in tests:
        try:
            t()
            print(f"  [TEST] {t.__name__:<60} PASS")
        except AssertionError as e:
            failures += 1
            print(f"  [TEST] {t.__name__:<60} FAIL: {e}")
    total = len(tests)
    print(f"\n=== Results: {total - failures}/{total} passed, {failures} failed ===")
    return failures


if __name__ == '__main__':
    sys.exit(1 if _run_all() > 0 else 0)
