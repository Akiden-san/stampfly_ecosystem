"""
Telemetry packet parser for StampFly WebSocket data

Parses extended batch packets (0xBD, 552 bytes) from WiFi WebSocket telemetry.
Extracted from tools/log_analyzer/wifi_capture.py with minimal dependencies.

StampFly WiFi WebSocket テレメトリデータのパケットパーサー
"""

import struct

# =============================================================================
# Extended Batch Packet (552 bytes, header 0xBD) - 400Hz unified telemetry
# =============================================================================
# Contains 4 samples of 136 bytes each with ESKF estimates and sensor data
# ESKF推定値とセンサデータを含む136バイト×4サンプル

EXTENDED_SAMPLE_SIZE = 136
EXTENDED_BATCH_SIZE = 4
EXTENDED_BATCH_HEADER_SIZE = 4  # header, type, count, reserved
EXTENDED_BATCH_FOOTER_SIZE = 4  # checksum + padding
EXTENDED_BATCH_PACKET_SIZE = 552  # 4 + 136*4 + 4

# ExtendedSample structure (136 bytes)
EXTENDED_SAMPLE_FORMAT = '<'
EXTENDED_SAMPLE_FORMAT += 'I'       # timestamp_us
EXTENDED_SAMPLE_FORMAT += 'fff'     # gyro_x/y/z (raw)
EXTENDED_SAMPLE_FORMAT += 'fff'     # accel_x/y/z (raw)
EXTENDED_SAMPLE_FORMAT += 'fff'     # gyro_corrected_x/y/z
EXTENDED_SAMPLE_FORMAT += 'ffff'    # ctrl_throttle/roll/pitch/yaw
EXTENDED_SAMPLE_FORMAT += 'ffff'    # quat_w/x/y/z
EXTENDED_SAMPLE_FORMAT += 'fff'     # pos_x/y/z
EXTENDED_SAMPLE_FORMAT += 'fff'     # vel_x/y/z
EXTENDED_SAMPLE_FORMAT += 'hhh'     # gyro_bias_x/y/z (int16)
EXTENDED_SAMPLE_FORMAT += 'hhh'     # accel_bias_x/y/z (int16)
EXTENDED_SAMPLE_FORMAT += 'B'       # eskf_status
EXTENDED_SAMPLE_FORMAT += '7B'      # padding1
EXTENDED_SAMPLE_FORMAT += 'f'       # baro_altitude
EXTENDED_SAMPLE_FORMAT += 'ff'      # tof_bottom/front
EXTENDED_SAMPLE_FORMAT += 'hh'      # flow_x/y
EXTENDED_SAMPLE_FORMAT += 'B'       # flow_quality
EXTENDED_SAMPLE_FORMAT += '3B'      # padding2


def parse_extended_batch_packet(data: bytes) -> list:
    """Parse extended batch packet (552 bytes, header 0xBD)

    Returns list of 4 sample dicts with ESKF estimates and sensor data,
    or None on parse failure.

    拡張バッチパケット(552バイト, ヘッダ0xBD)をパースする
    """
    if len(data) != EXTENDED_BATCH_PACKET_SIZE:
        return None

    if data[0] != 0xBD:
        return None

    # Verify checksum (XOR of bytes before checksum field)
    # チェックサム検証
    checksum_offset = EXTENDED_BATCH_HEADER_SIZE + EXTENDED_BATCH_SIZE * EXTENDED_SAMPLE_SIZE
    checksum = 0
    for i in range(checksum_offset):
        checksum ^= data[i]

    if checksum != data[checksum_offset]:
        return None

    # Parse samples
    # サンプルをパース
    samples = []
    for i in range(EXTENDED_BATCH_SIZE):
        offset = EXTENDED_BATCH_HEADER_SIZE + i * EXTENDED_SAMPLE_SIZE
        sample_data = data[offset:offset + EXTENDED_SAMPLE_SIZE]
        values = struct.unpack(EXTENDED_SAMPLE_FORMAT, sample_data)

        idx = 0
        sample = {'timestamp_us': values[idx]}
        idx += 1

        sample['gyro_x'] = values[idx]
        sample['gyro_y'] = values[idx + 1]
        sample['gyro_z'] = values[idx + 2]
        idx += 3

        sample['accel_x'] = values[idx]
        sample['accel_y'] = values[idx + 1]
        sample['accel_z'] = values[idx + 2]
        idx += 3

        sample['gyro_corrected_x'] = values[idx]
        sample['gyro_corrected_y'] = values[idx + 1]
        sample['gyro_corrected_z'] = values[idx + 2]
        idx += 3

        sample['ctrl_throttle'] = values[idx]
        sample['ctrl_roll'] = values[idx + 1]
        sample['ctrl_pitch'] = values[idx + 2]
        sample['ctrl_yaw'] = values[idx + 3]
        idx += 4

        sample['quat_w'] = values[idx]
        sample['quat_x'] = values[idx + 1]
        sample['quat_y'] = values[idx + 2]
        sample['quat_z'] = values[idx + 3]
        idx += 4

        sample['pos_x'] = values[idx]
        sample['pos_y'] = values[idx + 1]
        sample['pos_z'] = values[idx + 2]
        idx += 3

        sample['vel_x'] = values[idx]
        sample['vel_y'] = values[idx + 1]
        sample['vel_z'] = values[idx + 2]
        idx += 3

        # Gyro/accel bias (int16, scaled by 10000)
        sample['gyro_bias_x'] = values[idx] / 10000.0
        sample['gyro_bias_y'] = values[idx + 1] / 10000.0
        sample['gyro_bias_z'] = values[idx + 2] / 10000.0
        idx += 3
        sample['accel_bias_x'] = values[idx] / 10000.0
        sample['accel_bias_y'] = values[idx + 1] / 10000.0
        sample['accel_bias_z'] = values[idx + 2] / 10000.0
        idx += 3

        sample['eskf_status'] = values[idx]
        idx += 8  # eskf_status + 7 padding bytes

        sample['baro_altitude'] = values[idx]
        idx += 1

        sample['tof_bottom'] = values[idx]
        sample['tof_front'] = values[idx + 1]
        idx += 2

        sample['flow_x'] = values[idx]
        sample['flow_y'] = values[idx + 1]
        sample['flow_quality'] = values[idx + 2]

        samples.append(sample)

    return samples


def parse_packet(data: bytes) -> tuple:
    """Parse packet, auto-detecting format from header byte.

    Currently supports extended batch (0xBD) only.
    Returns (list_of_samples, mode_string) or (None, None) on failure.

    ヘッダバイトからフォーマットを自動検出してパケットをパースする
    """
    if len(data) == 0:
        return None, None

    header = data[0]

    if header == 0xBD and len(data) == EXTENDED_BATCH_PACKET_SIZE:
        samples = parse_extended_batch_packet(data)
        if samples:
            return samples, "extended"

    return None, None
