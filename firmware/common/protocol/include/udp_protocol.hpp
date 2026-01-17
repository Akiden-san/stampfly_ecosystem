/**
 * @file udp_protocol.hpp
 * @brief UDP communication protocol definition for StampFly
 *        StampFly用UDP通信プロトコル定義
 *
 * This header defines the packet structures and constants used for
 * UDP communication between Controller and Vehicle.
 * Controller-Vehicle間のUDP通信で使用するパケット構造と定数を定義。
 *
 * @note This file is shared between Vehicle and Controller firmware.
 *       このファイルはVehicleとControllerのファームウェアで共有される。
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace stampfly {
namespace udp {

// ============================================================================
// Port Configuration
// ポート設定
// ============================================================================

/// Control packet port (Controller -> Vehicle)
/// 制御パケットポート（Controller → Vehicle）
constexpr uint16_t CONTROL_PORT = 8888;

/// Telemetry packet port (Vehicle -> Controller)
/// テレメトリパケットポート（Vehicle → Controller）
constexpr uint16_t TELEMETRY_PORT = 8889;

// ============================================================================
// Packet Header Constants
// パケットヘッダ定数
// ============================================================================

/// Packet header magic byte
/// パケットヘッダマジックバイト
constexpr uint8_t PACKET_HEADER = 0xAA;

/// Packet type: Control command
/// パケットタイプ: 制御コマンド
constexpr uint8_t PKT_TYPE_CONTROL = 0x01;

/// Packet type: Telemetry data
/// パケットタイプ: テレメトリデータ
constexpr uint8_t PKT_TYPE_TELEMETRY = 0x02;

/// Packet type: Heartbeat
/// パケットタイプ: ハートビート
constexpr uint8_t PKT_TYPE_HEARTBEAT = 0x10;

// ============================================================================
// Control Flags (same as ESP-NOW protocol)
// 制御フラグ（ESP-NOWプロトコルと同一）
// ============================================================================

/// ARM flag - Motor armed when set
/// ARMフラグ - セットでモーター有効
constexpr uint8_t CTRL_FLAG_ARM = (1 << 0);

/// FLIP flag - Flip mode trigger
/// FLIPフラグ - フリップモードトリガー
constexpr uint8_t CTRL_FLAG_FLIP = (1 << 1);

/// MODE flag - Flight mode change (STABILIZE/ACRO)
/// MODEフラグ - 飛行モード変更（STABILIZE/ACRO）
constexpr uint8_t CTRL_FLAG_MODE = (1 << 2);

/// ALT_MODE flag - Altitude mode change (Auto ALT/Manual ALT)
/// ALT_MODEフラグ - 高度モード変更（自動高度/手動高度）
constexpr uint8_t CTRL_FLAG_ALT_MODE = (1 << 3);

// ============================================================================
// Device IDs
// デバイスID
// ============================================================================

/// Device ID for Controller
/// ControllerのデバイスID
constexpr uint8_t DEVICE_ID_CONTROLLER = 0x00;

/// Device ID range for GCS/PC (1-255)
/// GCS/PCのデバイスID範囲（1-255）
constexpr uint8_t DEVICE_ID_GCS_MIN = 0x01;
constexpr uint8_t DEVICE_ID_GCS_MAX = 0xFF;

// ============================================================================
// Packet Structures
// パケット構造体
// ============================================================================

#pragma pack(push, 1)

/**
 * @brief Control packet structure (16 bytes)
 *        制御パケット構造体（16バイト）
 *
 * Sent from Controller to Vehicle at 50Hz.
 * Controllerから Vehicleへ50Hzで送信。
 */
struct ControlPacket {
    uint8_t header;           ///< Packet header (0xAA) / パケットヘッダ
    uint8_t packet_type;      ///< Packet type (PKT_TYPE_CONTROL) / パケットタイプ
    uint8_t sequence;         ///< Sequence number / シーケンス番号
    uint8_t device_id;        ///< Sender device ID / 送信元デバイスID

    uint16_t throttle;        ///< Throttle [0-4095] / スロットル
    uint16_t roll;            ///< Roll [0-4095], 2048=center / ロール、2048=中央
    uint16_t pitch;           ///< Pitch [0-4095], 2048=center / ピッチ、2048=中央
    uint16_t yaw;             ///< Yaw [0-4095], 2048=center / ヨー、2048=中央

    uint8_t flags;            ///< Control flags / 制御フラグ
    uint8_t reserved;         ///< Reserved for future use / 予約
    uint16_t checksum;        ///< CRC16 checksum / CRC16チェックサム
};

static_assert(sizeof(ControlPacket) == 16, "ControlPacket size must be 16 bytes");

/**
 * @brief Telemetry packet structure (20 bytes)
 *        テレメトリパケット構造体（20バイト）
 *
 * Sent from Vehicle to Controller at 50Hz.
 * VehicleからControllerへ50Hzで送信。
 */
struct TelemetryPacket {
    uint8_t header;           ///< Packet header (0xAA) / パケットヘッダ
    uint8_t packet_type;      ///< Packet type (PKT_TYPE_TELEMETRY) / パケットタイプ
    uint8_t sequence;         ///< Sequence number / シーケンス番号
    uint8_t flight_state;     ///< Flight state enum / 飛行状態

    uint16_t battery_mv;      ///< Battery voltage [mV] / バッテリー電圧
    int16_t roll_deg10;       ///< Roll angle [0.1 deg] / ロール角
    int16_t pitch_deg10;      ///< Pitch angle [0.1 deg] / ピッチ角
    int16_t yaw_deg10;        ///< Yaw angle [0.1 deg] / ヨー角

    int16_t altitude_cm;      ///< Altitude [cm] / 高度
    int16_t velocity_z_cms;   ///< Vertical velocity [cm/s] / 垂直速度

    uint8_t rssi;             ///< Received signal strength / 受信信号強度
    uint8_t flags;            ///< Status flags / ステータスフラグ
    uint16_t checksum;        ///< CRC16 checksum / CRC16チェックサム
};

static_assert(sizeof(TelemetryPacket) == 20, "TelemetryPacket size must be 20 bytes");

/**
 * @brief Heartbeat packet structure (8 bytes)
 *        ハートビートパケット構造体（8バイト）
 *
 * Used for connection keep-alive.
 * 接続維持に使用。
 */
struct HeartbeatPacket {
    uint8_t header;           ///< Packet header (0xAA) / パケットヘッダ
    uint8_t packet_type;      ///< Packet type (PKT_TYPE_HEARTBEAT) / パケットタイプ
    uint8_t sequence;         ///< Sequence number / シーケンス番号
    uint8_t device_id;        ///< Sender device ID / 送信元デバイスID
    uint32_t timestamp_ms;    ///< Timestamp in milliseconds / タイムスタンプ（ミリ秒）
};

static_assert(sizeof(HeartbeatPacket) == 8, "HeartbeatPacket size must be 8 bytes");

#pragma pack(pop)

// ============================================================================
// CRC16 Calculation
// CRC16計算
// ============================================================================

/**
 * @brief Calculate CRC16-CCITT checksum
 *        CRC16-CCITTチェックサムを計算
 *
 * @param data Pointer to data buffer / データバッファへのポインタ
 * @param len Length of data in bytes / データ長（バイト）
 * @return CRC16 checksum / CRC16チェックサム
 */
inline uint16_t calculateCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Packet Validation
// パケット検証
// ============================================================================

/**
 * @brief Validate control packet
 *        制御パケットを検証
 *
 * @param pkt Reference to control packet / 制御パケットへの参照
 * @return true if valid, false otherwise / 有効ならtrue
 */
inline bool validateControlPacket(const ControlPacket& pkt) {
    // Check header
    // ヘッダをチェック
    if (pkt.header != PACKET_HEADER) {
        return false;
    }

    // Check packet type
    // パケットタイプをチェック
    if (pkt.packet_type != PKT_TYPE_CONTROL) {
        return false;
    }

    // Calculate and verify checksum (exclude checksum field itself)
    // チェックサムを計算・検証（チェックサムフィールド自体は除外）
    uint16_t calculated = calculateCRC16(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - sizeof(uint16_t)
    );

    return pkt.checksum == calculated;
}

/**
 * @brief Validate telemetry packet
 *        テレメトリパケットを検証
 *
 * @param pkt Reference to telemetry packet / テレメトリパケットへの参照
 * @return true if valid, false otherwise / 有効ならtrue
 */
inline bool validateTelemetryPacket(const TelemetryPacket& pkt) {
    // Check header
    // ヘッダをチェック
    if (pkt.header != PACKET_HEADER) {
        return false;
    }

    // Check packet type
    // パケットタイプをチェック
    if (pkt.packet_type != PKT_TYPE_TELEMETRY) {
        return false;
    }

    // Calculate and verify checksum
    // チェックサムを計算・検証
    uint16_t calculated = calculateCRC16(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(TelemetryPacket) - sizeof(uint16_t)
    );

    return pkt.checksum == calculated;
}

// ============================================================================
// Packet Builders
// パケットビルダー
// ============================================================================

/**
 * @brief Build a control packet with checksum
 *        チェックサム付き制御パケットを構築
 *
 * @param sequence Sequence number / シーケンス番号
 * @param device_id Device ID / デバイスID
 * @param throttle Throttle value [0-4095] / スロットル値
 * @param roll Roll value [0-4095] / ロール値
 * @param pitch Pitch value [0-4095] / ピッチ値
 * @param yaw Yaw value [0-4095] / ヨー値
 * @param flags Control flags / 制御フラグ
 * @return Constructed control packet / 構築された制御パケット
 */
inline ControlPacket buildControlPacket(
    uint8_t sequence,
    uint8_t device_id,
    uint16_t throttle,
    uint16_t roll,
    uint16_t pitch,
    uint16_t yaw,
    uint8_t flags
) {
    ControlPacket pkt = {};
    pkt.header = PACKET_HEADER;
    pkt.packet_type = PKT_TYPE_CONTROL;
    pkt.sequence = sequence;
    pkt.device_id = device_id;
    pkt.throttle = throttle;
    pkt.roll = roll;
    pkt.pitch = pitch;
    pkt.yaw = yaw;
    pkt.flags = flags;
    pkt.reserved = 0;

    // Calculate checksum
    // チェックサムを計算
    pkt.checksum = calculateCRC16(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - sizeof(uint16_t)
    );

    return pkt;
}

/**
 * @brief Build a telemetry packet with checksum
 *        チェックサム付きテレメトリパケットを構築
 *
 * @param sequence Sequence number / シーケンス番号
 * @param flight_state Flight state / 飛行状態
 * @param battery_mv Battery voltage in mV / バッテリー電圧（mV）
 * @param roll_deg10 Roll angle in 0.1 degrees / ロール角（0.1度）
 * @param pitch_deg10 Pitch angle in 0.1 degrees / ピッチ角（0.1度）
 * @param yaw_deg10 Yaw angle in 0.1 degrees / ヨー角（0.1度）
 * @param altitude_cm Altitude in cm / 高度（cm）
 * @param velocity_z_cms Vertical velocity in cm/s / 垂直速度（cm/s）
 * @param rssi Received signal strength / 受信信号強度
 * @param flags Status flags / ステータスフラグ
 * @return Constructed telemetry packet / 構築されたテレメトリパケット
 */
inline TelemetryPacket buildTelemetryPacket(
    uint8_t sequence,
    uint8_t flight_state,
    uint16_t battery_mv,
    int16_t roll_deg10,
    int16_t pitch_deg10,
    int16_t yaw_deg10,
    int16_t altitude_cm,
    int16_t velocity_z_cms,
    uint8_t rssi,
    uint8_t flags
) {
    TelemetryPacket pkt = {};
    pkt.header = PACKET_HEADER;
    pkt.packet_type = PKT_TYPE_TELEMETRY;
    pkt.sequence = sequence;
    pkt.flight_state = flight_state;
    pkt.battery_mv = battery_mv;
    pkt.roll_deg10 = roll_deg10;
    pkt.pitch_deg10 = pitch_deg10;
    pkt.yaw_deg10 = yaw_deg10;
    pkt.altitude_cm = altitude_cm;
    pkt.velocity_z_cms = velocity_z_cms;
    pkt.rssi = rssi;
    pkt.flags = flags;

    // Calculate checksum
    // チェックサムを計算
    pkt.checksum = calculateCRC16(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(TelemetryPacket) - sizeof(uint16_t)
    );

    return pkt;
}

// ============================================================================
// Communication Mode
// 通信モード
// ============================================================================

/**
 * @brief Communication mode enumeration
 *        通信モード列挙型
 */
enum class CommMode : uint8_t {
    ESPNOW = 0,      ///< ESP-NOW direct (default) / ESP-NOW直接通信（デフォルト）
    UDP = 1,         ///< UDP over Vehicle's AP / Vehicle APを介したUDP
    USB_HID = 2,     ///< USB HID mode (Controller only) / USB HIDモード（Controllerのみ）
};

/// NVS key for storing communication mode
/// 通信モード保存用NVSキー
constexpr const char* NVS_KEY_COMM_MODE = "comm_mode";

// ============================================================================
// Network Configuration
// ネットワーク設定
// ============================================================================

/// Default Vehicle AP IP address
/// デフォルトのVehicle AP IPアドレス
constexpr const char* DEFAULT_VEHICLE_IP = "192.168.4.1";

/// Control packet timeout in milliseconds
/// 制御パケットタイムアウト（ミリ秒）
constexpr uint32_t CONTROL_TIMEOUT_MS = 500;

/// Telemetry send rate in Hz
/// テレメトリ送信レート（Hz）
constexpr uint32_t TELEMETRY_RATE_HZ = 50;

}  // namespace udp
}  // namespace stampfly
