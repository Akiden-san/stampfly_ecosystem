"""
udp_client.py - UDP client for sending control packets to StampFly

Sends ControlPacket (16 bytes) to StampFly's UDP server.
StampFlyのUDPサーバーに制御パケットを送信

Packet structure (from firmware/common/protocol/include/udp_protocol.hpp):
- header: 0xAA
- packet_type: 0x01
- sequence: uint8
- device_id: uint8
- throttle: uint16 [0-4095]
- roll: uint16 [0-4095], 2048=center
- pitch: uint16 [0-4095], 2048=center
- yaw: uint16 [0-4095], 2048=center
- flags: uint8 (bit0=ARM, bit1=FLIP, bit2=MODE, bit3=ALT_MODE)
- reserved: uint8
- checksum: CRC16-CCITT
"""

import socket
import struct
import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


# Constants from udp_protocol.hpp
CONTROL_PORT = 8888
PACKET_HEADER = 0xAA
PKT_TYPE_CONTROL = 0x01

# Control flags
CTRL_FLAG_ARM = (1 << 0)
CTRL_FLAG_FLIP = (1 << 1)
CTRL_FLAG_MODE = (1 << 2)
CTRL_FLAG_ALT_MODE = (1 << 3)

# Device IDs
DEVICE_ID_GCS = 0x01  # ROS2 bridge as GCS

# Stick center and range
STICK_CENTER = 2048
STICK_MAX = 4095
STICK_MIN = 0


def calculate_crc16(data: bytes) -> int:
    """Calculate CRC16-CCITT checksum.

    Same algorithm as in udp_protocol.hpp.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class ControlInput:
    """Control input values.

    Normalized values:
    - throttle: 0.0 to 1.0
    - roll/pitch/yaw: -1.0 to 1.0
    """
    throttle: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    armed: bool = False


def normalize_to_stick(value: float, is_throttle: bool = False) -> int:
    """Convert normalized value to stick value [0-4095].

    Args:
        value: Normalized value (-1.0 to 1.0, or 0.0 to 1.0 for throttle)
        is_throttle: True if this is throttle (0-1 range)

    Returns:
        Stick value [0-4095]
    """
    if is_throttle:
        # Throttle: 0.0-1.0 → 0-4095
        return int(max(0.0, min(1.0, value)) * STICK_MAX)
    else:
        # Roll/Pitch/Yaw: -1.0 to 1.0 → 0-4095 (2048=center)
        return int(STICK_CENTER + max(-1.0, min(1.0, value)) * STICK_CENTER)


def build_control_packet(
    sequence: int,
    device_id: int,
    throttle: int,
    roll: int,
    pitch: int,
    yaw: int,
    flags: int
) -> bytes:
    """Build a control packet with CRC16 checksum.

    Args:
        sequence: Sequence number (0-255)
        device_id: Sender device ID
        throttle: Throttle value [0-4095]
        roll: Roll value [0-4095], 2048=center
        pitch: Pitch value [0-4095], 2048=center
        yaw: Yaw value [0-4095], 2048=center
        flags: Control flags

    Returns:
        16-byte control packet
    """
    # Pack without checksum (14 bytes)
    packet_without_crc = struct.pack(
        '<BBBB HHHH BB',
        PACKET_HEADER,
        PKT_TYPE_CONTROL,
        sequence & 0xFF,
        device_id,
        throttle,
        roll,
        pitch,
        yaw,
        flags,
        0  # reserved
    )

    # Calculate CRC16
    crc = calculate_crc16(packet_without_crc)

    # Append CRC16
    packet = packet_without_crc + struct.pack('<H', crc)

    assert len(packet) == 16, f"Packet size mismatch: {len(packet)}"
    return packet


class UDPControlClient:
    """UDP client for sending control packets to StampFly.

    Usage:
        client = UDPControlClient(host="192.168.4.1")
        client.connect()

        # Send control
        client.send_control(ControlInput(throttle=0.5, roll=0.1, armed=True))

        # On shutdown
        client.disconnect()
    """

    DEFAULT_HOST = "192.168.4.1"
    DEFAULT_PORT = CONTROL_PORT

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        device_id: int = DEVICE_ID_GCS,
    ):
        """Initialize UDP client.

        Args:
            host: StampFly IP address
            port: UDP control port
            device_id: Device ID for this client
        """
        self.host = host
        self.port = port
        self.device_id = device_id

        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        self._connected = False

        # Statistics
        self.packets_sent = 0
        self.errors = 0

    @property
    def connected(self) -> bool:
        """Check if connected."""
        return self._connected

    def connect(self) -> bool:
        """Create UDP socket.

        Note: UDP is connectionless, this just creates the socket.
        """
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setblocking(False)
            self._connected = True
            logger.info(f"UDP client ready to send to {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to create UDP socket: {e}")
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Close UDP socket."""
        if self._socket:
            self._socket.close()
            self._socket = None
        self._connected = False
        logger.info("UDP client disconnected")

    def send_control(self, control: ControlInput) -> bool:
        """Send control packet to StampFly.

        Args:
            control: Control input values

        Returns:
            True if sent successfully
        """
        if not self._socket:
            return False

        # Convert normalized values to stick values
        throttle = normalize_to_stick(control.throttle, is_throttle=True)
        roll = normalize_to_stick(control.roll)
        pitch = normalize_to_stick(control.pitch)
        yaw = normalize_to_stick(control.yaw)

        # Build flags
        flags = CTRL_FLAG_ARM if control.armed else 0

        # Build packet
        packet = build_control_packet(
            sequence=self._sequence,
            device_id=self.device_id,
            throttle=throttle,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            flags=flags
        )

        try:
            self._socket.sendto(packet, (self.host, self.port))
            self._sequence = (self._sequence + 1) & 0xFF
            self.packets_sent += 1
            return True
        except Exception as e:
            logger.error(f"Failed to send control packet: {e}")
            self.errors += 1
            return False

    def send_raw(
        self,
        throttle: int,
        roll: int,
        pitch: int,
        yaw: int,
        flags: int = 0
    ) -> bool:
        """Send raw control packet with stick values.

        Args:
            throttle: Raw throttle [0-4095]
            roll: Raw roll [0-4095], 2048=center
            pitch: Raw pitch [0-4095], 2048=center
            yaw: Raw yaw [0-4095], 2048=center
            flags: Control flags

        Returns:
            True if sent successfully
        """
        if not self._socket:
            return False

        packet = build_control_packet(
            sequence=self._sequence,
            device_id=self.device_id,
            throttle=throttle,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            flags=flags
        )

        try:
            self._socket.sendto(packet, (self.host, self.port))
            self._sequence = (self._sequence + 1) & 0xFF
            self.packets_sent += 1
            return True
        except Exception as e:
            logger.error(f"Failed to send control packet: {e}")
            self.errors += 1
            return False

    def arm(self) -> bool:
        """Send ARM command (throttle=0)."""
        return self.send_raw(0, STICK_CENTER, STICK_CENTER, STICK_CENTER, CTRL_FLAG_ARM)

    def disarm(self) -> bool:
        """Send DISARM command."""
        return self.send_raw(0, STICK_CENTER, STICK_CENTER, STICK_CENTER, 0)
