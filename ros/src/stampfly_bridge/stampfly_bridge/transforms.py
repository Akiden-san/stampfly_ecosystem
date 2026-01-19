"""
transforms.py - NED to ENU coordinate transformations

StampFly uses NED (North-East-Down) frame.
ROS uses ENU (East-North-Up) frame.

NED → ENU transformation:
- Position: (x_enu, y_enu, z_enu) = (y_ned, x_ned, -z_ned)
- Velocity: same as position
- Quaternion: needs rotation around axis

References:
- REP-103: Standard Units of Measure and Coordinate Conventions
  https://www.ros.org/reps/rep-0103.html
"""

from typing import Tuple
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    Point, Quaternion, Vector3,
    PoseStamped, TwistStamped, TransformStamped
)
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Header

from stampfly_msgs.msg import (
    ImuRaw, ImuCorrected, ESKFState, ControlInput,
    RangeSensors, OpticalFlow
)

from .packet_parser import ExtendedSample


def ned_to_enu_position(x_ned: float, y_ned: float, z_ned: float) -> Tuple[float, float, float]:
    """Convert position from NED to ENU frame.

    NED (North-East-Down) → ENU (East-North-Up)
    - x_enu = y_ned (East)
    - y_enu = x_ned (North)
    - z_enu = -z_ned (Up)

    Args:
        x_ned: North position [m]
        y_ned: East position [m]
        z_ned: Down position [m]

    Returns:
        (x_enu, y_enu, z_enu): ENU coordinates
    """
    return (y_ned, x_ned, -z_ned)


def ned_to_enu_velocity(vx_ned: float, vy_ned: float, vz_ned: float) -> Tuple[float, float, float]:
    """Convert velocity from NED to ENU frame.

    Same transformation as position.
    """
    return (vy_ned, vx_ned, -vz_ned)


def ned_to_enu_quaternion(
    qw_ned: float, qx_ned: float, qy_ned: float, qz_ned: float
) -> Tuple[float, float, float, float]:
    """Convert quaternion from NED to ENU frame.

    The frame transformation from NED to ENU is a 180° rotation around
    the axis (1, 1, 0) / sqrt(2), which can be expressed as quaternion:
    q_rot = (0, sqrt(2)/2, sqrt(2)/2, 0)

    For body frame transformation, we need:
    q_enu = q_rot * q_ned * q_rot^(-1)

    However, for the common case of aircraft/drone attitude,
    a simpler transformation is often used:
    - Swap x and y components
    - Negate z component

    Args:
        qw_ned: Quaternion w (scalar)
        qx_ned: Quaternion x
        qy_ned: Quaternion y
        qz_ned: Quaternion z

    Returns:
        (qw_enu, qx_enu, qy_enu, qz_enu): ENU quaternion
    """
    # Frame rotation transformation
    # NED to ENU: 90° yaw + 180° roll equivalent
    # q_enu = q_ned with x↔y swap and z negated
    return (qw_ned, qy_ned, qx_ned, -qz_ned)


def ned_to_enu_angular_velocity(
    wx_ned: float, wy_ned: float, wz_ned: float
) -> Tuple[float, float, float]:
    """Convert angular velocity from NED to ENU frame.

    Body-frame angular velocity follows the same swap pattern.
    """
    return (wy_ned, wx_ned, -wz_ned)


def ned_to_enu_acceleration(
    ax_ned: float, ay_ned: float, az_ned: float
) -> Tuple[float, float, float]:
    """Convert linear acceleration from NED to ENU frame.

    Same transformation as position/velocity.
    """
    return (ay_ned, ax_ned, -az_ned)


def create_header(stamp: Time, frame_id: str) -> Header:
    """Create ROS2 Header message."""
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def timestamp_us_to_ros_time(timestamp_us: int) -> Time:
    """Convert microsecond timestamp to ROS Time.

    Args:
        timestamp_us: Microseconds since boot

    Returns:
        ROS Time message
    """
    time = Time()
    time.sec = timestamp_us // 1_000_000
    time.nanosec = (timestamp_us % 1_000_000) * 1000
    return time


def sample_to_imu_raw(sample: ExtendedSample, frame_id: str = "imu_link") -> ImuRaw:
    """Convert ExtendedSample to ImuRaw message.

    Raw IMU data in body frame (no NED→ENU conversion needed for body frame).
    """
    msg = ImuRaw()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    # Body-frame angular velocity (no frame conversion needed)
    msg.angular_velocity.x = sample.gyro_x
    msg.angular_velocity.y = sample.gyro_y
    msg.angular_velocity.z = sample.gyro_z

    # Body-frame acceleration
    msg.linear_acceleration.x = sample.accel_x
    msg.linear_acceleration.y = sample.accel_y
    msg.linear_acceleration.z = sample.accel_z

    return msg


def sample_to_imu_corrected(sample: ExtendedSample, frame_id: str = "imu_link") -> ImuCorrected:
    """Convert ExtendedSample to ImuCorrected message.

    Bias-corrected IMU data in body frame.
    """
    msg = ImuCorrected()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    msg.angular_velocity.x = sample.gyro_corrected_x
    msg.angular_velocity.y = sample.gyro_corrected_y
    msg.angular_velocity.z = sample.gyro_corrected_z

    # Note: We don't have bias-corrected accel in the packet,
    # so we use raw accel minus accel bias
    msg.linear_acceleration.x = sample.accel_x - sample.accel_bias_x
    msg.linear_acceleration.y = sample.accel_y - sample.accel_bias_y
    msg.linear_acceleration.z = sample.accel_z - sample.accel_bias_z

    return msg


def sample_to_eskf_state(sample: ExtendedSample, frame_id: str = "odom") -> ESKFState:
    """Convert ExtendedSample to ESKFState message.

    Converts from NED to ENU frame.
    """
    msg = ESKFState()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    # Quaternion: NED → ENU
    qw, qx, qy, qz = ned_to_enu_quaternion(
        sample.quat_w, sample.quat_x, sample.quat_y, sample.quat_z
    )
    msg.orientation.w = qw
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz

    # Position: NED → ENU
    x, y, z = ned_to_enu_position(sample.pos_x, sample.pos_y, sample.pos_z)
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z

    # Velocity: NED → ENU
    vx, vy, vz = ned_to_enu_velocity(sample.vel_x, sample.vel_y, sample.vel_z)
    msg.velocity.x = vx
    msg.velocity.y = vy
    msg.velocity.z = vz

    # Biases (body frame, no conversion)
    msg.gyro_bias.x = sample.gyro_bias_x
    msg.gyro_bias.y = sample.gyro_bias_y
    msg.gyro_bias.z = sample.gyro_bias_z

    msg.accel_bias.x = sample.accel_bias_x
    msg.accel_bias.y = sample.accel_bias_y
    msg.accel_bias.z = sample.accel_bias_z

    msg.status = sample.eskf_status

    return msg


def sample_to_control_input(sample: ExtendedSample, frame_id: str = "base_link") -> ControlInput:
    """Convert ExtendedSample to ControlInput message."""
    msg = ControlInput()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    msg.throttle = sample.ctrl_throttle
    msg.roll = sample.ctrl_roll
    msg.pitch = sample.ctrl_pitch
    msg.yaw = sample.ctrl_yaw

    return msg


def sample_to_range_sensors(sample: ExtendedSample, frame_id: str = "base_link") -> RangeSensors:
    """Convert ExtendedSample to RangeSensors message."""
    msg = RangeSensors()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    msg.tof_bottom = sample.tof_bottom
    msg.tof_front = sample.tof_front
    msg.baro_altitude = sample.baro_altitude

    return msg


def sample_to_optical_flow(sample: ExtendedSample, frame_id: str = "flow_link") -> OpticalFlow:
    """Convert ExtendedSample to OpticalFlow message."""
    msg = OpticalFlow()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    msg.flow_x = sample.flow_x
    msg.flow_y = sample.flow_y
    msg.quality = sample.flow_quality

    return msg


def sample_to_pose_stamped(sample: ExtendedSample, frame_id: str = "odom") -> PoseStamped:
    """Convert ExtendedSample to geometry_msgs/PoseStamped.

    Standard ROS pose in ENU frame.
    """
    msg = PoseStamped()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    # Position: NED → ENU
    x, y, z = ned_to_enu_position(sample.pos_x, sample.pos_y, sample.pos_z)
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    # Orientation: NED → ENU
    qw, qx, qy, qz = ned_to_enu_quaternion(
        sample.quat_w, sample.quat_x, sample.quat_y, sample.quat_z
    )
    msg.pose.orientation.w = qw
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz

    return msg


def sample_to_twist_stamped(sample: ExtendedSample, frame_id: str = "base_link") -> TwistStamped:
    """Convert ExtendedSample to geometry_msgs/TwistStamped.

    Linear velocity in ENU frame, angular velocity in body frame.
    """
    msg = TwistStamped()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    # Linear velocity: NED → ENU
    vx, vy, vz = ned_to_enu_velocity(sample.vel_x, sample.vel_y, sample.vel_z)
    msg.twist.linear.x = vx
    msg.twist.linear.y = vy
    msg.twist.linear.z = vz

    # Angular velocity (body frame, bias-corrected)
    msg.twist.angular.x = sample.gyro_corrected_x
    msg.twist.angular.y = sample.gyro_corrected_y
    msg.twist.angular.z = sample.gyro_corrected_z

    return msg


def sample_to_imu(sample: ExtendedSample, frame_id: str = "imu_link") -> Imu:
    """Convert ExtendedSample to sensor_msgs/Imu.

    Standard ROS IMU message with orientation, angular velocity, and acceleration.
    """
    msg = Imu()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    # Orientation (NED → ENU)
    qw, qx, qy, qz = ned_to_enu_quaternion(
        sample.quat_w, sample.quat_x, sample.quat_y, sample.quat_z
    )
    msg.orientation.w = qw
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz

    # Angular velocity (body frame, bias-corrected)
    msg.angular_velocity.x = sample.gyro_corrected_x
    msg.angular_velocity.y = sample.gyro_corrected_y
    msg.angular_velocity.z = sample.gyro_corrected_z

    # Linear acceleration (body frame, raw)
    msg.linear_acceleration.x = sample.accel_x
    msg.linear_acceleration.y = sample.accel_y
    msg.linear_acceleration.z = sample.accel_z

    # Covariance (unknown, set to -1 for first element)
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity_covariance[0] = -1.0
    msg.linear_acceleration_covariance[0] = -1.0

    return msg


def sample_to_range_bottom(sample: ExtendedSample, frame_id: str = "tof_bottom_link") -> Range:
    """Convert ExtendedSample to sensor_msgs/Range for bottom ToF."""
    msg = Range()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    msg.radiation_type = Range.INFRARED
    msg.field_of_view = 0.44  # ~25° typical for VL53L1X
    msg.min_range = 0.04     # 40mm min
    msg.max_range = 4.0      # 4m max
    msg.range = sample.tof_bottom

    return msg


def sample_to_range_front(sample: ExtendedSample, frame_id: str = "tof_front_link") -> Range:
    """Convert ExtendedSample to sensor_msgs/Range for front ToF."""
    msg = Range()
    msg.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), frame_id)

    msg.radiation_type = Range.INFRARED
    msg.field_of_view = 0.44
    msg.min_range = 0.04
    msg.max_range = 4.0
    msg.range = sample.tof_front

    return msg


def sample_to_transform(
    sample: ExtendedSample,
    parent_frame: str = "odom",
    child_frame: str = "base_link"
) -> TransformStamped:
    """Convert ExtendedSample to TransformStamped for TF2.

    Creates transform from odom frame to base_link frame.
    """
    tf = TransformStamped()
    tf.header = create_header(timestamp_us_to_ros_time(sample.timestamp_us), parent_frame)
    tf.child_frame_id = child_frame

    # Translation: NED → ENU
    x, y, z = ned_to_enu_position(sample.pos_x, sample.pos_y, sample.pos_z)
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z

    # Rotation: NED → ENU
    qw, qx, qy, qz = ned_to_enu_quaternion(
        sample.quat_w, sample.quat_x, sample.quat_y, sample.quat_z
    )
    tf.transform.rotation.w = qw
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz

    return tf
