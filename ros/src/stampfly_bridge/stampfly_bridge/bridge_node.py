"""
bridge_node.py - ROS2 node bridging StampFly WebSocket telemetry to ROS2 topics

Main node that connects to StampFly's WebSocket and publishes telemetry to ROS2.
Also supports bidirectional control via UDP.
StampFly WebSocket → ROS2 トピックのブリッジノード（双方向制御対応）

Topics published:
- /stampfly/imu/raw (stampfly_msgs/ImuRaw) - 400Hz
- /stampfly/imu/corrected (stampfly_msgs/ImuCorrected) - 400Hz
- /stampfly/eskf/state (stampfly_msgs/ESKFState) - 400Hz
- /stampfly/control/input (stampfly_msgs/ControlInput) - 400Hz
- /stampfly/range/sensors (stampfly_msgs/RangeSensors) - 400Hz
- /stampfly/flow (stampfly_msgs/OpticalFlow) - 400Hz
- /stampfly/pose (geometry_msgs/PoseStamped) - 400Hz
- /stampfly/velocity (geometry_msgs/TwistStamped) - 400Hz
- /stampfly/imu (sensor_msgs/Imu) - 400Hz
- /stampfly/range/bottom (sensor_msgs/Range) - 400Hz
- /stampfly/range/front (sensor_msgs/Range) - 400Hz

Topics subscribed (Phase 2 - Control):
- /stampfly/cmd_vel (geometry_msgs/Twist) - velocity command

Services:
- /stampfly/arm (std_srvs/SetBool) - ARM/DISARM

TF2 broadcasts:
- odom → base_link
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from sensor_msgs.msg import Imu, Range
from std_srvs.srv import SetBool

from stampfly_msgs.msg import (
    ImuRaw, ImuCorrected, ESKFState, ControlInput,
    RangeSensors, OpticalFlow
)

from .websocket_client import ThreadedWebSocketClient
from .udp_client import UDPControlClient, ControlInput as UDPControlInput
from .transforms import (
    sample_to_imu_raw, sample_to_imu_corrected, sample_to_eskf_state,
    sample_to_control_input, sample_to_range_sensors, sample_to_optical_flow,
    sample_to_pose_stamped, sample_to_twist_stamped, sample_to_imu,
    sample_to_range_bottom, sample_to_range_front, sample_to_transform,
)


class StampFlyBridgeNode(Node):
    """ROS2 Node for StampFly WebSocket telemetry bridge with control."""

    def __init__(self):
        super().__init__('stampfly_bridge')

        # Declare parameters
        # パラメータ宣言
        self.declare_parameter('host', '192.168.4.1')
        self.declare_parameter('port', 80)
        self.declare_parameter('path', '/ws')
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        # Control parameters (Phase 2)
        # 制御パラメータ
        self.declare_parameter('enable_control', False)
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('max_throttle', 0.8)   # Safety limit
        self.declare_parameter('max_roll_rate', 1.0)  # Normalized
        self.declare_parameter('max_pitch_rate', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)

        # Get parameters
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        path = self.get_parameter('path').get_parameter_value().string_value
        auto_reconnect = self.get_parameter('auto_reconnect').get_parameter_value().bool_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value

        # Control parameters
        self.enable_control = self.get_parameter('enable_control').get_parameter_value().bool_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.max_throttle = self.get_parameter('max_throttle').get_parameter_value().double_value
        self.max_roll_rate = self.get_parameter('max_roll_rate').get_parameter_value().double_value
        self.max_pitch_rate = self.get_parameter('max_pitch_rate').get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value

        # QoS profile for high-rate telemetry
        # 高頻度テレメトリ用QoSプロファイル
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Create publishers
        # パブリッシャ作成
        self.pub_imu_raw = self.create_publisher(
            ImuRaw, '/stampfly/imu/raw', sensor_qos)
        self.pub_imu_corrected = self.create_publisher(
            ImuCorrected, '/stampfly/imu/corrected', sensor_qos)
        self.pub_eskf_state = self.create_publisher(
            ESKFState, '/stampfly/eskf/state', sensor_qos)
        self.pub_control_input = self.create_publisher(
            ControlInput, '/stampfly/control/input', sensor_qos)
        self.pub_range_sensors = self.create_publisher(
            RangeSensors, '/stampfly/range/sensors', sensor_qos)
        self.pub_optical_flow = self.create_publisher(
            OpticalFlow, '/stampfly/flow', sensor_qos)

        # Standard ROS message publishers
        self.pub_pose = self.create_publisher(
            PoseStamped, '/stampfly/pose', sensor_qos)
        self.pub_velocity = self.create_publisher(
            TwistStamped, '/stampfly/velocity', sensor_qos)
        self.pub_imu = self.create_publisher(
            Imu, '/stampfly/imu', sensor_qos)
        self.pub_range_bottom = self.create_publisher(
            Range, '/stampfly/range/bottom', sensor_qos)
        self.pub_range_front = self.create_publisher(
            Range, '/stampfly/range/front', sensor_qos)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Create WebSocket client
        # WebSocketクライアント作成
        self.ws_client = ThreadedWebSocketClient(
            host=host,
            port=port,
            path=path,
            auto_reconnect=auto_reconnect,
        )

        # Statistics
        self.samples_published = 0
        self.last_stats_time = self.get_clock().now()

        # Timer for processing samples (100Hz = 10ms)
        # Timer runs faster than packet rate to minimize latency
        # サンプル処理タイマー（100Hz = 10ms）
        self.timer = self.create_timer(0.002, self.timer_callback)  # 500Hz polling

        # Status timer (1Hz)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # ====================================================================
        # Phase 2: Control (ROS2 → StampFly)
        # ====================================================================
        self.armed = False
        self.last_cmd_vel = Twist()
        self.udp_client = None

        if self.enable_control:
            # Create UDP client for control
            # 制御用UDPクライアント作成
            self.udp_client = UDPControlClient(host=host)
            self.udp_client.connect()

            # cmd_vel subscriber
            self.sub_cmd_vel = self.create_subscription(
                Twist,
                '/stampfly/cmd_vel',
                self.cmd_vel_callback,
                10
            )

            # ARM service
            self.srv_arm = self.create_service(
                SetBool,
                '/stampfly/arm',
                self.arm_callback
            )

            # Control send timer
            control_period = 1.0 / self.control_rate
            self.control_timer = self.create_timer(control_period, self.control_timer_callback)

            self.get_logger().info(f'Control enabled at {self.control_rate} Hz')

        self.get_logger().info(f'StampFly Bridge starting, connecting to {host}:{port}{path}')

        # Start WebSocket client
        self.ws_client.start()

    def timer_callback(self):
        """Process received samples and publish to ROS2."""
        samples = self.ws_client.get_all_samples()

        for sample in samples:
            self.publish_sample(sample)
            self.samples_published += 1

    def publish_sample(self, sample):
        """Publish a single sample to all topics."""
        # Custom StampFly messages
        self.pub_imu_raw.publish(
            sample_to_imu_raw(sample, self.imu_frame))
        self.pub_imu_corrected.publish(
            sample_to_imu_corrected(sample, self.imu_frame))
        self.pub_eskf_state.publish(
            sample_to_eskf_state(sample, self.odom_frame))
        self.pub_control_input.publish(
            sample_to_control_input(sample, self.base_frame))
        self.pub_range_sensors.publish(
            sample_to_range_sensors(sample, self.base_frame))
        self.pub_optical_flow.publish(
            sample_to_optical_flow(sample, self.base_frame))

        # Standard ROS messages
        self.pub_pose.publish(
            sample_to_pose_stamped(sample, self.odom_frame))
        self.pub_velocity.publish(
            sample_to_twist_stamped(sample, self.base_frame))
        self.pub_imu.publish(
            sample_to_imu(sample, self.imu_frame))
        self.pub_range_bottom.publish(
            sample_to_range_bottom(sample, f"{self.base_frame}_tof_bottom"))
        self.pub_range_front.publish(
            sample_to_range_front(sample, f"{self.base_frame}_tof_front"))

        # TF broadcast
        if self.publish_tf:
            tf = sample_to_transform(sample, self.odom_frame, self.base_frame)
            self.tf_broadcaster.sendTransform(tf)

    def status_callback(self):
        """Log connection status periodically."""
        stats = self.ws_client.stats
        now = self.get_clock().now()
        dt = (now - self.last_stats_time).nanoseconds / 1e9

        if dt > 0:
            rate = self.samples_published / dt
            self.get_logger().info(
                f'Connected: {self.ws_client.connected}, '
                f'Rate: {rate:.1f} Hz, '
                f'Samples: {stats.samples_received}, '
                f'Errors: {stats.checksum_errors}, '
                f'Queue: {self.ws_client.queue_size}'
            )

        self.samples_published = 0
        self.last_stats_time = now

    # ========================================================================
    # Phase 2: Control Callbacks
    # ========================================================================

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity command from ROS2.

        Twist mapping:
        - linear.z → throttle (0 to max_throttle)
        - angular.x → roll rate (-max_roll_rate to max_roll_rate)
        - angular.y → pitch rate
        - angular.z → yaw rate
        """
        self.last_cmd_vel = msg

    def arm_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle ARM/DISARM service request."""
        self.armed = request.data

        if self.armed:
            self.get_logger().warn('ARMED! Motors will spin when throttle is applied.')
            if self.udp_client:
                self.udp_client.arm()
        else:
            self.get_logger().info('DISARMED')
            if self.udp_client:
                self.udp_client.disarm()

        response.success = True
        response.message = 'Armed' if self.armed else 'Disarmed'
        return response

    def control_timer_callback(self):
        """Send control packet to StampFly at control_rate Hz."""
        if not self.udp_client:
            return

        # Convert Twist to control input
        # Twist → 制御入力変換
        cmd = self.last_cmd_vel

        # Clamp values to safety limits
        throttle = max(0.0, min(self.max_throttle, cmd.linear.z))
        roll = max(-self.max_roll_rate, min(self.max_roll_rate, cmd.angular.x))
        pitch = max(-self.max_pitch_rate, min(self.max_pitch_rate, cmd.angular.y))
        yaw = max(-self.max_yaw_rate, min(self.max_yaw_rate, cmd.angular.z))

        # Send control
        control = UDPControlInput(
            throttle=throttle,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            armed=self.armed
        )
        self.udp_client.send_control(control)

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down StampFly Bridge...')

        # Disarm before shutdown
        # シャットダウン前にDISARM
        if self.udp_client and self.armed:
            self.udp_client.disarm()
            self.get_logger().info('Disarmed on shutdown')

        # Close clients
        if self.udp_client:
            self.udp_client.disconnect()
        self.ws_client.stop()

        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = StampFlyBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
