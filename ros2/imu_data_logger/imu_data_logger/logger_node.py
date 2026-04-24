"""Log multiple robot telemetry topics for later analysis."""

import csv
from functools import partial
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu


def _stamp_to_seconds(stamp):
    """Convert a ROS timestamp message to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def _topic_to_file_stem(topic_name):
    """Convert a ROS topic name into a safe file stem."""
    return topic_name.strip('/').replace('/', '_') or 'topic'


def _imu_csv_header():
    """Return the CSV header used for IMU messages."""
    header = [
        'timestamp',
        'received_time',
        'frame_id',
        'orientation_x',
        'orientation_y',
        'orientation_z',
        'orientation_w',
        'angular_velocity_x',
        'angular_velocity_y',
        'angular_velocity_z',
        'linear_acceleration_x',
        'linear_acceleration_y',
        'linear_acceleration_z',
    ]

    for prefix in (
            'orientation_covariance',
            'angular_velocity_covariance',
            'linear_acceleration_covariance'):
        header.extend(f'{prefix}_{index}' for index in range(9))

    return header


class ImuLoggerNode(Node):
    """Record raw IMU, filtered IMU, and robot state topics."""

    def __init__(self):
        """Create subscriptions and prepare log files."""
        super().__init__('imu_logger')

        self.declare_parameter('output_dir', '.')
        self.declare_parameter('raw_imu_topic', '/imu/data')
        self.declare_parameter('filtered_imu_topic', '/imu/filtered')
        self.declare_parameter('robot_state_topic', '/robot/state')

        self.output_dir = Path(self._get_string_parameter('output_dir'))
        self.raw_imu_topic = self._get_string_parameter('raw_imu_topic')
        self.filtered_imu_topic = self._get_string_parameter(
            'filtered_imu_topic')
        self.robot_state_topic = self._get_string_parameter(
            'robot_state_topic')

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.raw_imu_csv = self._imu_csv_path(self.raw_imu_topic)
        self.filtered_imu_csv = self._imu_csv_path(self.filtered_imu_topic)
        self.robot_state_jsonl = self.output_dir / 'robot_state.jsonl'

        self._ensure_csv_file(self.raw_imu_csv, _imu_csv_header())
        self._ensure_csv_file(self.filtered_imu_csv, _imu_csv_header())

        self._subscriptions = [
            self.create_subscription(
                Imu,
                self.raw_imu_topic,
                partial(self._write_imu_message, self.raw_imu_csv),
                qos_profile_sensor_data),
            self.create_subscription(
                Imu,
                self.filtered_imu_topic,
                partial(self._write_imu_message, self.filtered_imu_csv),
                qos_profile_sensor_data),
        ]

        self._robot_state_subscription = None
        self._robot_state_wait_logged = False
        self._robot_state_probe_timer = self.create_timer(
            1.0,
            self._try_subscribe_robot_state)

        self.get_logger().info(
            f'Logging {self.raw_imu_topic} to {self.raw_imu_csv}')
        self.get_logger().info(
            f'Logging {self.filtered_imu_topic} to {self.filtered_imu_csv}')
        self.get_logger().info(
            f'Waiting to discover {self.robot_state_topic} message type')

    def _get_string_parameter(self, name):
        """Read a declared string parameter."""
        return self.get_parameter(name).get_parameter_value().string_value

    def _imu_csv_path(self, topic_name):
        """Build the output CSV path for an IMU topic."""
        file_stem = _topic_to_file_stem(topic_name)
        return self.output_dir / f'{file_stem}.csv'

    def _ensure_csv_file(self, csv_path, header):
        """Create a CSV file with a header if it does not already exist."""
        if csv_path.is_file():
            return

        with csv_path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(header)

    def _write_imu_message(self, csv_path, msg):
        """Append one IMU message to a CSV file."""
        timestamp = _stamp_to_seconds(msg.header.stamp)
        received_time = self.get_clock().now().nanoseconds * 1e-9

        row = [
            timestamp,
            received_time,
            msg.header.frame_id,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        row.extend(msg.orientation_covariance)
        row.extend(msg.angular_velocity_covariance)
        row.extend(msg.linear_acceleration_covariance)

        with csv_path.open('a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(row)

    def _try_subscribe_robot_state(self):
        """Discover and subscribe to the robot state topic dynamically."""
        if self._robot_state_subscription is not None:
            return

        topic_type = self._find_topic_type(self.robot_state_topic)
        if topic_type is None:
            if not self._robot_state_wait_logged:
                self.get_logger().info(
                    f'Waiting for topic {self.robot_state_topic}')
                self._robot_state_wait_logged = True
            return

        try:
            message_type = get_message(topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().error(
                f'Cannot import message type {topic_type}: {exc}')
            return

        self._robot_state_subscription = self.create_subscription(
            message_type,
            self.robot_state_topic,
            partial(self._write_robot_state_message, topic_type=topic_type),
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self._subscriptions.append(self._robot_state_subscription)
        self._robot_state_probe_timer.cancel()
        self.get_logger().info(
            f'Logging {self.robot_state_topic} ({topic_type}) to '
            f'{self.robot_state_jsonl}')

    def _find_topic_type(self, topic_name):
        """Return the first discovered type for a topic, if available."""
        for discovered_topic, topic_types in self.get_topic_names_and_types():
            if discovered_topic == topic_name and topic_types:
                if len(topic_types) > 1:
                    self.get_logger().warn(
                        f'{topic_name} has multiple types; using '
                        f'{topic_types[0]}')
                return topic_types[0]
        return None

    def _write_robot_state_message(self, msg, topic_type):
        """Append one robot state message to a JSON Lines file."""
        received_time = self.get_clock().now().nanoseconds * 1e-9
        timestamp = received_time

        header = getattr(msg, 'header', None)
        if header is not None:
            timestamp = _stamp_to_seconds(header.stamp)

        record = {
            'timestamp': timestamp,
            'received_time': received_time,
            'topic': self.robot_state_topic,
            'type': topic_type,
            'message': message_to_ordereddict(msg),
        }

        with self.robot_state_jsonl.open('a') as file:
            file.write(json.dumps(record) + '\n')


def main(args=None):
    """Run the IMU logger node."""
    rclpy.init(args=args)
    node = ImuLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
