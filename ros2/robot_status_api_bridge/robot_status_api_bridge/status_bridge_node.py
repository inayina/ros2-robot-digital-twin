"""Aggregate ROS 2 topics into a dashboard-friendly robot status snapshot."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from math import asin, atan2, degrees, sqrt
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Optional, Tuple
from urllib.error import URLError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


ROBOT_STATE_LABELS = {
    0: 'normal',
    1: 'vibration',
    2: 'collision',
    3: 'tip_over',
}


@dataclass
class ImuSnapshot:
    """Hold the latest data for one IMU topic."""

    topic: str
    header_stamp_sec: Optional[float] = None
    received_time_sec: Optional[float] = None
    quaternion: Optional[Tuple[float, float, float, float]] = None


@dataclass
class RobotStateSnapshot:
    """Hold the latest robot_state value."""

    topic: str
    value: Optional[int] = None
    received_time_sec: Optional[float] = None


def _stamp_to_seconds(stamp):
    """Convert a ROS timestamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def _unix_to_iso8601(unix_time_sec):
    """Convert a Unix timestamp to an ISO-8601 UTC string."""
    return datetime.fromtimestamp(
        unix_time_sec,
        tz=timezone.utc).isoformat().replace('+00:00', 'Z')


def _normalize_quaternion(x, y, z, w):
    """Return a normalized quaternion or ``None`` if the input is invalid."""
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return None

    return (
        x / norm,
        y / norm,
        z / norm,
        w / norm,
    )


def _quaternion_to_rpy(quaternion):
    """Convert a normalized quaternion to roll, pitch, yaw."""
    x, y, z, w = quaternion

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _robot_state_label(value):
    """Return a readable label for the numeric robot_state."""
    return ROBOT_STATE_LABELS.get(value, 'unknown')


def _find_repo_root(start_path):
    """Return the nearest parent that looks like this repository root."""
    current = start_path if start_path.is_dir() else start_path.parent
    for candidate in [current, *current.parents]:
        if (candidate / '.git').exists() and (candidate / 'ros2').is_dir():
            return candidate
    return None


def _resolve_output_path(configured_path):
    """Resolve the configured output path against the repository root."""
    path = Path(configured_path).expanduser()
    if path.is_absolute():
        return path

    repo_root = _find_repo_root(Path.cwd())
    if repo_root is None:
        repo_root = _find_repo_root(Path(__file__).resolve())
    if repo_root is None:
        repo_root = Path.cwd()

    return repo_root / path


class RobotStatusApiBridgeNode(Node):
    """Aggregate ROS 2 robot state into a dashboard status file."""

    def __init__(self):
        """Create subscriptions, timers, and output paths."""
        super().__init__('robot_status_api_bridge')

        self.declare_parameter('raw_imu_topic', '/imu/data')
        self.declare_parameter('filtered_imu_topic', '/imu/filtered')
        self.declare_parameter('robot_state_topic', '/robot/state')
        self.declare_parameter(
            'output_path',
            'data/dashboard_state/latest_robot_status.json')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('imu_stale_timeout_sec', 1.5)
        self.declare_parameter('robot_state_stale_timeout_sec', 2.5)
        self.declare_parameter('enable_http_post', False)
        self.declare_parameter('http_endpoint', '')
        self.declare_parameter('http_timeout_sec', 2.0)

        self.raw_imu_topic = self._get_string_parameter('raw_imu_topic')
        self.filtered_imu_topic = self._get_string_parameter(
            'filtered_imu_topic')
        self.robot_state_topic = self._get_string_parameter(
            'robot_state_topic')
        self.output_path = _resolve_output_path(
            self._get_string_parameter('output_path'))
        self.publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value)
        self.imu_stale_timeout_sec = float(
            self.get_parameter('imu_stale_timeout_sec').value)
        self.robot_state_stale_timeout_sec = float(
            self.get_parameter('robot_state_stale_timeout_sec').value)
        self.enable_http_post = bool(
            self.get_parameter('enable_http_post').value)
        self.http_endpoint = self._get_string_parameter('http_endpoint')
        self.http_timeout_sec = float(
            self.get_parameter('http_timeout_sec').value)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'publish_rate_hz must be positive; falling back to 1.0 Hz')
            self.publish_rate_hz = 1.0

        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.raw_imu = ImuSnapshot(topic=self.raw_imu_topic)
        self.filtered_imu = ImuSnapshot(topic=self.filtered_imu_topic)
        self.robot_state = RobotStateSnapshot(topic=self.robot_state_topic)
        self._invalid_orientation_topics = set()
        self._http_endpoint_warned = False

        self._subscriptions = [
            self.create_subscription(
                Imu,
                self.raw_imu_topic,
                self._handle_raw_imu,
                qos_profile_sensor_data),
            self.create_subscription(
                Imu,
                self.filtered_imu_topic,
                self._handle_filtered_imu,
                qos_profile_sensor_data),
            self.create_subscription(
                Int32,
                self.robot_state_topic,
                self._handle_robot_state,
                QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT)),
        ]

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_status)
        self._publish_status()

        self.get_logger().info(
            f'Writing robot status snapshots to {self.output_path}')
        self.get_logger().info(
            f'Subscribed to {self.raw_imu_topic}, '
            f'{self.filtered_imu_topic}, and {self.robot_state_topic}')
        if self.enable_http_post:
            self.get_logger().info(
                'Optional HTTP POST bridge enabled for dashboard backend')

    def _get_string_parameter(self, name):
        """Return a declared string parameter."""
        return self.get_parameter(name).get_parameter_value().string_value

    def _now_seconds(self):
        """Return the node clock time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _handle_raw_imu(self, msg):
        """Store the latest raw IMU message."""
        self._update_imu_snapshot(self.raw_imu, msg)

    def _handle_filtered_imu(self, msg):
        """Store the latest filtered IMU message."""
        self._update_imu_snapshot(self.filtered_imu, msg)

    def _update_imu_snapshot(self, snapshot, msg):
        """Update the stored snapshot for an IMU topic."""
        snapshot.header_stamp_sec = _stamp_to_seconds(msg.header.stamp)
        snapshot.received_time_sec = self._now_seconds()
        snapshot.quaternion = _normalize_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)

        if snapshot.quaternion is None:
            if snapshot.topic not in self._invalid_orientation_topics:
                self.get_logger().warn(
                    f'{snapshot.topic} published an invalid quaternion; '
                    'orientation fields will stay null until a valid sample '
                    'arrives')
                self._invalid_orientation_topics.add(snapshot.topic)
        elif snapshot.topic in self._invalid_orientation_topics:
            self._invalid_orientation_topics.remove(snapshot.topic)

    def _handle_robot_state(self, msg):
        """Store the latest robot_state message."""
        self.robot_state.value = int(msg.data)
        self.robot_state.received_time_sec = self._now_seconds()

    def _select_imu_snapshot(self):
        """Return the freshest available IMU snapshot."""
        candidates = [
            snapshot for snapshot in (self.filtered_imu, self.raw_imu)
            if snapshot.received_time_sec is not None
        ]
        if not candidates:
            return None

        return max(
            candidates,
            key=lambda snapshot: (
                snapshot.received_time_sec,
                1 if snapshot.topic == self.filtered_imu_topic else 0))

    def _source_status(self, last_received_time_sec, stale_timeout_sec):
        """Return the freshness status and age for one data source."""
        if last_received_time_sec is None:
            return 'missing', None

        age_sec = max(0.0, self._now_seconds() - last_received_time_sec)
        if age_sec <= stale_timeout_sec:
            return 'ok', round(age_sec, 3)

        return 'stale', round(age_sec, 3)

    def _build_source_payload(
            self,
            topic,
            last_received_time_sec,
            stale_timeout_sec,
            header_stamp_sec=None):
        """Build one topic health record."""
        status, age_sec = self._source_status(
            last_received_time_sec,
            stale_timeout_sec)
        payload = {
            'topic': topic,
            'status': status,
            'age_sec': age_sec,
            'last_received_time': (
                None if last_received_time_sec is None
                else _unix_to_iso8601(last_received_time_sec)),
        }
        if header_stamp_sec is not None:
            payload['last_header_stamp'] = header_stamp_sec
        else:
            payload['last_header_stamp'] = None
        return payload

    def _build_health_status(self, source_payloads):
        """Build the overall health summary used by the dashboard backend."""
        raw_status = source_payloads['imu_raw']['status']
        filtered_status = source_payloads['imu_filtered']['status']
        robot_status = source_payloads['robot_state']['status']

        fresh_imu_count = sum(
            status == 'ok' for status in (raw_status, filtered_status))
        any_data_received = any(
            payload['status'] != 'missing'
            for payload in (
                source_payloads['imu_raw'],
                source_payloads['imu_filtered'],
                source_payloads['robot_state']))

        if not any_data_received:
            status = 'waiting'
            summary = 'Waiting for IMU and robot_state topics.'
        elif robot_status == 'ok' and fresh_imu_count == 2:
            status = 'ok'
            summary = 'Both IMU topics and robot_state are fresh.'
        elif robot_status == 'ok' and fresh_imu_count == 1:
            status = 'degraded'
            summary = 'Only one IMU topic is fresh; robot_state is fresh.'
        elif robot_status == 'missing' and fresh_imu_count > 0:
            status = 'degraded'
            summary = 'IMU is present but robot_state has not arrived yet.'
        else:
            status = 'stale'
            summary = 'Required robot status inputs are stale.'

        return {
            'status': status,
            'summary': summary,
            'sources': source_payloads,
        }

    def _build_payload(self):
        """Assemble the JSON snapshot written to disk or posted later."""
        now_sec = self._now_seconds()
        selected_imu = self._select_imu_snapshot()

        quaternion_payload = None
        rpy_payload = None
        latest_imu_timestamp = None
        selected_imu_topic = None

        if selected_imu is not None:
            latest_imu_timestamp = selected_imu.header_stamp_sec
            selected_imu_topic = selected_imu.topic
            if selected_imu.quaternion is not None:
                roll, pitch, yaw = _quaternion_to_rpy(selected_imu.quaternion)
                quaternion_payload = {
                    'x': selected_imu.quaternion[0],
                    'y': selected_imu.quaternion[1],
                    'z': selected_imu.quaternion[2],
                    'w': selected_imu.quaternion[3],
                }
                rpy_payload = {
                    'roll_rad': roll,
                    'pitch_rad': pitch,
                    'yaw_rad': yaw,
                    'roll_deg': degrees(roll),
                    'pitch_deg': degrees(pitch),
                    'yaw_deg': degrees(yaw),
                }

        source_payloads = {
            'imu_raw': self._build_source_payload(
                self.raw_imu.topic,
                self.raw_imu.received_time_sec,
                self.imu_stale_timeout_sec,
                self.raw_imu.header_stamp_sec),
            'imu_filtered': self._build_source_payload(
                self.filtered_imu.topic,
                self.filtered_imu.received_time_sec,
                self.imu_stale_timeout_sec,
                self.filtered_imu.header_stamp_sec),
            'robot_state': self._build_source_payload(
                self.robot_state.topic,
                self.robot_state.received_time_sec,
                self.robot_state_stale_timeout_sec),
            'motor_state': {
                'topic': '/motor/state',
                'status': 'reserved',
                'age_sec': None,
                'last_received_time': None,
                'last_header_stamp': None,
            },
            'motor_actual_rpm': {
                'topic': '/motor/actual_rpm',
                'status': 'reserved',
                'age_sec': None,
                'last_received_time': None,
                'last_header_stamp': None,
            },
        }

        return {
            'schema_version': 1,
            'latest_imu_timestamp': latest_imu_timestamp,
            'selected_imu_topic': selected_imu_topic,
            'quaternion': quaternion_payload,
            'rpy': rpy_payload,
            'robot_state': {
                'value': self.robot_state.value,
                'label': (
                    None if self.robot_state.value is None
                    else _robot_state_label(self.robot_state.value)),
            },
            'health_status': self._build_health_status(source_payloads),
            'last_update_time': _unix_to_iso8601(now_sec),
            'output_mode': 'json_file',
            'http_post': {
                'enabled': self.enable_http_post,
                'endpoint': self.http_endpoint if self.enable_http_post else '',
            },
        }

    def _write_payload(self, payload):
        """Atomically write the latest payload to disk."""
        temp_path = None
        try:
            with NamedTemporaryFile(
                    mode='w',
                    encoding='utf-8',
                    dir=self.output_path.parent,
                    delete=False) as file:
                json.dump(payload, file, indent=2, sort_keys=True)
                file.write('\n')
                temp_path = Path(file.name)

            temp_path.replace(self.output_path)
        finally:
            if temp_path is not None and temp_path.exists():
                temp_path.unlink()

    def _post_payload(self, payload):
        """Optionally POST the payload to a dashboard backend."""
        if not self.enable_http_post:
            return

        if not self.http_endpoint:
            if not self._http_endpoint_warned:
                self.get_logger().warn(
                    'enable_http_post is true but http_endpoint is empty; '
                    'skipping POST')
                self._http_endpoint_warned = True
            return

        request = Request(
            self.http_endpoint,
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST')

        try:
            with urlopen(request, timeout=self.http_timeout_sec) as response:
                response.read()
                if response.status >= 400:
                    self.get_logger().warn(
                        f'Dashboard backend returned HTTP {response.status}')
        except URLError as exc:
            self.get_logger().warn(f'Failed to POST robot status: {exc}')

    def _publish_status(self):
        """Write the latest robot status snapshot and POST if enabled."""
        payload = self._build_payload()
        self._write_payload(payload)
        self._post_payload(payload)


def main(args=None):
    """Run the robot status API bridge node."""
    rclpy.init(args=args)
    node = RobotStatusApiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
