"""Live plot raw and filtered IMU messages."""

from collections import deque
import math

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


AXES = ('x', 'y', 'z')
ATTITUDE_AXES = ('roll', 'pitch', 'yaw')
SIGNALS = (
    ('linear_acceleration', 'Linear acceleration', 'm/s^2', AXES),
    ('angular_velocity', 'Angular velocity', 'rad/s', AXES),
    ('attitude', 'Attitude', 'deg', ATTITUDE_AXES),
)


def _stamp_to_seconds(stamp):
    """Convert a ROS timestamp message to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def _is_zero_stamp(stamp):
    """Return True when a message has no useful header timestamp."""
    return stamp.sec == 0 and stamp.nanosec == 0


def _quaternion_to_euler_degrees(orientation):
    """Convert a quaternion message to roll, pitch, and yaw in degrees."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        }

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y))
    pitch_input = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_input)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z))

    return {
        'roll': math.degrees(roll),
        'pitch': math.degrees(pitch),
        'yaw': math.degrees(yaw),
    }


class LiveImuPlotNode(Node):
    """Subscribe to raw and filtered IMU topics for live plotting."""

    def __init__(self):
        """Create topic subscriptions and data buffers."""
        super().__init__('imu_live_plot')

        self.declare_parameter('raw_imu_topic', '/imu/data')
        self.declare_parameter('filtered_imu_topic', '/imu/filtered')
        self.declare_parameter('time_window', 20.0)
        self.declare_parameter('update_interval', 0.05)
        self.declare_parameter('use_header_stamp', True)

        self.raw_imu_topic = self._get_string_parameter('raw_imu_topic')
        self.filtered_imu_topic = self._get_string_parameter(
            'filtered_imu_topic')
        self.time_window = self._get_double_parameter('time_window')
        self.update_interval = self._get_double_parameter('update_interval')
        self.use_header_stamp = self._get_bool_parameter('use_header_stamp')

        self.raw_samples = deque()
        self.filtered_samples = deque()
        self.start_time = None

        self._subscriptions = [
            self.create_subscription(
                Imu,
                self.raw_imu_topic,
                self._record_raw_sample,
                qos_profile_sensor_data),
            self.create_subscription(
                Imu,
                self.filtered_imu_topic,
                self._record_filtered_sample,
                qos_profile_sensor_data),
        ]

        self.get_logger().info(
            f'Live plotting raw IMU topic: {self.raw_imu_topic}')
        self.get_logger().info(
            f'Live plotting filtered IMU topic: {self.filtered_imu_topic}')

    def _get_string_parameter(self, name):
        """Read a declared string parameter."""
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_double_parameter(self, name):
        """Read a declared floating point parameter."""
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_bool_parameter(self, name):
        """Read a declared boolean parameter."""
        return self.get_parameter(name).get_parameter_value().bool_value

    def _record_raw_sample(self, msg):
        """Store one raw IMU sample."""
        self._record_sample(self.raw_samples, msg)

    def _record_filtered_sample(self, msg):
        """Store one filtered IMU sample."""
        self._record_sample(self.filtered_samples, msg)

    def _record_sample(self, samples, msg):
        """Store one IMU message in a plotting buffer."""
        timestamp = self._message_time(msg)

        if self.start_time is None:
            self.start_time = timestamp

        relative_time = timestamp - self.start_time
        samples.append({
            'time': relative_time,
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'attitude': _quaternion_to_euler_degrees(msg.orientation),
        })
        self._trim_old_samples(samples, relative_time)

    def _message_time(self, msg):
        """Return the timestamp used for plotting one IMU message."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.use_header_stamp and not _is_zero_stamp(msg.header.stamp):
            return _stamp_to_seconds(msg.header.stamp)
        return now

    def _trim_old_samples(self, samples, latest_time):
        """Remove samples older than the visible time window."""
        while samples and latest_time - samples[0]['time'] > self.time_window:
            samples.popleft()


class LiveImuPlot:
    """Draw raw and filtered IMU streams with matplotlib."""

    def __init__(self, node):
        """Create a figure and line handles."""
        self.node = node
        self.figure, axes = plt.subplots(
            len(SIGNALS),
            len(AXES),
            sharex=True,
            figsize=(14, 9))
        self.axes = axes
        self.lines = {
            'raw': {},
            'filtered': {},
        }

        self._configure_figure()

    def _configure_figure(self):
        """Configure subplot labels, legends, and initial limits."""
        self.figure.suptitle('Raw vs Filtered IMU')

        for signal_index, signal in enumerate(SIGNALS):
            signal_name, title, unit, signal_axes = signal
            for axis_index, axis_name in enumerate(signal_axes):
                axis = self.axes[signal_index][axis_index]
                raw_line, = axis.plot(
                    [],
                    [],
                    color='tab:orange',
                    alpha=0.75,
                    label='raw')
                filtered_line, = axis.plot(
                    [],
                    [],
                    color='tab:blue',
                    linewidth=1.8,
                    label='filtered')
                axis.set_title(f'{title} {axis_name}')
                axis.set_ylabel(unit)
                axis.set_xlim(0.0, self.node.time_window)
                axis.grid(True, alpha=0.3)

                self.lines['raw'][(signal_name, axis_name)] = raw_line
                self.lines['filtered'][(signal_name, axis_name)] = (
                    filtered_line)

        for axis in self.axes[-1]:
            axis.set_xlabel('time (s)')

        self.axes[0][0].legend(loc='upper right')
        self.figure.tight_layout()

    def update(self):
        """Refresh plot data from the node buffers."""
        latest_time = self._latest_time()

        for signal_name, _, _, signal_axes in SIGNALS:
            for axis_name in signal_axes:
                self._update_line(
                    'raw',
                    signal_name,
                    axis_name,
                    self.node.raw_samples)
                self._update_line(
                    'filtered',
                    signal_name,
                    axis_name,
                    self.node.filtered_samples)

        self._update_axes(latest_time)
        self.figure.canvas.draw_idle()

    def _update_line(self, stream_name, signal_name, axis_name, samples):
        """Update one plotted line from a sample buffer."""
        times = [sample['time'] for sample in samples]
        values = [
            sample[signal_name][axis_name]
            for sample in samples
        ]
        line = self.lines[stream_name][(signal_name, axis_name)]
        line.set_data(times, values)

    def _latest_time(self):
        """Return the newest sample time in either stream."""
        latest_times = []
        if self.node.raw_samples:
            latest_times.append(self.node.raw_samples[-1]['time'])
        if self.node.filtered_samples:
            latest_times.append(self.node.filtered_samples[-1]['time'])

        if not latest_times:
            return self.node.time_window
        return max(latest_times)

    def _update_axes(self, latest_time):
        """Update x/y limits after line data changes."""
        start_time = max(0.0, latest_time - self.node.time_window)
        end_time = max(self.node.time_window, latest_time)

        for axis_row in self.axes:
            for axis in axis_row:
                axis.set_xlim(start_time, end_time)
                axis.relim()
                axis.autoscale_view(scalex=False, scaley=True)


def main(args=None):
    """Run the live IMU plotting node."""
    rclpy.init(args=args)
    node = LiveImuPlotNode()
    plot = LiveImuPlot(node)

    plt.ion()
    plt.show(block=False)

    try:
        while rclpy.ok() and plt.fignum_exists(plot.figure.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            plot.update()
            plt.pause(node.update_interval)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
