#!/usr/bin/env python3

from math import atan2, cos, sin, sqrt
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from sensor_msgs.msg import Imu


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')

        self.declare_parameter('model_name', 'mpu6050')
        self.declare_parameter('world_name', 'default')
        self.declare_parameter('imu_topic', '/imu/filtered')
        self.declare_parameter('model_file', '')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.35)
        self.declare_parameter('update_rate', 15.0)
        self.declare_parameter('use_imu_estimator', True)
        self.declare_parameter('accel_correction', 0.04)
        self.declare_parameter('gyro_deadband', 0.015)

        self.model_name = self.get_parameter('model_name').value
        self.world_name = self.get_parameter('world_name').value
        self.model_file = self.resolve_model_file()
        self.position = (
            float(self.get_parameter('x').value),
            float(self.get_parameter('y').value),
            float(self.get_parameter('z').value),
        )
        self.use_imu_estimator = bool(
            self.get_parameter('use_imu_estimator').value
        )
        self.accel_correction = float(
            self.get_parameter('accel_correction').value
        )
        self.gyro_deadband = float(self.get_parameter('gyro_deadband').value)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_imu_time = None

        self.spawn_client = self.create_client(
            SpawnEntity,
            f'/world/{self.world_name}/create'
        )
        self.pose_client = self.create_client(
            SetEntityPose,
            f'/world/{self.world_name}/set_pose'
        )

        self.wait_for_gazebo_services()
        self.spawn_model()

        imu_topic = self.get_parameter('imu_topic').value
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            20
        )

        self.latest_orientation = None
        self.request_in_flight = False
        update_rate = float(self.get_parameter('update_rate').value)
        self.timer = self.create_timer(1.0 / update_rate, self.push_model_state)

        mode = 'IMU estimator' if self.use_imu_estimator else 'message quaternion'
        self.get_logger().info(
            f'Gazebo bridge node started: {imu_topic} -> {self.model_name} ({mode})'
        )

    def resolve_model_file(self):
        configured_path = self.get_parameter('model_file').value
        if configured_path:
            return Path(configured_path).expanduser()

        try:
            share_dir = Path(get_package_share_directory('robot_state_monitor'))
            model_file = share_dir / 'models' / 'mpu6050.sdf'
            if model_file.exists():
                return model_file
        except Exception:
            pass

        return Path(__file__).resolve().parents[1] / 'models' / 'mpu6050.sdf'

    def wait_for_gazebo_services(self):
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Waiting for /world/{self.world_name}/create service...'
            )

        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Waiting for /world/{self.world_name}/set_pose service...'
            )

    def spawn_model(self):
        if not self.model_file.exists():
            self.get_logger().error(f'Model file not found: {self.model_file}')
            return

        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]
        pose.orientation.w = 1.0

        factory = EntityFactory()
        factory.name = self.model_name
        factory.allow_renaming = False
        factory.sdf_filename = str(self.model_file)
        factory.pose = pose
        factory.relative_to = 'world'

        request = SpawnEntity.Request()
        request.entity_factory = factory

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error('Timed out while spawning MPU6050 model.')
            return

        response = future.result()
        if response.success:
            self.get_logger().info(f'Spawned Gazebo model: {self.model_name}')
        else:
            self.get_logger().warn(
                f'Could not spawn {self.model_name}; it may already exist.'
            )

    def imu_callback(self, msg):
        if self.use_imu_estimator:
            self.latest_orientation = self.estimate_orientation(msg)
            return

        q = msg.orientation
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            return

        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        self.latest_orientation = q

    def estimate_orientation(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_time is None:
            dt = 0.02
        else:
            dt = max(0.001, min(0.1, now - self.last_imu_time))
        self.last_imu_time = now

        gx = self.apply_deadband(msg.angular_velocity.x)
        gy = self.apply_deadband(msg.angular_velocity.y)
        gz = self.apply_deadband(msg.angular_velocity.z)

        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_norm = sqrt(ax * ax + ay * ay + az * az)

        if accel_norm > 1e-3:
            accel_roll = atan2(ay, az)
            accel_pitch = atan2(-ax, sqrt(ay * ay + az * az))
            alpha = max(0.0, min(1.0, self.accel_correction))
            self.roll = (1.0 - alpha) * self.roll + alpha * accel_roll
            self.pitch = (1.0 - alpha) * self.pitch + alpha * accel_pitch

        return self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

    def apply_deadband(self, value):
        if abs(value) < self.gyro_deadband:
            return 0.0
        return value

    def euler_to_quaternion(self, roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = cos(half_roll)
        sr = sin(half_roll)
        cp = cos(half_pitch)
        sp = sin(half_pitch)
        cy = cos(half_yaw)
        sy = sin(half_yaw)

        q = Pose().orientation
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def push_model_state(self):
        if self.latest_orientation is None or self.request_in_flight:
            return

        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]
        pose.orientation = self.latest_orientation

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = self.model_name
        req.entity.type = Entity.MODEL
        req.pose = pose

        self.request_in_flight = True
        future = self.pose_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error('Failed to set model pose.')
        except Exception as exc:
            self.get_logger().error(f'Service call failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
