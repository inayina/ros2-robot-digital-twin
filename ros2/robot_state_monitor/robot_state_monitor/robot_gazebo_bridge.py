#!/usr/bin/env python3

from math import asin, atan2, cos, sin, sqrt
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
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
        self.declare_parameter('lock_yaw', True)

        self.model_name = self.get_parameter('model_name').value
        self.world_name = self.get_parameter('world_name').value
        self.model_file = self.resolve_model_file()
        self.position = (
            float(self.get_parameter('x').value),
            float(self.get_parameter('y').value),
            float(self.get_parameter('z').value),
        )
        self.lock_yaw_enabled = bool(self.get_parameter('lock_yaw').value)
        self.yaw_reference = None

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
        imu_qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            imu_qos
        )

        self.latest_orientation = None
        self.warned_invalid_orientation = False
        self.request_in_flight = False
        update_rate = float(self.get_parameter('update_rate').value)
        self.timer = self.create_timer(1.0 / update_rate, self.push_model_state)

        self.get_logger().info(
            f'Gazebo bridge node started: {imu_topic} -> {self.model_name} '
            f'(message quaternion, qos=best_effort, '
            f'lock_yaw={self.lock_yaw_enabled})'
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
        q = msg.orientation
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            if not self.warned_invalid_orientation:
                self.get_logger().warn(
                    'Ignoring IMU message with invalid orientation quaternion.'
                )
                self.warned_invalid_orientation = True
            return

        orientation = Quaternion(
            x=q.x / norm,
            y=q.y / norm,
            z=q.z / norm,
            w=q.w / norm,
        )
        if self.lock_yaw_enabled:
            orientation = self.with_locked_yaw(orientation)

        self.latest_orientation = orientation

    def with_locked_yaw(self, q):
        roll, pitch, yaw = self.quaternion_to_euler(q)
        if self.yaw_reference is None:
            self.yaw_reference = yaw

        return self.euler_to_quaternion(roll, pitch, self.yaw_reference)

    def quaternion_to_euler(self, q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

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

        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
