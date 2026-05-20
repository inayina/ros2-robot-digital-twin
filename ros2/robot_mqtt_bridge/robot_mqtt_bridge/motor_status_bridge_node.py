# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bridge ROS 2 /motor/status into dashboard MQTT robot/motor/status."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_mqtt_bridge.motor_payload import normalize_motor_status_payload
from robot_mqtt_bridge.mqtt_client import create_mqtt_client
from std_msgs.msg import String


class MotorStatusBridge(Node):
    """Subscribe to /motor/status and mirror normalized data to MQTT."""

    def __init__(self) -> None:
        super().__init__('robot_motor_status_bridge')

        self.declare_parameter('motor_status_topic', '/motor/status')
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/motor/status')
        self.declare_parameter('robot_id', 'amr-001')
        self.declare_parameter('mqtt_qos', 0)

        self.motor_status_topic = str(
            self.get_parameter('motor_status_topic').value)
        self.mqtt_host = str(self.get_parameter('mqtt_host').value)
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = str(self.get_parameter('mqtt_topic').value)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.mqtt_qos = int(self.get_parameter('mqtt_qos').value)
        self._mqtt_connected = False
        self._warned_disconnected = False

        self._mqtt = create_mqtt_client('robot-motor-status-bridge')
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect
        self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
        self._mqtt.loop_start()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            String,
            self.motor_status_topic,
            self._handle_motor_status,
            qos)

        self._publish_count = 0
        self.get_logger().info(
            'Bridging %s -> MQTT %s:%d %s' % (
                self.motor_status_topic,
                self.mqtt_host,
                self.mqtt_port,
                self.mqtt_topic))

    def destroy_node(self) -> bool:
        try:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        except Exception:
            pass
        return super().destroy_node()

    def _on_connect(self, _client, _userdata, _flags, reason_code, _properties=None) -> None:
        reason_value = int(getattr(reason_code, 'value', reason_code))
        if reason_value == 0:
            self._mqtt_connected = True
            self._warned_disconnected = False
            self.get_logger().info(
                f'Connected MQTT broker {self.mqtt_host}:{self.mqtt_port}')
            return

        self._mqtt_connected = False
        self.get_logger().warning(
            f'MQTT connect failed for {self.mqtt_topic} reason_code={reason_code}')

    def _on_disconnect(self, _client, _userdata, reason_code, _properties=None) -> None:
        self._mqtt_connected = False
        self.get_logger().warning(
            f'MQTT disconnected for {self.mqtt_topic} reason_code={reason_code}')

    def _handle_motor_status(self, msg: String) -> None:
        if not self._mqtt_connected:
            if not self._warned_disconnected:
                self.get_logger().warning(
                    f'Skipping publish while MQTT is disconnected topic={self.mqtt_topic}')
                self._warned_disconnected = True
            return

        payload = normalize_motor_status_payload(
            msg.data,
            robot_id=self.robot_id,
            ros_topic=self.motor_status_topic)
        payload_json = json.dumps(payload, ensure_ascii=True, separators=(',', ':'))
        result = self._mqtt.publish(
            self.mqtt_topic,
            payload_json,
            qos=self.mqtt_qos,
            retain=False)
        status = getattr(result, 'rc', 0)
        if status != 0:
            self.get_logger().warning(
                f'MQTT publish failed topic={self.mqtt_topic} rc={status}')
            return

        self._publish_count += 1
        if self._publish_count == 1 or self._publish_count % 50 == 0:
            self.get_logger().info(
                f'Published {self._publish_count} motor status messages')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorStatusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
