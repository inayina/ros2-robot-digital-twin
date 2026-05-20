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

"""Bridge MQTT robot/motor/cmd into ROS 2 /motor/cmd."""

from __future__ import annotations

from collections import deque
import json
import threading

import rclpy
from rclpy.node import Node

from robot_mqtt_bridge.motor_command_payload import normalize_motor_command_payload
from robot_mqtt_bridge.mqtt_client import create_mqtt_client
from std_msgs.msg import String


class MotorCmdBridge(Node):
    """Subscribe to MQTT robot/motor/cmd and publish normalized ROS String."""

    def __init__(self) -> None:
        super().__init__('robot_motor_cmd_bridge')

        self.declare_parameter('ros_cmd_topic', '/motor/cmd')
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/motor/cmd')
        self.declare_parameter('robot_id', 'amr-001')
        self.declare_parameter('mqtt_qos', 0)
        self.declare_parameter('publish_period_ms', 20)
        self.declare_parameter('default_enabled', True)
        self.declare_parameter('default_closed_loop', True)
        self.declare_parameter('default_target_rpm', 0.0)
        self.declare_parameter('max_abs_target_rpm', 300.0)
        self.declare_parameter('default_max_pwm', 0.25)
        self.declare_parameter('max_pwm_limit', 0.5)
        self.declare_parameter('default_timeout_ms', 800)
        self.declare_parameter('min_timeout_ms', 100)
        self.declare_parameter('max_timeout_ms', 5000)

        self.ros_cmd_topic = str(self.get_parameter('ros_cmd_topic').value)
        self.mqtt_host = str(self.get_parameter('mqtt_host').value)
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = str(self.get_parameter('mqtt_topic').value)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.mqtt_qos = int(self.get_parameter('mqtt_qos').value)
        self.publish_period_ms = max(5, int(self.get_parameter('publish_period_ms').value))
        self.default_enabled = bool(self.get_parameter('default_enabled').value)
        self.default_closed_loop = bool(self.get_parameter('default_closed_loop').value)
        self.default_target_rpm = float(self.get_parameter('default_target_rpm').value)
        self.max_abs_target_rpm = float(self.get_parameter('max_abs_target_rpm').value)
        self.default_max_pwm = float(self.get_parameter('default_max_pwm').value)
        self.max_pwm_limit = float(self.get_parameter('max_pwm_limit').value)
        self.default_timeout_ms = int(self.get_parameter('default_timeout_ms').value)
        self.min_timeout_ms = int(self.get_parameter('min_timeout_ms').value)
        self.max_timeout_ms = int(self.get_parameter('max_timeout_ms').value)

        self._publisher = self.create_publisher(String, self.ros_cmd_topic, 10)
        self._pending_payloads: deque[str] = deque()
        self._pending_lock = threading.Lock()
        self._publish_count = 0

        self._mqtt = create_mqtt_client('robot-motor-cmd-bridge')
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect
        self._mqtt.on_message = self._on_message
        self._mqtt.connect_async(self.mqtt_host, self.mqtt_port)
        self._mqtt.loop_start()

        self.create_timer(self.publish_period_ms / 1000.0, self._drain_pending)

        self.get_logger().info(
            'Bridging MQTT %s:%d %s -> %s' % (
                self.mqtt_host,
                self.mqtt_port,
                self.mqtt_topic,
                self.ros_cmd_topic))

    def destroy_node(self) -> bool:
        try:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        except Exception:
            pass
        return super().destroy_node()

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None) -> None:
        if int(getattr(reason_code, 'value', reason_code)) == 0:
            client.subscribe(self.mqtt_topic, qos=self.mqtt_qos)
            self.get_logger().info(f'Subscribed MQTT topic {self.mqtt_topic}')
            return

        self.get_logger().warning(
            f'MQTT connect failed for {self.mqtt_topic} reason_code={reason_code}')

    def _on_disconnect(self, _client, _userdata, reason_code, _properties=None) -> None:
        self.get_logger().warning(
            f'MQTT disconnected for {self.mqtt_topic} reason_code={reason_code}')

    def _on_message(self, _client, _userdata, message) -> None:
        raw_payload = message.payload.decode('utf-8', errors='replace')
        normalized = normalize_motor_command_payload(
            raw_payload,
            robot_id=self.robot_id,
            default_enabled=self.default_enabled,
            default_closed_loop=self.default_closed_loop,
            default_target_rpm=self.default_target_rpm,
            max_abs_target_rpm=self.max_abs_target_rpm,
            default_max_pwm=self.default_max_pwm,
            max_pwm_limit=self.max_pwm_limit,
            default_timeout_ms=self.default_timeout_ms,
            min_timeout_ms=self.min_timeout_ms,
            max_timeout_ms=self.max_timeout_ms)
        payload_json = json.dumps(normalized, ensure_ascii=True, separators=(',', ':'))
        with self._pending_lock:
            self._pending_payloads.append(payload_json)

    def _drain_pending(self) -> None:
        while True:
            with self._pending_lock:
                if not self._pending_payloads:
                    return
                payload_json = self._pending_payloads.popleft()

            msg = String()
            msg.data = payload_json
            self._publisher.publish(msg)
            self._publish_count += 1
            if self._publish_count == 1 or self._publish_count % 20 == 0:
                self.get_logger().info(
                    f'Published {self._publish_count} motor command messages')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
