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

"""Helpers for normalizing dashboard motor commands before ROS 2 publish."""

from __future__ import annotations

from typing import Any

from robot_mqtt_bridge.motor_payload import _first_bool
from robot_mqtt_bridge.motor_payload import _first_number
from robot_mqtt_bridge.motor_payload import _parse_json_object
from robot_mqtt_bridge.motor_payload import utc_now_iso


def clamp_float(value: float, min_value: float, max_value: float) -> float:
    """Clamp a float into the provided inclusive range."""
    return max(min_value, min(max_value, value))


def clamp_int(value: int, min_value: int, max_value: int) -> int:
    """Clamp an int into the provided inclusive range."""
    return max(min_value, min(max_value, value))


def normalize_motor_command_payload(
    raw_payload: str,
    *,
    robot_id: str,
    default_enabled: bool,
    default_closed_loop: bool,
    default_target_rpm: float,
    max_abs_target_rpm: float,
    default_max_pwm: float,
    max_pwm_limit: float,
    default_timeout_ms: int,
    min_timeout_ms: int,
    max_timeout_ms: int,
) -> dict[str, Any]:
    """Normalize MQTT robot/motor/cmd JSON before forwarding to /motor/cmd."""
    parsed = _parse_json_object(raw_payload)

    target_rpm = _first_number(parsed, ('target_rpm',))
    enabled = _first_bool(parsed, ('enabled', 'enable', 'control_enabled'))
    closed_loop = _first_bool(parsed, ('closed_loop',))
    max_pwm = _first_number(parsed, ('max_pwm',))
    timeout_ms = _first_number(parsed, ('timeout_ms', 'command_timeout_ms'))
    stop = _first_bool(parsed, ('stop', 'estop', 'estop_active'))

    normalized_target_rpm = clamp_float(
        target_rpm if target_rpm is not None else default_target_rpm,
        -max_abs_target_rpm,
        max_abs_target_rpm)
    normalized_enabled = enabled if enabled is not None else default_enabled
    normalized_closed_loop = closed_loop if closed_loop is not None else default_closed_loop
    normalized_max_pwm = clamp_float(
        max_pwm if max_pwm is not None else default_max_pwm,
        0.0,
        max_pwm_limit)
    normalized_timeout_ms = clamp_int(
        int(timeout_ms) if timeout_ms is not None else default_timeout_ms,
        min_timeout_ms,
        max_timeout_ms)
    normalized_stop = stop if stop is not None else False

    if normalized_stop:
        normalized_target_rpm = 0.0

    return {
        'robot_id': parsed.get('robot_id') or robot_id,
        'source': parsed.get('source') or 'mqtt_motor_cmd_bridge',
        'command_id': parsed.get('command_id'),
        'issued_at': parsed.get('issued_at') or utc_now_iso(),
        'target_rpm': normalized_target_rpm,
        'enabled': normalized_enabled,
        'closed_loop': normalized_closed_loop,
        'max_pwm': normalized_max_pwm,
        'timeout_ms': normalized_timeout_ms,
        'stop': normalized_stop,
    }
