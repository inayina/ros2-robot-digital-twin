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

"""Helpers for normalizing motor status payloads before MQTT publish."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from typing import Any


def utc_now_iso() -> str:
    """Return the current UTC time in ISO8601 format."""
    return datetime.now(timezone.utc).isoformat()


def _parse_json_object(raw_payload: str) -> dict[str, Any]:
    try:
        parsed = json.loads(raw_payload)
    except json.JSONDecodeError:
        return {'raw_payload': raw_payload}

    if isinstance(parsed, dict):
        return parsed
    return {'value': parsed}


def _first_value(payload: dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        if key in payload:
            return payload[key]
    return None


def _first_number(payload: dict[str, Any], keys: tuple[str, ...]) -> float | None:
    value = _first_value(payload, keys)
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str) and value.strip():
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _first_bool(payload: dict[str, Any], keys: tuple[str, ...]) -> bool | None:
    value = _first_value(payload, keys)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {'1', 'true', 'yes', 'on'}:
            return True
        if normalized in {'0', 'false', 'no', 'off'}:
            return False
    return None


def normalize_motor_status_payload(
    raw_payload: str,
    *,
    robot_id: str,
    ros_topic: str,
) -> dict[str, Any]:
    """Normalize a ROS 2 /motor/status JSON string for dashboard MQTT."""
    parsed = _parse_json_object(raw_payload)
    measured_rpm = _first_number(parsed, ('measured_rpm', 'actual_rpm'))
    target_rpm = _first_number(parsed, ('target_rpm',))
    error_rpm = _first_number(parsed, ('error_rpm',))
    pwm = _first_number(parsed, ('pwm', 'pwm_duty'))
    enabled = _first_bool(parsed, ('enabled', 'control_enabled'))
    closed_loop = _first_bool(parsed, ('closed_loop',))
    fault = _first_bool(parsed, ('fault', 'fault_active'))
    timeout = _first_bool(parsed, ('timeout', 'timeout_active'))
    estop = _first_bool(parsed, ('estop', 'estop_active', 'stop'))
    command_timeout_ms = _first_number(parsed, ('command_timeout_ms', 'timeout_ms'))
    max_pwm = _first_number(parsed, ('max_pwm',))
    timestamp_ms = _first_number(parsed, ('timestamp_ms', 'updated_ms'))
    publish_ms = _first_number(parsed, ('publish_ms',))
    sample_age_ms = _first_number(parsed, ('sample_age_ms',))
    abs_error_rpm = _first_number(parsed, ('abs_error_rpm',))
    pwm_ratio = _first_number(parsed, ('pwm_ratio',))
    direction = _first_number(parsed, ('direction',))
    saturated = _first_bool(parsed, ('saturated', 'output_saturated'))
    hardware_outputs_enabled = _first_bool(
        parsed, ('hardware_outputs_enabled',))
    numeric_valid = _first_bool(parsed, ('numeric_valid',))

    motor_state = dict(parsed)
    if measured_rpm is not None and 'actual_rpm' not in motor_state:
        motor_state['actual_rpm'] = measured_rpm
    if measured_rpm is not None and 'measured_rpm' not in motor_state:
        motor_state['measured_rpm'] = measured_rpm
    if pwm is not None and 'pwm_duty' not in motor_state:
        motor_state['pwm_duty'] = pwm
    if pwm is not None and 'pwm' not in motor_state:
        motor_state['pwm'] = pwm
    if enabled is not None and 'control_enabled' not in motor_state:
        motor_state['control_enabled'] = enabled
    if enabled is not None and 'enabled' not in motor_state:
        motor_state['enabled'] = enabled

    status = str(parsed.get('status') or '').strip().lower()
    if not status:
        status = 'fault' if fault else 'stopped' if estop else 'stale' if timeout else 'ok'

    payload = {
        'robot_id': parsed.get('robot_id') or robot_id,
        'status': status,
        'source': 'ros2_motor_status_bridge',
        'ros_topic': ros_topic,
        'actual_rpm': measured_rpm,
        'measured_rpm': measured_rpm,
        'target_rpm': target_rpm,
        'error_rpm': error_rpm,
        'pwm': pwm,
        'pwm_duty': pwm,
        'enabled': enabled,
        'control_enabled': enabled,
        'closed_loop': closed_loop,
        'fault': fault,
        'timeout': timeout,
        'estop': estop,
        'stop': estop,
        'last_update_time': parsed.get('last_update_time') or utc_now_iso(),
        'motor_state': motor_state,
    }

    if 'loop' in parsed:
        payload['loop'] = parsed['loop']
    if 'source' in parsed:
        payload['active_source'] = parsed['source']
    if timestamp_ms is not None:
        payload['timestamp_ms'] = int(timestamp_ms)
    if publish_ms is not None:
        payload['publish_ms'] = int(publish_ms)
    if sample_age_ms is not None:
        payload['sample_age_ms'] = int(sample_age_ms)
    if abs_error_rpm is not None:
        payload['abs_error_rpm'] = abs_error_rpm
    if pwm_ratio is not None:
        payload['pwm_ratio'] = pwm_ratio
    if direction is not None:
        payload['direction'] = int(direction)
    if saturated is not None:
        payload['saturated'] = saturated
        payload['output_saturated'] = saturated
    if hardware_outputs_enabled is not None:
        payload['hardware_outputs_enabled'] = hardware_outputs_enabled
    if numeric_valid is not None:
        payload['numeric_valid'] = numeric_valid
    if max_pwm is not None:
        payload['max_pwm'] = max_pwm
    if command_timeout_ms is not None:
        payload['command_timeout_ms'] = int(command_timeout_ms)

    return payload
