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

from robot_mqtt_bridge.motor_command_payload import normalize_motor_command_payload


def test_normalize_motor_command_payload_clamps_and_defaults():
    payload = normalize_motor_command_payload(
        '{"target_rpm":420.0,"enabled":1,"max_pwm":0.8,"timeout_ms":50}',
        robot_id='amr-test',
        default_enabled=True,
        default_closed_loop=True,
        default_target_rpm=0.0,
        max_abs_target_rpm=300.0,
        default_max_pwm=0.25,
        max_pwm_limit=0.5,
        default_timeout_ms=800,
        min_timeout_ms=100,
        max_timeout_ms=5000)

    assert payload['robot_id'] == 'amr-test'
    assert payload['target_rpm'] == 300.0
    assert payload['enabled'] is True
    assert payload['closed_loop'] is True
    assert payload['max_pwm'] == 0.5
    assert payload['timeout_ms'] == 100
    assert payload['stop'] is False


def test_normalize_motor_command_payload_stop_forces_zero_target():
    payload = normalize_motor_command_payload(
        '{"target_rpm":120.0,"stop":true}',
        robot_id='amr-test',
        default_enabled=True,
        default_closed_loop=True,
        default_target_rpm=0.0,
        max_abs_target_rpm=300.0,
        default_max_pwm=0.25,
        max_pwm_limit=0.5,
        default_timeout_ms=800,
        min_timeout_ms=100,
        max_timeout_ms=5000)

    assert payload['target_rpm'] == 0.0
    assert payload['stop'] is True
