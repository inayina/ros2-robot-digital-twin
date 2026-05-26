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

from robot_mqtt_bridge.motor_payload import normalize_motor_status_payload


def test_normalize_motor_status_payload_adds_dashboard_aliases():
    payload = normalize_motor_status_payload(
        (
            '{"target_rpm":120.0,"actual_rpm":118.5,"error_rpm":1.5,'
            '"pwm_duty":0.4,"control_enabled":1,"closed_loop":1,'
            '"fault":0,"timeout":0,"estop":0,"source":"target_rpm","loop":42,'
            '"timestamp_ms":123456,"publish_ms":123460,"sample_age_ms":4,'
            '"abs_error_rpm":1.5,"pwm_ratio":0.8,"direction":1,'
            '"saturated":1,"hardware_outputs_enabled":1,"numeric_valid":1}'
        ),
        robot_id='amr-test',
        ros_topic='/motor/status')

    assert payload['robot_id'] == 'amr-test'
    assert payload['actual_rpm'] == 118.5
    assert payload['measured_rpm'] == 118.5
    assert payload['pwm'] == 0.4
    assert payload['enabled'] is True
    assert payload['closed_loop'] is True
    assert payload['fault'] is False
    assert payload['timestamp_ms'] == 123456
    assert payload['publish_ms'] == 123460
    assert payload['sample_age_ms'] == 4
    assert payload['abs_error_rpm'] == 1.5
    assert payload['pwm_ratio'] == 0.8
    assert payload['direction'] == 1
    assert payload['saturated'] is True
    assert payload['output_saturated'] is True
    assert payload['hardware_outputs_enabled'] is True
    assert payload['numeric_valid'] is True
    assert payload['motor_state']['control_enabled'] == 1
    assert payload['motor_state']['loop'] == 42


def test_normalize_motor_status_payload_wraps_non_json():
    payload = normalize_motor_status_payload(
        'raw-motor-status',
        robot_id='amr-test',
        ros_topic='/motor/status')

    assert payload['robot_id'] == 'amr-test'
    assert payload['motor_state']['raw_payload'] == 'raw-motor-status'
    assert payload['status'] == 'ok'


def test_normalize_motor_status_payload_marks_stop_and_limits():
    payload = normalize_motor_status_payload(
        (
            '{"target_rpm":0.0,"measured_rpm":0.0,"pwm":0.0,'
            '"enabled":false,"closed_loop":true,"fault":false,'
            '"stop":true,"max_pwm":0.25,"timeout_ms":800}'
        ),
        robot_id='amr-test',
        ros_topic='/motor/status')

    assert payload['status'] == 'stopped'
    assert payload['estop'] is True
    assert payload['stop'] is True
    assert payload['enabled'] is False
    assert payload['max_pwm'] == 0.25
    assert payload['command_timeout_ms'] == 800


def test_normalize_motor_status_payload_marks_fault_and_timeout():
    payload = normalize_motor_status_payload(
        (
            '{"target_rpm":100.0,"actual_rpm":82.0,"error_rpm":18.0,'
            '"control_enabled":1,"closed_loop":1,"fault":1,"timeout":1}'
        ),
        robot_id='amr-test',
        ros_topic='/motor/status')

    assert payload['status'] == 'fault'
    assert payload['fault'] is True
    assert payload['timeout'] is True
    assert payload['enabled'] is True
