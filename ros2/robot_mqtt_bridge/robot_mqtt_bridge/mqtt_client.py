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

"""Small MQTT client wrapper shared by bridge nodes."""

from __future__ import annotations

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - depends on runtime environment
    mqtt = None


def create_mqtt_client(client_id: str):
    """Create a paho MQTT client across paho 1.x and 2.x."""
    if mqtt is None:
        raise RuntimeError('paho-mqtt is not installed')

    callback_api_version = getattr(mqtt, 'CallbackAPIVersion', None)
    if callback_api_version is not None:
        try:
            return mqtt.Client(
                callback_api_version=callback_api_version.VERSION1,
                client_id=client_id)
        except TypeError:
            pass

    return mqtt.Client(client_id=client_id)
