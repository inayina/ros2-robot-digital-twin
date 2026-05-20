"""Optional MQTT publisher for dashboard robot status snapshots."""

import json

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - depends on optional runtime package
    mqtt = None


class MqttStatusPublisher:
    """Publish aggregate robot status payloads to MQTT topics."""

    def __init__(self, host, port, topic_prefix, logger):
        """Store connection settings for the optional MQTT publisher."""
        self.host = host
        self.port = int(port)
        self.topic_prefix = self._normalize_topic_prefix(topic_prefix)
        self.logger = logger
        self._client = None
        self._started = False
        self._missing_dependency_warned = False
        self._connect_error_warned = False
        self._publish_error_warned = False

    @staticmethod
    def _normalize_topic_prefix(topic_prefix):
        """Normalize the MQTT topic prefix."""
        prefix = str(topic_prefix or '').strip('/')
        return prefix or 'robot'

    def start(self):
        """Start the MQTT client loop if dependencies are available."""
        if self._started:
            return True

        if mqtt is None:
            if not self._missing_dependency_warned:
                self.logger.warn(
                    'enable_mqtt_publish is true but paho-mqtt is not '
                    'installed; skipping MQTT publish')
                self._missing_dependency_warned = True
            return False

        self._client = self._create_client()

        try:
            self._client.connect_async(self.host, self.port)
            self._client.loop_start()
        except Exception as exc:  # pragma: no cover - network dependent
            if not self._connect_error_warned:
                self.logger.warn(f'Failed to start MQTT publisher: {exc}')
                self._connect_error_warned = True
            self._client = None
            return False

        self._started = True
        self.logger.info(
            f'MQTT status publisher enabled at {self.host}:{self.port} '
            f'with topic prefix {self.topic_prefix}')
        return True

    def stop(self):
        """Stop the MQTT client loop."""
        client = self._client
        self._client = None
        self._started = False
        if client is None:
            return

        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass

    def publish_payloads(self, payloads):
        """Publish a mapping of topic suffixes to JSON payloads."""
        if not self.start() or self._client is None:
            return

        for topic_suffix, payload in payloads.items():
            topic = self.topic(topic_suffix)
            body = json.dumps(
                payload,
                ensure_ascii=True,
                separators=(',', ':'))
            try:
                self._client.publish(topic, body)
            except Exception as exc:  # pragma: no cover - network dependent
                if not self._publish_error_warned:
                    self.logger.warn(
                        f'Failed to publish MQTT robot status: {exc}')
                    self._publish_error_warned = True

    def topic(self, topic_suffix):
        """Build a full MQTT topic from a suffix."""
        suffix = str(topic_suffix or '').strip('/')
        if not suffix:
            return self.topic_prefix
        return f'{self.topic_prefix}/{suffix}'

    @staticmethod
    def _create_client():
        """Create a paho MQTT client across paho 1.x and 2.x."""
        callback_api_version = getattr(mqtt, 'CallbackAPIVersion', None)
        if callback_api_version is not None:
            try:
                return mqtt.Client(
                    callback_api_version=callback_api_version.VERSION1,
                    client_id='robot-status-api-bridge')
            except TypeError:
                pass

        return mqtt.Client(client_id='robot-status-api-bridge')
