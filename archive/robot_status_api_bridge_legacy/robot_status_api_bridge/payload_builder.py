"""Build dashboard and MQTT payload fragments for robot status output."""


def _source(payload, key):
    """Return one source freshness record from the aggregate payload."""
    return payload.get('health_status', {}).get('sources', {}).get(key, {})


def _selected_imu_source_key(payload):
    """Return the source key for the selected IMU topic."""
    selected_topic = payload.get('selected_imu_topic')
    sources = payload.get('health_status', {}).get('sources', {})
    for key in ('imu_filtered', 'imu_raw'):
        if sources.get(key, {}).get('topic') == selected_topic:
            return key
    return None


def _selected_imu_freshness(payload):
    """Return freshness for the selected IMU sample."""
    selected_key = _selected_imu_source_key(payload)
    if selected_key is None:
        return {
            'topic': payload.get('selected_imu_topic'),
            'status': 'missing',
            'age_sec': None,
            'last_received_time': None,
            'last_header_stamp': None,
        }
    return _source(payload, selected_key)


def _motor_status(rpm_freshness, state_freshness):
    """Return a compact status for the motor payload."""
    statuses = {
        rpm_freshness.get('status', 'missing'),
        state_freshness.get('status', 'missing'),
    }
    if statuses == {'missing'}:
        return 'reserved'
    if 'stale' in statuses:
        return 'stale'
    if 'missing' in statuses:
        return 'missing'
    return 'ok'


def build_mqtt_topic_payloads(payload):
    """Split an aggregate robot status payload into MQTT topic payloads."""
    imu_raw_freshness = _source(payload, 'imu_raw')
    imu_filtered_freshness = _source(payload, 'imu_filtered')
    robot_state_freshness = _source(payload, 'robot_state')
    motor_rpm_freshness = _source(payload, 'motor_actual_rpm')
    motor_state_freshness = _source(payload, 'motor_state')
    motor_payload = payload.get('motor', {})

    imu_payload = {
        'schema_version': payload.get('schema_version'),
        'topic': payload.get('selected_imu_topic'),
        'latest_imu_timestamp': payload.get('latest_imu_timestamp'),
        'quaternion': payload.get('quaternion'),
        'rpy': payload.get('rpy'),
        'freshness': _selected_imu_freshness(payload),
        'sources': {
            'imu_raw': imu_raw_freshness,
            'imu_filtered': imu_filtered_freshness,
        },
        'last_update_time': payload.get('last_update_time'),
    }

    state_payload = {
        'schema_version': payload.get('schema_version'),
        'value': payload.get('robot_state', {}).get('value'),
        'label': payload.get('robot_state', {}).get('label'),
        'freshness': robot_state_freshness,
        'last_update_time': payload.get('last_update_time'),
    }

    motor_status_payload = {
        'schema_version': payload.get('schema_version'),
        'status': _motor_status(motor_rpm_freshness, motor_state_freshness),
        'actual_rpm': motor_payload.get('actual_rpm'),
        'motor_state': motor_payload.get('motor_state'),
        'freshness': {
            'actual_rpm': motor_rpm_freshness,
            'motor_state': motor_state_freshness,
        },
        'last_update_time': payload.get('last_update_time'),
    }

    return {
        'status': payload,
        'imu': imu_payload,
        'state': state_payload,
        'motor/status': motor_status_payload,
    }
