#include "bridge/motor_command_parser.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

const char* skipWhitespace(const char* cursor) {
    while (cursor != nullptr && *cursor != '\0' && isspace((unsigned char)*cursor)) {
        cursor++;
    }
    return cursor;
}

const char* findFieldValue(const char* payload, const char* key) {
    if (payload == nullptr || key == nullptr) {
        return nullptr;
    }

    char pattern[48];
    const int pattern_len = snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    if (pattern_len <= 0 || pattern_len >= (int)sizeof(pattern)) {
        return nullptr;
    }

    const char* cursor = payload;
    while ((cursor = strstr(cursor, pattern)) != nullptr) {
        const char* colon = strchr(cursor + pattern_len, ':');
        if (colon != nullptr) {
            return skipWhitespace(colon + 1);
        }
        cursor += pattern_len;
    }

    return nullptr;
}

bool parseNumberField(const char* payload,
                      const char* const* keys,
                      size_t key_count,
                      float& out_value) {
    for (size_t i = 0; i < key_count; ++i) {
        const char* value_start = findFieldValue(payload, keys[i]);
        if (value_start == nullptr) {
            continue;
        }

        char* end = nullptr;
        const float parsed = strtof(value_start, &end);
        if (end != value_start) {
            out_value = parsed;
            return true;
        }
    }

    return false;
}

bool parseUnsignedField(const char* payload,
                        const char* const* keys,
                        size_t key_count,
                        uint32_t& out_value) {
    for (size_t i = 0; i < key_count; ++i) {
        const char* value_start = findFieldValue(payload, keys[i]);
        if (value_start == nullptr) {
            continue;
        }

        char* end = nullptr;
        const unsigned long parsed = strtoul(value_start, &end, 10);
        if (end != value_start) {
            out_value = (uint32_t)parsed;
            return true;
        }
    }

    return false;
}

bool parseBoolToken(const char* value_start, bool& out_value) {
    if (value_start == nullptr) {
        return false;
    }

    if (strncmp(value_start, "true", 4) == 0) {
        out_value = true;
        return true;
    }
    if (strncmp(value_start, "false", 5) == 0) {
        out_value = false;
        return true;
    }
    if (*value_start == '1') {
        out_value = true;
        return true;
    }
    if (*value_start == '0') {
        out_value = false;
        return true;
    }
    return false;
}

bool parseBoolField(const char* payload,
                    const char* const* keys,
                    size_t key_count,
                    bool& out_value) {
    for (size_t i = 0; i < key_count; ++i) {
        const char* value_start = findFieldValue(payload, keys[i]);
        if (parseBoolToken(value_start, out_value)) {
            return true;
        }
    }

    return false;
}

}  // namespace

bool motorCommandParseJson(const char* payload, MotorCommandMessage& message) {
    message = {};

    if (payload == nullptr) {
        return false;
    }

    const char* cursor = skipWhitespace(payload);
    if (cursor == nullptr || *cursor != '{') {
        return false;
    }

    const char* target_rpm_keys[] = {"target_rpm"};
    const char* max_pwm_keys[] = {"max_pwm"};
    const char* timeout_keys[] = {"timeout_ms", "command_timeout_ms"};
    const char* enabled_keys[] = {"enabled", "enable", "control_enabled"};
    const char* closed_loop_keys[] = {"closed_loop"};
    const char* stop_keys[] = {"stop", "estop", "estop_active"};
    const char* hardware_enable_keys[] = {"hardware_enable", "hardware_outputs_enabled", "arm"};

    message.has_target_rpm = parseNumberField(
        cursor, target_rpm_keys, sizeof(target_rpm_keys) / sizeof(target_rpm_keys[0]), message.target_rpm);
    message.has_max_pwm = parseNumberField(
        cursor, max_pwm_keys, sizeof(max_pwm_keys) / sizeof(max_pwm_keys[0]), message.max_pwm);
    message.has_timeout_ms = parseUnsignedField(
        cursor, timeout_keys, sizeof(timeout_keys) / sizeof(timeout_keys[0]), message.timeout_ms);
    message.has_enabled = parseBoolField(
        cursor, enabled_keys, sizeof(enabled_keys) / sizeof(enabled_keys[0]), message.enabled);
    message.has_closed_loop = parseBoolField(
        cursor,
        closed_loop_keys,
        sizeof(closed_loop_keys) / sizeof(closed_loop_keys[0]),
        message.closed_loop);
    message.has_stop = parseBoolField(
        cursor, stop_keys, sizeof(stop_keys) / sizeof(stop_keys[0]), message.stop);
    message.has_hardware_enable = parseBoolField(
        cursor,
        hardware_enable_keys,
        sizeof(hardware_enable_keys) / sizeof(hardware_enable_keys[0]),
        message.hardware_enable);

    return message.has_target_rpm ||
        message.has_max_pwm ||
        message.has_timeout_ms ||
        message.has_enabled ||
        message.has_closed_loop ||
        message.has_stop ||
        message.has_hardware_enable;
}
