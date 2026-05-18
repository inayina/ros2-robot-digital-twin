#include "motor/tb6612_driver.h"

#include <Arduino.h>
#include <math.h>

namespace {

bool isValidPin(int8_t pin) {
    return pin >= 0;
}

bool isValidPwmResolution(uint8_t resolution_bits) {
    return resolution_bits > 0U && resolution_bits <= 16U;
}

bool isChannelConfigured(const Tb6612ChannelConfig& channel_config) {
    return isValidPin(channel_config.pwm_pin) ||
        isValidPin(channel_config.dir_in1_pin) ||
        isValidPin(channel_config.dir_in2_pin) ||
        channel_config.pwm_frequency_hz != 0U ||
        channel_config.pwm_resolution_bits != 0U;
}

bool isValidChannelConfig(const Tb6612ChannelConfig& channel_config) {
    return isValidPin(channel_config.pwm_pin) &&
        isValidPin(channel_config.dir_in1_pin) &&
        isValidPin(channel_config.dir_in2_pin) &&
        channel_config.pwm_frequency_hz != 0U &&
        isValidPwmResolution(channel_config.pwm_resolution_bits);
}

float clampDuty(float duty) {
    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }
    return duty;
}

uint32_t maxPwmValue(uint8_t resolution_bits) {
    return (1UL << resolution_bits) - 1UL;
}

void writeDirectionPins(const Tb6612ChannelConfig& channel_config,
                        Tb6612OutputMode mode) {
    switch (mode) {
        case Tb6612OutputMode::kForward:
            digitalWrite(channel_config.dir_in1_pin, HIGH);
            digitalWrite(channel_config.dir_in2_pin, LOW);
            break;
        case Tb6612OutputMode::kReverse:
            digitalWrite(channel_config.dir_in1_pin, LOW);
            digitalWrite(channel_config.dir_in2_pin, HIGH);
            break;
        case Tb6612OutputMode::kBrake:
            digitalWrite(channel_config.dir_in1_pin, HIGH);
            digitalWrite(channel_config.dir_in2_pin, HIGH);
            break;
        case Tb6612OutputMode::kCoast:
        default:
            digitalWrite(channel_config.dir_in1_pin, LOW);
            digitalWrite(channel_config.dir_in2_pin, LOW);
            break;
    }
}

}  // namespace

Tb6612Driver::Tb6612Driver()
    : config_(tb6612MakeUnsetDriverConfig()),
      initialized_(false) {
}

bool Tb6612Driver::begin(const Tb6612DriverConfig& config) {
    initialized_ = false;
    config_ = config;

    const bool channel_a_configured = isChannelConfigured(config_.channel_a);
    const bool channel_b_configured = isChannelConfigured(config_.channel_b);

    if (!isValidPin(config_.standby_pin) ||
        (!channel_a_configured && !channel_b_configured) ||
        (channel_a_configured && !setupChannel(config_.channel_a)) ||
        (channel_b_configured && !setupChannel(config_.channel_b))) {
        return false;
    }

    pinMode(config_.standby_pin, OUTPUT);
    setStandby(false);
    stopAll(Tb6612OutputMode::kCoast);

    initialized_ = true;
    setStandby(config_.enable_on_begin);
    return true;
}

bool Tb6612Driver::initialized() const {
    return initialized_;
}

void Tb6612Driver::setStandby(bool enabled) {
    if (!isValidPin(config_.standby_pin)) {
        return;
    }

    const bool pin_high = config_.standby_active_high ? enabled : !enabled;
    digitalWrite(config_.standby_pin, pin_high ? HIGH : LOW);
}

bool Tb6612Driver::writeDuty(Tb6612Channel channel, float signed_duty) {
    Tb6612OutputMode mode = Tb6612OutputMode::kCoast;
    float duty = signed_duty;

    const Tb6612ChannelConfig& channel_config =
        (channel == Tb6612Channel::kA) ? config_.channel_a : config_.channel_b;

    if (channel_config.invert_direction) {
        duty = -duty;
    }

    if (duty > 0.0f) {
        mode = Tb6612OutputMode::kForward;
    } else if (duty < 0.0f) {
        mode = Tb6612OutputMode::kReverse;
    }

    return writeOutput(channel, mode, fabsf(duty));
}

bool Tb6612Driver::writeOutput(Tb6612Channel channel,
                               Tb6612OutputMode mode,
                               float duty) {
    if (!initialized_) {
        return false;
    }

    const Tb6612ChannelConfig& channel_config =
        (channel == Tb6612Channel::kA) ? config_.channel_a : config_.channel_b;
    return writeChannel(channel_config, mode, duty);
}

void Tb6612Driver::stopAll(Tb6612OutputMode stop_mode) {
    writeChannel(config_.channel_a, stop_mode, 0.0f);
    writeChannel(config_.channel_b, stop_mode, 0.0f);
}

bool Tb6612Driver::setupChannel(const Tb6612ChannelConfig& channel_config) {
    if (!isValidChannelConfig(channel_config)) {
        return false;
    }

    pinMode(channel_config.dir_in1_pin, OUTPUT);
    pinMode(channel_config.dir_in2_pin, OUTPUT);
    writeDirectionPins(channel_config, Tb6612OutputMode::kCoast);

    ledcSetup(channel_config.pwm_channel,
              channel_config.pwm_frequency_hz,
              channel_config.pwm_resolution_bits);
    ledcAttachPin(channel_config.pwm_pin, channel_config.pwm_channel);
    ledcWrite(channel_config.pwm_channel, 0U);
    return true;
}

bool Tb6612Driver::writeChannel(const Tb6612ChannelConfig& channel_config,
                                Tb6612OutputMode mode,
                                float duty) {
    if (!isValidChannelConfig(channel_config)) {
        return false;
    }

    const uint32_t pwm_value =
        (uint32_t)(clampDuty(duty) * (float)maxPwmValue(channel_config.pwm_resolution_bits));
    ledcWrite(channel_config.pwm_channel, pwm_value);
    writeDirectionPins(channel_config, mode);
    return true;
}

Tb6612ChannelConfig tb6612MakeUnsetChannelConfig() {
    Tb6612ChannelConfig config = {};
    config.pwm_pin = kTb6612PinUnset;
    config.dir_in1_pin = kTb6612PinUnset;
    config.dir_in2_pin = kTb6612PinUnset;
    config.pwm_channel = 0U;
    config.pwm_frequency_hz = 0U;
    config.pwm_resolution_bits = 0U;
    config.invert_direction = false;
    return config;
}

Tb6612DriverConfig tb6612MakeUnsetDriverConfig() {
    Tb6612DriverConfig config = {};
    config.channel_a = tb6612MakeUnsetChannelConfig();
    config.channel_b = tb6612MakeUnsetChannelConfig();
    config.standby_pin = kTb6612PinUnset;
    config.standby_active_high = true;
    config.enable_on_begin = false;
    return config;
}
