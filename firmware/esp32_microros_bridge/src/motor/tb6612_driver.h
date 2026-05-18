#ifndef TB6612_DRIVER_H
#define TB6612_DRIVER_H

#include <stdint.h>

constexpr int8_t kTb6612PinUnset = -1;

enum class Tb6612Channel : uint8_t {
    kA = 0,
    kB = 1,
};

enum class Tb6612OutputMode : uint8_t {
    kCoast = 0,
    kBrake = 1,
    kForward = 2,
    kReverse = 3,
};

struct Tb6612ChannelConfig {
    int8_t pwm_pin;
    int8_t dir_in1_pin;
    int8_t dir_in2_pin;
    uint8_t pwm_channel;
    uint32_t pwm_frequency_hz;
    uint8_t pwm_resolution_bits;
    bool invert_direction;
};

struct Tb6612DriverConfig {
    Tb6612ChannelConfig channel_a;
    Tb6612ChannelConfig channel_b;
    int8_t standby_pin;
    bool standby_active_high;
    bool enable_on_begin;
};

class Tb6612Driver {
public:
    Tb6612Driver();

    bool begin(const Tb6612DriverConfig& config);
    bool initialized() const;

    void setStandby(bool enabled);
    bool writeDuty(Tb6612Channel channel, float signed_duty);
    bool writeOutput(Tb6612Channel channel, Tb6612OutputMode mode, float duty);
    void stopAll(Tb6612OutputMode stop_mode);

private:
    bool setupChannel(const Tb6612ChannelConfig& channel_config);
    bool writeChannel(const Tb6612ChannelConfig& channel_config,
                      Tb6612OutputMode mode,
                      float duty);

    Tb6612DriverConfig config_;
    bool initialized_;
};

Tb6612ChannelConfig tb6612MakeUnsetChannelConfig();
Tb6612DriverConfig tb6612MakeUnsetDriverConfig();

#endif
