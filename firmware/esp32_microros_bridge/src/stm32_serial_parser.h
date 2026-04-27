#ifndef STM32_SERIAL_PARSER_H
#define STM32_SERIAL_PARSER_H

#include <Arduino.h>
#include <stdint.h>

struct STM32ImuSample {
    const char* source;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float qx;
    float qy;
    float qz;
    float qw;
};

typedef void (*STM32ImuSampleCallback)(const STM32ImuSample& sample);
typedef void (*STM32StateCallback)(int32_t state);
typedef void (*STM32DebugLineCallback)(const char* line);

class STM32SerialParser {
public:
    STM32SerialParser(STM32ImuSampleCallback imu_callback,
                      STM32StateCallback state_callback,
                      STM32DebugLineCallback debug_callback = nullptr,
                      bool accept_train_csv_as_imu = false);

    void handle(Stream& serial);
    bool parseFrame(const char* line);

    uint32_t imuFrameCount() const;
    uint32_t stateFrameCount() const;
    uint32_t droppedFrameCount() const;
    void resetCounts();

private:
    enum {
        kLineMax = 256
    };

    int countCommas(const char* line) const;
    bool isQuaternionValid(float qx, float qy, float qz, float qw) const;
    bool publishImu(const char* source,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float qx, float qy, float qz, float qw);
    bool publishState(int32_t state);
    bool publishDebugLine(const char* line);

    STM32ImuSampleCallback imu_callback_;
    STM32StateCallback state_callback_;
    STM32DebugLineCallback debug_callback_;
    bool accept_train_csv_as_imu_;
    char line_buffer_[kLineMax];
    size_t line_len_;
    uint32_t imu_frame_count_;
    uint32_t state_frame_count_;
    uint32_t dropped_frame_count_;
};

#endif
