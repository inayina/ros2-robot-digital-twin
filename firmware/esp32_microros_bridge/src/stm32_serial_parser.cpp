#include "stm32_serial_parser.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

STM32SerialParser::STM32SerialParser(STM32ImuSampleCallback imu_callback,
                                     STM32StateCallback state_callback,
                                     STM32DebugLineCallback debug_callback,
                                     bool accept_train_csv_as_imu)
    : imu_callback_(imu_callback),
      state_callback_(state_callback),
      debug_callback_(debug_callback),
      accept_train_csv_as_imu_(accept_train_csv_as_imu),
      line_len_(0),
      imu_frame_count_(0),
      state_frame_count_(0),
      dropped_frame_count_(0) {
    line_buffer_[0] = '\0';
}

void STM32SerialParser::handle(Stream& serial) {
    while (serial.available()) {
        char c = (char)serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (line_len_ > 0) {
                line_buffer_[line_len_] = '\0';
                if (!parseFrame(line_buffer_)) {
                    dropped_frame_count_++;
                }
                line_len_ = 0;
            }
            continue;
        }

        if (line_len_ < kLineMax - 1) {
            line_buffer_[line_len_++] = c;
        } else {
            line_len_ = 0;
            dropped_frame_count_++;
        }
    }
}

bool STM32SerialParser::parseFrame(const char* line) {
    if (strncmp(line, "IMUQ,", 5) == 0) {
        float ax, ay, az, gx, gy, gz, qx, qy, qz, qw, temp;
        if (sscanf(line, "IMUQ,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &ax, &ay, &az, &gx, &gy, &gz, &qx, &qy, &qz, &qw, &temp) == 11) {
            if (!isQuaternionValid(qx, qy, qz, qw)) {
                return false;
            }
            return publishImu("IMUQ", ax, ay, az, gx, gy, gz, qx, qy, qz, qw);
        }
        return false;
    }

    if (strncmp(line, "IMU,", 4) == 0) {
        float ax, ay, az, gx, gy, gz, temp;
        if (sscanf(line, "IMU,%f,%f,%f,%f,%f,%f,%f",
                   &ax, &ay, &az, &gx, &gy, &gz, &temp) == 7) {
            return publishImu("IMU", ax, ay, az, gx, gy, gz, 0.0f, 0.0f, 0.0f, 1.0f);
        }
        return false;
    }

    if (accept_train_csv_as_imu_ && isDigit(line[0]) && countCommas(line) == 7) {
        unsigned long sample_ms;
        float ax, ay, az, gx, gy, gz;
        int state;
        if (sscanf(line, "%lu,%f,%f,%f,%f,%f,%f,%d",
                   &sample_ms, &ax, &ay, &az, &gx, &gy, &gz, &state) == 8) {
            return publishImu("CSV", ax, ay, az, gx, gy, gz, 0.0f, 0.0f, 0.0f, 1.0f);
        }
        return false;
    }

    if (strncmp(line, "State:", 6) == 0) {
        int state = 0;
        if (sscanf(line, "State:%d", &state) == 1) {
            return publishState(state);
        }
        return false;
    }

    if (strncmp(line, "DBG:", 4) == 0) {
        return publishDebugLine(line + 4);
    }

    return false;
}

uint32_t STM32SerialParser::imuFrameCount() const {
    return imu_frame_count_;
}

uint32_t STM32SerialParser::stateFrameCount() const {
    return state_frame_count_;
}

uint32_t STM32SerialParser::droppedFrameCount() const {
    return dropped_frame_count_;
}

void STM32SerialParser::resetCounts() {
    imu_frame_count_ = 0;
    state_frame_count_ = 0;
    dropped_frame_count_ = 0;
}

int STM32SerialParser::countCommas(const char* line) const {
    int count = 0;
    while (*line != '\0') {
        if (*line == ',') {
            count++;
        }
        line++;
    }
    return count;
}

bool STM32SerialParser::isQuaternionValid(float qx, float qy, float qz, float qw) const {
    float norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    return norm_sq > 0.5f && norm_sq < 1.5f;
}

bool STM32SerialParser::publishImu(const char* source,
                                   float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float qx, float qy, float qz, float qw) {
    if (imu_callback_ == NULL) {
        return false;
    }

    STM32ImuSample sample = {
        source,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        qx,
        qy,
        qz,
        qw
    };

    imu_callback_(sample);
    imu_frame_count_++;
    return true;
}

bool STM32SerialParser::publishState(int32_t state) {
    if (state_callback_ == NULL) {
        return false;
    }

    state_callback_(state);
    state_frame_count_++;
    return true;
}

bool STM32SerialParser::publishDebugLine(const char* line) {
    if (debug_callback_ == NULL) {
        return false;
    }

    debug_callback_(line);
    return true;
}
