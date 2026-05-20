#ifndef UROS_PUB_H
#define UROS_PUB_H

#include <Arduino.h>
#include <stdint.h>

#include "motor/motor_control_shared.h"

bool urosPubInit(Print& log, float imu_filter_alpha);
void urosPubFini(Print& log);

struct UrosPubStats {
    uint32_t imu_data_publish_count;
    uint32_t filtered_imu_publish_count;
    uint32_t robot_state_publish_count;
    uint32_t motor_actual_rpm_publish_count;
    uint32_t motor_state_publish_count;
    uint32_t motor_status_publish_count;
    uint32_t imu_data_publish_error_count;
    uint32_t filtered_imu_publish_error_count;
    uint32_t robot_state_publish_error_count;
    uint32_t motor_actual_rpm_publish_error_count;
    uint32_t motor_state_publish_error_count;
    uint32_t motor_status_publish_error_count;
};

void urosPubPublishImuRaw(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float qx, float qy, float qz, float qw);
void urosPubPublishFilteredImu(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               float qx, float qy, float qz, float qw);
void urosPubPublishState(int32_t state);
void urosPubPublishMotorActualRpm(float actual_rpm);
void urosPubPublishMotorState(const MotorControlStateSnapshot& state);
UrosPubStats urosPubGetStats();

#endif
