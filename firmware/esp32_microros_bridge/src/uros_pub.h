#ifndef UROS_PUB_H
#define UROS_PUB_H

#include <Arduino.h>
#include <stdint.h>

bool urosPubInit(Print& log, float imu_filter_alpha);
void urosPubFini(Print& log);

void urosPubPublishImu(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float qx, float qy, float qz, float qw);
void urosPubPublishState(int32_t state);

#endif
