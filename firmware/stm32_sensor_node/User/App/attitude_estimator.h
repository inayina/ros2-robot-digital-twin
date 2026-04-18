#ifndef __ATTITUDE_ESTIMATOR_H
#define __ATTITUDE_ESTIMATOR_H

#include <stdint.h>

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_error[3];
    uint8_t initialized;
} AttitudeEstimator_t;

void AttitudeEstimator_Init(AttitudeEstimator_t *estimator);
void AttitudeEstimator_Update(AttitudeEstimator_t *estimator,
                              float ax, float ay, float az,
                              float gx_dps, float gy_dps, float gz_dps,
                              float dt);

#endif
