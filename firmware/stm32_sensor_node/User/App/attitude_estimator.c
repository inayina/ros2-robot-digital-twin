#include "attitude_estimator.h"
#include <math.h>

#define ATTITUDE_ESTIMATOR_KP 2.0f
#define ATTITUDE_ESTIMATOR_KI 0.005f
#define DEG_TO_RAD 0.017453292519943295f

static void normalize_quaternion(AttitudeEstimator_t *estimator)
{
    float norm = sqrtf(estimator->q0 * estimator->q0 +
                       estimator->q1 * estimator->q1 +
                       estimator->q2 * estimator->q2 +
                       estimator->q3 * estimator->q3);

    if (norm <= 0.0f) {
        estimator->q0 = 1.0f;
        estimator->q1 = 0.0f;
        estimator->q2 = 0.0f;
        estimator->q3 = 0.0f;
        return;
    }

    float inv_norm = 1.0f / norm;
    estimator->q0 *= inv_norm;
    estimator->q1 *= inv_norm;
    estimator->q2 *= inv_norm;
    estimator->q3 *= inv_norm;
}

static void initialize_from_accel(AttitudeEstimator_t *estimator, float ax, float ay, float az)
{
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    estimator->q0 = cr * cp;
    estimator->q1 = sr * cp;
    estimator->q2 = cr * sp;
    estimator->q3 = -sr * sp;
    estimator->initialized = 1;
    normalize_quaternion(estimator);
}

void AttitudeEstimator_Init(AttitudeEstimator_t *estimator)
{
    if (estimator == 0) {
        return;
    }

    estimator->q0 = 1.0f;
    estimator->q1 = 0.0f;
    estimator->q2 = 0.0f;
    estimator->q3 = 0.0f;
    estimator->integral_error[0] = 0.0f;
    estimator->integral_error[1] = 0.0f;
    estimator->integral_error[2] = 0.0f;
    estimator->initialized = 0;
}

void AttitudeEstimator_Update(AttitudeEstimator_t *estimator,
                              float ax, float ay, float az,
                              float gx_dps, float gy_dps, float gz_dps,
                              float dt)
{
    if (estimator == 0) {
        return;
    }

    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.01f;
    }

    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_norm > 0.001f) {
        if (!estimator->initialized) {
            initialize_from_accel(estimator, ax, ay, az);
        }

        float inv_accel_norm = 1.0f / accel_norm;
        ax *= inv_accel_norm;
        ay *= inv_accel_norm;
        az *= inv_accel_norm;

        float q0 = estimator->q0;
        float q1 = estimator->q1;
        float q2 = estimator->q2;
        float q3 = estimator->q3;

        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        estimator->integral_error[0] += ATTITUDE_ESTIMATOR_KI * ex * dt;
        estimator->integral_error[1] += ATTITUDE_ESTIMATOR_KI * ey * dt;
        estimator->integral_error[2] += ATTITUDE_ESTIMATOR_KI * ez * dt;

        gx_dps += (ATTITUDE_ESTIMATOR_KP * ex + estimator->integral_error[0]) / DEG_TO_RAD;
        gy_dps += (ATTITUDE_ESTIMATOR_KP * ey + estimator->integral_error[1]) / DEG_TO_RAD;
        gz_dps += (ATTITUDE_ESTIMATOR_KP * ez + estimator->integral_error[2]) / DEG_TO_RAD;
    }

    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    float q0 = estimator->q0;
    float q1 = estimator->q1;
    float q2 = estimator->q2;
    float q3 = estimator->q3;

    estimator->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    estimator->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    estimator->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    estimator->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;

    normalize_quaternion(estimator);
}
