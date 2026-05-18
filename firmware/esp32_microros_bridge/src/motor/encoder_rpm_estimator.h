#ifndef ENCODER_RPM_ESTIMATOR_H
#define ENCODER_RPM_ESTIMATOR_H

#include <stdint.h>

struct EncoderRpmEstimatorConfig {
    float counts_per_output_rev;
    float filter_alpha;
};

struct EncoderRpmEstimatorRuntime {
    int32_t last_count;
    float filtered_rpm;
    bool initialized;
};

struct EncoderRpmEstimatorSample {
    int32_t encoder_count;
    int32_t delta_count;
    uint32_t sample_period_ms;
    int8_t direction;
    float actual_rpm;
    float filtered_rpm;
    bool valid;
};

void encoderRpmEstimatorInit(EncoderRpmEstimatorRuntime& runtime);
EncoderRpmEstimatorSample encoderRpmEstimatorUpdate(
    EncoderRpmEstimatorRuntime& runtime,
    const EncoderRpmEstimatorConfig& config,
    int32_t encoder_count,
    uint32_t sample_period_ms);

#endif
