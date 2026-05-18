#include "motor/encoder_rpm_estimator.h"

namespace {

float clampFilterAlpha(float alpha) {
    if (alpha < 0.0f) {
        return 0.0f;
    }
    if (alpha > 1.0f) {
        return 1.0f;
    }
    return alpha;
}

int8_t directionForDelta(int32_t delta_count) {
    if (delta_count > 0) {
        return 1;
    }
    if (delta_count < 0) {
        return -1;
    }
    return 0;
}

}  // namespace

void encoderRpmEstimatorInit(EncoderRpmEstimatorRuntime& runtime) {
    runtime.last_count = 0;
    runtime.filtered_rpm = 0.0f;
    runtime.initialized = false;
}

EncoderRpmEstimatorSample encoderRpmEstimatorUpdate(
    EncoderRpmEstimatorRuntime& runtime,
    const EncoderRpmEstimatorConfig& config,
    int32_t encoder_count,
    uint32_t sample_period_ms) {
    EncoderRpmEstimatorSample sample = {};
    sample.encoder_count = encoder_count;
    sample.sample_period_ms = sample_period_ms;

    if (!runtime.initialized) {
        runtime.last_count = encoder_count;
        runtime.filtered_rpm = 0.0f;
        runtime.initialized = true;
        sample.filtered_rpm = runtime.filtered_rpm;
        sample.valid = false;
        return sample;
    }

    sample.delta_count = encoder_count - runtime.last_count;
    runtime.last_count = encoder_count;
    sample.direction = directionForDelta(sample.delta_count);

    if (config.counts_per_output_rev <= 0.0f || sample_period_ms == 0U) {
        sample.filtered_rpm = runtime.filtered_rpm;
        sample.valid = false;
        return sample;
    }

    const float sample_period_min = (float)sample_period_ms / 60000.0f;
    sample.actual_rpm = ((float)sample.delta_count / config.counts_per_output_rev) / sample_period_min;

    const float alpha = clampFilterAlpha(config.filter_alpha);
    runtime.filtered_rpm = alpha * sample.actual_rpm + (1.0f - alpha) * runtime.filtered_rpm;

    sample.filtered_rpm = runtime.filtered_rpm;
    sample.valid = true;
    return sample;
}
