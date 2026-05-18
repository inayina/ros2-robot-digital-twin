#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor/encoder_rpm_estimator.h"

namespace {

void assertNear(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

void test_first_sample_initializes_without_rpm() {
    EncoderRpmEstimatorRuntime runtime = {};
    encoderRpmEstimatorInit(runtime);

    EncoderRpmEstimatorConfig config = {};
    config.counts_per_output_rev = 40.0f;
    config.filter_alpha = 0.5f;

    const EncoderRpmEstimatorSample sample =
        encoderRpmEstimatorUpdate(runtime, config, 100, 100U);

    assert(!sample.valid);
    assert(sample.encoder_count == 100);
    assert(sample.delta_count == 0);
    assertNear(sample.filtered_rpm, 0.0f, 0.001f);
}

void test_converts_delta_count_to_rpm() {
    EncoderRpmEstimatorRuntime runtime = {};
    encoderRpmEstimatorInit(runtime);

    EncoderRpmEstimatorConfig config = {};
    config.counts_per_output_rev = 40.0f;
    config.filter_alpha = 1.0f;

    encoderRpmEstimatorUpdate(runtime, config, 100, 100U);
    const EncoderRpmEstimatorSample sample =
        encoderRpmEstimatorUpdate(runtime, config, 120, 100U);

    assert(sample.valid);
    assert(sample.delta_count == 20);
    assert(sample.direction == 1);
    assertNear(sample.actual_rpm, 300.0f, 0.001f);
    assertNear(sample.filtered_rpm, 300.0f, 0.001f);
}

void test_negative_delta_reports_reverse_direction() {
    EncoderRpmEstimatorRuntime runtime = {};
    encoderRpmEstimatorInit(runtime);

    EncoderRpmEstimatorConfig config = {};
    config.counts_per_output_rev = 60.0f;
    config.filter_alpha = 1.0f;

    encoderRpmEstimatorUpdate(runtime, config, 300, 100U);
    const EncoderRpmEstimatorSample sample =
        encoderRpmEstimatorUpdate(runtime, config, 270, 100U);

    assert(sample.valid);
    assert(sample.delta_count == -30);
    assert(sample.direction == -1);
    assertNear(sample.actual_rpm, -300.0f, 0.001f);
}

}  // namespace

int main() {
    test_first_sample_initializes_without_rpm();
    test_converts_delta_count_to_rpm();
    test_negative_delta_reports_reverse_direction();

    puts("host encoder_rpm_estimator tests passed");
    return 0;
}
