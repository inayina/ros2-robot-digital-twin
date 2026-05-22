#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

#include "motor/motor_control_shared.h"
#include "motor/motor_response_model.h"

struct MotorControllerRuntime {
    MotorResponseModelRuntime response_model;
};

void motorControllerInit(MotorControllerRuntime& runtime);
MotorControlStateSnapshot motorControllerUpdateMock(
    MotorControllerRuntime& runtime,
    const MotorControlCommandSnapshot& command,
    uint32_t now_ms,
    uint32_t loop_count);

void motorControllerSetHardwareOutputsEnabled(bool enabled);
bool motorControllerHardwareOutputsEnabled();
void motorControllerApplyHardwareOutputs(const MotorControlStateSnapshot& state);
float motorControllerReadEncoderRpm();

#endif
