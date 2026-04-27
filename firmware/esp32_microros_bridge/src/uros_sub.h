#ifndef UROS_SUB_H
#define UROS_SUB_H

#include <Arduino.h>
#include <stdint.h>

typedef void (*UrosCmdVelCallback)(float linear_x, float angular_z);

bool urosSubInit(Print& log);
void urosSubFini(Print& log);
void urosSubSetCmdVelCallback(UrosCmdVelCallback callback);
void urosSubSpinSome(uint32_t timeout_ms);

#endif
