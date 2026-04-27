#ifndef __TB6612_H
#define __TB6612_H

#include "main.h"

uint8_t TB6612_Init(void);
void TB6612_SetStandby(uint8_t enabled);
void TB6612_MotorA_SetDuty(int16_t duty);
void TB6612_MotorA_Stop(void);
void TB6612_EmergencyStop(void);
uint16_t TB6612_MotorA_GetMaxDuty(void);

#endif
