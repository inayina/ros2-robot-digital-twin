#ifndef __LED_ALARM_H
#define __LED_ALARM_H

#include "main.h"

void LED_Alarm_Init(void);
void LED_Alarm_SetState(uint8_t state);
void LED_Alarm_Update(void);
uint8_t LED_Alarm_GetState(void);

#endif