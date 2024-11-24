#ifndef __TEMP_UTIL_H
#define __TEMP_UTIL_H

#include "tim.h"
#include <stdint.h>

void setPWMDutyCycle(TIM_HandleTypeDef *pwmHandle, uint32_t pwmChannel,
		uint32_t dutyCycle);

float thermistor_temp(double analogValue);

float internal_temp(double temp_raw);

#endif // __TEMP_UTIL_H
