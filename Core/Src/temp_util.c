#include "temp_util.h"
#include <stdint.h>
#include <math.h>
#include <float.h>

#define FLT_CMP(a, b) (fabsf((a) - (b)) < FLT_EPSILON)
void setPWMDutyCycle(TIM_HandleTypeDef *pwmHandle, uint32_t pwmChannel, uint32_t dutyCycle) {
	assert(pwmHandle != NULL);
	assert((dutyCycle >= 0) && (dutyCycle <= 100));

	/* reload register (ARR) -> Period*/
	uint32_t newRegVal;
	if (dutyCycle != 0)
		newRegVal = ((__HAL_TIM_GET_AUTORELOAD(pwmHandle) + 1) * dutyCycle) / 100 - 1;
	else
		newRegVal = 0;
	__HAL_TIM_SET_COMPARE(pwmHandle, pwmChannel, newRegVal);
}

float thermistor_temp(double analogValue) {
	if (FLT_CMP(analogValue, 4095.0f))
		return -273.15;
	double t;
	t = log((analogValue * 10000.0f) / (4095.0f - analogValue));
	t = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * t * t)) * t);
	t = t - 273.15;
	return (float) t;
}

float internal_temp(double temp_raw) {
	float sensor_vol = temp_raw * 3.3 / 4095.0;
	return (((temp_raw * 3300 / 4095) - 760) / 2.5) + 25.0;
}
