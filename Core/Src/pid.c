#include "pid.h"

float PIDUpdate(PID *pid, float setpoint, float measurement) {
	float error = setpoint - measurement;
	float proportional = pid->Kp * error;

	pid->integrator = pid->integrator
			+ 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	// Anti-wind-up via integrator clamping
	if (pid->integrator > pid->limMaxInt)
		pid->integrator = pid->limMaxInt;
	else if (pid->integrator < pid->limMinInt)
		pid->integrator = pid->limMinInt;

	// Derivative with filter (band-limited differentiator)
	pid->differentiator = -(2.0f * pid->Kd
			* (measurement - pid->prevMeasurement) // ! derivative on measurement -> minus sign in front of equation !
	+ (2.0f * pid->tau - pid->T) * pid->differentiator)
			/ (2.0f * pid->tau + pid->T);

	float out = proportional + pid->integrator + pid->differentiator;

	// Output saturation
	if (out > pid->limMax)
		out = pid->limMax;
	else if (out < pid->limMin)
		out = pid->limMin;

	pid->prevError = error;
	pid->prevMeasurement = measurement;

	return out;
}
