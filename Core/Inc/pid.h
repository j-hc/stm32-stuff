#ifndef __PID_H
#define __PID_H

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	// Derivative low-pass filter time constant
	float tau;

	// Output limits
	float limMin;
	float limMax;

	// Integrator limits
	float limMinInt;
	float limMaxInt;

	// Sample time
	float T;

	float integrator;
	float prevError; // for integrator
	float differentiator;
	float prevMeasurement; // for differentiator
} PID;

float PIDUpdate(PID *pid, float setpoint, float measurement);

#endif // __PID_H
