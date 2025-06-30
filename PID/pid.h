#ifndef PID_H
#define PID_H

#include "stdint.h"

//#define KP 2.0f
//#define KI 0.3f
//#define KD 0.1f

#define KP 1.0f
#define KI 0.3f
#define KD 0.075f
#define SAMPLE_TIME 0.1f

typedef struct{
	float setpoint;
	float actual;
	float error;
	float last_error;
	float intergal;
	float derivative;
	float output;
	float output_prev;
	float kp, ki, kd;
} PID_Typedef;

void PID_Init(PID_Typedef* pid, float kp, float ki, float kd);
float PID_Calculate(PID_Typedef* pid, float _setpoint, float _actual);
float soft_start(float pwm_output, uint32_t start_time, float max_pwm, float ramp_time_ms, uint8_t* flag);
void PID_Reset(PID_Typedef* pid);

#endif
