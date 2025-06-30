#include "pid.h"
#include "main.h"
#include "math.h"

extern TIM_HandleTypeDef htim5;
Filter_State filter_pwm = {0};

void PID_Init(PID_Typedef* pid, float _kp, float _ki, float _kd){
	pid->kp = _kp;
	pid->ki = _ki;
	pid->kd = _kd;
}

float PID_Calculate(PID_Typedef* pid, float _setpoint, float _actual){
	pid->setpoint = _setpoint;
	pid->actual = _actual;
	pid->error = _setpoint - _actual;
	
	pid->intergal += pid->error * SAMPLE_TIME;
	pid->derivative = (pid->error - pid->last_error) / SAMPLE_TIME;
	pid->output = pid->kp * pid->error + pid->ki * pid->intergal + pid->kd * pid->derivative;
	
	if(pid->output > 1) pid->output = 1;
	if(pid->output < 0) pid->output = 0;
	
	if(pid->output - pid->output_prev >= 5.0f){
		pid->output = pid->output_prev + 5.0f;
	}
	
	pid->output_prev = pid->output;
	pid->last_error = pid->error;
	
	return pid->output;
}

void PID_Reset(PID_Typedef* pid){
	pid->actual = 0;
	pid->derivative = 0;
	pid->error = 0;
	pid->intergal = 0;
	pid->last_error = 0;
	pid->output = 0;
}

float soft_start(float pwm_output, uint32_t start_time, float max_pwm, float ramp_time_ms, uint8_t* flag) {
    // pwm_output: Giá tr? PWM t? PID (t? -100 d?n 100)
    // start_time: Th?i di?m b?t d?u ch?y (t? TIM5 counter)
    // max_pwm: Giá tr? PWM t?i da (100 cho ph?n tram)
    // ramp_time_ms: Th?i gian tang t?c (mili giây)

    // L?y th?i gian hi?n ti t? TIM5 
    uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim5);
    // V?i PSC = 9, m?i tick = 0.1 µs, 1 ms = 10000 tick
    float elapsed_time_ms = (current_time >= start_time) ? 
                           (current_time - start_time) / 10000.0f :
                           (4294967296ULL - start_time + current_time) / 10000.0f;

    // N?u v?n trong giai do?n ramp
    if (elapsed_time_ms < ramp_time_ms) {
        // Tính t? l? ramp (0 d?n 1)
        float ramp_ratio = elapsed_time_ms / ramp_time_ms;
        // Gi?i h?n PWM theo t? l? ramp
		float out = pwm_output * ramp_ratio;
        return (out > 30.0f) ? 30.0f : out;
    } else {
        // Sau giai do?n ramp, tr? v? PWM g?c
		*flag = 0;
        //return filter(&filter_pwm, pwm_output, 100.0f);
		return pwm_output;
    }
}
