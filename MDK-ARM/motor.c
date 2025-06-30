#include "motor.h"

void pwm_set_duty(TIM_HandleTypeDef* htim, uint16_t channel, float duty){
	uint32_t ccr = (duty * htim->Instance->ARR) / 100;
	switch(channel){
		case TIM_CHANNEL_1:
			htim->Instance->CCR1 = ccr;
			break;
		case TIM_CHANNEL_2:
			htim->Instance->CCR2 = ccr;
			break;
		case TIM_CHANNEL_3:
			htim->Instance->CCR3 = ccr;
			break;
		case TIM_CHANNEL_4:
			htim->Instance->CCR4 = ccr;
			break;
	}
}

void control_motor(Motor_TypeDef *motor){
	
	switch(motor->direct){
		case STOP:
			HAL_GPIO_WritePin(motor->IO_Port_1, motor->IO_Pin_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->IO_Port_2, motor->IO_Pin_2, GPIO_PIN_RESET);
			pwm_set_duty(motor->htim, motor->Tim_Channel, 0);
			break;
		case FORWARD:
			HAL_GPIO_WritePin(motor->IO_Port_1, motor->IO_Pin_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->IO_Port_2, motor->IO_Pin_2, GPIO_PIN_SET);
			pwm_set_duty(motor->htim, motor->Tim_Channel, motor->Speed);
			break;
		case BACKWARD:
			HAL_GPIO_WritePin(motor->IO_Port_1, motor->IO_Pin_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->IO_Port_2, motor->IO_Pin_2, GPIO_PIN_RESET);
			pwm_set_duty(motor->htim, motor->Tim_Channel, motor->Speed);
			break;
		default:
			break;
	}
}

void Motor_Setup(Motor_TypeDef *motor, Motor_Direct Direct, float speed){
	motor->direct = Direct;
	motor->Speed = speed;
	control_motor(motor);
}

void motor_init(Motor_TypeDef *motor, GPIO_TypeDef *_IO_Port_1, uint16_t _IO_Pin_1,
		GPIO_TypeDef *_IO_Port_2, uint16_t _IO_Pin_2, TIM_HandleTypeDef *_htim, uint32_t _Tim_Channel){
	motor->htim = _htim;
	motor->IO_Port_1 = _IO_Port_1;
	motor->IO_Pin_1 = _IO_Pin_1;
	motor->IO_Port_2 = _IO_Port_2;
	motor->IO_Pin_2 = _IO_Pin_2;
	motor->Tim_Channel = _Tim_Channel;
	HAL_TIM_PWM_Start(motor->htim, motor->Tim_Channel);
	Motor_Setup(motor, STOP, 0);
}



