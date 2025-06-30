#ifndef MOTOR_H_
#define MOTOR_H_
#include "stm32f4xx.h"
typedef enum{
	STOP,
	FORWARD, 
	BACKWARD,
}Motor_Direct;

typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t Tim_Channel;
	GPIO_TypeDef *IO_Port_1;
	uint16_t IO_Pin_1;
	GPIO_TypeDef *IO_Port_2;
	uint16_t IO_Pin_2;
	float Speed;
	Motor_Direct direct;
}Motor_TypeDef;

void Motor_Setup(Motor_TypeDef *motor, Motor_Direct Direct, float speed);
void motor_init(Motor_TypeDef *motor, GPIO_TypeDef *IO_Port_1, uint16_t IO_Pin_1,
		GPIO_TypeDef *IO_Port_2, uint16_t IO_Pin_2, TIM_HandleTypeDef *htim, uint32_t Tim_Channel);
#endif
