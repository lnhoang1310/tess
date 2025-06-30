#ifndef CAR_H
#define CAR_H

#include "stm32f4xx.h"
#include "motor.h"

typedef enum{
	CAR_STOP,
	CAR_RUN,
	INVALID
}Car_Direct;

void car_init(void);
void car_control(Car_Direct _direct, float speed_front, float speed_left, float speed_right);

#endif
