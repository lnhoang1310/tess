#include "car.h"
#include "math.h"

extern Motor_TypeDef motor_front;
extern Motor_TypeDef motor_left;
extern Motor_TypeDef motor_right;
void car_init(void){
	car_control(CAR_STOP, 0, 0, 0);
}

void car_control(Car_Direct _direct, float speed_front, float speed_left, float speed_right){
	switch(_direct){
		case CAR_STOP:
			Motor_Setup(&motor_front, STOP, 0);
			Motor_Setup(&motor_left, STOP, 0);
			Motor_Setup(&motor_right, STOP, 0);
			break;
		case CAR_RUN:
			Motor_Setup(&motor_front, (speed_front > 0) ? FORWARD : BACKWARD, (speed_front > 0) ? speed_front : (speed_front * (-1)));
			Motor_Setup(&motor_left, (speed_left > 0) ? FORWARD : BACKWARD, (speed_left > 0) ? speed_left : (speed_left * (-1)));
			Motor_Setup(&motor_right, (speed_right > 0) ? FORWARD : BACKWARD, (speed_right > 0) ? speed_right : (speed_right * (-1)));
			break;
		default:
			break;
	}
}
