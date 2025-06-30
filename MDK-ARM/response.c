#include "response.h"
#include "car.h"
#include "string.h"
#include "stdio.h"

static char* message;
int8_t speed_left = 0;
int8_t speed_right = 0;
int8_t speed_front = 0;
extern Car_Direct direct;
extern uint8_t is_running;

int8_t get_speed(char* data){
	uint8_t flag = 0;
	uint8_t count_minus = 0;
	int8_t speed = 0;
	for(uint8_t k=0; k<strlen(data); k++){
		if(data[k] == '-'){
			if(count_minus >= 2 || (k > 0 && data[k-1] == '-')){
				direct = INVALID;
				return 0;
			}
			flag = 1;
			continue;
		}
		speed = speed * 10 + (data[k] - '0');
	}
	if(flag) speed *= (-1);
	return speed;
}

void get_info_cmd(char** argv){
	speed_front = get_speed(argv[0]);
	speed_left = get_speed(argv[1]);
	speed_right = get_speed(argv[2]);
	if(speed_left == 0 && speed_right == 0 && speed_front == 0){
		direct = CAR_STOP;
	}else{
		direct = CAR_RUN;
	}
	return;
}

char* response_uart(char** argv){
	get_info_cmd(argv);
	if(speed_left > 100 || speed_right > 100 || speed_front > 100 || direct == INVALID) return "Command Error\n";
	message = "Done\n";
	return message;
}
