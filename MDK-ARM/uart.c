#include "uart.h"
#include "string.h"
#include "response.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim5;
extern uint8_t uart_buff[40];
uint8_t flag_cpltReceive = 0;
uint8_t buffer_index = 0;
char* message;
extern int8_t speed_left;
extern int8_t speed_right;
extern int8_t speed_front;
extern uint32_t start_time_front, start_time_left, start_time_right;
extern uint8_t is_starting_front, is_starting_left, is_starting_right;
extern float v_front, v_left, v_right;

void uart_receive_data(uint8_t data_rx){
	if(data_rx == '\n'){
		flag_cpltReceive = 1;
		uart_buff[buffer_index] = '\0';
	}else{
		if(data_rx != 13){
			uart_buff[buffer_index++] = data_rx;
		}
	}
		
}

void uart_handle(){
	if(flag_cpltReceive){
		char* argv[15];
		uint8_t index = 0;
		char *token = strtok((char*)uart_buff, " ");
		while(token != NULL){
			argv[index++] = token;
			token = strtok(NULL, " ");
		}
		message = response_uart(argv);
		if (speed_front != 0 && v_front == 0 && !is_starting_front) {
			start_time_front = __HAL_TIM_GET_COUNTER(&htim5);
			is_starting_front = 1;
		}
		if (speed_left != 0 && v_left == 0 && !is_starting_left) {
			start_time_left = __HAL_TIM_GET_COUNTER(&htim5);
			is_starting_left = 1;
		}
		if (speed_right != 0 && v_right == 0 && !is_starting_right) {
			start_time_right = __HAL_TIM_GET_COUNTER(&htim5);
			is_starting_right = 1;
		}
		//HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
		flag_cpltReceive = 0;
		buffer_index = 0;
	}
}
