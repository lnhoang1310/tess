#ifndef UART_H
#define UART_H
#include "stm32f4xx_hal.h"

void uart_receive_data(uint8_t data_rx);
void uart_handle(void);

#endif
