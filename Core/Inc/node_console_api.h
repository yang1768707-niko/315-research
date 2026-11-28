#ifndef NODE_CONSOLE_API_H
#define NODE_CONSOLE_API_H
#include <stdint.h>
uint8_t NodeConsole_UART_IRQHandler(void); /* 1: RX and HAL TX serviced; 0: use HAL fallback. */
#endif
