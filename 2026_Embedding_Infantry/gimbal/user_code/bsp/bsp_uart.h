#ifndef DRV_UART_H
#define DRV_UART_H
#include "usart.h"
#include "struct_typedef.h"

void UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);
void UART_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size);


#endif


