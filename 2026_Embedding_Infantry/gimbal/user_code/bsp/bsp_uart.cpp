#include "bsp_uart.h"

void UART_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, Rx_Buffer_Size);

    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}


void UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    if(huart != NULL  && Data != NULL && Length > 0) 
	{
        HAL_UART_Transmit_DMA(huart, Data, Length);
       //HAL_UART_Transmit_IT(huart, Data, Length);
   }
}

