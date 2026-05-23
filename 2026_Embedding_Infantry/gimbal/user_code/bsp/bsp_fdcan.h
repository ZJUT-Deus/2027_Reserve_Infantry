#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H
#include "main.h"
#include "fdcan.h"

// 添加调试变量声明
extern volatile uint32_t send_failure_count;
extern volatile uint32_t success_send_count;

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint32_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
void FDCAN_Reactivate(FDCAN_HandleTypeDef *hfdcan);

__weak void fdcan1_rx_callback(void);
__weak void fdcan2_rx_callback(void);
__weak void fdcan3_rx_callback(void);

#endif 

