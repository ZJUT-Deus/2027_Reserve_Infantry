/**
 * @file    bsp_fdcan.h
 * @brief   FDCAN总线底层驱动接口
 * @author  kk
 * @date    2026-05-22
 */

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint32_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf, uint16_t *id);
void FDCAN_Reactivate(FDCAN_HandleTypeDef *hfdcan);

__weak void fdcan1_rx_callback(void);
__weak void fdcan2_rx_callback(void);

#ifdef __cplusplus
}
#endif

#endif
