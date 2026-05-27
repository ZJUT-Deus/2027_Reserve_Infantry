/**
 * @file    bsp_fdcan.cpp
 * @brief   FDCAN总线底层驱动实现
 * @author  kk
 * @date    2026-05-22
 */

#include "bsp_fdcan.h"

/**
 * @brief  配置FDCAN的接收过滤器 (标准ID 0x000~0x7FF 存入FIFO0)
 * @param  hfdcan FDCAN句柄
 */
static void fdcan_config_filter(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x000;
    fdcan_filter.FilterID2 = 0x7FF;

    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
}

/**
 * @brief  初始化CAN总线, 启动FDCAN1/FDCAN2并配置中断
 */
void can_bsp_init(void)
{
    can_filter_init();

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
}

/**
 * @brief  配置CAN滤波器 (标准ID 0x000~0x7FF, 存入FIFO0)
 */
void can_filter_init(void)
{
    fdcan_config_filter(&hfdcan1);
    fdcan_config_filter(&hfdcan2);
}

/**
 * @brief  通过FDCAN发送数据
 * @param  hfdcan FDCAN句柄
 * @param  id     CAN标准ID
 * @param  data   发送数据缓冲区
 * @param  len    数据长度 (字节)
 * @return 0: 发送成功, 1: 发送失败
 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = len;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief  从FDCAN接收数据
 * @param  hfdcan FDCAN句柄
 * @param  buf    接收数据缓冲区
 * @param  id     输出参数, 接收到的消息ID
 * @return 实际接收的数据长度, 0表示无数据
 */
uint32_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf, uint16_t *id)
{
    FDCAN_RxHeaderTypeDef fdcan_RxHeader;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf) != HAL_OK)
        return 0;

    if (id)
        *id = fdcan_RxHeader.Identifier;

    return fdcan_RxHeader.DataLength;
}

/**
 * @brief  FDCAN1 接收回调弱定义, 由 Communicate 模块重写
 */
__weak void fdcan1_rx_callback(void)
{
}

/**
 * @brief  FDCAN2 接收回调弱定义, 由 Communicate 模块重写
 */
__weak void fdcan2_rx_callback(void)
{
}

/**
 * @brief  重新激活FDCAN外设 (用于Bus-Off恢复)
 * @param  hfdcan FDCAN句柄
 */
void FDCAN_Reactivate(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Stop(hfdcan);

    fdcan_config_filter(hfdcan);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);

    HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief  FDCAN错误状态回调, 自动处理Bus-Off恢复
 * @param  hfdcan        FDCAN句柄
 * @param  ErrorStatusITs 错误中断标志
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
        FDCAN_Reactivate(hfdcan);
    }
}
