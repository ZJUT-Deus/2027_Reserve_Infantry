/**
 * @file    Communicate.cpp
 * @brief   底盘 CAN 通信模块实现 (电机反馈解析、遥控器数据处理)
 * @author  kk
 * @date    2026-05-25
 */

#include "Communicate.h"
#include "Can_receive.h"
#include "Shoot.h"
#include <cstring>

Communicate communicate;

static uint8_t  rx_data[8] = {0};
static uint16_t rx_id;
static uint8_t  rx2_data[8] = {0};
static uint16_t rx2_id;

/**
 * @brief  通信模块初始化
 */
void Communicate::init()
{
}

/**
 * @brief  将遥控器通道值线性映射为底盘速度指令
 * @param  rc_val      遥控器通道值
 * @param  rc_max      遥控器通道最大值
 * @param  chassis_max 底盘速度最大值
 * @return 映射后的底盘速度
 */
fp32 Communicate::map_speed(fp32 rc_val, fp32 rc_max, fp32 chassis_max)
{
    return (rc_val / rc_max) * chassis_max;
}

/**
 * @brief  处理遥控器数据, 根据在线状态和模式控制底盘运动
 */
void Communicate::handle_rc()
{
    i6x.update_online(HAL_GetTick());

    if (i6x.i6x_rc_ctrl.online == 0)
    {
        chassis_stop();
    }
    else if (i6x.chassis_cmd.enable == 0)
    {
        chassis_stop();
    }
    else if (i6x.chassis_cmd.mode == I6X_CHASSIS_FREE)
    {
        fp32 vx = map_speed(i6x.chassis_cmd.vx,
                            I6X_CHASSIS_MAX_VX, NORMAL_MAX_CHASSIS_SPEED_X);
        fp32 vy = map_speed(i6x.chassis_cmd.vy,
                            I6X_CHASSIS_MAX_VY, NORMAL_MAX_CHASSIS_SPEED_Y);

        chassis_set_velocity(vx, vy, 0.0f);
    }
    else if (i6x.chassis_cmd.mode == I6X_CHASSIS_TOP)
    {
        fp32 vx = map_speed(i6x.chassis_cmd.vx,
                            I6X_CHASSIS_MAX_VX, NORMAL_MAX_CHASSIS_SPEED_X);
        fp32 vy = map_speed(i6x.chassis_cmd.vy,
                            I6X_CHASSIS_MAX_VY, NORMAL_MAX_CHASSIS_SPEED_Y);

        chassis_set_spin(vx, vy, SPIN_WZ_SPEED);
    }
    else
    {
        chassis_stop();
    }

    /* ---- 射击控制 (SW_A=摩擦轮, SW_B=连发射击, 底盘无力时强制停止) ---- */
    if (i6x.i6x_rc_ctrl.online == 0 || i6x.chassis_cmd.enable == 0)
    {
        shoot_stop();
    }
    else if (i6x_2pos_switch_is_up(i6x.i6x_rc_ctrl.rc.s[I6X_SW_A])
          && i6x_2pos_switch_is_up(i6x.i6x_rc_ctrl.rc.s[I6X_SW_B]))
    {
        shoot_continue_bullet();
    }
    else if (i6x_2pos_switch_is_up(i6x.i6x_rc_ctrl.rc.s[I6X_SW_A]))
    {
        shoot_ready();
    }
    else
    {
        shoot_stop();
    }
}

/**
 * @brief  FDCAN 接收 FIFO 回调, 根据 CAN 外设分发到对应的回调函数
 * @param  hfdcan      FDCAN 句柄
 * @param  RxFifo0ITs  RX FIFO0 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan == &hfdcan1)
        {
            fdcan1_rx_callback();
        }
        if (hfdcan == &hfdcan2)
        {
            fdcan2_rx_callback();
        }
    }
}

/**
 * @brief  CAN 接收回调, 根据 ID 分发到对应电机测量数据结构
 */
void fdcan1_rx_callback(void)
{
    std::memset(rx_data, 0, sizeof(rx_data));
    if (fdcanx_receive(&CHASSIS_CAN, rx_data, &rx_id) == 0)
        return;

    switch (rx_id)
    {
    case CHASSIS_FR_MST_ID:
        can_receive.get_chassis_motor_measure(&can_receive.chassis_motor_measure[0], rx_data);
        break;
    case CHASSIS_FL_MST_ID:
        can_receive.get_chassis_motor_measure(&can_receive.chassis_motor_measure[1], rx_data);
        break;
    case CHASSIS_BR_MST_ID:
        can_receive.get_chassis_motor_measure(&can_receive.chassis_motor_measure[2], rx_data);
        break;
    case CHASSIS_BL_MST_ID:
        can_receive.get_chassis_motor_measure(&can_receive.chassis_motor_measure[3], rx_data);
        break;
    default:
        break;
    }
}

/**
 * @brief  FDCAN2 接收回调, 处理 C610 拨弹电机反馈数据
 */
void fdcan2_rx_callback(void)
{
    std::memset(rx2_data, 0, sizeof(rx2_data));
    if (fdcanx_receive(&TOP_CAN, rx2_data, &rx2_id) == 0)
        return;

    switch (rx2_id)
    {
    case C610_FEEDBACK_TRIGGER_ID:
        can_receive.get_c610_motor_measure(&can_receive.trigger_motor_measure, rx2_data);
        break;
    default:
        break;
    }
}

/**
 * @brief  CAN Tx 发送数据
 * @param  id   CAN 标准 ID
 * @param  data 发送数据缓冲区
 * @param  len  数据长度 (字节)
 * @return 0: 发送成功, 1: 发送失败
 */
uint8_t communicate_send(uint16_t id, uint8_t *data, uint32_t len)
{
    return fdcanx_send_data(&CHASSIS_CAN, id, data, len);
}
