/**
 * @file    Can_receive.cpp
 * @brief   CAN总线数据接收与解析实现 (DM3519 电机)
 * @author  kk
 * @date    2026-05-22
 */

#include "Can_receive.h"

Can_receive can_receive;

/**
 * @brief  解析电机反馈数据帧
 * @param  motor 电机测量数据结构体指针
 * @param  data  CAN 接收数据 (8 字节)
 */
void Can_receive::get_chassis_motor_measure(dm_motor_measure_t *motor, uint8_t data[8])
{
    motor->id    = data[0] & 0x0F;
    motor->err   = data[0] >> 4;
    motor->p_int = (data[1] << 8) | data[2];
    motor->v_int = (data[3] << 4) | (data[4] & 0x0F);
    motor->t_int = ((data[4] >> 4) << 8) | data[5];
    motor->Tmos  = (float)(data[6]);
    motor->Tcoil = (float)(data[7]);
}

/**
 * @brief  DM3519 电机 MIT 模式控制指令
 * @param  pos 位置设定值
 * @param  vel 速度设定值
 * @param  Kp  位置环比例系数
 * @param  Kd  速度环微分系数
 * @param  tor 前馈力矩值
 * @param  id  电机 CAN ID
 * @param  tmp 电机参数限制 (pmax/vmax/tmax)
 */
void Can_receive::can_cmd_mit_dm_motor(fp32 pos, fp32 vel, fp32 Kp, fp32 Kd, fp32 tor, uint16_t id, esc_inf_t tmp)
{
    uint16_t pos_int = float_to_uint(pos, -tmp.pmax, tmp.pmax, 16);
    uint16_t vel_int = float_to_uint(vel, -tmp.vmax, tmp.vmax, 12);
    uint16_t Kp_int  = float_to_uint(Kp, 0.0f, 500.0f, 12);
    uint16_t Kd_int  = float_to_uint(Kd, 0.0f, 5.0f, 12);
    uint16_t tor_int = float_to_uint(tor, -tmp.tmax, tmp.tmax, 12);

    can_send_data[0] = (pos_int >> 8) & 0xFF;
    can_send_data[1] = pos_int & 0xFF;
    can_send_data[2] = (vel_int >> 4) & 0xFF;
    can_send_data[3] = ((vel_int & 0x0F) << 4) | ((Kp_int >> 8) & 0x0F);
    can_send_data[4] = Kp_int & 0xFF;
    can_send_data[5] = (Kd_int >> 4) & 0xFF;
    can_send_data[6] = ((Kd_int & 0x0F) << 4) | ((tor_int >> 8) & 0x0F);
    can_send_data[7] = tor_int & 0xFF;

    fdcanx_send_data(&CHASSIS_CAN, id, can_send_data, sizeof(can_send_data));
}

/**
 * @brief  DM3519 电机使能/失能控制
 * @param  id    电机 CAN ID
 * @param  state ENABLE: 使能, DISABLE: 失能
 */
void Can_receive::CTRL_DM3519(uint16_t id, uint16_t state)
{
    switch (state)
    {
    case ENABLE:
        can_send_data[0] = 0xFF;
        can_send_data[1] = 0xFF;
        can_send_data[2] = 0xFF;
        can_send_data[3] = 0xFF;
        can_send_data[4] = 0xFF;
        can_send_data[5] = 0xFF;
        can_send_data[6] = 0xFF;
        can_send_data[7] = 0xFC;
        fdcanx_send_data(&CHASSIS_CAN, id, can_send_data, sizeof(can_send_data));
        break;
    case DISABLE:
        can_send_data[0] = 0xFF;
        can_send_data[1] = 0xFF;
        can_send_data[2] = 0xFF;
        can_send_data[3] = 0xFF;
        can_send_data[4] = 0xFF;
        can_send_data[5] = 0xFF;
        can_send_data[6] = 0xFF;
        can_send_data[7] = 0xFD;
        fdcanx_send_data(&CHASSIS_CAN, id, can_send_data, sizeof(can_send_data));
        break;
    default:
        break;
    }
}

/**
 * @brief  获取指定电机测量数据指针
 * @param  i 电机索引 (0~3: FR, FL, BL, BR)
 * @return 电机测量数据结构体指针
 */
const dm_motor_measure_t *Can_receive::get_chassis_motor_measure_point(uint8_t i)
{
    return &chassis_motor_measure[i];
}
