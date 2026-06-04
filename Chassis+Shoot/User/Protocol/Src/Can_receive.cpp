/**
 * @file    Can_receive.cpp
 * @brief   CAN总线数据接收与解析实现 (DM3519/C610/GM6020 电机)
 * @author  kk
 * @date    2026-05-22
 */

#include "Can_receive.h"
#include <cstring>

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

/**
 * @brief  解析 C610 电机 CAN 反馈数据帧 (8字节)
 * @param  motor C610 测量数据结构体指针
 * @param  data  CAN 接收数据 (8 字节)
 */
void Can_receive::get_c610_motor_measure(c610_motor_measure_t *motor, uint8_t data[8])
{
    motor->angle          = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm      = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->torque_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temp           = data[6];
}

/**
 * @brief  获取拨弹电机测量数据指针
 * @return C610 测量数据结构体指针
 */
const c610_motor_measure_t *Can_receive::get_c610_motor_measure_point(void)
{
    return &trigger_motor_measure;
}

/**
 * @brief  C610 拨弹电机电流控制指令
 * @note   控制帧 0x200(电机1-4)/0x1FF(电机5-8), 每电机2字节 int16
 *         当前电机 ID=2 → 控制帧 0x200, DATA[2-3]; 其余置零
 * @param  current 目标电流值 (-10000 ~ +10000)
 */
void Can_receive::can_cmd_c610_motor(int16_t current)
{
    std::memset(can_send_data, 0, sizeof(can_send_data));

    can_send_data[2] = (uint8_t)((current >> 8) & 0xFF);
    can_send_data[3] = (uint8_t)(current & 0xFF);

    fdcanx_send_data(&TOP_CAN, C610_CONTROL_TRIGGER_ID, can_send_data, 8);
}

/**
 * @brief  解析 DJI GM6020 电机 CAN 反馈数据帧 (8字节)
 * @param  motor GM6020 测量数据结构体指针
 * @param  data  CAN 接收数据 (8 字节)
 */
void Can_receive::get_dji_motor_measure(dji_motor_measure_t *motor, uint8_t data[8])
{
    motor->last_ecd       = motor->ecd;
    motor->ecd            = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm      = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current  = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperate      = data[6];
    motor->last_update_ms = HAL_GetTick();
    motor->online         = 1U;
}

/**
 * @brief  获取云台 GM6020 测量数据指针
 * @param  i 电机索引 (0: yaw, 1: pitch)
 * @return GM6020 测量数据结构体指针
 */
const dji_motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor_measure[i];
}

/**
 * @brief  GM6020 云台电机电流控制指令
 * @param  yaw_current   yaw 电机电流, ID=1, 发送到 0x1FF 第 1 路
 * @param  pitch_current pitch 电机电流, ID=5, 发送到 0x2FF 第 1 路
 */
void Can_receive::can_cmd_gimbal_motor(int16_t yaw_current, int16_t pitch_current)
{
    uint8_t data_1_4[8] = {0};
    uint8_t data_5_7[8] = {0};

    data_1_4[0] = (yaw_current >> 8) & 0xFF;
    data_1_4[1] = yaw_current & 0xFF;

    data_5_7[0] = (pitch_current >> 8) & 0xFF;
    data_5_7[1] = pitch_current & 0xFF;

    fdcanx_send_data(&TOP_CAN, GIMBAL_CMD_CAN_ID_1_4, data_1_4, sizeof(data_1_4));
    fdcanx_send_data(&TOP_CAN, GIMBAL_CMD_CAN_ID_5_7, data_5_7, sizeof(data_5_7));
}
