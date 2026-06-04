/**
 * @file    Motor.cpp
 * @brief   电机设备驱动模块实现 (DM3519)
 * @author  kk
 * @date    2026-05-23
 */

#include "Motor.h"

/**
 * @brief  Motor 默认构造函数, 初始化成员变量
 * @retval none
 */
Motor::Motor() : can_id(0)
{
    speed = 0.0f;
    speed_set = 0.0f;
    current_t = 0.0f;
    current_give = 0.0f;
}

/**
 * @brief  Motor 构造函数, 初始化成员变量
 * @param  id 电机 CAN 反馈 ID
 * @retval none
 */
Motor::Motor(uint16_t id) : can_id(id)
{
    speed = 0.0f;
    speed_set = 0.0f;
    current_t = 0.0f;
    current_give = 0.0f;
}

/**
 * @brief  设置电机目标值
 * @param  set  设定值 (速度环: rad/s)
 * @param  mode 控制模式 (SPEED)
 * @retval none
 */
void Motor::set(fp32 set, uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        speed_set = set;
        break;
    default:
        break;
    }
}

/**
 * @brief  执行 PID 计算, 输出力矩指令
 * @param  mode 控制模式 (SPEED)
 * @retval none
 */
void Motor::solve(uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        current_t = speed_pid.pid_calc();
        current_give = current_t;
        break;
    default:
        break;
    }
}

/**
 * @brief  DM3519 电机参数初始化
 * @retval none
 */
void DM3519::DM3519_Init()
{
    tmp.pmax = 3.14159f;
    tmp.vmax = 200.0f;
    tmp.tmax = 10.0f;
    ctrl.mode = MODE;
}

/**
 * @brief  从 CAN 测量数据更新电机速度
 * @retval none
 */
void DM3519::update_measure()
{
    speed = uint_to_float(measure->v_int, -tmp.vmax, tmp.vmax, 12);
}

/**
 * @brief  C615 初始化, 绑定PWM通道并输出中立脉宽
 * @param  channel TIM通道号
 */
void C615::init(uint32_t channel)
{
    tim_channel = channel;
    max_speed = 500.0f;
    pwm_set_pulse(channel, C615_PULSE_MID);
}

/**
 * @brief  C615 无CAN反馈, 速度直接取设定值 (开环)
 */
void C615::update_measure()
{
    speed = speed_set;
}

/**
 * @brief  C610 初始化, 绑定CAN ID和反馈数据指针
 * @param  id  电机 CAN ID
 * @param  m   C610 反馈数据指针
 */
void C610::init(uint16_t id, const c610_motor_measure_t *m)
{
    can_id  = id;
    measure = m;
}

/**
 * @brief  从 C610 CAN 反馈更新电机速度 (rpm -> rad/s)
 */
void C610::update_measure()
{
    speed = (fp32)measure->speed_rpm * 0.104719755f; // rpm * 2*PI/60
}

