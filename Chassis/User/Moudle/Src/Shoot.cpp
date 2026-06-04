/**
 * @file    Shoot.cpp
 * @brief   射击控制模块实现 (C615摩擦轮 + C610拨弹)
 * @author  kk
 * @date    2026-06-04
 */

#include "Shoot.h"
#include "bsp_pwm.h"
#include "pid.h"

Shoot shoot;

static float SHOOT_FRICTION_SPEED_PID[6] = {
    10.0f, 0.0f, 0.0f,   // Kp, Ki, Kd
    0.0f,                 // Kf
    5000.0f, 5000.0f      // max_iout, max_out
};

/**
 * @brief  射击模块初始化, 配置双摩擦轮 C615
 */
void Shoot::init()
{
    friction_left.init(PWM_C615_FRICTION_L);
    friction_left.speed_pid.init(PID_SPEED, SHOOT_FRICTION_SPEED_PID,
                                 &friction_left.speed,
                                 &friction_left.speed_set, 0.0f);
    friction_left.speed_pid.pid_clear();

    friction_right.init(PWM_C615_FRICTION_R);
    friction_right.speed_pid.init(PID_SPEED, SHOOT_FRICTION_SPEED_PID,
                                  &friction_right.speed,
                                  &friction_right.speed_set, 0.0f);
    friction_right.speed_pid.pid_clear();
}

/**
 * @brief  更新电机测量数据
 */
void Shoot::feedback_update()
{
    friction_left.update_measure();
    friction_right.update_measure();
}

/**
 * @brief  设置控制量: 双摩擦轮同速旋转
 */
void Shoot::set_control()
{
    friction_left.speed_set  =  SHOOT_TEST_SPEED;
    friction_right.speed_set = -SHOOT_TEST_SPEED;
}

/**
 * @brief  执行速度环 PID 计算
 */
void Shoot::solve()
{
    friction_left.solve(SPEED);
    friction_right.solve(SPEED);
}

/**
 * @brief  将速度映射为 PWM 脉宽输出到 C615 电调
 */
void Shoot::output()
{
    fp32 ratio_l = fp32_constrain(friction_left.speed_set / friction_left.max_speed, -1.0f, 1.0f);
    fp32 ratio_r = fp32_constrain(friction_right.speed_set / friction_right.max_speed, -1.0f, 1.0f);

    uint16_t pulse_l = (uint16_t)(C615_PULSE_MID + ratio_l * 500.0f);
    uint16_t pulse_r = (uint16_t)(C615_PULSE_MID + ratio_r * 500.0f);

    pwm_set_pulse(friction_left.tim_channel, pulse_l);
    pwm_set_pulse(friction_right.tim_channel, pulse_r);
}

/* ========== C 接口 ========== */

void shoot_init(void)
{
    shoot.init();
}

void shoot_control_loop(void)
{
    shoot.feedback_update();
    shoot.set_control();
    shoot.solve();
    shoot.output();
}
