/**
 * @file    Chassis.cpp
 * @brief   底盘运动控制模块实现 (DM3519 电机 + 麦轮运动学)
 * @author  kk
 * @date    2026-05-25
 */

#include "Chassis.h"
#include "bsp_fdcan.h"

Chassis chassis;

static float Chassis_SPEED_PID[6] = {
    CHASSIS_SPEED_PID_KP, CHASSIS_SPEED_PID_KI, CHASSIS_SPEED_PID_KD,
    CHASSIS_SPEED_PID_KF, CHASSIS_SPEED_PID_MAX_IOUT, CHASSIS_SPEED_PID_MAX_OUT
};

static const uint16_t CHASSIS_CAN_ID[4] = {
    CHASSIS_FR_CAN_ID, CHASSIS_FL_CAN_ID, CHASSIS_BR_CAN_ID, CHASSIS_BL_CAN_ID
};

static const uint16_t CHASSIS_CAN_MST_ID[4] = {
    CHASSIS_FR_MST_ID, CHASSIS_FL_MST_ID, CHASSIS_BR_MST_ID, CHASSIS_BL_MST_ID
};

/**
 * @brief  底盘初始化, 配置 4 个 DM3519 电机并初始化速度环 PID
 */
void Chassis::init()
{
    chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    user_vx_set = 0.0f;
    user_vy_set = 0.0f;
    user_wz_set = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        can_receive.CTRL_DM3519(CHASSIS_CAN_ID[i], ENABLE);
        chassis_motive_motor[i] = DM3519(CHASSIS_CAN_ID[i],
                                 CHASSIS_CAN_MST_ID[i],
                                 can_receive.get_chassis_motor_measure_point(i));
        chassis_motive_motor[i].DM3519_Init();
        chassis_motive_motor[i].speed_pid.init(PID_SPEED, Chassis_SPEED_PID,
                                               &chassis_motive_motor[i].speed,
                                               &chassis_motive_motor[i].speed_set, 0.0f);
        chassis_motive_motor[i].speed_pid.pid_clear();
        HAL_Delay(10);
    }

    const static fp32 filter_x_num[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 filter_y_num[1] = {CHASSIS_ACCEL_Y_NUM};

    chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, filter_x_num);
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, filter_y_num);

    x.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    x.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;

    y.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    y.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;

    z.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
    z.max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;

    feedback_update();
}

/**
 * @brief  底盘测量数据更新, 包括电机速度和底盘三轴速度
 */
void Chassis::feedback_update()
{
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].update_measure();
    }

    fp32 w0 = chassis_motive_motor[0].speed;
    fp32 w1 = chassis_motive_motor[1].speed;
    fp32 w2 = chassis_motive_motor[2].speed;
    fp32 w3 = chassis_motive_motor[3].speed;

    x.speed = ( w0 + w1 + w2 + w3) * MOTOR_WHEEL_RADIUS / 4.0f;
    y.speed = (-w0 + w1 + w2 - w3) * MOTOR_WHEEL_RADIUS / 4.0f;
    z.speed = (-w0 + w1 - w2 + w3) * MOTOR_WHEEL_RADIUS / (4.0f * MOTOR_DISTANCE_TO_CENTER);
}

/**
 * @brief  设置底盘控制值, 根据行为模式计算三轴速度并下发到电机
 */
void Chassis::set_control()
{
    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

    chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set);

    x.speed_set = fp32_constrain(vx_set, x.min_speed, x.max_speed);
    y.speed_set = fp32_constrain(vy_set, y.min_speed, y.max_speed);
    z.speed_set = fp32_constrain(wz_set, z.min_speed, z.max_speed);

    fp32 wheel_speed[4] = {0.0f};

    chassis_vector_to_mecanum_wheel_speed(wheel_speed);

    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].set(wheel_speed[i], SPEED);
    }
}

/**
 * @brief  执行电机速度环 PID 计算
 */
void Chassis::solve()
{
    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].solve(SPEED);
    }
}

/**
 * @brief  输出 DM3519 MIT 控制指令到 CAN 总线
 */
void Chassis::output()
{
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].current_give = 0.0f;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        esc_inf_t tmp = chassis_motive_motor[i].tmp;
        can_receive.can_cmd_mit_dm_motor(
            0.0f,
            chassis_motive_motor[i].speed_set,
            0.0f,
            0.0f,
            chassis_motive_motor[i].current_give,
            chassis_motive_motor[i].can_id,
            tmp);
    }
}

/**
 * @brief  根据不同底盘控制模式设置三轴控制量
 * @param  vx_set X 轴速度设定
 * @param  vy_set Y 轴速度设定
 * @param  wz_set Z 轴角速度设定
 */
void Chassis::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, wz_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_FREE)
    {
        chassis_free_control(vx_set, vy_set, wz_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_SPIN)
    {
        chassis_spin_control(vx_set, vy_set, wz_set);
    }
}

/**
 * @brief  无力模式: 所有速度指令为零
 * @param  vx_set X 轴速度指令
 * @param  vy_set Y 轴速度指令
 * @param  wz_set Z 轴角速度指令
 */
void Chassis::chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
 * @brief  自由速度模式: 用户指令经加速度滤波后输出, wz 直接响应
 * @param  vx_set X 轴速度指令
 * @param  vy_set Y 轴速度指令
 * @param  wz_set Z 轴角速度指令
 */
void Chassis::chassis_free_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    chassis_cmd_slow_set_vx.first_order_filter_cali(user_vx_set);
    chassis_cmd_slow_set_vy.first_order_filter_cali(user_vy_set);

    if (user_vx_set < CHASSIS_ACCEL_X_NUM && user_vx_set > -CHASSIS_ACCEL_X_NUM)
    {
        chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (user_vy_set < CHASSIS_ACCEL_Y_NUM && user_vy_set > -CHASSIS_ACCEL_Y_NUM)
    {
        chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
    *wz_set = user_wz_set;
}

/**
 * @brief  小陀螺模式: vx/vy 经滤波输出, wz 直接响应, 无死区抑制
 * @param  vx_set X 轴速度指令
 * @param  vy_set Y 轴速度指令
 * @param  wz_set Z 轴角速度指令
 */
void Chassis::chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    chassis_cmd_slow_set_vx.first_order_filter_cali(user_vx_set);
    chassis_cmd_slow_set_vy.first_order_filter_cali(user_vy_set);

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
    *wz_set = user_wz_set;
}

/**
 * @brief  麦轮逆运动学: 三轴底盘速度 -> 四轮目标转速
 * @param  wheel_speed 四轮目标转速 (rad/s)
 */
void Chassis::chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4])
{
    fp32 vx = x.speed_set;
    fp32 vy = y.speed_set;
    fp32 wz = z.speed_set;

    wheel_speed[0] = (vx - vy + MOTOR_DISTANCE_TO_CENTER * wz) / MOTOR_WHEEL_RADIUS;
    wheel_speed[1] = (vx + vy + MOTOR_DISTANCE_TO_CENTER * wz) / MOTOR_WHEEL_RADIUS;
    wheel_speed[2] = (-vx - vy + MOTOR_DISTANCE_TO_CENTER * wz) / MOTOR_WHEEL_RADIUS;
    wheel_speed[3] = (-vx + vy + MOTOR_DISTANCE_TO_CENTER * wz) / MOTOR_WHEEL_RADIUS;

    for (int i = 0; i < 4; i++)
    {
        wheel_speed[i] = fp32_constrain(wheel_speed[i], -20.0f, 20.0f);
    }
}

/**
 * @brief  底盘初始化 (C 接口)
 */
void chassis_init(void)
{
    chassis.init();
}

/**
 * @brief  设置底盘自由运动速度 (C 接口)
 * @param  vx 前进速度 (m/s), 正值前进
 * @param  vy 横移速度 (m/s), 正值左移
 * @param  wz 旋转角速度 (rad/s), 正值逆时针
 */
void chassis_set_velocity(fp32 vx, fp32 vy, fp32 wz)
{
    chassis.user_vx_set = vx;
    chassis.user_vy_set = vy;
    chassis.user_wz_set = wz;
    chassis.chassis_behaviour_mode = CHASSIS_FREE;
}

/**
 * @brief  设置底盘小陀螺运动 (C 接口)
 * @param  vx 前进速度 (m/s)
 * @param  vy 横移速度 (m/s)
 * @param  wz 旋转角速度 (rad/s)
 */
void chassis_set_spin(fp32 vx, fp32 vy, fp32 wz)
{
    chassis.user_vx_set = vx;
    chassis.user_vy_set = vy;
    chassis.user_wz_set = wz;
    chassis.chassis_behaviour_mode = CHASSIS_SPIN;
}

/**
 * @brief  底盘急停, 速度归零并切到无力模式 (C 接口)
 */
void chassis_stop(void)
{
    chassis.user_vx_set = 0.0f;
    chassis.user_vy_set = 0.0f;
    chassis.user_wz_set = 0.0f;
    chassis.chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
}

/**
 * @brief  底盘控制主循环, 每个控制周期调用一次 (C 接口)
 */
void chassis_control_loop(void)
{
    chassis.feedback_update();
    chassis.set_control();
    chassis.solve();
    chassis.output();
}
