/**
 * @file    Gimbal.cpp
 * @brief   云台控制模块实现 (DJI GM6020, yaw/pitch 基础控制)
 * @author  kk
 * @date    2026-06-03
 */

#include "Gimbal.h"

Gimbal gimbal;

static fp32 GIMBAL_YAW_SPEED_PID[6] = {
    GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD,
    GIMBAL_YAW_SPEED_PID_KF, GIMBAL_YAW_SPEED_PID_MAX_IOUT, GIMBAL_YAW_SPEED_PID_MAX_OUT};

static fp32 GIMBAL_YAW_ANGLE_PID[6] = {
    GIMBAL_YAW_ANGLE_PID_KP, GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD,
    GIMBAL_YAW_ANGLE_PID_KF, GIMBAL_YAW_ANGLE_PID_MAX_IOUT, GIMBAL_YAW_ANGLE_PID_MAX_OUT};

static fp32 GIMBAL_PITCH_SPEED_PID[6] = {
    GIMBAL_PITCH_SPEED_PID_KP, GIMBAL_PITCH_SPEED_PID_KI, GIMBAL_PITCH_SPEED_PID_KD,
    GIMBAL_PITCH_SPEED_PID_KF, GIMBAL_PITCH_SPEED_PID_MAX_IOUT, GIMBAL_PITCH_SPEED_PID_MAX_OUT};

static fp32 GIMBAL_PITCH_ANGLE_PID[6] = {
    GIMBAL_PITCH_ANGLE_PID_KP, GIMBAL_PITCH_ANGLE_PID_KI, GIMBAL_PITCH_ANGLE_PID_KD,
    GIMBAL_PITCH_ANGLE_PID_KF, GIMBAL_PITCH_ANGLE_PID_MAX_IOUT, GIMBAL_PITCH_ANGLE_PID_MAX_OUT};

/**
 * @brief  云台模块初始化
 */
void Gimbal::init()
{
    gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
    last_gimbal_behaviour_mode = gimbal_behaviour_mode;
    user_yaw_speed_set = 0.0f;
    user_pitch_speed_set = 0.0f;
    yaw_offset_ready = false;
    pitch_offset_ready = false;

    yaw_motor = DJI_GM6020(GIMBAL_YAW_CAN_ID, can_receive.get_gimbal_motor_measure_point(0));
    pitch_motor = DJI_GM6020(GIMBAL_PITCH_CAN_ID, can_receive.get_gimbal_motor_measure_point(1));

    yaw_motor.speed_pid.init(PID_SPEED, GIMBAL_YAW_SPEED_PID,
                             &yaw_motor.speed, &yaw_motor.speed_set, 0.0f);
    yaw_motor.encode_angle_pid.init(PID_ANGLE, GIMBAL_YAW_ANGLE_PID,
                                    &yaw_motor.encode_angle, &yaw_motor.encode_angle_set, 0.0f);
    pitch_motor.speed_pid.init(PID_SPEED, GIMBAL_PITCH_SPEED_PID,
                               &pitch_motor.speed, &pitch_motor.speed_set, 0.0f);
    pitch_motor.encode_angle_pid.init(PID_ANGLE, GIMBAL_PITCH_ANGLE_PID,
                                      &pitch_motor.encode_angle, &pitch_motor.encode_angle_set, 0.0f);
    yaw_motor.speed_pid.pid_clear();
    yaw_motor.encode_angle_pid.pid_clear();
    pitch_motor.speed_pid.pid_clear();
    pitch_motor.encode_angle_pid.pid_clear();

    feedback_update();
}

/**
 * @brief  根据首次有效反馈对齐电机上电零点
 * @param  motor        GM6020 电机对象
 * @param  offset_ready 零点状态标志
 */
void Gimbal::align_motor_offset(DJI_GM6020 *motor, bool *offset_ready)
{
    if (motor == NULL || offset_ready == NULL || *offset_ready)
    {
        return;
    }

    if (motor->measure == NULL || motor->measure->online == 0U)
    {
        return;
    }

    motor->offset_ecd = motor->measure->ecd;
    motor->encode_angle = 0.0f;
    motor->encode_angle_set = 0.0f;
    *offset_ready = true;
}

/**
 * @brief  判断 GM6020 是否在线
 * @param  motor GM6020 电机对象
 * @return true: 在线, false: 离线
 */
bool Gimbal::motor_online(const DJI_GM6020 *motor) const
{
    if (motor == NULL || motor->measure == NULL || motor->measure->online == 0U)
    {
        return false;
    }

    return (HAL_GetTick() - motor->measure->last_update_ms) <= GIMBAL_MOTOR_OFFLINE_MS;
}

/**
 * @brief  pitch 机械限位
 * @param  pitch_angle pitch 目标角度 (rad)
 * @return 限幅后的 pitch 目标角度
 */
fp32 Gimbal::constrain_pitch_angle(fp32 pitch_angle) const
{
    return fp32_constrain(pitch_angle, GIMBAL_PITCH_MIN_ANGLE, GIMBAL_PITCH_MAX_ANGLE);
}

/**
 * @brief  更新云台反馈数据
 */
void Gimbal::feedback_update()
{
    last_gimbal_behaviour_mode = gimbal_behaviour_mode;

    align_motor_offset(&yaw_motor, &yaw_offset_ready);
    align_motor_offset(&pitch_motor, &pitch_offset_ready);

    yaw_motor.update_measure();
    pitch_motor.update_measure();
}

/**
 * @brief  设置 yaw 控制目标
 */
void Gimbal::set_control()
{
    if (gimbal_behaviour_mode == GIMBAL_ZERO_FORCE)
    {
        yaw_motor.speed_set = 0.0f;
        pitch_motor.speed_set = 0.0f;
        user_yaw_speed_set = 0.0f;
        user_pitch_speed_set = 0.0f;
        return;
    }

    yaw_motor.set(rad_format(yaw_motor.encode_angle_set + user_yaw_speed_set * GIMBAL_CONTROL_TIME), ENCODE_ANGLE);
    pitch_motor.set(constrain_pitch_angle(pitch_motor.encode_angle_set +
                                          user_pitch_speed_set * GIMBAL_CONTROL_TIME),
                    ENCODE_ANGLE);
}

/**
 * @brief  执行云台控制解算
 */
void Gimbal::solve()
{
    if (gimbal_behaviour_mode == GIMBAL_FREE && yaw_offset_ready && motor_online(&yaw_motor))
    {
        yaw_motor.solve(ENCODE_ANGLE);
    }
    else
    {
        yaw_motor.current_give = 0.0f;
        yaw_motor.speed_pid.Clear();
        yaw_motor.encode_angle_pid.Clear();
    }

    if (gimbal_behaviour_mode == GIMBAL_FREE && pitch_offset_ready && motor_online(&pitch_motor))
    {
        pitch_motor.solve(ENCODE_ANGLE);
    }
    else
    {
        pitch_motor.current_give = 0.0f;
        pitch_motor.speed_pid.Clear();
        pitch_motor.encode_angle_pid.Clear();
    }
}

/**
 * @brief  输出云台电机电流
 */
void Gimbal::output()
{
    int16_t yaw_current = 0;
    int16_t pitch_current = 0;

    if (gimbal_behaviour_mode == GIMBAL_FREE && yaw_offset_ready && motor_online(&yaw_motor))
    {
        fp32 yaw_current_fp = fp32_constrain(yaw_motor.current_give,
                                             -GIMBAL_GM6020_MAX_CURRENT,
                                             GIMBAL_GM6020_MAX_CURRENT);
        yaw_current = (int16_t)yaw_current_fp;
    }

    if (gimbal_behaviour_mode == GIMBAL_FREE && pitch_offset_ready && motor_online(&pitch_motor))
    {
        fp32 pitch_current_fp = fp32_constrain(pitch_motor.current_give,
                                               -GIMBAL_GM6020_MAX_CURRENT,
                                               GIMBAL_GM6020_MAX_CURRENT);
        pitch_current = (int16_t)pitch_current_fp;
    }

    can_receive.can_cmd_gimbal_motor(yaw_current, pitch_current);
}

/**
 * @brief  直接设置 yaw 编码器目标角度
 * @param  yaw_angle 目标角度 (rad)
 */
void Gimbal::set_yaw_angle(fp32 yaw_angle)
{
    gimbal_behaviour_mode = GIMBAL_FREE;
    user_yaw_speed_set = 0.0f;
    yaw_motor.set(rad_format(yaw_angle), ENCODE_ANGLE);
    yaw_motor.encode_angle_pid.align_state_to_current();
}

/**
 * @brief  设置 yaw 角速度输入, 内部转为角度目标增量
 * @param  yaw_speed yaw 角速度 (rad/s)
 */
void Gimbal::set_yaw_speed(fp32 yaw_speed)
{
    gimbal_behaviour_mode = GIMBAL_FREE;
    user_yaw_speed_set = fp32_constrain(yaw_speed, -GIMBAL_YAW_RC_SPEED_MAX, GIMBAL_YAW_RC_SPEED_MAX);
}

/**
 * @brief  直接设置 pitch 编码器目标角度
 * @param  pitch_angle 目标角度 (rad)
 */
void Gimbal::set_pitch_angle(fp32 pitch_angle)
{
    gimbal_behaviour_mode = GIMBAL_FREE;
    user_pitch_speed_set = 0.0f;
    pitch_motor.set(constrain_pitch_angle(pitch_angle), ENCODE_ANGLE);
    pitch_motor.encode_angle_pid.align_state_to_current();
}

/**
 * @brief  设置 pitch 角速度输入, 内部转为角度目标增量
 * @param  pitch_speed pitch 角速度 (rad/s)
 */
void Gimbal::set_pitch_speed(fp32 pitch_speed)
{
    gimbal_behaviour_mode = GIMBAL_FREE;
    user_pitch_speed_set = fp32_constrain(pitch_speed, -GIMBAL_PITCH_RC_SPEED_MAX, GIMBAL_PITCH_RC_SPEED_MAX);
}

/**
 * @brief  云台进入无力模式
 */
void Gimbal::stop()
{
    gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
    user_yaw_speed_set = 0.0f;
    user_pitch_speed_set = 0.0f;
    yaw_motor.speed_set = 0.0f;
    pitch_motor.speed_set = 0.0f;
    yaw_motor.current_give = 0.0f;
    pitch_motor.current_give = 0.0f;
    yaw_motor.speed_pid.Clear();
    yaw_motor.encode_angle_pid.Clear();
    pitch_motor.speed_pid.Clear();
    pitch_motor.encode_angle_pid.Clear();
}

void gimbal_init(void)
{
    gimbal.init();
}

void gimbal_set_yaw_angle(fp32 yaw_angle)
{
    gimbal.set_yaw_angle(yaw_angle);
}

void gimbal_set_yaw_speed(fp32 yaw_speed)
{
    gimbal.set_yaw_speed(yaw_speed);
}

void gimbal_set_pitch_angle(fp32 pitch_angle)
{
    gimbal.set_pitch_angle(pitch_angle);
}

void gimbal_set_pitch_speed(fp32 pitch_speed)
{
    gimbal.set_pitch_speed(pitch_speed);
}

void gimbal_stop(void)
{
    gimbal.stop();
}

void gimbal_control_loop(void)
{
    gimbal.feedback_update();
    gimbal.set_control();
    gimbal.solve();
    gimbal.output();
}
