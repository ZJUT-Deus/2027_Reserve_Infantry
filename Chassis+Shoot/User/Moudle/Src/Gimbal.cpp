/**
 * @file    Gimbal.cpp
 * @brief   云台控制模块实现 (DJI GM6020, yaw/pitch 基础控制)
 * @author  kk
 * @date    2026-06-03
 */

#include "Gimbal.h"
#include "spi.h"

#include <math.h>

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

static fp32 GIMBAL_YAW_GYRO_ANGLE_PID[6] = {
    GIMBAL_YAW_GYRO_ANGLE_PID_KP, GIMBAL_YAW_GYRO_ANGLE_PID_KI, GIMBAL_YAW_GYRO_ANGLE_PID_KD,
    GIMBAL_YAW_GYRO_ANGLE_PID_KF, GIMBAL_YAW_GYRO_ANGLE_PID_MAX_IOUT, GIMBAL_YAW_GYRO_ANGLE_PID_MAX_OUT};

/**
 * @brief  云台模块初始化
 */
void Gimbal::init()
{
    const static fp32 yaw_rate_filter_num[1] = {GIMBAL_IMU_YAW_RATE_FILTER_TAU};

    gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
    last_gimbal_behaviour_mode = gimbal_behaviour_mode;
    user_yaw_speed_set = 0.0f;
    user_pitch_speed_set = 0.0f;
    imu_yaw_angle = 0.0f;
    imu_yaw_angle_set = 0.0f;
    imu_yaw_rate = 0.0f;
    imu_yaw_rate_raw = 0.0f;
    imu_yaw_rate_filter.init(GIMBAL_CONTROL_TIME, yaw_rate_filter_num);
    imu_pitch_angle = 0.0f;
    imu_roll_angle = 0.0f;
    yaw_offset_ready = false;
    pitch_offset_ready = false;
    imu_ready = bmi088.init(&hspi2);

    yaw_motor = DJI_GM6020(GIMBAL_YAW_CAN_ID, can_receive.get_gimbal_motor_measure_point(0));
    pitch_motor = DJI_GM6020(GIMBAL_PITCH_CAN_ID, can_receive.get_gimbal_motor_measure_point(1));

    yaw_motor.speed_pid.init(PID_SPEED, GIMBAL_YAW_SPEED_PID,
                             &yaw_motor.speed, &yaw_motor.speed_set, 0.0f);
    yaw_motor.encode_angle_pid.init(PID_ANGLE, GIMBAL_YAW_ANGLE_PID,
                                    &yaw_motor.encode_angle, &yaw_motor.encode_angle_set, 0.0f);
    yaw_gyro_angle_pid.init(PID_ANGLE, GIMBAL_YAW_GYRO_ANGLE_PID,
                            &imu_yaw_angle, &imu_yaw_angle_set, 0.0f);
    pitch_motor.speed_pid.init(PID_SPEED, GIMBAL_PITCH_SPEED_PID,
                               &pitch_motor.speed, &pitch_motor.speed_set, 0.0f);
    pitch_motor.encode_angle_pid.init(PID_ANGLE, GIMBAL_PITCH_ANGLE_PID,
                                      &pitch_motor.encode_angle, &pitch_motor.encode_angle_set, 0.0f);
    yaw_motor.speed_pid.pid_clear();
    yaw_motor.encode_angle_pid.pid_clear();
    yaw_gyro_angle_pid.Clear();
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
 * @brief  判断云台是否处于有力闭环模式
 */
bool Gimbal::gimbal_active() const
{
    return gimbal_behaviour_mode == GIMBAL_FREE || gimbal_behaviour_mode == GIMBAL_TOP;
}

/**
 * @brief  更新 BMI088 姿态反馈
 */
void Gimbal::update_imu_feedback()
{
    const uint32_t now = HAL_GetTick();

    if (bmi088.update(now))
    {
        const bmi088_data_t *imu = bmi088.get_data_point();
        imu_yaw_angle = rad_format(imu->yaw * GIMBAL_IMU_YAW_DIRECTION);
        imu_yaw_rate_raw = imu->gyro[2] * GIMBAL_IMU_YAW_DIRECTION;
        imu_yaw_rate_filter.first_order_filter_cali(imu_yaw_rate_raw);
        imu_yaw_rate = imu_yaw_rate_filter.out;
        imu_yaw_rate = fp32_constrain(imu_yaw_rate,
                                      -GIMBAL_IMU_YAW_RATE_MAX,
                                      GIMBAL_IMU_YAW_RATE_MAX);
        if (fabsf(imu_yaw_rate) < GIMBAL_IMU_YAW_RATE_DEADBAND)
        {
            imu_yaw_rate = 0.0f;
        }
        imu_pitch_angle = rad_format(imu->roll * GIMBAL_IMU_PITCH_DIRECTION);
        imu_roll_angle = imu->pitch;
        imu_ready = true;
    }
    else
    {
        imu_ready = false;
    }
}

/**
 * @brief  模式切换时同步控制目标, 避免切换瞬间阶跃
 */
void Gimbal::handle_mode_change()
{
    if (last_gimbal_behaviour_mode == gimbal_behaviour_mode)
    {
        return;
    }

    if (gimbal_behaviour_mode == GIMBAL_TOP)
    {
        imu_yaw_angle_set = imu_yaw_angle;
        yaw_motor.encode_angle_set = yaw_motor.encode_angle;
        pitch_motor.encode_angle_set = constrain_pitch_angle(pitch_motor.encode_angle);
        yaw_motor.speed_set = 0.0f;
        pitch_motor.speed_set = 0.0f;
        yaw_motor.speed_pid.Clear();
        yaw_motor.encode_angle_pid.Clear();
        yaw_gyro_angle_pid.align_state_to_current();
        pitch_motor.speed_pid.Clear();
        pitch_motor.encode_angle_pid.Clear();
    }
    else if (gimbal_behaviour_mode == GIMBAL_FREE)
    {
        yaw_motor.encode_angle_set = yaw_motor.encode_angle;
        pitch_motor.encode_angle_set = constrain_pitch_angle(pitch_motor.encode_angle);
        yaw_motor.speed_set = 0.0f;
        pitch_motor.speed_set = 0.0f;
        yaw_motor.speed_pid.Clear();
        yaw_motor.encode_angle_pid.align_state_to_current();
        yaw_gyro_angle_pid.Clear();
        pitch_motor.speed_pid.Clear();
        pitch_motor.encode_angle_pid.align_state_to_current();
    }
    else
    {
        stop();
    }
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
    align_motor_offset(&yaw_motor, &yaw_offset_ready);
    align_motor_offset(&pitch_motor, &pitch_offset_ready);

    update_imu_feedback();

    yaw_motor.update_measure();
    yaw_motor.speed *= GIMBAL_YAW_DIRECTION;
    yaw_motor.encode_angle *= GIMBAL_YAW_DIRECTION;
    pitch_motor.update_measure();
    pitch_motor.speed *= GIMBAL_PITCH_DIRECTION;
    pitch_motor.encode_angle *= GIMBAL_PITCH_DIRECTION;
}

/**
 * @brief  设置 yaw 控制目标
 */
void Gimbal::set_control()
{
    handle_mode_change();

    if (gimbal_behaviour_mode == GIMBAL_ZERO_FORCE)
    {
        yaw_motor.speed_set = 0.0f;
        pitch_motor.speed_set = 0.0f;
        user_yaw_speed_set = 0.0f;
        user_pitch_speed_set = 0.0f;
        last_gimbal_behaviour_mode = gimbal_behaviour_mode;
        return;
    }

    if (gimbal_behaviour_mode == GIMBAL_TOP && imu_ready)
    {
        imu_yaw_angle_set = rad_format(imu_yaw_angle_set + user_yaw_speed_set * GIMBAL_CONTROL_TIME);
        pitch_motor.set(constrain_pitch_angle(pitch_motor.encode_angle_set +
                                              user_pitch_speed_set * GIMBAL_CONTROL_TIME),
                        ENCODE_ANGLE);
    }
    else
    {
        yaw_motor.set(rad_format(yaw_motor.encode_angle_set + user_yaw_speed_set * GIMBAL_CONTROL_TIME), ENCODE_ANGLE);
        pitch_motor.set(constrain_pitch_angle(pitch_motor.encode_angle_set +
                                              user_pitch_speed_set * GIMBAL_CONTROL_TIME),
                        ENCODE_ANGLE);
    }

    last_gimbal_behaviour_mode = gimbal_behaviour_mode;
}

/**
 * @brief  执行云台控制解算
 */
void Gimbal::solve()
{
    if (gimbal_behaviour_mode == GIMBAL_TOP && yaw_offset_ready && motor_online(&yaw_motor) && imu_ready)
    {
        yaw_motor.speed = imu_yaw_rate;
        yaw_motor.speed_set = fp32_constrain(yaw_gyro_angle_pid.pid_calc(),
                                             -GIMBAL_YAW_TOP_SPEED_MAX,
                                             GIMBAL_YAW_TOP_SPEED_MAX);
        yaw_motor.current_give = yaw_motor.speed_pid.pid_calc();
    }
    else if ((gimbal_behaviour_mode == GIMBAL_FREE ||
              (gimbal_behaviour_mode == GIMBAL_TOP && !imu_ready)) &&
             yaw_offset_ready && motor_online(&yaw_motor))
    {
        yaw_motor.solve(ENCODE_ANGLE);
    }
    else
    {
        yaw_motor.current_give = 0.0f;
        yaw_motor.speed_pid.Clear();
        yaw_motor.encode_angle_pid.Clear();
        yaw_gyro_angle_pid.Clear();
    }

    if (gimbal_active() && pitch_offset_ready && motor_online(&pitch_motor))
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

    if (gimbal_active() && yaw_offset_ready && motor_online(&yaw_motor))
    {
        fp32 yaw_current_fp = fp32_constrain(yaw_motor.current_give * GIMBAL_YAW_DIRECTION,
                                             -GIMBAL_GM6020_MAX_CURRENT,
                                             GIMBAL_GM6020_MAX_CURRENT);
        yaw_current = (int16_t)yaw_current_fp;
    }

    if (gimbal_active() && pitch_offset_ready && motor_online(&pitch_motor))
    {
        fp32 pitch_current_fp = fp32_constrain(pitch_motor.current_give * GIMBAL_PITCH_DIRECTION,
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
 * @brief  设置 top 模式角速度输入, yaw 由 BMI088 角度闭环自稳, pitch 保持编码器闭环
 */
void Gimbal::set_top_speed(fp32 yaw_speed, fp32 pitch_speed)
{
    gimbal_behaviour_mode = GIMBAL_TOP;
    user_yaw_speed_set = fp32_constrain(yaw_speed, -GIMBAL_YAW_RC_SPEED_MAX, GIMBAL_YAW_RC_SPEED_MAX);
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
    yaw_gyro_angle_pid.Clear();
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

void gimbal_set_top_speed(fp32 yaw_speed, fp32 pitch_speed)
{
    gimbal.set_top_speed(yaw_speed, pitch_speed);
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
