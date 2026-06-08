/**
 * @file    Gimbal.h
 * @brief   云台控制模块 (DJI GM6020, yaw/pitch 基础控制)
 * @author  kk
 * @date    2026-06-03
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "struct_typedef.h"
#include "pid.h"
#include "Motor.h"
#include "Can_receive.h"
#include "user_lib.h"
#include "BMI088.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 云台控制周期 (s) */
#define GIMBAL_CONTROL_TIME 0.002f
/** @brief 云台 yaw 遥控最大角速度 (rad/s) */
#define GIMBAL_YAW_RC_SPEED_MAX 3.5f
/** @brief top 模式 yaw 陀螺仪角度环输出限幅 (rad/s) */
#define GIMBAL_YAW_TOP_SPEED_MAX 8.0f
/** @brief 云台 pitch 遥控最大角速度 (rad/s) */
#define GIMBAL_PITCH_RC_SPEED_MAX 1.5f
/** @brief 云台电机离线超时 (ms) */
#define GIMBAL_MOTOR_OFFLINE_MS 100U
/** @brief GM6020 电流指令保守限幅 */
#define GIMBAL_GM6020_MAX_CURRENT 8000.0f
/** @brief yaw 轴安装方向: -1 表示编码器/电流正方向与期望 yaw 正方向相反 */
#define GIMBAL_YAW_DIRECTION 1.0f
/** @brief pitch 轴安装方向: -1 表示编码器/电流正方向与物理上仰方向相反 */
#define GIMBAL_PITCH_DIRECTION -1.0f
/** @brief pitch 机械上限 (rad), 暂按 2026 云台工程限位 */
#define GIMBAL_PITCH_MAX_ANGLE 0.7853981633974483f
/** @brief pitch 机械下限 (rad), 暂按 2026 云台工程限位 */
#define GIMBAL_PITCH_MIN_ANGLE -0.2f
/** @brief yaw 速度环 PID Kp */
#define GIMBAL_YAW_SPEED_PID_KP       573.0f
/** @brief yaw 速度环 PID Ki */
#define GIMBAL_YAW_SPEED_PID_KI       8.6f
/** @brief yaw 速度环 PID Kd */
#define GIMBAL_YAW_SPEED_PID_KD       0.0f
/** @brief yaw 速度环 PID Kf */
#define GIMBAL_YAW_SPEED_PID_KF       0.0f
/** @brief yaw 速度环积分限幅 */
#define GIMBAL_YAW_SPEED_PID_MAX_IOUT 4880.0f
/** @brief yaw 速度环输出限幅 */
#define GIMBAL_YAW_SPEED_PID_MAX_OUT  GIMBAL_GM6020_MAX_CURRENT

/** @brief yaw 编码器角度环 PID Kp */
#define GIMBAL_YAW_ANGLE_PID_KP       90.0f
/** @brief yaw 编码器角度环 PID Ki */
#define GIMBAL_YAW_ANGLE_PID_KI       0.0f
/** @brief yaw 编码器角度环 PID Kd */
#define GIMBAL_YAW_ANGLE_PID_KD       0.0f
/** @brief yaw 编码器角度环 PID Kf */
#define GIMBAL_YAW_ANGLE_PID_KF       0.0f
/** @brief yaw 编码器角度环积分限幅 */
#define GIMBAL_YAW_ANGLE_PID_MAX_IOUT 0.0f
/** @brief yaw 编码器角度环输出限幅 */
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT  75.4f

/** @brief pitch 速度环 PID Kp */
#define GIMBAL_PITCH_SPEED_PID_KP       573.0f
/** @brief pitch 速度环 PID Ki */
#define GIMBAL_PITCH_SPEED_PID_KI       8.6f
/** @brief pitch 速度环 PID Kd */
#define GIMBAL_PITCH_SPEED_PID_KD       0.0f
/** @brief pitch 速度环 PID Kf */
#define GIMBAL_PITCH_SPEED_PID_KF       0.0f
/** @brief pitch 速度环积分限幅 */
#define GIMBAL_PITCH_SPEED_PID_MAX_IOUT 4880.0f
/** @brief pitch 速度环输出限幅 */
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT  GIMBAL_GM6020_MAX_CURRENT

/** @brief pitch 编码器角度环 PID Kp */
#define GIMBAL_PITCH_ANGLE_PID_KP       90.0f
/** @brief pitch 编码器角度环 PID Ki */
#define GIMBAL_PITCH_ANGLE_PID_KI       0.0f
/** @brief pitch 编码器角度环 PID Kd */
#define GIMBAL_PITCH_ANGLE_PID_KD       0.0f
/** @brief pitch 编码器角度环 PID Kf */
#define GIMBAL_PITCH_ANGLE_PID_KF       0.0f
/** @brief pitch 编码器角度环积分限幅 */
#define GIMBAL_PITCH_ANGLE_PID_MAX_IOUT 0.0f
/** @brief pitch 编码器角度环输出限幅 */
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT  75.4f

/** @brief top 模式 yaw IMU 角度环 PID Kp, 参考 2024 sentry yaw gyro PID */
#define GIMBAL_YAW_GYRO_ANGLE_PID_KP       35.0f
/** @brief top 模式 yaw IMU 角度环 PID Ki, 参考 2024 sentry yaw gyro PID */
#define GIMBAL_YAW_GYRO_ANGLE_PID_KI       0.1f
/** @brief top 模式 yaw IMU 角度环 PID Kd, 参考 2024 sentry yaw gyro PID */
#define GIMBAL_YAW_GYRO_ANGLE_PID_KD       800.0f
/** @brief top 模式 yaw IMU 角度环 PID Kf */
#define GIMBAL_YAW_GYRO_ANGLE_PID_KF       0.0f
/** @brief top 模式 yaw IMU 角度环积分限幅 */
#define GIMBAL_YAW_GYRO_ANGLE_PID_MAX_IOUT 5.0f
/** @brief top 模式 yaw IMU 角度环输出限幅 */
#define GIMBAL_YAW_GYRO_ANGLE_PID_MAX_OUT  120.0f
/** @brief BMI088 yaw 符号方向, 若 top 模式反向可改为 -1 */
#define GIMBAL_IMU_YAW_DIRECTION -1.0f
/** @brief BMI088 yaw 角速度一阶低通时间常数 (s), 越大越稳但响应越慢 */
#define GIMBAL_IMU_YAW_RATE_FILTER_TAU 0.023f
/** @brief BMI088 yaw 角速度低通后死区 (rad/s) */
#define GIMBAL_IMU_YAW_RATE_DEADBAND 0.01f
/** @brief BMI088 yaw 角速度反馈限幅 (rad/s) */
#define GIMBAL_IMU_YAW_RATE_MAX 8.0f
/** @brief 云台 pitch 对应 BMI088 roll, 若点头方向反向可改为 -1 */
#define GIMBAL_IMU_PITCH_DIRECTION 1.0f

/** @brief 云台行为状态机 */
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, /**< 无力模式 */
    GIMBAL_TOP,            /**< BMI088 yaw 自稳, pitch 编码器闭环模式 */
    GIMBAL_FREE,           /**< 编码器角度闭环模式 */
} gimbal_behaviour_e;

#ifdef __cplusplus
}
#endif

/** @brief 云台控制主类 */
class Gimbal
{
public:
    DJI_GM6020 yaw_motor;   /**< yaw 轴 GM6020 */
    DJI_GM6020 pitch_motor; /**< pitch 轴 GM6020 */
    Pid yaw_gyro_angle_pid;   /**< top 模式 yaw IMU 角度环 */

    gimbal_behaviour_e gimbal_behaviour_mode;      /**< 当前行为模式 */
    gimbal_behaviour_e last_gimbal_behaviour_mode; /**< 上一次行为模式 */

    fp32 user_yaw_speed_set;   /**< 外部输入 yaw 角速度指令 */
    fp32 user_pitch_speed_set; /**< 外部输入 pitch 角速度指令 */
    fp32 imu_yaw_angle;        /**< BMI088 yaw 角度反馈 */
    fp32 imu_yaw_angle_set;    /**< BMI088 yaw 目标角度 */
    fp32 imu_yaw_rate;         /**< BMI088 yaw 角速度反馈 */
    fp32 imu_yaw_rate_raw;     /**< BMI088 yaw 角速度原始映射值 */
    First_order_filter imu_yaw_rate_filter; /**< BMI088 yaw 角速度一阶低通 */
    fp32 imu_pitch_angle;      /**< 云台 pitch 角度反馈, 由 BMI088 roll 映射 */
    fp32 imu_roll_angle;       /**< 云台 roll 方向反馈, 由 BMI088 pitch 映射 */
    bool yaw_offset_ready;     /**< yaw 上电零点是否已对齐 */
    bool pitch_offset_ready;   /**< pitch 上电零点是否已对齐 */
    bool imu_ready;            /**< BMI088 是否可用 */

    void init();
    void feedback_update();
    void set_control();
    void solve();
    void output();

    void set_yaw_angle(fp32 yaw_angle);
    void set_yaw_speed(fp32 yaw_speed);
    void set_pitch_angle(fp32 pitch_angle);
    void set_pitch_speed(fp32 pitch_speed);
    void set_top_speed(fp32 yaw_speed, fp32 pitch_speed);
    void stop();

private:
    void align_motor_offset(DJI_GM6020 *motor, bool *offset_ready);
    bool motor_online(const DJI_GM6020 *motor) const;
    bool gimbal_active() const;
    void handle_mode_change();
    void update_imu_feedback();
    fp32 constrain_pitch_angle(fp32 pitch_angle) const;
};

extern Gimbal gimbal;

#ifdef __cplusplus
extern "C" {
#endif

void gimbal_init(void);
void gimbal_set_yaw_angle(fp32 yaw_angle);
void gimbal_set_yaw_speed(fp32 yaw_speed);
void gimbal_set_pitch_angle(fp32 pitch_angle);
void gimbal_set_pitch_speed(fp32 pitch_speed);
void gimbal_set_top_speed(fp32 yaw_speed, fp32 pitch_speed);
void gimbal_stop(void);
void gimbal_control_loop(void);

#ifdef __cplusplus
}
#endif

#endif
