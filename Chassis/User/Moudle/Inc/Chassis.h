/**
 * @file    Chassis.h
 * @brief   底盘运动控制模块 (DM3519 电机 + 麦轮运动学)
 * @author  kk
 * @date    2026-05-25
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "struct_typedef.h"
#include "pid.h"
#include "Motor.h"
#include "Can_receive.h"
#include "user_lib.h"
extern Can_receive can_receive;

#ifdef __cplusplus
extern "C" {
#endif // CHASSIS_H

/** @brief 麦轮半径 (m) */
#define MOTOR_WHEEL_RADIUS         0.076f
/** @brief 轮距中心距离 (m) */
#define MOTOR_DISTANCE_TO_CENTER   0.185f
/** @brief 底盘 Vx 速度系数 */
#define MOTOR_SPEED_TO_CHASSIS_VX  0.25f
/** @brief 底盘 Vy 速度系数 */
#define MOTOR_SPEED_TO_CHASSIS_VY  0.25f
/** @brief 底盘 Wz 角速度系数 */
#define MOTOR_SPEED_TO_CHASSIS_WZ  0.25f

/** @brief X 轴最大速度 (m/s) */
#define NORMAL_MAX_CHASSIS_SPEED_X  2.0f
/** @brief Y 轴最大速度 (m/s) */
#define NORMAL_MAX_CHASSIS_SPEED_Y  1.5f
/** @brief Z 轴最大角速度 (rad/s) */
#define NORMAL_MAX_CHASSIS_SPEED_Z  14.0f
/** @brief 小陀螺固定旋转速度 (rad/s) */
#define SPIN_WZ_SPEED              5.0f

/** @brief X 轴加速度滤波系数 */
#define CHASSIS_ACCEL_X_NUM  0.1666666667f
/** @brief Y 轴加速度滤波系数 */
#define CHASSIS_ACCEL_Y_NUM  0.3333333333f
/** @brief 控制周期 (s) */
#define CHASSIS_CONTROL_TIME 0.002f

/** @brief 速度环 PID Kp */
#define CHASSIS_SPEED_PID_KP       2.0f
/** @brief 速度环 PID Ki */
#define CHASSIS_SPEED_PID_KI       0.0f
/** @brief 速度环 PID Kd */
#define CHASSIS_SPEED_PID_KD       0.3f
/** @brief 速度环 PID Kf */
#define CHASSIS_SPEED_PID_KF       0.0f
/** @brief 速度环 PID 积分限幅 */
#define CHASSIS_SPEED_PID_MAX_IOUT 10.0f
/** @brief 速度环 PID 输出限幅 */
#define CHASSIS_SPEED_PID_MAX_OUT  10.0f

/** @brief 底盘行为状态机 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, /**< 无力模式 */
    CHASSIS_FREE,           /**< 自由速度模式 */
    CHASSIS_SPIN,           /**< 小陀螺模式 */
} chassis_behaviour_e;

/** @brief 单轴速度数据结构 */
typedef struct
{
    fp32 speed;     /**< 当前速度 */
    fp32 speed_set; /**< 目标速度 */
    fp32 max_speed; /**< 最大速度限幅 */
    fp32 min_speed; /**< 最小速度限幅 */
} speed_t;

#ifdef __cplusplus
}
#endif

/** @brief 底盘运动控制主类 */
class Chassis
{
public:
    DM3519 chassis_motive_motor[4];         /**< 四轮 DM3519 电机对象 */

    chassis_behaviour_e chassis_behaviour_mode;     /**< 当前行为模式 */
    chassis_behaviour_e last_chassis_behaviour_mode; /**< 上一次行为模式 */

    speed_t x;  /**< X 轴 (前进) 速度 */
    speed_t y;  /**< Y 轴 (横移) 速度 */
    speed_t z;  /**< Z 轴 (旋转) 角速度 */

    First_order_filter chassis_cmd_slow_set_vx; /**< X 轴一阶滤波 */
    First_order_filter chassis_cmd_slow_set_vy; /**< Y 轴一阶滤波 */

    fp32 user_vx_set;   /**< 用户指令 Vx */
    fp32 user_vy_set;   /**< 用户指令 Vy */
    fp32 user_wz_set;   /**< 用户指令 Wz */

    bool motor_enabled;  /**< 电机使能状态标志 */

    void init();
    void startup_check();
    void feedback_update();
    void set_control();
    void solve();
    void output();

    void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
    void chassis_free_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
    void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
    void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
    void chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4]);
};

extern Chassis chassis;

#ifdef __cplusplus
extern "C" {
#endif

void chassis_init(void);
void chassis_set_velocity(fp32 vx, fp32 vy, fp32 wz);
void chassis_set_spin(fp32 vx, fp32 vy, fp32 wz);
void chassis_stop(void);
void chassis_control_loop(void);

#ifdef __cplusplus
}
#endif

#endif
