/**
 * @file    Shoot.h
 * @brief   射击控制模块 (C615摩擦轮PWM + C610拨弹CAN + 防卡弹)
 * @author  kk
 * @date    2026-06-05
 */

#ifndef SHOOT_H
#define SHOOT_H

#include "struct_typedef.h"
#include "Motor.h"
#include "user_lib.h"

/* ========== 射击状态枚举 ========== */
/** @brief 射击模式状态机 */
typedef enum
{
    SHOOT_STOP = 0,           /**< 停止: 摩擦轮停转, 拨弹停转 */
    SHOOT_READY,              /**< 准备: 摩擦轮旋转中, 拨弹停转 */
    SHOOT_CONTINUE_BULLET,    /**< 连发: 摩擦轮+拨弹同时旋转 */
} shoot_mode_e;

/* ========== 防卡弹状态枚举 ========== */
/** @brief 拨弹轮防卡弹状态机 */
typedef enum
{
    TRIGGER_ANTI_JAM_IDLE = 0,  /**< 正常连射 */
    TRIGGER_ANTI_JAM_REVERSE,   /**< 反转退弹 */
    TRIGGER_ANTI_JAM_RECOVERY,  /**< 反转后恢复等待 */
    TRIGGER_ANTI_JAM_LOCKED,    /**< 锁死, 需复位 */
} trigger_anti_jam_state_e;

/* ========== 射击控制参数 ========== */
/** @brief 摩擦轮目标转速 (rad/s) */
#define SHOOT_FRIC_SPEED       350.0f
/** @brief 拨弹轮连发目标转速 (rad/s) */
#define SHOOT_TRIGGER_SPEED    80.0f
/** @brief 摩擦轮一阶滤波时间常数 (s) */
#define SHOOT_FRIC_RAMP_TAU    0.3f
/** @brief 控制循环周期 (s), 对应 500Hz */
#define SHOOT_CONTROL_DT_S     0.002f

/* ========== 拨弹轮速度环 PID ========== */
/** @brief 速度环比例系数 */
#define TRIGGER_SPEED_PID_KP   2000.0f
/** @brief 速度环积分系数 */
#define TRIGGER_SPEED_PID_KI   0.5f
/** @brief 速度环微分系数 */
#define TRIGGER_SPEED_PID_KD   0.0f
/** @brief 速度环前馈系数 */
#define TRIGGER_SPEED_PID_KF   0.0f
/** @brief 积分输出限幅 (mA) */
#define TRIGGER_PID_MAX_IOUT   2000.0f
/** @brief PID 总输出限幅 (mA), 对应 8A */
#define TRIGGER_PID_MAX_OUT    8000.0f

/* ========== 防卡弹参数 ========== */
/** @brief 堵转判定转速阈值 (rad/s), 低于此值判为堵转 */
#define TRIGGER_JAM_SPEED_THRESHOLD   1.0f
/** @brief 堵转判定电流阈值 (mA), 高于此值判为堵转 */
#define TRIGGER_JAM_CURRENT_THRESHOLD 3000
/** @brief 堵转确认时间 (控制周期数, 每周期 2ms), 连续满足条件才触发 */
#define TRIGGER_JAM_DETECT_TIME       80
/** @brief 反转退弹持续时间 (控制周期数) */
#define TRIGGER_JAM_REVERSE_TIME      100
/** @brief 反转后恢复等待时间 (控制周期数) */
#define TRIGGER_JAM_RECOVERY_TIME     40
/** @brief 启动忽略时间 (控制周期数), 避开启动瞬间大电流 */
#define TRIGGER_JAM_STARTUP_IGNORE    100
/** @brief 正常运转后重置重试计数的时间 (控制周期数) */
#define TRIGGER_JAM_RETRY_RESET_TIME  150
/** @brief 最大重试次数, 超过后锁死 */
#define TRIGGER_JAM_RETRY_MAX         3
/** @brief 反转退弹目标转速 (rad/s) */
#define TRIGGER_JAM_REVERSE_SPEED     30.0f
/** @brief 最小有效指令转速 (rad/s), 低于此值不触发防卡弹 */
#define TRIGGER_JAM_CMD_MIN_SPEED     0.1f

class Shoot
{
public:
    C615 friction_left;
    C615 friction_right;
    C610 trigger_motor;

    shoot_mode_e shoot_mode;

    First_order_filter fric_ramp_left;
    First_order_filter fric_ramp_right;

    /* ---- 防卡弹状态 ---- */
    trigger_anti_jam_state_e trigger_anti_jam_state;
    fp32 trigger_forward_speed_set;
    uint16_t block_time;
    uint16_t reverse_time;
    uint16_t recovery_time;
    uint16_t startup_ignore_time;
    uint16_t forward_time;
    uint8_t  jam_retry_count;

    void init();
    void feedback_update();
    void set_control();
    void solve();
    void output();

    void trigger_motor_turn_back();
    void reset_trigger_anti_jam();
    bool_t trigger_motor_blocked();
};

extern Shoot shoot;

#ifdef __cplusplus
extern "C" {
#endif

void shoot_init(void);
void shoot_control_loop(void);
void shoot_ready(void);
void shoot_continue_bullet(void);
void shoot_stop(void);

#ifdef __cplusplus
}
#endif

#endif
