/**
 * @file    Motor.h
 * @brief   电机设备驱动模块 (DM3519)
 * @author  kk
 * @date    2026-05-23
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "struct_typedef.h"
#include "main.h"
#include "pid.h"
#include "user_lib.h"
#include "bsp_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 速度控制模式 */
#define SPEED 0
/** @brief MIT 模式控制字 */
#define MODE  0X00

/** @brief DM3519 电机反馈测量数据 */
typedef struct
{
    int id;         /**< 电机 CAN ID */
    int err;        /**< 错误码 */
    int p_int;      /**< 位置 (整数编码) */
    int v_int;      /**< 速度 (整数编码) */
    int t_int;      /**< 扭矩 (整数编码) */
    fp32 Tmos;      /**< MOS 管温度 */
    fp32 Tcoil;     /**< 线圈温度 */
} dm_motor_measure_t;

/** @brief C610 电机 CAN 反馈数据 (8字节) */
typedef struct
{
    uint16_t angle;         /**< 机械角度 (0-8191) */
    int16_t  speed_rpm;     /**< 转速 (rpm) */
    int16_t  torque_current; /**< 转矩电流 */
    uint8_t  temp;          /**< 温度 */
} c610_motor_measure_t;

/** @brief 电机物理参数限制 */
typedef struct
{
    fp32 pmax;      /**< 最大位置 */
    fp32 vmax;      /**< 最大速度 */
    fp32 tmax;      /**< 最大扭矩 */
} esc_inf_t;

/** @brief MIT 控制指令 */
typedef struct
{
    uint8_t mode;   /**< 控制模式 */
    fp32 pos_set;   /**< 位置设定 */
    fp32 vel_set;   /**< 速度设定 */
    fp32 tor_set;   /**< 扭矩设定 */
    fp32 cur_set;   /**< 电流设定 */
    fp32 kp_set;    /**< 位置环 Kp */
    fp32 kd_set;    /**< 速度环 Kd */
} motor_ctrl_t;

#ifdef __cplusplus
}
#endif

/** @brief 通用电机基类 */
class Motor
{
public:
    float speed;        /**< 当前转速 (rad/s) */
    float speed_set;    /**< 目标转速 (rad/s) */
    float current_t;    /**< 当前扭矩 (N·m) */
    float current_give; /**< 输出扭矩指令 (N·m) */
    uint16_t can_id;    /**< CAN 通信 ID */
    Pid speed_pid;      /**< 速度环 PID */

    Motor();
    Motor(uint16_t id);

    void set(fp32 set, uint8_t mode);
    void solve(uint8_t mode);
};

/** @brief DM3519 电机设备类 */
class DM3519 : public Motor
{
public:
    uint16_t mst_id;                    /**< Master ID (CAN 命令字) */
    const dm_motor_measure_t *measure;  /**< 电机反馈数据指针 */
    esc_inf_t tmp;                      /**< 物理参数限制 */
    motor_ctrl_t ctrl;                  /**< MIT 控制指令 */

    DM3519() : Motor(), mst_id(0), measure(NULL) {}
    DM3519(uint16_t id) : Motor(id), mst_id(0), measure(NULL) {}
    DM3519(uint16_t id, uint16_t mst_id, const dm_motor_measure_t *measure)
        : Motor(id), mst_id(mst_id), measure(measure) {}

    void DM3519_Init();
    void update_measure();
};

/** @brief C615 电调电机 (PWM控制, 无反馈) */
class C615 : public Motor
{
public:
    uint32_t tim_channel;   /**< PWM通道 */
    fp32 max_speed;         /**< 最大转速 (rad/s) */

    C615() : Motor(), tim_channel(0), max_speed(0.0f) {}
    C615(uint32_t channel) : Motor(0), tim_channel(channel), max_speed(0.0f) {}

    void init(uint32_t channel);
    void update_measure();
};

/** @brief C610 电机 (CAN电流控制, 拨弹轮) */
class C610 : public Motor
{
public:
    const c610_motor_measure_t *measure;  /**< C610 CAN 反馈数据指针 */

    C610() : Motor(), measure(NULL) {}
    C610(uint16_t id, const c610_motor_measure_t *m) : Motor(id), measure(m) {}

    void init(uint16_t id, const c610_motor_measure_t *m);
    void update_measure();
};

#endif
