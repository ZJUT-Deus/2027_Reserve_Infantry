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

#endif
