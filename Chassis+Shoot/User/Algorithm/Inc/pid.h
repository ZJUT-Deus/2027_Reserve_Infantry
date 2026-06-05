/**
 * @file    pid.h
 * @brief   PID 控制器
 * @author  kk
 * @date    2026-05-22
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus

#include "struct_typedef.h"
#include "user_lib.h"

/** @brief 速度环 PID 模式 */
#define PID_SPEED  0
/** @brief 角度环 PID 模式 */
#define PID_ANGLE  1

/** @brief PID 参数与运行时数据 */
typedef struct
{
    fp32 Kp;            /**< 比例系数 */
    fp32 Ki;            /**< 积分系数 */
    fp32 Kd;            /**< 微分系数 */
    fp32 kf;            /**< 前馈系数 */

    fp32 max_iout;      /**< 积分输出限幅 */
    fp32 max_out;       /**< 总输出限幅 */

    fp32 *set;          /**< 设定值指针 */
    fp32 set_last;      /**< 上一次设定值 */
    fp32 *ref;          /**< 反馈值指针 */
    fp32 ref_last;      /**< 上一次反馈值 */
    fp32 error;         /**< 当前误差 */
    fp32 last_error;    /**< 上一次误差 */

    fp32 error_delta;   /**< 误差变化量 */

    fp32 out;           /**< PID 总输出 */
    fp32 Pout;          /**< 比例项输出 */
    fp32 Iout;          /**< 积分项输出 */
    fp32 Dout;          /**< 微分项输出 */
    fp32 Fout;          /**< 前馈项输出 */
} pid_data_t;

/** @brief PID 控制器类 */
class Pid
{
public:
    uint8_t mode;       /**< PID 模式, PID_SPEED 或 PID_ANGLE */
    pid_data_t data;    /**< PID 参数与运行数据 */

    /** @brief 默认构造函数 */
    Pid() {}

    /** @brief 带参数构造函数 */
    Pid(uint8_t mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_);

    void init(uint8_t mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_);
    fp32 pid_calc();
    void pid_clear();
    void Clear();
    void align_state_to_current();

private:
    void load_params(const fp32 *pid_parm);
};

#endif
#endif
