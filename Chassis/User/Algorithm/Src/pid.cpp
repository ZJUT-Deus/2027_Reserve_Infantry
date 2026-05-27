/**
 * @file    pid.cpp
 * @brief   PID控制器实现
 * @author  kk
 * @date    2026-05-22
 */

#include "pid.h"

/**
 * @brief  将数值限制在 [-max, max] 范围内
 * @param  input 输入/输出值
 * @param  max   限幅绝对值
 */
static inline void LimitMax(fp32 &input, fp32 max)
{
    if (input > max)
    {
        input = max;
    }
    else if (input < -max)
    {
        input = -max;
    }
}

/**
 * @brief  从数组加载 PID 参数
 * @param  pid_parm PID 参数数组 [Kp, Ki, Kd, Kf, max_iout, max_out]
 */
void Pid::load_params(const fp32 *pid_parm)
{
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.kf = pid_parm[3];
    data.max_iout = pid_parm[4];
    data.max_out = pid_parm[5];
}

/**
 * @brief  带参数构造, 自动调用 init
 * @param  mode_   PID 模式 (PID_SPEED 或 PID_ANGLE)
 * @param  pid_parm PID 参数数组
 * @param  ref_    反馈值指针
 * @param  set_    设定值指针
 */
Pid::Pid(uint8_t mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_)
{
    init(mode_, pid_parm, ref_, set_, 0.0f);
}

/**
 * @brief  初始化 PID 控制器
 * @param  mode_      PID 模式 (PID_SPEED 或 PID_ANGLE)
 * @param  pid_parm   PID 参数数组
 * @param  ref_       反馈值指针
 * @param  set_       设定值指针
 * @param  erro_delta_ 初始误差变化量 (角度模式用)
 */
void Pid::init(uint8_t mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_)
{
    mode = mode_;
    load_params(pid_parm);
    data.set = set_;
    data.set_last = *set_;
    data.ref = ref_;
    data.ref_last = *ref_;
    data.error = *set_ - *ref_;

    data.last_error = data.error;
    data.error_delta = 0.0f;
    data.out = 0.0f;
    data.Pout = 0.0f;
    data.Iout = 0.0f;
    data.Dout = 0.0f;
    data.Fout = 0.0f;

    if (mode == PID_ANGLE)
    {
        data.error = rad_format(data.error);
        data.last_error = data.error;
        data.error_delta = erro_delta_;
    }
}

/**
 * @brief  执行一次 PID 计算
 * @return PID 输出值
 */
fp32 Pid::pid_calc()
{
    data.last_error = data.error;
    data.error = *data.set - *data.ref;

    if (mode == PID_SPEED)
        data.error_delta = data.error - data.last_error;

    if (mode == PID_ANGLE) {
        data.error = rad_format(data.error);
        data.error_delta = data.error - data.last_error;
    }

    data.Pout = data.Kp * data.error;
    data.Iout += data.Ki * data.error;
    data.Dout = data.Kd * data.error_delta;
    data.Fout = data.kf * (*data.set - data.set_last);

    LimitMax(data.Iout, data.max_iout);

    data.ref_last = *data.ref;
    data.set_last = *data.set;
    data.out = data.Pout + data.Iout + data.Dout + data.Fout;
    LimitMax(data.out, data.max_out);

    return data.out;
}

/**
 * @brief  清零 PID 所有状态
 */
void Pid::pid_clear()
{
    data.last_error = 0.0f;
    data.error = 0.0f;
    data.error_delta = 0.0f;
    data.set_last = 0.0f;
    data.ref_last = 0.0f;
    *data.set = 0.0f;
    *data.ref = 0.0f;
    data.out = 0.0f;
    data.Pout = 0.0f;
    data.Iout = 0.0f;
    data.Dout = 0.0f;
    data.Fout = 0.0f;
}

/**
 * @brief  重置 PID 积分和微分项 (保持 set/ref 当前值)
 */
void Pid::Clear()
{
    data.error_delta = 0.0f;
    data.set_last = *data.set;
    data.ref_last = *data.ref;
    data.Iout = 0.0f;
    data.Dout = 0.0f;
    data.out = 0.0f;
    data.Fout = 0.0f;
}

/**
 * @brief  将 PID 状态对齐到当前 set/ref, 消除历史累积
 */
void Pid::align_state_to_current()
{
    fp32 current_set = 0.0f;
    fp32 current_ref = 0.0f;
    fp32 current_error = 0.0f;

    if (data.set)
        current_set = *data.set;

    if (data.ref)
        current_ref = *data.ref;

    current_error = current_set - current_ref;
    if (mode == PID_ANGLE)
        current_error = rad_format(current_error);

    data.set_last = current_set;
    data.ref_last = current_ref;
    data.error = current_error;
    data.last_error = current_error;
    data.error_delta = 0.0f;
    data.out = 0.0f;
    data.Pout = 0.0f;
    data.Iout = 0.0f;
    data.Dout = 0.0f;
    data.Fout = 0.0f;
}
