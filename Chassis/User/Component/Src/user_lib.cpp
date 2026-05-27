/**
 * @file    user_lib.cpp
 * @brief   通用工具函数与一阶低通滤波器实现
 * @author  kk
 * @date    2026-05-22
 */

#include "user_lib.h"

/**
 * @brief  将浮点数值限制在 [minValue, maxValue] 范围内 (饱和限幅)
 * @param  Value     输入值
 * @param  minValue  下限
 * @param  maxValue  上限
 * @return 限幅后的值
 */
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief  将 int16_t 数值限制在 [minValue, maxValue] 范围内 (饱和限幅)
 * @param  Value     输入值
 * @param  minValue  下限
 * @param  maxValue  上限
 * @return 限幅后的值
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief  将浮点数值循环限制在 [minValue, maxValue] 范围内 (回绕限幅)
 * @param  Input     输入值
 * @param  minValue  下限
 * @param  maxValue  上限
 * @return 回绕后的值 (超出上限则从下限重新开始, 低于下限则从上限重新开始)
 */
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

/**
 * @brief  初始化一阶低通滤波器
 * @param  frame_period 滤波时间间隔 (s)
 * @param  num          滤波参数数组
 * @retval none
 */
void First_order_filter::init(fp32 frame_period_, const fp32 *num_)
{
    frame_period = frame_period_;
    num[0] = num_[0];
    input = 0.0f;
    out = 0.0f;
}

/**
 * @brief  执行一次滤波计算
 * @param  input 输入数据
 * @retval none
 */
void First_order_filter::first_order_filter_cali(fp32 input_)
{
    input = input_;
    out = num[0] / (num[0] + frame_period) * out +
          frame_period / (num[0] + frame_period) * input;
}
