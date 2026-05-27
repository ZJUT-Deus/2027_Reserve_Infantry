/**
 * @file    user_lib.h
 * @brief   通用工具函数与一阶低通滤波器
 * @author  kk
 * @date    2026-05-22
 */

#ifndef USER_LIB_H
#define USER_LIB_H

#include "struct_typedef.h"

#ifdef __cplusplus

/**
 * @brief  将浮点数线性映射为无符号整数 (用于 CAN 通信编码)
 * @param  x_float 浮点输入值
 * @param  x_min   输入范围下限
 * @param  x_max   输入范围上限
 * @param  bits    目标位宽
 * @return 映射后的整数值
 */
inline int float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits)
{
    if (x_float > x_max) x_float = x_max;
    if (x_float < x_min) x_float = x_min;
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    return (int)((x_float - offset) * ((fp32)((1 << bits) - 1)) / span);
}

/**
 * @brief  将无符号整数还原为浮点数 (CAN 通信解码)
 * @param  x_int  整型输入值
 * @param  x_min  原始范围下限
 * @param  x_max  原始范围上限
 * @param  bits   位宽
 * @return 还原后的浮点值
 */
inline fp32 uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits)
{
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    return ((fp32)x_int) * span / ((fp32)((1 << bits) - 1)) + offset;
}

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

/** @brief 角度格式化到 [-PI, PI] 范围 */
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

/** @brief 一阶低通滤波器类 */
class First_order_filter
{
public:
    fp32 input;         /**< 滤波器输入 */
    fp32 out;           /**< 滤波器输出 */
    fp32 num[1];        /**< 滤波系数 */
    fp32 frame_period;  /**< 采样周期 */

    void init(fp32 frame_period, const fp32 num[1]);
    void first_order_filter_cali(fp32 input);
};

#endif /* __cplusplus */

#endif /* USER_LIB_H */
