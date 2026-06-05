/**
 * @file    bsp_pwm.cpp
 * @brief   PWM底层驱动实现 (C615电调控制)
 * @author  kk
 * @date    2026-06-04
 */

#include "bsp_pwm.h"

/**
 * @brief  启动TIM1双路PWM并输出中立脉宽初始化电调
 */
void pwm_init(void)
{
    HAL_TIM_PWM_Start(&htim1, PWM_C615_FRICTION_L);
    HAL_TIM_PWM_Start(&htim1, PWM_C615_FRICTION_R);

    pwm_set_pulse(PWM_C615_FRICTION_L, C615_PULSE_MIN);
    pwm_set_pulse(PWM_C615_FRICTION_R, C615_PULSE_MIN);

    HAL_Delay(C615_SELFCHECK_TIME_MS);
}

/**
 * @brief  直接设置PWM通道的比较值 (脉宽)
 * @param  channel TIM通道 (PWM_C615_FRICTION_L / PWM_C615_FRICTION_R)
 * @param  pulse   脉宽 (tick), 范围 1000~2000
 */
void pwm_set_pulse(uint32_t channel, uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}
