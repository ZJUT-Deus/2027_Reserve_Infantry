/**
 * @file    bsp_pwm.h
 * @brief   PWM底层驱动接口 (C615电调控制)
 * @author  kk
 * @date    2026-06-04
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** C615电调PWM通道定义 */
#define PWM_C615_FRICTION_L  TIM_CHANNEL_1  /**< 左摩擦轮, PE9  */
#define PWM_C615_FRICTION_R  TIM_CHANNEL_3  /**< 右摩擦轮, PE13 */

/** C615电调脉宽范围 (tick, 1MHz定时器) */
#define C615_PULSE_MIN  1000  /**< 1.0ms, 全速反转 */
#define C615_PULSE_MAX  2000  /**< 2.0ms, 全速正转 */
#define C615_PULSE_MID  1500  /**< 1.5ms, 停止     */

void pwm_init(void);
void pwm_set_pulse(uint32_t channel, uint16_t pulse);

#ifdef __cplusplus
}
#endif

#endif
