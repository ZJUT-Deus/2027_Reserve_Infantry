/**
 * @file    Shoot.h
 * @brief   射击控制模块 (C615摩擦轮 + C610拨弹)
 * @author  kk
 * @date    2026-06-04
 */

#ifndef SHOOT_H
#define SHOOT_H

#include "struct_typedef.h"
#include "Motor.h"

/** @brief 摩擦轮测试转速 (rad/s) */
#define SHOOT_TEST_SPEED 300.0f

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

/** @brief 射击控制主类 */
class Shoot
{
public:
    C615 friction_left;   /**< 左摩擦轮 (C615 PWM) */
    C615 friction_right;  /**< 右摩擦轮 (C615 PWM) */

    void init();
    void feedback_update();
    void set_control();
    void solve();
    void output();
};

extern Shoot shoot;

#ifdef __cplusplus
extern "C" {
#endif

void shoot_init(void);
void shoot_control_loop(void);

#ifdef __cplusplus
}
#endif

#endif
