/**
 * @file    Shoot.cpp
 * @brief   射击控制模块 (C615摩擦轮 + C610拨弹 + 防卡弹)
 *          遥控器指令由 Communicate 模块通过 C API 下发
 * @author  kk
 * @date    2026-06-05
 */

#include "Shoot.h"
#include "bsp_pwm.h"
#include "Can_receive.h"

Shoot shoot;

void Shoot::init()
{
    friction_left.init(PWM_C615_FRICTION_L);
    friction_right.init(PWM_C615_FRICTION_R);

    trigger_motor.init(C610_TRIGGER_MOTOR_ID,
                       can_receive.get_c610_motor_measure_point());

    fp32 trigger_speed_pid[6] = {
        TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI,
        TRIGGER_SPEED_PID_KD, TRIGGER_SPEED_PID_KF,
        TRIGGER_PID_MAX_IOUT, TRIGGER_PID_MAX_OUT
    };
    trigger_motor.speed_pid.init(PID_SPEED, trigger_speed_pid,
                                  &trigger_motor.speed,
                                  &trigger_motor.speed_set, NULL);
    trigger_motor.speed_pid.pid_clear();

    const static fp32 fric_filter_num[1] = {SHOOT_FRIC_RAMP_TAU};
    fric_ramp_left.init(SHOOT_CONTROL_DT_S, fric_filter_num);
    fric_ramp_right.init(SHOOT_CONTROL_DT_S, fric_filter_num);

    shoot_mode = SHOOT_STOP;
    reset_trigger_anti_jam();

    feedback_update();
}

void Shoot::feedback_update()
{
    friction_left.update_measure();
    friction_right.update_measure();
    trigger_motor.update_measure();
}

void Shoot::set_control()
{
    if (shoot_mode == SHOOT_STOP)
    {
        friction_left.speed_set  = 0.0f;
        friction_right.speed_set = 0.0f;
        trigger_motor.speed_set  = 0.0f;

        fric_ramp_left.out  = 0.0f;
        fric_ramp_right.out = 0.0f;

        reset_trigger_anti_jam();
    }
    else
    {
        fric_ramp_left.first_order_filter_cali(SHOOT_FRIC_SPEED);
        fric_ramp_right.first_order_filter_cali(SHOOT_FRIC_SPEED);
        friction_left.speed_set  = fric_ramp_left.out;
        friction_right.speed_set = fric_ramp_right.out;

        if (shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            trigger_forward_speed_set = SHOOT_TRIGGER_SPEED;
            trigger_motor.speed_set   = SHOOT_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            trigger_motor.speed_set = 0.0f;
            reset_trigger_anti_jam();
        }
    }
}

void Shoot::solve()
{
    if (shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        trigger_motor.solve(SPEED);
    }
    else
    {
        trigger_motor.current_give = 0.0f;
    }
}

void Shoot::output()
{
    fp32 ratio_l = fp32_constrain(friction_left.speed_set / friction_left.max_speed, 0.0f, 1.0f);
    fp32 ratio_r = fp32_constrain(friction_right.speed_set / friction_right.max_speed, 0.0f, 1.0f);

    uint16_t pulse_l = (uint16_t)(C615_PULSE_MIN + ratio_l * (C615_PULSE_MAX - C615_PULSE_MIN));
    uint16_t pulse_r = (uint16_t)(C615_PULSE_MIN + ratio_r * (C615_PULSE_MAX - C615_PULSE_MIN));

    pwm_set_pulse(friction_left.tim_channel, pulse_l);
    pwm_set_pulse(friction_right.tim_channel, pulse_r);

    can_receive.can_cmd_c610_motor((int16_t)trigger_motor.current_give);
}

/* ========== 防卡弹 ========== */

void Shoot::reset_trigger_anti_jam()
{
    block_time          = 0;
    reverse_time        = 0;
    recovery_time       = 0;
    startup_ignore_time = 0;
    forward_time        = 0;
    jam_retry_count     = 0;
    trigger_anti_jam_state    = TRIGGER_ANTI_JAM_IDLE;
    trigger_forward_speed_set = 0.0f;
    trigger_motor.speed_pid.pid_clear();
}

bool_t Shoot::trigger_motor_blocked()
{
    if (trigger_motor.measure == NULL)
    {
        return FALSE;
    }

    return (fp32_abs(trigger_motor.speed) < TRIGGER_JAM_SPEED_THRESHOLD) &&
           (int16_abs(trigger_motor.measure->torque_current) > TRIGGER_JAM_CURRENT_THRESHOLD);
}

void Shoot::trigger_motor_turn_back()
{
    if (fp32_abs(trigger_forward_speed_set) < TRIGGER_JAM_CMD_MIN_SPEED)
    {
        reset_trigger_anti_jam();
        trigger_motor.speed_set = 0.0f;
        return;
    }

    switch (trigger_anti_jam_state)
    {
    case TRIGGER_ANTI_JAM_IDLE:
        trigger_motor.speed_set = trigger_forward_speed_set;

        if (startup_ignore_time < TRIGGER_JAM_STARTUP_IGNORE)
        {
            startup_ignore_time++;
            block_time = 0;
            return;
        }

        if (trigger_motor_blocked())
        {
            forward_time = 0;
            if (block_time < TRIGGER_JAM_DETECT_TIME)
            {
                block_time++;
            }

            if (block_time >= TRIGGER_JAM_DETECT_TIME)
            {
                block_time    = 0;
                reverse_time  = 0;
                recovery_time = 0;
                startup_ignore_time = 0;
                trigger_motor.speed_pid.pid_clear();

                if (jam_retry_count < 255)
                {
                    jam_retry_count++;
                }
                trigger_anti_jam_state = TRIGGER_ANTI_JAM_REVERSE;
            }
        }
        else
        {
            block_time = 0;
            if (forward_time < TRIGGER_JAM_RETRY_RESET_TIME)
            {
                forward_time++;
            }
            if (forward_time >= TRIGGER_JAM_RETRY_RESET_TIME)
            {
                jam_retry_count = 0;
            }
        }
        break;

    case TRIGGER_ANTI_JAM_REVERSE:
        trigger_motor.speed_set = -TRIGGER_JAM_REVERSE_SPEED;
        if (reverse_time < TRIGGER_JAM_REVERSE_TIME)
        {
            reverse_time++;
        }
        else
        {
            reverse_time  = 0;
            recovery_time = 0;
            trigger_motor.speed_pid.pid_clear();
            trigger_anti_jam_state = TRIGGER_ANTI_JAM_RECOVERY;
        }
        break;

    case TRIGGER_ANTI_JAM_RECOVERY:
        trigger_motor.speed_set = 0.0f;
        if (recovery_time < TRIGGER_JAM_RECOVERY_TIME)
        {
            recovery_time++;
        }
        else if (jam_retry_count >= TRIGGER_JAM_RETRY_MAX)
        {
            trigger_motor.speed_pid.pid_clear();
            trigger_anti_jam_state = TRIGGER_ANTI_JAM_LOCKED;
        }
        else
        {
            recovery_time       = 0;
            block_time          = 0;
            startup_ignore_time = 0;
            forward_time        = 0;
            trigger_motor.speed_pid.pid_clear();
            trigger_anti_jam_state = TRIGGER_ANTI_JAM_IDLE;
        }
        break;

    case TRIGGER_ANTI_JAM_LOCKED:
    default:
        trigger_motor.speed_set = 0.0f;
        break;
    }
}

/* ========== C 接口 (模块内部控制) ========== */

void shoot_init(void)
{
    shoot.init();
}

void shoot_control_loop(void)
{
    shoot.feedback_update();
    shoot.set_control();
    shoot.solve();
    shoot.output();
}

/* ========== C 接口 (Communicate 调用, 设置模式) ========== */

void shoot_ready(void)
{
    shoot.shoot_mode = SHOOT_READY;
}

void shoot_continue_bullet(void)
{
    shoot.shoot_mode = SHOOT_CONTINUE_BULLET;
}

void shoot_stop(void)
{
    shoot.shoot_mode = SHOOT_STOP;
}
