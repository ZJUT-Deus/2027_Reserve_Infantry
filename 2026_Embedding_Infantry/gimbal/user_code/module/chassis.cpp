#include "chassis.h"



//底盘模块 对象
Chassis chassis;
volatile top_tune_debug_t top_tune_debug = {0};

uint8_t key_pressed_num_ctrl = 0;

//小陀螺控制数据
fp32 top_angle = 0.0f;
//bool_t top_switch = TOP_SWITCH_DEFAULT;

//超电控制数据
bool_t super_cap_switch = 1; //超级电容使能开关

volatile uint16_t zh;


//底盘电机速度环PID       p     i    D    f    IOUT     OUT
float Chassis_SPEED_PID[6] = {800.0f,2.0f,15.0f,0.0f,2000.0f,8000.0f};

float Chassis_ENCODE_PID[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

//底盘跟随PID
float FOLLOW_PID[6] = {16.66666f,0.0099999f,666.0f,0.0f,0.233333f,10.0f};

uint16_t  CHASSIS_CANID[4]={CAN_MOTIVE_FR_MOTOR_ID,CAN_MOTIVE_FL_MOTOR_ID,CAN_MOTIVE_BL_MOTOR_ID,CAN_MOTIVE_BR_MOTOR_ID};

static fp32 chassis_power_abs(fp32 value)
{
    return value >= 0.0f ? value : -value;
}

static fp32 chassis_power_pick_conservative_current(fp32 command_current_a, fp32 feedback_current_a)
{
    return chassis_power_abs(command_current_a) >= chassis_power_abs(feedback_current_a) ? command_current_a : feedback_current_a;
}

static fp32 chassis_power_estimate_motor(fp32 current_a, fp32 omega)
{
    fp32 power = CHASSIS_POWER_MODEL_K0 * current_a * omega +
                 CHASSIS_POWER_MODEL_K1 * omega * omega +
                 CHASSIS_POWER_MODEL_K2 * current_a * current_a +
                 CHASSIS_POWER_MODEL_A;
    power *= CHASSIS_POWER_ESTIMATE_SCALE;
    return power > 0.0f ? power : 0.0f;
}

static fp32 chassis_power_estimate_total(const fp32 current_a[4], const fp32 omega[4], fp32 eta)
{
    fp32 total_power = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        total_power += chassis_power_estimate_motor(current_a[i] * eta, omega[i]);
    }
    return total_power;
}

static fp32 chassis_power_solve_eta(const fp32 current_a[4], const fp32 omega[4], fp32 power_budget)
{
    fp32 low = 0.0f;
    fp32 high = 1.0f;

    for (int i = 0; i < 20; i++)
    {
        fp32 mid = 0.5f * (low + high);
        fp32 power_mid = chassis_power_estimate_total(current_a, omega, mid);
        if (power_mid > power_budget)
        {
            high = mid;
        }
        else
        {
            low = mid;
        }
    }

    return low;
}

static fp32 get_top_var_stage_speed(top_var_stage_e stage)
{
    if (stage == TOP_VAR_STAGE_1)
    {
        return TOP_WZ_ANGLE_VAR_STAGE1;
    }
    else if (stage == TOP_VAR_STAGE_2)
    {
        return TOP_WZ_ANGLE_VAR_STAGE2;
    }
    else if (stage == TOP_VAR_STAGE_3)
    {
        return TOP_WZ_ANGLE_VAR_STAGE3;
    }

    return TOP_WZ_ANGLE_STAND;
}


void Chassis::init()
{    
    chassis_RC = vt13.get_vt13_remote_control_point();
    last_chassis_RC = vt13.get_last_vt13_remote_control_point();

    chassis_last_key_v = 0;
    
    //设置初试状态机
    chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    last_chassis_behaviour_mode = chassis_behaviour_mode;
    chassis_relative_angle = 0.0f;
    last_chassis_relative_angle = 0.0f;

    for(int i=0;i<4;i++)
    {
        chassis_motive_motor[i] = DJI_Motor(CHASSIS_CANID[i],  can_receive.get_motor_chassis_measure_point(i), 0, DJI_ECD_RANGE);
        chassis_motive_motor[i].speed_pid.init(PID_SPEED, Chassis_SPEED_PID, &chassis_motive_motor[i].speed, &chassis_motive_motor[i].speed_set, NULL);
        chassis_motive_motor[i].encode_angle_pid.init(PID_ANGLE, Chassis_ENCODE_PID, &chassis_motive_motor[i].encode_angle, &chassis_motive_motor[i].encode_angle_set, NULL);
        // chassis_motive_motor[i].gyro_angle_pid.init(PID_ANGLE, Chassis_GYRO_PID, &chassis_motive_motor[i].gyro_angle, &chassis_motive_motor[i].gyro_angle_set, NULL);
        chassis_motive_motor[i].speed_pid.pid_clear();
    }

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜波函数生成
    chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //速度限幅设置
    x.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    x.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;

    y.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    y.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;

    z.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
    z.max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;

    //更新一下数据
    feedback_update();
    last_chassis_relative_angle = chassis_relative_angle;

    power_eta = 1.0f;
    power_estimate_w = 0.0f;
    power_estimate_raw_w = 0.0f;
    power_budget_w = CHASSIS_POWER_LIMIT_W;

    reset_top_spin_state_machine();
    last_referee_hp = referee.current_HP;
    referee_hp_initialized = TRUE;
    top_switch = TOP_SWITCH_DEFAULT;
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]
 * @retval         none
 */
void Chassis::set_mode()
{
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    //遥控器设置模式
    if (switch_is_up(chassis_RC->rc.mode_sw)) //左C 底盘行为 跟随云台
    {
        chassis_behaviour_mode = CHASSIS_TOP;
    }
    else if (switch_is_mid(chassis_RC->rc.mode_sw)) //中N 底盘行为 自主运动
    {
        chassis_behaviour_mode = CHASSIS_FREE;
    }
    else if (switch_is_down(chassis_RC->rc.mode_sw)) //右S 底盘行为 无力
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
}


/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]
 * @retval         none
 */
void Chassis::feedback_update()
{   
    chassis_last_key_v = chassis_RC->key.v;
    last_chassis_mode = chassis_mode;
    //切入不跟随云台模式
    
    if ((last_chassis_mode == CHASSIS_ZERO_FORCE))
    {
        chassis_yaw_set = 0;
    }
    
    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        chassis_motive_motor[i].update();
    }


    x.speed = (-chassis_motive_motor[0].speed + chassis_motive_motor[1].speed + chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX/1.414f*MOTOR_WHEEL_RADIUS;
    y.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed + chassis_motive_motor[2].speed + chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY/1.414f*MOTOR_WHEEL_RADIUS;
    z.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed - chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ*MOTOR_WHEEL_RADIUS / MOTOR_DISTANCE_TO_CENTER;


    //底盘相对于云台的角度 编码器中的角度
     last_chassis_relative_angle = chassis_relative_angle;
     chassis_relative_angle = gimbal.gimbal_yaw_motor.encode_angle-2.3569f;


     //底盘功率控制
     can_receive.can_cmd_super_cap(super_cap_switch,0,referee.chassis_power_limit,referee.chassis_power_buffer,0x061);

     if (chassis_behaviour_mode != CHASSIS_TOP)
     {
         // 只有 TOP 模式才有资格消费“受击后变速”事件；离开 TOP 后立即丢弃历史 pulse。
         referee.discard_projectile_hit_event();
     }

     if (last_chassis_behaviour_mode != CHASSIS_TOP && chassis_behaviour_mode == CHASSIS_TOP)
     {
          // 状态机切换：刚进入 CHASSIS_TOP 时重置受击变速小陀螺，避免把历史掉血误判为新受击
          reset_top_spin_state_machine();
          referee.discard_projectile_hit_event();
          last_referee_hp = referee.current_HP;
          referee_hp_initialized = TRUE;
     }
     else if (last_chassis_behaviour_mode == CHASSIS_TOP && chassis_behaviour_mode != CHASSIS_TOP)
     {
          // 状态机切换：离开 CHASSIS_TOP 时清空变速序列，禁止把旧状态带到其它模式
          reset_top_spin_state_machine();
          referee.discard_projectile_hit_event();
          last_referee_hp = referee.current_HP;
          referee_hp_initialized = TRUE;
     }

     //UI彩灯更新

}

void Chassis::reset_top_spin_state_machine()
{
    top_spin_state = TOP_SPIN_STEADY;
    top_var_stage = TOP_VAR_STAGE_NONE;
    top_var_stage_start_ms = 0U;
    top_no_move_start_ms = 0U;
    top_hit_event_pending_start_ms = 0U;
    top_no_move_timer_started = FALSE;
    top_no_move_confirmed = FALSE;
    top_move_intent = FALSE;
    top_hit_event_pending = FALSE;
}

bool_t Chassis::chassis_has_move_intent()
{
    if (chassis_RC == NULL)
    {
        return FALSE;
    }

    if (chassis_RC->rc.ch[CHASSIS_X_CHANNEL] > TOP_MOVE_INTENT_RC_DEADLINE ||
        chassis_RC->rc.ch[CHASSIS_X_CHANNEL] < -TOP_MOVE_INTENT_RC_DEADLINE)
    {
        return TRUE;
    }

    if (chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] > TOP_MOVE_INTENT_RC_DEADLINE ||
        chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] < -TOP_MOVE_INTENT_RC_DEADLINE)
    {
        return TRUE;
    }

    if (KEY_CHASSIS_FRONT || KEY_CHASSIS_BACK || KEY_CHASSIS_LEFT || KEY_CHASSIS_RIGHT)
    {
        return TRUE;
    }

    return FALSE;
}

void Chassis::update_top_no_move_confirm(uint32_t now_ms)
{
    top_move_intent = chassis_has_move_intent();

    if (top_move_intent == TRUE)
    {
        top_no_move_timer_started = FALSE;
        top_no_move_confirmed = FALSE;
        top_no_move_start_ms = 0U;
        return;
    }

    if (top_no_move_timer_started == FALSE)
    {
        top_no_move_timer_started = TRUE;
        top_no_move_start_ms = now_ms;
        top_no_move_confirmed = FALSE;
        return;
    }

    top_no_move_confirmed = ((now_ms - top_no_move_start_ms) >= TOP_NO_MOVE_CONFIRM_TIME_MS) ? TRUE : FALSE;
}

bool_t Chassis::consume_valid_hit_event()
{
    // 底盘侧不再自己解释 referee 裸快照，只消费通信层已经严格筛过的一次性命中事件。
    return referee.consume_projectile_hit_event();
}

void Chassis::start_top_var_spin(uint32_t now_ms)
{
    top_spin_state = TOP_SPIN_VAR_SPIN;
    top_var_stage = TOP_VAR_STAGE_1;
    top_var_stage_start_ms = now_ms;
}

fp32 Chassis::update_top_spin_command(uint32_t now_ms)
{
    update_top_no_move_confirm(now_ms);

#if TOP_VAR_SPIN_ENABLE
    const bool_t valid_hit_event = consume_valid_hit_event();
#endif

    if (top_move_intent == TRUE)
    {
        if (top_spin_state != TOP_SPIN_STEADY || top_var_stage != TOP_VAR_STAGE_NONE || top_hit_event_pending == TRUE)
        {
            // 状态机切换：操作手一旦给出移动意图，立即退出变速小陀螺并回到常态 15rad/s
            reset_top_spin_state_machine();
        }
        return TOP_WZ_ANGLE_STAND;
    }

#if !TOP_VAR_SPIN_ENABLE
    if (top_spin_state != TOP_SPIN_STEADY || top_var_stage != TOP_VAR_STAGE_NONE || top_hit_event_pending == TRUE)
    {
        reset_top_spin_state_machine();
    }
    return TOP_WZ_ANGLE_STAND;
#else
    if (valid_hit_event == TRUE)
    {
        if (top_no_move_confirmed == TRUE)
        {
            // 状态机切换：静止窗口内检测到装甲模块被弹丸攻击导致的扣血，进入/重启阶梯变速小陀螺
            start_top_var_spin(now_ms);
            top_hit_event_pending = FALSE;
            top_hit_event_pending_start_ms = 0U;
            return get_top_var_stage_speed(top_var_stage);
        }

        top_hit_event_pending = TRUE;
        top_hit_event_pending_start_ms = now_ms;
    }

    if (top_hit_event_pending == TRUE)
    {
        if (top_no_move_confirmed == TRUE)
        {
            // 状态机切换：刚恢复静止确认时补触发上一拍受击，避免事件在重新静止的空档里被吃掉
            start_top_var_spin(now_ms);
            top_hit_event_pending = FALSE;
            top_hit_event_pending_start_ms = 0U;
            return get_top_var_stage_speed(top_var_stage);
        }
        else if ((now_ms - top_hit_event_pending_start_ms) > TOP_NO_MOVE_CONFIRM_TIME_MS)
        {
            top_hit_event_pending = FALSE;
            top_hit_event_pending_start_ms = 0U;
        }
    }

    if (top_spin_state == TOP_SPIN_VAR_SPIN)
    {
        const uint32_t stage_elapsed_ms = now_ms - top_var_stage_start_ms;

        if (top_var_stage == TOP_VAR_STAGE_1 && stage_elapsed_ms >= TOP_WZ_VAR_STAGE1_TIME_MS)
        {
            // 状态机切换：Stage1 -> Stage2，继续提升到受击峰值小陀螺速度
            top_var_stage = TOP_VAR_STAGE_2;
            top_var_stage_start_ms = now_ms;
        }
        else if (top_var_stage == TOP_VAR_STAGE_2 && stage_elapsed_ms >= TOP_WZ_VAR_STAGE2_TIME_MS)
        {
            // 状态机切换：Stage2 -> Stage3，从峰值回落到过渡速度
            top_var_stage = TOP_VAR_STAGE_3;
            top_var_stage_start_ms = now_ms;
        }
        else if (top_var_stage == TOP_VAR_STAGE_3 && stage_elapsed_ms >= TOP_WZ_VAR_STAGE3_TIME_MS)
        {
            // 状态机切换：变速序列结束，回到常态小陀螺 15rad/s
            reset_top_spin_state_machine();
            return TOP_WZ_ANGLE_STAND;
        }

        return get_top_var_stage_speed(top_var_stage);
    }

    return TOP_WZ_ANGLE_STAND;
#endif
}

fp32 Chassis::calc_top_feedforward_delay(fp32 relative_omega_abs)
{
    if (TOP_RELATIVE_FF_OMEGA_MAX <= TOP_RELATIVE_FF_OMEGA_MIN)
    {
        fp32 fixed_scale = 1.0f + (1.0f - fp32_constrain(power_eta, 0.0f, 1.0f)) * TOP_RELATIVE_FF_POWER_GAIN;
        fixed_scale = fp32_constrain(fixed_scale, 1.0f, TOP_RELATIVE_FF_DELAY_SCALE_MAX);
        return fp32_constrain(TOP_RELATIVE_FF_DELAY_MAX * fixed_scale,
                              TOP_RELATIVE_FF_DELAY_MIN,
                              TOP_RELATIVE_FF_DELAY_MAX * TOP_RELATIVE_FF_DELAY_SCALE_MAX);
    }

    fp32 omega_schedule = fp32_constrain(relative_omega_abs,
                                         TOP_RELATIVE_FF_OMEGA_MIN,
                                         TOP_RELATIVE_FF_OMEGA_MAX);
    fp32 omega_ratio = (omega_schedule - TOP_RELATIVE_FF_OMEGA_MIN) /
                       (TOP_RELATIVE_FF_OMEGA_MAX - TOP_RELATIVE_FF_OMEGA_MIN);
    fp32 base_delay = TOP_RELATIVE_FF_DELAY_MIN +
                      omega_ratio * (TOP_RELATIVE_FF_DELAY_MAX - TOP_RELATIVE_FF_DELAY_MIN);
    fp32 eta_scale = 1.0f + (1.0f - fp32_constrain(power_eta, 0.0f, 1.0f)) * TOP_RELATIVE_FF_POWER_GAIN;
    eta_scale = fp32_constrain(eta_scale, 1.0f, TOP_RELATIVE_FF_DELAY_SCALE_MAX);
    return fp32_constrain(base_delay * eta_scale,
                          TOP_RELATIVE_FF_DELAY_MIN,
                          TOP_RELATIVE_FF_DELAY_MAX * TOP_RELATIVE_FF_DELAY_SCALE_MAX);
}

fp32 Chassis::calc_top_feedforward_delta_angle(fp32 *feedforward_delay)
{
    fp32 relative_delta_angle = rad_format(chassis_relative_angle - last_chassis_relative_angle);
    fp32 relative_omega = relative_delta_angle / CHASSIS_CONTROL_TIME;
    fp32 current_feedforward_delay = calc_top_feedforward_delay(fabs(relative_omega));
    fp32 feedforward_delta_angle = relative_omega * current_feedforward_delay;

    if (feedforward_delay != NULL)
    {
        *feedforward_delay = current_feedforward_delay;
    }

    return fp32_constrain(feedforward_delta_angle,
                          -TOP_RELATIVE_FF_MAX_DELTA_ANGLE,
                          TOP_RELATIVE_FF_MAX_DELTA_ANGLE);
}

void Chassis::clear_top_tune_debug()
{
    top_tune_debug.ref_vx = 0.0f;
    top_tune_debug.ref_vy = 0.0f;
    top_tune_debug.ref_speed_norm = 0.0f;
    top_tune_debug.meas_vx_ref = 0.0f;
    top_tune_debug.meas_vy_ref = 0.0f;
    top_tune_debug.meas_speed_norm = 0.0f;
    top_tune_debug.direction_error_rad = 0.0f;
    top_tune_debug.direction_error_deg = 0.0f;
    top_tune_debug.lateral_speed = 0.0f;
    top_tune_debug.lateral_ratio = 0.0f;
    top_tune_debug.relative_omega = 0.0f;
    top_tune_debug.equivalent_delay_s = 0.0f;
    top_tune_debug.equivalent_delay_ms = 0.0f;
    top_tune_debug.feedforward_delay_s = 0.0f;
    top_tune_debug.feedforward_delay_ms = 0.0f;
    top_tune_debug.feedforward_delta_angle_rad = 0.0f;
    top_tune_debug.feedforward_delta_angle_deg = 0.0f;
    top_tune_debug.active = 0;
    top_tune_debug.valid_direction = 0;
    top_tune_debug.valid_delay = 0;
}

void Chassis::update_top_tune_debug(fp32 vx_set, fp32 vy_set, fp32 feedforward_delay, fp32 feedforward_delta_angle)
{
    fp32 measured_sin_yaw = sin(chassis_relative_angle);
    fp32 measured_cos_yaw = cos(chassis_relative_angle);
    fp32 ref_speed_norm = sqrt(vx_set * vx_set + vy_set * vy_set);
    fp32 meas_vx_ref = measured_cos_yaw * x.speed + measured_sin_yaw * y.speed;
    fp32 meas_vy_ref = measured_sin_yaw * x.speed - measured_cos_yaw * y.speed;
    fp32 meas_speed_norm = sqrt(meas_vx_ref * meas_vx_ref + meas_vy_ref * meas_vy_ref);
    fp32 relative_delta_angle = rad_format(chassis_relative_angle - last_chassis_relative_angle);
    fp32 relative_omega = relative_delta_angle / CHASSIS_CONTROL_TIME;

    top_tune_debug.ref_vx = vx_set;
    top_tune_debug.ref_vy = vy_set;
    top_tune_debug.ref_speed_norm = ref_speed_norm;
    top_tune_debug.meas_vx_ref = meas_vx_ref;
    top_tune_debug.meas_vy_ref = meas_vy_ref;
    top_tune_debug.meas_speed_norm = meas_speed_norm;
    top_tune_debug.relative_omega = relative_omega;
    top_tune_debug.feedforward_delay_s = feedforward_delay;
    top_tune_debug.feedforward_delay_ms = feedforward_delay * 1000.0f;
    top_tune_debug.feedforward_delta_angle_rad = feedforward_delta_angle;
    top_tune_debug.feedforward_delta_angle_deg = feedforward_delta_angle * TOP_TUNE_RAD_TO_DEG;
    top_tune_debug.active = 1;
    top_tune_debug.valid_direction = 0;
    top_tune_debug.valid_delay = 0;

    if (ref_speed_norm > TOP_TUNE_MIN_REF_SPEED && meas_speed_norm > TOP_TUNE_MIN_REF_SPEED)
    {
        fp32 direction_cross = vx_set * meas_vy_ref - vy_set * meas_vx_ref;
        fp32 direction_dot = vx_set * meas_vx_ref + vy_set * meas_vy_ref;
        fp32 lateral_speed = (-vy_set * meas_vx_ref + vx_set * meas_vy_ref) / ref_speed_norm;
        fp32 direction_error_rad = atan2(direction_cross, direction_dot);

        top_tune_debug.direction_error_rad = direction_error_rad;
        top_tune_debug.direction_error_deg = direction_error_rad * TOP_TUNE_RAD_TO_DEG;
        top_tune_debug.lateral_speed = lateral_speed;
        top_tune_debug.lateral_ratio = lateral_speed / ref_speed_norm;
        top_tune_debug.valid_direction = 1;

        if (fabs(relative_omega) > TOP_TUNE_MIN_RELATIVE_OMEGA)
        {
            top_tune_debug.equivalent_delay_s = direction_error_rad / relative_omega;
            top_tune_debug.equivalent_delay_ms = top_tune_debug.equivalent_delay_s * 1000.0f;
            top_tune_debug.valid_delay = 1;
        }
        else
        {
            top_tune_debug.equivalent_delay_s = 0.0f;
            top_tune_debug.equivalent_delay_ms = 0.0f;
        }
    }
    else
    {
        top_tune_debug.direction_error_rad = 0.0f;
        top_tune_debug.direction_error_deg = 0.0f;
        top_tune_debug.lateral_speed = 0.0f;
        top_tune_debug.lateral_ratio = 0.0f;
        top_tune_debug.equivalent_delay_s = 0.0f;
        top_tune_debug.equivalent_delay_ms = 0.0f;
    }
}

fp32 move_top_xyz_parm[3] = {1.0, 1.0, 1.3};
fp32 chassis_power_cap_buffer = 0.0f; //电容剩余能量
/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Chassis::set_contorl()
{
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

    //获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);

    //小陀螺模式下由于电控和机械延时导致移动偏移 增加解算前馈
    if (chassis_behaviour_mode == CHASSIS_TOP)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        fp32 rotate_relative_angle = chassis_relative_angle;
        fp32 feedforward_delay = 0.0f;
        fp32 feedforward_delta_angle = 0.0f;

        if (top_switch == TRUE)
        {
            feedforward_delta_angle = calc_top_feedforward_delta_angle(&feedforward_delay);
            rotate_relative_angle = rad_format(chassis_relative_angle + TOP_RELATIVE_FF_SIGN * feedforward_delta_angle);
        }

        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = sin(rotate_relative_angle);
        cos_yaw = cos(rotate_relative_angle);

        x.speed_set = cos_yaw * vx_set + sin_yaw * vy_set;
        y.speed_set = -(-sin_yaw * vx_set + cos_yaw * vy_set);


        z.speed_set = angle_set;

        x.speed_set = fp32_constrain(x.speed_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(y.speed_set, y.min_speed, y.max_speed);
        z.speed_set = fp32_constrain(z.speed_set, z.min_speed, z.max_speed);

        update_top_tune_debug(vx_set, vy_set, feedforward_delay, feedforward_delta_angle);
         
    }
    else if (chassis_behaviour_mode == CHASSIS_FREE)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = sin(chassis_relative_angle);
        cos_yaw = cos(chassis_relative_angle);

        x.speed_set = cos_yaw * vx_set + sin_yaw * vy_set;
        y.speed_set = -(-sin_yaw * vx_set + cos_yaw * vy_set);

        x.speed_set = fp32_constrain(x.speed_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(y.speed_set, y.min_speed, y.max_speed);
        z.speed_set = 0;
        clear_top_tune_debug();
    }
    else
    {
        clear_top_tune_debug();
    }

    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //动力电机目标速度

    chassis_vector_to_mecanum_wheel_speed(wheel_speed);

    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].set(wheel_speed[i], SPEED);
       
    }

 

}


void Chassis::solve()
{
    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].solve(SPEED);
				
	
    }
}
void Chassis::power_ctrl()
{
    power_budget_w = CHASSIS_POWER_LIMIT_W - CHASSIS_POWER_GUARD_W;
    if (power_budget_w < 0.0f)
    {
        power_budget_w = 0.0f;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        power_eta = 1.0f;
        power_estimate_raw_w = 0.0f;
        power_estimate_w = 0.0f;
        return;
    }

    fp32 current_a[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    fp32 omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 4; i++)
    {
        fp32 command_current_a = chassis_motive_motor[i].current_give * CHASSIS_POWER_CURRENT_TO_A;
        fp32 feedback_current_a = chassis_motive_motor[i].motor_measure->given_current * CHASSIS_POWER_CURRENT_TO_A;
        current_a[i] = chassis_power_pick_conservative_current(command_current_a, feedback_current_a);
        omega[i] = chassis_motive_motor[i].speed;
    }

    power_estimate_raw_w = chassis_power_estimate_total(current_a, omega, 1.0f);

    fp32 eta = 1.0f;
    if (power_estimate_raw_w > power_budget_w)
    {
        if (chassis_power_estimate_total(current_a, omega, 0.0f) >= power_budget_w)
        {
            eta = 0.0f;
        }
        else
        {
            eta = chassis_power_solve_eta(current_a, omega, power_budget_w);
        }
    }

    eta = fp32_constrain(eta, 0.0f, 1.0f);
    power_eta = eta;

    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].current_give *= eta;
        chassis_motive_motor[i].speed_pid.data.Iout *= eta;
        chassis_motive_motor[i].speed_pid.data.Iout = fp32_constrain(chassis_motive_motor[i].speed_pid.data.Iout,
                                                                     -chassis_motive_motor[i].speed_pid.data.max_iout,
                                                                     chassis_motive_motor[i].speed_pid.data.max_iout);
        chassis_motive_motor[i].current_give = fp32_constrain(chassis_motive_motor[i].current_give,
                                                              -MAX_MOTOR_CAN_CURRENT,
                                                              MAX_MOTOR_CAN_CURRENT);
    }

    power_estimate_w = chassis_power_estimate_total(current_a, omega, eta);
}




void Chassis::output()
{
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].current_give = 0;
        }
    }
    can_receive.can_cmd_dji_motor(chassis_motive_motor[0].current_give, chassis_motive_motor[1].current_give,
                                chassis_motive_motor[2].current_give, chassis_motive_motor[3].current_give,CAN_CHASSIS_MOTIVE_ALL_ID);
}


/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]       包括底盘所有信息.
 * @retval         none
 */
void Chassis::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_TOP)
    {
        chassis_top_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_FREE)
    {
        chassis_free_control(vx_set, vy_set, angle_set);
    }

}

 
void Chassis::chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}




/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @retval         返回空
 */
void Chassis::chassis_top_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set);
    
    // if (IF_MOUSE_PRESSED_MID_VT13_LAST && !IF_MOUSE_PRESSED_MID_VT13)
    // {
    //     // 鼠标中键点击事件，切换小陀螺开关状态
    //     top_switch = !top_switch;
    // }
    if (top_switch == FALSE)
    {
        zh++;
        reset_top_spin_state_machine();
        referee.discard_projectile_hit_event();
        last_referee_hp = referee.current_HP;
        referee_hp_initialized = TRUE;
        *angle_set = 0.0f;
        return;
    }

    top_angle = update_top_spin_command(HAL_GetTick());

		
    *angle_set = top_angle;
}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         返回空
 */
void Chassis::chassis_free_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set);
}



/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @retval         none
 */
void Chassis::chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //键盘控制
    if (KEY_CHASSIS_FRONT)
    {
        vy_set_channel = y.min_speed;
    }
    else if (KEY_CHASSIS_BACK)
    {
        vy_set_channel = y.max_speed;
    }

    if (KEY_CHASSIS_LEFT)
    {
        vx_set_channel = x.min_speed;
    }
    else if (KEY_CHASSIS_RIGHT)
    {
        vx_set_channel = x.max_speed;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
    chassis_cmd_slow_set_vx.first_order_filter_cali(vx_set_channel);
    chassis_cmd_slow_set_vy.first_order_filter_cali(vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
}

/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
void Chassis::chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4])
{
		
    wheel_speed[0] =  ((- x.speed_set + y.speed_set)*1.414f - MOTOR_DISTANCE_TO_CENTER * z.speed_set)/MOTOR_WHEEL_RADIUS;
    wheel_speed[1] =  ((x.speed_set + y.speed_set)*1.414f - MOTOR_DISTANCE_TO_CENTER * z.speed_set)/MOTOR_WHEEL_RADIUS;
    wheel_speed[2] =  ((x.speed_set - y.speed_set)*1.414f - MOTOR_DISTANCE_TO_CENTER * z.speed_set)/MOTOR_WHEEL_RADIUS;
    wheel_speed[3] =  ((- x.speed_set - y.speed_set)*1.414f - MOTOR_DISTANCE_TO_CENTER * z.speed_set)/MOTOR_WHEEL_RADIUS;
}
