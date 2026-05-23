#include "gimbal.h"
#include "Serialplot.h"
#include "chassis.h"

extern Serialplot serialplot;

//调试参数
float fuck = 0.001f;
float negative_fuck = -0.46105f;
float positive_fuck = 0.4690f;
const fp32 yaw_startup_speed_deadband = 0.05f;
const fp32 yaw_startup_torque_deadband = 0.01f;
const fp32 yaw_bias_fade_error_zero_rad = 0.0002f;
const fp32 yaw_bias_fade_error_hold_rad = 0.001f;
const fp32 yaw_bias_fade_error_knee_rad = 0.01f;
const fp32 yaw_bias_fade_error_full_rad = 0.02f;
const fp32 yaw_bias_fade_min_scale = 0.25f;
const fp32 yaw_bias_fade_knee_scale = 0.80f;
const fp32 yaw_bias_hold_enter_error_rad = 0.001f;
const fp32 yaw_bias_hold_exit_error_rad = 0.003f;
const fp32 yaw_bias_hold_speed_radps = 0.03f;
const fp32 top_yaw_ff_anchor_omega_radps = 10.0f;
const fp32 top_yaw_ff_anchor_torque_nm = 0.65f;
const fp32 top_yaw_ff_omega_gain = 0.0125f;
const fp32 top_yaw_ff_enable_omega_radps = 1.0f;
const fp32 top_yaw_ff_max_torque_nm = 1.0f;
const fp32 top_yaw_ff_filter_tau_s = 0.03f;
const fp32 top_yaw_ff_error_hold_rad = 0.004f;
const fp32 top_yaw_ff_error_backoff_rad = 0.02f;
const fp32 top_yaw_ff_min_scale = 0.25f;
static uint8_t yaw_bias_hold_latched = 0U;
static fp32 top_yaw_ff_filtered_relative_omega = 0.0f;
static uint8_t top_yaw_ff_filter_initialized = 0U;
static const fp32 vision_yaw_release_weights[VISION_YAW_RELEASE_SLOTS] = {0.25f, 0.25f, 0.25f, 0.25f};
static uint8_t vision_takeover_window_cycles = 0U;

//扫敌模式角度积累
static fp32 sweep_360_yaw_accumulated = 0.0f;

static fp32 abs_fp32(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

static fp32 get_yaw_bias_fade_scale(fp32 control_error_rad)
{
    const fp32 error_abs = abs_fp32(control_error_rad);

    if (error_abs <= yaw_bias_fade_error_zero_rad)
    {
        return 0.0f;
    }

    if (error_abs >= yaw_bias_fade_error_full_rad)
    {
        return 1.0f;
    }

    if (error_abs <= yaw_bias_fade_error_hold_rad)
    {
        return yaw_bias_fade_min_scale * (error_abs - yaw_bias_fade_error_zero_rad) / (yaw_bias_fade_error_hold_rad - yaw_bias_fade_error_zero_rad);
    }

    if (error_abs >= yaw_bias_fade_error_knee_rad)
    {
        const fp32 upper_scale = (error_abs - yaw_bias_fade_error_knee_rad) / (yaw_bias_fade_error_full_rad - yaw_bias_fade_error_knee_rad);
        return yaw_bias_fade_knee_scale + (1.0f - yaw_bias_fade_knee_scale) * upper_scale;
    }

    return yaw_bias_fade_min_scale + (yaw_bias_fade_knee_scale - yaw_bias_fade_min_scale) * (error_abs - yaw_bias_fade_error_hold_rad) / (yaw_bias_fade_error_knee_rad - yaw_bias_fade_error_hold_rad);
}

static void reset_top_yaw_ff_filter_state(void)
{
    top_yaw_ff_filtered_relative_omega = 0.0f;
    top_yaw_ff_filter_initialized = 0U;
}

static fp32 get_top_yaw_ff_error_scale(fp32 relative_omega_radps, fp32 gyro_error_rad)
{
    const fp32 error_abs = abs_fp32(gyro_error_rad);

    if (relative_omega_radps * gyro_error_rad >= 0.0f)
    {
        return 1.0f;
    }

    if (error_abs <= top_yaw_ff_error_hold_rad)
    {
        return 1.0f;
    }

    if (error_abs >= top_yaw_ff_error_backoff_rad)
    {
        return top_yaw_ff_min_scale;
    }

    return 1.0f -
           (1.0f - top_yaw_ff_min_scale) *
               (error_abs - top_yaw_ff_error_hold_rad) /
               (top_yaw_ff_error_backoff_rad - top_yaw_ff_error_hold_rad);
}

static fp32 get_top_yaw_ff_torque(fp32 relative_omega_radps, fp32 gyro_error_rad)
{
    fp32 omega_ff = relative_omega_radps;
    fp32 top_command_omega = chassis.z.speed_set;
    fp32 ff_reference_omega = 0.0f;

    if (omega_ff < 0.0f)
    {
        omega_ff = 0.0f;
    }

    if (top_command_omega < 0.0f)
    {
        top_command_omega = 0.0f;
    }

    if (top_yaw_ff_filter_initialized == 0U)
    {
        top_yaw_ff_filtered_relative_omega = omega_ff;
        top_yaw_ff_filter_initialized = 1U;
    }
    else
    {
        const fp32 filter_alpha = GIMBAL_CONTROL_TIME / (top_yaw_ff_filter_tau_s + GIMBAL_CONTROL_TIME);
        top_yaw_ff_filtered_relative_omega +=
            filter_alpha * (omega_ff - top_yaw_ff_filtered_relative_omega);
    }

    ff_reference_omega = top_yaw_ff_filtered_relative_omega;
    if (ff_reference_omega < top_command_omega)
    {
        ff_reference_omega = top_command_omega;
    }

    if (ff_reference_omega < top_yaw_ff_enable_omega_radps)
    {
        return 0.0f;
    }

    fp32 top_yaw_ff_torque = top_yaw_ff_anchor_torque_nm +
                             top_yaw_ff_omega_gain *
                                 (ff_reference_omega - top_yaw_ff_anchor_omega_radps);

    top_yaw_ff_torque = fp32_constrain(top_yaw_ff_torque, 0.0f, top_yaw_ff_max_torque_nm);

    return top_yaw_ff_torque * get_top_yaw_ff_error_scale(relative_omega_radps, gyro_error_rad);
}

static uint8_t get_yaw_bias_hold_disable(fp32 control_error_rad, fp32 actual_speed_radps, uint8_t bias_fade_enable)
{
    if (bias_fade_enable == 0U)
    {
        yaw_bias_hold_latched = 0U;
        return 0U;
    }

    const fp32 error_abs = abs_fp32(control_error_rad);
    const fp32 speed_abs = abs_fp32(actual_speed_radps);

    if (error_abs <= yaw_bias_hold_enter_error_rad && speed_abs <= yaw_bias_hold_speed_radps)
    {
        yaw_bias_hold_latched = 1U;
    }
    else if (error_abs >= yaw_bias_hold_exit_error_rad || speed_abs >= (yaw_bias_hold_speed_radps * 2.0f))
    {
        yaw_bias_hold_latched = 0U;
    }

    return yaw_bias_hold_latched;
}

static fp32 get_yaw_startup_bias_nm(fp32 speed_set, fp32 torque_cmd_nm, fp32 control_error_rad, fp32 actual_speed_radps, uint8_t fade_enable)
{
    fp32 base_bias = 0.0f;

    if (speed_set > yaw_startup_speed_deadband || torque_cmd_nm > yaw_startup_torque_deadband)
    {
        base_bias = positive_fuck;
    }
    else if (speed_set < -yaw_startup_speed_deadband || torque_cmd_nm < -yaw_startup_torque_deadband)
    {
        base_bias = negative_fuck;
    }

    if (fade_enable == 0U)
    {
        return base_bias;
    }

    if (get_yaw_bias_hold_disable(control_error_rad, actual_speed_radps, fade_enable) != 0U)
    {
        return 0.0f;
    }

    return base_bias * get_yaw_bias_fade_scale(control_error_rad);
}
//重力补偿参数


double test_tor=-1.55;
double initial_angle=-0.266;
double deviation_angle=0; 
double final_tor=0;

fp32 final_yaw_tor = 0.0f;


// bool_t auto_switch = false; //自瞄开关放在Communicate中声明了

//云台模块 对象
Gimbal gimbal;

static fp32 constrain_symmetric(fp32 value, fp32 limit)
{
    return fp32_constrain(value, -limit, limit);
}

static fp32 safe_pitch_set_from_feedback(void)
{
    const fp32 pitch_feedback = gimbal.gimbal_pitch_motor.encode_angle;

    if (pitch_feedback >= PITCH_MIN_ANGLE && pitch_feedback <= PITCH_MAX_ANGLE)
    {
        return pitch_feedback;
    }

    return fp32_constrain(gimbal.gimbal_pitch_motor.encode_angle_set, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
}

static uint8_t vision_delta_mode_allowed(void)
{
    // return (auto_switch == true && gimbal.gimbal_mode == GIMBAL_TOP && chassis.chassis_behaviour_mode == CHASSIS_TOP) ? 1U : 0U;
    //为了测试而暂时的改成进入top就自瞄
     return (gimbal.gimbal_mode == GIMBAL_TOP && chassis.chassis_behaviour_mode == CHASSIS_TOP) ? 1U : 0U;
}

static uint8_t top_mode_under_vision_takeover(void)
{
    if (vision_delta_mode_allowed() == 0U)
    {
        return 0U;
    }

    return (vision.rt.state == VISION_TRACK_UNLOCKED || vision.rt.state == VISION_TRACK_LOCKED) ? 1U : 0U;
}

static void vision_delta_clear_yaw_release(void)
{
    memset(&vision.yaw_release_rt, 0, sizeof(vision.yaw_release_rt));
}

static void vision_takeover_clear_window(void)
{
    vision_takeover_window_cycles = 0U;
}

static void vision_delta_release_yaw_step(void)
{
    const fp32 release_delta = vision.yaw_release_rt.slots[vision.yaw_release_rt.index];

    if (release_delta != 0.0f)
    {
        gimbal.gimbal_yaw_motor.gyro_angle_set = rad_format(gimbal.gimbal_yaw_motor.gyro_angle_set + release_delta);
        vision.yaw_release_rt.slots[vision.yaw_release_rt.index] = 0.0f;
    }

    vision.yaw_release_rt.index++;
    if (vision.yaw_release_rt.index >= VISION_YAW_RELEASE_SLOTS)
    {
        vision.yaw_release_rt.index = 0U;
    }
}

static void vision_delta_sync_setpoint_to_feedback(void)
{
    vision_delta_clear_yaw_release();
    vision_takeover_clear_window();
    gimbal.gimbal_yaw_motor.encode_angle_set = gimbal.gimbal_yaw_motor.encode_angle;
    gimbal.gimbal_yaw_motor.gyro_angle_set = gimbal.gimbal_yaw_motor.gyro_angle;
    gimbal.gimbal_yaw_motor.speed_set = 0.0f;
    gimbal.gimbal_pitch_motor.encode_angle_set = gimbal.gimbal_pitch_motor.encode_angle;
    gimbal.gimbal_pitch_motor.speed_set = 0.0f;

    gimbal.gimbal_yaw_motor.speed_pid.align_state_to_current();
    gimbal.gimbal_yaw_motor.encode_angle_pid.align_state_to_current();
    gimbal.gimbal_yaw_motor.gyro_angle_pid.align_state_to_current();
    gimbal.gimbal_pitch_motor.speed_pid.align_state_to_current();
    gimbal.gimbal_pitch_motor.encode_angle_pid.align_state_to_current();
}

static void vision_delta_enter_acquire(void)
{
    vision_delta_sync_setpoint_to_feedback();
    vision.clear_lock_qualification();
    vision.rt.fresh = 0U;
    vision.rt.consumed = 1U;
    vision.rt.timeout = 0U;
    vision.rt.target_lost_count = 0U;
    vision.rt.state = VISION_ACQUIRE;
}

static void vision_delta_enter_lost_hold(void)
{
    vision_delta_sync_setpoint_to_feedback();
    vision.clear_lock_qualification();
    vision.rt.fresh = 0U;
    vision.rt.consumed = 1U;
    vision.rt.timeout = 1U;
    vision.rt.target_lost_count = 0U;
    vision.rt.state = VISION_LOST_HOLD;
}

static void vision_delta_disable(void)
{
    vision.reset_runtime_state(0U);
}

static void vision_delta_update_lock_qualification(void)
{
    if (vision.vision_recv_data.centre_lock != 0U)
    {
        if (vision.rt.lock_enter_count < vision.delta_param.lock_enter_threshold)
        {
            vision.rt.lock_enter_count++;
        }
        vision.rt.lock_exit_count = 0U;

        if (vision.rt.lock_enter_count >= vision.delta_param.lock_enter_threshold)
        {
            vision.rt.state = VISION_TRACK_LOCKED;
            vision.rt.fire_qualified = 1U;
        }
        else
        {
            vision.rt.state = VISION_TRACK_UNLOCKED;
            vision.rt.fire_qualified = 0U;
        }
    }
    else
    {
        vision.rt.lock_enter_count = 0U;
        if (vision.rt.lock_exit_count < vision.delta_param.lock_exit_threshold)
        {
            vision.rt.lock_exit_count++;
        }
        vision.rt.fire_qualified = 0U;
        if (vision.rt.lock_exit_count >= vision.delta_param.lock_exit_threshold)
        {
            vision.rt.state = VISION_TRACK_UNLOCKED;
        }
    }
}

//云台视觉增量处理函数
static void vision_delta_process_frame(void)
{   
    if(vision.vision_recv_data.CmdID == 0x00){
        vision.rt.invalid_frame = 1U;
        return;
    }
    else{
        vision.rt.invalid_frame = 0U;
        if (vision.rt.fresh == 0U || vision.rt.consumed != 0U)
        {
         vision_delta_release_yaw_step();
        return;
        }

        if (vision.vision_recv_data.identify_target == 0U)
        {
            if (vision.rt.state == VISION_ACQUIRE)
            {
                vision.clear_lock_qualification();
                vision.rt.fresh = 0U;
                vision.rt.consumed = 1U;
                return;
            }

            if (vision.rt.target_lost_count < vision.delta_param.target_lost_threshold)
            {
                vision.rt.target_lost_count++;
            }

                vision.clear_lock_qualification();

            if (vision.rt.target_lost_count >= vision.delta_param.target_lost_threshold)
            {
            vision_delta_enter_lost_hold();
            }

            vision.rt.fresh = 0U;
            vision.rt.consumed = 1U;
            return;
    }

    vision_delta_release_yaw_step();

    vision.rt.target_lost_count = 0U;
    vision.rt.timeout = 0U;
    if (vision.rt.state == VISION_ACQUIRE)
    {
        vision.rt.state = VISION_TRACK_UNLOCKED;
        vision_takeover_window_cycles = VISION_YAW_RELEASE_SLOTS;
    }

    const fp32 frame_yaw_delta = constrain_symmetric(vision.vision_recv_data.yaw_angle * vision.delta_param.k_yaw,
                                                     vision.delta_param.yaw_step_max);
    uint8_t slot = vision.yaw_release_rt.index;

    for (uint8_t i = 0U; i < VISION_YAW_RELEASE_SLOTS; i++)
    {
        vision.yaw_release_rt.slots[slot] += frame_yaw_delta * vision_yaw_release_weights[i];
        slot++;
        if (slot >= VISION_YAW_RELEASE_SLOTS)
        {
            slot = 0U;
        }
    }

    gimbal.gimbal_pitch_motor.encode_angle_set = fp32_constrain(
        gimbal.gimbal_pitch_motor.encode_angle_set +
        constrain_symmetric(vision.vision_recv_data.pitch_angle * vision.delta_param.k_pitch,
                            vision.delta_param.pitch_step_max),
        PITCH_MIN_ANGLE,
        PITCH_MAX_ANGLE);

    vision_delta_update_lock_qualification();

    vision.rt.fresh = 0U;
    vision.rt.consumed = 1U;
}
}

//云台控制系统的状态更新函数
static void vision_delta_update_runtime(void)
{   
    const uint32_t now = HAL_GetTick();
    
    if (vision_delta_mode_allowed() == 0U)
    {
        vision_delta_disable();
        return;
    }

    if (vision.rt.state == VISION_DISABLED)
    {
        vision_delta_enter_acquire();
        return;
    }

    if (vision.rt.last_rx_tick != 0U &&
        (now - vision.rt.last_rx_tick) > vision.delta_param.vision_timeout_ms)
    {
        if (vision.rt.state == VISION_TRACK_UNLOCKED || vision.rt.state == VISION_TRACK_LOCKED)
        {
            vision_delta_enter_lost_hold();
            return;
        }

        vision.rt.timeout = 1U;
        vision.clear_lock_qualification();
        vision.rt.fresh = 0U;
        vision.rt.consumed = 1U;
        return;
    }

    vision.rt.timeout = 0U;

    if (vision.rt.state == VISION_LOST_HOLD)
    {
        vision_delta_enter_acquire();
        return;
    }

    vision_delta_process_frame();
}

// Yaw0 Pitch1
//弧度制需要控制精度在0.01rad以内，角度制需要控制精度在0.5度以内
//因为编码器控制和陀螺仪控制的时候反馈对象不一样，一个是电机内部的编码器，
//一个是带载的云台角度，所以二者在调整pid的时候，陀螺仪模式受到的由于摩擦力、负载受到的扰动更大
//所以在两种模式下都请保证编码器角度的误差在0.01rad以内（即目标值和当前值在上位机上的偏差），
//陀螺仪角度的误差在0.5度以内，这样才能保证接入自瞄模块后能够较好地进行自瞄控制

float Gimbal_SPEED_PID[2][6] = {{715.0f,0.0f,0.0f,0.0f,3.0f,4000.0f},{8000.0f,0.0f,0.0f,0.0f,10.0f,20000.0f}};

float Gimbal_ENCODE_PID[2][6] = {{6.0f,0.0f,2.0f,0.0f,1.0f,50.0f},{30.0f,0.01f,3.0f,0.0f,1.0f,100.0f}};

float Gimbal_GYRO_PID[6] ={0.02f,0.001f,0.0f,0.0f,3.0f,100.0f};

//原有speed PID参数 {1350.0f,0.0f,110.0f,0.0f,1.0f,3500.0f}
//原有encode PID参数 {10.0f,0.0f,25.0f,0.0f,1.0f,50.0f}
//原有gyro PID参数 {6.0f,0.0f,15.0f,0.0f,1.0f,50.0f}
//   分别是          P I D F IOUT MAX_IOUT MAX_OUT


//ENCODE P 8 I 0 D 15 IOUT 0 MAXOUT 16
/**
 * @brief          初始化云台
 * @Author         WSJ
 */
void Gimbal::init()
{
    /*---------------------------遥控器---------------------------------*/
    //遥控器数据指针获取
    gimbal_RC = vt13.get_vt13_remote_control_point();
    last_gimbal_RC = vt13.get_last_vt13_remote_control_point();

    /*----------------------------电机-----------------------------------*/
    gimbal_yaw_motor=DM_Motor(CAN_YAW_MOTOR_ID,MASTER_YAW_MOTOR_ID, can_receive.get_dm_motor_measure_point(0));
    gimbal_yaw_motor.dm_mit_init();
    gimbal_yaw_motor.speed_pid.init(PID_SPEED, Gimbal_SPEED_PID[0], &gimbal_yaw_motor.speed, &gimbal_yaw_motor.speed_set, NULL);
    gimbal_yaw_motor.encode_angle_pid.init(PID_ANGLE, Gimbal_ENCODE_PID[0], &gimbal_yaw_motor.encode_angle, &gimbal_yaw_motor.encode_angle_set, NULL);
    gimbal_yaw_motor.gyro_angle_pid.init(PID_ANGLE, Gimbal_GYRO_PID, &gimbal_yaw_motor.gyro_angle, &gimbal_yaw_motor.gyro_angle_set, NULL);

    gimbal_pitch_motor=DM_Motor(CAN_PITCH_MOTOR_ID,MASTER_PITCH_MOTOR_ID,can_receive.get_dm_motor_measure_point(1));
    gimbal_pitch_motor.dm_mit_init();
    gimbal_pitch_motor.speed_pid.init(PID_SPEED, Gimbal_SPEED_PID[1], &gimbal_pitch_motor.speed, &gimbal_pitch_motor.speed_set,NULL);
    gimbal_pitch_motor.encode_angle_pid.init(PID_ANGLE, Gimbal_ENCODE_PID[1], &gimbal_pitch_motor.encode_angle, &gimbal_pitch_motor.encode_angle_set, NULL);

    /*------------------------云台状态机初始化----------------------------*/
    //初始化初始状态为无力模式
    gimbal_mode = GIMBAL_ZERO_FORCE;
    last_gimbal_mode = gimbal_mode;

    /*--------------------------滤波值初始化-----------------------------*/
    // const static fp32 gimbal_yaw_high_pass_filter_para[1] = {GIMBAL_ACCEL_YAW_NUM};
    // const static fp32 gimbal_pitch_high_pass_filter_para[1] = {GIMBAL_ACCEL_PITCH_NUM};
    //一阶高通滤波初始化
    //gimbal_yaw_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_yaw_high_pass_filter_para);
    //gimbal_pitch_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_pitch_high_pass_filter_para);

    feedback_update();
}

void Gimbal::feedback_update()
{
    //按键数据更新
    //key_state_update();
    gimbal_data_update();
    //模式切换数据保存
    mode_change_save();
    last_gimbal_mode = gimbal_mode;
}

void Gimbal::mode_change_save()
{
    //切入底盘跟随云台模式
    if (last_gimbal_mode != GIMBAL_TOP && gimbal_mode == GIMBAL_TOP)
    {   
        gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle;
        gimbal_yaw_motor.gyro_angle_set = gimbal_yaw_motor.gyro_angle;
        gimbal_pitch_motor.encode_angle_set = safe_pitch_set_from_feedback();
    }
    //切入自由控制模式
    else if (last_gimbal_mode != GIMBAL_FREE && gimbal_mode == GIMBAL_FREE)
    {   
        
        gimbal_yaw_motor.speed_pid.pid_clear();        
        gimbal_yaw_motor.encode_angle_pid.pid_clear(); 
        gimbal_yaw_motor.gyro_angle_pid.pid_clear();   // 模式切换安全清零

#if YAW_SYSID_ENABLE
        gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle;
#else
        //这里不用归中！！！！！
        gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle-0.2f;//归中稍不准 有机会拆云台重新设置零点
#endif
        gimbal_pitch_motor.encode_angle_set = safe_pitch_set_from_feedback();
    }

}

/**
 * @brief          云台数据计算更新
 * @Author         WSJ
 */
void Gimbal::gimbal_data_update()
{
    
    gimbal_pitch_motor.update();
    gimbal_yaw_motor.update();
    gimbal_yaw_motor.gyro_angle = angle_to_rad(imu.get_imu_output_info_point()->yaw);
    gimbal_yaw_motor.encode_angle = uint_to_float(gimbal_yaw_motor.motor_measure->p_int, -gimbal_yaw_motor.tmp.pmax, gimbal_yaw_motor.tmp.pmax, 16);//gimbal_pitch_motor.motor_measure->p_int
    gimbal_yaw_motor.speed = uint_to_float(gimbal_yaw_motor.motor_measure->v_int, -gimbal_yaw_motor.tmp.vmax, gimbal_yaw_motor.tmp.vmax, 12);
    // 视觉发送的pitch在Vision::send()中使用IMU，这里保持电机反馈独立。
}

void Gimbal::set_mode()
{
    //记录上一次的模式
    last_gimbal_mode = gimbal_mode;

#if YAW_SYSID_ENABLE
    gimbal_mode = GIMBAL_FREE;
    serialplot.update_yaw_sysid_state(gimbal_mode);
#else
    //拨杆改变模式(在这里修改拨杆值对应的模式)
    switch_control();
#endif

}


/**
 * @brief 云台拨杆控制模式切换函数
 * 
 * 根据遥控器模式拨杆的位置切换云台的工作模式：
 * - 拨杆向上：切换到GIMBAL_TOP模式（跟随底盘模式）
 * - 拨杆中位：切换到GIMBAL_FREE模式（自由模式）
 * - 拨杆向下：切换到GIMBAL_ZERO_FORCE模式（无力模式）
 * 
 * 该函数通过读取遥控器mode_sw开关的状态，设置相应的云台工作模式。
 */
void Gimbal::switch_control()
{
    if (switch_is_up(gimbal_RC->rc.mode_sw))
    {
        gimbal_mode = GIMBAL_TOP;
    }
    else if (switch_is_mid(gimbal_RC->rc.mode_sw))
    {
        gimbal_mode = GIMBAL_FREE;
    }
    else if (switch_is_down(gimbal_RC->rc.mode_sw))
    {
        gimbal_mode = GIMBAL_ZERO_FORCE;
    }
}


void Gimbal::set_control()
{
    
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

#if YAW_SYSID_ENABLE
    if (serialplot.is_yaw_sysid_mode_active() != 0U)
    {
        auto_switch=false;
        return;
    }
    else
#endif
    if (gimbal_mode == GIMBAL_TOP)
    {
       gimbal_top_control(&add_yaw_angle, &add_pitch_angle);
      
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {   
        auto_switch=false;//关闭自瞄
        gimbal_free_control(&add_yaw_angle, &add_pitch_angle);//遥控器决定云台怎么动
        
    }
}




#define rc_deadband_limit(input, output, dealine) {      \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                 \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



void Gimbal::gimbal_top_control(fp32 *yaw, fp32 *pitch)
{
    //sweep_360_yaw_accumulated = 0.0f;
    if(is_sweeping_360){
        fp32 sweeping_speed = 30.0f;
        fp32 sweep_step = sweeping_speed * GIMBAL_CONTROL_TIME;

        if (sweep_360_yaw_accumulated < 2.0f * PI)
        {
            *yaw += sweep_step;
            sweep_360_yaw_accumulated += sweep_step;
        }
        else
        {
            is_sweeping_360 = false;
        }
    }
    gimbal_rc_to_control_angle(yaw, pitch);

    gimbal_yaw_motor.gyro_angle_set = rad_format(gimbal_yaw_motor.gyro_angle_set + *yaw);

    gimbal_pitch_motor.set(fp32_constrain(gimbal_pitch_motor.encode_angle_set+*pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE), ENCODE_ANGLE);
    
    gimbal_yaw_motor.set(gimbal_yaw_motor.gyro_angle_set , GYRO_ANGLE);
 
}

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_free_control(fp32 *yaw, fp32 *pitch)
{
    gimbal_rc_to_control_angle(yaw, pitch);

    gimbal_pitch_motor.set(fp32_constrain(gimbal_pitch_motor.encode_angle_set+*pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE), ENCODE_ANGLE);

    gimbal_yaw_motor.set(rad_format(gimbal_yaw_motor.encode_angle_set+*yaw) , ENCODE_ANGLE);

}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
// void Gimbal::gimbal_motionless_control(fp32 *yaw, fp32 *pitch)
// {
//     if (yaw == NULL || pitch == NULL)
//     {
//         return;
//     }
//     *yaw = 0.0f;
//     *pitch = 0.0f;
// }


void Gimbal::solve()
{
		
	
#if YAW_SYSID_ENABLE
    if (serialplot.is_yaw_sysid_mode_active() != 0U)
    {
        gimbal_yaw_motor.speed_set = 0.0f;
        gimbal_yaw_motor.current_give = serialplot.get_yaw_sysid_torque_raw_eq();
        gimbal_pitch_motor.solve(ENCODE_ANGLE);
    }
    else
#endif
    if (gimbal_mode == GIMBAL_TOP)
    {
        fp32 gyro_error = rad_format(gimbal_yaw_motor.gyro_angle_set - gimbal_yaw_motor.gyro_angle);
        fp32 active_gyro_error_map_ratio = TOP_MANUAL_GYRO_ERROR_MAP_RATIO;

        if (top_mode_under_vision_takeover() != 0U)
        {
            active_gyro_error_map_ratio = TOP_GYRO_ERROR_MAP_RATIO;
        }

        if (vision_takeover_window_cycles > 0U)
        {
            if (top_mode_under_vision_takeover() != 0U)
            {
                active_gyro_error_map_ratio = TOP_TAKEOVER_GYRO_ERROR_MAP_RATIO;
            }
            else
            {
                active_gyro_error_map_ratio = TOP_MANUAL_TAKEOVER_GYRO_ERROR_MAP_RATIO;
            }
            vision_takeover_window_cycles--;
        }
        
        gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle +
                                            active_gyro_error_map_ratio * gyro_error +
                                            gimbal_yaw_motor.gyro_angle_pid.pid_calc();
        
        gimbal_yaw_motor.speed_set = gimbal_yaw_motor.encode_angle_pid.pid_calc();
        gimbal_yaw_motor.speed_set = fp32_constrain(gimbal_yaw_motor.speed_set, -gimbal_yaw_motor.tmp.vmax, gimbal_yaw_motor.tmp.vmax);
        
        gimbal_yaw_motor.current_give = gimbal_yaw_motor.speed_pid.pid_calc();

         gimbal_pitch_motor.solve(ENCODE_ANGLE);
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {
         vision_takeover_clear_window();
         gimbal_yaw_motor.solve(ENCODE_ANGLE);


        gimbal_pitch_motor.solve(ENCODE_ANGLE);
    }
    else
    {
        vision_takeover_clear_window();
    }

}


void Gimbal::output()
{   
    const uint8_t yaw_motor_ready = (gimbal_yaw_motor.motor_measure != NULL && gimbal_yaw_motor.motor_measure->state != 0) ? 1U : 0U;
    const uint8_t pitch_motor_ready = (gimbal_pitch_motor.motor_measure != NULL && gimbal_pitch_motor.motor_measure->state != 0) ? 1U : 0U;
    fp32 yaw_control_error = 0.0f;
    fp32 yaw_output_torque = 0.0f;
    uint8_t yaw_bias_fade_enable = 1U;

    deviation_angle = gimbal.gimbal_pitch_motor.encode_angle_set - initial_angle;
    final_tor = test_tor * cos(deviation_angle);
     gimbal_yaw_motor.current_give = fuck*gimbal_yaw_motor.current_give;

    if (gimbal_mode == GIMBAL_TOP)
    {
        yaw_control_error = rad_format(gimbal_yaw_motor.gyro_angle_set - gimbal_yaw_motor.gyro_angle);
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {
        yaw_control_error = rad_format(gimbal_yaw_motor.encode_angle_set - gimbal_yaw_motor.encode_angle);
    }

    if (gimbal_mode == GIMBAL_FREE)
    {
        gimbal_yaw_motor.current_give += get_yaw_startup_bias_nm(gimbal_yaw_motor.speed_set, gimbal_yaw_motor.current_give, yaw_control_error, gimbal_yaw_motor.speed, yaw_bias_fade_enable);
    }

    final_yaw_tor = 0.0f;
    if (gimbal_mode == GIMBAL_TOP && chassis.chassis_behaviour_mode == CHASSIS_TOP)
    {
#if TOP_YAW_FF_TEST_ENABLE
        final_yaw_tor = get_top_yaw_ff_torque(top_tune_debug.relative_omega, yaw_control_error);
#else
        reset_top_yaw_ff_filter_state();
#endif
    }
    else
    {
        reset_top_yaw_ff_filter_state();
    }

    yaw_output_torque = gimbal_yaw_motor.current_give + final_yaw_tor;
	//这里×0.001只是用来做缩放比例的，现在正常使用dm6006电机作为yaw轴，而yaw轴电机目前是纯力矩控制的电机
    //所以如果在电机调参震动的时候，上位机发现current_give绝对值达到或超过4，就是达到额定扭矩了
    //力矩和电流的转化关系在mit模式里，达妙电机中是这样：
    //i=tot/kt_out,kt_out=kt*减速比,kt=1.5*极对数*磁链，这些参数dm上位机可以看
    //本人测试大约，4310电机：kt=1.5*14*0.00439719,减速比为10
    //6006电机kt=1.5*14*0.006071686,减速比为6（每个电机实际都不一样，相同的种类相近而已）
    if (gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_yaw_motor.current_give = 0;
        gimbal_pitch_motor.current_give = 0;
        final_yaw_tor = 0.0f;
        yaw_output_torque = 0.0f;
        reset_top_yaw_ff_filter_state();
        can_receive.DISABLE_DM_Motor_mode(gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode);
        can_receive.DISABLE_DM_Motor_mode(gimbal_pitch_motor.id,gimbal_yaw_motor.ctrl.mode);
    }
    else
    {
        if (pitch_motor_ready == 0U || yaw_motor_ready == 0U)
        {
            can_receive.ENABLE_DM_Motor_mode(gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode);
            can_receive.ENABLE_DM_Motor_mode(gimbal_pitch_motor.id,gimbal_yaw_motor.ctrl.mode);

#if YAW_SYSID_ENABLE
            serialplot.send_yaw_sysid_frame();
#endif
            return;
        }
    }


    //mit格式:pos,kp,kd,vel,tor
    //pitch轴
    //can_receive.mit_ctrl(0,0,0,0,gimbal_pitch_motor.current_give,gimbal_pitch_motor.id,gimbal_pitch_motor.ctrl.mode,gimbal_pitch_motor.tmp);
    // can_receive.mit_ctrl(gimbal_pitch_motor.encode_angle_set,65,1.4,0,final_tor,gimbal_pitch_motor.id,gimbal_pitch_motor.ctrl.mode,gimbal_pitch_motor.tmp);
     can_receive.mit_ctrl(gimbal_pitch_motor.encode_angle_set,24.0,1.15,0,final_tor,gimbal_pitch_motor.id,gimbal_pitch_motor.ctrl.mode,gimbal_pitch_motor.tmp);
    can_receive.mit_ctrl(0,0,0,0,yaw_output_torque,gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode,gimbal_yaw_motor.tmp);
    
   

#if YAW_SYSID_ENABLE
    serialplot.send_yaw_sysid_frame();
#endif
    // can_receive.mit_ctrl(0,0,0,0,gimbal_yaw_motor.current_give,gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode,gimbal_yaw_motor.tmp);
    //can_receive.mit_ctrl(gimbal_yaw_motor.encode_angle_set,30,1.2,0,0,gimbal_yaw_motor.id,gimbal_yaw_motor.ctrl.mode,gimbal_yaw_motor.tmp);
    //yaw轴
} 


/**
 * @brief 将遥控器输入转换为云台控制角度
 * 
 * 该函数根据当前模式（自瞄模式或手动模式）处理遥控器和鼠标输入，
 * 计算出云台的偏航角(yaw)和俯仰角(pitch)控制值。
 * 在自瞄模式下且识别到目标时，云台控制权交给mini PC；
 * 否则使用遥控器和鼠标输入进行手动控制。
 * 
 * @param[in,out] yaw 指向偏航角控制值的指针，函数会将计算结果存入该地址
 * @param[in,out] pitch 指向俯仰角控制值的指针，函数会将计算结果存入该地址
 */
void Gimbal::gimbal_rc_to_control_angle(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

#if VISION_CTRL_MODE == VISION_CTRL_MODE_DELTA
    vision_delta_update_runtime();

   if ((vision.rt.state == VISION_TRACK_UNLOCKED || vision.rt.state == VISION_TRACK_LOCKED) &&
       vision_delta_mode_allowed() != 0U)
   {
       *pitch = 0;
       *yaw = 0;
       BSP_Buzzer_Off();
   }
   else
    {
         fp32 yaw_channel = 0, pitch_channel = 0;
         //死区限制
         rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
         rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

         *yaw = yaw_channel * YAW_RC_SEN + gimbal_RC->mouse.x * YAW_MOUSE_SEN;
         *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;
    
         BSP_Buzzer_On();
     }
#else
   if ( vision.vision_if_find_target() == TRUE && auto_switch==true)
   {
       vision.vision_get_angle(&gimbal_yaw_motor.gyro_angle_set, &gimbal_pitch_motor.encode_angle_set);
       *pitch = 0;
       *yaw = 0;
       BSP_Buzzer_Off();
   }
   else
    {
         fp32 yaw_channel = 0, pitch_channel = 0;
         rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
         rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

         *yaw = yaw_channel * YAW_RC_SEN + gimbal_RC->mouse.x * YAW_MOUSE_SEN;
         *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;
    
         BSP_Buzzer_On();
    }
#endif
}

int Gimbal::float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits)
{
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return (int) ((x_float-offset)*((fp32)((1<<bits)-1))/span);
}


fp32 Gimbal::uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

fp32 Gimbal::angle_to_rad(fp32 angle)
{
    return angle * PI / 180.0f;
}
