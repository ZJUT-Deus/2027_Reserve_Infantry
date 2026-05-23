#include "Shoot.h"
#include "Referee.h"

#include "main.h"
#include "simple_ref_test.h"

// #include "bsp_fric.h"
#include "user_lib.h"

// #ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
// extern "C"
// {
// #include "bsp_laser.h"
// }
// #endif

// #include "detect_task.h"
// #include "bsp_buzzer.h"

// #define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
// #define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
// #define shoot_fric_off() fric_off()         //关闭两个摩擦轮

// #define shoot_laser_on() laser_on()   //激光开启宏定义
// #define shoot_laser_off() laser_off() //激光关闭宏定义
// 微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

#define POWER_LIMIT 80.0f
#define WARNING_POWER 40.0f
#define WARNING_POWER_BUFF 50.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f
/*
shoot射速上限 15 18 30 m/s
shoot热量上限 50 100 150 280 400
shoot热量冷却 10 20 30 40 60 80
一发shoot 10热量

shoot射速上限 10 16 m/s
shoot热量上限 100 200 300 350 500
shoot热量冷却 20 40 60 80 100 120
一发shoot 100热量
*/

// 通过读取裁判数据,直接修改射速和射频等级
// 射速等级  摩擦电机
fp32 shoot_fric_speed = 100.0;

// 射频等级 拨弹电机
fp32 shoot_trigger_speed = 60.0;

int fricflag = 1;
int is_shoot = 1;
int press_l = 0;
bool shoot_unlocked = false;  // 射击解锁标志
bool stop_pressed = false;    // STOP键是否正在被按下
uint16_t stop_press_time = 0; // STOP键按下计时
Shoot shoot;

static int16_t abs_int16(int16_t x)
{
    return (x < 0) ? -x : x;
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void Shoot::init()
{
    shoot_rc = vt13.get_vt13_remote_control_point();
    last_shoot_rc = vt13.get_last_vt13_remote_control_point();
    // 设置初试模式
    shoot_mode = SHOOT_STOP;

    // 摩擦轮电机
    // 获取电机数据
    fric_motor_left = DJI_Motor(CAN_LEFT_FRIC_MOTOR_ID, can_receive.get_dji_motor_measure_point(LEFT_FRIC), 0, 8191);
    // 初始化PID
    fp32 fric_left_speed_pid_parm[6] = {FRIC_left_SPEED_PID_KP, FRIC_left_SPEED_PID_KI, FRIC_left_SPEED_PID_KD, FRIC_left_SPEED_PID_KF, FRIC_left_PID_MAX_IOUT, FRIC_left_PID_MAX_OUT};
    fric_motor_left.speed_pid.init(PID_SPEED, fric_left_speed_pid_parm, &fric_motor_left.speed, &fric_motor_left.speed_set, NULL);
    fric_motor_left.speed_pid.pid_clear();

    // 设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    //  fric_motor_left.max_speed = FRIC_MAX_SPEED;
    //  fric_motor_left.min_speed = -FRIC_MAX_SPEED;
    //  fric_motor_left.require_speed = -FRIC_MAX_REQUIRE_SPEED;

    // 获取电机数据
    fric_motor_right = DJI_Motor(CAN_RIGHT_FRIC_MOTOR_ID, can_receive.get_dji_motor_measure_point(RIGHT_FRIC), 0, 8191);
    // 初始化PID
    fp32 fric_right_speed_pid_parm[6] = {FRIC_right_SPEED_PID_KP, FRIC_right_SPEED_PID_KI, FRIC_right_SPEED_PID_KD, FRIC_right_SPEED_PID_KF, FRIC_right_PID_MAX_IOUT, FRIC_right_PID_MAX_OUT};
    fric_motor_right.speed_pid.init(PID_SPEED, fric_right_speed_pid_parm, &fric_motor_right.speed, &fric_motor_right.speed_set, NULL);
    fric_motor_right.speed_pid.pid_clear();

    // 设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    //  fric_motor_right.max_speed = FRIC_MAX_SPEED;
    //  fric_motor_right.min_speed = -FRIC_MAX_SPEED;
    //  fric_motor_right.require_speed = -FRIC_MAX_REQUIRE_SPEED;
    // 拨弹电机
    trigger_motor = DJI_Motor(CAN_TRIGGER_MOTOR_ID, can_receive.get_dji_motor_measure_point(TRIGGER), 0, 8191);
    fp32 trigger_speed_pid_parm[6] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_ANGLE_PID_KF, TRIGGER_READY_PID_MAX_IOUT, TRIGGER_READY_PID_MAX_OUT};
    trigger_motor.speed_pid.init(PID_SPEED, trigger_speed_pid_parm, &trigger_motor.speed, &trigger_motor.speed_set, NULL);
    trigger_motor.speed_pid.pid_clear();
    // trigger_motor.init(can_receive.get_shoot_motor_measure_point(TRIGGER));
    // //初始化PID
    // fp32 trigger_speed_pid_parm[5] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_READY_PID_MAX_IOUT, TRIGGER_READY_PID_MAX_OUT};
    // trigger_motor.speed_pid.init(PID_SPEED, trigger_speed_pid_parm, &trigger_motor.speed, &trigger_motor.speed_set, NULL);
    // trigger_motor.angle_pid.pid_clear();
    // //TODO,此处限幅,暂时不设置
    // //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    // trigger_motor.max_speed = FRIC_MAX_SPEED_RMP;
    // trigger_motor.min_speed = -FRIC_MAX_SPEED_RMP;
    // trigger_motor.require_speed = -FRIC_REQUIRE_SPEED_RMP;

    // 摩擦轮 限位舵机状态
    fric_status = FALSE;
    limit_switch_status = FALSE;

    // TODO 此处先添加,后面可能会删去
    // trigger_motor.angle = trigger_motor.motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    // trigger_motor.ecd_count = 0;
    // trigger_motor.current_give = 0;
    // trigger_motor.angle_set = trigger_motor.angle;
    // trigger_motor.speed = 0.0f;
    // trigger_motor.speed_set = 0.0f;

    const static fp32 chassis_x_order_filter[1] = {SHOOT_ACCEL_FRIC_LEFT_NUM};
    const static fp32 chassis_y_order_filter[1] = {SHOOT_ACCEL_FRIC_RIGHT_NUM};

    shoot_cmd_slow_fric_left.init(SHOOT_CONTROL_DT_S, chassis_x_order_filter);
    shoot_cmd_slow_fric_right.init(SHOOT_CONTROL_DT_S, chassis_y_order_filter);

    // 更新数据
    feedback_update();

    move_flag = 0;
    cover_move_flag = 0;
    key_time = 0;
    local_heat = 0.0f;
    heat_limit = SHOOT_HEAT_LIMIT;
    heat_guard = SHOOT_HEAT_GUARD;
    heat_allow_fire = TRUE;
    heat_block_latched = FALSE;
    heat_fire_window = FALSE;
    reset_fric_shot_detector();
    reset_trigger_anti_jam();

    // 未防止卡弹, 上电后自动开启摩擦轮,可以手动关闭
    shoot_mode = SHOOT_READY_FRIC;  
    // buzzer_on(5, 10000);
}

/**
 * @brief          射击状态机设置，遥控器下不射击，中间为摩擦轮开启状态，长按3s暂停键后按住扳机为射击状态
 * @param[in]      void
 * @retval         void
 */

void Shoot::set_mode()
{
    // static int8_t last_s= RC_SW_UP; 记录上一次遥控器按键值
    uint8_t prev_manual_fire = 0U;
    if (shoot_mode == SHOOT_CONTINUE_BULLET && switch_is_mid(shoot_rc->rc.mode_sw))
    {
        prev_manual_fire = 1U;
    }

    if (switch_is_down(shoot_rc->rc.mode_sw))
    {
        shoot_mode = SHOOT_STOP;
    }
    else if (switch_is_mid(shoot_rc->rc.mode_sw))
    {
        if ((shoot_unlocked && shoot_rc->rc.shutter) || IF_MOUSE_PRESSED_LEFT_VT13)
        {
            shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else
        {
            shoot_mode = SHOOT_READY;
        }
    }
    else if(switch_is_up(shoot_rc->rc.mode_sw)) {
#if VISION_CTRL_MODE == VISION_CTRL_MODE_DELTA
        if ((vision.rt.fire_qualified != 0U) || IF_MOUSE_PRESSED_LEFT_VT13){
#else
        if ((vision.vision_recv_data.centre_lock) || IF_MOUSE_PRESSED_LEFT_VT13){
#endif
            
            shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else
        {
            shoot_mode = SHOOT_READY;
        }
    }

    if (prev_manual_fire == 0U &&
        switch_is_mid(shoot_rc->rc.mode_sw) &&
        shoot_unlocked &&
        shoot_rc->rc.shutter &&
        shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        simple_ref_test_on_start();
    }
}    


/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */

 void Shoot::feedback_update()
 {
        shoot_last_key_v = shoot_rc->key.v;

        // 更新摩擦轮电机速度
        fric_motor_left.update();
        fric_motor_right.update();
        trigger_motor.update();

        // 拨弹轮电机速度滤波一下
        static fp32 trigger_speed_fliter_1 = 0.0f;
        static fp32 trigger_speed_fliter_2 = 0.0f;
        static fp32 trigger_speed_fliter_3 = 0.0f;

        static const fp32 trigger_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
        // 二阶低通滤波
        trigger_speed_fliter_1 = trigger_speed_fliter_2;
        trigger_speed_fliter_2 = trigger_speed_fliter_3;
        trigger_speed_fliter_3 = trigger_speed_fliter_2 * trigger_fliter_num[0] + trigger_speed_fliter_1 * trigger_fliter_num[1] + (trigger_motor.motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * trigger_fliter_num[2];
        trigger_motor.speed = trigger_speed_fliter_3;

        // 鼠标按键
        last_press_l = press_l;
        last_press_r = press_r;
        press_l = shoot_rc->mouse.press_l;
        press_r = shoot_rc->mouse.press_r;

        // STOP键事件检测
        if (stop_on(shoot_rc->rc.stop))
        {
            if (!stop_pressed)
            {
                // 刚开始按下
                stop_pressed = true;
                stop_press_time = 0;
            }
            else
            {
                // 持续按下，计时
                if (stop_press_time < RC_S_LONG_TIME)
                {
                    stop_press_time++;
                }
            }
        }
        else
        // stop键松开事件检测
        {
            if (stop_pressed)
            {
                // 刚松开
                if (stop_press_time >= RC_S_LONG_TIME)
                {
                    // 长按：解锁
                    shoot_unlocked = true;
                }
                else
                {
                    // 短按：上锁
                    shoot_unlocked = false;
                }
                stop_pressed = false;
            }
        }
 
 }

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
void Shoot::set_control()
{
        // 对摩擦轮电机输入控制值
        fric_motor_left.speed_set = shoot_fric_speed*FRIC_STD_SPEED_RATIO;
        fric_motor_right.speed_set = -shoot_fric_speed*FRIC_STD_SPEED_RATIO;
        if (shoot_mode == SHOOT_STOP)
        {
            reset_trigger_anti_jam();
            fric_motor_left.speed_set = 0;

            fric_motor_right.speed_set = 0;

            trigger_forward_speed_set = 0.0f;
            trigger_motor.speed_set = 0.0f;
            cooling_ctrl();
        }
        else if (shoot_mode == SHOOT_READY)
        {
            reset_trigger_anti_jam();
            trigger_forward_speed_set = 0.0f;
            trigger_motor.speed_set = 0.0f;
            cooling_ctrl();
        }
        else if (shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            trigger_motor.speed_set = shoot_trigger_speed;
            trigger_forward_speed_set = trigger_motor.speed_set;
            cooling_ctrl();
            trigger_forward_speed_set = trigger_motor.speed_set;
            trigger_motor_turn_back();
        }
        else
        {
            reset_trigger_anti_jam();
            trigger_forward_speed_set = 0.0f;
            trigger_motor.speed_set = 0.0f;
            cooling_ctrl();
        }
}

/**
 * @brief          发射机构弹速和热量控制
 * @param[in]      void
 * @retval
 */
void Shoot::cooling_ctrl()
{
        bool_t shot_detected = FALSE;
        fp32 effective_rearm_guard = SHOOT_HEAT_REARM_GUARD;
        fp32 min_required_rearm_guard = 0.0f;
        fp32 heat_block_threshold = 0.0f;
        fp32 heat_rearm_threshold = 0.0f;

        heat_fire_window = shot_detector_feed_window();
        shot_detected = detect_fric_shot();

        local_heat -= SHOOT_HEAT_COOLDOWN_PER_S * SHOOT_CONTROL_DT_S;
        if (local_heat < 0.0f)
        {
            local_heat = 0.0f;
        }

        if (shot_detected == TRUE)
        {
            local_heat += SHOOT_HEAT_PER_BULLET;
        }

        if (referee.online == TRUE && referee.fresh == TRUE)
        {
            if ((fp32)referee.shooter_17mm_heat > local_heat)
            {
                local_heat = (fp32)referee.shooter_17mm_heat;
            }
        }

        min_required_rearm_guard = heat_guard + SHOOT_HEAT_DETECTION_LEAD + SHOOT_HEAT_PER_BULLET;
        if (effective_rearm_guard < min_required_rearm_guard)
        {
            effective_rearm_guard = min_required_rearm_guard;
        }

        heat_block_threshold = heat_limit - heat_guard - SHOOT_HEAT_DETECTION_LEAD;
        heat_rearm_threshold = heat_limit - effective_rearm_guard;
        if (heat_block_threshold < 0.0f)
        {
            heat_block_threshold = 0.0f;
        }
        if (heat_rearm_threshold < 0.0f)
        {
            heat_rearm_threshold = 0.0f;
        }

        // 使用停火/恢复双阈值, 避免在热量上限附近反复开火。
        if (heat_block_latched == FALSE)
        {
            if (local_heat >= heat_block_threshold)
            {
                heat_block_latched = TRUE;
            }
        }
        else if (local_heat <= heat_rearm_threshold)
        {
            heat_block_latched = FALSE;
        }

        heat_allow_fire = (heat_block_latched == FALSE) ? TRUE : FALSE;

        if (shoot_mode == SHOOT_CONTINUE_BULLET &&
            trigger_anti_jam_state == TRIGGER_ANTI_JAM_IDLE &&
            heat_allow_fire == FALSE)
        {
            trigger_motor.speed_set = 0.0f;
        }
}

/**
 * @brief          PID计算
 * @param[in]      void
 * @retval
 */
void Shoot::solve()
{
        if (shoot_mode == SHOOT_STOP)
        {
            // shoot_laser_off();
            fric_status = FALSE;
            // 按理来说不需要在这里再设置一次速度了
        }
        else if (shoot_mode == SHOOT_READY)
        {

            // 调拨弹轮，暂时关闭摩擦轮
        }
        else if (shoot_mode == SHOOT_CONTINUE_BULLET)
        {
        }

        if (shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            trigger_motor.solve(SPEED);
        }
        else
        {
            trigger_motor.current_give = 0.0f;
        }
        fric_motor_left.solve(SPEED);
        fric_motor_right.solve(SPEED);

}

void Shoot::output()
{
        // fp32 trigger_current_cmd = 0.0f;

        // if (shoot_mode == SHOOT_CONTINUE_BULLET)
        // {
        //     trigger_current_cmd = trigger_motor.current_give;
        // }

        // // 发送电流  yyh暂停摩擦轮太耗电
        //   can_receive.can_cmd_shoot_motor(fric_motor_left.current_give, fric_motor_right.current_give, trigger_current_cmd, CAN_SHOOT_ALL_ID);
        // can_receive.can_cmd_shoot_motor(1000, 1000, 0,CAN_SHOOT_ALL_ID);

}





/**
 * @brief          摩擦轮刚打开时,云台抬头
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t shoot_open_fric_cmd_to_gimbal_up()
{
        if (shoot.shoot_mode > SHOOT_READY_FRIC)
        {
            return 1;
        }
        else
        {
            return 1;
        }
}

float fabss(float x)
{
        return (x < 0) ? -x : x;
}

void Shoot::reset_trigger_anti_jam()
{
        block_time = 0;
        reverse_time = 0;
        recovery_time = 0;
        startup_ignore_time = 0;
        forward_time = 0;
        jam_retry_count = 0;
        trigger_anti_jam_state = TRIGGER_ANTI_JAM_IDLE;
        trigger_forward_speed_set = 0.0f;
        trigger_motor.speed_pid.Clear();
}

bool_t Shoot::trigger_motor_blocked()
{
        if (trigger_motor.motor_measure == NULL)
        {
            return false;
        }

        return (fabss(trigger_motor.speed) < TRIGGER_JAM_SPEED_THRESHOLD) &&
               (abs_int16(trigger_motor.motor_measure->given_current) > TRIGGER_JAM_CURRENT_THRESHOLD);
}

void Shoot::reset_fric_shot_detector()
{
        fric_shot_state = FRIC_SHOT_STOPPED;
        fric_shot_current_raw = 0.0f;
        fric_shot_current_fast = 0.0f;
        fric_shot_current_slow = 0.0f;
        fric_shot_current_contrast = 0.0f;
        fric_ready_ticks = 0U;
        fric_suspect_ticks = 0U;
        fric_refractory_ticks = 0U;
}

bool_t Shoot::shot_detector_feed_window()
{
        if (shoot_mode != SHOOT_CONTINUE_BULLET)
        {
            return FALSE;
        }

        if (heat_allow_fire == FALSE || heat_block_latched == TRUE)
        {
            return FALSE;
        }

        if (trigger_anti_jam_state != TRIGGER_ANTI_JAM_IDLE)
        {
            return FALSE;
        }

        if (fabss(trigger_forward_speed_set) < TRIGGER_JAM_CMD_MIN_SPEED)
        {
            return FALSE;
        }

        if (startup_ignore_time < TRIGGER_JAM_STARTUP_IGNORE_TIME)
        {
            return FALSE;
        }

        if (trigger_motor.motor_measure == NULL)
        {
            return FALSE;
        }

        if (trigger_motor_blocked())
        {
            return FALSE;
        }

        return TRUE;
}

bool_t Shoot::shot_detector_friction_ready()
{
        bool_t left_ready = FALSE;
        bool_t right_ready = FALSE;

        if (fric_motor_left.motor_measure == NULL || fric_motor_right.motor_measure == NULL)
        {
            fric_ready_ticks = 0U;
            return FALSE;
        }

        if (fabss(fric_motor_left.speed_set) < 0.1f || fabss(fric_motor_right.speed_set) < 0.1f)
        {
            fric_ready_ticks = 0U;
            return FALSE;
        }

        left_ready = (fabss(fric_motor_left.speed) >= fabss(fric_motor_left.speed_set) * SHOOT_FRIC_READY_SPEED_RATIO) ? TRUE : FALSE;
        right_ready = (fabss(fric_motor_right.speed) >= fabss(fric_motor_right.speed_set) * SHOOT_FRIC_READY_SPEED_RATIO) ? TRUE : FALSE;

        if (left_ready == TRUE && right_ready == TRUE)
        {
            if (fric_ready_ticks < 65535U)
            {
                fric_ready_ticks++;
            }
        }
        else
        {
            fric_ready_ticks = 0U;
        }

        return (fric_ready_ticks >= SHOOT_FRIC_READY_DWELL_TICKS) ? TRUE : FALSE;
}

bool_t Shoot::detect_fric_shot()
{
        fp32 left_current = 0.0f;
        fp32 right_current = 0.0f;
        fp32 contrast_enter_threshold = 0.0f;
        fp32 contrast_release_threshold = 0.0f;
        bool_t feed_window = FALSE;

        if (fric_motor_left.motor_measure == NULL || fric_motor_right.motor_measure == NULL)
        {
            reset_fric_shot_detector();
            return FALSE;
        }

        left_current = (fp32)abs_int16(fric_motor_left.motor_measure->given_current) * SHOOT_FRIC_DETECT_CURRENT_SCALE;
        right_current = (fp32)abs_int16(fric_motor_right.motor_measure->given_current) * SHOOT_FRIC_DETECT_CURRENT_SCALE;
        fric_shot_current_raw = (left_current > right_current) ? left_current : right_current;

        fric_shot_current_fast += (fric_shot_current_raw - fric_shot_current_fast) * SHOOT_FRIC_DETECT_FAST_ALPHA;
        fric_shot_current_slow += (fric_shot_current_raw - fric_shot_current_slow) * SHOOT_FRIC_DETECT_SLOW_ALPHA;
        fric_shot_current_contrast = fric_shot_current_fast - fric_shot_current_slow;
        if (fric_shot_current_contrast < 0.0f)
        {
            fric_shot_current_contrast = 0.0f;
        }

        if (shot_detector_friction_ready() == FALSE)
        {
            fric_shot_state = FRIC_SHOT_STOPPED;
            fric_suspect_ticks = 0U;
            fric_refractory_ticks = 0U;
            return FALSE;
        }

        if (fric_shot_state == FRIC_SHOT_STOPPED)
        {
            fric_shot_state = FRIC_SHOT_READY;
        }

        feed_window = shot_detector_feed_window();
        if (feed_window == FALSE)
        {
            fric_shot_state = FRIC_SHOT_READY;
            fric_shot_current_fast = fric_shot_current_raw;
            fric_shot_current_slow = fric_shot_current_raw;
            fric_shot_current_contrast = 0.0f;
            fric_suspect_ticks = 0U;
            fric_refractory_ticks = 0U;
            return FALSE;
        }

        contrast_enter_threshold = fric_shot_current_slow * SHOOT_FRIC_DETECT_CONTRAST_RATIO;
        if (contrast_enter_threshold < SHOOT_FRIC_DETECT_CONTRAST_MIN)
        {
            contrast_enter_threshold = SHOOT_FRIC_DETECT_CONTRAST_MIN;
        }
        contrast_release_threshold = contrast_enter_threshold * SHOOT_FRIC_DETECT_RELEASE_RATIO;

        switch (fric_shot_state)
        {
        case FRIC_SHOT_READY:
            if (fric_shot_current_contrast >= contrast_enter_threshold)
            {
                fric_shot_state = FRIC_SHOT_SUSPECT;
                fric_suspect_ticks = 1U;
            }
            break;

        case FRIC_SHOT_SUSPECT:
            if (fric_shot_current_contrast < contrast_release_threshold)
            {
                fric_shot_state = FRIC_SHOT_READY;
                fric_suspect_ticks = 0U;
            }
            else
            {
                if (fric_suspect_ticks < 65535U)
                {
                    fric_suspect_ticks++;
                }

                if (fric_suspect_ticks >= SHOOT_FRIC_DETECT_CONFIRM_TICKS)
                {
                    fric_shot_state = FRIC_SHOT_REFRACTORY;
                    fric_suspect_ticks = 0U;
                    fric_refractory_ticks = 0U;
                    return TRUE;
                }
            }
            break;

        case FRIC_SHOT_REFRACTORY:
            if (fric_refractory_ticks < SHOOT_FRIC_DETECT_REFRACTORY_TICKS)
            {
                fric_refractory_ticks++;
            }
            else if (fric_shot_current_contrast < contrast_release_threshold)
            {
                fric_refractory_ticks = 0U;
                fric_shot_state = FRIC_SHOT_READY;
            }
            break;

        case FRIC_SHOT_STOPPED:
        default:
            fric_shot_state = FRIC_SHOT_READY;
            break;
        }

        return FALSE;
}

//拨弹轮反转控制
//当前用状态机控制防卡弹逻辑：
//刚开始连发不判断卡弹--检测卡弹--无卡弹继续连发--有卡弹反转--反转一段时间后停止--停止一段时间后继续尝试连发--如果多次尝试仍然卡弹则锁死拨弹轮（即设置目标速度为0）
void Shoot::trigger_motor_turn_back()
{
        if (fabss(trigger_forward_speed_set) < TRIGGER_JAM_CMD_MIN_SPEED)
        {
            reset_trigger_anti_jam();
            trigger_motor.speed_set = 0.0f;
            return;
        }

        switch (trigger_anti_jam_state)
        {
        case TRIGGER_ANTI_JAM_IDLE:
            trigger_motor.speed_set = trigger_forward_speed_set;

            if (startup_ignore_time < TRIGGER_JAM_STARTUP_IGNORE_TIME)
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
                    block_time = 0;
                    reverse_time = 0;
                    recovery_time = 0;
                    startup_ignore_time = 0;
                    trigger_motor.speed_pid.Clear();
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
            trigger_motor.speed_set = -TRIGGER_JAM_REVERSE_SPEED * SHOOT_TRIGGER_DIRECTION;
            if (reverse_time < TRIGGER_JAM_REVERSE_TIME)
            {
                reverse_time++;
            }
            else
            {
                reverse_time = 0;
                recovery_time = 0;
                trigger_motor.speed_pid.Clear();
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
                trigger_motor.speed_pid.Clear();
                trigger_anti_jam_state = TRIGGER_ANTI_JAM_LOCKED;
            }
            else
            {
                recovery_time = 0;
                block_time = 0;
                startup_ignore_time = 0;
                forward_time = 0;
                trigger_motor.speed_pid.Clear();
                trigger_anti_jam_state = TRIGGER_ANTI_JAM_IDLE;
            }
            break;

        case TRIGGER_ANTI_JAM_LOCKED:
        default:
            trigger_motor.speed_set = 0.0f;
            break;
        }
}
