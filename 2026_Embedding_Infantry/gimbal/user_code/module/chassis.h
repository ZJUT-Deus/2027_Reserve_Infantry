#ifndef CHASSIS_H
#define CHASSIS_H


#include "Pid.h"
#include "Motor.h"
#include "gimbal.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "user_lib.h"
#include "Super_cap.h"
#include "communicate.h"
#include "struct_typedef.h"
#include "First_order_filter.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3

//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 0

//初始yaw轴角度
#define INIT_YAW_SET  -0.2f

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 1

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.005f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define TOP_MOVE_INTENT_RC_DEADLINE 30

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f//0.50f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define MOTOR_WHEEL_RADIUS 0.076f //轮子半径 0.076m
#define MOTOR_DISTANCE_TO_CENTER 0.185f //底盘半径

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
#define CHASSIS_POWER_LIMIT_W 120.0f
#define CHASSIS_POWER_MODEL_K0 0.2962f
#define CHASSIS_POWER_MODEL_K1 0.0f
#define CHASSIS_POWER_MODEL_K2 0.1519f
#define CHASSIS_POWER_MODEL_A 1.3544f
#define CHASSIS_POWER_CURRENT_TO_A (20.0f / 16384.0f)
#define CHASSIS_POWER_SOLVE_EPS 1e-6f
#define CHASSIS_POWER_GUARD_W 12.0f
#define CHASSIS_POWER_ESTIMATE_SCALE 1.25f

/*----------------按键-------------------------*/


//底盘前后左右控制按键WASD
#define KEY_CHASSIS_FRONT           IF_KEY_PRESSED_W_VT13
#define KEY_CHASSIS_BACK            IF_KEY_PRESSED_S_VT13
#define KEY_CHASSIS_LEFT            IF_KEY_PRESSED_A_VT13
#define KEY_CHASSIS_RIGHT           IF_KEY_PRESSED_D_VT13

// #define KEY_CHASSIS_SWING           if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_SWING)
// #define KEY_CHASSIS_PISA            if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_PISA)
// #define KEY_CHASSIS_SUPER_CAP       if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_CHASSIS_SUPER_CAP)
// #define KEY_UI_UPDATE               if_key_singal_pessed(chassis_RC, last_chassis_RC, KEY_PRESSED_UI_UPDATE)

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR


//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 15.0f //4
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5
//底盘运动过程最大旋转速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 14.0f

//原地旋转小陀螺下Z轴常态转速
//#define TOP_WZ_ANGLE_STAND 14.0f
#define TOP_WZ_ANGLE_STAND 10.0f
//小陀螺开关 0关 1开
//小陀螺开关 0关 1开已整合入Communicate.h
//变速小陀螺开关 0关 1开
#define TOP_VAR_SPIN_ENABLE 0

//受击后阶梯变速小陀螺参数
// #define TOP_WZ_ANGLE_VAR_STAGE1 16.0f
// #define TOP_WZ_ANGLE_VAR_STAGE2 24.0f
// #define TOP_WZ_ANGLE_VAR_STAGE3 16.0f
#define TOP_WZ_ANGLE_VAR_STAGE1 14.0f
#define TOP_WZ_ANGLE_VAR_STAGE2 20.0f
#define TOP_WZ_ANGLE_VAR_STAGE3 14.0f
#define TOP_WZ_VAR_STAGE1_TIME_MS 500U
#define TOP_WZ_VAR_STAGE2_TIME_MS 1000U
#define TOP_WZ_VAR_STAGE3_TIME_MS 500U

//只有在操作手连续无移动意图一段时间后，才允许受击触发变速小陀螺
#define TOP_NO_MOVE_CONFIRM_TIME_MS 50U

//小陀螺模式 底盘相对云台的前馈补偿 全部参数
#define TOP_RELATIVE_FF_DELAY_MIN 0.045f
#define TOP_RELATIVE_FF_DELAY_MAX 0.065f
#define TOP_RELATIVE_FF_OMEGA_MIN 3.333333f
#define TOP_RELATIVE_FF_OMEGA_MAX 18.0f
#define TOP_RELATIVE_FF_MAX_DELTA_ANGLE 0.45f
#define TOP_RELATIVE_FF_SIGN 1.0f
#define TOP_RELATIVE_FF_POWER_GAIN 1.2f
#define TOP_RELATIVE_FF_DELAY_SCALE_MAX 2.5f

#define TOP_TUNE_MIN_REF_SPEED 0.15f
#define TOP_TUNE_MIN_RELATIVE_OMEGA 0.5f
#define TOP_TUNE_RAD_TO_DEG 57.295779513f


#define CHASSIS_WZ_SET_SCALE 0.1f


//电机反馈码盘值范围
#define DJI_HALF_ECD_RANGE 4096
#define DJI_ECD_RANGE 8192

//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

#define MISS_CLOSE 0
#define MISS_BEGIN 1
#define MISS_OVER 2


#define switch_is_down(x) ((x) == 0)
#define switch_is_mid(x) ((x) == 1)
#define switch_is_up(x)  ((x) == 2)

#define CHASSIS_CONTROL_TIME 0.002f
#define PISA_DELAY_TIME 500
#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上





typedef enum
{
    CHASSIS_ZERO_FORCE,                 //无力模式

    CHASSIS_FREE,                       //自由/跟随模式

    CHASSIS_TOP,                        //小陀螺模式
                                 
} chassis_behaviour_e;                   

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  //底盘跟随云台,底盘移动速度由遥控器和键盘决定,旋转速度由云台角度差计算出CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW 选择的控制模式

    CHASSIS_VECTOR_NO_FOLLOW_YAW,      //底盘不跟随云台,底盘移动速度和旋转速度由遥控器决定，无角度环控制CHASSIS_NO_FOLLOW_YAW 和 CHASSIS_NO_MOVE 选择的控制模式*/

    CHASSIS_VECTOR_RAW,                //底盘不跟随云台.底盘电机电流控制值是直接由遥控器通道值计算出来的，将直接发送到 CAN 总线上CHASSIS_OPEN 和 CHASSIS_ZERO_FORCE 选择的控制模式*/


} chassis_mode_e; 

//CHASSIS_TOP 内部的二级状态机：常态小陀螺 / 受击后变速小陀螺
typedef enum
{
    TOP_SPIN_STEADY = 0,
    TOP_SPIN_VAR_SPIN,
} top_spin_state_e;

//受击后阶梯变速小陀螺的阶段状态机
typedef enum
{
    TOP_VAR_STAGE_NONE = 0,
    TOP_VAR_STAGE_1,
    TOP_VAR_STAGE_2,
    TOP_VAR_STAGE_3,
} top_var_stage_e;

struct speed_t
{
    fp32 speed;
    fp32 speed_set;
    fp32 max_speed;
    fp32 min_speed;
};

//细化观测小陀螺前馈误差
struct top_tune_debug_t
{
    fp32 ref_vx;
    fp32 ref_vy;
    fp32 ref_speed_norm;
    fp32 meas_vx_ref;
    fp32 meas_vy_ref;
    fp32 meas_speed_norm;
    fp32 direction_error_rad;
    fp32 direction_error_deg;
    fp32 lateral_speed;
    fp32 lateral_ratio;
    fp32 relative_omega;
    fp32 equivalent_delay_s;
    fp32 equivalent_delay_ms;
    fp32 feedforward_delay_s;
    fp32 feedforward_delay_ms;
    fp32 feedforward_delta_angle_rad;
    fp32 feedforward_delta_angle_deg;
    uint8_t active;
    uint8_t valid_direction;
    uint8_t valid_delay;
};



class Chassis {
public:

    const VT13_RC_ctrl_t *chassis_RC; //底盘使用的遥控器指针
    VT13_RC_ctrl_t *last_chassis_RC; //底盘使用的遥控器指针

    uint16_t chassis_last_key_v;  //遥控器上次按键

    chassis_behaviour_e chassis_behaviour_mode; //底盘行为状态机
    chassis_behaviour_e last_chassis_behaviour_mode; //底盘上次行为状态机

    chassis_mode_e chassis_mode; //底盘控制状态机
    chassis_mode_e last_chassis_mode; //底盘上次控制状态机
    
    DJI_Motor chassis_motive_motor[4]; //底盘动力电机数据

    First_order_filter chassis_cmd_slow_set_vx;        //使用一阶低通滤波减缓设定值
    First_order_filter chassis_cmd_slow_set_vy;        //使用一阶低通滤波减缓设定值

    Pid chassis_wz_angle_pid;        //底盘角度pid

    speed_t x;
    speed_t y;
    speed_t z;

    fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad
    fp32 last_chassis_relative_angle;
    fp32 chassis_relative_angle_set; //设置相对云台控制角度
    fp32 chassis_yaw_set;

    fp32 chassis_yaw;   //底盘的yaw角度
    //功率控制参数
    volatile fp32 power_eta;
    volatile fp32 power_estimate_w;
    volatile fp32 power_estimate_raw_w;
    volatile fp32 power_budget_w;

    //CHASSIS_TOP 内部受击变速小陀螺状态机数据
    top_spin_state_e top_spin_state;
    top_var_stage_e top_var_stage;
    uint32_t top_var_stage_start_ms;
    uint32_t top_no_move_start_ms;
    uint32_t top_hit_event_pending_start_ms;
    bool_t top_no_move_timer_started;
    bool_t top_no_move_confirmed;
    bool_t top_move_intent;
    bool_t top_hit_event_pending;
    uint16_t last_referee_hp;
    bool_t referee_hp_initialized;

    


    //任务流程
    void init();

    void set_mode();

    void feedback_update();

    void set_contorl();

    void solve();

    void power_ctrl();

    void output();

    //行为控制

    void chassis_behaviour_mode_set();

    void chassis_behaviour_control_set(fp32 *vx_set_, fp32 *vy_set_, fp32 *angle_set);

    void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set);

    void chassis_free_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    void chassis_top_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    //功能性函数
    void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set);

    void chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4]);

    fp32 calc_top_feedforward_delay(fp32 relative_omega_abs);

    fp32 calc_top_feedforward_delta_angle(fp32 *feedforward_delay);

    //CHASSIS_TOP 内部受击变速小陀螺状态机辅助函数
    void reset_top_spin_state_machine();

    bool_t chassis_has_move_intent();

    void update_top_no_move_confirm(uint32_t now_ms);

    bool_t consume_valid_hit_event();

    void start_top_var_spin(uint32_t now_ms);

    fp32 update_top_spin_command(uint32_t now_ms);

    void clear_top_tune_debug();

    void update_top_tune_debug(fp32 vx_set, fp32 vy_set, fp32 feedforward_delay, fp32 feedforward_delta_angle);
    
    
};


extern Chassis chassis;
extern volatile top_tune_debug_t top_tune_debug;



#endif
