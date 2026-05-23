#ifndef SHOOT_H
#define SHOOT_H

#include "Motor.h"
//#include "Config.h"
#include "user_lib.h"
#include "Communicate.h"
#include "struct_typedef.h"
#include "First_order_filter.h"
#include "ShootHeatConfig.h"

#define TRIGGER_CCW 1 //拨盘顺时针
#define TRIGGER_CW -1 //拨盘逆时针

#define SHOOT_TRIGGER_DIRECTION TRIGGER_CCW

//暂停按键
#define stop_on(x) ((x) == 1)
#define stop_off(x) ((x) == 0)

//云台行为模式判断
#define switch_is_down(x) ((x) == 0)
#define switch_is_mid(x) ((x) == 1)
#define switch_is_up(x)  ((x) == 2)


//开启摩擦轮的斜坡
#define SHOOT_FRIC_ADD_VALUE 0.1f

#define SHOOT_CONTROL_DT_S 0.002f // 控制算法使用的秒级周期, 用于滤波和热量积分


//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 15
//鼠标左键长按判断
#define PRESS_L_LONG_TIME 400
#define PRESS_R_LONG_TIME 50

//摩擦轮开启按键延时
#define KEY_FRIC_LONG_TIME 200

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清空弹夹
#define RC_S_LONG_TIME 3000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//摩擦轮电机rmp 变化成 旋转速度的比例
#define FRIC_RPM_TO_SPEED 0.000415809748903494517209f
#define FRIC_STD_SPEED_RATIO 0.433f //摩擦轮规范速度倍率

#define FRIC_REQUIRE_SPEED_RMP 500.0f
#define FRIC_MAX_SPEED_RMP 4000.0f

#define FRIC_MAX_SPEED 80.0f
#define FRIC_MAX_REQUIRE_SPEED 30.0f

//拨盘电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f * SHOOT_TRIGGER_DIRECTION
#define FULL_COUNT 18

//拨弹速度
#define TRIGGER_SPEED 10.0f * SHOOT_TRIGGER_DIRECTION          //10
#define CONTINUE_TRIGGER_SPEED 15.0f * SHOOT_TRIGGER_DIRECTION //15
#define READY_TRIGGER_SPEED 5.0f * SHOOT_TRIGGER_DIRECTION     //5

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1
//防卡弹参数设置
#define TRIGGER_JAM_SPEED_THRESHOLD 1.0f //低速阈值 低于这个值 拨弹等于没转
#define TRIGGER_JAM_CURRENT_THRESHOLD 3000 //高电流阈值 高于这个值 等于卡弹
#define TRIGGER_JAM_DETECT_TIME 80 //检测卡弹时间 连续满足这个条件 等于真卡弹
#define TRIGGER_JAM_REVERSE_TIME 100//反转时间 
#define TRIGGER_JAM_RECOVERY_TIME 40//恢复时间 停止以后尝试前拨
#define TRIGGER_JAM_STARTUP_IGNORE_TIME 100//启动忽略时间
#define TRIGGER_JAM_RETRY_RESET_TIME 150//正常拨弹到这个时间 就把尝试次数重置 避免次数累计
#define TRIGGER_JAM_RETRY_MAX 3//最大重试次数
#define TRIGGER_JAM_CMD_MIN_SPEED 0.1f//如果前拨目标值接近0 就不判断卡弹了 避免误判
#define TRIGGER_JAM_REVERSE_SPEED 30.0f//反转拨弹速度

#define SHOOT_HEAT_GUARD 20.0f
#define SHOOT_HEAT_REARM_GUARD 50.0f
#define SHOOT_HEAT_DETECTION_LEAD 30.0f
#define SHOOT_HEAT_PER_BULLET 12.0f//每颗小弹的热量10 

#define SHOOT_FRIC_DETECT_CURRENT_SCALE 100.0f
#define SHOOT_FRIC_READY_SPEED_RATIO 0.85f
#define SHOOT_FRIC_READY_DWELL_TICKS 30U
#define SHOOT_FRIC_DETECT_FAST_ALPHA 0.35f
#define SHOOT_FRIC_DETECT_SLOW_ALPHA 0.05f
#define SHOOT_FRIC_DETECT_CONTRAST_MIN 30000.0f
#define SHOOT_FRIC_DETECT_CONTRAST_RATIO 0.35f
#define SHOOT_FRIC_DETECT_RELEASE_RATIO 0.45f
#define SHOOT_FRIC_DETECT_CONFIRM_TICKS 2U
#define SHOOT_FRIC_DETECT_REFRACTORY_TICKS 10U

/*---------------------------pid----------------------*/
//左摩擦轮电机PID
#define FRIC_left_SPEED_PID_KP 2000.0f // 1800
#define FRIC_left_SPEED_PID_KI 0.8f    // 0.5
#define FRIC_left_SPEED_PID_KD 2.0f    // 2.0
#define FRIC_left_SPEED_PID_KF 0.0f
#define FRIC_left_PID_MAX_IOUT 200.0f
#define FRIC_left_PID_MAX_OUT  6000.0f

//右摩擦轮电机PID
#define FRIC_right_SPEED_PID_KP 2000.0f // 1800
#define FRIC_right_SPEED_PID_KI 0.8f    // 0.5
#define FRIC_right_SPEED_PID_KD 2.0f    // 2.0
#define FRIC_right_SPEED_PID_KF 0.0f
#define FRIC_right_PID_MAX_IOUT 200.0f
#define FRIC_right_PID_MAX_OUT  6000.0f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 2000.0f 
#define TRIGGER_ANGLE_PID_KI 0.5f   
#define TRIGGER_ANGLE_PID_KD 0.0f
#define TRIGGER_ANGLE_PID_KF 0.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 1000.0f
#define TRIGGER_BULLET_PID_MAX_OUT 4000.0f

#define TRIGGER_READY_PID_MAX_IOUT 2000.0f
#define TRIGGER_READY_PID_MAX_OUT 4000.0f


//一阶低通滤波参数
#define SHOOT_ACCEL_FRIC_LEFT_NUM 0.2666666667f
#define SHOOT_ACCEL_FRIC_RIGHT_NUM 0.2666666667f


//电机序号
#define LEFT_FRIC 0
#define RIGHT_FRIC 1
#define TRIGGER 2

typedef enum
{
  SHOOT_STOP = 0,        //停止发射结构
  SHOOT_READY_FRIC,      //摩擦轮准备中
  SHOOT_READY_BULLET,    //拨盘准备中,摩擦轮已达到转速
  SHOOT_READY,           //整个发射机构准备完成
  SHOOT_BULLET,          //单发
  SHOOT_CONTINUE_BULLET, //连发
  SHOOT_DONE,

} shoot_mode_e;

typedef enum
{
  TRIGGER_ANTI_JAM_IDLE = 0,//闲置状态机（正常连射）
  TRIGGER_ANTI_JAM_REVERSE,//反转状态机
  TRIGGER_ANTI_JAM_RECOVERY,//反转恢复状态机
  TRIGGER_ANTI_JAM_LOCKED,//锁死状态机
} trigger_anti_jam_state_e;

typedef enum
{
  FRIC_SHOT_STOPPED = 0,
  FRIC_SHOT_READY,
  FRIC_SHOT_SUSPECT,
  FRIC_SHOT_REFRACTORY,
} fric_shot_detect_state_e;

class Shoot
{
public:
  const VT13_RC_ctrl_t *shoot_rc;
  VT13_RC_ctrl_t *last_shoot_rc;

  uint16_t shoot_last_key_v;

  shoot_mode_e shoot_mode;

  //摩擦轮电机
 DJI_Motor fric_motor_left;
 DJI_Motor fric_motor_right;
  //拨弹电机
 DJI_Motor trigger_motor;
 

  
  First_order_filter shoot_cmd_slow_fric_left; //使用一阶低通滤波减缓设定值
  First_order_filter shoot_cmd_slow_fric_right; //使用一阶低通滤波减缓设定值

  //摩擦轮电机 限位开关 状态
  bool_t fric_status;
  bool_t limit_switch_status;
  //TODO 添加收到开关激光

  //鼠标状态
  bool_t press_l;
  bool_t press_r;
  bool_t last_press_l;
  bool_t last_press_r;
  uint16_t press_l_time;
  uint16_t press_r_time;
  uint16_t rc_s_time;

  uint16_t block_time;
  uint16_t reverse_time;
  uint16_t recovery_time;
  uint16_t startup_ignore_time;
  uint16_t forward_time;
  uint16_t move_flag;
  uint16_t cover_move_flag;
  uint8_t jam_retry_count;
  trigger_anti_jam_state_e trigger_anti_jam_state;
  fp32 trigger_forward_speed_set;

  fp32 local_heat;
  fp32 heat_limit;
  fp32 heat_guard;
  bool_t heat_allow_fire;
  bool_t heat_block_latched;
  bool_t heat_fire_window;
  fric_shot_detect_state_e fric_shot_state;
  fp32 fric_shot_current_raw;
  fp32 fric_shot_current_fast;
  fp32 fric_shot_current_slow;
  fp32 fric_shot_current_contrast;
  uint16_t fric_ready_ticks;
  uint16_t fric_suspect_ticks;
  uint16_t fric_refractory_ticks;

  //TODO 暂时未安装微动开关
  //微动开关
  bool_t key;
  uint8_t key_time;

  void init();            //云台初始化
  void set_mode();        //设置发射机构控制模式
  void feedback_update(); //发射数据反馈
  void set_control();     //设置发射机构控制量
  void cooling_ctrl();    //发射机构弹速和热量控制
  void solve();           //发射机构控制PID计算
  void output();          //输出电流

  //拨盘旋转相关函数
  void trigger_motor_turn_back();
  void reset_trigger_anti_jam();
  bool_t trigger_motor_blocked();
  void reset_fric_shot_detector();
  bool_t shot_detector_feed_window();
  bool_t shot_detector_friction_ready();
  bool_t detect_fric_shot();
  

};

//发射机构控制云台不动
bool_t shoot_cmd_to_gimbal_stop();
//发射机构控制云台抬头
bool_t shoot_open_fric_cmd_to_gimbal_up();

extern float fabss(float x);
extern Shoot shoot;

#endif
