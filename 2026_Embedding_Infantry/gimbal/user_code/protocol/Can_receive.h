#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "bsp_fdcan.h"

#include "struct_typedef.h"

//DM设备和DJI设备CAN总线分离
#define CHASSIS_CAN hfdcan1
#define SHOOT_CAN hfdcan1
#define GIMBAL_CAN hfdcan2

typedef enum
{

  //底盘动力电机接收ID  fdcan1
    CAN_MOTIVE_FR_MOTOR_ID = 0x201,
    CAN_MOTIVE_FL_MOTOR_ID = 0x202,
    CAN_MOTIVE_BL_MOTOR_ID = 0x203,
    CAN_MOTIVE_BR_MOTOR_ID = 0x204,
   CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //云台发射机构电机接收ID fdcan1
    CAN_LEFT_FRIC_MOTOR_ID = 0x205, //
    CAN_RIGHT_FRIC_MOTOR_ID = 0x206,//
    CAN_TRIGGER_MOTOR_ID = 0x207,   //拨弹轮
    CAN_SHOOT_ALL_ID = 0x1FF,     

  //裁判系统数据接收ID fdcan1
    CAN_REFEREE_DATA_ID = 0x103,
    CAN_REFEREE_DATA_EXT_ID = 0x104,
  //超级电容数据接收ID fdcan1
    CAN_SUPER_CAP_ID = 0x051,
  //云台控制机构电机接收ID fdcan2
    CAN_YAW_MOTOR_ID = 0x01, //yaw轴控制id
    MASTER_YAW_MOTOR_ID = 0x011, //yaw轴反馈id
    CAN_PITCH_MOTOR_ID = 0x02, //pitch轴控制id
    MASTER_PITCH_MOTOR_ID = 0x012, //pitch轴反馈id
    CAN_GIMBAL_ALL_ID = 0x1FF,//不需要 可删去



    
} can_msg_id_e;

//DJI 电机数据结构体
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} dji_motor_measure_t;

//DM 电机数据结构体
typedef struct 
{
  int id;
  int state;
  int p_int;
  int v_int;
  int t_int;
  fp32 Tmos;
  fp32 Tcoil;
} dm_motor_measure_t;

// DM 电机参数设置结构体
typedef struct 
{
    uint8_t mode;
    fp32 pos_set;
    fp32 vel_set;
    fp32 tor_set;
    fp32 cur_set;
    fp32 kp_set;
    fp32 kd_set;
} motor_ctrl_t;

typedef struct {
    fp32 pmax;
    fp32 vmax;
    fp32 tmax;
} esc_inf_t;

//接收裁判系统数据体 
typedef struct
{
    uint8_t color; //己方颜色 0为红 1为蓝
    uint8_t game_progress; //比赛进程
    uint16_t current_HP;//机器人当前血量
    uint16_t shooter_17mm_heat;//枪管当前热量
    uint16_t launching_frequnency;//当前射频
    float initial_speed;//当前射速

} Referee_data_t;

class Can_receive{
public:

//底盘电机
dji_motor_measure_t chassis_motive_motor[4];

//云台控制机构电机反馈数据结构体
dm_motor_measure_t gimbal_dm_motor[2];
motor_ctrl_t dm_motor_ctrl[2];
esc_inf_t dm_motor_esc_inf[2];

//云台发射机构电机反馈数据结构体
dji_motor_measure_t shoot_motor[3];

//发送数组
uint8_t can_send_data[8];



//初始化CAN通信
void init();

//云台发射机构电机数据接收
void get_shoot_motor_measure(uint8_t num, uint8_t data[8]);
void get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8]);
const dji_motor_measure_t *get_dji_motor_measure_point(uint8_t i);
const dji_motor_measure_t *get_motor_chassis_measure_point(uint8_t i);

//底盘能量数据发送超级电容
void can_cmd_super_cap(uint8_t enableDCDC, uint8_t systemRestart, uint16_t feedbackRefereePowerLimit, uint16_t feedbackRefereeEnergyBuffer, uint32_t ID);
//电机控制命令发送
void can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger,uint16_t ID);
void can_cmd_dji_motor(int16_t current1, int16_t current2, int16_t current3, int16_t current4,uint16_t ID);

//云台控制机构电机数据接收
void get_dm_motor_measure(dm_motor_measure_t *dm_motor, uint8_t data[8]);
const dm_motor_measure_t *get_dm_motor_measure_point(uint8_t i);
void mit_ctrl(fp32 pos, fp32 kp, fp32 kd,fp32 vel,fp32 tor, uint16_t id, uint16_t mode_id,esc_inf_t tmp);
void ps_ctrl(float _pos, float _vel,uint16_t id, uint16_t mode_id);
int float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits);
fp32 uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits);
void ENABLE_DM_Motor_mode(uint16_t id, uint16_t mode_id);
void DISABLE_DM_Motor_mode(uint16_t id, uint16_t mode_id);

//UI数据板间通讯
void send_ui_board_com(uint8_t _right_button,uint8_t _top_switch,float _distance,uint16_t ID);

};

#endif
