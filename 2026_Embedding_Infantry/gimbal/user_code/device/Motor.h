#ifndef MOTOR_H
#define MOTOR_H

#include "Imu.h"
#include "Pid.h"
#include "Can_receive.h"

#define SPEED 0
#define ENCODE_ANGLE 1
#define GYRO_ANGLE 2

//使用MIT模式，可控制定速度，用于巡逻模式，可指定位置，听上位机指令
#define DM_MOTOR 0x01 //目前6006电机做YAW轴
#define MIT_MODE 0x000 //其余模式看情况使用
#define PS_MODE 0x100
#define DM_KP 30   //目前双环控制 KP==30  KD==1.2   单环模式KD==1  
#define DM_KI 0
#define DM_KD 1.2
#define PMAX 12.5f		//位置映射范围
#define	VMAX 30.0f		//速度映射范围
#define	TMAX 10.0f		//扭矩映射范围

#define DJI_RPM_TO_RAD 0.0066446545f //2*PI/60/15.76
#define DM_DEG_TO_RAD 0.0174532925f //IMU角度转弧度


/*
电机父类
分析所有电机，共性如下
控制所需属性
1.canid
2.pid
3.速度及目标
4.角度及目标
5.电流值


控制所需函数
1.更新数据
2.设置目标
3.求解


*/

class Motor{
public:



    //speed & speed_set rad/s
    float speed;
    float speed_set;

    //encode_angle & encode_angle_set  rad
    float encode_angle;
    float encode_angle_set;
    
    //gyro_angle & gyro_angle_set rad
    float gyro_angle;
    float gyro_angle_set;

    //canID
    uint16_t motor_id;

    //speed_PID
    Pid speed_pid;
    //gyro_angle_PID
    Pid gyro_angle_pid;
    //encode_angle_PID
    Pid encode_angle_pid;

    float current_give;
   
    Motor(){}
    Motor(uint16_t motor_id);
                                                                                                                        
    void update();
    void set(float set,uint8_t mode);
    void solve(uint8_t mode);

};

class DJI_Motor : public Motor
{
public:

    //the zero point ecd
    uint16_t offset_ecd;

    //the whole ecd
    uint16_t max_ecd;

    DJI_Motor(){}
    const dji_motor_measure_t *motor_measure;

    DJI_Motor(uint8_t motor_id,const dji_motor_measure_t* motor_measure,uint16_t offset_ecd,uint16_t max_ecd):
    Motor(motor_id),offset_ecd(offset_ecd),max_ecd(max_ecd),motor_measure(motor_measure){}
    fp32 ecd_to_angle(uint16_t ecd);

    void update();

};

class DM_Motor : public Motor
{
public:

    //成员变量
    uint16_t id;        //控制帧id
    uint16_t mst_id;    //反馈帧id
    const dm_motor_measure_t *motor_measure;  //电机反馈数据
    esc_inf_t tmp;      //映射表
    motor_ctrl_t ctrl;  //控制参数
    // const protocol_info_t *gyro_angle_measure;
    

    //构造函数
    DM_Motor(){}
    DM_Motor(uint16_t id,uint16_t mst_id,const dm_motor_measure_t* motor_measure):
    Motor(motor_id),id(id),mst_id(mst_id),motor_measure(motor_measure){}

    fp32 uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits);
    
    void dm_mit_init();
    // void dm_ps_init();
    void update();

   
};


extern Can_receive can_receive ;




#endif
