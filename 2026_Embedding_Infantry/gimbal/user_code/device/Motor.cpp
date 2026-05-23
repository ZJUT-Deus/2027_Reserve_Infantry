#include "Motor.h"


float yyyh=0;
float yyyyh=0;

Motor::Motor(uint16_t motor_id): motor_id(motor_id)
    {
        this->speed=0;
        this->speed_set=0;

        this->encode_angle=0;
        this->encode_angle_set=0;

        this->gyro_angle=0;
        this->gyro_angle_set=0;
    }
   
void Motor::update()
{
}

void Motor::set(float set, uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        this->speed_set = set;
        break;
    case ENCODE_ANGLE:
        this->encode_angle_set = set;
        break;
    case GYRO_ANGLE:
        this->gyro_angle_set = set;
        break;
    default:
        break;
    }
}

void Motor::solve(uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        current_give = speed_pid.pid_calc();
        break;
    case ENCODE_ANGLE:
        speed_set = encode_angle_pid.pid_calc();
        current_give = speed_pid.pid_calc();
        yyyyh = speed_set;
        yyyh = current_give;
        break;
    case GYRO_ANGLE:
        speed_set = gyro_angle_pid.pid_calc();
        current_give = speed_pid.pid_calc();
        break;
    default:
        break;
    }
}

fp32 DJI_Motor::ecd_to_angle(uint16_t ecd)
{
    int16_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > max_ecd/2)
    {
        relative_ecd -= max_ecd;
    }
    else if (relative_ecd < -max_ecd/2)
    {
        relative_ecd += max_ecd;
    }

    return relative_ecd * 2 * PI / max_ecd;
}

void DM_Motor::dm_mit_init()
{
    //ӳ�����ʼ��
    tmp.pmax = 3.1415926;    //λ�÷�Χ
    tmp.vmax = 45.0f;   //�ٶȷ�Χ
    tmp.tmax = 20.0f;   //���ط�Χ
    //����Ĳ�����׼ dm��λ���鿴
    //���Ʋ�����ʼ��
    ctrl.mode = MIT_MODE;
}

// void DM_Motor::dm_ps_init()
// {
//     //ӳ�����ʼ��
//     tmp.pmax = 3.14;    //λ�÷�Χ
//     tmp.vmax = 45.0f;   //�ٶȷ�Χ
//     tmp.tmax = 20.0f;   //���ط�Χ
//     //���Ʋ�����ʼ��
//     ctrl.mode = PS_MODE;
// }

void DJI_Motor::update()
{
    
    this->speed = motor_measure->speed_rpm * DJI_RPM_TO_RAD;
    this->encode_angle = ecd_to_angle(motor_measure->ecd);
}
 

fp32 DM_Motor::uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

void DM_Motor::update()
{
    //this->gyro_angle = gyro_angle_measure->yaw * DM_DEG_TO_RAD;
    this->speed = uint_to_float(motor_measure->v_int, -tmp.vmax, tmp.vmax, 12);
    this->encode_angle = uint_to_float(motor_measure->p_int, -tmp.pmax, tmp.pmax, 16);
}
