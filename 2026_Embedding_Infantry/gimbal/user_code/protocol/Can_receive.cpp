#include "can_receive.h"





void Can_receive::init()
{
    can_bsp_init();
}

void Can_receive::can_cmd_super_cap(uint8_t enableDCDC, uint8_t systemRestart, uint16_t feedbackRefereePowerLimit, uint16_t feedbackRefereeEnergyBuffer, uint32_t ID)
{
// 字节0: 位域设置
can_send_data[0] = (enableDCDC & 0x01) | ((systemRestart & 0x01) << 1);

// 字节1-2: feedbackRefereePowerLimit (小端序：低字节在前)
can_send_data[1] = feedbackRefereePowerLimit & 0xFF;        // 低字节
can_send_data[2] = (feedbackRefereePowerLimit >> 8) & 0xFF; // 高字节

// 字节3-4: feedbackRefereeEnergyBuffer (小端序：低字节在前)
can_send_data[3] = feedbackRefereeEnergyBuffer & 0xFF;        // 低字节
can_send_data[4] = (feedbackRefereeEnergyBuffer >> 8) & 0xFF; // 高字节

// 字节5-7: 保留字节
can_send_data[5] = 0;
can_send_data[6] = 0;
can_send_data[7] = 0;

// 确保使用正确的CAN ID: 0x061
fdcanx_send_data(&CHASSIS_CAN, ID, can_send_data, sizeof(can_send_data));
}

void Can_receive::can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger,uint16_t ID)
{

    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = trigger >> 8;
    can_send_data[5] = trigger;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    fdcanx_send_data(&SHOOT_CAN, ID, can_send_data,sizeof(can_send_data));
}


/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      current1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      current2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      current3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      current4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void Can_receive::can_cmd_dji_motor(int16_t current1, int16_t current2, int16_t current3, int16_t current4,uint16_t ID)
{
    
    can_send_data[0] = current1 >> 8;
    can_send_data[1] = current1;
    can_send_data[2] = current2 >> 8;
    can_send_data[3] = current2;
    can_send_data[4] = current3 >> 8;
    can_send_data[5] = current3;
    can_send_data[6] = current4 >> 8;
    can_send_data[7] = current4;
    fdcanx_send_data(&CHASSIS_CAN, ID, can_send_data, sizeof(can_send_data));
}

void Can_receive::send_ui_board_com(uint8_t _right_button,uint8_t _top_switch,float _distance,uint16_t ID){
	// 将 float 的内存数据强制转换为 32 位无符号整数以便进行移位操作
	uint32_t distance_bits = *(uint32_t*)&_distance;

	can_send_data[0] = _right_button;
	can_send_data[1] = _top_switch;
	can_send_data[2] = (uint8_t)(distance_bits >> 24);
	can_send_data[3] = (uint8_t)(distance_bits >> 16);
	can_send_data[4] = (uint8_t)(distance_bits >> 8);
	can_send_data[5] = (uint8_t)distance_bits;
	can_send_data[6] = 0;
	can_send_data[7] = 0;
	fdcanx_send_data(&CHASSIS_CAN, ID, can_send_data, sizeof(can_send_data));
}

/**
 * @brief          DM电机电流接收(云台上为yaw轴和pitch轴电机)
 * @param[out]     dm_motor: 指向电机数据的指针
 */
void Can_receive::get_dm_motor_measure(dm_motor_measure_t *dm_motor, uint8_t data[8])
{
    dm_motor->id = (data[0])&0x0F;
	dm_motor->state = (data[0])>>4;
	dm_motor->p_int=(data[1]<<8)|data[2];
	dm_motor->v_int=(data[3]<<4)|(data[4]>>4);
	dm_motor->t_int=((data[4]&0xF)<<8)|data[5];
	dm_motor->Tmos = (float)(data[6]);
	dm_motor->Tcoil = (float)(data[7]);
}
/**
 * @brief          DJI电机电流接收
 * @param[out]     dji_motor: 指向电机数据的指针
 */
void Can_receive::get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8])
{

    dji_motor->last_ecd = dji_motor->ecd;
    dji_motor->ecd = (uint16_t)(data[0] << 8 | data[1]);
    dji_motor->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    dji_motor->given_current = (int16_t)(data[4] << 8 | data[5]);
    dji_motor->temperate = data[6];

}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (int16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
}


//---------------------------达妙电机控制相关函数 start----------------------//
//    发送控制结构电机（yaw轴或Pitch轴电机）电流(直接使用mit_ctrl函数即可)
void Can_receive::mit_ctrl(fp32 pos,fp32 kp, fp32 kd, fp32 vel, fp32 tor, uint16_t id,uint16_t mode,esc_inf_t tmp)//mit模式下的控制
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint16_t ID = id + mode;

    pos_tmp = float_to_uint(pos,-tmp.pmax,tmp.pmax,16);
	vel_tmp = float_to_uint(vel,-tmp.vmax,tmp.vmax,12);
	tor_tmp = float_to_uint(tor,-tmp.tmax,tmp.tmax,12);
	kp_tmp  = float_to_uint(kp,  0, 500, 12);
	kd_tmp  = float_to_uint(kd,  0, 5, 12);

    can_send_data[0] = (pos_tmp >> 8);
	can_send_data[1] = pos_tmp;
	can_send_data[2] = (vel_tmp >> 4);
	can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	can_send_data[4] = kp_tmp;
	can_send_data[5] = (kd_tmp >> 4);
	can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	can_send_data[7] = tor_tmp;

    fdcanx_send_data(&GIMBAL_CAN, ID , can_send_data, sizeof(can_send_data));
}

void Can_receive::ps_ctrl(float _pos, float _vel,uint16_t id, uint16_t mode_id)
{
	uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

	uint16_t ID = id + mode_id;

	can_send_data[0] =*pbuf;
	can_send_data[1] = *(pbuf+1);
	can_send_data[2] = *(pbuf+2);
	can_send_data[3] = *(pbuf+3);
	can_send_data[4] = *vbuf;
	can_send_data[5] =*(vbuf+1);
	can_send_data[6] = *(vbuf+2);
	can_send_data[7] = *(vbuf+3);

	fdcanx_send_data(&GIMBAL_CAN, ID , can_send_data, sizeof(can_send_data));
}

int Can_receive::float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits)
{
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return (int) ((x_float-offset)*((fp32)((1<<bits)-1))/span);
}

fp32 Can_receive::uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

void Can_receive::ENABLE_DM_Motor_mode(uint16_t id,uint16_t mode)
{
    uint16_t ID =id+mode;
    
	can_send_data[0] = 0xFF;
	can_send_data[1] = 0xFF;
	can_send_data[2] = 0xFF;
	can_send_data[3] = 0xFF;
	can_send_data[4] = 0xFF;
	can_send_data[5] = 0xFF;
	can_send_data[6] = 0xFF;
	can_send_data[7] = 0xFC;

    fdcanx_send_data(&GIMBAL_CAN, ID , can_send_data, sizeof(can_send_data));
}

void Can_receive::DISABLE_DM_Motor_mode(uint16_t id,uint16_t mode)
{
    uint16_t ID =id+mode;
    
	can_send_data[0] = 0xFF;
	can_send_data[1] = 0xFF;
	can_send_data[2] = 0xFF;
	can_send_data[3] = 0xFF;
	can_send_data[4] = 0xFF;
	can_send_data[5] = 0xFF;
	can_send_data[6] = 0xFF;
	can_send_data[7] = 0xFD;

    fdcanx_send_data(&GIMBAL_CAN, ID , can_send_data, sizeof(can_send_data));
}
// ---------------------------达妙电机控制相关函数 end ----------------------//

//返回云台控制电机 DM电机数据指针
const dm_motor_measure_t *Can_receive::get_dm_motor_measure_point(uint8_t i)
{
    return &gimbal_dm_motor[i];
}

//返回云台发射电机 DJI电机数据指针
const dji_motor_measure_t *Can_receive::get_dji_motor_measure_point(uint8_t i)
{
   return &shoot_motor[i];
}

// 返回底盘动力电机 DJI电机数据指针
const dji_motor_measure_t *Can_receive::get_motor_chassis_measure_point(uint8_t i)
{
    return &chassis_motive_motor[i];
}
