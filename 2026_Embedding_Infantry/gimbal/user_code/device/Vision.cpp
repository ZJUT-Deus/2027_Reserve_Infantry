#include "Vision.h"

#include <cstring>

extern IMU imu;

void Vision::clear_lock_qualification()
{
    rt.lock_enter_count = 0U;
    rt.lock_exit_count = 0U;
    rt.fire_qualified = 0U;
}

void Vision::reset_runtime_state(uint8_t clear_last_rx_tick)
{
    if (clear_last_rx_tick != 0U)
    {
        rt.last_rx_tick = 0U;
    }

    rt.fresh = 0U;
    rt.consumed = 1U;
    rt.timeout = 0U;
    rt.invalid_frame = 0U;
    rt.target_lost_count = 0U;
    memset(&yaw_release_rt, 0, sizeof(yaw_release_rt));
    clear_lock_qualification();
    rt.state = VISION_DISABLED;
}

void Vision::init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size)
{
    this->Rx_Buffer_Size = Rx_buf_size;
    UART_Init(huart, this->Rx_Buffer, this->Rx_Buffer_Size);

    memset(&vision_recv_data, 0, sizeof(vision_recv_data));
    memset(&vision_send_data, 0, sizeof(vision_send_data));

    reset_runtime_state(1U);

    delta_param.vision_timeout_ms = 60U;
    delta_param.lock_enter_threshold = 2U;
    delta_param.lock_exit_threshold = 1U;
    delta_param.target_lost_threshold = 1U;
//平移无预测
    // delta_param.k_yaw = 0.1299998f;
    // delta_param.k_pitch = 0.103000f;
    // delta_param.yaw_step_max = 0.299989f;
    // delta_param.pitch_step_max = 0.132998f;
//仅仅旋转无预测 且自身不小陀螺
    delta_param.k_yaw = 0.5120000f;
    delta_param.k_pitch = 0.4439999f;
    delta_param.yaw_step_max = 1.4813299f;
    delta_param.pitch_step_max = 1.432999f;
}

uint8_t vision_send_pack[50] = {0};

void Vision::send()
{
    int i; //循环发送次数
    uint16_t shoot_speed_limit;
    uint16_t bullet_speed;
    //get_shooter_shoot_speed_limit_and_bullet_speed(&shoot_speed_limit, &bullet_speed);
  
    this->vision_send_data.BEGIN = VISION_BEGIN;
  
    this->vision_send_data.CmdID = 0x01; //颜色
    this->vision_send_data.speed = referee.initial_speed;  
    this->vision_send_data.yaw = ANGLE_TO_RAD*imu.g_output_info.yaw;
    this->vision_send_data.pitch = ANGLE_TO_RAD*imu.g_output_info.pitch;
    this->vision_send_data.roll = ANGLE_TO_RAD*imu.g_output_info.roll;
    this->vision_send_data.angle_x = imu.g_output_info.angle_x;
    this->vision_send_data.angle_y = imu.g_output_info.angle_y;
    this->vision_send_data.angle_z = imu.g_output_info.angle_z;
    this->vision_send_data.END = 0xFF;
  
    memcpy(vision_send_pack, &vision_send_data, VISION_SEND_LEN_PACKED);

  //通过DMA发送数据，避免阻塞CPU
   //HAL_UART_Transmit_DMA(VISION_UART, vision_send_pack, VISION_SEND_LEN_PACKED);
  
    //将打包好的数据通过串口移位发送到上位机
				for (i = 0; i < VISION_SEND_LEN_PACKED; i++)
      {
           HAL_UART_Transmit(VISION_UART, &vision_send_pack[i], sizeof(vision_send_pack[0]), 0xFFF);
       }
    
  
    memset(vision_send_pack, 0, 50);
    }
void Vision::unpack()
{ 
//判断帧头数据是否为0xA5
if (Rx_Buffer[0] == VISION_BEGIN)
{
  //判断帧尾数据是否为0xff
  if (Rx_Buffer[VISION_READ_LEN_PACKED - 1] == VISION_END)
  {

    //接收数据拷贝
    memcpy(&(this->vision_recv_data), Rx_Buffer, VISION_READ_LEN_PACKED);

    rt.last_rx_tick = HAL_GetTick();
    rt.fresh = 1U;
    rt.consumed = 0U;
    rt.timeout = 0U;
    rt.invalid_frame = 0U;


    // //帧计算
    // Vision_Time_Test[NOW] = xTaskGetTickCount();
    // Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
    // Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
  }
}
}

void Vision::vision_get_angle(fp32 *yaw, fp32 *pitch)
{
    *yaw = this->vision_recv_data.yaw_angle;
    *pitch = this->vision_recv_data.pitch_angle;
}

vision_led_state_e Vision::get_led_state() const
{
    if (rt.invalid_frame != 0U || rt.timeout != 0U || rt.state == VISION_LOST_HOLD)
    {
        return VISION_LED_ERROR;
    }

    if (rt.state == VISION_TRACK_LOCKED)
    {
        return VISION_LED_LOCKED;
    }

    if (rt.state == VISION_TRACK_UNLOCKED)
    {
        return VISION_LED_TRACKING;
    }

    if (rt.state == VISION_ACQUIRE)
    {
        return VISION_LED_SEARCHING;
    }

    return VISION_LED_OFF;
}

bool_t Vision::vision_if_find_target()
{
	return vision_recv_data.identify_target;
}
