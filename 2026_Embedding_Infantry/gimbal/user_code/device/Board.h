#ifndef BOARD_H
#define BOARD_H

#include "VT13.h"
#include "main.h"
#include "ws2812.h"
#include "gimbal.h"
#include "bsp_uart.h"
#include "CRC8_CRC16.h"
#include "struct_typedef.h"

#define UART_BOARD USART2
#define DOWN_BOARD_RX_BUFFER_SIZE 21
#define UP_BOARD_RX_BUFFER_SIZE 21
#define DOWN_BOARD_TX_BUFFER_SIZE 21
#define UP_BOARD_TX_BUFFER_SIZE 21
#define BOARD_UART (&huart2)

// 声明外部VT13对象

extern VT13 vt13;


//云台发送数据结构体
typedef struct{
    //云台状态
    uint8_t s0;
    uint8_t gimbal_behaviour;
    float gimbal_yaw_angle;
    float gimbal_pitch_angle;
    bool_t auto_state;
    bool_t aim_state;
    bool_t fric_state;

} gimbal_send_t;



//云台接收数据结构体
typedef struct{
    uint8_t color;                //判断红蓝方
    uint8_t robot_id;             //机器人编号
    uint8_t level;
    //测速速度及底盘模式
    uint16_t shooter_barrel_heat_limit;//17mm测速热量上限
    uint16_t shooter_17mm_1_barrel_heat; //1号17mm测速实时热量
    fp32 bullet_speed;      // shoot测速实时射速

    uint8_t chassis_behaviour;
    uint8_t game_progress;

} gimbal_receive_t;



class Board
{
public:
    
    gimbal_send_t gimbal_data;
    gimbal_receive_t chassis_data;
    //数据接收
    //数据接收
    uint8_t Rx_Buffer[UP_BOARD_RX_BUFFER_SIZE];
    uint8_t Tx_Buffer[UP_BOARD_TX_BUFFER_SIZE];
    // uint8_t *Rx_Buffer;
    // uint16_t Rx_Buffer_Size;
    
    //void init(UART_HandleTypeDef *huart,uint8_t *Rx_buf,uint16_t Rx_buf_size);
    void init();
    void send();
    void unpack();
   
};

#endif

