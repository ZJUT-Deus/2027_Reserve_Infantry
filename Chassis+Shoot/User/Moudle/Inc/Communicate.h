/**
 * @file    Communicate.h
 * @brief   底盘 CAN 通信模块声明 (数据收发与回调)
 * @author  kk
 * @date    2026-05-25
 */

#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "struct_typedef.h"
#include "bsp_fdcan.h"
#include "I6X.h"
#include "Can_receive.h"
#include "Chassis.h"
#include "Shoot.h"

/** @brief 底盘通信管理类 (CAN 收发 + 遥控器数据处理) */
class Communicate
{
public:
    void init();
    void handle_rc();

private:
    static fp32 map_speed(fp32 rc_val, fp32 rc_max, fp32 chassis_max);
};

extern Can_receive can_receive;
extern Communicate communicate;
extern Chassis chassis;

void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
uint8_t communicate_send(uint16_t id, uint8_t *data, uint32_t len);

#endif
