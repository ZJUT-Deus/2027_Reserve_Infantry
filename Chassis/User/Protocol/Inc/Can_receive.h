/**
 * @file    Can_receive.h
 * @brief   CAN 总线数据接收与电机控制 (DM3519 协议)
 * @author  kk
 * @date    2026-05-22
 */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"
#include "bsp_fdcan.h"
#include "Motor.h"
#include "user_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 底盘 CAN 总线句柄 */
#define CHASSIS_CAN hfdcan1
/** @brief 电机使能 */
#define ENABLE  0x01
/** @brief 电机失能 */
#define DISABLE 0x00

/** @brief 底盘电机 CAN ID 枚举 */
typedef enum
{
    CHASSIS_FR_CAN_ID = 0x01,   /**< 右前轮 CAN ID */
    CHASSIS_FL_CAN_ID = 0x02,   /**< 左前轮 CAN ID */
    CHASSIS_BR_CAN_ID = 0x03,   /**< 右后轮 CAN ID */
    CHASSIS_BL_CAN_ID = 0x04,   /**< 左后轮 CAN ID */
} motor_can_id_t;

/** @brief 底盘电机 Master ID (命令字) 枚举 */
typedef enum
{
    CHASSIS_FR_MST_ID = 0x11,   /**< 右前轮 Master ID */
    CHASSIS_FL_MST_ID = 0x12,   /**< 左前轮 Master ID */
    CHASSIS_BR_MST_ID = 0x13,   /**< 右后轮 Master ID */
    CHASSIS_BL_MST_ID = 0x14,   /**< 左后轮 Master ID */
} motor_mst_id_t;

#ifdef __cplusplus
}
#endif

/** @brief CAN 接收与电机控制协议类 */
class Can_receive
{
public:
    dm_motor_measure_t chassis_motor_measure[4];    /**< 四轮电机反馈数据 */
    uint8_t can_send_data[8];                       /**< CAN 发送缓冲区 */

    void get_chassis_motor_measure(dm_motor_measure_t *motor, uint8_t data[8]);
    const dm_motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
    void can_cmd_mit_dm_motor(fp32 pos, fp32 vel, fp32 Kp, fp32 Kd, fp32 tor, uint16_t id, esc_inf_t tmp);
    void CTRL_DM3519(uint16_t id, uint16_t state);
};

extern Can_receive can_receive;

#endif
