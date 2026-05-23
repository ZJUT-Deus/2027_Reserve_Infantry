#ifndef I6X_H
#define I6X_H

#include "struct_typedef.h"
#include "main.h"
#include "bsp_uart.h"

#define I6X_FRAME_LENGTH        25
#define I6X_RX_BUFFER_SIZE      I6X_FRAME_LENGTH

// iA6B 接收机 SBUS 推荐使用 100000 8E2，对应工程里通常可以放在 UART5。
#define UART_I6X                UART5
#define I6X_UART                (&huart5)

// 摇杆通道原始值大约为 -784 ~ 783，这里按文档映射到 -660 ~ 660，方便复用旧工程控制参数。
#define I6X_CH_VALUE_MAX        660
#define I6X_CH_VALUE_MIN       -660

// 拨杆统一成三态：上 1，中 0，下 -1。
#define I6X_SW_UP               ((int8_t)1)
#define I6X_SW_MID              ((int8_t)0)
#define I6X_SW_DOWN             ((int8_t)-1)

#define i6x_switch_is_down(s)   ((s) == I6X_SW_DOWN)
#define i6x_switch_is_mid(s)    ((s) == I6X_SW_MID)
#define i6x_switch_is_up(s)     ((s) == I6X_SW_UP)

// 为了兼容旧项目里 switch_is_down/up/mid 的写法，也可以直接用下面三个宏。
#define switch_is_down(x)       ((x) == I6X_SW_DOWN)
#define switch_is_mid(x)        ((x) == I6X_SW_MID)
#define switch_is_up(x)         ((x) == I6X_SW_UP)

// 当前帧拨杆状态。
#define IF_I6X_SW0_UP           i6x_switch_is_up(i6x.i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW0_MID          i6x_switch_is_mid(i6x.i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW0_DOWN         i6x_switch_is_down(i6x.i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW1_UP           i6x_switch_is_up(i6x.i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW1_MID          i6x_switch_is_mid(i6x.i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW1_DOWN         i6x_switch_is_down(i6x.i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW2_UP           i6x_switch_is_up(i6x.i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_MID          i6x_switch_is_mid(i6x.i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_DOWN         i6x_switch_is_down(i6x.i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW3_UP           i6x_switch_is_up(i6x.i6x_rc_ctrl.rc.s[3])
#define IF_I6X_SW3_MID          i6x_switch_is_mid(i6x.i6x_rc_ctrl.rc.s[3])
#define IF_I6X_SW3_DOWN         i6x_switch_is_down(i6x.i6x_rc_ctrl.rc.s[3])

// 上一帧拨杆状态，用于做边沿触发。
#define IF_I6X_SW0_UP_LAST      i6x_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW0_MID_LAST     i6x_switch_is_mid(i6x.last_i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW0_DOWN_LAST    i6x_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[0])
#define IF_I6X_SW1_UP_LAST      i6x_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW1_MID_LAST     i6x_switch_is_mid(i6x.last_i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW1_DOWN_LAST    i6x_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[1])
#define IF_I6X_SW2_UP_LAST      i6x_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_MID_LAST     i6x_switch_is_mid(i6x.last_i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW2_DOWN_LAST    i6x_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[2])
#define IF_I6X_SW3_UP_LAST      i6x_switch_is_up(i6x.last_i6x_rc_ctrl.rc.s[3])
#define IF_I6X_SW3_MID_LAST     i6x_switch_is_mid(i6x.last_i6x_rc_ctrl.rc.s[3])
#define IF_I6X_SW3_DOWN_LAST    i6x_switch_is_down(i6x.last_i6x_rc_ctrl.rc.s[3])

// 遥控器到底盘命令的映射配置。后续换操作手习惯时优先改这里，不要在底盘代码里散写通道编号。
#define I6X_CHASSIS_VX_CH       0
#define I6X_CHASSIS_VY_CH       1
#define I6X_CHASSIS_WZ_CH       2

#define I6X_CHASSIS_MODE_SW     0

#define I6X_RC_DEADBAND         20
#define I6X_REMOTE_TIMEOUT_MS   100

// 新麦轮车的默认速度上限，实车调试时按机械和电机能力修改。
#define I6X_CHASSIS_MAX_VX      3.0f
#define I6X_CHASSIS_MAX_VY      3.0f
#define I6X_CHASSIS_MAX_WZ      6.0f

typedef enum
{
        I6X_CHASSIS_ZERO_FORCE = 0,     // 无力模式。
        I6X_CHASSIS_FREE,               // 自由运动模式。
        I6X_CHASSIS_TOP,                // 小陀螺模式。
} i6x_chassis_mode_e;

typedef __packed struct
{
        __packed struct
        {
                int16_t ch[6];         // 通道 0~5：四个摇杆 + 两个旋钮，范围约 -660 ~ 660。
                int8_t s[4];           // 拨杆 0~3：上 1，中 0，下 -1。
                uint8_t frame_lost;    // SBUS 丢帧标志。
                uint8_t failsafe;      // SBUS 失控保护标志。
        } rc;

        uint32_t last_update_ms;        // 最近一次有效帧时间，用于在线判断。
        uint8_t online;                 // 在线状态。

} I6X_RC_ctrl_t;

typedef __packed struct
{
        fp32 vx;
        fp32 vy;
        fp32 wz;
        uint8_t enable;
        uint8_t mode;
} I6X_Chassis_cmd_t;

class I6X
{
public:
    I6X_RC_ctrl_t i6x_rc_ctrl;
    I6X_RC_ctrl_t last_i6x_rc_ctrl;
    I6X_Chassis_cmd_t chassis_cmd;

    uint8_t Rx_Buffer[I6X_RX_BUFFER_SIZE];
    uint16_t Rx_Buffer_Size;

    void init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size);

    const I6X_RC_ctrl_t *get_i6x_remote_control_point();
    I6X_RC_ctrl_t *get_last_i6x_remote_control_point();

    void unpack(uint32_t now_ms);
    void update_online(uint32_t now_ms);
    void update_command();
};

extern I6X i6x;

#endif
