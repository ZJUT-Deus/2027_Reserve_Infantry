#ifndef VT13_H
#define VT13_H

#include "struct_typedef.h"
#include "main.h"
#include "bsp_uart.h"
#include "CRC8_CRC16.h"

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define VT13_HEADER_SIZE        sizeof(VT13_RC_ctrl_t)
#define UART_VT13               UART7
#define VT13_UART               (&huart7)
#define VT13_RX_BUFFER_SIZE     21

/* 键盘按键偏移定义 */
#define KEY_PRESSED_OFFSET_W    ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S    ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A    ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D    ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q    ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E    ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R    ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F    ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G    ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z    ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X    ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C    ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V    ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B    ((uint16_t)1 << 15)

/* 当前帧键盘按键状态检测 */
#define IF_KEY_PRESSED_VT13         (vt13.vt13_rc_ctrl.key.v)
#define IF_KEY_PRESSED_W_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)     != 0)
#define IF_KEY_PRESSED_S_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)     != 0)
#define IF_KEY_PRESSED_A_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)     != 0)
#define IF_KEY_PRESSED_D_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)     != 0)
#define IF_KEY_PRESSED_Q_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)     != 0)
#define IF_KEY_PRESSED_E_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)     != 0)
#define IF_KEY_PRESSED_G_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)     != 0)
#define IF_KEY_PRESSED_X_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)     != 0)
#define IF_KEY_PRESSED_Z_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)     != 0)
#define IF_KEY_PRESSED_C_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)     != 0)
#define IF_KEY_PRESSED_B_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)     != 0)
#define IF_KEY_PRESSED_V_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)     != 0)
#define IF_KEY_PRESSED_F_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)     != 0)
#define IF_KEY_PRESSED_R_VT13       ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)     != 0)
#define IF_KEY_PRESSED_CTRL_VT13    ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)  != 0)
#define IF_KEY_PRESSED_SHIFT_VT13   ((vt13.vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0)

/*上一帧键盘按键状态检测 */
#define IF_KEY_PRESSED_VT13_LAST         (vt13.last_vt13_rc_ctrl.key.v)
#define IF_KEY_PRESSED_W_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)     != 0)
#define IF_KEY_PRESSED_S_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)     != 0)
#define IF_KEY_PRESSED_A_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)     != 0)
#define IF_KEY_PRESSED_D_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)     != 0)
#define IF_KEY_PRESSED_Q_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)     != 0)
#define IF_KEY_PRESSED_E_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)     != 0)
#define IF_KEY_PRESSED_G_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)     != 0)
#define IF_KEY_PRESSED_X_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)     != 0)
#define IF_KEY_PRESSED_Z_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)     != 0)
#define IF_KEY_PRESSED_C_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)     != 0)
#define IF_KEY_PRESSED_B_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)     != 0)
#define IF_KEY_PRESSED_V_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)     != 0)
#define IF_KEY_PRESSED_F_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)     != 0)
#define IF_KEY_PRESSED_R_VT13_LAST       ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)     != 0)
#define IF_KEY_PRESSED_CTRL_VT13_LAST    ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)  != 0)
#define IF_KEY_PRESSED_SHIFT_VT13_LAST   ((vt13.last_vt13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0)

/* 鼠标按键状态检测*/
#define IF_MOUSE_PRESSED_LEFT_VT13    (vt13.vt13_rc_ctrl.mouse.press_l == 1)
#define IF_MOUSE_PRESSED_RIGH_VT13    (vt13.vt13_rc_ctrl.mouse.press_r == 1)
#define IF_MOUSE_PRESSED_MID_VT13     (vt13.vt13_rc_ctrl.mouse.middle == 1)

/*上一帧鼠标按键状态检测*/
#define IF_MOUSE_PRESSED_LEFT_VT13_LAST    (vt13.last_vt13_rc_ctrl.mouse.press_l == 1)
#define IF_MOUSE_PRESSED_RIGH_VT13_LAST    (vt13.last_vt13_rc_ctrl.mouse.press_r == 1)
#define IF_MOUSE_PRESSED_MID_VT13_LAST     (vt13.last_vt13_rc_ctrl.mouse.middle == 1)

/* 获取鼠标移动速度 */
#define MOUSE_X_MOVE_SPEED_VT13    (vt13.vt13_rc_ctrl.mouse.x)
#define MOUSE_Y_MOVE_SPEED_VT13    (vt13.vt13_rc_ctrl.mouse.y)
#define MOUSE_Z_MOVE_SPEED_VT13    (vt13.vt13_rc_ctrl.mouse.z)


/* 通道值定义 */
#define RC_CH_VALUE_MAX 1684
#define RC_CH_VALUE_MIN 364

typedef __packed struct
{
        __packed struct
        {
                int16_t ch[4];     //通道0~3       
                uint8_t mode_sw;    //档位开关
                uint8_t stop;     //暂停按键
                uint8_t left_button;    //自定义左;
                uint8_t	right_button;   //自定义右;
                int16_t wheel;          //拨轮
                uint8_t shutter;        //扳机
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
                uint8_t middle;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;
        uint16_t crc16;

} VT13_RC_ctrl_t;


//extern uint8_t VT13_Rx_Buffer[VT13_RX_BUFFER_SIZE];

class VT13
{
public:
    VT13_RC_ctrl_t vt13_rc_ctrl;
    VT13_RC_ctrl_t last_vt13_rc_ctrl;
    
    // 数据接收
    uint8_t Rx_Buffer[VT13_RX_BUFFER_SIZE];
    uint16_t Rx_Buffer_Size;

    // 初始化
    void init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size);

    const VT13_RC_ctrl_t *get_vt13_remote_control_point();
    VT13_RC_ctrl_t *get_last_vt13_remote_control_point();

    // 解包函数
    void unpack();
};


#endif
