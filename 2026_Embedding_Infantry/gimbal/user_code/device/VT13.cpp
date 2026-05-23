#include "VT13.h"




void VT13::init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size)
{
    this->Rx_Buffer_Size = Rx_buf_size;
    UART_Init(huart, this->Rx_Buffer, this->Rx_Buffer_Size);
}

const VT13_RC_ctrl_t *VT13::get_vt13_remote_control_point()
{
    return &vt13_rc_ctrl;
}

VT13_RC_ctrl_t *VT13::get_last_vt13_remote_control_point()
{
    return &last_vt13_rc_ctrl;
}

void VT13::unpack()
{
    // 保留上一次遥控器值
    last_vt13_rc_ctrl = vt13_rc_ctrl;

    // 检查帧头和CRC校验
    if(Rx_Buffer[0] == 0xa9 && Rx_Buffer[1] == 0x53 && verify_CRC16_check_sum(Rx_Buffer, 21)) 
    {
        // 解析通道0值
        vt13_rc_ctrl.rc.ch[0] = (Rx_Buffer[2] | (Rx_Buffer[3] << 8)) & 0x07ff;
        // 解析通道1值
        vt13_rc_ctrl.rc.ch[1] = ((Rx_Buffer[3] >> 3) | (Rx_Buffer[4] << 5)) & 0x07ff;
        // 解析通道2值
        vt13_rc_ctrl.rc.ch[2] = ((Rx_Buffer[4] >> 6) | (Rx_Buffer[5] << 2) | (Rx_Buffer[6] << 10)) & 0x07ff;
        // 解析通道3值
        vt13_rc_ctrl.rc.ch[3] = ((Rx_Buffer[6] >> 1) | (Rx_Buffer[7] << 7)) & 0x07ff;
        // 解析模式开关值
        vt13_rc_ctrl.rc.mode_sw = ((Rx_Buffer[7] >> 4) & 0x0003); 
        // 解析停止按钮值
        vt13_rc_ctrl.rc.stop = ((Rx_Buffer[7] >> 6) & 0x01);
        // 解析左按钮值
        vt13_rc_ctrl.rc.left_button = ((Rx_Buffer[7] >> 7) & 0x01);
        // 解析右按钮值
        vt13_rc_ctrl.rc.right_button = ((Rx_Buffer[8] >> 0) & 0x01);
        // 解析轮子值
        vt13_rc_ctrl.rc.wheel = ((Rx_Buffer[8] >> 1) | (Rx_Buffer[9] << 7)) & 0x07FF;
        // 解析扳机值
        vt13_rc_ctrl.rc.shutter = (Rx_Buffer[9] >> 4) & 0x01;
        
        // 解析鼠标X轴值
        vt13_rc_ctrl.mouse.x = (Rx_Buffer[10] | (Rx_Buffer[11] << 8));
        // 解析鼠标Y轴值
        vt13_rc_ctrl.mouse.y = (Rx_Buffer[12] | (Rx_Buffer[13] << 8));
        // 解析鼠标Z轴值
        vt13_rc_ctrl.mouse.z = (Rx_Buffer[14] | (Rx_Buffer[15] << 8));
        
        // 解析鼠标左键状态
        vt13_rc_ctrl.mouse.press_l = (Rx_Buffer[16] >> 0) & 0x03;
        // 解析鼠标右键状态
        vt13_rc_ctrl.mouse.press_r = (Rx_Buffer[16] >> 2) & 0x03;
        // 解析鼠标中键状态
        vt13_rc_ctrl.mouse.middle = (Rx_Buffer[16] >> 4) & 0x03;
        
        // 解析键盘值
        vt13_rc_ctrl.key.v = (Rx_Buffer[17] | (Rx_Buffer[18] << 8));
        
        // 解析CRC16校验值
        vt13_rc_ctrl.crc16 = (Rx_Buffer[19] | (Rx_Buffer[20] << 8));
        
        // 对各通道值和轮子值减去偏移量
        vt13_rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
        vt13_rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
        vt13_rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
        vt13_rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
        vt13_rc_ctrl.rc.wheel -= RC_CH_VALUE_OFFSET;
    }
}
