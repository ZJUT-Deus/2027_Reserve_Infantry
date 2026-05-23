#include "Board.h"




// void Board::init(UART_HandleTypeDef *huart,uint8_t *Rx_buf,uint16_t Rx_buf_size){
//     this->Rx_Buffer = Rx_buf;
//     this->Rx_Buffer_Size = Rx_buf_size;
//     UART_Init(huart, this->Rx_Buffer, this->Rx_Buffer_Size);
// }

void Board::init()
{
    UART_Init(BOARD_UART, this->Rx_Buffer, UP_BOARD_RX_BUFFER_SIZE);
}

void Board::send()
{
     // ��VT13ң�������ݴ�������ͻ�����?
    // ֡ͷ
    this->Tx_Buffer[0] = 0xb9;
    this->Tx_Buffer[1] = 0x54;
    
    // ���ң����ͨ������? (����VT13���ݸ�ʽ)
    // ͨ��0
    this->Tx_Buffer[2] = vt13.vt13_rc_ctrl.rc.ch[0] & 0xFF;
    this->Tx_Buffer[3] = (vt13.vt13_rc_ctrl.rc.ch[0] >> 8) & 0xFF;
    
    // ͨ��1
    this->Tx_Buffer[4] = vt13.vt13_rc_ctrl.rc.ch[1] & 0xFF;
    this->Tx_Buffer[5] = (vt13.vt13_rc_ctrl.rc.ch[1] >> 8) & 0xFF;
    
    // ͨ��2
    this->Tx_Buffer[6] = vt13.vt13_rc_ctrl.rc.ch[2] & 0xFF;
    this->Tx_Buffer[7] = (vt13.vt13_rc_ctrl.rc.ch[2] >> 8) & 0xFF;
    
    // ͨ��3
    this->Tx_Buffer[8] = vt13.vt13_rc_ctrl.rc.ch[3] & 0xFF;
    this->Tx_Buffer[9] = (vt13.vt13_rc_ctrl.rc.ch[3] >> 8) & 0xFF;
    
    // ģʽ����
    this->Tx_Buffer[10] = vt13.vt13_rc_ctrl.rc.mode_sw & 0xFF;
    
    // ֹͣ��ť
    this->Tx_Buffer[11] = vt13.vt13_rc_ctrl.rc.stop & 0xFF;
    
    // ��ť
    this->Tx_Buffer[12] = vt13.vt13_rc_ctrl.rc.left_button & 0xFF;
    
    // �Ұ�ť
    this->Tx_Buffer[13] = vt13.vt13_rc_ctrl.rc.right_button & 0xFF;
    
    // ����
    this->Tx_Buffer[14] = vt13.vt13_rc_ctrl.rc.wheel & 0xFF;
    this->Tx_Buffer[15] = (vt13.vt13_rc_ctrl.rc.wheel >> 8) & 0xFF;
    
    // ���?
    this->Tx_Buffer[16] =vt13.vt13_rc_ctrl.rc.shutter & 0xFF ;

    //this->Tx_Buffer[17] = gimbal.gimbal_yaw_motor.encode_angle ; 
    memcpy(&this->Tx_Buffer[17], &gimbal.gimbal_yaw_motor.encode_angle, 4);
    
    // ��֤CRC16У��
    //append_CRC16_check_sum(this->Tx_Buffer, 18);

    UART_Send_Data(BOARD_UART,this->Tx_Buffer, 21);
}

void Board::unpack()
{
    
}





