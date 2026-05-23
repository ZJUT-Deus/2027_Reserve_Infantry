#include "Communicate.h"
#include "Referee.h"
#include "chassis.h"

#include "simple_ref_test.h"

#include <cstring>

//接收中断调试标志

IMU imu;
VT13 vt13;
Vision vision;
Communicate communicate;
Can_receive can_receive;
Serialplot serialplot;

enum
{
	WS2812_LED_TOP = 0U,
	WS2812_LED_AUTO = 5U,
	WS2812_LED_VISION = 7U,
};

static uint8_t ws2812_last_frame[WS2812_NUM_LEDS * 3] = {0};
static uint8_t ws2812_last_frame_valid = 0U;

static void referee_can_assembly_reset(void);
static void ws2812_frame_set_rgb(uint8_t frame[WS2812_NUM_LEDS * 3], uint16_t index, uint8_t r, uint8_t g, uint8_t b);
static uint8_t ws2812_vision_led_allowed(gimbal_mode_e gimbal_mode, chassis_behaviour_e chassis_mode, bool_t auto_state);
static void ws2812_render_status_frame(uint8_t frame[WS2812_NUM_LEDS * 3]);
static void ws2812_apply_status_frame(const uint8_t frame[WS2812_NUM_LEDS * 3]);

static void referee_can_assembly_reset(void);
static uint8_t referee_rx_data[13] = {0};
static uint8_t referee_frame0_ready = 0U;   // 已收到 0x103，正在等待 0x104 补齐后半帧
static uint32_t referee_frame0_tick_ms = 0U; // 记录 0x103 到达时刻，用于拼帧超时保护

bool_t top_switch = TOP_SWITCH_DEFAULT;//小陀螺模式开关定义
bool_t is_sweeping_360 = false;//扫敌模式开关定义
bool_t auto_switch = false;//自瞄模式开关定义

static void ws2812_frame_set_rgb(uint8_t frame[WS2812_NUM_LEDS * 3], uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
	if (index >= WS2812_NUM_LEDS)
	{
		return;
	}

	frame[index * 3 + 0] = g;
	frame[index * 3 + 1] = r;
	frame[index * 3 + 2] = b;
}

static uint8_t ws2812_vision_led_allowed(gimbal_mode_e gimbal_mode, chassis_behaviour_e chassis_mode, bool_t auto_state)
{
	return (auto_state == TRUE && gimbal_mode == GIMBAL_TOP && chassis_mode == CHASSIS_TOP) ? 1U : 0U;
}

static void ws2812_render_status_frame(uint8_t frame[WS2812_NUM_LEDS * 3])
{
	const chassis_behaviour_e chassis_mode = chassis.chassis_behaviour_mode;
	const gimbal_mode_e gimbal_mode = (gimbal_mode_e)gimbal.gimbal_mode;
	const bool_t top_switch_state = top_switch;
	const bool_t auto_switch_state = auto_switch;
	const vision_led_state_e vision_led_state = vision.get_led_state();

	std::memset(frame, 0, WS2812_NUM_LEDS * 3);

	if (chassis_mode == CHASSIS_TOP)
	{
		if (top_switch_state == TRUE)
		{
			ws2812_frame_set_rgb(frame, WS2812_LED_TOP, 0U, 255U, 0U);
		}
		else
		{
			ws2812_frame_set_rgb(frame, WS2812_LED_TOP, 255U, 0U, 0U);
		}
	}

	if (gimbal_mode == GIMBAL_TOP)
	{
		if (auto_switch_state == TRUE)
		{
			ws2812_frame_set_rgb(frame, WS2812_LED_AUTO, 0U, 255U, 0U);
		}
		else
		{
			ws2812_frame_set_rgb(frame, WS2812_LED_AUTO, 255U, 0U, 0U);
		}
	}

	if (ws2812_vision_led_allowed(gimbal_mode, chassis_mode, auto_switch_state) == 0U)
	{
		return;
	}

	switch (vision_led_state)
	{
	case VISION_LED_ERROR:
		ws2812_frame_set_rgb(frame, WS2812_LED_VISION, 255U, 0U, 0U);
		break;
	case VISION_LED_SEARCHING:
		ws2812_frame_set_rgb(frame, WS2812_LED_VISION, 255U, 255U, 0U);
		break;
	case VISION_LED_TRACKING:
		ws2812_frame_set_rgb(frame, WS2812_LED_VISION, 0U, 0U, 255U);
		break;
	case VISION_LED_LOCKED:
		ws2812_frame_set_rgb(frame, WS2812_LED_VISION, 0U, 255U, 0U);
		break;
	case VISION_LED_OFF:
	default:
		break;
	}
}

static void ws2812_apply_status_frame(const uint8_t frame[WS2812_NUM_LEDS * 3])
{
	if (ws2812_last_frame_valid != 0U &&
		std::memcmp(frame, ws2812_last_frame, sizeof(ws2812_last_frame)) == 0)
	{
		return;
	}

	std::memcpy(ws2812.buffer, frame, sizeof(ws2812.buffer));
	WS2812_Show(&ws2812);
	std::memcpy(ws2812_last_frame, frame, sizeof(ws2812_last_frame));
	ws2812_last_frame_valid = 1U;
}

//uint32_t  bus_off_recovery_count = 0;
void Communicate::init()
{
	BSP_Buzzer_Init();
	WS2812_Init(&ws2812, &htim2, TIM_CHANNEL_3);
	WS2812_Show(&ws2812);
	std::memcpy(ws2812_last_frame, ws2812.buffer, sizeof(ws2812_last_frame));
	ws2812_last_frame_valid = 1U;
    can_receive.init();
    referee.init_runtime();
    referee_can_assembly_reset();
#if YAW_SYSID_ENABLE == 0
    vt13.init(VT13_UART, vt13.Rx_Buffer, VT13_RX_BUFFER_SIZE);
#endif
	imu.init(IMU_UART,g_uart_rx_buf,g_decode_data,IMU_RX_BUFFER_SIZE);
	vision.init(VISION_UART,vision.Rx_Buffer, VISION_RX_BUFFER_SIZE);
	serialplot.init(SERIALPLOT_UART);

}

void Communicate::run()
{
	const uint32_t now_ms = HAL_GetTick();
	uint8_t ws2812_frame[WS2812_NUM_LEDS * 3] = {0};
	referee.update_runtime(now_ms);
	if (referee_frame0_ready != 0U &&
		(now_ms - referee_frame0_tick_ms) > REFEREE_FRAME_PAIR_TIMEOUT_MS)
	{
		// 只保留短时间内的前半帧，超时后直接丢弃，禁止旧 0x103 与新 0x104 混帧。
		referee_frame0_ready = 0U;
		referee_frame0_tick_ms = 0U;
	}

  	//板间通讯回来了！
	uint8_t top_judge = (vt13.vt13_rc_ctrl.rc.mode_sw == 2);

	can_receive.send_ui_board_com(vt13.vt13_rc_ctrl.rc.right_button,top_judge,vision.vision_recv_data.distance,0x801);
	//视觉通讯
	vision.send();
	//物理"UI"
	ws2812_render_status_frame(ws2812_frame);
	ws2812_apply_status_frame(ws2812_frame);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    // 判断是否是 Bus Off 导致的错误
	if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        // 这里不需要再手动清除标志位了，HAL_FDCAN_IRQHandler 帮你清过了
        
        // 执行重启逻辑
        if (hfdcan == &hfdcan1)
        {
			 // 裁判数据走 FDCAN1。BusOff 恢复前先清缓存和运行时状态，避免异常恢复后误判一次新命中。
			 referee_can_assembly_reset();
			 referee.reset_runtime();
			 FDCAN_Reactivate(&hfdcan1); // 调用之前封装好的重启函数
        }
        else if (hfdcan == &hfdcan2)
        {
			 FDCAN_Reactivate(&hfdcan2);
        }
		else if (hfdcan == &hfdcan3)
		{
			 FDCAN_Reactivate(&hfdcan3);
		}
		
		// 可以在这里加一个全局计数器，记录重启了多少次，方便调试
		//bus_off_recovery_count++; 
	}
}
uint8_t rx_data1[8] = {0};
uint16_t id1;

static void referee_can_assembly_reset(void)
{
	// 统一清空裁判拼帧现场，供上电初始化、BusOff 恢复、异常超时等场景复用。
	std::memset(referee_rx_data, 0, sizeof(referee_rx_data));
	referee_frame0_ready = 0U;
	referee_frame0_tick_ms = 0U;
}

void fdcan1_rx_callback(void)
{
	const uint32_t now_ms = HAL_GetTick();
	std::memset(rx_data1, 0, sizeof(rx_data1));
	id1=fdcanx_receive(&hfdcan1, rx_data1);
	switch (id1)
	{
		case CAN_MOTIVE_FR_MOTOR_ID:
		can_receive.get_dji_motor_measure(&can_receive.chassis_motive_motor[0],rx_data1);
		break;
	case CAN_MOTIVE_FL_MOTOR_ID:
		can_receive.get_dji_motor_measure(&can_receive.chassis_motive_motor[1],rx_data1);
		break;
    case CAN_MOTIVE_BL_MOTOR_ID:
		can_receive.get_dji_motor_measure(&can_receive.chassis_motive_motor[2],rx_data1);
		break;
    case CAN_MOTIVE_BR_MOTOR_ID:
		can_receive.get_dji_motor_measure(&can_receive.chassis_motive_motor[3],rx_data1);
		break;

	case CAN_RIGHT_FRIC_MOTOR_ID:
		can_receive.get_shoot_motor_measure(1, rx_data1);
		break;
	case CAN_LEFT_FRIC_MOTOR_ID:
		can_receive.get_shoot_motor_measure(0, rx_data1);
		break;
	case CAN_TRIGGER_MOTOR_ID:
		can_receive.get_shoot_motor_measure(2, rx_data1);
		break;
	case CAN_REFEREE_DATA_ID:
		std::memcpy(referee_rx_data, rx_data1, sizeof(rx_data1));
		std::memset(referee_rx_data + 8, 0, 5);
		referee_frame0_ready = 1U;
		referee_frame0_tick_ms = now_ms;
		break;
	case CAN_REFEREE_DATA_EXT_ID:
		if (referee_frame0_ready != 0U)
		{
			if ((now_ms - referee_frame0_tick_ms) <= REFEREE_FRAME_PAIR_TIMEOUT_MS)
			{
				std::memcpy(referee_rx_data + 8, rx_data1, 5);
				referee.process_complete_frame(referee_rx_data, now_ms);
				simple_ref_test_on_referee(referee_rx_data);
			}

			referee_frame0_ready = 0U;
			referee_frame0_tick_ms = 0U;
		}
		break;

	//case CAN_SUPER_CAP_ID:
		
		//break;
	default:
		break;
	}
	
	}
	uint8_t rx_data2[8] = {0};
	uint16_t id2;
	void fdcan2_rx_callback(void)
	{
		id2=fdcanx_receive(&hfdcan2, rx_data2);
		switch (id2)
		{
		case MASTER_PITCH_MOTOR_ID:
	 		can_receive.get_dm_motor_measure( &can_receive.gimbal_dm_motor[1],rx_data2);
			break;
		case MASTER_YAW_MOTOR_ID:
			can_receive.get_dm_motor_measure( &can_receive.gimbal_dm_motor[0],rx_data2);
			break;
		default:
			break;
		}
		
	  }
uint8_t rx_data3[8] = {0};
uint16_t id3;
void fdcan3_rx_callback(void)
{
	id3=fdcanx_receive(&hfdcan3, rx_data3);
	switch (id3)
	{
	case 0X203:
		
		break;
	
	default:
		break;
	}
	
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UART_RxEventTypeTypeDef rx_event_type = HAL_UARTEx_GetRxEventType(huart);

	 if (huart->Instance == UART_VISION)
	 {
	 	if (Size == vision.Rx_Buffer_Size &&
	 		 (rx_event_type == HAL_UART_RXEVENT_TC || rx_event_type == HAL_UART_RXEVENT_IDLE))
	 	{
	 		vision.unpack();
	 	}

	 	HAL_UARTEx_ReceiveToIdle_DMA(huart, vision.Rx_Buffer , vision.Rx_Buffer_Size);
	 	if (huart->hdmarx != NULL)
	 	{
	 		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	 	}
		
	 }

#if YAW_SYSID_ENABLE == 0
	if (huart->Instance == UART_VT13)
	{
		
		vt13.unpack();
		//小陀螺开关控制
		if (IF_MOUSE_PRESSED_MID_VT13_LAST && !IF_MOUSE_PRESSED_MID_VT13)
		{
			// 鼠标中键点击事件，切换小陀螺开关状态
			top_switch = !top_switch;
		}
		if(IF_KEY_PRESSED_E_VT13_LAST && !IF_KEY_PRESSED_E_VT13)
		{
			//鼠标E键单击事件，切换扫敌开关状态
			is_sweeping_360 = !is_sweeping_360;
		}
		if(IF_MOUSE_PRESSED_RIGH_VT13_LAST && !IF_MOUSE_PRESSED_RIGH_VT13)
		{
			//鼠标R键单击事件，切换自瞄模式状态
			auto_switch = !auto_switch;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(VT13_UART, vt13.Rx_Buffer, vt13.Rx_Buffer_Size);
	}
#endif

	if(huart->Instance == USART10)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(huart, imu.Rx_Buffer_1 , imu.Rx_Buffer_Size);
			g_uart_rx_cnt = Size ;
			imu.imu_data_unpack();
		}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{	
	#if YAW_SYSID_ENABLE == 0
	if (huart->Instance == UART_VT13){
	HAL_UARTEx_ReceiveToIdle_DMA(VT13_UART, vt13.Rx_Buffer, vt13.Rx_Buffer_Size);
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);  // 禁用半传输中断
	}
	#endif
	if(huart->Instance == USART10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, imu.Rx_Buffer_1 , imu.Rx_Buffer_Size);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);  // 禁用半传输中断
	}
		if(huart->Instance == USART1)
	{
	 	HAL_UARTEx_ReceiveToIdle_DMA(huart, vision.Rx_Buffer , vision.Rx_Buffer_Size);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);  // 禁用半传输中断
	}	
}
