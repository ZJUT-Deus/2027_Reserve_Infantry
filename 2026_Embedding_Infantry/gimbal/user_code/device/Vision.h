#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "bsp_uart.h"
#include "imu.h"
#include "Referee.h"

#define UART_VISION USART1
#define VISION_UART (&huart1)
#define VISION_RX_BUFFER_SIZE VISION_READ_LEN_PACKED

#define VISION_BEGIN (0xA5) //可更改
#define VISION_END (0xFF)	//帧尾

#define VISION_SEND_LEN_PACKED 31 //发送数据包长度
#define VISION_READ_LEN_PACKED 18 //接受数据包长度

#define VISION_CTRL_MODE_ABS 0
#define VISION_CTRL_MODE_DELTA 1
#define VISION_CTRL_MODE VISION_CTRL_MODE_DELTA // 切回原目标值模式时改成 VISION_CTRL_MODE_ABS

typedef enum
{
	VISION_MANU = 0,
	VISION_BUFF = 1,
	VISION_AUTO = 2,
} VisionActData_t; //视觉模式选择

typedef enum
{
	VISION_DISABLED = 0,
	VISION_ACQUIRE,
	VISION_TRACK_UNLOCKED,
	VISION_TRACK_LOCKED,
	VISION_LOST_HOLD,
} vision_ctrl_state_e;

typedef enum
{
	VISION_LED_OFF = 0,
	VISION_LED_ERROR,
	VISION_LED_SEARCHING,
	VISION_LED_TRACKING,
	VISION_LED_LOCKED,
} vision_led_state_e;

typedef __packed struct
{
	/* 头 */
	uint8_t BEGIN; //帧头起始位,暂定0xA5
	uint8_t CmdID; //指令

	/* 数据 */
	float pitch_angle;      // ABS: pitch目标值; DELTA: pitch偏差值
	float yaw_angle;        // ABS: yaw目标值;   DELTA: yaw偏差值
	float distance;			 //距离
	uint8_t centre_lock;	 //是否瞄准到了中间  0没有  1瞄准到了
	uint8_t identify_target; //视野内是否有目标/是否识别到了目标   0否  1是
	uint8_t identify_buff;	 //打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到

	uint8_t END;

} VisionRecvData_t;

// STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef __packed struct
{
	uint8_t BEGIN; //帧头起始位,暂定0xA5
	uint8_t CmdID; //颜色

	fp32 speed; //射速

	fp32 yaw;

	fp32 pitch;

	fp32 roll;

	fp32 angle_x;

	fp32 angle_y;

	fp32 angle_z;

	uint8_t END;

} VisionSendData_t;

typedef struct
{
	volatile uint32_t last_rx_tick;
	volatile uint8_t fresh;
	volatile uint8_t consumed;
	volatile uint8_t timeout;
	volatile uint8_t invalid_frame;
	uint8_t lock_enter_count;
	uint8_t lock_exit_count;
	uint8_t target_lost_count;
	volatile uint8_t fire_qualified;
	vision_ctrl_state_e state;
} VisionRuntimeState;

#define VISION_YAW_RELEASE_SLOTS 4U

typedef struct
{
	fp32 slots[VISION_YAW_RELEASE_SLOTS];
	uint8_t index;
} VisionYawReleaseRuntime;

typedef struct
{
	uint16_t vision_timeout_ms;
	uint8_t lock_enter_threshold;
	uint8_t lock_exit_threshold;
	uint8_t target_lost_threshold;
	fp32 k_yaw;
	fp32 k_pitch;
	fp32 yaw_step_max;
	fp32 pitch_step_max;
} VisionDeltaParam;

class Vision
{
public:
    VisionRecvData_t vision_recv_data;
    VisionSendData_t vision_send_data;
    VisionRuntimeState rt;
    VisionYawReleaseRuntime yaw_release_rt;
    VisionDeltaParam delta_param;

    //数据接收
    uint8_t Rx_Buffer[ VISION_READ_LEN_PACKED];
    uint16_t Rx_Buffer_Size;
    
    void init(UART_HandleTypeDef *huart,uint8_t *Rx_buf,uint16_t Rx_buf_size);
    void send();
    void unpack();
	void vision_get_angle(fp32 *yaw, fp32 *pitch);
    void reset_runtime_state(uint8_t clear_last_rx_tick);
    void clear_lock_qualification();
	vision_led_state_e get_led_state() const;
   
	bool_t vision_if_find_target();

};

extern Referee referee;

#endif
