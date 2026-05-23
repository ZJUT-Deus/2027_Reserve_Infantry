#ifndef SERIALPLOT_H
#define SERIALPLOT_H

#include <stdint.h>

#include "main.h"
#include "bsp_uart.h"

#define YAW_SYSID_ENABLE 0 // 开启后自动进入 yaw sysid，VOFA/扫频输出改走 UART7，不再依赖 VT13 遥控器拨杆
#if YAW_SYSID_ENABLE
#define UART_SERIALPLOT UART7
#define SERIALPLOT_UART (&huart7)
#else
#define UART_SERIALPLOT USART2
#define SERIALPLOT_UART (&huart2)
#endif

#define YAW_SYSID_VIRTUAL_MODE_CODE 3.0f

enum SerialplotYawChannelIndex
{
    SERIALPLOT_CH_SWEEP_SIGNAL = 0,
    SERIALPLOT_CH_YAW_TORQUE_CMD_NM,
    SERIALPLOT_CH_YAW_ANGLE_RAD,
    SERIALPLOT_CH_YAW_SPEED_RADPS,
    SERIALPLOT_CH_COUNT,
};

typedef struct {
    float fdata[SERIALPLOT_CH_COUNT];      // 存放要发送的浮点数
    unsigned char tail[4];      // 存放固定的帧尾
} Vofa_Frame_t;

class Serialplot {
public:
    void init(UART_HandleTypeDef *huart);
    void send_yaw_sysid_frame();
    void update_yaw_sysid_state(uint16_t base_mode);
    uint8_t is_yaw_sysid_mode_active() const;
    float get_yaw_sysid_torque_raw_eq() const;

private:
    float make_relative_time_seconds(uint32_t timestamp_us, uint32_t &base_timestamp_us, uint8_t &initialized_flag);
    void transmit_frame();
    void start_yaw_sysid();
    void stop_yaw_sysid();
    void sync_yaw_hold_targets();
    float compute_yaw_sysid_torque_nm(float elapsed_s) const;

private:
    UART_HandleTypeDef *serialplot_uart;
    uint32_t sample_timestamp_base_us;
    uint32_t ready_timestamp_base_us;
    uint8_t sample_timestamp_initialized;
    uint8_t ready_timestamp_initialized;
    uint8_t yaw_sysid_active;
    uint8_t yaw_sysid_mode_allowed_latched;
    uint32_t yaw_sysid_start_tick_ms;
    float yaw_sysid_elapsed_s;
    float yaw_sysid_torque_nm;

public:
    Vofa_Frame_t vofa_frame;
};

#endif
