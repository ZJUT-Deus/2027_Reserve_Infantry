#include "Serialplot.h"

#include <math.h>

#include "gimbal.h"
#include "Communicate.h"

namespace
{
const float YAW_SYSID_SWEEP_START_HZ = 0.2f;
const float YAW_SYSID_SWEEP_END_HZ = 6.0f;
const float YAW_SYSID_SWEEP_DURATION_S = 60.0f;
const float YAW_SYSID_SWEEP_PEAK_TORQUE_NM = 0.8f;
const float YAW_SYSID_SWEEP_TORQUE_LIMIT_NM = 1.2f;
const float YAW_SYSID_RAW_SCALE = 1000.0f;
const float YAW_SYSID_TWO_PI = 6.2831853072f;
}

float Serialplot::make_relative_time_seconds(uint32_t timestamp_us, uint32_t &base_timestamp_us, uint8_t &initialized_flag)
{
    if (initialized_flag == 0U)
    {
        base_timestamp_us = timestamp_us;
        initialized_flag = 1U;
    }

    return (float)(timestamp_us - base_timestamp_us) * 1e-6f;
}

void Serialplot::transmit_frame()
{
    if (serialplot_uart == 0)
    {
        return;
    }

    HAL_UART_Transmit(serialplot_uart, (uint8_t *)&vofa_frame, sizeof(vofa_frame), 2U);
}

float Serialplot::compute_yaw_sysid_torque_nm(float elapsed_s) const
{
    if (elapsed_s <= 0.0f)
    {
        return 0.0f;
    }

    const float freq_ratio = YAW_SYSID_SWEEP_END_HZ / YAW_SYSID_SWEEP_START_HZ;
    const float log_ratio = (float)log(freq_ratio);
    const float phase = (YAW_SYSID_TWO_PI * YAW_SYSID_SWEEP_START_HZ * YAW_SYSID_SWEEP_DURATION_S / log_ratio) *
                        ((float)pow(freq_ratio, elapsed_s / YAW_SYSID_SWEEP_DURATION_S) - 1.0f);
    float torque_nm = YAW_SYSID_SWEEP_PEAK_TORQUE_NM * (float)sin(phase);

    if (torque_nm > YAW_SYSID_SWEEP_TORQUE_LIMIT_NM)
    {
        torque_nm = YAW_SYSID_SWEEP_TORQUE_LIMIT_NM;
    }
    else if (torque_nm < -YAW_SYSID_SWEEP_TORQUE_LIMIT_NM)
    {
        torque_nm = -YAW_SYSID_SWEEP_TORQUE_LIMIT_NM;
    }

    return torque_nm;
}

void Serialplot::sync_yaw_hold_targets()
{
    gimbal.gimbal_yaw_motor.encode_angle_set = gimbal.gimbal_yaw_motor.encode_angle;
    gimbal.gimbal_yaw_motor.gyro_angle_set = gimbal.gimbal_yaw_motor.gyro_angle;
    gimbal.gimbal_yaw_motor.speed_set = 0.0f;
    gimbal.gimbal_pitch_motor.encode_angle_set = gimbal.gimbal_pitch_motor.encode_angle;
}

void Serialplot::start_yaw_sysid()
{
    yaw_sysid_active = 1U;
    yaw_sysid_start_tick_ms = HAL_GetTick();
    yaw_sysid_elapsed_s = 0.0f;
    yaw_sysid_torque_nm = 0.0f;

    gimbal.gimbal_yaw_motor.speed_pid.pid_clear();
    gimbal.gimbal_yaw_motor.encode_angle_pid.pid_clear();
    gimbal.gimbal_yaw_motor.gyro_angle_pid.pid_clear();
    sync_yaw_hold_targets();
}

void Serialplot::stop_yaw_sysid()
{
    yaw_sysid_active = 0U;
    yaw_sysid_elapsed_s = 0.0f;
    yaw_sysid_torque_nm = 0.0f;

    gimbal.gimbal_yaw_motor.speed_pid.pid_clear();
    gimbal.gimbal_yaw_motor.encode_angle_pid.pid_clear();
    gimbal.gimbal_yaw_motor.gyro_angle_pid.pid_clear();
    sync_yaw_hold_targets();
}

void Serialplot::update_yaw_sysid_state(uint16_t base_mode)
{
    const dm_motor_measure_t *yaw_motor_measure = gimbal.gimbal_yaw_motor.motor_measure;
    const uint8_t yaw_motor_ready = (yaw_motor_measure != 0 && yaw_motor_measure->state != 0) ? 1U : 0U;

#if YAW_SYSID_ENABLE == 0
    const uint8_t mode_allowed = (base_mode == GIMBAL_FREE && yaw_motor_ready != 0U) ? 1U : 0U;
    const uint8_t mode_allowed_rising_edge = (mode_allowed != 0U && yaw_sysid_mode_allowed_latched == 0U) ? 1U : 0U;
    yaw_sysid_mode_allowed_latched = mode_allowed;
    if (yaw_sysid_active != 0U)
    {
        stop_yaw_sysid();
    }
    yaw_sysid_elapsed_s = 0.0f;
    yaw_sysid_torque_nm = 0.0f;
    return;
#endif

    // const uint8_t mode_allowed = yaw_motor_ready;
    // const uint8_t mode_allowed_rising_edge = (mode_allowed != 0U && yaw_sysid_mode_allowed_latched == 0U) ? 1U : 0U;

    yaw_sysid_mode_allowed_latched = mode_allowed;

    if (yaw_sysid_active == 0U)
    {
        yaw_sysid_elapsed_s = 0.0f;
        yaw_sysid_torque_nm = 0.0f;
        if (mode_allowed_rising_edge != 0U)
        {
            start_yaw_sysid();
        }
        return;
    }

    if (mode_allowed == 0U)
    {
        stop_yaw_sysid();
        return;
    }

    yaw_sysid_elapsed_s = (float)(HAL_GetTick() - yaw_sysid_start_tick_ms) * 0.001f;
    if (yaw_sysid_elapsed_s >= YAW_SYSID_SWEEP_DURATION_S)
    {
        stop_yaw_sysid();
        return;
    }

    sync_yaw_hold_targets();
    yaw_sysid_torque_nm = compute_yaw_sysid_torque_nm(yaw_sysid_elapsed_s);
}

uint8_t Serialplot::is_yaw_sysid_mode_active() const
{
    return yaw_sysid_active;
}

float Serialplot::get_yaw_sysid_torque_raw_eq() const
{
    return yaw_sysid_torque_nm * YAW_SYSID_RAW_SCALE;
}

void Serialplot::init(UART_HandleTypeDef *huart)
{
    serialplot_uart = huart;
    sample_timestamp_base_us = 0U;
    ready_timestamp_base_us = 0U;
    sample_timestamp_initialized = 0U;
    ready_timestamp_initialized = 0U;
    yaw_sysid_active = 0U;
    yaw_sysid_mode_allowed_latched = 0U;
    yaw_sysid_start_tick_ms = 0U;
    yaw_sysid_elapsed_s = 0.0f;
    yaw_sysid_torque_nm = 0.0f;

    vofa_frame.tail[0] = 0x00;
    vofa_frame.tail[1] = 0x00;
    vofa_frame.tail[2] = 0x80;
    vofa_frame.tail[3] = 0x7f;
}

void Serialplot::send_yaw_sysid_frame()
{
    vofa_frame.fdata[SERIALPLOT_CH_SWEEP_SIGNAL] = yaw_sysid_torque_nm;
    vofa_frame.fdata[SERIALPLOT_CH_YAW_TORQUE_CMD_NM] = gimbal.gimbal_yaw_motor.current_give;
    vofa_frame.fdata[SERIALPLOT_CH_YAW_ANGLE_RAD] = gimbal.gimbal_yaw_motor.gyro_angle;
    vofa_frame.fdata[SERIALPLOT_CH_YAW_SPEED_RADPS] = gimbal.gimbal_yaw_motor.speed;

    transmit_frame();
}
