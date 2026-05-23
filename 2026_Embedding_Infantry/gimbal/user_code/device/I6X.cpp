#include "I6X.h"

#include <math.h>
#include <string.h>

I6X i6x;

static int16_t i6x_map_to_660(int16_t val)
{
    if (val >= 0)
    {
        return (int16_t)floorf((660.0f / 783.0f) * (float)val + 0.5f);
    }
    else
    {
        return (int16_t)floorf((660.0f / 784.0f) * (float)val - 0.5f);
    }
}

static int8_t i6x_to_switch(int16_t val)
{
    if (val > 200)
    {
        return I6X_SW_DOWN;
    }
    else if (val < -200)
    {
        return I6X_SW_UP;
    }

    return I6X_SW_MID;
}

static int16_t i6x_unpack_channel(const uint8_t *sbus_data, uint8_t index)
{
    uint16_t raw = 1024;

    switch (index)
    {
    case 0:
        raw = (sbus_data[1] | (sbus_data[2] << 8)) & 0x07FF;
        break;
    case 1:
        raw = ((sbus_data[2] >> 3) | (sbus_data[3] << 5)) & 0x07FF;
        break;
    case 2:
        raw = ((sbus_data[3] >> 6) | (sbus_data[4] << 2) | (sbus_data[5] << 10)) & 0x07FF;
        break;
    case 3:
        raw = ((sbus_data[5] >> 1) | (sbus_data[6] << 7)) & 0x07FF;
        break;
    case 4:
        raw = ((sbus_data[6] >> 4) | (sbus_data[7] << 4)) & 0x07FF;
        break;
    case 5:
        raw = ((sbus_data[7] >> 7) | (sbus_data[8] << 1) | (sbus_data[9] << 9)) & 0x07FF;
        break;
    case 6:
        raw = ((sbus_data[9] >> 2) | (sbus_data[10] << 6)) & 0x07FF;
        break;
    case 7:
        raw = ((sbus_data[10] >> 5) | (sbus_data[11] << 3)) & 0x07FF;
        break;
    case 8:
        raw = (sbus_data[12] | (sbus_data[13] << 8)) & 0x07FF;
        break;
    case 9:
        raw = ((sbus_data[13] >> 3) | (sbus_data[14] << 5)) & 0x07FF;
        break;
    default:
        break;
    }

    return (int16_t)((int32_t)raw - 1024);
}

static int16_t i6x_deadband_limit(int16_t val, int16_t deadband)
{
    if (val > -deadband && val < deadband)
    {
        return 0;
    }

    return val;
}

static fp32 i6x_norm_ch(int16_t val)
{
    if (val > I6X_CH_VALUE_MAX)
    {
        val = I6X_CH_VALUE_MAX;
    }
    else if (val < I6X_CH_VALUE_MIN)
    {
        val = I6X_CH_VALUE_MIN;
    }

    return (fp32)val / (fp32)I6X_CH_VALUE_MAX;
}

static void i6x_clear_command(I6X_Chassis_cmd_t *chassis_cmd)
{
    memset(chassis_cmd, 0, sizeof(I6X_Chassis_cmd_t));
    chassis_cmd->mode = I6X_CHASSIS_ZERO_FORCE;
}

void I6X::init(UART_HandleTypeDef *huart, uint8_t *Rx_buf, uint16_t Rx_buf_size)
{
    (void)Rx_buf;

    memset(&i6x_rc_ctrl, 0, sizeof(i6x_rc_ctrl));
    memset(&last_i6x_rc_ctrl, 0, sizeof(last_i6x_rc_ctrl));
    i6x_clear_command(&chassis_cmd);

    this->Rx_Buffer_Size = Rx_buf_size;
    UART_Init(huart, this->Rx_Buffer, this->Rx_Buffer_Size);
}

const I6X_RC_ctrl_t *I6X::get_i6x_remote_control_point()
{
    return &i6x_rc_ctrl;
}

I6X_RC_ctrl_t *I6X::get_last_i6x_remote_control_point()
{
    return &last_i6x_rc_ctrl;
}

void I6X::unpack(uint32_t now_ms)
{
    last_i6x_rc_ctrl = i6x_rc_ctrl;

    if (Rx_Buffer[0] != 0x0F || Rx_Buffer[24] != 0x00)
    {
        return;
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        i6x_rc_ctrl.rc.ch[i] = i6x_map_to_660(i6x_unpack_channel(Rx_Buffer, i));
    }

    i6x_rc_ctrl.rc.s[0] = i6x_to_switch(i6x_unpack_channel(Rx_Buffer, 6));
    i6x_rc_ctrl.rc.s[1] = i6x_to_switch(i6x_unpack_channel(Rx_Buffer, 7));
    i6x_rc_ctrl.rc.s[2] = i6x_to_switch(i6x_unpack_channel(Rx_Buffer, 8));
    i6x_rc_ctrl.rc.s[3] = i6x_to_switch(i6x_unpack_channel(Rx_Buffer, 9));

    i6x_rc_ctrl.rc.frame_lost = (Rx_Buffer[23] >> 2) & 0x01;
    i6x_rc_ctrl.rc.failsafe = (Rx_Buffer[23] >> 3) & 0x01;
    i6x_rc_ctrl.last_update_ms = now_ms;
    i6x_rc_ctrl.online = (i6x_rc_ctrl.rc.failsafe == 0) ? 1 : 0;

    update_command();
}

void I6X::update_online(uint32_t now_ms)
{
    if ((now_ms - i6x_rc_ctrl.last_update_ms) > I6X_REMOTE_TIMEOUT_MS)
    {
        i6x_rc_ctrl.online = 0;
    }

    update_command();
}

void I6X::update_command()
{
    int16_t vx_channel;
    int16_t vy_channel;
    int16_t wz_channel;

    if (i6x_rc_ctrl.online == 0 ||
        i6x_rc_ctrl.rc.failsafe != 0)
    {
        i6x_clear_command(&chassis_cmd);
        return;
    }

    if (i6x_switch_is_up(i6x_rc_ctrl.rc.s[I6X_CHASSIS_MODE_SW]))
    {
        chassis_cmd.mode = I6X_CHASSIS_TOP;
    }
    else if (i6x_switch_is_mid(i6x_rc_ctrl.rc.s[I6X_CHASSIS_MODE_SW]))
    {
        chassis_cmd.mode = I6X_CHASSIS_FREE;
    }
    else
    {
        i6x_clear_command(&chassis_cmd);
        return;
    }

    chassis_cmd.enable = 1;

    vx_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_CHASSIS_VX_CH], I6X_RC_DEADBAND);
    vy_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_CHASSIS_VY_CH], I6X_RC_DEADBAND);
    wz_channel = i6x_deadband_limit(i6x_rc_ctrl.rc.ch[I6X_CHASSIS_WZ_CH], I6X_RC_DEADBAND);

    chassis_cmd.vx = i6x_norm_ch(vx_channel) * I6X_CHASSIS_MAX_VX;
    chassis_cmd.vy = i6x_norm_ch(vy_channel) * I6X_CHASSIS_MAX_VY;
    chassis_cmd.wz = i6x_norm_ch(wz_channel) * I6X_CHASSIS_MAX_WZ;
}
