#include "Referee.h"

Referee referee;

Referee::Referee()
{
    color = 0U;
    game_progress = 0U;
    current_HP = 0U;
    shooter_17mm_heat = 0U;
    chassis_power_limit = 0U;
    chassis_power_buffer = 0U;
    launching_frequency = 0U;
    initial_speed = 0.0f;
    armor_id = 0U;
    HP_deduction_reason = 0U;
    init_runtime();
}

void Referee::init_runtime()
{
    reset_runtime();
}

void Referee::reset_runtime()
{
    // 统一回到“裁判链路不可信”的初态：不保留历史 pulse，也不允许直接判命中。
    link_state = REFEREE_STATE_OFFLINE;
    last_frame_tick_ms = 0U;
    warmup_start_tick_ms = 0U;
    warmup_frame_count = 0U;
    online = FALSE;
    fresh = FALSE;
    projectile_hit_pulse = FALSE;
    event_last_hp = current_HP;
}

void Referee::update_runtime(uint32_t now_ms)
{
    // fresh 只表示“这一拍刚收到了完整新帧”，因此每拍开始先清零。
    fresh = FALSE;

    if (online == TRUE &&
        last_frame_tick_ms != 0U &&
        (now_ms - last_frame_tick_ms) > REFEREE_OFFLINE_TIMEOUT_MS)
    {
        // 超过离线阈值后，重新回到 OFFLINE，要求后续重新热身再解锁受击事件。
        reset_runtime();
    }
}

void Referee::unpack_from_can(const uint8_t data[13])
{
    if (data == 0)
    {
        return;
    }

    // 0: 高4位为颜色，低4位为比赛进程
    this->color = (data[0] >> 4) & 0x0F;
    this->game_progress = data[0] & 0x0F;

    // 1-2: 当前血量 (低八位在前，高八位在后)
    this->current_HP = (uint16_t)(data[1] | (data[2] << 8));
    //3-4: 底盘功率限制 (低八位在前，高八位在后)
    this->chassis_power_limit = (uint16_t)(data[3] | (data[4] << 8));
    //5-6: 底盘缓冲能量
    this->chassis_power_buffer = (uint16_t)(data[5] | (data[6] << 8));
    // 7-8: 17mm枪管热量 (低八位在前，高八位在后)
    this->shooter_17mm_heat = (uint16_t)(data[7] | (data[8] << 8));


    // 9: 射频
    this->launching_frequency = data[9];

    // 10-11: 射速 (发送端乘了100，这里除以100.0f还原为float)
    uint16_t speed_int = (uint16_t)(data[10] | (data[11] << 8));
    this->initial_speed = (float)speed_int / 100.0f;

    // 12: robot_hurt 扩展字节，低4位为armor_id，高4位为扣血原因
    this->armor_id = data[12] & 0x0F;
    this->HP_deduction_reason = (data[12] >> 4) & 0x0F;
}

void Referee::process_complete_frame(const uint8_t data[13], uint32_t now_ms)
{
    if (data == 0)
    {
        return;
    }

    unpack_from_can(data);

    last_frame_tick_ms = now_ms;
    online = TRUE;
    fresh = TRUE;
    projectile_hit_pulse = FALSE;

    if (link_state == REFEREE_STATE_OFFLINE)
    {
        // 刚从 OFFLINE 收到第一帧时，只进入热身期并同步基线 HP，禁止首帧直接触发受击。
        link_state = REFEREE_STATE_WARMUP;
        warmup_start_tick_ms = now_ms;
        warmup_frame_count = 1U;
        event_last_hp = current_HP;
        return;
    }

    if (link_state == REFEREE_STATE_WARMUP)
    {
        if (warmup_frame_count < 0xFFU)
        {
            warmup_frame_count++;
        }

        // 热身期内只更新基线 HP，不把任何掉血当成有效命中事件。
        event_last_hp = current_HP;

        if (warmup_frame_count >= REFEREE_WARMUP_FRAME_COUNT &&
            (now_ms - warmup_start_tick_ms) >= REFEREE_WARMUP_TIME_MS)
        {
            // 同时满足“累计帧数 + 累计时间”后才正式解锁受击事件。
            link_state = REFEREE_STATE_ARMED;
        }

        return;
    }

    if (current_HP < event_last_hp &&
        HP_deduction_reason == 0U)
    {
        // 这里只负责产生一次性 pulse；是否允许进入变速小陀螺，仍由底盘原状态机决定。
        projectile_hit_pulse = TRUE;
    }

    event_last_hp = current_HP;
}

bool_t Referee::consume_projectile_hit_event()
{
    if (projectile_hit_pulse == FALSE)
    {
        return FALSE;
    }

    // pulse 只可消费一次，避免同一次掉血在底盘侧被重复理解为多次命中。
    projectile_hit_pulse = FALSE;
    return TRUE;
}

void Referee::discard_projectile_hit_event()
{
    projectile_hit_pulse = FALSE;
}
