#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "struct_typedef.h"

#define REFEREE_FRAME_PAIR_TIMEOUT_MS 10U
#define REFEREE_WARMUP_TIME_MS 100U
#define REFEREE_WARMUP_FRAME_COUNT 3U
#define REFEREE_OFFLINE_TIMEOUT_MS 200U

typedef enum
{
    REFEREE_STATE_OFFLINE = 0, // 裁判链路离线/刚复位，禁止产生命中事件
    REFEREE_STATE_WARMUP,      // 已收到完整帧，但仍处于上电/重连后的热身观察期
    REFEREE_STATE_ARMED,       // 裁判链路稳定，可正式把掉血识别为一次命中事件
} referee_link_state_e;

class Referee
{
public:
    uint8_t color;                  // 己方颜色 0为红 1为蓝
    uint8_t game_progress;          // 比赛进程
    uint16_t current_HP;            // 机器人当前血量
    uint16_t shooter_17mm_heat;     // 枪管当前热量
    uint16_t chassis_power_limit;   // 底盘功率限制
    uint16_t chassis_power_buffer;  // 底盘缓冲能量 
    uint16_t launching_frequency;   // 当前射频
    float initial_speed;            // 当前射速
    uint8_t armor_id;               //装甲板id
    uint8_t HP_deduction_reason;    //扣血原因
    referee_link_state_e link_state;
    uint32_t last_frame_tick_ms;   // 最近一次完整裁判帧到达时间，用于离线判断
    uint32_t warmup_start_tick_ms; // 进入 WARMUP 的时刻
    uint8_t warmup_frame_count;    // 热身期内累计收到的完整裁判帧数量
    bool_t online;                 // 当前裁判链路是否仍被视为在线
    bool_t fresh;                  // 本控制周期内是否刚收到过完整新帧
    bool_t projectile_hit_pulse;   // 一次性命中脉冲：置位一次，只允许底盘消费一次
    uint16_t event_last_hp;        // 事件判定基线 HP，只用于受击事件边沿比较

    Referee();
    void init_runtime();
    void reset_runtime();
    void update_runtime(uint32_t now_ms);
    void unpack_from_can(const uint8_t data[13]);
    void process_complete_frame(const uint8_t data[13], uint32_t now_ms);
    bool_t consume_projectile_hit_event();
    void discard_projectile_hit_event();
};

extern Referee referee;

#endif
