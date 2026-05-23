#include "simple_ref_test.h"

#include "stm32h7xx_hal.h"

volatile uint8_t simple_test_enable = 0U;
volatile uint8_t simple_test_waiting_referee = 0U;

volatile uint32_t simple_test_start_ms = 0U;
volatile uint32_t simple_test_timeout_ms = 1000U;

volatile uint16_t simple_test_freq_base = 0U;
volatile float simple_test_speed_base = 0.0f;
volatile uint16_t simple_test_heat_base = 0U;

volatile uint16_t simple_test_ref_freq_latest = 0U;
volatile float simple_test_ref_speed_latest = 0.0f;
volatile uint16_t simple_test_ref_heat_latest = 0U;

volatile uint8_t simple_test_last_valid = 0U;
volatile uint8_t simple_test_last_timeout = 0U;
volatile uint32_t simple_test_last_delay_ms = SIMPLE_REF_TEST_DELAY_INVALID;

volatile uint16_t simple_test_last_freq_value = 0U;
volatile float simple_test_last_speed_value = 0.0f;
volatile uint16_t simple_test_last_heat_value = 0U;

volatile uint32_t simple_test_shot_count = 0U;
volatile uint32_t simple_test_timeout_count = 0U;

void simple_ref_test_init(void)
{
    simple_test_enable = 1U;
    simple_test_waiting_referee = 0U;

    simple_test_start_ms = 0U;
    simple_test_timeout_ms = 1000U;

    simple_test_freq_base = 0U;
    simple_test_speed_base = 0.0f;
    simple_test_heat_base = 0U;

    simple_test_ref_freq_latest = 0U;
    simple_test_ref_speed_latest = 0.0f;
    simple_test_ref_heat_latest = 0U;

    simple_test_last_valid = 0U;
    simple_test_last_timeout = 0U;
    simple_test_last_delay_ms = SIMPLE_REF_TEST_DELAY_INVALID;

    simple_test_last_freq_value = 0U;
    simple_test_last_speed_value = 0.0f;
    simple_test_last_heat_value = 0U;

    simple_test_shot_count = 0U;
    simple_test_timeout_count = 0U;
}

void simple_ref_test_on_start(void)
{
    if (simple_test_enable == 0U || simple_test_waiting_referee != 0U)
    {
        return;
    }

    simple_test_waiting_referee = 1U;
    simple_test_start_ms = HAL_GetTick();
    simple_test_freq_base = simple_test_ref_freq_latest;
    simple_test_speed_base = simple_test_ref_speed_latest;
    simple_test_heat_base = simple_test_ref_heat_latest;

    simple_test_last_valid = 0U;
    simple_test_last_timeout = 0U;
    simple_test_last_delay_ms = SIMPLE_REF_TEST_DELAY_INVALID;
}

void simple_ref_test_on_referee(const uint8_t referee_data[12])
{
    uint16_t heat_now;
    uint16_t freq_now;
    uint16_t speed_raw;
    float speed_now;

    if (simple_test_enable == 0U || referee_data == NULL)
    {
        return;
    }

    heat_now = (uint16_t)(referee_data[7] | (referee_data[8] << 8));
    freq_now = (uint16_t)referee_data[9];
    speed_raw = (uint16_t)(referee_data[10] | (referee_data[11] << 8));
    speed_now = (float)speed_raw / 100.0f;

    simple_test_ref_heat_latest = heat_now;
    simple_test_ref_freq_latest = freq_now;
    simple_test_ref_speed_latest = speed_now;

    if (simple_test_waiting_referee == 0U)
    {
        return;
    }

    if (freq_now != simple_test_freq_base)
    {
        simple_test_last_delay_ms = HAL_GetTick() - simple_test_start_ms;
        simple_test_last_valid = 1U;
        simple_test_last_timeout = 0U;
        simple_test_last_freq_value = freq_now;
        simple_test_last_speed_value = speed_now;
        simple_test_last_heat_value = heat_now;
        simple_test_waiting_referee = 0U;
        simple_test_shot_count++;
    }
}

void simple_ref_test_periodic(void)
{
    if (simple_test_enable == 0U || simple_test_waiting_referee == 0U)
    {
        return;
    }

    if ((HAL_GetTick() - simple_test_start_ms) > simple_test_timeout_ms)
    {
        simple_test_last_valid = 0U;
        simple_test_last_timeout = 1U;
        simple_test_last_delay_ms = SIMPLE_REF_TEST_DELAY_INVALID;
        simple_test_waiting_referee = 0U;
        simple_test_timeout_count++;
    }
}
