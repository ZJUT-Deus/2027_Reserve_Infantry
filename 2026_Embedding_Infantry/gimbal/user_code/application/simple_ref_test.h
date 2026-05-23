#ifndef SIMPLE_REF_TEST_H
#define SIMPLE_REF_TEST_H

#include "main.h"

#define SIMPLE_REF_TEST_DELAY_INVALID 0xFFFFFFFFU

extern volatile uint8_t simple_test_enable;
extern volatile uint8_t simple_test_waiting_referee;

extern volatile uint32_t simple_test_start_ms;
extern volatile uint32_t simple_test_timeout_ms;

extern volatile uint16_t simple_test_freq_base;
extern volatile float simple_test_speed_base;
extern volatile uint16_t simple_test_heat_base;

extern volatile uint16_t simple_test_ref_freq_latest;
extern volatile float simple_test_ref_speed_latest;
extern volatile uint16_t simple_test_ref_heat_latest;

extern volatile uint8_t simple_test_last_valid;
extern volatile uint8_t simple_test_last_timeout;
extern volatile uint32_t simple_test_last_delay_ms;

extern volatile uint16_t simple_test_last_freq_value;
extern volatile float simple_test_last_speed_value;
extern volatile uint16_t simple_test_last_heat_value;

extern volatile uint32_t simple_test_shot_count;
extern volatile uint32_t simple_test_timeout_count;

void simple_ref_test_init(void);
void simple_ref_test_on_start(void);
void simple_ref_test_on_referee(const uint8_t referee_data[12]);
void simple_ref_test_periodic(void);

#endif
