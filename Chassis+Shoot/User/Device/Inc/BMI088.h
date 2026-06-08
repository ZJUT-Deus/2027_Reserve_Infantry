/**
 * @file    BMI088.h
 * @brief   Lightweight BMI088 driver for gimbal attitude feedback.
 */

#ifndef BMI088_H
#define BMI088_H

#include "main.h"
#include "spi.h"
#include "struct_typedef.h"

typedef struct
{
    fp32 accel[3];      /**< m/s^2 */
    fp32 gyro[3];       /**< rad/s */
    fp32 quat[4];       /**< w, x, y, z */
    fp32 roll;          /**< rad */
    fp32 pitch;         /**< rad */
    fp32 yaw;           /**< rad, from quaternion fusion */
    uint32_t last_update_ms;
    uint32_t last_update_us;
    uint32_t update_count;
    uint32_t error_count;
    uint32_t retry_count;
    fp32 gyro_zero_offset[3];
    fp32 accel_chi_square;
    uint16_t gyro_calib_count;
    uint8_t accel_valid;
    uint8_t gyro_valid;
    uint8_t gyro_calibrated;
    uint8_t fusion_ready;
    uint8_t ekf_updated;
    uint8_t online;
} bmi088_data_t;

typedef struct
{
    volatile uint8_t accel_ready;
    volatile uint8_t gyro_ready;
    volatile uint8_t accel_updated;
    volatile uint8_t gyro_updated;
    uint32_t accel_ready_us;
    uint32_t gyro_ready_us;
    uint32_t last_transfer_us;
    uint32_t transfer_timeout_count;
} bmi088_status_t;

class BMI088
{
public:
    bool init(SPI_HandleTypeDef *hspi);
    bool update(uint32_t now_ms);
    void reset_yaw(fp32 yaw = 0.0f);
    void exti_callback(uint16_t gpio_pin);
    const bmi088_data_t *get_data_point() const;
    const bmi088_status_t *get_status_point() const;

private:
    SPI_HandleTypeDef *spi;
    bmi088_data_t data;
    bmi088_status_t status;
    uint32_t last_integrate_us;
    uint32_t last_cycle_count;
    uint32_t cycle_remainder;
    uint64_t time_us_accum;
    fp32 gyro_zero_offset[3];
    fp32 ekf_p[4][4];

    void init_timebase();
    uint32_t now_us();
    bool calibrate_gyro_zero();
    bool accel_read(uint8_t reg, uint8_t *value);
    bool gyro_read(uint8_t reg, uint8_t *value);
    bool accel_write(uint8_t reg, uint8_t value);
    bool gyro_write(uint8_t reg, uint8_t value);
    bool accel_write_checked(uint8_t reg, uint8_t value, uint8_t mask);
    bool gyro_write_checked(uint8_t reg, uint8_t value, uint8_t mask);
    bool read_accel_raw(int16_t raw[3]);
    bool read_gyro_raw(int16_t raw[3]);
    bool read_accel_raw_retry(int16_t raw[3]);
    bool read_gyro_raw_retry(int16_t raw[3]);
    bool probe_accel();
    bool probe_gyro();
    bool config_accel();
    bool config_gyro();
    bool validate_accel() const;
    bool validate_gyro() const;
    void init_fusion_from_accel(fp32 yaw);
    void ekf_reset_covariance();
    void fusion_update(fp32 dt);
    void ekf_predict(fp32 dt);
    bool ekf_update_accel();
    void update_euler_from_quat();
};

extern BMI088 bmi088;

#endif
