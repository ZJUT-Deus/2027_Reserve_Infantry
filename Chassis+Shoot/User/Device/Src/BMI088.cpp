/**
 * @file    BMI088.cpp
 * @brief   BMI088 driver with lightweight quaternion fusion for gimbal TOP mode.
 */

#include "BMI088.h"
#include "user_lib.h"

#include <math.h>
#include <string.h>

BMI088 bmi088;

#define BMI088_ACCEL_CHIP_ID_REG     0x00U
#define BMI088_ACCEL_CHIP_ID         0x1EU
#define BMI088_ACCEL_X_LSB           0x12U
#define BMI088_ACCEL_CONF            0x40U
#define BMI088_ACCEL_RANGE           0x41U
#define BMI088_ACCEL_INT1_IO_CTRL    0x53U
#define BMI088_ACCEL_INT_MAP_DATA    0x58U
#define BMI088_ACCEL_PWR_CONF        0x7CU
#define BMI088_ACCEL_PWR_CTRL        0x7DU
#define BMI088_ACCEL_SOFTRESET       0x7EU

#define BMI088_GYRO_CHIP_ID_REG      0x00U
#define BMI088_GYRO_CHIP_ID          0x0FU
#define BMI088_GYRO_X_LSB            0x02U
#define BMI088_GYRO_RANGE            0x0FU
#define BMI088_GYRO_BANDWIDTH        0x10U
#define BMI088_GYRO_SOFTRESET        0x14U
#define BMI088_GYRO_INT_CTRL         0x15U
#define BMI088_GYRO_INT_IO_CONF      0x16U
#define BMI088_GYRO_INT_IO_MAP       0x18U

#define BMI088_READ_MASK             0x80U
#define BMI088_SPI_TIMEOUT_MS        2U
#define BMI088_GRAVITY               9.7947f
#define BMI088_ACCEL_RANGE_CFG       0x03U   /* +/-24g */
#define BMI088_ACCEL_CONF_CFG        ((uint8_t)((0x0AU << 4) | 0x0CU))
#define BMI088_GYRO_RANGE_CFG        0x00U   /* 2000 dps */
#define BMI088_GYRO_BANDWIDTH_CFG    0x01U
#define BMI088_ACCEL_INT1_CFG        0x0AU   /* INT1 output, push-pull, active high */
#define BMI088_ACCEL_INT1_DRDY_MAP   0x04U
#define BMI088_GYRO_DRDY_ENABLE      0x80U
#define BMI088_GYRO_INT3_CFG         0x01U   /* INT3 push-pull, active high */
#define BMI088_GYRO_INT3_DRDY_MAP    0x01U

#define BMI088_ACC_NORM_MIN          (0.45f * BMI088_GRAVITY)
#define BMI088_ACC_NORM_MAX          (1.65f * BMI088_GRAVITY)
#define BMI088_DT_TIMEOUT_S          0.1f
#define BMI088_DEFAULT_DT_S          0.002f
#define BMI088_EKF_INIT_P            0.02f
#define BMI088_EKF_GYRO_Q            0.00008f
#define BMI088_EKF_ACC_R             0.018f
#define BMI088_EKF_MIN_P             0.000001f
#define BMI088_EKF_MAX_P             0.20f
#define BMI088_EKF_ACC_CHI_GATE      9.0f
#define BMI088_GYRO_CALIB_SAMPLES    500U
#define BMI088_GYRO_CALIB_DELAY_MS   1U
#define BMI088_GYRO_CALIB_MAX_ABS    0.35f

static const fp32 bmi088_default_gyro_zero_offset[3] = {
    -0.005450708363333335f,
    -0.0008284202383333334f,
    -0.0006914497383333334f,
};

static inline fp32 bmi088_accel_scale()
{
    return (24.0f * BMI088_GRAVITY) / 32768.0f;
}

static inline fp32 bmi088_gyro_scale()
{
    return (2000.0f * PI / 180.0f) / 32768.0f;
}

static int16_t make_i16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)((uint16_t)lsb | ((uint16_t)msb << 8));
}

static void cs_write(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static void quat_normalize(fp32 q[4])
{
    const fp32 norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 1.0e-6f && isfinite(norm))
    {
        const fp32 inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
    else
    {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }
}

static fp32 quat_get_yaw(const fp32 q[4])
{
    return rad_format(atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                             1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])));
}

static void quat_from_euler(fp32 roll, fp32 pitch, fp32 yaw, fp32 q[4])
{
    const fp32 half_roll = 0.5f * roll;
    const fp32 half_pitch = 0.5f * pitch;
    const fp32 half_yaw = 0.5f * yaw;

    const fp32 cr = cosf(half_roll);
    const fp32 sr = sinf(half_roll);
    const fp32 cp = cosf(half_pitch);
    const fp32 sp = sinf(half_pitch);
    const fp32 cy = cosf(half_yaw);
    const fp32 sy = sinf(half_yaw);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    quat_normalize(q);
}

static void quat_keep_yaw_from_gyro(fp32 q[4], fp32 yaw)
{
    const fp32 roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                             1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    const fp32 pitch = asinf(fp32_constrain(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f));

    quat_from_euler(roll, pitch, yaw, q);
}

static bool invert_3x3(const fp32 a[3][3], fp32 inv[3][3])
{
    const fp32 det =
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
        a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
        a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if (fabsf(det) < 1.0e-9f || !isfinite(det))
    {
        return false;
    }

    const fp32 inv_det = 1.0f / det;

    inv[0][0] =  (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
    inv[0][1] = -(a[0][1] * a[2][2] - a[0][2] * a[2][1]) * inv_det;
    inv[0][2] =  (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
    inv[1][0] = -(a[1][0] * a[2][2] - a[1][2] * a[2][0]) * inv_det;
    inv[1][1] =  (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
    inv[1][2] = -(a[0][0] * a[1][2] - a[0][2] * a[1][0]) * inv_det;
    inv[2][0] =  (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
    inv[2][1] = -(a[0][0] * a[2][1] - a[0][1] * a[2][0]) * inv_det;
    inv[2][2] =  (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;

    return true;
}

bool BMI088::init(SPI_HandleTypeDef *hspi)
{
    spi = hspi;
    memset(&data, 0, sizeof(data));
    memset(&status, 0, sizeof(status));
    data.quat[0] = 1.0f;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        gyro_zero_offset[i] = bmi088_default_gyro_zero_offset[i];
        data.gyro_zero_offset[i] = gyro_zero_offset[i];
    }
    ekf_reset_covariance();
    init_timebase();
    last_integrate_us = now_us();

    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_SET);
    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    if (!probe_accel() || !probe_gyro())
    {
        data.online = 0U;
        data.error_count++;
        return false;
    }

    if (!accel_write(BMI088_ACCEL_SOFTRESET, 0xB6U) ||
        !gyro_write(BMI088_GYRO_SOFTRESET, 0xB6U))
    {
        data.online = 0U;
        data.error_count++;
        return false;
    }
    HAL_Delay(50);

    if (!config_accel() || !config_gyro())
    {
        data.online = 0U;
        data.error_count++;
        return false;
    }

    data.gyro_calibrated = calibrate_gyro_zero() ? 1U : 0U;

    data.online = 1U;
    data.last_update_ms = HAL_GetTick();
    data.last_update_us = now_us();
    last_integrate_us = data.last_update_us;
    return true;
}

void BMI088::init_timebase()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_cycle_count = DWT->CYCCNT;
    cycle_remainder = 0U;
    time_us_accum = (uint64_t)HAL_GetTick() * 1000ULL;
}

uint32_t BMI088::now_us()
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U || SystemCoreClock == 0U)
    {
        return HAL_GetTick() * 1000U;
    }

    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
    {
        return HAL_GetTick() * 1000U;
    }

    const uint32_t cycle_now = DWT->CYCCNT;
    const uint32_t cycle_delta = cycle_now - last_cycle_count;
    last_cycle_count = cycle_now;

    const uint64_t total_cycles = (uint64_t)cycle_remainder + (uint64_t)cycle_delta;
    time_us_accum += total_cycles / cycles_per_us;
    cycle_remainder = (uint32_t)(total_cycles % cycles_per_us);

    return (uint32_t)time_us_accum;
}

bool BMI088::probe_accel()
{
    uint8_t id = 0U;
    for (uint8_t i = 0U; i < 5U; i++)
    {
        if (accel_read(BMI088_ACCEL_CHIP_ID_REG, &id) && id == BMI088_ACCEL_CHIP_ID)
        {
            return true;
        }
        HAL_Delay(10);
    }
    return false;
}

bool BMI088::probe_gyro()
{
    uint8_t id = 0U;
    for (uint8_t i = 0U; i < 5U; i++)
    {
        if (gyro_read(BMI088_GYRO_CHIP_ID_REG, &id) && id == BMI088_GYRO_CHIP_ID)
        {
            return true;
        }
        HAL_Delay(10);
    }
    return false;
}

bool BMI088::calibrate_gyro_zero()
{
    fp32 sum[3] = {0.0f};
    fp32 new_offset[3] = {0.0f};
    int16_t raw[3] = {0};
    const fp32 gyro_scale = bmi088_gyro_scale();

    data.gyro_calib_count = 0U;

    for (uint16_t sample = 0U; sample < BMI088_GYRO_CALIB_SAMPLES; sample++)
    {
        if (!read_gyro_raw_retry(raw))
        {
            return false;
        }

        for (uint8_t i = 0U; i < 3U; i++)
        {
            sum[i] += (fp32)raw[i] * gyro_scale;
        }
        data.gyro_calib_count++;
        HAL_Delay(BMI088_GYRO_CALIB_DELAY_MS);
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        const fp32 avg = sum[i] / (fp32)data.gyro_calib_count;
        if (!isfinite(avg) || fabsf(avg) > BMI088_GYRO_CALIB_MAX_ABS)
        {
            return false;
        }

        new_offset[i] = -avg;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        gyro_zero_offset[i] = new_offset[i];
        data.gyro_zero_offset[i] = gyro_zero_offset[i];
    }

    return true;
}

bool BMI088::config_accel()
{
    bool ok = true;

    ok = accel_write_checked(BMI088_ACCEL_PWR_CTRL, 0x04U, 0x04U) && ok;
    HAL_Delay(5);
    ok = accel_write_checked(BMI088_ACCEL_PWR_CONF, 0x00U, 0x03U) && ok;
    HAL_Delay(5);
    ok = accel_write_checked(BMI088_ACCEL_CONF, BMI088_ACCEL_CONF_CFG, 0xFFU) && ok;
    ok = accel_write_checked(BMI088_ACCEL_RANGE, BMI088_ACCEL_RANGE_CFG, 0x03U) && ok;
    ok = accel_write_checked(BMI088_ACCEL_INT1_IO_CTRL, BMI088_ACCEL_INT1_CFG, 0x1EU) && ok;
    ok = accel_write_checked(BMI088_ACCEL_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_MAP, BMI088_ACCEL_INT1_DRDY_MAP) && ok;

    return ok;
}

bool BMI088::config_gyro()
{
    bool ok = true;

    ok = gyro_write_checked(BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_CFG, 0x07U) && ok;
    ok = gyro_write_checked(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BANDWIDTH_CFG, 0xFFU) && ok;
    ok = gyro_write_checked(BMI088_GYRO_INT_CTRL, BMI088_GYRO_DRDY_ENABLE, BMI088_GYRO_DRDY_ENABLE) && ok;
    ok = gyro_write_checked(BMI088_GYRO_INT_IO_CONF, BMI088_GYRO_INT3_CFG, 0x0FU) && ok;
    ok = gyro_write_checked(BMI088_GYRO_INT_IO_MAP, BMI088_GYRO_INT3_DRDY_MAP, BMI088_GYRO_INT3_DRDY_MAP) && ok;

    return ok;
}

bool BMI088::accel_read(uint8_t reg, uint8_t *value)
{
    uint8_t tx[3] = {(uint8_t)(reg | BMI088_READ_MASK), 0U, 0U};
    uint8_t rx[3] = {0U};

    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(spi, tx, rx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK || value == NULL)
    {
        data.error_count++;
        return false;
    }

    *value = rx[2];
    return true;
}

bool BMI088::gyro_read(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | BMI088_READ_MASK), 0U};
    uint8_t rx[2] = {0U};

    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(spi, tx, rx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK || value == NULL)
    {
        data.error_count++;
        return false;
    }

    *value = rx[1];
    return true;
}

bool BMI088::accel_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef ret = HAL_SPI_Transmit(spi, tx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK)
    {
        data.error_count++;
        return false;
    }
    return true;
}

bool BMI088::gyro_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef ret = HAL_SPI_Transmit(spi, tx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK)
    {
        data.error_count++;
        return false;
    }
    return true;
}

bool BMI088::accel_write_checked(uint8_t reg, uint8_t value, uint8_t mask)
{
    uint8_t read_back = 0U;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (accel_write(reg, value))
        {
            HAL_Delay(1);
            if (accel_read(reg, &read_back) && ((read_back & mask) == (value & mask)))
            {
                return true;
            }
        }
    }
    data.error_count++;
    return false;
}

bool BMI088::gyro_write_checked(uint8_t reg, uint8_t value, uint8_t mask)
{
    uint8_t read_back = 0U;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (gyro_write(reg, value))
        {
            HAL_Delay(1);
            if (gyro_read(reg, &read_back) && ((read_back & mask) == (value & mask)))
            {
                return true;
            }
        }
    }
    data.error_count++;
    return false;
}

bool BMI088::read_accel_raw(int16_t raw[3])
{
    uint8_t tx[8] = {(uint8_t)(BMI088_ACCEL_X_LSB | BMI088_READ_MASK)};
    uint8_t rx[8] = {0U};

    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(spi, tx, rx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
    cs_write(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK)
    {
        data.error_count++;
        return false;
    }

    raw[0] = make_i16(rx[2], rx[3]);
    raw[1] = make_i16(rx[4], rx[5]);
    raw[2] = make_i16(rx[6], rx[7]);
    return true;
}

bool BMI088::read_gyro_raw(int16_t raw[3])
{
    uint8_t tx[7] = {(uint8_t)(BMI088_GYRO_X_LSB | BMI088_READ_MASK)};
    uint8_t rx[7] = {0U};

    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(spi, tx, rx, sizeof(tx), BMI088_SPI_TIMEOUT_MS);
    cs_write(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK)
    {
        data.error_count++;
        return false;
    }

    raw[0] = make_i16(rx[1], rx[2]);
    raw[1] = make_i16(rx[3], rx[4]);
    raw[2] = make_i16(rx[5], rx[6]);
    return true;
}

bool BMI088::read_accel_raw_retry(int16_t raw[3])
{
    if (read_accel_raw(raw))
    {
        status.accel_updated = 1U;
        return true;
    }

    data.retry_count++;
    if (read_accel_raw(raw))
    {
        status.accel_updated = 1U;
        return true;
    }
    return false;
}

bool BMI088::read_gyro_raw_retry(int16_t raw[3])
{
    if (read_gyro_raw(raw))
    {
        status.gyro_updated = 1U;
        return true;
    }

    data.retry_count++;
    if (read_gyro_raw(raw))
    {
        status.gyro_updated = 1U;
        return true;
    }
    return false;
}

bool BMI088::validate_accel() const
{
    const fp32 norm = sqrtf(data.accel[0] * data.accel[0] +
                            data.accel[1] * data.accel[1] +
                            data.accel[2] * data.accel[2]);

    return isfinite(data.accel[0]) && isfinite(data.accel[1]) && isfinite(data.accel[2]) &&
           norm >= BMI088_ACC_NORM_MIN && norm <= BMI088_ACC_NORM_MAX;
}

bool BMI088::validate_gyro() const
{
    return isfinite(data.gyro[0]) && isfinite(data.gyro[1]) && isfinite(data.gyro[2]);
}

void BMI088::init_fusion_from_accel(fp32 yaw)
{
    const fp32 accel_norm = sqrtf(data.accel[0] * data.accel[0] +
                                  data.accel[1] * data.accel[1] +
                                  data.accel[2] * data.accel[2]);
    if (accel_norm <= 1.0e-6f || !isfinite(accel_norm))
    {
        data.quat[0] = 1.0f;
        data.quat[1] = 0.0f;
        data.quat[2] = 0.0f;
        data.quat[3] = 0.0f;
        data.fusion_ready = 0U;
        return;
    }

    const fp32 pitch = asinf(fp32_constrain(-data.accel[0] / accel_norm, -1.0f, 1.0f));
    const fp32 roll = atan2f(data.accel[1], data.accel[2]);

    quat_from_euler(roll, pitch, yaw, data.quat);
    ekf_reset_covariance();
    data.fusion_ready = 1U;
    data.ekf_updated = 0U;
    update_euler_from_quat();
}

void BMI088::ekf_reset_covariance()
{
    memset(ekf_p, 0, sizeof(ekf_p));
    for (uint8_t i = 0U; i < 4U; i++)
    {
        ekf_p[i][i] = BMI088_EKF_INIT_P;
    }
}

void BMI088::fusion_update(fp32 dt)
{
    if (dt <= 0.0f || dt > BMI088_DT_TIMEOUT_S || !isfinite(dt))
    {
        dt = BMI088_DEFAULT_DT_S;
        status.transfer_timeout_count++;
    }

    if (!data.fusion_ready)
    {
        init_fusion_from_accel(data.yaw);
    }

    ekf_predict(dt);
    const fp32 gyro_yaw = quat_get_yaw(data.quat);
    data.ekf_updated = (data.accel_valid && ekf_update_accel()) ? 1U : 0U;
    if (data.ekf_updated)
    {
        /* Accel can correct tilt, but it cannot observe yaw; keep yaw from gyro prediction. */
        quat_keep_yaw_from_gyro(data.quat, gyro_yaw);
    }
    quat_normalize(data.quat);
    update_euler_from_quat();
}

void BMI088::ekf_predict(fp32 dt)
{
    const fp32 half_dt = 0.5f * dt;
    const fp32 gx = data.gyro[0];
    const fp32 gy = data.gyro[1];
    const fp32 gz = data.gyro[2];
    const fp32 q0 = data.quat[0];
    const fp32 q1 = data.quat[1];
    const fp32 q2 = data.quat[2];
    const fp32 q3 = data.quat[3];
    fp32 f[4][4] = {0.0f};
    fp32 fp[4][4] = {0.0f};
    fp32 p_new[4][4] = {0.0f};

    data.quat[0] += half_dt * (-q1 * gx - q2 * gy - q3 * gz);
    data.quat[1] += half_dt * ( q0 * gx + q2 * gz - q3 * gy);
    data.quat[2] += half_dt * ( q0 * gy - q1 * gz + q3 * gx);
    data.quat[3] += half_dt * ( q0 * gz + q1 * gy - q2 * gx);
    quat_normalize(data.quat);

    f[0][0] = 1.0f;
    f[0][1] = -half_dt * gx;
    f[0][2] = -half_dt * gy;
    f[0][3] = -half_dt * gz;
    f[1][0] = half_dt * gx;
    f[1][1] = 1.0f;
    f[1][2] = half_dt * gz;
    f[1][3] = -half_dt * gy;
    f[2][0] = half_dt * gy;
    f[2][1] = -half_dt * gz;
    f[2][2] = 1.0f;
    f[2][3] = half_dt * gx;
    f[3][0] = half_dt * gz;
    f[3][1] = half_dt * gy;
    f[3][2] = -half_dt * gx;
    f[3][3] = 1.0f;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++)
        {
            for (uint8_t k = 0U; k < 4U; k++)
            {
                fp[i][j] += f[i][k] * ekf_p[k][j];
            }
        }
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++)
        {
            for (uint8_t k = 0U; k < 4U; k++)
            {
                p_new[i][j] += fp[i][k] * f[j][k];
            }
        }
    }

    const fp32 q_scale = BMI088_EKF_GYRO_Q * (dt / BMI088_DEFAULT_DT_S);
    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++)
        {
            ekf_p[i][j] = 0.5f * (p_new[i][j] + p_new[j][i]);
        }
        ekf_p[i][i] = fp32_constrain(ekf_p[i][i] + q_scale, BMI088_EKF_MIN_P, BMI088_EKF_MAX_P);
    }
}

bool BMI088::ekf_update_accel()
{
    const fp32 accel_norm = sqrtf(data.accel[0] * data.accel[0] +
                                  data.accel[1] * data.accel[1] +
                                  data.accel[2] * data.accel[2]);
    if (accel_norm <= 1.0e-6f || !isfinite(accel_norm))
    {
        return false;
    }

    const fp32 z[3] = {
        data.accel[0] / accel_norm,
        data.accel[1] / accel_norm,
        data.accel[2] / accel_norm,
    };
    const fp32 q0 = data.quat[0];
    const fp32 q1 = data.quat[1];
    const fp32 q2 = data.quat[2];
    const fp32 q3 = data.quat[3];
    const fp32 h[3] = {
        2.0f * (q1 * q3 - q0 * q2),
        2.0f * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
    };
    const fp32 residual[3] = {
        z[0] - h[0],
        z[1] - h[1],
        z[2] - h[2],
    };
    const fp32 h_jacobian[3][4] = {
        {-2.0f * q2,  2.0f * q3, -2.0f * q0,  2.0f * q1},
        { 2.0f * q1,  2.0f * q0,  2.0f * q3,  2.0f * q2},
        { 2.0f * q0, -2.0f * q1, -2.0f * q2,  2.0f * q3},
    };
    fp32 ph_t[4][3] = {0.0f};
    fp32 s[3][3] = {0.0f};
    fp32 s_inv[3][3] = {0.0f};
    fp32 k_gain[4][3] = {0.0f};
    fp32 correction[4] = {0.0f};
    fp32 kh[4][4] = {0.0f};
    fp32 p_new[4][4] = {0.0f};

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            for (uint8_t k = 0U; k < 4U; k++)
            {
                ph_t[i][j] += ekf_p[i][k] * h_jacobian[j][k];
            }
        }
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            for (uint8_t k = 0U; k < 4U; k++)
            {
                s[i][j] += h_jacobian[i][k] * ph_t[k][j];
            }
        }
        s[i][i] += BMI088_EKF_ACC_R;
    }

    if (!invert_3x3(s, s_inv))
    {
        data.error_count++;
        return false;
    }

    data.accel_chi_square = 0.0f;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        fp32 weighted_residual = 0.0f;
        for (uint8_t j = 0U; j < 3U; j++)
        {
            weighted_residual += s_inv[i][j] * residual[j];
        }
        data.accel_chi_square += residual[i] * weighted_residual;
    }

    if (data.accel_chi_square > BMI088_EKF_ACC_CHI_GATE)
    {
        return false;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            for (uint8_t k = 0U; k < 3U; k++)
            {
                k_gain[i][j] += ph_t[i][k] * s_inv[k][j];
            }
            correction[i] += k_gain[i][j] * residual[j];
        }
        data.quat[i] += correction[i];
    }

    quat_normalize(data.quat);

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++)
        {
            for (uint8_t k = 0U; k < 3U; k++)
            {
                kh[i][j] += k_gain[i][k] * h_jacobian[k][j];
            }
        }
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++)
        {
            p_new[i][j] = ekf_p[i][j];
            for (uint8_t k = 0U; k < 4U; k++)
            {
                p_new[i][j] -= kh[i][k] * ekf_p[k][j];
            }
        }
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++)
        {
            ekf_p[i][j] = 0.5f * (p_new[i][j] + p_new[j][i]);
        }
        ekf_p[i][i] = fp32_constrain(ekf_p[i][i], BMI088_EKF_MIN_P, BMI088_EKF_MAX_P);
    }

    return true;
}

void BMI088::update_euler_from_quat()
{
    const fp32 q0 = data.quat[0];
    const fp32 q1 = data.quat[1];
    const fp32 q2 = data.quat[2];
    const fp32 q3 = data.quat[3];

    data.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));
    data.pitch = asinf(fp32_constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
    data.yaw = quat_get_yaw(data.quat);
}

bool BMI088::update(uint32_t now_ms)
{
    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};

    if (spi == NULL || !read_accel_raw_retry(accel_raw) || !read_gyro_raw_retry(gyro_raw))
    {
        data.online = 0U;
        data.accel_valid = 0U;
        data.gyro_valid = 0U;
        return false;
    }

    const fp32 accel_scale = bmi088_accel_scale();
    const fp32 gyro_scale = bmi088_gyro_scale();

    for (uint8_t i = 0U; i < 3U; i++)
    {
        data.accel[i] = (fp32)accel_raw[i] * accel_scale;
        data.gyro[i] = (fp32)gyro_raw[i] * gyro_scale + gyro_zero_offset[i];
    }

    data.accel_valid = validate_accel() ? 1U : 0U;
    data.gyro_valid = validate_gyro() ? 1U : 0U;
    if (!data.gyro_valid)
    {
        data.online = 0U;
        data.error_count++;
        return false;
    }

    const uint32_t update_us = now_us();
    fp32 dt = (fp32)(update_us - last_integrate_us) * 0.000001f;
    fusion_update(dt);

    data.last_update_ms = now_ms;
    data.last_update_us = update_us;
    data.update_count++;
    data.online = 1U;
    status.last_transfer_us = update_us;
    status.accel_ready = 0U;
    status.gyro_ready = 0U;
    last_integrate_us = update_us;
    return true;
}

void BMI088::reset_yaw(fp32 yaw)
{
    init_fusion_from_accel(rad_format(yaw));
    data.yaw = rad_format(yaw);
    last_integrate_us = now_us();
}

void BMI088::exti_callback(uint16_t gpio_pin)
{
    const uint32_t timestamp_us = now_us();

    if (gpio_pin == BMI088_ACCEL__INTERRUPT_Pin)
    {
        status.accel_ready = 1U;
        status.accel_ready_us = timestamp_us;
    }
    else if (gpio_pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        status.gyro_ready = 1U;
        status.gyro_ready_us = timestamp_us;
    }
}

const bmi088_data_t *BMI088::get_data_point() const
{
    return &data;
}

const bmi088_status_t *BMI088::get_status_point() const
{
    return &status;
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    bmi088.exti_callback(GPIO_Pin);
}
