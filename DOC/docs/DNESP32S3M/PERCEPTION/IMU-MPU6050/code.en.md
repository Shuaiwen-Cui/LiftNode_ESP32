# MPU6050 CODE

## Component Architecture

```plaintext
- driver
    - mpu6050
        - include
            - mpu6050.h
        - mpu6050.c
        - CMakeLists.txt
```

## driver/rng/CMakeLists.txt

```cmake
set(src_dirs
    .
)

set(include_dirs
    include
)

set(requires
    driver
    unity
    i2c
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
```

## mpu6050.h
    
```c
/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MPU6050 driver
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "i2c.h"

#define MPU6050_I2C_ADDRESS 0x68u   /*!< I2C address with AD0 pin low */
#define MPU6050_I2C_ADDRESS_1 0x69u /*!< I2C address with AD0 pin high */
#define MPU6050_WHO_AM_I_VAL 0x68u

    typedef enum
    {
        ACCE_FS_2G = 0,  /*!< Accelerometer full scale range is +/- 2g */
        ACCE_FS_4G = 1,  /*!< Accelerometer full scale range is +/- 4g */
        ACCE_FS_8G = 2,  /*!< Accelerometer full scale range is +/- 8g */
        ACCE_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
    } mpu6050_acce_fs_t;

    typedef enum
    {
        GYRO_FS_250DPS = 0,  /*!< Gyroscope full scale range is +/- 250 degree per sencond */
        GYRO_FS_500DPS = 1,  /*!< Gyroscope full scale range is +/- 500 degree per sencond */
        GYRO_FS_1000DPS = 2, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
        GYRO_FS_2000DPS = 3, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    } mpu6050_gyro_fs_t;

    typedef enum
    {
        INTERRUPT_PIN_ACTIVE_HIGH = 0, /*!< The mpu6050 sets its INT pin HIGH on interrupt */
        INTERRUPT_PIN_ACTIVE_LOW = 1   /*!< The mpu6050 sets its INT pin LOW on interrupt */
    } mpu6050_int_pin_active_level_t;

    typedef enum
    {
        INTERRUPT_PIN_PUSH_PULL = 0, /*!< The mpu6050 configures its INT pin as push-pull */
        INTERRUPT_PIN_OPEN_DRAIN = 1 /*!< The mpu6050 configures its INT pin as open drain*/
    } mpu6050_int_pin_mode_t;

    typedef enum
    {
        INTERRUPT_LATCH_50US = 0,         /*!< The mpu6050 produces a 50 microsecond pulse on interrupt */
        INTERRUPT_LATCH_UNTIL_CLEARED = 1 /*!< The mpu6050 latches its INT pin to its active level, until interrupt is cleared */
    } mpu6050_int_latch_t;

    typedef enum
    {
        INTERRUPT_CLEAR_ON_ANY_READ = 0,   /*!< INT_STATUS register bits are cleared on any register read */
        INTERRUPT_CLEAR_ON_STATUS_READ = 1 /*!< INT_STATUS register bits are cleared only by reading INT_STATUS value*/
    } mpu6050_int_clear_t;

    typedef struct
    {
        gpio_num_t interrupt_pin;                     /*!< GPIO connected to mpu6050 INT pin       */
        mpu6050_int_pin_active_level_t active_level;  /*!< Active level of mpu6050 INT pin         */
        mpu6050_int_pin_mode_t pin_mode;              /*!< Push-pull or open drain mode for INT pin*/
        mpu6050_int_latch_t interrupt_latch;          /*!< The interrupt pulse behavior of INT pin */
        mpu6050_int_clear_t interrupt_clear_behavior; /*!< Interrupt status clear behavior         */
    } mpu6050_int_config_t;

    extern const uint8_t MPU6050_DATA_RDY_INT_BIT;      /*!< DATA READY interrupt bit               */
    extern const uint8_t MPU6050_I2C_MASTER_INT_BIT;    /*!< I2C MASTER interrupt bit               */
    extern const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT; /*!< FIFO Overflow interrupt bit            */
    extern const uint8_t MPU6050_MOT_DETECT_INT_BIT;    /*!< MOTION DETECTION interrupt bit         */
    extern const uint8_t MPU6050_ALL_INTERRUPTS;        /*!< All interrupts supported by mpu6050    */

    typedef struct
    {
        int16_t raw_acce_x;
        int16_t raw_acce_y;
        int16_t raw_acce_z;
    } mpu6050_raw_acce_value_t;

    typedef struct
    {
        int16_t raw_gyro_x;
        int16_t raw_gyro_y;
        int16_t raw_gyro_z;
    } mpu6050_raw_gyro_value_t;

    typedef struct
    {
        float acce_x;
        float acce_y;
        float acce_z;
    } mpu6050_acce_value_t;

    typedef struct
    {
        float gyro_x;
        float gyro_y;
        float gyro_z;
    } mpu6050_gyro_value_t;

    typedef struct
    {
        float temp;
    } mpu6050_temp_value_t;

    typedef struct
    {
        float roll;
        float pitch;
    } complimentary_angle_t;

    typedef void *mpu6050_handle_t;

    extern mpu6050_handle_t mpu6050;

    typedef gpio_isr_t mpu6050_isr_t;

    /**
     * @brief Create and init sensor object and return a sensor handle
     *
     * @param port I2C port number
     * @param dev_addr I2C device address of sensor
     *
     * @return
     *     - NULL Fail
     *     - Others Success
     */
    mpu6050_handle_t mpu6050_create(i2c_port_t port, const uint16_t dev_addr);

    /**
     * @brief Delete and release a sensor object
     *
     * @param sensor object handle of mpu6050
     */
    void mpu6050_delete(mpu6050_handle_t sensor);

    /**
     * @brief Get device identification of MPU6050
     *
     * @param sensor object handle of mpu6050
     * @param deviceid a pointer of device ID
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid);

    /**
     * @brief Wake up MPU6050
     *
     * @param sensor object handle of mpu6050
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor);

    /**
     * @brief Enter sleep mode
     *
     * @param sensor object handle of mpu6050
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_sleep(mpu6050_handle_t sensor);

    /**
     * @brief Set accelerometer and gyroscope full scale range
     *
     * @param sensor object handle of mpu6050
     * @param acce_fs accelerometer full scale range
     * @param gyro_fs gyroscope full scale range
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);

    /**
     * @brief Get accelerometer sensitivity
     *
     * @param sensor object handle of mpu6050
     * @param acce_sensitivity accelerometer sensitivity
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity);

    /**
     * @brief Get gyroscope sensitivity
     *
     * @param sensor object handle of mpu6050
     * @param gyro_sensitivity gyroscope sensitivity
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity);

    /**
     * @brief Configure INT pin behavior and setup target GPIO.
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_configuration mpu6050 INT pin configuration parameters
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or incorrect
     *      - ESP_FAIL Failed to configure INT pin on mpu6050
     */
    esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration);

    /**
     * @brief Register an Interrupt Service Routine to handle mpu6050 interrupts.
     *
     * @param sensor object handle of mpu6050
     * @param isr function to handle interrupts produced by mpu6050
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to register ISR
     */
    esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr);

    /**
     * @brief Enable specific interrupts from mpu6050
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_sources bit mask with interrupt sources to enable
     *
     * This function does not disable interrupts not set in interrupt_sources. To disable
     * specific mpu6050 interrupts, use mpu6050_disable_interrupts().
     *
     * To enable all mpu6050 interrupts, pass MPU6050_ALL_INTERRUPTS as the argument
     * for interrupt_sources.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
     */
    esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);

    /**
     * @brief Disable specific interrupts from mpu6050
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_sources bit mask with interrupt sources to disable
     *
     * This function does not enable interrupts not set in interrupt_sources. To enable
     * specific mpu6050 interrupts, use mpu6050_enable_interrupts().
     *
     * To disable all mpu6050 interrupts, pass MPU6050_ALL_INTERRUPTS as the
     * argument for interrupt_sources.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
     */
    esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);

    /**
     * @brief Get the interrupt status of mpu6050
     *
     * @param sensor object handle of mpu6050
     * @param out_intr_status[out] bit mask that is assigned a value representing the interrupts triggered by the mpu6050
     *
     * This function can be used by the mpu6050 ISR to determine the source of
     * the mpu6050 interrupt that it is handling.
     *
     * After this function returns, the bits set in out_intr_status are
     * the sources of the latest interrupt triggered by the mpu6050. For example,
     * if MPU6050_DATA_RDY_INT_BIT is set in out_intr_status, the last interrupt
     * from the mpu6050 was a DATA READY interrupt.
     *
     * The behavior of the INT_STATUS register of the mpu6050 may change depending on
     * the value of mpu6050_int_clear_t used on interrupt configuration.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to retrieve interrupt status from mpu6050
     */
    esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status);

    /**
     * @brief Determine if the last mpu6050 interrupt was due to data ready.
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt was not produced due to data ready
     *      - Any other positive integer: Interrupt was a DATA_READY interrupt
     */
    extern uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status);

    /**
     * @brief Determine if the last mpu6050 interrupt was an I2C master interrupt.
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt is not an I2C master interrupt
     *      - Any other positive integer: Interrupt was an I2C master interrupt
     */
    extern uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status);

    /**
     * @brief Determine if the last mpu6050 interrupt was triggered by a fifo overflow.
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt is not a fifo overflow interrupt
     *      - Any other positive integer: Interrupt was triggered by a fifo overflow
     */
    extern uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status);

    /**
     * @brief Read raw accelerometer measurements
     *
     * @param sensor object handle of mpu6050
     * @param raw_acce_value raw accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value);

    /**
     * @brief Read raw gyroscope measurements
     *
     * @param sensor object handle of mpu6050
     * @param raw_gyro_value raw gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value);

    /**
     * @brief Read accelerometer measurements
     *
     * @param sensor object handle of mpu6050
     * @param acce_value accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value);

    /**
     * @brief Read gyro values
     *
     * @param sensor object handle of mpu6050
     * @param gyro_value gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value);

    /**
     * @brief Read temperature values
     *
     * @param sensor object handle of mpu6050
     * @param temp_value temperature measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value);

    /**
     * @brief Use complimentory filter to calculate roll and pitch
     *
     * @param sensor object handle of mpu6050
     * @param acce_value accelerometer measurements
     * @param gyro_value gyroscope measurements
     * @param complimentary_angle complimentary angle
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                           const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle);

    /**
     * @brief i2c master initialization
     * @note Must do i2c_bus_init() before calling this function, which is provided by i2c.c
     */
    void i2c_sensor_mpu6050_init(void);

#ifdef __cplusplus
}
#endif

```

## mpu6050.c

```c
/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include "mpu6050.h"

#define ALPHA 0.99f             /*!< Weight of gyroscope */
#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

/* MPU6050 register */
#define MPU6050_GYRO_CONFIG 0x1Bu
#define MPU6050_ACCEL_CONFIG 0x1Cu
#define MPU6050_INTR_PIN_CFG 0x37u
#define MPU6050_INTR_ENABLE 0x38u
#define MPU6050_INTR_STATUS 0x3Au
#define MPU6050_ACCEL_XOUT_H 0x3Bu
#define MPU6050_GYRO_XOUT_H 0x43u
#define MPU6050_TEMP_XOUT_H 0x41u
#define MPU6050_PWR_MGMT_1 0x6Bu
#define MPU6050_WHO_AM_I 0x75u

const uint8_t MPU6050_DATA_RDY_INT_BIT = (uint8_t)BIT0;
const uint8_t MPU6050_I2C_MASTER_INT_BIT = (uint8_t)BIT3;
const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT = (uint8_t)BIT4;
const uint8_t MPU6050_MOT_DETECT_INT_BIT = (uint8_t)BIT6;
const uint8_t MPU6050_ALL_INTERRUPTS = (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

mpu6050_handle_t mpu6050 = NULL;

typedef struct
{
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt; /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} mpu6050_dev_t;

static esp_err_t mpu6050_write(mpu6050_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mpu6050_read(mpu6050_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

mpu6050_handle_t mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
{
    mpu6050_dev_t *sensor = (mpu6050_dev_t *)calloc(1, sizeof(mpu6050_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
    return (mpu6050_handle_t)sensor;
}

void mpu6050_delete(mpu6050_handle_t sensor)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    free(sens);
}

esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid)
{
    return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret)
    {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret)
    {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs)
    {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs)
    {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration)
{
    esp_err_t ret = ESP_OK;

    if (NULL == interrupt_configuration)
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin))
    {
        // Set GPIO connected to MPU6050 INT pin only when user configures interrupts.
        mpu6050_dev_t *sensor_device = (mpu6050_dev_t *)sensor;
        sensor_device->int_pin = interrupt_configuration->interrupt_pin;
    }
    else
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    uint8_t int_pin_cfg = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level)
    {
        int_pin_cfg |= BIT7;
    }

    if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode)
    {
        int_pin_cfg |= BIT6;
    }

    if (INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch)
    {
        int_pin_cfg |= BIT5;
    }

    if (INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior)
    {
        int_pin_cfg |= BIT4;
    }

    ret = mpu6050_write(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    gpio_int_type_t gpio_intr_type;

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level)
    {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    }
    else
    {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin)};

    ret = gpio_config(&int_gpio_config);

    return ret;
}

esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr)
{
    esp_err_t ret;
    mpu6050_dev_t *sensor_device = (mpu6050_dev_t *)sensor;

    if (NULL == sensor_device)
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = gpio_isr_handler_add(
        sensor_device->int_pin,
        ((gpio_isr_t) * (isr)),
        ((void *)sensor));

    if (ESP_OK != ret)
    {
        return ret;
    }

    ret = gpio_intr_enable(sensor_device->int_pin);

    return ret;
}

esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    if (enabled_interrupts != interrupt_sources)
    {

        enabled_interrupts |= interrupt_sources;

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    if (0 != (enabled_interrupts & interrupt_sources))
    {
        enabled_interrupts &= (~interrupt_sources);

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status)
{
    esp_err_t ret;

    if (NULL == out_intr_status)
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = mpu6050_read(sensor, MPU6050_INTR_STATUS, out_intr_status, 1);

    return ret;
}

inline uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status)
{
    return (MPU6050_DATA_RDY_INT_BIT == (MPU6050_DATA_RDY_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status)
{
    return (uint8_t)(MPU6050_I2C_MASTER_INT_BIT == (MPU6050_I2C_MASTER_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status)
{
    return (uint8_t)(MPU6050_FIFO_OVERFLOW_INT_BIT == (MPU6050_FIFO_OVERFLOW_INT_BIT & interrupt_status));
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = mpu6050_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK)
    {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK)
    {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;

    sens->counter++;
    if (sens->counter == 1)
    {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);

    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;

    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}

/**
 * @brief i2c master initialization
 * @note Must do i2c_bus_init() before calling this function, which is provided by i2c.c
 */
void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

```

## main.c

```c
/**
 * @file main.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief
 * @version 1.0
 * @date 2025-03-18
 *
 * @copyright Copyright (c) 2025
 *
 */

/* DEPENDENCIES */
// ESP
#include "esp_system.h"    // ESP32 System
#include "nvs_flash.h"     // ESP32 NVS
#include "esp_chip_info.h" // ESP32 Chip Info
#include "esp_psram.h"     // ESP32 PSRAM
#include "esp_flash.h"     // ESP32 Flash
#include "esp_log.h"       // ESP32 Logging

// BSP
#include "led.h"
#include "exit.h"
#include "spi.h"
#include "i2c.h"
#include "lcd.h"
#include "tim.h"
#include "esp_rtc.h"
#include "spi_sdcard.h"
#include "wifi_wpa2_enterprise.h"
#include "mqtt.h"
#include "mpu6050.h"

/* Variables */
const char *TAG = "NEXNODE";

/**
 * @brief Entry point of the program
 * @param None
 * @retval None
 */
void app_main(void)
{
    esp_err_t ret;
    uint32_t flash_size;
    esp_chip_info_t chip_info;

    char mqtt_pub_buff[64];
    int count = 0;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase if needed
        ret = nvs_flash_init();
    }

    // Get FLASH size
    esp_flash_get_size(NULL, &flash_size);
    esp_chip_info(&chip_info);

    // Display CPU core count
    printf("CPU Cores: %d\n", chip_info.cores);

    // Display FLASH size
    printf("Flash size: %ld MB flash\n", flash_size / (1024 * 1024));

    // Display PSRAM size
    printf("PSRAM size: %d bytes\n", esp_psram_get_size());

    // BSP Initialization
    led_init();
    exit_init();
    spi2_init();
    lcd_init();
    i2c_bus_init();
    i2c_sensor_mpu6050_init();

    // spiffs_test();                                                  /* Run SPIFFS test */
    while (sd_card_init()) /* SD card not detected */
    {
        lcd_show_string(0, 0, 200, 16, 16, "SD Card Error!", RED);
        vTaskDelay(500);
        lcd_show_string(0, 20, 200, 16, 16, "Please Check!", RED);
        vTaskDelay(500);
    }

    // clean the screen
    lcd_clear(WHITE);

    lcd_show_string(0, 0, 200, 16, 16, "SD Initialized!", RED);

    sd_card_test_filesystem(); /* Run SD card test */

    lcd_show_string(0, 0, 200, 16, 16, "SD Tested CSW! ", RED);

    // sd_card_unmount();

    vTaskDelay(3000);

    lcd_show_string(0, 0, lcd_self.width, 16, 16, "WiFi STA Test  ", RED);

    ret = wifi_sta_wpa2_init();
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_WIFI, "WiFi STA Init OK");
        lcd_show_string(0, 0, lcd_self.width, 16, 16, "WiFi STA Test OK", RED);
    }
    else
    {
        ESP_LOGE(TAG_WIFI, "WiFi STA Init Failed");
    }

    // only when the ip is obtained, start mqtt
    EventBits_t ev = 0;
    ev = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    if (ev & CONNECTED_BIT)
    {
        mqtt_app_start();
    }

    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    complimentary_angle_t angle;

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

    while (1)
    {
        // led test
        led_toggle();

        // hellow world test
        ESP_LOGI(TAG, "Hello World!");

        // mpu6050 test
        ret = mpu6050_get_acce(mpu6050, &acce);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "acce_x:%.6f, acce_y:%.6f, acce_z:%.6f\n", acce.acce_x, acce.acce_y, acce.acce_z);

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "gyro_x:%.6f, gyro_y:%.6f, gyro_z:%.6f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        ret = mpu6050_get_temp(mpu6050, &temp);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "t:%.6f \n", temp.temp);

        ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &angle);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "pitch:%.6f roll:%.6f \n", angle.pitch, angle.roll);

        // mqtt test publish acc data
        if (s_is_mqtt_connected)
        {
            snprintf(mqtt_pub_buff, 64, "{\"acce_x\":\"%.6f\",\"acce_y\":\"%.6f\",\"acce_z\":\"%.6f\"}", acce.acce_x, acce.acce_y, acce.acce_z);
            esp_mqtt_client_publish(s_mqtt_client, MQTT_PUBLIC_TOPIC,
                                    mqtt_pub_buff, strlen(mqtt_pub_buff), 1, 0);
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

```
