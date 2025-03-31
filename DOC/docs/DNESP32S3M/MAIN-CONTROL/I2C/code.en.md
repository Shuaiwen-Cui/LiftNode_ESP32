# I2C CODE

## Component Architecture

```plaintext
- driver
    - i2c
        - include
            - i2c.h
        - i2c.c
        - CMakeLists.txt
```

## driver/spi/CMakeLists.txt

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
    log
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
```

!!! note
    Note that in the drivers, we used i2c and gpio related functions from the builtin `driver` library, therefore, we need to indicate these dependencies in the `CMakeLists.txt` file by adding `driver` to the `REQUIRES` field.

## spi.h
    
```c
/**
 * @file spi.h
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief
 * @version 1.0
 * @date 2024-11-18
 * @ref Alientek SPI driver
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __SPI_H__
#define __SPI_H__

/* Dependencies */
#include <string.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/* GPIO Definitions */
#define SPI2_MOSI_GPIO_PIN GPIO_NUM_11 /* SPI2_MOSI */
#define SPI2_CLK_GPIO_PIN GPIO_NUM_12  /* SPI2_CLK */
#define SPI2_MISO_GPIO_PIN GPIO_NUM_13 /* SPI2_MISO */

/* Function Prototypes */

/**
 * @brief       Initialize SPI
 * @param       None
 * @retval      None
 */
void spi2_init(void);

/**
 * @brief       Send command via SPI
 * @param       handle : SPI handle
 * @param       cmd    : Command to send
 * @retval      None
 */
void spi2_write_cmd(spi_device_handle_t handle, uint8_t cmd);

/**
 * @brief       Send data via SPI
 * @param       handle : SPI handle
 * @param       data   : Data to send
 * @param       len    : Length of data to send
 * @retval      None
 */
void spi2_write_data(spi_device_handle_t handle, const uint8_t *data, int len);

/**
 * @brief       Process data via SPI
 * @param       handle       : SPI handle
 * @param       data         : Data to send
 * @retval      t.rx_data[0] : Received data
 */
uint8_t spi2_transfer_byte(spi_device_handle_t handle, uint8_t byte);

#endif
```
## i2c.h
    
```c
/**
 * @file i2c.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This file contains the function prototypes for i2c master initialization. This is to serve the peripherals that require I2C communication.
 * @version 1.0
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __I2C_H__
#define __I2C_H__

#include <stdio.h>
#include "esp_log.h"
#include "unity.h" // This is for unity testing
#include "driver/i2c.h"
#include "esp_system.h"

#define I2C_MASTER_SCL_IO 4      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

/**
 * @brief i2c master initialization
 */
void i2c_bus_init(void);

#endif /* __I2C_H__ */
```

## i2c.c

```c
/**
 * @file i2c.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This file contains the function prototypes for i2c master initialization. This is to serve the peripherals that require I2C communication.
 * @version 1.0
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "i2c.h"

/**
 * @brief i2c master initialization
 */
void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}
```

## main.c

```c
Refer to the codes for i2c peripherals.
```

!!! note
    Please note that, the code in this chapter should be used in conjunction with other components that use the i2c to see the effect. Before you use the components based on i2c communication, you need to initialize the i2c bus by calling the `i2c_bus_init()` function in the `main.c` file.