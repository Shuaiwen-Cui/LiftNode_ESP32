# I2C 代码

## 组件架构

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
    注意，在驱动程序中，我们使用了 ESP-IDF 内置的 `driver` 库中的i2c和gpio相关函数，因此，我们需要在 `CMakeLists.txt` 文件的 `REQUIRES` 字段中指明这个依赖关系。

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
    请注意，本章需要与其他使用i2c的部件联合使用，才能看到效果。在使用基于i2c的外设时，请确保您已经正确配置和初始化了i2c总线。(例如在主程序中调用`i2c_bus_init()`函数)