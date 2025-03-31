# FILE SYSTEM (SPIFFS)

## Introduction

We have introduced how to drive the SD card, but the operations on the SD card are all direct or write binary data, which is not user-friendly. So we need a file system to manage files, so that users can operate files by file name, rather than by address.

The file system we use is SPIFFS, which is a file system designed for SPI flash devices. It is a lightweight file system that is designed for microcontrollers with a small amount of RAM and ROM. It is designed to work with SPI NOR flash devices, which are the most common type of flash memory in microcontrollers.

SPIFFS has the following features:

- **Small Footprint:** Designed for limited RAM environments, avoiding the need for heap allocation.
- **Erase Constraints:** Only large blocks of data can be erased at once.
- **Erase Behavior:** Erasing resets all bits in a block to `1`.
- **Write Behavior:** Write operations flip bits from `1` to `0`.
- **Erase to Reset:** Bits set to `0` can only be reset to `1` through an erase operation.
- **Wear Leveling:** Implements wear leveling to ensure balanced usage of flash memory.

- **Optimized for Low RAM Usage:** Designed to use a static RAM buffer size, independent of the number of files.
- **POSIX-like Interface:** Supports operations such as open, close, read, write, seek, and stat, resembling a portable operating system interface.
- **Wide Compatibility:** Runs on any NOR flash memory, not just SPI flash, and can theoretically operate on embedded flash in microprocessors.
- **Multiple Configurations:** Allows multiple SPIFFS configurations to run on the same target, even on the same SPI flash device.
- **Static Wear Leveling:** Maintains the lifespan of flash memory through static wear leveling.
- **Filesystem Integrity Checks:** Includes a built-in file system consistency checker.
- **Highly Configurable:** Offers extensive customization options.

!!! note "VFS"
    VFS (Virtual File System) is a component of ESP-IDF that provides a unified interface for different file systems. This allows various file systems to be operated through a single API. SPIFFS is one such implementation under VFS.

## Use Case

1. Create a `holle.txt` file in the specified area of the NOR flash, then perform read and write operations on this file.
2. Blink an LED to indicate the program is running.

## Resources Involved

1. **LED**
   - Pin: IO0
2. **0.96-inch LCD**
3. **SPIFFS**

## SPIFFS API

### Register and Mount SPIFFS

This function registers SPIFFS to the VFS (Virtual File System) and mounts it with a given path prefix. The function prototype is as follows:

```c
esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t * conf);
```

Function Parameter Description

| Parameter | Description |
| :--- | :--- |
| `conf` | Pointer to the configuration structure of type `esp_vfs_spiffs_conf_t` |

Return Values

| Return Value | Description |
| :--- | :--- |
| `ESP_OK` | Returns `0`, configuration successful |
| `ESP_ERR_NO_MEM` | If objects cannot be allocated |
| `ESP_ERR_INVALID_STATE` | If already mounted or the partition is encrypted |
| `ESP_ERR_NOT_FOUND` | If the SPIFFS partition cannot be found |
| `ESP_FAIL` | If mounting or formatting fails |

Configuration Structure Definition

The function uses a structure variable of type `esp_vfs_spiffs_conf_t` as its parameter. The structure is defined as follows:

| Member Variable            | Description                                                                 |
| :--------------------- | :------------------------------------------------------------------ |
| `base_path`            | File path prefix associated with the file system.                                |
| `partition_label`      | Optional, the label of the SPIFFS partition to use. If set to `NULL`, the default partition is used. |
| `max_files`            | Maximum number of files that can be opened simultaneously.                       |
| `format_if_mount_failed` | If `true`, formats the file system if mounting fails.                          |

### Retrieve SPIFFS Information

This function is used to retrieve information about SPIFFS. Its function prototype is as follows:

```c
esp_err_t esp_spiffs_info(const char* partition_label,
                          size_t *total_bytes,
                          size_t *used_bytes);
```

The function parameters are described as follows:

| Parameter         | Description                                           |
| :---              | :---                                                 |
| `partition_label` | Label of the SPIFFS partition                        |
| `total_bytes`     | Pointer to a variable that stores the total byte count |
| `used_bytes`      | Pointer to a variable that stores the used byte count |

Return Values:

- `ESP_OK`: Success
- Other: Failure

### Unregister and Unmount SPIFFS

This function unregisters SPIFFS from the VFS and unmounts it. The function prototype is as follows:

```c
esp_err_t esp_vfs_spiffs_unregister(const char* partition_label);
```

The function parameters are described as follows:

| Parameter         | Description                                     |
| :---              | :---                                           |
| `partition_label` | Pointer to the partition table, partition name |

Return Values:

- `ESP_OK`: Success
- Others: Failure

## Driver Code

### spiffs.h

```c
/**
 * @file fs.h
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This file wraps up some frequently used functions for file system operations.
 * @version 1.0
 * @date 2024-11-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/* Dependencies */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "led.h"
#include "spi.h"
#include "lcd.h"
// #include "esp_spiffs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include <sys/stat.h>
#include "esp_log.h"


/* Dependencies */
// Basic
#include "esp_system.h"
#include "esp_chip_info.h"
// #include "esp_psram.h"
// #include "esp_flash.h"
// #include "nvs_flash.h"
#include "spi_sdcard.h"

/* Definitions */
#define DEFAULT_FD_NUM          5
#define DEFAULT_MOUNT_POINT     "/spiffs"
#define WRITE_DATA              "ESP32-S3\r\n"

/**
 * @brief       SPIFFS initialization
 * @param       partition_label: The name of the partition in the partition table
 * @param       mount_point: The path prefix associated with the file system
 * @param       max_files: Maximum number of files that can be opened simultaneously
 * @retval      None
 */
esp_err_t spiffs_init(char *partition_label, char *mount_point, size_t max_files);

/**
 * @brief       Deinitialize SPIFFS
 * @param       partition_label: Partition label
 * @retval      None
 */
esp_err_t spiffs_deinit(char *partition_label);

/**
 * @brief       Test SPIFFS functionality
 * @param       None
 * @retval      None
 */
void spiffs_test(void);

```

### spiffs.c

```c
/**
 * @file fs.c
 * @author 
 * @brief This file wraps up some frequently used functions for file system operations.
 * @version 1.0
 * @date 2024-11-19
 * 
 */

#include "spiffs.h"

/* Variables */
const char               *TAG = "spiffs";

/**
 * @brief       Initialize SPIFFS
 * @param       partition_label: Partition label in the partition table
 * @param       mount_point: Prefix path associated with the file system
 * @param       max_files: Maximum number of files that can be opened simultaneously
 * @retval      None
 */
esp_err_t spiffs_init(char *partition_label, char *mount_point, size_t max_files)
{
    /* Configure parameters for the SPIFFS file system */
    esp_vfs_spiffs_conf_t conf = {
        .base_path = mount_point,
        .partition_label = partition_label,
        .max_files = max_files,
        .format_if_mount_failed = true,
    };

    /* Initialize and mount the SPIFFS file system using the above configuration */
    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    /* Check if SPIFFS mount and initialization were successful */
    if (ret_val != ESP_OK)
    {
        if (ret_val == ESP_FAIL)
        {
            printf("Failed to mount or format filesystem\n");
        }
        else if (ret_val == ESP_ERR_NOT_FOUND)
        {
            printf("Failed to find SPIFFS partition\n");
        }
        else
        {
            printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret_val));
        }

        return ESP_FAIL;
    }

    /* Print SPIFFS storage information */
    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);

    if (ret_val != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

/**
 * @brief       Unregister SPIFFS
 * @param       partition_label: Partition label in the partition table
 * @retval      None
 */
esp_err_t spiffs_deinit(char *partition_label)
{
    return esp_vfs_spiffs_unregister(partition_label);
}

/**
 * @brief       Test SPIFFS
 * @param       None
 * @retval      None
 */
void spiffs_test(void)
{

    ESP_LOGI(TAG, "Opening file");
    /* Create a write-only file named /spiffs/hello.txt */
    FILE* f = fopen("/spiffs/hello.txt", "w");

    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    /* Write data to the file */
    fprintf(f, WRITE_DATA);

    fclose(f);
    ESP_LOGI(TAG, "File written");
 
    /* Check if the target file exists before renaming */
    struct stat st;

    if (stat("/spiffs/foo.txt", &st) == 0) /* Get file information, returns 0 on success */
    {
        /* Remove a file name from the file system. 
           If the name is the last link to a file and no other processes have it open,
           the file is deleted. */
        unlink("/spiffs/foo.txt");
    }
 
    /* Rename the created file */
    ESP_LOGI(TAG, "Renaming file");

    if (rename("/spiffs/hello.txt", "/spiffs/foo.txt") != 0)
    {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }
 
    /* Open the renamed file and read it */
    ESP_LOGI(TAG, "Reading file");
    f = fopen("/spiffs/foo.txt", "r");

    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    
    char* pos = strchr(line, '\n'); /* Pointer pos points to the first occurrence of '\n' */

    if (pos)
    {
        *pos = '\0';                /* Replace '\n' with '\0' */
    }

    ESP_LOGI(TAG, "Read from file: %s", line);

    lcd_show_string(90, 20, 200, 16, 16, line, RED);
}

```

!!! note
    在CMakeLists.txt中添加源文件和头文件，同时增加依赖项spiffs.c

```cmake
set(requires
            driver
            fatfs
            esp_timer
            newlib
            spiffs
            )
```

### main.c

```c
/**
 * @file main.c
 * @author
 * @brief Main application to demonstrate the use of ESP32 internal temperature sensor
 * @version 1.0
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Dependencies */
// Basic
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <sys/stat.h>

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// BSP
#include "led.h"
#include "rgb.h"
#include "key.h"
#include "exit.h"
#include "lcd.h"
#include "spi.h"
#include "esp_rtc.h"
#include "temp.h"
#include "rng.h"
#include "spi_sdcard.h"
#include "spiffs.h"

/**
 * @brief       Program entry point
 * @param       None
 * @retval      None
 */
void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();                                         /* Initialize NVS */
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    led_init();                                                     /* Initialize LED */
    spi2_init();                                                    /* Initialize SPI */
    lcd_init();                                                     /* Initialize LCD */
    spiffs_init("storage", DEFAULT_MOUNT_POINT, DEFAULT_FD_NUM);    /* Initialize SPIFFS */

    /* Display test information */
    lcd_show_string(0, 0, 200, 16, 16, "SPIFFS TEST", RED);
    lcd_show_string(0, 20, 200, 16, 16, "Read file:", BLUE);

    spiffs_test();                                                  /* Run SPIFFS test */

    while (1)
    {
        led_toggle();
        vTaskDelay(500);
    }
}

```

