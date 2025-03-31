# ESP_RTC CODE

!!! warning
    Looks there is a builtin file named `rtc.h` in the ESP-IDF, so we need to name our file as `esp_rtc.h` to avoid conflicts. So as to the `rtc.c` file.

## Component Architecture

```plaintext
- driver
    - esp_rtc
        - include
            - esp_rtc.h
        - esp_rtc.c
        - CMakeLists.txt
```

## driver/esp_rtc/CMakeLists.txt

```cmake
set(src_dirs
    .
)

set(include_dirs
    include
)

set(requires
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
```

## esp_rtc.h
    
```c
/**
 * @file esp_rtc.h
 * ! Do not name as rtc.h, as it will conflict with the built-in rtc.h
 * @author
 * @brief Header file for the RTC driver
 * @version 1.0
 * @date 2024-11-18
 * @ref Alienteck RTC Driver
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ESP_RTC_H__
#define __ESP_RTC_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/time.h>

/* Time structure, including year, month, day, week, hour, minute, and second */
typedef struct
{
    uint8_t hour; /* Hour */
    uint8_t min;  /* Minute */
    uint8_t sec;  /* Second */
    /* Gregorian year, month, day, and week */
    uint16_t year; /* Year */
    uint8_t month; /* Month */
    uint8_t date;  /* Day */
    uint8_t week;  /* Weekday */
} _calendar_obj;

extern _calendar_obj calendar; /* Time structure */

/* Function declarations */
/**
 * @brief       Set the RTC time
 * @param       year    : Year
 * @param       mon     : Month
 * @param       mday    : Day
 * @param       hour    : Hour
 * @param       min     : Minute
 * @param       sec     : Second
 * @retval      None
 */
void rtc_set_time(int year, int mon, int mday, int hour, int min, int sec); /* Set time */

/**
 * @brief       Get the current time
 * @param       None
 * @retval      None
 */
void rtc_get_time(void); /* Get time */

/**
 * @brief       Convert year, month, and day to the day of the week
 * @note        Calculates the weekday based on the Gregorian calendar.
 *              Utilizes the Kim Larson formula for calculation.
 *              For more details, refer to:
 *              https://www.cnblogs.com/fengbohello/p/3264300.html
 * @param       year : Year
 * @param       month: Month
 * @param       day  : Day
 * @retval      0: Sunday; 1 ~ 6: Monday ~ Saturday
 */
uint8_t rtc_get_week(uint16_t year, uint8_t month, uint8_t day); /* Get the weekday */

#endif /* __ESP_RTC_H__ */
```

## esp_rtc.c

```c
/**
 * @file esp_rtc.c
 * ! Do not name as rtc.c, as it will conflict with the built-in rtc.c
 * @author
 * @brief This file contains the implementation of the RTC driver
 * @version 1.0
 * @date 2024-11-18
 * @ref Alienteck RTC Driver
 *
 */

#include "esp_rtc.h"

_calendar_obj calendar; /* Time structure */

/**
 * @brief       Set the RTC time
 * @param       year    : Year
 * @param       mon     : Month
 * @param       mday    : Day
 * @param       hour    : Hour
 * @param       min     : Minute
 * @param       sec     : Second
 * @retval      None
 */
void rtc_set_time(int year, int mon, int mday, int hour, int min, int sec)
{
    struct tm datetime;
    /* Set time */
    datetime.tm_year = year - 1900;
    datetime.tm_mon = mon - 1;
    datetime.tm_mday = mday;
    datetime.tm_hour = hour;
    datetime.tm_min = min;
    datetime.tm_sec = sec;
    datetime.tm_isdst = -1;
    /* Get total seconds since 1970-01-01 */
    time_t second = mktime(&datetime);
    struct timeval val = {.tv_sec = second, .tv_usec = 0};
    /* Set current time */
    settimeofday(&val, NULL);
}

/**
 * @brief       Get the current time
 * @param       None
 * @retval      None
 */
void rtc_get_time(void)
{
    struct tm *datetime;
    time_t second;
    /* Get the time elapsed since (1970-01-01 00:00:00 UTC) in seconds */
    time(&second);
    datetime = localtime(&second);

    calendar.hour = datetime->tm_hour; /* Hour */
    calendar.min = datetime->tm_min;   /* Minute */
    calendar.sec = datetime->tm_sec;   /* Second */
    /* Gregorian year, month, day, and week */
    calendar.year = datetime->tm_year + 1900; /* Year */
    calendar.month = datetime->tm_mon + 1;    /* Month */
    calendar.date = datetime->tm_mday;        /* Day */
    /* Weekday */
    calendar.week = rtc_get_week(calendar.year, calendar.month, calendar.date);
}

/**
 * @brief       Convert year, month, and day to the day of the week
 * @note        Calculates the weekday based on the Gregorian calendar.
 *              Utilizes the Kim Larson formula for calculation.
 *              For more details, refer to:
 *              https://www.cnblogs.com/fengbohello/p/3264300.html
 * @param       year : Year
 * @param       month: Month
 * @param       day  : Day
 * @retval      0: Sunday; 1 ~ 6: Monday ~ Saturday
 */
uint8_t rtc_get_week(uint16_t year, uint8_t month, uint8_t day)
{
    uint8_t week = 0;

    if (month < 3)
    {
        month += 12;
        --year;
    }

    week = (day + 1 + 2 * month + 3 * (month + 1) / 5 + year + (year >> 2) - year / 100 + year / 400) % 7;
    return week;
}
```

## main.c

```c
/**
 * @file main.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief
 * @version 1.0
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Dependencies */
// Basic
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_log.h"

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// BSP
#include "led.h"
#include "exit.h"
#include "esp_rtc.h"

/* Global variables */
char *weekdays[] = {"Sunday", "Monday", "Tuesday", "Wednesday",
                    "Thursday", "Friday", "Saterday"};

/**
 * @brief Entry point of the program
 * @param None
 * @retval None
 */
void app_main(void)
{
    esp_err_t ret;
    uint8_t tbuf[40];
    uint8_t t = 0;

    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    led_init();
    rtc_set_time(2025, 02, 18, 22, 23, 00);

    while (1)
    {
        t++;

        if ((t % 10) == 0)
        {
            rtc_get_time();
            sprintf((char *)tbuf, "Time:%02d:%02d:%02d", calendar.hour, calendar.min, calendar.sec);
            printf("Time:%02d:%02d:%02d\r\n", calendar.hour, calendar.min, calendar.sec);
            sprintf((char *)tbuf, "Date:%04d-%02d-%02d", calendar.year, calendar.month, calendar.date);
            printf("Date:%02d-%02d-%02d\r\n", calendar.year, calendar.month, calendar.date);
            sprintf((char *)tbuf, "Week:%s", weekdays[calendar.week - 1]);
        }

        if ((t % 20) == 0)
        {
            led_toggle();
        }

        vTaskDelay(10);
    }
}
```