# 代码

## 组件架构

```plaintext
- driver
    - wifi
        - include
            - wifi_wpa2_enterprise.h
        - wifi_wpa2_enterprise.c
        - CMakeLists.txt
```

## driver/wifi/CMakeLists.txt

```cmake
set(src_dirs
    .
)

set(include_dirs
    include
)

set(requires
    led
    lcd
    esp_wifi
    wpa_supplicant
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
```

!!! tip
    注意将下文代码中的用户名和密码替换为您的用户名和密码。

## wifi_wpa2_enterprise.h

```c
/**
 * @file wifi_wpa2_enterprise.h
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This file contains the function prototypes for wifi connection using WPA2 enterprise.
 * @version 1.0
 * @date 2025-02-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __WIFI_WPA2_ENTERPRISE_H__
#define __WIFI_WPA2_ENTERPRISE_H__

/* Dependencies */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
// #include "esp_wpa2.h"
#include "esp_eap_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"

/* Macros */
#define ENTERPRISE_WIFI_SSID "NTUSECURE" //SSID of WiFi
#define ENTERPRISE_WIFI_USERNAME "YOUR USER NAME" // Username
#define ENTERPRISE_WIFI_PASSWORD "YOUR PASSWORD" // Password

/* Variables */
extern const char *TAG_WIFI; // tag for logging
extern EventGroupHandle_t wifi_event_group;
extern const int CONNECTED_BIT;

/* Function Prototypes */
/**
 * @name wifi_sta_wpa2_init
 * @brief Initialize the WIFI station to connect to a WPA2 Enterprise network
 * @param void
 * @return esp_err_t 
 */
esp_err_t wifi_sta_wpa2_init(void);

#endif /* __WIFI_WPA2_ENTERPRISE_H__ */
```

## wifi_wpa2_enterprise.c

```c
/**
 * @file wifi_wpa2_enterprise.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This file contains the functions to connect to a WPA2 Enterprise network.
 * @version 1.0
 * @date 2025-02-24
 *
 * @copyright Copyright (c) 2025
 *
 */

/* Dependencies */
#include "wifi_wpa2_enterprise.h"

/* Variables */
const char *TAG_WIFI = "CSW-WIFI"; // tag for logging
EventGroupHandle_t wifi_event_group; // FreeRTOS event group to signal when we are connected & ready to make a request
esp_netif_t *sta_netif = NULL;       // esp netif object representing the WIFI station
const int CONNECTED_BIT = BIT0;      // use bit 0 to indicate whether connected

/**
 * @name wifi_event_handler
 * @brief Event handler for WIFI and IP events
 * @param arg argument passed to the handler
 * @param event_base base of the event
 * @param event_id id of the event
 * @param event_data data of the event
 * @return void
 *
 */
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();                                    
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT); // clear the connected bit
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT); // only when the ip is obtained, set the connected bit
    }
}

/**
 * @name wifi_sta_wpa2_init
 * @brief Initialize the WIFI station to connect to a WPA2 Enterprise network
 * @param void
 * @return esp_err_t
 */
esp_err_t wifi_sta_wpa2_init(void)
{
    esp_err_t ret;

    /* Preparation */
    ESP_ERROR_CHECK(esp_netif_init()); // TCP/IP stack initialization
    wifi_event_group = xEventGroupCreate(); // create the event group
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // create the event group, later can be used to deal with a series of events by registing call back functions
    sta_netif = esp_netif_create_default_wifi_sta(); // creat STA object using default configuration
    assert(sta_netif);

    /* WiFi Configuration*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // use default configuration
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // initialize the wifi using the configuration struct
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL)); // register the event handler for WIFI events: port wifi events to the wifi_event_handler for processing
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL)); // register the event handler for IP events: port ip events to the wifi_event_handler for processing
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // set the storage to RAM
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ENTERPRISE_WIFI_SSID, // set the SSID
        },
    };
    ESP_LOGI(TAG_WIFI, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // set the mode to station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)); // set the configuration with the configured struct
    // ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EXAMPLE_EAP_ID, strlen(EXAMPLE_EAP_ID)) ); // set the identity (not necessary for WAP2-ENTERPRISE)

    ESP_ERROR_CHECK(esp_eap_client_set_username((uint8_t *)ENTERPRISE_WIFI_USERNAME, strlen(ENTERPRISE_WIFI_USERNAME))); // ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_username((uint8_t *)ENTERPRISE_WIFI_USERNAME, strlen(ENTERPRISE_WIFI_USERNAME))); 
    ESP_ERROR_CHECK(esp_eap_client_set_password((uint8_t *)ENTERPRISE_WIFI_PASSWORD, strlen(ENTERPRISE_WIFI_PASSWORD))); // ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_password((uint8_t *)ENTERPRISE_WIFI_PASSWORD, strlen(ENTERPRISE_WIFI_PASSWORD)));
    
    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable()); // ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_enable());
    
    /* WiFi Start */
    ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_WIFI, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}


```

## main.c

```c
/**
 * @file main.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief 
 * @version 1.0
 * @date 2024-11-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/* DEPENDENCIES */
// ESP
#include "esp_system.h" // ESP32 System
#include "nvs_flash.h"  // ESP32 NVS
#include "esp_chip_info.h" // ESP32 Chip Info
#include "esp_psram.h" // ESP32 PSRAM
#include "esp_flash.h" // ESP32 Flash
#include "esp_log.h" // ESP32 Logging

// BSP
#include "led.h"
#include "exit.h"
#include "spi.h"
#include "lcd.h"
#include "tim.h"
#include "esp_rtc.h"
#include "spi_sdcard.h"
#include "wifi_wpa2_enterprise.h"

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

    // spiffs_test();                                                  /* Run SPIFFS test */
    while (sd_card_init())                               /* SD card not detected */
    {
        lcd_show_string(0, 0, 200, 16, 16, "SD Card Error!", RED);
        vTaskDelay(500);
        lcd_show_string(0, 20, 200, 16, 16, "Please Check!", RED);
        vTaskDelay(500);
    }

    // clean the screen
    lcd_clear(WHITE);

    lcd_show_string(0, 0, 200, 16, 16, "SD Initialized!", RED);

    sd_card_test_filesystem();                                        /* Run SD card test */

    lcd_show_string(0, 0, 200, 16, 16, "SD Tested CSW! ", RED);

    // sd_card_unmount();

    vTaskDelay(3000);

    lcd_show_string(0, 0, lcd_self.width, 16, 16, "WiFi STA Test  ", RED);
    
    ret = wifi_sta_wpa2_init();
    if(ret == ESP_OK)
    {
        ESP_LOGI(TAG_WIFI, "WiFi STA Init OK");
        lcd_show_string(0, 0, lcd_self.width, 16, 16, "WiFi STA Test OK", RED);
    }
    else
    {
        ESP_LOGE(TAG_WIFI, "WiFi STA Init Failed");
    }

    while (1)
    {
        led_toggle();
        ESP_LOGI(TAG, "Hello World!");
        vTaskDelay(1000);
    }
}

```

!!! tip
    对于主程序，只有函数`wifi_sta_wpa2_init()`与WiFi相关。你应该适当地将这个函数插入到你的程序中。不要忘记在主文件中 `#include "wifi_wpa2_enterprise.h"`。WIFI功能为我们下一步进行MQTT通信做好了准备。

