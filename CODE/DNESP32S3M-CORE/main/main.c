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
#include "i2c.h"
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
    i2c_bus_init();
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



// static void wpa2_enterprise_example_task(void *pvParameters)
// {
//     esp_netif_ip_info_t ip;
//     memset(&ip, 0, sizeof(esp_netif_ip_info_t));
//     vTaskDelay(2000 / portTICK_PERIOD_MS);

//     while (1) {
//         vTaskDelay(2000 / portTICK_PERIOD_MS);

//         if (esp_netif_get_ip_info(sta_netif, &ip) == 0) {
//             ESP_LOGI(TAG, "~~~~~~~~~~~");
//             ESP_LOGI(TAG, "IP:"IPSTR, IP2STR(&ip.ip));
//             ESP_LOGI(TAG, "MASK:"IPSTR, IP2STR(&ip.netmask));
//             ESP_LOGI(TAG, "GW:"IPSTR, IP2STR(&ip.gw));
//             ESP_LOGI(TAG, "~~~~~~~~~~~");
//         }
//     }
// }
