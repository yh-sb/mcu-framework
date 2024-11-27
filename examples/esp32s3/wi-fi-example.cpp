// Example for ESP32-S3

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

static void heartbeat_task(void *pvParameters)
{
    while(1)
    {
        // Toggle the LED
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void wifi_event_handler(void *ctx, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    printf("wifi_event_handler() event_id=%d\n", event_id);
    
    switch(event_id)
    {
        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event = static_cast<wifi_event_ap_staconnected_t *>(event_data);
            printf("station " MACSTR " join, AID=%d\n", MAC2STR(event->mac), event->aid);
        }
        break;
        
        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t *event = static_cast<wifi_event_ap_stadisconnected_t *>(event_data);
            printf("station " MACSTR " leave, AID=%d\n", MAC2STR(event->mac), event->aid);
        }
        break;
    }
}

static void wifi_init_softap(void)
{
    esp_netif_init();
    
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr);
    
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    
    wifi_config_t wifi_cfg = {};
    wifi_cfg.ap.max_connection = 4;
    strcpy((char *)wifi_cfg.ap.ssid, "esp32-ap2");
    wifi_cfg.ap.ssid_len = sizeof("esp32-ap2") - 1;
    strcpy((char *)wifi_cfg.ap.password, "wifipass");
    wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
    
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
    esp_wifi_start();
}

// uart baud rate is 115200
extern "C" void app_main()
{
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        res = nvs_flash_init();
    }
    wifi_init_softap();
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, nullptr, 1, nullptr);
}
