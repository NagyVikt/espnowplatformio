#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// Method 1: via Wi-Fi driver
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

// Method 2: direct from efuse
#include "esp_efuse.h"
#include "esp_efuse_mac.h"

static const char* TAG = "MAC_EXAMPLE";

extern "C" void app_main(void)
{
    // --- Method 1: Initialize Wi-Fi and read with esp_wifi_get_mac ---
    ESP_LOGI(TAG, "Initializing TCP/IP stack and Wi-Fi driver...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac1[6] = {0};
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac1);
    if (err == ESP_OK) {
        printf("Method1: STA MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac1[0], mac1[1], mac1[2],
               mac1[3], mac1[4], mac1[5]);
    } else {
        ESP_LOGE(TAG, "esp_wifi_get_mac failed: %d", err);
    }

    // --- Method 2: Read directly from eFUSEs ---
    uint8_t mac2[6] = {0};
    err = esp_efuse_mac_get_default(mac2);
    if (err == ESP_OK) {
        printf("Method2: STA MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac2[0], mac2[1], mac2[2],
               mac2[3], mac2[4], mac2[5]);
    } else {
        ESP_LOGE(TAG, "esp_efuse_mac_get_default failed: %d", err);
    }

    // just loop forever so you can read the prints
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
