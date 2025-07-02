#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"    // ide kerül az esp_read_mac deklaráció

void app_main(void)
{
    uint8_t mac[6];
    // Kérjük le a gyári MAC-címet a Wi-Fi STA interfészre
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        printf("Failed to read MAC: %d\n", err);
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // Kiírjuk
    printf("SLAVE_MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);

    // Lefagyunk, hogy lásd a soros kimenetet
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
