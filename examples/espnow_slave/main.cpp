#include <string.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_mac.h"         
static const char *TAG = "espnow_slave";

typedef struct {
    uint8_t id;
    char message[32];
} slave_message_t;

// NEW signature: first parameter is a pointer to esp_now_recv_info_t
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), MACSTR, MAC2STR(info->src_addr));

    const slave_message_t *msg = (const slave_message_t *)data;
    ESP_LOGI(TAG,
             "Received from %s | id=%u | msg=%s",
             macStr,
             msg->id,
             msg->message);
}

extern "C" void app_main() {
    // 1) NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2) netif + event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3) Wi-Fi STA
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 4) ESP-Now + recv callback
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onDataRecv));

    ESP_LOGI(TAG, "ESP-Now slave ready");
    // now just wait for incoming packets…
}
