#include <string.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_mac.h"          // <<< add this
static const char *TAG = "espnow_master";
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

typedef struct {
    uint8_t id;
    char message[32];
} master_message_t;

// unchanged: send callback still gets (mac, status)
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send to " MACSTR " status: %s",
             MAC2STR(mac_addr),
             status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
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

    // 4) ESP-Now + send callback
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(onDataSent));

    // 5) Add broadcast peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx   = WIFI_IF_STA;    // must be WIFI_IF_STA in IDF v5
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // 6) Send periodically
    master_message_t msg = { .id = 1 };
    strcpy(msg.message, "Hello from master");

    while (true) {
        ESP_ERROR_CHECK(esp_now_send(broadcast_mac,
                                     (uint8_t*)&msg,
                                     sizeof(msg)));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
