// app_main.cpp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include <cstring>

extern "C" {
    #include "nvs_flash.h"
    #include "esp_netif.h"
    #include "esp_event.h"
    #include "esp_wifi.h"
    #include "esp_now.h"
}

static const char *TAG = "MASTER";

// ======== Konstansok ========
static constexpr uint8_t CMD_BYTES[4] = {0xFF,0xFF,0xFF,0xFF};
static constexpr uint8_t ACK_BYTES[4] = {0xEE,0xEE,0xEE,0xEE};

// Itt add meg a slave-ek MAC-címeit:
static constexpr int NUM_SLAVES = 2;
static const uint8_t slave_macs[NUM_SLAVES][6] = {
    {0xAA,0xBB,0xCC,0xDD,0xEE,0x01},
    {0xAA,0xBB,0xCC,0xDD,0xEE,0x02},
};

// Fogadott üzenet struktúra
typedef struct {
    uint8_t src_addr[6];
    uint8_t data[128];
    int len;
} recv_data_t;

static QueueHandle_t recv_queue = nullptr;

// ======== ESP-NOW callback ========
static void on_data_recv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len)
{
    recv_data_t msg;
    memcpy(msg.src_addr, info->src_addr, 6);
    msg.len = len > sizeof(msg.data) ? sizeof(msg.data) : len;
    memcpy(msg.data, data, msg.len);
    xQueueSendFromISR(recv_queue, &msg, nullptr);
}

// ======== Helper: peer hozzáadása ========
static esp_err_t add_peer(const uint8_t mac[6]) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    return esp_now_add_peer(&peer);
}

// ======== Master feladat ========
static void master_task(void *) {
    // 1) CMD kiküldése, ACK-ek begyűjtése
    ESP_LOGI(TAG, "Sending CMD to all slaves...");
    for (int i = 0; i < NUM_SLAVES; ++i) {
        esp_now_send(slave_macs[i], CMD_BYTES, sizeof(CMD_BYTES));
    }
    int acks = 0;
    while (acks < NUM_SLAVES) {
        recv_data_t msg;
        if (xQueueReceive(recv_queue, &msg, pdMS_TO_TICKS(2000))) {
            if (msg.len == 4 && memcmp(msg.data, ACK_BYTES, 4) == 0) {
                ESP_LOGI(TAG, "Received ACK from %02X:%02X:%02X:%02X:%02X:%02X",
                         msg.src_addr[0],msg.src_addr[1],msg.src_addr[2],
                         msg.src_addr[3],msg.src_addr[4],msg.src_addr[5]);
                ++acks;
            }
        } else {
            ESP_LOGW(TAG, "Timeout waiting for ACK");
            break;
        }
    }

    // 2) MONITOR parancs küldése
    const char monitor_cmd[] = "MONITOR 1,2,3";
    ESP_LOGI(TAG, "Sending MONITOR→\"%s\"", monitor_cmd);
    for (int i = 0; i < NUM_SLAVES; ++i) {
        esp_now_send(slave_macs[i],
                     (const uint8_t*)monitor_cmd,
                     strlen(monitor_cmd) + 1);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // várunk, hogy a slave-ek beállítsák a monitored[] tömböt

    // 3) CHECK parancs küldése, majd SUCCESS/FAILURE üzenetek fogadása
    const char check_cmd[] = "CHECK";
    ESP_LOGI(TAG, "Sending CHECK");
    for (int i = 0; i < NUM_SLAVES; ++i) {
        esp_now_send(slave_macs[i],
                     (const uint8_t*)check_cmd,
                     strlen(check_cmd) + 1);
    }

    int responses = 0;
    while (responses < NUM_SLAVES) {
        recv_data_t msg;
        if (xQueueReceive(recv_queue, &msg, pdMS_TO_TICKS(2000))) {
            // minden nem-ACK üzenet a slave eredménye
            if (!(msg.len == 4 && memcmp(msg.data, ACK_BYTES, 4)==0)) {
                ESP_LOGI(TAG, "Result from %02X:%02X:%02X:%02X:%02X:%02X → %.*s",
                         msg.src_addr[0],msg.src_addr[1],msg.src_addr[2],
                         msg.src_addr[3],msg.src_addr[4],msg.src_addr[5],
                         msg.len, msg.data);
                ++responses;
            }
        } else {
            ESP_LOGW(TAG, "Timeout waiting for slave result");
            break;
        }
    }

    ESP_LOGI(TAG, "Master sequence done.");
    vTaskDelete(nullptr);
}

// ======== Inicializálás ========
extern "C" void app_main() {
    ESP_LOGI(TAG, "Master starting...");

    // 1) NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    // 2) TCP/IP stack + event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 3) Wi-Fi STA
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // 4) ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    // 5) Peer-ek hozzáadása
    for (int i = 0; i < NUM_SLAVES; ++i) {
        ESP_ERROR_CHECK(add_peer(slave_macs[i]));
    }

    // 6) Üzenetek fogadására szolgáló queue
    recv_queue = xQueueCreate(10, sizeof(recv_data_t));

    // 7) Elindítjuk a master feladatot
    xTaskCreate(master_task, "master_task", 4096, nullptr, 5, nullptr);
}
