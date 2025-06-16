/*******************************PURPOSE OF MODULE***********************************
 * This module implements an ESP32 HUB that serves as the central communication node 
 * in an ESP-NOW network.
 **********************************************************************************/

#include "hub.h"
#include "stats.h"

#include "esp_err.h"           // ESP_ERROR_CHECK
#include "esp_log.h"           // ESP_LOG*
#include "esp_timer.h"         // esp_timer_get_time()
#include "esp_now.h"           // ESP-NOW API
#include "esp_wifi.h"          // Wi-Fi driver
#include "nvs_flash.h"         // nvs_flash_init()
#include "esp_netif.h"         // esp_netif_init()
#include "esp_event.h"         // event loop
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdint.h>

#define TAG "HUB"
#define HUB_COMM_TASK_PRIO    5
#define HUB_SPEED_TASK_PRIO   5

static const uint8_t CMD_FRAME[] = {0xFF,0xFF,0xFF,0xFF};
static const uint8_t ACK_FRAME[] = {0xEE,0xEE,0xEE,0xEE};

/* fill in your peer addresses here */
static const uint8_t PEER_MAC[][6] = {
    {0xF0,0xF5,0xBD,0x01,0xB3,0x48},
};

typedef struct {
    int len;
    uint8_t data[ESP_NOW_MAX_DATA_LEN_V2];
} recv_msg_t;

static QueueHandle_t  send_cb_queue;
static QueueHandle_t  recv_cb_queue;
static SemaphoreHandle_t mtx_bits, mtx_cycles, mtx_time;
static double total_bits = 0, total_cycles = 0, total_time = 0;

/*----------------------------------------------------------
 * Callback when an ESP-NOW packet is sent
 *---------------------------------------------------------*/
static void hub_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    xQueueSend(send_cb_queue, &status, pdMS_TO_TICKS(5));
}

/*----------------------------------------------------------
 * Callback when an ESP-NOW packet is received
 *---------------------------------------------------------*/
static void hub_espnow_recv_cb(const esp_now_recv_info_t *info,
                               const uint8_t *data, int len)
{
    recv_msg_t msg;
    xSemaphoreTake(mtx_bits, pdMS_TO_TICKS(5));
    total_bits += len * 8;
    xSemaphoreGive(mtx_bits);

    if (len > ESP_NOW_MAX_DATA_LEN_V2) len = ESP_NOW_MAX_DATA_LEN_V2;
    msg.len = len;
    memcpy(msg.data, data, len);
    xQueueSend(recv_cb_queue, &msg, pdMS_TO_TICKS(5));
}

/*----------------------------------------------------------
 * Add a peer to ESP-NOW
 *---------------------------------------------------------*/
static void hub_connect_peer(const uint8_t mac[6]) {
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx   = ESP_IF_WIFI_STA,
        .encrypt = false
    };
    memcpy(peer.peer_addr, mac, 6);
    ESP_ERROR_CHECK( esp_now_add_peer(&peer) );
}

/*----------------------------------------------------------
 * Communication task: send CMD, wait ACK, then data
 *---------------------------------------------------------*/
static void hub_comm_task(void *arg) {
    const uint8_t *peer_mac = arg;
    hub_connect_peer(peer_mac);

    esp_now_send_status_t send_status;
    recv_msg_t           recv_msg;
    bool                 got_cmd, got_ack;
    int64_t              t0, t1;

    while (1) {
        got_cmd = got_ack = false;
        t0 = esp_timer_get_time();

        /* send CMD and wait callback */
        ESP_ERROR_CHECK( esp_now_send(peer_mac, CMD_FRAME, sizeof(CMD_FRAME)) );
        xQueueReceive(send_cb_queue, &send_status, pdMS_TO_TICKS(100));
        got_cmd = (send_status == ESP_NOW_SEND_SUCCESS);

        /* if CMD ok, wait for ACK frame */
        if (got_cmd) {
            if (xQueueReceive(recv_cb_queue, &recv_msg, pdMS_TO_TICKS(100))) {
                got_ack = (recv_msg.len == sizeof(ACK_FRAME) &&
                           memcmp(recv_msg.data, ACK_FRAME, sizeof(ACK_FRAME)) == 0);
            }
        }

        /* if ACK ok, fetch actual data */
        if (got_ack) {
            if (xQueueReceive(recv_cb_queue, &recv_msg, pdMS_TO_TICKS(100))) {
				ESP_LOG_BUFFER_HEX(TAG, recv_msg.data, recv_msg.len);
                // process data here…
            }
        }

        t1 = esp_timer_get_time();
        xSemaphoreTake(mtx_time, pdMS_TO_TICKS(5)); total_time  += (t1 - t0); xSemaphoreGive(mtx_time);
        xSemaphoreTake(mtx_cycles, pdMS_TO_TICKS(5)); total_cycles++;            xSemaphoreGive(mtx_cycles);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*----------------------------------------------------------
 * Speed measurement/logging task
 *---------------------------------------------------------*/
static void hub_speed_task(void *arg) {
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        xSemaphoreTake(mtx_bits,   pdMS_TO_TICKS(5));
        xSemaphoreTake(mtx_cycles, pdMS_TO_TICKS(5));
        xSemaphoreTake(mtx_time,   pdMS_TO_TICKS(5));

        double kbps = total_bits / 1000.0;
        double avg_latency = total_time / total_cycles;

        ESP_LOGI(TAG, "Throughput: %.2f kBits/s", kbps);
        ESP_LOGI(TAG, "Cycles per sec: %.0f", total_cycles);
        ESP_LOGI(TAG, "Avg latency: %.2f us", avg_latency);

        // reset counters
        total_bits   = 0;
        total_cycles = 0;
        total_time   = 0;

        xSemaphoreGive(mtx_time);
        xSemaphoreGive(mtx_cycles);
        xSemaphoreGive(mtx_bits);
    }
}

/*----------------------------------------------------------
 * Print this device’s MAC on boot
 *---------------------------------------------------------*/
static void hub_print_mac(void) {
    uint8_t mac[6];
    ESP_ERROR_CHECK( esp_wifi_get_mac(ESP_IF_WIFI_STA, mac) );
    ESP_LOGI("HUB", "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

/*----------------------------------------------------------
 * Public init function
 *---------------------------------------------------------*/
void hub_init(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );

    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(hub_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(hub_espnow_recv_cb) );

    send_cb_queue  = xQueueCreate(1, sizeof(esp_now_send_status_t));
    recv_cb_queue  = xQueueCreate(1, sizeof(recv_msg_t));
    mtx_bits       = xSemaphoreCreateMutex();
    mtx_cycles     = xSemaphoreCreateMutex();
    mtx_time       = xSemaphoreCreateMutex();

    hub_print_mac();

    /* start tasks, passing first peer’s MAC as argument */
    xTaskCreatePinnedToCore(hub_comm_task,  "hub_comm", 4096, (void*)PEER_MAC[0], HUB_COMM_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(hub_speed_task, "hub_speed",4096, NULL,               HUB_SPEED_TASK_PRIO, NULL, tskNO_AFFINITY);
}
