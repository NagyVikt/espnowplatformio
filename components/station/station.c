/*******************************BEGIN: PURPOSE OF MODULE*******************************
* This module implements an ESP32 STATION that serves as a slave node in an ESP-NOW network 
*******************************END: PURPOSE OF MODULE*******************************/

/*******************************BEGIN: MODULE HEADER FILE INCLUDE*******************************/
#include "station.h"
#include "esp_err.h"
#include "esp_log_buffer.h"
#include "esp_now.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "esp_err.h"			// Required for ESP_ERROR_CHECK()
#include "esp_event.h"			// Required for event driver programming (esp_event_... functions)
#include "esp_wifi.h"			// Required for the Wi-Fi
#include "nvs_flash.h"			// Required for nvs_flash_init()
#include "esp_netif.h"			// Required for esp_netif_init()
#include "esp_now.h"			// Required for ESP-NOW
#include"esp_log.h"
#include <string.h>
#include "sdkconfig.h"
/*******************************END: MODULE HEADER FILE INCLUDE*******************************/

/*******************************BEGIN: STRUCTS, ENUMS, UNIONS, DEFINES*******************************/
#define DEBUG_LOG CONFIG_DEBUG_LOG
#define TAG "STATION"

#define STA_COMMUNICATION_TASK_PRIO 5
#define PEER_ARR_SIZE 1

typedef struct {
	uint8_t mac_addr[6];
}PEER_t;

typedef struct {
	int len;
	uint8_t data[ESP_NOW_MAX_DATA_LEN_V2];
}RECEIVE_DATA_t;

struct s_queue_handlers {
	QueueHandle_t send_cb_msg_queue;
	QueueHandle_t recv_cb_msg_queue;
};
/*******************************END: GSTRUCTS, ENUMS, UNIONS, DEFINES*******************************/

/*******************************BEGIN: GLOBAL VARIABLES PRIVATE TO MODULE*******************************/
uint8_t cmd[] = {0xFF, 0xFF, 0xFF, 0xFF};
uint8_t ack[] = {0xEE, 0xEE, 0xEE, 0xEE};
uint8_t data[128];

static const char *mac_str_arr[1] = {
	CONFIG_MASTER
};

static PEER_t peer_arr[PEER_ARR_SIZE];

struct s_queue_handlers queue_handlers;
/*******************************END: GLOBAL VARIABLES PRIVATE TO MODULE*******************************/

/*******************************BEGIN: HELPER FUNCTION PROTOTYPES PRIVATE TO MODULE*******************************/
static void sta_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void sta_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void sta_connect_peer(PEER_t g_peer);
static void sta_init_tasks(void);
static void sta_communication_task(void *arg);
static void sta_print_mac_addr(void);
static void hub_peer_arr_init(void);
/*******************************END: HELPER FUNCTION PROTOTYPES PRIVATE TO MODULE*******************************/

/*******************************BEGIN: APIs EXPOSED BY THIS MODULE*******************************/

/*******************************API INFORMATION*******************************
 * @fn			- 
 * 
 * @brief		- Initializes the communications (Wi-Fi, ESP-NOW), creates 
 *				  semaphores and starts the necessary tasks. Initializes the data
 *				  to be sent later.
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
 void sta_init(void) {
	ESP_ERROR_CHECK(nvs_flash_init());										
												
	ESP_ERROR_CHECK(esp_netif_init());			
	
	ESP_ERROR_CHECK(esp_event_loop_create_default());	
																									
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();									
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));			
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));	
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_set_channel(6,  WIFI_SECOND_CHAN_NONE));

	ESP_ERROR_CHECK(esp_now_init());

	ESP_ERROR_CHECK(esp_now_register_send_cb(sta_espnow_send_cb));
	ESP_ERROR_CHECK(esp_now_register_recv_cb(sta_espnow_recv_cb));

	queue_handlers.send_cb_msg_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
	queue_handlers.recv_cb_msg_queue = xQueueCreate(1, sizeof(RECEIVE_DATA_t));

	sta_print_mac_addr();

	memset(data, 'A', sizeof(data));

	hub_peer_arr_init();

	sta_init_tasks();
 }

/*******************************END: APIs EXPOSED BY THIS MODULE*******************************/
 
 
 
/*******************************BEGIN: HELPER FUNCTION DEFINITIONS*******************************/

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- sta_espnow_send_cb()
 * 
 * @brief		- Passes the result of a send (success or fail) to a communication
 *				  channel (queue).
 * 
 * @param[in]	- Information about the sender and the receiver, and message
 * @param[in]	- Data sent successfully or not
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- Gets called automatically when the message is sent.
 *****************************************************************************/
static void sta_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xQueueSendFromISR(queue_handlers.send_cb_msg_queue, &status, &xHigherPriorityTaskWoken);
    
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

 /*******************************FUNCTION INFORMATION*******************************
 * @fn			- sta_espnow_recv_cb()
 * 
 * @brief		- Passes the data and its length to a queue.
 * 
 * @param[in]	- Information about the message
 * @param[in]	- The received data
 * @param[in]	- The length of the received data
 * 
 * @return		- none
 * 
 * @note		- Gets called automatically when a message arrives
 *****************************************************************************/
static void sta_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
	RECEIVE_DATA_t recv_data;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(len > ESP_NOW_MAX_DATA_LEN_V2) {
		len = ESP_NOW_MAX_DATA_LEN_V2;
	}
	
	recv_data.len = len;

	memcpy(recv_data.data, data, len);

    xQueueSendFromISR(queue_handlers.recv_cb_msg_queue, &recv_data, &xHigherPriorityTaskWoken);

	if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- sta_connect_peer()
 * 
 * @brief		- Initializes the peer, adds it to the peer list.
 * 
 * @param[in]	- The peer object
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
 static void sta_connect_peer(PEER_t g_peer) {
	esp_now_peer_info_t peer = {		
		.channel = 6,					
		.ifidx = ESP_IF_WIFI_STA,		
		.encrypt = false
	};
	memcpy(peer.peer_addr, g_peer.mac_addr, 6);
	ESP_ERROR_CHECK(esp_now_add_peer(&peer));
 }

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- sta_init_tasks()
 * 
 * @brief		- Initializes and starts the tasks.
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
 static void sta_init_tasks(void) {
	xTaskCreatePinnedToCore(sta_communication_task, "StaTask", 4096, (void*)&peer_arr[0], STA_COMMUNICATION_TASK_PRIO, NULL, tskNO_AFFINITY);
 }

 /*******************************FUNCTION INFORMATION*******************************
 * @fn			- sta_communication_task()
 * 
 * @brief		- The task that is repsonsible for the communication with the HUB
 * 
 * @param[in]	- A pointer to the HUB object
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
 static void sta_communication_task(void *arg) {
	PEER_t *peer = (PEER_t*)arg;

	sta_connect_peer(*peer);

	esp_now_send_status_t msg_from_send_cb;

	RECEIVE_DATA_t data_from_recv_cb;

	bool cmd_recv;
	bool ack_sent;
	bool data_sent;

	while(1) {
		cmd_recv = false;
		ack_sent = false;
		data_sent = false;

		// Receive CMD
		if (xQueueReceive(queue_handlers.recv_cb_msg_queue, &data_from_recv_cb, pdMS_TO_TICKS(100)) == pdTRUE) {
			if (data_from_recv_cb.len == sizeof(cmd)) {
				cmd_recv = true;
				for (uint16_t i = 0; i < sizeof(cmd); i++) {
					if (data_from_recv_cb.data[i] != cmd[i]) {
						cmd_recv = false;
						break;
					}
				}
			}
		}

#if DEBUG_LOG
		if(cmd_recv) {
			ESP_LOGI(TAG, "CMD received");
		}
		else {
			ESP_LOGI(TAG, "CMD receive failed");
		}
#endif

		// Send ACK
		if(cmd_recv) {
			ESP_ERROR_CHECK(esp_now_send(peer->mac_addr, ack, sizeof(ack)));
			if(xQueueReceive(queue_handlers.send_cb_msg_queue, &msg_from_send_cb, pdMS_TO_TICKS(100)) == pdTRUE) {
				if(msg_from_send_cb == ESP_NOW_SEND_SUCCESS) {
					ack_sent = true;
#if DEBUG_LOG
					ESP_LOGI(TAG, "ACK sent successfully.");
#endif
				}
			}
#if DEBUG_LOG
			else {
				ESP_LOGI(TAG, "ACK send failed.");
			}
#endif
		}

		// If ACK has been sent, send data
		if(ack_sent) {
			esp_now_send(peer->mac_addr, data, sizeof(data));
			if(xQueueReceive(queue_handlers.send_cb_msg_queue, &msg_from_send_cb, pdMS_TO_TICKS(100)) == pdTRUE) {
				if(msg_from_send_cb == ESP_NOW_SEND_SUCCESS) {
					data_sent = true;
#if DEBUG_LOG
					ESP_LOGI(TAG, "Data sent successfully.");
#endif
				}
			}
#if DEBUG_LOG
			else {
				ESP_LOGI(TAG, "Data send failed");
			}
#endif
		}

		// If data has been sent, delete task
		/*if(data_sent) {
#if DEBUG_LOG
			ESP_LOGI(TAG, "Deleting task.");
#endif
			vTaskDelete(NULL);
		}*/

		// Delay to yield CPU
		vTaskDelay(pdMS_TO_TICKS(1));
	}
 }

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- sta_print_mac_addr()
 * 
 * @brief		- Prints the MAC address of the device on boot
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
 static void sta_print_mac_addr(void) {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
    ESP_LOGI("MAC_ADDRESS", "MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- hub_peer_arr_init()
 * 
 * @brief		- Parses the MAC addresses from menuconfig to uint8_t type
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
static void hub_peer_arr_init(void) {
	for (int i = 0; i < PEER_ARR_SIZE; i++) {
		int b[6];
		if (sscanf(mac_str_arr[i], "%x:%x:%x:%x:%x:%x",
				&b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
			for (int j = 0; j < 6; j++) {
				peer_arr[i].mac_addr[j] = (uint8_t)b[j];
			}
		} else {
			ESP_LOGE(TAG, "Invalid MAC address format at index %d", i);
		}
	}
}
 /*******************************END: HELPER FUNCTION DEFINITIONS*******************************/