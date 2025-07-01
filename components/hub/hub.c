/*******************************BEGIN: PURPOSE OF MODULE*******************************
* This module implements an ESP32 HUB that serves as the central communication node 
* in an ESP-NOW network. 
*******************************END: PURPOSE OF MODULE*******************************/

/*******************************BEGIN: MODULE HEADER FILE INCLUDE*******************************/
#include "hub.h"
#include "esp_err.h"
#include "esp_log_buffer.h"
#include "esp_now.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"			
#include "esp_event.h"		
#include "esp_wifi.h"			
#include "nvs_flash.h"		
#include "esp_netif.h"	
#include "esp_now.h"		
#include "esp_log.h"
#include <stdint.h>
#include "esp_timer.h"
#include "portmacro.h"
#include "sdkconfig.h"
/*******************************END: MODULE HEADER FILE INCLUDE*******************************/

/*******************************BEGIN: STRUCTS, ENUMS, UNIONS, DEFINES (PRIVATE)*******************************/
#define TAG "HUB"

#define HUB_COMM_TASK_PRIO 5
#define HUB_COMM_ALL_TASK_PRIO 4
#define HUB_SPEED_MEASUREMENT_TASK_PRIO 3

#define PEER_ARR_SIZE CONFIG_NUM_MACS

typedef struct {
	int len;
	uint8_t data[ESP_NOW_MAX_DATA_LEN_V2];
}RECEIVE_DATA_t;

struct s_queue_handlers {
	QueueHandle_t send_cb_msg_queue;
	QueueHandle_t recv_cb_msg_queue;
};

struct s_semaphore_handlers {
	SemaphoreHandle_t semaph_measurement;
	SemaphoreHandle_t semaph_hub_comm_task_finished;
};

struct s_task_handlers {
	TaskHandle_t handle_measurement_task;
	TaskHandle_t handle_hub_comm_task;
};

struct s_measurement_variables {
	double num_bits_recv;
	double num_cycles;
	double total_time;
};
/*******************************END: STRUCTS, ENUMS, UNIONS, DEFINES (PRIVATE)*******************************/

/*******************************BEGIN: GLOBAL VARIABLES PRIVATE TO MODULE*******************************/
static uint8_t cmd[] = {0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t ack[] = {0xEE, 0xEE, 0xEE, 0xEE};

static struct s_queue_handlers queue_handlers;
static struct s_semaphore_handlers semaphore_handlers;
static struct s_task_handlers task_handlers;
static struct s_measurement_variables measurement_variables;

static PEER_t peer_arr[PEER_ARR_SIZE];

static const char *mac_str_arr[10] = {
	CONFIG_STATION_1,
	CONFIG_STATION_2,
	CONFIG_STATION_3,
	CONFIG_STATION_4,
	CONFIG_STATION_5,
	CONFIG_STATION_6,
	CONFIG_STATION_7,
	CONFIG_STATION_8,
	CONFIG_STATION_9,
	CONFIG_STATION_10
};
/*******************************END: GLOBAL VARIABLES PRIVATE TO MODULE*******************************/

/*******************************BEGIN: CALLBACK FUNCTION PROTOTYPES PRIVATE TO MODULE*******************************/
static void hub_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void hub_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
/*******************************END: CALLBACK FUNCTION PROTOTYPES PRIVATE TO MODULE*******************************/

/*******************************BEGIN: HELPER FUNCTION PROTOTYPES PRIVATE TO MODULE*******************************/
static void hub_connect_peer(PEER_t g_peer);
static void hub_disconnect_peer(PEER_t g_peer);
static void hub_print_mac_addr(void);
static void hub_peer_arr_init(void);
/*******************************END: HELPER FUNCTION PROTOTYPES PRIVATE TO MODULE*******************************/

/*******************************BEGIN: TASK PROTOTYPES PRIVATE TO MODULE*******************************/
static void hub_comm_task(void *arg);
static void hub_comm_loop_task(void *arg);
static void hub_comm_all_task(void *arg);
static void hub_speed_measurement_task(void *arg);
/*******************************END: TASK PROTOTYPES PRIVATE TO MODULE*******************************/

/*******************************BEGIN: APIs EXPOSED BY THIS MODULE*******************************/

/*******************************API INFORMATION*******************************
 * @fn			- hub_init()
 * 
 * @brief		- Initializes the communications (Wi-Fi, ESP-NOW), creates 
 *				  semaphores.
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
 void hub_init(void) {
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

	ESP_ERROR_CHECK(esp_now_register_send_cb(hub_espnow_send_cb));
	ESP_ERROR_CHECK(esp_now_register_recv_cb(hub_espnow_recv_cb));

	queue_handlers.send_cb_msg_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
	queue_handlers.recv_cb_msg_queue = xQueueCreate(1, sizeof(RECEIVE_DATA_t));

	semaphore_handlers.semaph_measurement = xSemaphoreCreateMutex();
	xSemaphoreGive(semaphore_handlers.semaph_measurement);  

	semaphore_handlers.semaph_hub_comm_task_finished = xSemaphoreCreateBinary();  
	xSemaphoreGive(semaphore_handlers.semaph_hub_comm_task_finished); 

	hub_peer_arr_init();

	hub_print_mac_addr();
}

/*******************************API INFORMATION*******************************
 * @fn			- hub_spawn_comm_task()
 * 
 * @brief		- Spawns a communication task. 			  
 * 
 * @param[in]	- A peer object.
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- Dynamically allocates memory for the peer, this way once the
				  function returns, the memory won't get corrupted and can be 
				  used from other functions.
 *****************************************************************************/
void hub_spawn_comm_task(PEER_t	peer) {
    PEER_t *peer_arg = malloc(sizeof(PEER_t));		
    if (peer_arg == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for peer");
        return;
    }
    *peer_arg = peer;

    xTaskCreatePinnedToCore(hub_comm_task, "HubCommTask", 4096, (void*)peer_arg,
                            HUB_COMM_TASK_PRIO, &task_handlers.handle_hub_comm_task, tskNO_AFFINITY);
}

/*******************************API INFORMATION*******************************
 * @fn			- hub_spawn_comm_loop_task()
 * 
 * @brief		- Spawns a communication task, which is looping. 			  
 * 
 * @param[in]	- A peer object.
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- Dynamically allocates memory for the peer, this way once the
				  function returns, the memory won't get corrupted and can be 
				  used from other functions.
 *****************************************************************************/
void hub_spawn_comm_loop_task(PEER_t peer) {
 	PEER_t *peer_arg = malloc(sizeof(PEER_t));		
	if (peer_arg == NULL) {
		ESP_LOGE(TAG, "Failed to allocate memory for peer");
		return;
	}
	*peer_arg = peer;

	xTaskCreatePinnedToCore(hub_comm_loop_task, "HubCommLoopTask", 4096, (void*)peer_arg,
							HUB_COMM_TASK_PRIO, NULL, tskNO_AFFINITY);
}

/*******************************API INFORMATION*******************************
 * @fn			- hub_spawn_comm_all_loop_task()
 * 
 * @brief		- Spawns a communication task with all peers, which is either looping or not. 			  
 * 
 * @param[in]	- Whether to loop.
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- Dynamically allocates memory for the peer, this way once the
				  function returns, the memory won't get corrupted and can be 
				  used from other functions.
 *****************************************************************************/
void hub_spawn_comm_all_loop_task(int loop) {
	int *p_loop = malloc(sizeof(loop));
	if (p_loop == NULL) {
		ESP_LOGE(TAG, "Failed to allocate memory.");
		return;
	}
	*p_loop = loop;

	xTaskCreatePinnedToCore(hub_comm_all_task, "HubCommAllTask", 4096, (void*)p_loop,
		HUB_COMM_TASK_PRIO, NULL, tskNO_AFFINITY);
}

/*******************************API INFORMATION*******************************
 * @fn			- hub_spawn_measurement_task()
 * 
 * @brief		- Spawns a measurement task. 			  
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
void hub_spawn_measurement_task(void) {
	xTaskCreatePinnedToCore(hub_speed_measurement_task, "Measurement task", 4096, NULL, HUB_SPEED_MEASUREMENT_TASK_PRIO, &task_handlers.handle_measurement_task, tskNO_AFFINITY);
}


/*******************************API INFORMATION*******************************
 * @fn			- hub_delete_measurement_task()
 * 
 * @brief		- Deletes the measurement task created before. 			  
 * 
 * @param[in]	- none
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
void hub_delete_measurement_task(void) {
	vTaskDelete(task_handlers.handle_measurement_task);
}

/*******************************END: APIs EXPOSED BY THIS MODULE*******************************/

/*******************************BEGIN: CALLBACK FUNCTION DEFINITIONS*******************************/

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- hub_espnow_send_cb()
 * 
 * @brief		- Passes the result of a send (success or fail) to a communication
 *				  channel (queue).
 * 
 * @param[out]	- Information about the sender, receiver, and message
 * @param[out]	- Data sent successfully or not
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- Gets called automatically when the message is sent
 *****************************************************************************/
static void hub_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xQueueSendFromISR(queue_handlers.send_cb_msg_queue, &status, &xHigherPriorityTaskWoken);
    
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- hub_espnow_recv_cb()
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
static void hub_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    RECEIVE_DATA_t recv_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(xSemaphoreTakeFromISR(semaphore_handlers.semaph_measurement, &xHigherPriorityTaskWoken) == pdTRUE) {
        measurement_variables.num_bits_recv += len * 8;
        xSemaphoreGiveFromISR(semaphore_handlers.semaph_measurement, &xHigherPriorityTaskWoken);
    }
    
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

/*******************************END: CALLBACK FUNCTION DEFINITIONS*******************************/
 
/*******************************BEGIN: HELPER FUNCTION DEFINITIONS*******************************/

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- hub_connect_peers()
 * 
 * @brief		- Initializes peers, adds them to the peer list.
 * 
 * @param[in]	- The peer object
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
static void hub_connect_peer(PEER_t g_peer) {
	esp_now_peer_info_t peer = {		
		.channel = 6,					
		.ifidx = ESP_IF_WIFI_STA,		
		.encrypt = false
	};

	memcpy(peer.peer_addr, g_peer.mac_addr, 6);
	ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- hub_disconnect_peer()
 * 
 * @brief		- Removes a peer from the peer list.
 * 
 * @param[in]	- The peer object
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
static void hub_disconnect_peer(PEER_t g_peer) {
	ESP_ERROR_CHECK(esp_now_del_peer(g_peer.mac_addr));
}

/*******************************FUNCTION INFORMATION*******************************
 * @fn			- hub_print_mac_addr()
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
static void hub_print_mac_addr(void) {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
    ESP_LOGI("MAC_ADDRESS", "%02X:%02X:%02X:%02X:%02X:%02X", 
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

/*******************************START: TASK DEFINITIONS*******************************/

/*******************************TASK INFORMATION*******************************
 * @fn			- hub_comm_task()
 * 
 * @brief		- The task that is repsonsible for the communication with the stations
 * 
 * @param[in]	- A pointer to the peer (the other end of the communication).
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
static void hub_comm_task(void *arg) {
	PEER_t *peer = (PEER_t*)arg;

	PEER_t peer_cpy = *peer;

	free(peer);

	hub_connect_peer(peer_cpy);

	esp_now_send_status_t msg_from_send_cb;

	RECEIVE_DATA_t data_from_recv_cb;

	bool cmd_sent;
	bool ack_recv;
	bool delete_task;

	int64_t start_us, end_us;

	while(1) {
		cmd_sent = false;
		ack_recv = false;
		delete_task = false;

		start_us = esp_timer_get_time();

		// Send cmd data
		esp_err_t err = esp_now_send(peer_cpy.mac_addr, cmd, sizeof(cmd));
		if (err != ESP_OK) {
			ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
		}

		xQueueReceive(queue_handlers.send_cb_msg_queue, &msg_from_send_cb, pdMS_TO_TICKS(100));
		if(msg_from_send_cb == ESP_NOW_SEND_SUCCESS) {
#if CONFIG_DEBUG_LOG
			ESP_LOGI(TAG, "CMD sent successfully.");
#endif
			cmd_sent = true;
		}
#if CONFIG_DEBUG_LOG
		else {
			ESP_LOGI(TAG, "Sending of CMD failed.");
		}
#endif

		// Receive ack data, if not received, then send command again
		if(cmd_sent) {
			ack_recv = false;
			if (xQueueReceive(queue_handlers.recv_cb_msg_queue, &data_from_recv_cb, pdMS_TO_TICKS(100)) == pdTRUE) {
				if (data_from_recv_cb.len == sizeof(ack)) {
					ack_recv = true;
					for (uint16_t i = 0; i < sizeof(ack); i++) {
						if (data_from_recv_cb.data[i] != ack[i]) {
							ack_recv = false;
							break;
						}
					}
				}
			}
		}

#if CONFIG_DEBUG_LOG
		if(ack_recv) {
			ESP_LOGI(TAG, "ACK received successfully.");
		}
#endif

		// If ACK received, can receive actual data
		if(ack_recv && xQueueReceive(queue_handlers.recv_cb_msg_queue, &data_from_recv_cb, pdMS_TO_TICKS(100)) == pdTRUE) {
#if CONFIG_DEBUG_LOG
			ESP_LOG_BUFFER_CHAR(TAG, data_from_recv_cb.data, data_from_recv_cb.len);
			ESP_LOGI(TAG, "Data received successfully.");
#endif

			delete_task = true;
		}

		xSemaphoreTake(semaphore_handlers.semaph_measurement, portMAX_DELAY);

		end_us = esp_timer_get_time();
		
		measurement_variables.total_time += end_us - start_us;
		measurement_variables.num_cycles++;

		xSemaphoreGive(semaphore_handlers.semaph_measurement);

		if(delete_task) {
			hub_disconnect_peer(peer_cpy);
			xSemaphoreGive(semaphore_handlers.semaph_hub_comm_task_finished);
			vTaskDelete(NULL);
		}
	}
}


/*******************************TASK INFORMATION*******************************
 * @fn			- hub_comm_loop_task()
 * 
 * @brief		- This tasks spawns multiple communication tasks in a loop
 * 
 * @param[in]	- A pointer to the peer (the other end of the communication).
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- BUGGY, IF DELAY IS TOO SMALL IT TRIGGERS THE IDLE0 WATCHDOG.
				  YET TO BE FIXED.
 *****************************************************************************/
static void hub_comm_loop_task(void *arg) {
	PEER_t *peer = (PEER_t*)arg;
	PEER_t peer_cpy = *peer;
	free(peer);

	while(1) {
		peer = malloc(sizeof(PEER_t));
		*peer = peer_cpy;
		xSemaphoreTake(semaphore_handlers.semaph_hub_comm_task_finished, portMAX_DELAY);

		xTaskCreatePinnedToCore(hub_comm_task, "HubCommTask", 4096, (void*)peer,
        	HUB_COMM_TASK_PRIO, &task_handlers.handle_hub_comm_task, tskNO_AFFINITY);
	}
}

/*******************************TASK INFORMATION*******************************
 * @fn			- hub_comm_all_task()
 * 
 * @brief		- This task completes a communication cycle with every station.
 * 
 * @param[in]	- Whether to loop or not.
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
static void hub_comm_all_task(void *arg) {
	int *loop = (int*)arg;
	int loop_cpy = *loop;
	free(loop);

	PEER_t *peer;

    uint16_t current_task = 0;
    
    while(1) {
        // Wait for the current communication task to finish
		peer = malloc(sizeof(PEER_t));
		*peer = peer_arr[current_task];

        xSemaphoreTake(semaphore_handlers.semaph_hub_comm_task_finished, portMAX_DELAY);
        
        xTaskCreatePinnedToCore(hub_comm_task, "HubCommTask", 4096, 
        	(void*)peer, HUB_COMM_TASK_PRIO, 
            &task_handlers.handle_hub_comm_task, tskNO_AFFINITY);

		if(!loop_cpy && current_task == PEER_ARR_SIZE - 1) {
			vTaskDelete(NULL);
		}

		current_task = (current_task + 1) % PEER_ARR_SIZE;
	}
} 

/*******************************TASK INFORMATION*******************************
 * @fn			- hub_speed_measurement_task()
 * 
 * @brief		- The task that is repsonsible for logging the latency and 
 *              - throughput information.
 * 
 * @param[in]	- NULL
 * @param[in]	- none
 * @param[in]	- none
 * 
 * @return		- none
 * 
 * @note		- none
 *****************************************************************************/
static void hub_speed_measurement_task(void *arg) {
    double avg_delay;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay first to allow data collection
        
        if(xSemaphoreTake(semaphore_handlers.semaph_measurement, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "******************************");
            ESP_LOGI(TAG, "Speed: %.2f kBits/s", measurement_variables.num_bits_recv / 1000);
            ESP_LOGI(TAG, "Number of communication cycles: %d", (int)measurement_variables.num_cycles);
            avg_delay = measurement_variables.total_time / measurement_variables.num_cycles;
            ESP_LOGI(TAG, "Average latency: %.2f us", avg_delay);
            
            // Reset counters
            measurement_variables.num_bits_recv = 0;
            measurement_variables.num_cycles = 0;
            measurement_variables.total_time = 0;
            
            xSemaphoreGive(semaphore_handlers.semaph_measurement);
        }
    }
}

/*******************************END: TASK DEFINITIONS*******************************/