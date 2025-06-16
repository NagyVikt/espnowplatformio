# components/station/CMakeLists.txt

idf_component_register(
    SRCS         "station.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_wifi     # brings in esp_now.h & APIs
                 nvs_flash    # nvs_flash_init()
                 esp_netif    # esp_netif_init()
                 esp_event    # esp_event_loop_create_default()
                 freertos     # queues, tasks, semaphores
)
