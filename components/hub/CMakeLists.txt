idf_component_register(
    SRCS        "hub.c"
    INCLUDE_DIRS "."
    REQUIRES    stats
                esp_wifi
                nvs_flash
                esp_netif
                esp_event
                esp_timer
                freertos
)
