idf_component_register(
    SRCS "main.c" "wifi_softap.c" "http_control.c" "camera.c"
    INCLUDE_DIRS "."
    REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_i2c esp_driver_i2s esp_adc
             nvs_flash esp_wifi esp_netif esp_event esp_http_server json
             esp32-camera
)
