idf_component_register(
    SRCS "main.c" "wifi_softap.c" "http_control.c" "camera.c"
         "i2c_bitbang.c" "es8311.c" "audio_capture.c" "command_bus.c" "led_indicator.c"
         "wake_word.c"
    INCLUDE_DIRS "."
    REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_i2c esp_driver_i2s esp_adc esp_timer
             nvs_flash esp_wifi esp_netif esp_event esp_http_server json
             esp32-camera esp-sr
)
