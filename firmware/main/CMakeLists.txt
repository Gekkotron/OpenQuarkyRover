idf_component_register(
    SRCS "main.c" "i2c_bitbang.c" "es8311.c" "audio_capture.c" "command_bus.c" "led_indicator.c"
    INCLUDE_DIRS "."
    REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_i2c esp_adc esp_driver_i2s esp_timer
)
