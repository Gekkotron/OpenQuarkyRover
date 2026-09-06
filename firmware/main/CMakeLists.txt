idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_i2c esp_adc
)
