idf_component_register(
    SRCS "input.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        board_config 
        esp_adc 
        esp_driver_gpio 
        esp_driver_pcnt
)