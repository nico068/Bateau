idf_component_register(
    SRCS "sensor_manager.c" 
    INCLUDE_DIRS "include" 
    REQUIRES 
        driver 
        esp_timer 
        bme280 
        mpu9250 
        board_config
)
