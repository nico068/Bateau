idf_component_register(
    SRCS "app_main.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        board_config 
        can_driver 
        boat_protocol 
        input 
        sensor_manager
)
