idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        board_config 
        can_driver 
        boat_protocol 
        input 
        sensor_manager
)
