idf_component_register(
    SRCS
    INCLUDE_DIRS "."
    REQUIRES 
        board_config 
        can_driver 
        boat_protocol 
        input 
        sensor_manager
)
