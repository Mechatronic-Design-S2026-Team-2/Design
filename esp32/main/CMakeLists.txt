idf_component_register(
    SRCS
        main.c
        esp32_serial_transport.c
        dsy_rs485_servo.c
        motor_velocity_bridge.c
        motor_power_manager.c
    INCLUDE_DIRS "."
)
