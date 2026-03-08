idf_component_register(
    SRCS
        main.c
        esp32_serial_transport.c
        force_sensor_adc.c
        mpu9250.c
        character_lcd.c
    INCLUDE_DIRS "."
    REQUIRES
        esp_adc
        driver
        esp_timer
        micro_ros_espidf_component
)
