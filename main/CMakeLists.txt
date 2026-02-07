idf_component_register(SRCS "main.c" "angle_controller.c" "kalman_filter.c" "motor_controller.c" "mpu6050.c" "state.c" "input_receiver.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_mcpwm esp_timer esp_wifi nvs_flash)
