idf_component_register(SRCS "mpu6050.c" "motion_decode.c" INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c freertos log fifo_guard esp_timer)

