idf_component_register(SRCS "acquisition.c" "timing_trace_core.c" INCLUDE_DIRS "include"
    REQUIRES mpu6050 board_config esp_timer esp_driver_gpio freertos raw_capture fifo_guard pose_stream)

