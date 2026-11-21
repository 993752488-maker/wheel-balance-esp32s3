idf_component_register(SRCS "app_main.c" "link_probe.c" "fifo_probe.c" "frame_probe.c" "raw_console.c" "timing_console.c" INCLUDE_DIRS "."
    REQUIRES acquisition optical_ref serial_console sync_analysis live_analysis diagnostic_web product_session coast_selection coast_precision coherent_average fifo_guard esp_driver_gpio esp_timer esp_app_format raw_capture pose_stream)

