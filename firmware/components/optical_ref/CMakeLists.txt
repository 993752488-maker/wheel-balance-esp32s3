idf_component_register(SRCS "optical_ref.c" INCLUDE_DIRS "include"
    REQUIRES rpm_core board_config esp_timer esp_driver_gpio freertos raw_capture)

