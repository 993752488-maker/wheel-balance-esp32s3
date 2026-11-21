idf_component_register(SRCS "raw_capture.c" "raw_capture_core.c" "raw_pose_core.c" "raw_export.c" INCLUDE_DIRS "include"
    REQUIRES esp_timer heap freertos)
