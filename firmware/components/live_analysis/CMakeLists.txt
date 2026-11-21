idf_component_register(SRCS "live_analysis.c" "live_analysis_core.c" "live_analysis_record.c" INCLUDE_DIRS "include"
    REQUIRES acquisition optical_ref sync_analysis coast_analysis board_config esp_timer heap freertos)
