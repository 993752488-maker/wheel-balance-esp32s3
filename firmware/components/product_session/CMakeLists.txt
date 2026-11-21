idf_component_register(SRCS "product_engine.c" "product_assessment.c" "product_job.c" "product_trial.c" "product_auto_axis.c" "product_retest.c" "product_speed.c" "product_window.c" "product_session.c" INCLUDE_DIRS "include"
    REQUIRES balance_core coast_selection coherent_average live_analysis raw_capture acquisition optical_ref
        nvs_flash esp_timer esp_hw_support esp_app_format freertos heap)
