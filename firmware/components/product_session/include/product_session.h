#pragma once
#include "product_engine.h"
#include "product_trial.h"
#include "product_retest.h"
#include "product_speed.h"
#include "product_window.h"
#include "product_assessment.h"
#include "esp_err.h"
typedef enum { PRODUCT_NEW,PRODUCT_START_ORIGINAL,PRODUCT_START_TRIAL,PRODUCT_START_RETEST,
    PRODUCT_CANCEL,PRODUCT_DEMO,PRODUCT_DEMO_TRIAL,PRODUCT_DEMO_RETEST,PRODUCT_TRIAL_PLAN,PRODUCT_FINISH } product_command_t;
typedef struct { product_command_t action;uint32_t id,revision;product_config_t config;bool trial_removed;
    bool guided,trial_installed,manual_axis;double piece_mass_g,confirmed_mass_g;product_trial_source_t plan_source;
    product_retest_condition_t retest_condition;
    product_capture_config_t capture;bool capture_config_present;
} product_request_t;
esp_err_t product_session_start(const char *boot_id);
bool product_session_submit(const product_request_t *);
void product_session_get(product_snapshot_t *);
void product_session_get_view(product_snapshot_t *,product_trial_guide_t *);
void product_session_get_full_view(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *);
void product_session_get_real_config(product_config_t *);
void product_session_get_speed_view(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,product_speed_t *);
size_t product_session_history_speed_views(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,product_speed_t *,size_t capacity);
void product_session_get_product_view(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,product_speed_t *,product_window_t *);
size_t product_session_history_product_views(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,product_speed_t *,product_window_t *,size_t capacity);
size_t product_session_history(product_snapshot_t *,size_t capacity);
size_t product_session_history_views(product_snapshot_t *,product_trial_guide_t *,size_t capacity);
size_t product_session_history_full_views(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,size_t capacity);

void product_session_get_assessment_view(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,product_speed_t *,product_window_t *,product_assessment_t *);
size_t product_session_history_assessment_views(product_snapshot_t *,product_trial_guide_t *,product_retest_log_t *,product_speed_t *,product_window_t *,product_assessment_t *,size_t capacity);
