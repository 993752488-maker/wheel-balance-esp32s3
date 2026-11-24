#pragma once
#include "product_engine.h"
typedef enum { JOB_NONE,JOB_START,JOB_STOP,JOB_FAIL } product_job_action_t;
typedef enum { CAPTURE_END_UNKNOWN,CAPTURE_END_REFERENCE_SILENCE,
    CAPTURE_END_OPERATOR,CAPTURE_END_LIMIT,CAPTURE_END_TIMED } product_capture_end_t;
typedef enum { CAPTURE_COAST,CAPTURE_TIMED } product_capture_mode_t;
typedef enum { CAPTURE_BICYCLE,CAPTURE_ELECTRIC,CAPTURE_OTHER } product_vehicle_type_t;
/* Zero seconds selects a duration at the measured operating speed. */
typedef struct { uint8_t mode,vehicle_type;uint16_t seconds; } product_capture_config_t;
product_capture_config_t product_capture_default(void);
bool product_capture_config_valid(const product_capture_config_t *);
void product_capture_apply_preset(product_capture_config_t *);
unsigned product_capture_max_rpm(product_capture_mode_t);
unsigned product_capture_auto_seconds(double rpm);
double product_capture_baseline_rpm(const product_stage_summary_t *);
#define PRODUCT_CAPTURE_MAX_MS 120000U
#define PRODUCT_STEADY_SPEED_TOLERANCE 0.05
typedef struct {
    product_state_t state;product_reason_t reason;
    uint32_t epoch;uint64_t initial_edges,last_edges,deadline_us,start_us,last_motion_us,last_period_us;
    product_capture_end_t end_reason;
    product_capture_config_t capture;
    unsigned steady_turns,minimum_seconds;
    double steady_min_rpm,steady_max_rpm,target_rpm,reference_rpm;
} product_job_t;
void product_job_arm(product_job_t *,uint64_t now,uint64_t edges,uint32_t epoch,product_capture_config_t);
void product_job_recording(product_job_t *,uint64_t now,uint64_t edges);
bool product_job_finish(product_job_t *);
product_job_action_t product_job_tick(product_job_t *,uint64_t now,uint64_t edges,
    uint32_t epoch,bool online,bool reference_ok,bool data_fault,bool rpm_valid,double rpm);
