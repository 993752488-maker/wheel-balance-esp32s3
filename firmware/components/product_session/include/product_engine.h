#pragma once
#include "balance_core.h"
#include "coast_selection.h"
#include "coherent_average.h"

#define PRODUCT_MAX_TURNS 96U
#define PRODUCT_HISTORY 3U
typedef enum { PRODUCT_ORIGINAL,PRODUCT_TRIAL,PRODUCT_RETEST } product_stage_t;
typedef enum { PRODUCT_IDLE,PRODUCT_ARMED,PRODUCT_RECORDING,PRODUCT_PROCESSING,
    PRODUCT_COMPLETE,PRODUCT_NO_RESULT,PRODUCT_CANCELLED,PRODUCT_TIMEOUT } product_state_t;
typedef enum { PRODUCT_REASON_NONE,PRODUCT_SENSOR,PRODUCT_DATA_FAULT,PRODUCT_REFERENCE,
    PRODUCT_TOO_FEW_TURNS,PRODUCT_NO_COMMON_SPEED,PRODUCT_WEAK_RESPONSE,
    PRODUCT_INCONSISTENT,PRODUCT_CONTEXT,PRODUCT_NUMERIC,PRODUCT_BUSY,
    PRODUCT_WAIT_TIMEOUT,PRODUCT_BASELINE_ZERO,PRODUCT_STORAGE,PRODUCT_LOW_DIRECTION_COHERENCE,
    PRODUCT_CAPTURE_LIMIT,PRODUCT_CONFLICT } product_reason_t;
typedef struct {
    double mass_g,trial_radius_mm,trial_angle_deg,correction_radius_mm;
    unsigned axis; int direction; char vehicle[64];
} product_config_t;
typedef struct {
    double rpm,previous_rpm,real,imag;
    uint64_t reference,time_us;
} product_turn_t;
typedef struct {
    bool captured,integrity_valid;
    uint32_t capture_id,epoch,samples,references,turn_count,usable_turns,rejected_turns;
    uint64_t start_us,end_us;
    double min_rpm,max_rpm,real,imag,peak,scatter,coherence;
    product_reason_t reason;
} product_stage_summary_t;
typedef struct {
    product_stage_summary_t summary;
    size_t count;
    product_turn_t turns[PRODUCT_MAX_TURNS];
} product_run_t;
typedef struct {
    bool calculation_valid,engineering_available,accuracy_validated;
    unsigned matched_turns;
    double lower_rpm,upper_rpm,mean_rpm,original_peak,trial_peak,delta_peak;
    double delta_coherence,mean_noise_scale;
    balance_result_t calculation;
    product_reason_t reason;
} product_solution_t;
product_reason_t product_correction_status(const product_solution_t *);
typedef struct {
    bool available;unsigned matched_turns;
    double original_peak,retest_peak,improvement_percent,lower_rpm,upper_rpm;
    product_reason_t reason;
} product_comparison_t;

typedef enum { PRODUCT_REPEAT_UNASSESSED,PRODUCT_REPEAT_UNCOMPARABLE,
    PRODUCT_REPEAT_OBSERVATION_CLOSE,PRODUCT_REPEAT_VARIABLE } product_repeat_state_t;
typedef struct {
    product_repeat_state_t state;
    uint32_t previous_capture_id;
    unsigned matched_turns;
    bool phase_available;
    double previous_peak,current_peak,amplitude_change_percent,phase_change_deg;
    product_reason_t reason;
} product_repeat_observation_t;
typedef struct {
    uint32_t schema,id,revision;
    char boot_id[17],firmware[32];
    product_config_t config;
    product_stage_t stage;product_state_t state;product_reason_t reason;
    bool simulated,historical,storage_ok;
    uint64_t armed_us,deadline_us;
    product_stage_summary_t stages[3];
    product_solution_t solution;product_comparison_t comparison;
} product_snapshot_t;

bool product_config_valid(const product_config_t *);
product_config_t product_default_config(void);
const char *product_state_name(product_state_t);
const char *product_reason_name(product_reason_t);
/* Fully recorded, unchanged context; weak individual SNR alone is not a
 * rejection. Structural/transient/reference rejection bits remain enforced. */
bool product_extract_run(const selection_turn_t *,size_t,unsigned,product_run_t *);
bool product_solve_runs(const product_config_t *,const product_run_t *,const product_run_t *,product_solution_t *);
bool product_compare_runs(const product_config_t *,const product_run_t *,const product_run_t *,
    const product_solution_t *,product_comparison_t *);
/* Readiness count only; no response/phase/result used to end a recording. */
unsigned product_common_turns(const product_run_t *,const product_run_t *,double lower_rpm,double upper_rpm);
/* Same physical correction must be explicitly confirmed by the operator.
 * Uses the frozen calibration band and the same current/previous-RPM matcher.
 * 25% magnitude ratio / 30 degree coherent phase change are prototype warning
 * bands, not accuracy, uncertainty, correction-effect or acceptance targets. */
void product_observe_repeat(const product_config_t *,const product_run_t *,const product_run_t *,
    const product_solution_t *,product_repeat_observation_t *);
const char *product_repeat_state_name(product_repeat_state_t);
