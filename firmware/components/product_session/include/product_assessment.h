#pragma once
#include "product_engine.h"

#define PRODUCT_ASSESSMENT_BANDS PRODUCT_SAVED_BANDS
#define PRODUCT_ASSESSMENT_POLICY 5U
typedef enum { ASSESS_NONE,ASSESS_LIMITED,ASSESS_CLOSE,ASSESS_VARIABLE } product_assessment_state_t;
enum { ASSESS_SINGLE_BAND=1U,ASSESS_NO_SPLIT=2U,ASSESS_SPLIT_FAILED=4U,
    ASSESS_SPLIT_DIFFERENT=8U,ASSESS_BANDS_DIFFERENT=16U,ASSESS_AXES_DIFFERENT=32U };
typedef struct {
    double score,split_relative_difference,split_phase_difference;
    double original_real,original_imag;
    uint32_t split_available,split_valid;
} product_band_detail_t;
typedef struct {
    uint32_t lower_rpm,pairs,available,reason,split_available,split_valid;
    double mass_g,angle_deg,score,response_snr,split_relative_difference,split_phase_difference;
    double influence_real,influence_imag,original_real,original_imag;
} product_assessment_band_t;
/* New optional sidecar. Released snapshot/guide/retest/speed/window ABIs stay
 * unchanged. Ranges describe observed estimates, never accuracy or coverage. */
typedef struct {
    uint32_t schema,session_id,snapshot_revision,policy;
    char boot_id[17];
    uint32_t simulated,automatic_axis,axis_finalized,axis,state,flags;
    uint32_t capture_id[2],epoch,valid_bands,selected_band,axis_available_mask;
    uint32_t axis_conflict_mask; /* Policy 3 uses the former alignment padding. */
    uint64_t start_us[2],end_us[2];
    double axis_scores[3],axis_mass_g[3],axis_angle_deg[3];
    double mass_min_g,mass_max_g,max_relative_difference,max_phase_difference;
    uint32_t joint_used,joint_band_mask,joint_bands;
    double correction_real_g_mm,correction_imag_g_mm,model_residual_ratio;
    product_assessment_band_t bands[PRODUCT_ASSESSMENT_BANDS];
} product_assessment_t;

/* A pair belongs to the original turn's RPM band. The other turn may be on
 * either side of that boundary. Use one whole-window pairing for all bands. */
typedef struct {
    size_t count;
    struct { uint8_t original, measured; } turns[PRODUCT_MAX_TURNS];
} product_pairing_t;
product_reason_t product_match_runs(const product_run_t *,const product_run_t *,
    double lower_rpm,double upper_rpm,product_pairing_t *);
bool product_evaluate_matched_band(const product_config_t *,const product_run_t *,const product_run_t *,
    const product_pairing_t *,double lower_rpm,product_solution_t *,product_band_detail_t *);

/* Existing mathematical/fault gates, plus disjoint first/second-half review.
 * Half-record differences affect ranking and warnings, not a new hard gate. */
bool product_evaluate_band(const product_config_t *,const product_run_t *,const product_run_t *,
    double lower_rpm,product_solution_t *,product_band_detail_t *);
bool product_assess_runs(const product_config_t *,const product_run_t *,const product_run_t *,
    product_solution_t *,product_assessment_t *);
/* Original and trial contain all 3 axes from their respective frozen captures.
 * An internally contradictory axis remains visible in the report. */
bool product_assess_axes(const product_config_t *,const product_run_t original[3],const product_run_t trial[3],
    unsigned *selected_axis,product_solution_t *,product_assessment_t *);
void product_assessment_reset(product_assessment_t *,const product_snapshot_t *);
void product_assessment_bind(product_assessment_t *,const product_snapshot_t *);
bool product_assessment_valid(const product_assessment_t *,const product_snapshot_t *);
const char *product_assessment_state_name(product_assessment_state_t);
