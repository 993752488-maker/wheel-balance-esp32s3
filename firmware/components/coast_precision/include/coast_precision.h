#pragma once
#include "coast_analysis.h"

typedef enum { PRECISION_OK,PRECISION_ARGUMENT,PRECISION_INPUT,PRECISION_SAMPLES,PRECISION_NUMERIC } precision_status_t;
typedef struct {
    coast_config_t fit;
    size_t min_samples;
    double alpha; /* IID reference probability=1-alpha. Not physical coverage. */
} precision_config_t;
typedef struct {
    double real,imag,peak;
    double iid_radius,block_radius[3],radius;
    double phase_deg,phase_halfwidth_deg;
    bool phase_bounded;
} precision_axis_t;
typedef struct {
    precision_status_t status;
    bool valid,simulated,measurement_qualified; /* qualification always false */
    double alpha;
    coast_result_t fit;
    precision_axis_t axis[3];
} precision_result_t;
typedef struct {
    bool valid,resolved,phase_bounded,measurement_qualified;
    double real,imag,peak,radius,phase_deg,phase_halfwidth_deg,rpm_mismatch;
} precision_delta_t;

precision_config_t precision_default_config(void);
/* OLS IID confidence reference plus 4/8/16 contiguous-cluster score sensitivity
 * envelopes. Blocks are NOT independent normal observations: their envelope
 * has no certified coverage. No timing/model/mount uncertainty is included.
 * No heap/RTOS; diagnostics only; never qualifies physical phase/balance. */
precision_status_t coast_precision(const precision_config_t *,const sync_sample_t *,size_t,
    const sync_reference_t [3],sync_quality_t,precision_result_t *);
/* Caller must establish unchanged installation, side/direction, boot and
 * reference. Weak/undefined baseline phase is permitted. Both current and
 * preceding RPM must match. The radius sum is a sensitivity bound, NOT a
 * certified confidence bound. No H, mass or placement recommendation. */
bool coast_precision_delta(const precision_result_t *,const precision_result_t *,unsigned,
    double max_rpm_mismatch,bool same_setup_confirmed,precision_delta_t *);
