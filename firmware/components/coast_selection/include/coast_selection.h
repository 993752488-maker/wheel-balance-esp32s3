#pragma once
#include "coast_analysis.h"

#define COAST_SELECTION_MAX_REFS 1024U
enum {
    SELECT_FIT=1U,SELECT_STARTUP=2U,SELECT_WEAK_PHASE=4U,SELECT_IMPULSE=8U,
    SELECT_DC_CHANGE=16U,SELECT_PHASE_CHANGE=32U,SELECT_AMPLITUDE_CHANGE=64U,
    SELECT_REFERENCE_RESTART=128U,SELECT_RESIDUAL_DECAY=256U
};
typedef enum { SELECTION_OK,SELECTION_ARGUMENT,SELECTION_INPUT,SELECTION_CAPACITY } selection_status_t;
typedef struct {sync_reference_t ref;bool has_period;} selection_reference_t;
typedef struct {
    coast_config_t fit;
    double max_residual_crest,min_transient_peak,max_dc_step;
    double max_neighbor_speed_change,max_phase_step_deg,max_amplitude_ratio;
    double max_residual_decay_ratio;
    unsigned startup_turns;
} selection_config_t;
typedef struct {
    coast_result_t fit;
    uint64_t first_sample_sequence,last_sample_sequence;
    uint32_t reasons[3];
    bool candidate[3]; /* diagnostic candidate, never a balance measurement */
    bool measurement_qualified; /* always false */
} selection_turn_t;

selection_config_t selection_default_config(void);
/* Caller owns bounded output; no heap/RTOS. Keeps individual-turn RPMs and
 * phasors. Detectable disturbances are marked, not silently averaged away.
 * Cannot confirm hand release, isolate same-order frame motion, detect every
 * systematically missed mark, or qualify phase/balance precision. */
selection_status_t coast_select(const selection_config_t *,const sync_sample_t *,size_t,
    const selection_reference_t *,size_t,sync_quality_t,selection_turn_t *,size_t,size_t *);
/* Identical calculations; optional task-context yield between fitted turns. */
selection_status_t coast_select_progress(const selection_config_t *,const sync_sample_t *,size_t,
    const selection_reference_t *,size_t,sync_quality_t,selection_turn_t *,size_t,size_t *,
    void (*progress)(void *),void *context);
/* Compare candidates only when BOTH current and preceding revolution-average
 * RPMs match: equal current RPM alone does not match the deceleration profile.
 * No mass, H, recommendation or cross-
 * speed normalization is computed. Caller must ensure the same physical setup. */
bool coast_select_pair(const selection_turn_t *,const selection_turn_t *,unsigned axis,
    double max_rpm_mismatch,double *mismatch,double *relative_complex_difference,double *phase_difference_deg);
