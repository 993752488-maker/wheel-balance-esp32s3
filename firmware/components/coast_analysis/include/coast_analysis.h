#pragma once
#include "sync_analysis.h"

typedef enum {
    COAST_WAIT_INPUT, COAST_OK, COAST_ERR_ARGUMENT, COAST_ERR_INPUT,
    COAST_ERR_REFERENCE, COAST_ERR_SPEED_CHANGE, COAST_ERR_ACCELERATING,
    COAST_ERR_SAMPLES, COAST_ERR_NUMERIC
} coast_status_t;

typedef struct {
    size_t min_samples;
    uint64_t max_sample_gap_us;
    double max_speed_change, acceleration_tolerance, max_angular_gap_fraction;
    double min_phase_peak, min_fit_snr;
} coast_config_t;

typedef struct {
    double dc, real, imag, peak, rms, residual_rms, phase_estimate_deg, fit_snr;
    double residual_peak, residual_crest; /* after fitting DC and 1x/2x/3x */
    /* Split by elapsed time within the turn, not by sample count. These
     * describe nonstationary fitted residuals, never a hand-release proof. */
    double residual_early_rms, residual_late_rms;
    /* Fit/amplitude/SNR gate only. A single turn has no inter-turn coherence. */
    bool phase_available;
} coast_axis_t;

typedef struct {
    coast_status_t status;
    bool valid, simulated, measurement_qualified; /* qualification always false */
    size_t samples;
    uint32_t epoch;
    uint64_t previous_ref_sequence, ref_first, ref_last;
    uint64_t previous_ref_us, start_us, end_us;
    double rpm_previous, rpm, speed_change, phase_warp;
    coast_axis_t axis[3];
} coast_result_t;

coast_config_t coast_default_config(void);
const char *coast_status_name(coast_status_t status);
/* Diagnostic only: uses the newest turn [refs[1], refs[2]) with a quadratic
 * angle passing through 3 once-per-turn references. This is exact for a
 * constant-angular-acceleration model, not a proof of real angular position.
 * Samples must be consecutive full-rate input, not the decimated web stream.
 * Does not confirm drive disengagement, average unlike RPMs, measure
 * inter-turn coherence, calculate influence coefficients or balance weights.
 * ESTIMATED/SYNTHETIC only; never eligible as a balancing measurement. */
coast_status_t coast_analyze(const coast_config_t *config,
    const sync_sample_t *samples, size_t count, const sync_reference_t refs[3],
    sync_quality_t quality, coast_result_t *result);
