#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct { double real, imag; } balance_complex_t;
typedef enum {
    BALANCE_OK, BALANCE_NO_CORRECTION, BALANCE_ERR_ARGUMENT,
    BALANCE_ERR_UNQUALIFIED, BALANCE_ERR_SOURCE, BALANCE_ERR_CONTEXT,
    BALANCE_ERR_UNSTABLE, BALANCE_ERR_SPEED, BALANCE_ERR_PHASE,
    BALANCE_ERR_WEAK_RESPONSE, BALANCE_ERR_LIMIT, BALANCE_ERR_NUMERIC
} balance_status_t;
typedef struct {
    balance_complex_t vibration; /* peak m/s^2; same axis/sign and reference */
    bool valid, phase_valid, qualified, simulated;
    double noise_rms, rpm, rpm_cv, coherence;
    uint32_t setup_id, epoch;
    unsigned axis; /* 0..2 */
    int direction; /* +1 clockwise, -1 counterclockwise from the fixed side */
    /* Captured live-window provenance; not needed for pure synthetic solves. */
    uint64_t result_sequence, window_ref_start_sequence, window_ref_end_sequence;
    uint64_t window_start_us, window_end_us;
} balance_measurement_t;
typedef struct {
    double trial_mass_g, trial_radius_mm, trial_angle_deg, correction_radius_mm;
    double max_rpm_cv, max_rpm_mismatch, min_coherence;
    double min_response, min_response_snr, zero_vibration, max_mass_g;
} balance_config_t;
typedef struct {
    balance_status_t status;
    bool calculation_valid, recommendation_valid, simulated, angle_valid;
    balance_complex_t influence, correction_g_mm;
    double mass_g, angle_deg, delta_peak, response_snr, rpm_mismatch;
} balance_result_t;

balance_config_t balance_default_config(void);
/* H=(V1-V0)/Ut; Uc=-V0/H. Angle is from the white mark, along rotation,
 * viewed from the SAME side as the trial. Uc assumes TRIAL WEIGHT REMOVED.
 * Synthetic calculations are allowed but can NEVER be recommendations.
 * Unknown/estimated hardware timing must set qualified=false and is rejected.
 * No heap/RTOS/hardware dependency. Errors clear all correction values. */
balance_status_t balance_solve(const balance_config_t *config,
    const balance_measurement_t *original, const balance_measurement_t *trial,
    balance_result_t *result);
const char *balance_status_name(balance_status_t status);
/* Same numerical solver and input checks, with uncalibrated engineering
 * estimates allowed. Never sets recommendation_valid. Caller establishes
 * data integrity, comparable speed and aggregate response applicability. */
balance_status_t balance_estimate(const balance_config_t *config,
    const balance_measurement_t *original,const balance_measurement_t *trial,
    balance_result_t *result);
