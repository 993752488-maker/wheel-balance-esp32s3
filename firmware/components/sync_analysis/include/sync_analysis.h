#ifndef SYNC_ANALYSIS_H
#define SYNC_ANALYSIS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SYNC_MAX_TURNS 64u
#define SYNC_MAX_SAMPLES 65536u

typedef enum {
    SYNC_OK = 0, SYNC_ERR_ARGUMENT, SYNC_ERR_TIME_QUALITY, SYNC_ERR_INPUT_FAULT,
    SYNC_ERR_SAMPLE_GAP, SYNC_ERR_REFERENCE, SYNC_ERR_EPOCH, SYNC_ERR_SAMPLE_TIME,
    SYNC_ERR_INSUFFICIENT, SYNC_ERR_UNSTABLE_RPM, SYNC_ERR_SINGULAR, SYNC_ERR_NUMERIC
} sync_status_t;
typedef enum { SYNC_TIME_UNKNOWN, SYNC_TIME_SYNTHETIC, SYNC_TIME_VALIDATED,
    SYNC_TIME_ESTIMATED } sync_time_quality_t;
enum { SYNC_INPUT_SENSOR_FAULT = 1u, SYNC_INPUT_SAMPLE_LOSS = 2u,
       SYNC_INPUT_REFERENCE_LOSS = 4u, SYNC_INPUT_SATURATION = 8u };
typedef struct { sync_time_quality_t time_quality; uint32_t faults; } sync_quality_t;
typedef struct {
    uint64_t sequence, time_us;
    uint32_t epoch;
    double acceleration[3]; /* caller supplies m/s^2, gravity/DC may be present */
} sync_sample_t;
typedef struct { uint64_t sequence, time_us; } sync_reference_t;
typedef struct {
    size_t min_turns, min_samples_per_turn;
    unsigned max_harmonic; /* 1..3; DC always fitted */
    uint64_t max_sample_gap_us;
    double max_angular_gap_fraction, max_rpm_cv, max_rpm_deviation;
    double min_phase_amplitude, min_coherence;
} sync_config_t;
typedef struct {
    double dc, real, imag, peak, rms, phase_deg;
    double mean_turn_peak, coherence, residual_rms;
    bool phase_valid;             /* mathematical phase, amplitude/coherence gate */
    bool measurement_qualified;   /* false for synthetic/estimated/unknown timing */
} sync_axis_t;
typedef struct {
    sync_status_t status;
    bool valid, simulated;
    size_t turns, samples;
    double rpm_mean, rpm_min, rpm_max, rpm_std, rpm_cv, max_rpm_deviation;
    sync_axis_t axis[3];
} sync_result_t;

sync_config_t sync_default_config(void);
/* References bracket complete revolutions; samples cover [first,last).
 * Fits DC plus harmonics per turn, then equally averages complex 1x values.
 * Convention: y = DC + A*cos(theta+phi), V = C-i*S = A*exp(i*phi).
 * Each theta is interpolated within its reference interval: constant speed
 * inside that revolution is an assumption, not a measured angular encoder.
 * No heap, GPIO, RTOS or global state. Batch FIFO readout times are NOT
 * qualified sample timestamps and must not be passed as VALIDATED timing.
 * Only SYNC_OK with result.valid is a completed mathematical analysis.
 * No mass/angle recommendation is implemented here. */
sync_status_t sync_analyze(const sync_config_t *config,
    const sync_sample_t *samples, size_t sample_count,
    const sync_reference_t *references, size_t reference_count,
    sync_quality_t quality, sync_result_t *result);
#ifdef __cplusplus
}
#endif
#endif
