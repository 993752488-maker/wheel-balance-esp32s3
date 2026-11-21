#ifndef RPM_CORE_H
#define RPM_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pulses_per_revolution;
    uint64_t min_interval_us;
    uint64_t stop_timeout_us;
} rpm_core_config_t;

typedef enum {
    RPM_CORE_OK = 0,
    RPM_CORE_REFERENCE = 1,
    RPM_CORE_PERIOD = 2,
    RPM_CORE_REJECTED_BOUNCE = 3,
    RPM_CORE_REJECTED_TIME = 4,
    RPM_CORE_ERR_ARGUMENT = -1,
    RPM_CORE_ERR_NOT_INITIALIZED = -2
} rpm_core_result_t;

typedef struct {
    bool rpm_valid;
    bool has_reference;
    bool has_accepted_edge;
    double rpm;
    uint64_t period_us;
    uint64_t last_accepted_edge_us;
    uint64_t accepted_edges;
    uint64_t rejected_edges;
    uint64_t rejected_bounce_edges;
    uint64_t rejected_time_edges;
    uint64_t clock_faults;
    uint64_t timeouts;
} rpm_core_snapshot_t;

/* Public storage permits static allocation. Treat members as private. */
typedef struct {
    rpm_core_config_t config;
    uint64_t last_observed_us;
    uint64_t last_input_us;
    uint64_t last_accepted_us;
    uint64_t period_us;
    uint64_t accepted_edges;
    uint64_t rejected_edges;
    uint64_t rejected_bounce_edges;
    uint64_t rejected_time_edges;
    uint64_t clock_faults;
    uint64_t timeouts;
    bool initialized;
    bool has_observed_time;
    bool has_input_time;
    bool has_accepted_edge;
    bool has_reference;
    bool rpm_valid;
} rpm_core_t;

/* Requires PPR > 0 and 1 <= min_interval_us < stop_timeout_us.
 * Invalid configuration clears core so an earlier valid RPM cannot survive.
 */
rpm_core_result_t rpm_core_init(rpm_core_t *core,
                                const rpm_core_config_t *config);

/* Clears observations/statistics and retains the validated configuration. */
rpm_core_result_t rpm_core_reset(rpm_core_t *core);
/* Known observation interruption outside a recorded window. Preserve lifetime
 * counters, invalidate the period; two new references are required again. */
rpm_core_result_t rpm_core_break_reference(rpm_core_t *core);

/* Use timestamps captured at the selected GPIO edge, in microseconds.
 * The first accepted edge (including after timeout/fault) establishes a
 * reference only. A second accepted edge is required for RPM.
 * Too-close edges are ignored without moving the accepted-edge reference.
 * Duplicate/backward timestamps are rejected and invalidate the reference.
 */
rpm_core_result_t rpm_core_on_pulse(rpm_core_t *core, uint64_t timestamp_us);

/* Mutates timeout state and returns a snapshot at now_us. Expiration is
 * inclusive: age >= stop_timeout_us invalidates RPM and the reference.
 * Backward time invalidates the reference and returns REJECTED_TIME.
 * Invalid RPM is always represented by rpm_valid=false, rpm=0, period_us=0.
 *
 * Serialize all calls in one task; this module is not ISR/thread safe.
 * Drain timestamped edge queues before requesting a snapshot. Time must not
 * move backward across either API. After a clock fault, time must catch up
 * to the previous maximum (or the caller must explicitly reset the core).
 * uint64_t timestamp wrap is treated as a clock fault, not as elapsed time.
 */
rpm_core_result_t rpm_core_get_snapshot(rpm_core_t *core, uint64_t now_us,
                                        rpm_core_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
