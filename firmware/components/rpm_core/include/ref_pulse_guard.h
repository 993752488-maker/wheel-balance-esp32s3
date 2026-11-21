#ifndef REF_PULSE_GUARD_H
#define REF_PULSE_GUARD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    REF_GUARD_NONE,
    REF_GUARD_PULSE,
    REF_GUARD_CLOCK_FAULT
} ref_guard_result_t;

typedef struct {
    bool active_high, level, armed, candidate;
    bool candidate_resynced, last_pulse_resynced;
    uint64_t stable_us, changed_us, candidate_us;
    uint32_t rejected_pulses, level_resyncs, clock_faults;
    uint32_t resynced_pulses;
    bool have_sample;
    uint64_t last_sample_us, max_sample_gap_us;
    uint32_t sample_gaps;
} ref_pulse_guard_t;

/* One owner, no allocation. Both levels must last stable_us before another
 * complete mark can count. A pulse is reported at its trailing edge, using
 * the beginning of the latest observed active run, never the report time.
 * A resynchronized run has no precise leading-edge evidence and is flagged
 * last_pulse_resynced. It cannot qualify phase accuracy.
 * stable_us must be nonzero. Startup on a mark cannot invent a first mark. */
void ref_pulse_guard_init(ref_pulse_guard_t *g, bool active_high,
                          bool initial_level, uint64_t now, uint64_t stable_us);
/* Controlled idle maintenance only; preserves fault/rejection counters and
 * requires a fresh complete inactive/active cycle after observation resumes. */
void ref_pulse_guard_resume(ref_pulse_guard_t *g,bool level,uint64_t now);
ref_guard_result_t ref_pulse_guard_edge(ref_pulse_guard_t *g, bool level,
                                       uint64_t now, uint64_t *leading_us);
/* Periodic level observation, not an edge ISR. Repeated unchanged samples
 * preserve a candidate. Gaps longer than stable_us invalidate the current
 * cycle; callers must keep RPM invalid after sample_gaps until reset. */
ref_guard_result_t ref_pulse_guard_sample(ref_pulse_guard_t *g, bool level,
                                         uint64_t now, uint64_t *observed_us);

#endif
