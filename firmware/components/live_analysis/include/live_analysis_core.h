#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sync_analysis.h"
#include "coast_analysis.h"

typedef enum {
    LIVE_ANALYSIS_DISABLED, LIVE_ANALYSIS_WAIT_SENSOR, LIVE_ANALYSIS_WAIT_REFERENCE, LIVE_ANALYSIS_COLLECTING,
    LIVE_ANALYSIS_READY_ESTIMATED, LIVE_ANALYSIS_UNSTABLE_RPM, LIVE_ANALYSIS_INPUT_LOSS,
    LIVE_ANALYSIS_TIMING_ERROR, LIVE_ANALYSIS_INSUFFICIENT, LIVE_ANALYSIS_SATURATION
} live_analysis_state_t;

typedef struct {
    bool enabled;
    bool timing_estimated; /* always true for the current FIFO/polling pipeline */
    bool measurement_qualified; /* always false: never suitable for balancing */
    live_analysis_state_t state;
    sync_result_t result;
    uint64_t result_sequence;
    int64_t updated_us; /* last completed result; zero when no usable result */
    /* The half-open reference window used for this result; zero when invalid.
     * This is provenance, not a qualification of the polled timing. */
    uint64_t window_ref_start_sequence, window_ref_end_sequence;
    uint64_t window_start_us, window_end_us;
    uint32_t window_epoch;
    uint32_t sample_queue_drops, reference_queue_drops, timing_resets;
    int64_t last_time_error_us;
    size_t buffered_samples, buffered_references, sample_capacity;
    /* Separate variable-speed diagnostic. Never consumed by balance_workflow. */
    coast_result_t coast;
    uint64_t coast_sequence;
    int64_t coast_updated_us;
} live_analysis_status_t;

typedef struct {
    uint64_t sequence;
    uint32_t epoch;
    int64_t count_begin_us, count_end_us;
    uint16_t pending_frames, batch_index;
    int16_t raw[3];
} live_input_sample_t;

#define LIVE_REFERENCE_CAPACITY 32U
typedef struct {
    sync_sample_t *samples;
    size_t capacity, count, reference_count;
    sync_reference_t references[LIVE_REFERENCE_CAPACITY];
    uint64_t last_sequence, anchor_sequence, last_observed_sequence, analyzed_reference;
    uint64_t coast_analyzed_reference;
    uint32_t epoch;
    int64_t anchor_us, last_observed_us;
    double period_us;
    bool have_anchor;
    live_analysis_status_t status;
} live_analysis_core_t;

void live_core_init(live_analysis_core_t *core, sync_sample_t *storage, size_t capacity);
void live_core_reset(live_analysis_core_t *core, live_analysis_state_t reason);
void live_core_sample(live_analysis_core_t *core, const live_input_sample_t *sample);
void live_core_reference(live_analysis_core_t *core, uint64_t sequence, uint64_t time_us);
/* Batch replay also uses the same current affine FIFO clock. This does not
 * change the clock's ESTIMATED qualification or perform any vibration fit. */
bool live_core_retime(live_analysis_core_t *core);
void live_core_analyze(live_analysis_core_t *core, int64_t now_us);

