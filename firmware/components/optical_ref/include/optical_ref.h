#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "rpm_core.h"

typedef struct {
    bool enabled;
    bool timed_out;
    int input_level; /* -1 when disabled */
    uint32_t raw_edges;
    uint32_t glitch_rejected;
    uint32_t pulse_rejected;
    uint32_t level_resyncs;
    uint32_t clock_faults;
    uint32_t resynced_pulses;
    uint32_t sample_gaps;
    uint64_t max_sample_gap_us;
    uint32_t input_queue_drops;
    uint32_t report_queue_drops;
    uint32_t analysis_queue_drops;
    rpm_core_snapshot_t rpm;
} optical_ref_status_t;

typedef struct {
    /* Start of the observed active run; after resync it is not edge timing.
     * Reported after the trailing edge. No phase qualification in V0. */
    uint64_t timestamp_us;
    uint64_t accepted_sequence;
    bool has_period;
    bool timestamp_resynced;
} optical_ref_event_t;

esp_err_t optical_ref_start(void);
void optical_ref_get_status(optical_ref_status_t *status);
bool optical_ref_pop_event(optical_ref_event_t *event);
bool optical_ref_pop_analysis_event(optical_ref_event_t *event);
/* Single task owner, only while holding the idle raw-recorder maintenance
 * reservation. No recorded window may span this explicit reference break. */
esp_err_t optical_ref_maintenance_begin(void);
esp_err_t optical_ref_maintenance_end(void);

