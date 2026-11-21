#include "rpm_core.h"

#include <stddef.h>
#include <string.h>

static void increment_saturated(uint64_t *value)
{
    if (*value < UINT64_MAX) {
        ++*value;
    }
}

static void invalidate_reference(rpm_core_t *core)
{
    core->has_reference = false;
    core->rpm_valid = false;
    core->period_us = 0;
}

static bool time_is_backward(const rpm_core_t *core, uint64_t timestamp_us)
{
    return core->has_observed_time && timestamp_us < core->last_observed_us;
}

static void observe_time(rpm_core_t *core, uint64_t timestamp_us)
{
    core->last_observed_us = timestamp_us;
    core->has_observed_time = true;
}

static void expire_if_needed(rpm_core_t *core, uint64_t now_us)
{
    /* The caller has established monotonic time before this subtraction. */
    if (core->has_reference &&
        now_us - core->last_accepted_us >= core->config.stop_timeout_us) {
        invalidate_reference(core);
        increment_saturated(&core->timeouts);
    }
}

static void fill_snapshot(const rpm_core_t *core, rpm_core_snapshot_t *snapshot)
{
    snapshot->rpm_valid = core->rpm_valid;
    snapshot->has_reference = core->has_reference;
    snapshot->has_accepted_edge = core->has_accepted_edge;
    /* Divide separately: period_us * PPR could overflow uint64_t. */
    snapshot->rpm = core->rpm_valid
                        ? (60000000.0 / (double)core->period_us) /
                              (double)core->config.pulses_per_revolution
                        : 0.0;
    snapshot->period_us = core->rpm_valid ? core->period_us : 0;
    snapshot->last_accepted_edge_us = core->last_accepted_us;
    snapshot->accepted_edges = core->accepted_edges;
    snapshot->rejected_edges = core->rejected_edges;
    snapshot->rejected_bounce_edges = core->rejected_bounce_edges;
    snapshot->rejected_time_edges = core->rejected_time_edges;
    snapshot->clock_faults = core->clock_faults;
    snapshot->timeouts = core->timeouts;
}

rpm_core_result_t rpm_core_init(rpm_core_t *core,
                                const rpm_core_config_t *config)
{
    rpm_core_config_t saved_config;
    if (core == NULL) {
        return RPM_CORE_ERR_ARGUMENT;
    }
    if (config == NULL) {
        memset(core, 0, sizeof(*core));
        return RPM_CORE_ERR_ARGUMENT;
    }
    saved_config = *config;
    memset(core, 0, sizeof(*core));
    if (saved_config.pulses_per_revolution == 0 ||
        saved_config.min_interval_us == 0 ||
        saved_config.stop_timeout_us <= saved_config.min_interval_us) {
        return RPM_CORE_ERR_ARGUMENT;
    }
    core->config = saved_config;
    core->initialized = true;
    return RPM_CORE_OK;
}

rpm_core_result_t rpm_core_reset(rpm_core_t *core)
{
    if (core == NULL) {
        return RPM_CORE_ERR_ARGUMENT;
    }
    if (!core->initialized) {
        return RPM_CORE_ERR_NOT_INITIALIZED;
    }
    return rpm_core_init(core, &core->config);
}

rpm_core_result_t rpm_core_on_pulse(rpm_core_t *core, uint64_t timestamp_us)
{
    uint64_t interval_us;
    if (core == NULL) {
        return RPM_CORE_ERR_ARGUMENT;
    }
    if (!core->initialized) {
        return RPM_CORE_ERR_NOT_INITIALIZED;
    }
    if (time_is_backward(core, timestamp_us) ||
        (core->has_input_time && timestamp_us <= core->last_input_us)) {
        increment_saturated(&core->rejected_edges);
        increment_saturated(&core->rejected_time_edges);
        increment_saturated(&core->clock_faults);
        invalidate_reference(core);
        return RPM_CORE_REJECTED_TIME;
    }
    observe_time(core, timestamp_us);
    core->last_input_us = timestamp_us;
    core->has_input_time = true;
    expire_if_needed(core, timestamp_us);
    if (!core->has_reference) {
        core->last_accepted_us = timestamp_us;
        core->has_accepted_edge = true;
        core->has_reference = true;
        increment_saturated(&core->accepted_edges);
        return RPM_CORE_REFERENCE;
    }
    interval_us = timestamp_us - core->last_accepted_us;
    if (interval_us < core->config.min_interval_us) {
        increment_saturated(&core->rejected_edges);
        increment_saturated(&core->rejected_bounce_edges);
        return RPM_CORE_REJECTED_BOUNCE;
    }
    core->last_accepted_us = timestamp_us;
    core->period_us = interval_us;
    core->rpm_valid = true;
    increment_saturated(&core->accepted_edges);
    return RPM_CORE_PERIOD;
}
rpm_core_result_t rpm_core_break_reference(rpm_core_t *core)
{
    if(!core)return RPM_CORE_ERR_ARGUMENT;
    if(!core->initialized)return RPM_CORE_ERR_NOT_INITIALIZED;
    invalidate_reference(core);
    core->has_accepted_edge=false;
    return RPM_CORE_OK;
}

rpm_core_result_t rpm_core_get_snapshot(rpm_core_t *core, uint64_t now_us,
                                        rpm_core_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return RPM_CORE_ERR_ARGUMENT;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    if (core == NULL) {
        return RPM_CORE_ERR_ARGUMENT;
    }
    if (!core->initialized) {
        return RPM_CORE_ERR_NOT_INITIALIZED;
    }
    if (time_is_backward(core, now_us)) {
        increment_saturated(&core->clock_faults);
        invalidate_reference(core);
        fill_snapshot(core, snapshot);
        return RPM_CORE_REJECTED_TIME;
    }
    observe_time(core, now_us);
    expire_if_needed(core, now_us);
    fill_snapshot(core, snapshot);
    return RPM_CORE_OK;
}
