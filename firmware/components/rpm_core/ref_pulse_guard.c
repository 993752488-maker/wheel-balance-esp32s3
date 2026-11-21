#include "ref_pulse_guard.h"
void ref_pulse_guard_resume(ref_pulse_guard_t *g,bool level,uint64_t now)
{
    if(!g)return;
    if(g->have_sample&&now<g->last_sample_us&&g->clock_faults<UINT32_MAX)++g->clock_faults;
    g->level=level;g->armed=false;g->candidate=false;g->candidate_resynced=false;g->last_pulse_resynced=false;
    g->changed_us=now;g->last_sample_us=now;g->have_sample=false;
}

static void increment(uint32_t *v)
{
    if (*v < UINT32_MAX) ++*v;
}

void ref_pulse_guard_init(ref_pulse_guard_t *g, bool active_high,
                          bool initial_level, uint64_t now, uint64_t stable_us)
{
    *g = (ref_pulse_guard_t){.active_high = active_high, .level = initial_level,
                            .changed_us = now, .stable_us = stable_us};
}

ref_guard_result_t ref_pulse_guard_edge(ref_pulse_guard_t *g, bool level,
                                       uint64_t now, uint64_t *leading_us)
{
    if (now < g->changed_us) {
        increment(&g->clock_faults);
        g->armed = g->candidate = false;
        g->candidate_resynced = false;
        g->level = level;
        g->changed_us = now;
        return REF_GUARD_CLOCK_FAULT;
    }
    if (level == g->level) {
        /* The pin returned before ISR observation. Restart the observed run
         * here, with explicit timestamp uncertainty. Clearing the active
         * candidate used to discard even a subsequent long, stable mark. */
        increment(&g->level_resyncs);
        if (level != g->active_high && now - g->changed_us >= g->stable_us)
            g->armed = true;
        g->candidate = level == g->active_high && g->armed;
        g->candidate_us = now;
        g->candidate_resynced = g->candidate;
        g->changed_us = now;
        return REF_GUARD_NONE;
    }
    const uint64_t duration = now - g->changed_us;
    ref_guard_result_t result = REF_GUARD_NONE;
    if (level == g->active_high) {
        if (duration >= g->stable_us) g->armed = true;
        g->candidate = g->armed;
        g->candidate_us = now;
        g->candidate_resynced = false;
    } else {
        if (g->candidate && g->armed && duration >= g->stable_us) {
            *leading_us = g->candidate_us;
            g->last_pulse_resynced = g->candidate_resynced;
            if (g->last_pulse_resynced) increment(&g->resynced_pulses);
            result = REF_GUARD_PULSE;
            g->armed = false;
        } else {
            increment(&g->rejected_pulses);
        }
        g->candidate = false;
        g->candidate_resynced = false;
    }
    g->level = level;
    g->changed_us = now;
    return result;
}

ref_guard_result_t ref_pulse_guard_sample(ref_pulse_guard_t *g, bool level,
                                         uint64_t now, uint64_t *observed_us)
{
    if (g->have_sample) {
        if (now < g->last_sample_us) {
            increment(&g->clock_faults);
            g->armed = g->candidate = g->candidate_resynced = false;
            g->level = level;
            g->changed_us = g->last_sample_us = now;
            return REF_GUARD_CLOCK_FAULT;
        }
        const uint64_t gap = now - g->last_sample_us;
        if (gap > g->max_sample_gap_us) g->max_sample_gap_us = gap;
        if (gap > g->stable_us) {
            increment(&g->sample_gaps);
            g->armed = g->candidate = g->candidate_resynced = false;
            g->level = level;
            g->changed_us = g->last_sample_us = now;
            return REF_GUARD_NONE;
        }
    } else {
        /* No observed interval exists before the first timer callback. */
        g->level = level;
        g->changed_us = now;
        g->armed = g->candidate = g->candidate_resynced = false;
    }
    g->have_sample = true;
    g->last_sample_us = now;
    if (level == g->level) return REF_GUARD_NONE;
    const ref_guard_result_t result = ref_pulse_guard_edge(g, level, now, observed_us);
    if (g->candidate) g->candidate_resynced = true;
    if (result == REF_GUARD_PULSE && !g->last_pulse_resynced) {
        g->last_pulse_resynced = true;
        increment(&g->resynced_pulses);
    }
    return result;
}
