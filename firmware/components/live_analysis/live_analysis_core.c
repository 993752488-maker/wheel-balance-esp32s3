#include "live_analysis_core.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NOMINAL_PERIOD_US 2000.0
#define MAX_TIME_ERROR_US 20000
#define RAW_TO_SI (9.80665 / 8192.0)

static void invalidate(live_analysis_core_t *c, live_analysis_state_t state)
{
    memset(&c->status.result, 0, sizeof(c->status.result));
    c->status.updated_us = 0;
    c->status.window_ref_start_sequence = c->status.window_ref_end_sequence = 0;
    c->status.window_start_us = c->status.window_end_us = 0;
    c->status.window_epoch = 0;
    c->status.state = state;
}

static void invalidate_coast(live_analysis_core_t *c, coast_status_t reason)
{
    memset(&c->status.coast,0,sizeof(c->status.coast));
    c->status.coast.status=reason;
    c->status.coast_updated_us=0;
}

void live_core_init(live_analysis_core_t *c, sync_sample_t *storage, size_t capacity)
{
    memset(c, 0, sizeof(*c));
    c->samples = storage;
    c->capacity = capacity;
    c->period_us = NOMINAL_PERIOD_US;
    c->status.enabled = storage && capacity >= 64;
    c->status.timing_estimated = true;
    c->status.sample_capacity = capacity;
    c->status.state = c->status.enabled ? LIVE_ANALYSIS_WAIT_SENSOR : LIVE_ANALYSIS_DISABLED;
}

void live_core_reset(live_analysis_core_t *c, live_analysis_state_t reason)
{
    c->count = c->reference_count = 0;
    c->last_sequence = c->anchor_sequence = c->analyzed_reference = 0;
    c->coast_analyzed_reference=0;
    c->last_observed_sequence = 0;
    c->have_anchor = false;
    c->period_us = NOMINAL_PERIOD_US;
    c->status.buffered_samples = c->status.buffered_references = 0;
    invalidate(c, reason);
    invalidate_coast(c,COAST_WAIT_INPUT);
}

static bool mapped_time(const live_analysis_core_t *c, uint64_t sequence, uint64_t *out)
{
    /* Subtract unsigned indices before conversion to retain precision. */
    const double delta = sequence >= c->anchor_sequence ?
        (double)(sequence-c->anchor_sequence) : -(double)(c->anchor_sequence-sequence);
    const double value = (double)c->anchor_us + delta*c->period_us;
    if (!isfinite(value) || value < 0 || value >= (double)INT64_MAX) return false;
    *out = (uint64_t)llround(value);
    return true;
}

static bool observe_count(live_analysis_core_t *c, const live_input_sample_t *s)
{
    if (s->batch_index) return c->have_anchor;
    if (s->pending_frames == 0 || s->count_begin_us < 0 ||
        s->count_end_us < s->count_begin_us ||
        s->count_end_us-s->count_begin_us > MAX_TIME_ERROR_US ||
        s->sequence > UINT64_MAX-(s->pending_frames-1U)) return false;
    const uint64_t newest = s->sequence + s->pending_frames-1U;
    const int64_t midpoint = s->count_begin_us+(s->count_end_us-s->count_begin_us)/2;
    /* COUNT can observe the last complete frame anywhere within a sample
     * period. Mid-period is an estimate, never a measured sample instant.
     * Full pending_frames includes unread catch-up batches beyond 32 frames. */
    const int64_t observed = midpoint-1000;
    if (observed < 0) return false;
    if (!c->have_anchor) {
        c->anchor_sequence = newest;
        c->anchor_us = observed;
        c->have_anchor = true;
    } else {
        if (newest < c->last_observed_sequence || observed < c->last_observed_us) return false;
        uint64_t predicted;
        if (!mapped_time(c, newest, &predicted)) return false;
        c->status.last_time_error_us = observed-(int64_t)predicted;
        if (llabs(c->status.last_time_error_us) > MAX_TIME_ERROR_US) return false;
        /* Estimate the sensor rate from cumulative complete-frame sequence
         * and host count observations. At least 0.5 s at nominal rate avoids
         * fitting a few milliseconds of count/transaction quantization.
         * This affine clock is still uncalibrated and therefore ESTIMATED. */
        if (newest > c->last_observed_sequence && newest > c->anchor_sequence &&
            newest-c->anchor_sequence >= 250U) {
            const double estimate = (double)(observed-c->anchor_us)/(double)(newest-c->anchor_sequence);
            if (!isfinite(estimate) || estimate < 1900 || estimate > 2100) return false;
            c->period_us = estimate;
        }
    }
    c->last_observed_sequence = newest;
    c->last_observed_us = observed;
    return true;
}

void live_core_sample(live_analysis_core_t *c, const live_input_sample_t *s)
{
    if (!c || !s || !c->status.enabled) return;
    if (!s->sequence || !s->epoch || s->batch_index >= s->pending_frames) {
        live_core_reset(c, LIVE_ANALYSIS_INPUT_LOSS);
        return;
    }
    if (c->last_sequence && (s->epoch != c->epoch || c->last_sequence == UINT64_MAX ||
        s->sequence != c->last_sequence+1U)) {
        live_core_reset(c, LIVE_ANALYSIS_INPUT_LOSS);
        if (s->batch_index) return;
    }
    c->epoch = s->epoch;
    if (!c->have_anchor && s->batch_index) return; /* wait for a full batch observation */
    if (!observe_count(c, s)) {
        ++c->status.timing_resets;
        live_core_reset(c, LIVE_ANALYSIS_TIMING_ERROR);
        return;
    }
    uint64_t time;
    if (!mapped_time(c, s->sequence, &time)) {
        ++c->status.timing_resets;
        live_core_reset(c, LIVE_ANALYSIS_TIMING_ERROR);
        return;
    }
    for (unsigned axis=0; axis<3; ++axis) if (s->raw[axis] <= -32760 || s->raw[axis] >= 32760) {
        live_core_reset(c, LIVE_ANALYSIS_SATURATION);
        return;
    }
    if (c->count == c->capacity) {
        /* Bounded bulk eviction rather than a full memmove per 500 Hz sample. */
        const size_t remove = c->capacity/8U;
        memmove(c->samples, c->samples+remove, (c->count-remove)*sizeof(*c->samples));
        c->count -= remove;
        invalidate(c, LIVE_ANALYSIS_INSUFFICIENT);
        invalidate_coast(c,COAST_ERR_SAMPLES);
    }
    sync_sample_t *target = &c->samples[c->count++];
    *target = (sync_sample_t){.sequence=s->sequence, .time_us=time, .epoch=s->epoch};
    for (unsigned axis=0; axis<3; ++axis) target->acceleration[axis] = s->raw[axis]*RAW_TO_SI;
    c->last_sequence = s->sequence;
    c->status.buffered_samples = c->count;
    if (c->status.state == LIVE_ANALYSIS_WAIT_SENSOR) c->status.state = LIVE_ANALYSIS_WAIT_REFERENCE;
}

void live_core_reference(live_analysis_core_t *c, uint64_t sequence, uint64_t time_us)
{
    if (!c || !c->status.enabled) return;
    if (!sequence || time_us > INT64_MAX || (c->reference_count &&
        (c->references[c->reference_count-1].sequence == UINT64_MAX ||
         sequence != c->references[c->reference_count-1].sequence+1U ||
         time_us <= c->references[c->reference_count-1].time_us))) {
        live_core_reset(c, LIVE_ANALYSIS_INPUT_LOSS);
        return;
    }
    if (c->reference_count == LIVE_REFERENCE_CAPACITY) {
        memmove(c->references, c->references+1, (LIVE_REFERENCE_CAPACITY-1)*sizeof(*c->references));
        --c->reference_count;
    }
    c->references[c->reference_count++] = (sync_reference_t){.sequence=sequence, .time_us=time_us};
    c->status.buffered_references = c->reference_count;
    if (c->reference_count < 5) invalidate(c, LIVE_ANALYSIS_COLLECTING);
}

static void analyze_coast(live_analysis_core_t *c,int64_t now_us)
{
    const sync_reference_t *ref=c->references+c->reference_count-3;
    if(ref[2].sequence==c->coast_analyzed_reference)return;
    if(c->samples[c->count-1].time_us<ref[2].time_us) {
        invalidate_coast(c,COAST_WAIT_INPUT);return;
    }
    size_t first=0,end=0;
    while(first<c->count && c->samples[first].time_us<ref[1].time_us)++first;
    end=first;
    while(end<c->count && c->samples[end].time_us<ref[2].time_us)++end;
    c->coast_analyzed_reference=ref[2].sequence;
    if(end==first) {invalidate_coast(c,COAST_ERR_SAMPLES);return;}
    const coast_config_t cfg=coast_default_config();
    coast_result_t result;
    if(coast_analyze(&cfg,c->samples+first,end-first,ref,
        (sync_quality_t){SYNC_TIME_ESTIMATED,0},&result)==COAST_OK) {
        c->status.coast=result;c->status.coast_updated_us=now_us;
        ++c->status.coast_sequence;
    } else {
        invalidate_coast(c,result.status);
    }
}

bool live_core_retime(live_analysis_core_t *c)
{
    if(!c || !c->status.enabled || !c->have_anchor || !c->count)return false;
    for(size_t i=0;i<c->count;++i) {
        if(!mapped_time(c,c->samples[i].sequence,&c->samples[i].time_us))return false;
        if(i && c->samples[i].time_us<=c->samples[i-1].time_us)return false;
    }
    return true;
}

void live_core_analyze(live_analysis_core_t *c, int64_t now_us)
{
    if (!c || !c->status.enabled || !c->have_anchor || c->reference_count < 3 || !c->count) return;
    const uint64_t latest=c->references[c->reference_count-1].sequence;
    const bool stable_pending=c->reference_count>=5 && latest!=c->analyzed_reference;
    if(!stable_pending && latest==c->coast_analyzed_reference)return;
    /* Reconstruct the whole window from one current affine clock. Updating
     * its rate can never make adjacent sample times inconsistent. */
    if(!live_core_retime(c)) {
        ++c->status.timing_resets;
        live_core_reset(c,LIVE_ANALYSIS_TIMING_ERROR);
        return;
    }
    analyze_coast(c,now_us);
    if(c->reference_count<5)return;
    const sync_reference_t *ref = c->references+c->reference_count-5;
    if(ref[4].sequence==c->analyzed_reference)return;
    if (c->samples[c->count-1].time_us < ref[4].time_us) {
        invalidate(c, LIVE_ANALYSIS_COLLECTING);
        return; /* trailing-edge reporting does not guarantee FIFO caught up */
    }
    if (c->samples[0].time_us > ref[0].time_us &&
        c->samples[0].time_us-ref[0].time_us > 4000U) {
        invalidate(c, LIVE_ANALYSIS_INSUFFICIENT);
        return;
    }
    size_t first=0, end=0;
    while (first<c->count && c->samples[first].time_us < ref[0].time_us) ++first;
    end=first;
    while (end<c->count && c->samples[end].time_us < ref[4].time_us) ++end;
    if (end == first) { invalidate(c, LIVE_ANALYSIS_INSUFFICIENT); return; }
    sync_config_t cfg = sync_default_config();
    sync_result_t result;
    const sync_status_t err = sync_analyze(&cfg, c->samples+first, end-first, ref, 5,
        (sync_quality_t){SYNC_TIME_ESTIMATED, 0}, &result);
    c->analyzed_reference = ref[4].sequence;
    if (err == SYNC_OK) {
        c->status.result = result;
        c->status.state = LIVE_ANALYSIS_READY_ESTIMATED;
        c->status.updated_us = now_us;
        c->status.window_ref_start_sequence = ref[0].sequence;
        c->status.window_ref_end_sequence = ref[4].sequence;
        c->status.window_start_us = ref[0].time_us;
        c->status.window_end_us = ref[4].time_us;
        c->status.window_epoch = c->epoch;
        ++c->status.result_sequence;
    } else {
        invalidate(c, err == SYNC_ERR_UNSTABLE_RPM ? LIVE_ANALYSIS_UNSTABLE_RPM : LIVE_ANALYSIS_INSUFFICIENT);
        /* Preserve quality diagnostics (RPM CV etc), never valid axes. */
        c->status.result = result;
    }
    /* Retain the latest four-turn beginning plus its preceding sample. */
    if (first > 0) {
        const size_t remove = first-1;
        memmove(c->samples, c->samples+remove, (c->count-remove)*sizeof(*c->samples));
        c->count -= remove;
        c->status.buffered_samples = c->count;
    }
}
