#include "live_analysis_record.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

size_t live_analysis_record(char *out, size_t capacity,
                            const live_analysis_status_t *s, unsigned axis)
{
    if (!out || !capacity) return 0;
    out[0]='\0';
    if (!s || axis>2 || !s->enabled || !s->timing_estimated || s->measurement_qualified ||
        s->state!=LIVE_ANALYSIS_READY_ESTIMATED || !s->result.valid || s->result.simulated ||
        !s->result_sequence || !s->window_epoch || !s->window_ref_start_sequence ||
        s->window_ref_end_sequence<=s->window_ref_start_sequence || s->result.turns<4 ||
        s->window_ref_end_sequence-s->window_ref_start_sequence!=s->result.turns ||
        !s->window_start_us || s->window_end_us<=s->window_start_us ||
        s->updated_us<=0 || (uint64_t)s->updated_us<s->window_end_us ||
        !isfinite(s->result.rpm_mean) || s->result.rpm_mean<=0 ||
        !isfinite(s->result.rpm_cv) || s->result.rpm_cv<0 || s->result.rpm_cv>1) return 0;
    const sync_axis_t *a=&s->result.axis[axis];
    if (a->measurement_qualified || !isfinite(a->real) || !isfinite(a->imag) ||
        !isfinite(a->peak) || a->peak<0 || !isfinite(a->rms) || a->rms<0 ||
        !isfinite(a->coherence) || a->coherence<0 || a->coherence>1 ||
        !isfinite(a->residual_rms) || a->residual_rms<0) return 0;
    char phase[32]="NA";
    if (a->phase_valid) {
        if (!isfinite(a->phase_deg) || a->phase_deg<0 || a->phase_deg>=360) return 0;
        snprintf(phase,sizeof(phase),"%.9g",a->phase_deg);
    }
    const int n=snprintf(out,capacity,
        "ONE_X source=live_estimated qualified=0 seq=%" PRIu64 " epoch=%" PRIu32
        " ref_first=%" PRIu64 " ref_last=%" PRIu64 " start_us=%" PRIu64 " end_us=%" PRIu64
        " updated_us=%" PRId64 " turns=%u samples=%u axis=%c rpm=%.9g cv=%.9g"
        " real=%.9g imag=%.9g peak=%.9g rms=%.9g phase=%s phase_valid=%d"
        " coherence=%.9g residual=%.9g\n",
        s->result_sequence,s->window_epoch,s->window_ref_start_sequence,s->window_ref_end_sequence,
        s->window_start_us,s->window_end_us,s->updated_us,(unsigned)s->result.turns,
        (unsigned)s->result.samples,"XYZ"[axis],s->result.rpm_mean,s->result.rpm_cv,
        a->real,a->imag,a->peak,a->rms,phase,a->phase_valid,a->coherence,a->residual_rms);
    if (n<0 || (size_t)n>=capacity) { out[0]='\0'; return 0; }
    return (size_t)n;
}

size_t live_coast_record(char *out,size_t capacity,const live_analysis_status_t *s,unsigned axis)
{
    if(!out || !capacity)return 0;
    out[0]='\0';
    if(!s || axis>2 || !s->enabled || !s->timing_estimated || s->measurement_qualified ||
        !s->coast.valid || s->coast.simulated || s->coast.measurement_qualified ||
        s->coast.status!=COAST_OK || !s->coast_sequence || s->coast_updated_us<=0 ||
        !s->coast.epoch || !s->coast.previous_ref_sequence ||
        s->coast.previous_ref_sequence==UINT64_MAX || s->coast.ref_first!=s->coast.previous_ref_sequence+1 ||
        s->coast.ref_first==UINT64_MAX || s->coast.ref_last!=s->coast.ref_first+1 ||
        s->coast.end_us<=s->coast.start_us || s->coast.start_us<=s->coast.previous_ref_us ||
        (uint64_t)s->coast_updated_us<s->coast.end_us)return 0;
    const coast_result_t *r=&s->coast;const coast_axis_t *a=&r->axis[axis];
    if(!isfinite(r->rpm) || r->rpm<=0 || !isfinite(r->speed_change) || r->speed_change<0 ||
        !isfinite(r->phase_warp) || !isfinite(a->peak) || a->peak<0 ||
        !isfinite(a->real) || !isfinite(a->imag) || !isfinite(a->residual_rms) || a->residual_rms<0 ||
        !isfinite(a->fit_snr) || a->fit_snr<0)return 0;
    char phase[32]="NA";
    if(a->phase_available) {
        if(!isfinite(a->phase_estimate_deg) || a->phase_estimate_deg<0 || a->phase_estimate_deg>=360)return 0;
        snprintf(phase,sizeof(phase),"%.9g",a->phase_estimate_deg);
    }
    int n=snprintf(out,capacity,
        "COAST_X source=live_estimated diagnostic_only=1 qualified=0 drive_off_confirmed=0"
        " seq=%" PRIu64 " epoch=%" PRIu32 " ref_previous=%" PRIu64 " ref_first=%" PRIu64
        " ref_last=%" PRIu64 " previous_us=%" PRIu64 " start_us=%" PRIu64 " end_us=%" PRIu64
        " axis=%c samples=%u rpm=%.9g speed_change=%.9g warp=%.9g real=%.9g imag=%.9g"
        " peak=%.9g phase=%s phase_available=%d residual=%.9g fit_snr=%.9g\n",
        s->coast_sequence,r->epoch,r->previous_ref_sequence,r->ref_first,r->ref_last,
        r->previous_ref_us,r->start_us,r->end_us,"XYZ"[axis],(unsigned)r->samples,
        r->rpm,r->speed_change,r->phase_warp,a->real,a->imag,a->peak,phase,a->phase_available,
        a->residual_rms,a->fit_snr);
    if(n<0 || (size_t)n>=capacity){out[0]='\0';return 0;}
    return (size_t)n;
}
