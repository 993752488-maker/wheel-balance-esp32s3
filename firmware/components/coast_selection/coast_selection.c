#include "coast_selection.h"
#include <math.h>
#include <string.h>

selection_config_t selection_default_config(void)
{
    return (selection_config_t){.fit=coast_default_config(),.max_residual_crest=8,
        .min_transient_peak=.05,.max_dc_step=.25,.max_neighbor_speed_change=.15,
        .max_phase_step_deg=35,.max_amplitude_ratio=2,.max_residual_decay_ratio=2,.startup_turns=1};
}
static bool positive(double x){return isfinite(x)&&x>0;}
static double angle_difference(double a,double b){return fmod(a-b+540,360)-180;}
selection_status_t coast_select_progress(const selection_config_t *cfg,const sync_sample_t *s,size_t ns,
    const selection_reference_t *refs,size_t nr,sync_quality_t q,selection_turn_t *out,size_t cap,size_t *count,
    void (*progress)(void *),void *context)
{
    if(count)*count=0;
    if(!count || !cfg || !s || !ns || ns>SYNC_MAX_SAMPLES || !out || (nr && !refs) ||
       nr>COAST_SELECTION_MAX_REFS || !positive(cfg->max_residual_crest) || !positive(cfg->min_transient_peak) ||
       !positive(cfg->max_dc_step) || !positive(cfg->max_neighbor_speed_change) || cfg->max_neighbor_speed_change>=1 ||
       !positive(cfg->max_phase_step_deg) || cfg->max_phase_step_deg>180 ||
       !positive(cfg->max_amplitude_ratio) || cfg->max_amplitude_ratio<=1 ||
       !positive(cfg->max_residual_decay_ratio) || cfg->max_residual_decay_ratio<=1 ||
       cfg->startup_turns>COAST_SELECTION_MAX_REFS)
        return SELECTION_ARGUMENT;
    if(q.faults || (q.time_quality!=SYNC_TIME_SYNTHETIC && q.time_quality!=SYNC_TIME_ESTIMATED))return SELECTION_INPUT;
    if(nr>=3 && cap<nr-2)return SELECTION_CAPACITY;
    for(size_t i=0;i<ns;++i) {
        if(!s[i].sequence || !s[i].epoch || (i && (s[i-1].sequence==UINT64_MAX ||
           s[i].sequence!=s[i-1].sequence+1 || s[i].epoch!=s[0].epoch || s[i].time_us<=s[i-1].time_us ||
           s[i].time_us-s[i-1].time_us>cfg->fit.max_sample_gap_us)))return SELECTION_INPUT;
        for(unsigned a=0;a<3;++a)if(!isfinite(s[i].acceleration[a]))return SELECTION_INPUT;
    }
    for(size_t i=0;i<nr;++i) {
        if(!refs[i].ref.sequence || (i && (refs[i-1].ref.sequence==UINT64_MAX ||
           refs[i].ref.sequence!=refs[i-1].ref.sequence+1 || refs[i].ref.time_us<=refs[i-1].ref.time_us)))return SELECTION_INPUT;
    }
    if(nr<3)return SELECTION_OK;
    memset(out,0,(nr-2)*sizeof(*out));
    size_t start=0,end=0,run_start=0;
    for(size_t i=0;i<nr;++i) {
        if(!refs[i].has_period)run_start=i;
        if(i<2)continue;
        selection_turn_t *t=&out[i-2];
        while(start<ns && s[start].time_us<refs[i-1].ref.time_us)++start;
        end=start;while(end<ns && s[end].time_us<refs[i].ref.time_us)++end;
        sync_reference_t triplet[3]={refs[i-2].ref,refs[i-1].ref,refs[i].ref};
        if(end>start){t->first_sample_sequence=s[start].sequence;t->last_sample_sequence=s[end-1].sequence;}
        /* Keep even rejected windows' bounds/RPM for an explanatory report. */
        (void)coast_analyze(&cfg->fit,s+start,end-start,triplet,q,&t->fit);
        if(t->fit.status==COAST_ERR_ARGUMENT)return SELECTION_ARGUMENT;
        if(!t->fit.valid) {
            t->fit.previous_ref_sequence=triplet[0].sequence;t->fit.ref_first=triplet[1].sequence;t->fit.ref_last=triplet[2].sequence;
            t->fit.previous_ref_us=triplet[0].time_us;t->fit.start_us=triplet[1].time_us;t->fit.end_us=triplet[2].time_us;
            t->fit.rpm=60e6/(double)(triplet[2].time_us-triplet[1].time_us);
        }
        for(unsigned a=0;a<3;++a) {
            const coast_axis_t *v=&t->fit.axis[a];
            if(!t->fit.valid)t->reasons[a]|=SELECT_FIT;
            if(i-run_start<=cfg->startup_turns)t->reasons[a]|=SELECT_STARTUP;
            if(i-run_start<2 || !refs[i-1].has_period || !refs[i].has_period)t->reasons[a]|=SELECT_REFERENCE_RESTART;
            if(t->fit.valid && !v->phase_available)t->reasons[a]|=SELECT_WEAK_PHASE;
            if(t->fit.valid && v->residual_peak>=cfg->min_transient_peak && v->residual_crest>cfg->max_residual_crest)t->reasons[a]|=SELECT_IMPULSE;
            /* A ringing transient need not have an impulsive crest factor.
             * Residuals exclude the full DC+1x+2x+3x fitted waveform. */
            if(t->fit.valid && v->residual_peak>=cfg->min_transient_peak &&
               v->residual_early_rms>cfg->max_residual_decay_ratio*fmax(v->residual_late_rms,1e-12))
                t->reasons[a]|=SELECT_RESIDUAL_DECAY;
        }
        if(progress)progress(context);
    }
    for(size_t i=1;i<nr-2;++i) {
        selection_turn_t *left=out+i-1,*right=out+i;
        if(!left->fit.valid || !right->fit.valid || left->fit.ref_last!=right->fit.ref_first ||
           (right->reasons[0]&SELECT_REFERENCE_RESTART) ||
           fabs(left->fit.rpm-right->fit.rpm)/fmax(left->fit.rpm,right->fit.rpm)>cfg->max_neighbor_speed_change)continue;
        for(unsigned a=0;a<3;++a) {
            const coast_axis_t *x=&left->fit.axis[a],*y=&right->fit.axis[a];uint32_t reason=0;
            if(fabs(x->dc-y->dc)>cfg->max_dc_step)reason|=SELECT_DC_CHANGE;
            if(x->phase_available && y->phase_available) {
                if(fabs(angle_difference(x->phase_estimate_deg,y->phase_estimate_deg))>cfg->max_phase_step_deg)reason|=SELECT_PHASE_CHANGE;
                if(fmax(x->peak,y->peak)>cfg->max_amplitude_ratio*fmin(x->peak,y->peak))reason|=SELECT_AMPLITUDE_CHANGE;
            }
            left->reasons[a]|=reason;right->reasons[a]|=reason;
        }
    }
    for(size_t i=0;i<nr-2;++i)for(unsigned a=0;a<3;++a)out[i].candidate[a]=out[i].reasons[a]==0;
    *count=nr-2;return SELECTION_OK;
}
selection_status_t coast_select(const selection_config_t *cfg,const sync_sample_t *s,size_t ns,
    const selection_reference_t *refs,size_t nr,sync_quality_t q,selection_turn_t *out,size_t cap,size_t *count)
{return coast_select_progress(cfg,s,ns,refs,nr,q,out,cap,count,NULL,NULL);}
bool coast_select_pair(const selection_turn_t *a,const selection_turn_t *b,unsigned axis,double maximum,
    double *mismatch,double *relative,double *phase)
{
    if(mismatch)*mismatch=0;
    if(relative)*relative=0;
    if(phase)*phase=0;
    if(!a || !b || axis>2 || !mismatch || !relative || !phase || !positive(maximum) || maximum>=1 ||
       !a->candidate[axis] || !b->candidate[axis] || !a->fit.valid || !b->fit.valid ||
       !positive(a->fit.rpm) || !positive(b->fit.rpm) || !positive(a->fit.rpm_previous) || !positive(b->fit.rpm_previous) ||
       !a->fit.epoch || a->fit.epoch!=b->fit.epoch ||
       a->fit.simulated!=b->fit.simulated)return false;
    const double diff=fabs(a->fit.rpm-b->fit.rpm)/fmax(a->fit.rpm,b->fit.rpm);
    const double previous=fabs(a->fit.rpm_previous-b->fit.rpm_previous)/fmax(a->fit.rpm_previous,b->fit.rpm_previous);
    if(diff>maximum || previous>maximum)return false;
    const coast_axis_t *x=&a->fit.axis[axis],*y=&b->fit.axis[axis];
    if(!x->phase_available || !y->phase_available || !positive(x->peak) || !positive(y->peak) ||
       !isfinite(x->real)||!isfinite(x->imag)||!isfinite(y->real)||!isfinite(y->imag)||
       !isfinite(x->phase_estimate_deg)||!isfinite(y->phase_estimate_deg))return false;
    const double d=hypot(x->real-y->real,x->imag-y->imag)/fmax(x->peak,y->peak);
    if(!isfinite(d))return false;
    *mismatch=diff;*relative=d;*phase=angle_difference(y->phase_estimate_deg,x->phase_estimate_deg);return true;
}
