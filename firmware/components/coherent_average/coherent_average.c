#include "coherent_average.h"
#include <math.h>
#include <string.h>

static coherent_status_t fail(coherent_average_t *c,coherent_status_t status)
{ if(c && c->failure==COHERENT_ADDED)c->failure=status;return status; }
bool coherent_average_init(coherent_average_t *c,const coherent_config_t *cfg)
{
    if(!c)return false;
    memset(c,0,sizeof(*c));
    if(!cfg || !isfinite(cfg->lower_rpm) || !isfinite(cfg->upper_rpm) ||
       cfg->lower_rpm<0 || cfg->upper_rpm<=cfg->lower_rpm ||
       cfg->min_turns<2 || cfg->min_turns>COHERENT_MAX_TURNS || !cfg->epoch ||
       (cfg->time_quality!=SYNC_TIME_SYNTHETIC && cfg->time_quality!=SYNC_TIME_ESTIMATED)) {
        c->failure=COHERENT_ARGUMENT;return false;
    }
    c->config=*cfg;c->initialized=true;return true;
}
coherent_status_t coherent_average_push(coherent_average_t *c,const coherent_input_t *v)
{
    if(!c)return COHERENT_ARGUMENT;
    if(c->failure!=COHERENT_ADDED)return c->failure;
    if(!c->initialized || !v)return fail(c,COHERENT_ARGUMENT);
    if(v->faults || v->epoch!=c->config.epoch)return fail(c,COHERENT_CONTEXT);
    if(!v->reference_sequence || !v->time_us || (c->last_reference &&
       (v->reference_sequence<=c->last_reference || v->time_us<=c->last_time_us)))return fail(c,COHERENT_ORDER);
    if(!isfinite(v->rpm) || v->rpm<=0 || !isfinite(v->real) || !isfinite(v->imag))return fail(c,COHERENT_NUMERIC);
    c->last_reference=v->reference_sequence;c->last_time_us=v->time_us;
    if(v->rpm<c->config.lower_rpm || v->rpm>=c->config.upper_rpm)return COHERENT_SKIPPED;
    if(c->count==COHERENT_MAX_TURNS)return fail(c,COHERENT_CAPACITY);
    const size_t n=c->count+1;
    const double dx=v->real-c->mean_real,dy=v->imag-c->mean_imag;
    const double real=c->mean_real+dx/n,imag=c->mean_imag+dy/n;
    const double m2=c->m2+dx*(v->real-real)+dy*(v->imag-imag);
    const double sum=c->sum_magnitudes+hypot(v->real,v->imag);
    if(!isfinite(real)||!isfinite(imag)||!isfinite(m2)||!isfinite(sum))return fail(c,COHERENT_NUMERIC);
    if(!c->count)c->min_rpm=c->max_rpm=v->rpm;
    else{c->min_rpm=fmin(c->min_rpm,v->rpm);c->max_rpm=fmax(c->max_rpm,v->rpm);}
    c->count=n;c->mean_real=real;c->mean_imag=imag;c->m2=fmax(0,m2);c->sum_magnitudes=sum;
    return COHERENT_ADDED;
}
void coherent_average_result(const coherent_average_t *c,coherent_result_t *r)
{
    if(!r)return;
    memset(r,0,sizeof(*r));r->diagnostic_only=true;
    if(!c || !c->initialized){r->status=COHERENT_ARGUMENT;return;}
    r->status=c->failure;r->epoch=c->config.epoch;
    if(c->failure!=COHERENT_ADDED)return;
    r->count=c->count;
    if(!c->count)return;
    r->real=c->mean_real;r->imag=c->mean_imag;r->peak=hypot(r->real,r->imag);
    r->scalar_peak_mean=c->sum_magnitudes/c->count;
    r->direction_coherence=c->sum_magnitudes>0?fmin(1,r->peak*c->count/c->sum_magnitudes):0;
    r->phase_defined=c->sum_magnitudes>0 && r->peak>1e-12*r->scalar_peak_mean;
    if(r->phase_defined)r->phase_deg=fmod(atan2(r->imag,r->real)*180/3.14159265358979323846+360,360);
    r->vector_scatter_rms=c->count>1?sqrt(c->m2/(c->count-1)):0;
    r->mathematical_valid=c->count>=c->config.min_turns;
    r->min_rpm=c->min_rpm;r->max_rpm=c->max_rpm;
}
