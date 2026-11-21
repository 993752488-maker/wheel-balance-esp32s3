#include "product_trial.h"
#include <math.h>
#include <string.h>

_Static_assert(sizeof(product_trial_guide_t)==952,"trial sidecar ABI changed");
_Static_assert(offsetof(product_trial_attempt_t,stage)==16,"trial stage ABI changed");

static bool mass(double x){return isfinite(x)&&x>=.1&&x<=1000;}
static bool same(double x,double y){return fabs(x-y)<=1e-8*fmax(1,fmax(fabs(x),fabs(y)));}
const char *product_trial_source_name(product_trial_source_t source)
{static const char *v[]={"starter_policy","manual","response_projection"};return (unsigned)source<3?v[source]:"unknown";}
const char *product_trial_next_name(product_trial_next_t reason)
{static const char *v[]={"READY","BASELINE_REQUIRED","BUSY","FINISHED","HARD_FAULT","ATTEMPT_LIMIT","NO_SUPPORTED_MASS_PREDICTION","RESPONSE_PROJECTION"};
 return (unsigned)reason<8?v[reason]:"HARD_FAULT";}
bool product_trial_init(product_trial_guide_t *g,product_snapshot_t *s,double piece,bool guided)
{
    if(!g||!s||!s->id||!memchr(s->boot_id,0,sizeof(s->boot_id))||!mass(piece)||!product_config_valid(&s->config))return false;
    const double initial=guided?2*piece:s->config.mass_g;
    if(!mass(initial))return false;
    memset(g,0,sizeof(*g));g->schema=3;g->session_id=s->id;g->snapshot_revision=s->revision;memcpy(g->boot_id,s->boot_id,sizeof(g->boot_id));
    g->piece_mass_g=piece;g->initial_mass_g=initial;g->source=guided?TRIAL_PLAN_STARTER:TRIAL_PLAN_MANUAL;
    g->automatic_axis=true;g->axis_resolved=s->stages[0].captured;
    s->config.mass_g=initial;return true;
}
bool product_trial_valid(const product_trial_guide_t *g,const product_snapshot_t *s)
{
    if(!g||!s||(g->schema!=2&&g->schema!=3)||!g->session_id||g->session_id!=s->id||g->snapshot_revision!=s->revision||
        !memchr(g->boot_id,0,sizeof(g->boot_id))||strcmp(g->boot_id,s->boot_id)||
        !mass(g->piece_mass_g)||!mass(g->initial_mass_g)||g->count>PRODUCT_TRIAL_ATTEMPTS||(unsigned)g->source>TRIAL_PLAN_RESPONSE)return false;
    for(unsigned i=0;i<g->count;++i){const product_trial_attempt_t *a=&g->attempts[i];
        if(!mass(a->mass_g)||(unsigned)a->source>TRIAL_PLAN_RESPONSE||!a->stage.capture_id||
            a->stage.start_us>a->stage.end_us||(unsigned)a->stage.reason>PRODUCT_CONFLICT||(g->schema>=3&&a->axis>2))return false;
        if(i&&(a->stage.capture_id==g->attempts[i-1].stage.capture_id||a->stage.start_us<g->attempts[i-1].stage.end_us))return false;}
    return true;
}
bool product_trial_restore(product_trial_guide_t *g,const product_snapshot_t *s,size_t stored_bytes)
{
    if(!g)return false;
    if(stored_bytes==920&&g->schema==1){
        _Static_assert(offsetof(product_trial_guide_t,automatic_axis)==920,"v1 sidecar layout changed");
        memset((unsigned char *)g+920,0,sizeof(*g)-920);g->schema=2;g->axis_resolved=s&&s->stages[0].captured;
    }else if(stored_bytes!=sizeof(*g))return false;
    return product_trial_valid(g,s);
}
static bool hard(product_reason_t r)
{return r==PRODUCT_SENSOR||r==PRODUCT_DATA_FAULT||r==PRODUCT_REFERENCE||r==PRODUCT_CONTEXT||r==PRODUCT_NUMERIC;}
void product_trial_advice(const product_trial_guide_t *g,const product_snapshot_t *s,product_trial_advice_t *out)
{
    if(!out)return;
    memset(out,0,sizeof(*out));out->reason=TRIAL_NEXT_HARD_FAULT;
    if(!product_trial_valid(g,s)||s->simulated||s->historical)return;
    if(s->state==PRODUCT_ARMED||s->state==PRODUCT_RECORDING||s->state==PRODUCT_PROCESSING){out->reason=TRIAL_NEXT_BUSY;return;}
    if(!s->stages[0].captured||!s->stages[0].integrity_valid){out->reason=TRIAL_NEXT_NO_BASELINE;return;}
    if(s->solution.calculation_valid){out->reason=TRIAL_NEXT_FINISHED;return;}
    if(hard(s->reason)){out->reason=TRIAL_NEXT_HARD_FAULT;return;}
    if(g->count==PRODUCT_TRIAL_ATTEMPTS){out->reason=TRIAL_NEXT_LIMIT;return;}
    out->editable=true;out->retry_available=g->count>0;out->reason=TRIAL_NEXT_READY;
    if(!g->count)return;
    const product_trial_attempt_t *last=&g->attempts[g->count-1];const product_solution_t *r=&last->solution;
    /* A finite vector alone does not justify escalating mass. Project only a
     * coherent, weak difference from intact, speed-matched physical captures.
     * Scale is descriptive scatter, NOT calibrated uncertainty. */
    out->reason=TRIAL_NEXT_NO_PREDICTION;
    if(last->stage.reason!=PRODUCT_WEAK_RESPONSE||!last->stage.captured||!last->stage.integrity_valid||
        r->matched_turns<3||!isfinite(r->delta_peak)||r->delta_peak<=0||!isfinite(r->delta_coherence)||
        r->delta_coherence<.65||r->delta_coherence>1||!isfinite(r->mean_noise_scale)||r->mean_noise_scale<0)return;
    const double target=fmax(1.01e-5,2*r->mean_noise_scale);
    const double projected=last->mass_g*fmax(1.25,target/r->delta_peak);
    const double candidate=ceil((projected/g->piece_mass_g)-1e-10)*g->piece_mass_g;
    if(!mass(candidate)||candidate<=last->mass_g||candidate>2*last->mass_g+1e-8||candidate>4*g->initial_mass_g+1e-8)return;
    out->suggested=true;out->mass_g=candidate;out->reason=TRIAL_NEXT_RESPONSE;
}
bool product_trial_set_plan(product_trial_guide_t *g,product_snapshot_t *s,double value,product_trial_source_t source)
{
    product_trial_advice_t advice;product_trial_advice(g,s,&advice);
    if(!advice.editable||!mass(value)||(unsigned)source>TRIAL_PLAN_RESPONSE)return false;
    if(source==TRIAL_PLAN_STARTER&&(g->count||!same(value,2*g->piece_mass_g)))return false;
    if(source==TRIAL_PLAN_RESPONSE&&(!advice.suggested||!same(value,advice.mass_g)))return false;
    s->config.mass_g=value;g->source=source;return true;
}
bool product_trial_can_start(const product_trial_guide_t *g,const product_snapshot_t *s,double confirmed)
{
    product_trial_advice_t advice;product_trial_advice(g,s,&advice);
    return advice.editable&&(!g->automatic_axis||g->axis_resolved)&&mass(confirmed)&&same(confirmed,s->config.mass_g);
}
bool product_trial_record(product_trial_guide_t *g,const product_snapshot_t *s)
{
    if(!product_trial_valid(g,s)||s->simulated||s->stage!=PRODUCT_TRIAL||!s->stages[1].capture_id||g->count>=PRODUCT_TRIAL_ATTEMPTS)return false;
    const product_stage_summary_t *stage=&s->stages[1];
    for(unsigned i=0;i<g->count;++i)if(g->attempts[i].stage.capture_id==stage->capture_id)return false;
    product_trial_attempt_t *a=&g->attempts[g->count++];a->mass_g=s->config.mass_g;a->source=g->source;
    a->axis=s->config.axis;a->stage=*stage;a->solution=s->solution;return true;
}
