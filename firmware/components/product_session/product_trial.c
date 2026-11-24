#include "product_trial.h"
#include <math.h>
#include <string.h>

_Static_assert(sizeof(product_trial_guide_t)==984,"trial sidecar ABI changed");
_Static_assert(offsetof(product_trial_guide_t,initial_angle_deg)==952,"legacy trial sidecar moved");
_Static_assert(offsetof(product_trial_attempt_t,stage)==16,"trial stage ABI changed");

static bool mass(double x){return isfinite(x)&&x>=.1&&x<=1000;}
static bool angle(double x){return isfinite(x)&&x>=0&&x<360;}
static bool same(double x,double y){return fabs(x-y)<=1e-8*fmax(1,fmax(fabs(x),fabs(y)));}
const char *product_trial_source_name(product_trial_source_t source)
{static const char *v[]={"starter_policy","manual","response_projection"};return (unsigned)source<3?v[source]:"unknown";}
const char *product_trial_next_name(product_trial_next_t reason)
{static const char *v[]={"READY","BASELINE_REQUIRED","BUSY","FINISHED","HARD_FAULT","ATTEMPT_LIMIT","NO_SUPPORTED_MASS_PREDICTION","RESPONSE_PROJECTION","LIGHTER_COMPARISON",
    "OPPOSITE_COMPARISON","STEP_UP_COMPARISON","MATCH_BASELINE_SPEED","LONGER_OR_CLEANER_RECORD"};
 return (unsigned)reason<sizeof(v)/sizeof(v[0])?v[reason]:"HARD_FAULT";}
bool product_trial_init(product_trial_guide_t *g,product_snapshot_t *s,double piece,bool guided)
{
    if(!g||!s||!s->id||!memchr(s->boot_id,0,sizeof(s->boot_id))||!mass(piece)||!product_config_valid(&s->config)||
       (guided&&piece>PRODUCT_GUIDED_TRIAL_MAX_G))return false;
    const double initial=guided?(piece*2<=PRODUCT_GUIDED_TRIAL_MAX_G?piece*2:piece):s->config.mass_g;
    if(!mass(initial))return false;
    memset(g,0,sizeof(*g));g->schema=4;g->session_id=s->id;g->snapshot_revision=s->revision;memcpy(g->boot_id,s->boot_id,sizeof(g->boot_id));
    g->piece_mass_g=piece;g->initial_mass_g=initial;g->source=guided?TRIAL_PLAN_STARTER:TRIAL_PLAN_MANUAL;
    g->automatic_axis=true;g->axis_resolved=s->stages[0].captured;
    g->initial_angle_deg=s->config.trial_angle_deg;s->config.mass_g=initial;return true;
}
bool product_trial_plan_original(product_trial_guide_t *g,product_snapshot_t *s,double rpm)
{
    if(!product_trial_valid(g,s)||g->count||s->historical||s->simulated||g->source!=TRIAL_PLAN_STARTER||
       !s->stages[0].captured||!s->stages[0].integrity_valid||!isfinite(rpm)||rpm<PRODUCT_MIN_RPM||rpm>=PRODUCT_TIMED_MAX_RPM_EXCLUSIVE)return false;
    /* Use the same nominal centrifugal excitation for a small pilot where
     * possible. The actual structural response is learned from that pilot. */
    const double pilot_g=fmin(10,5*(160/s->config.trial_radius_mm)*pow(300/rpm,2));
    const double max_pieces=fmax(1,fmin(8,floor(10/g->piece_mass_g+1e-9)));
    const double pieces=fmax(1,fmin(max_pieces,ceil(pilot_g/g->piece_mass_g-1e-9)));
    g->initial_mass_g=pieces*g->piece_mass_g;
    s->config.mass_g=g->initial_mass_g;
    return true;
}
bool product_trial_valid(const product_trial_guide_t *g,const product_snapshot_t *s)
{
    if(!g||!s||g->schema<2||g->schema>4||!g->session_id||g->session_id!=s->id||g->snapshot_revision!=s->revision||
        !memchr(g->boot_id,0,sizeof(g->boot_id))||strcmp(g->boot_id,s->boot_id)||
        !mass(g->piece_mass_g)||!mass(g->initial_mass_g)||g->count>PRODUCT_TRIAL_ATTEMPTS||(unsigned)g->source>TRIAL_PLAN_RESPONSE||
        (g->schema>=4&&!angle(g->initial_angle_deg)))return false;
    for(unsigned i=0;i<g->count;++i){const product_trial_attempt_t *a=&g->attempts[i];
        if(!mass(a->mass_g)||(unsigned)a->source>TRIAL_PLAN_RESPONSE||!a->stage.capture_id||
            a->stage.start_us>a->stage.end_us||(unsigned)a->stage.reason>PRODUCT_CONFLICT||(g->schema>=3&&a->axis>2)||
            (g->schema>=4&&!angle(g->attempt_angle_deg[i])))return false;
        if(i&&(a->stage.capture_id==g->attempts[i-1].stage.capture_id||a->stage.start_us<g->attempts[i-1].stage.end_us))return false;}
    return true;
}
bool product_trial_restore(product_trial_guide_t *g,const product_snapshot_t *s,size_t stored_bytes)
{
    if(!g)return false;
    if(stored_bytes==920&&g->schema==1){
        _Static_assert(offsetof(product_trial_guide_t,automatic_axis)==920,"v1 sidecar layout changed");
        memset((unsigned char *)g+920,0,sizeof(*g)-920);g->schema=2;g->axis_resolved=s&&s->stages[0].captured;
    }else if(stored_bytes==952&&(g->schema==2||g->schema==3)){
        memset((unsigned char *)g+952,0,sizeof(*g)-952);
    }else if(stored_bytes!=sizeof(*g))return false;
    else if(g->schema<4)memset((unsigned char *)g+952,0,sizeof(*g)-952);
    return product_trial_valid(g,s);
}
static bool hard(product_reason_t r)
{return r==PRODUCT_SENSOR||r==PRODUCT_DATA_FAULT||r==PRODUCT_REFERENCE||r==PRODUCT_CONTEXT||r==PRODUCT_NUMERIC;}
static double attempt_angle(const product_trial_guide_t *g,const product_snapshot_t *s,unsigned i)
{return g->schema>=4?g->attempt_angle_deg[i]:s->config.trial_angle_deg;}
static bool already_tested(const product_trial_guide_t *g,const product_snapshot_t *s,double candidate,double degrees)
{
    for(unsigned i=0;i<g->count;++i){
        const product_trial_attempt_t *a=&g->attempts[i];
        if(a->stage.captured&&a->stage.integrity_valid&&a->solution.matched_turns>=3&&same(a->mass_g,candidate)&&
           same(attempt_angle(g,s,i),degrees))return true;
    }
    return false;
}
static void suggest(product_trial_advice_t *out,double grams,double degrees,product_trial_next_t reason)
{
    out->suggested=true;out->mass_g=grams;out->angle_deg=degrees;out->reason=reason;
}
static bool increase_allowed(const product_trial_guide_t *g,const product_snapshot_t *s,
                             const product_trial_attempt_t *last,double candidate,double degrees)
{
    return mass(candidate)&&candidate>last->mass_g&&candidate<=2*last->mass_g+1e-8&&
        candidate<=fmin(PRODUCT_GUIDED_TRIAL_MAX_G,fmin(4*g->initial_mass_g,8*g->piece_mass_g))+1e-8&&
        !already_tested(g,s,candidate,degrees);
}
void product_trial_advice(const product_trial_guide_t *g,const product_snapshot_t *s,product_trial_advice_t *out)
{
    if(!out)return;
    memset(out,0,sizeof(*out));out->reason=TRIAL_NEXT_HARD_FAULT;
    if(!product_trial_valid(g,s)||s->simulated||s->historical)return;
    if(s->state==PRODUCT_ARMED||s->state==PRODUCT_RECORDING||s->state==PRODUCT_PROCESSING){out->reason=TRIAL_NEXT_BUSY;return;}
    if(!s->stages[0].captured||!s->stages[0].integrity_valid){out->reason=TRIAL_NEXT_NO_BASELINE;return;}
    if(hard(s->reason)){out->reason=TRIAL_NEXT_HARD_FAULT;return;}
    if(s->solution.calculation_valid&&s->reason!=PRODUCT_CONFLICT){out->reason=TRIAL_NEXT_FINISHED;return;}
    if(g->count==PRODUCT_TRIAL_ATTEMPTS){out->reason=TRIAL_NEXT_LIMIT;return;}
    out->editable=true;out->retry_available=g->count>0;out->reason=TRIAL_NEXT_READY;
    if(!g->count)return;
    const product_trial_attempt_t *last=&g->attempts[g->count-1];const product_solution_t *r=&last->solution;
    out->reason=TRIAL_NEXT_NO_PREDICTION;
    if(s->reason==PRODUCT_NO_COMMON_SPEED||s->reason==PRODUCT_WAIT_TIMEOUT){out->reason=TRIAL_NEXT_MATCH_SPEED;return;}
    if(s->reason==PRODUCT_INCONSISTENT||s->reason==PRODUCT_LOW_DIRECTION_COHERENCE||s->reason==PRODUCT_TOO_FEW_TURNS||
       s->reason==PRODUCT_CAPTURE_LIMIT){out->reason=TRIAL_NEXT_CLEAN_RECORD;return;}
    if(s->reason!=last->stage.reason)return;
    if(!last->stage.captured||!last->stage.integrity_valid||last->stage.epoch!=s->stages[0].epoch||
        r->matched_turns<3||!isfinite(r->delta_peak)||r->delta_peak<0||!isfinite(r->delta_coherence)||
        r->delta_coherence<0||r->delta_coherence>1||!isfinite(r->mean_noise_scale)||r->mean_noise_scale<0)return;
    const double degrees=attempt_angle(g,s,g->count-1);
    if(s->reason==PRODUCT_CONFLICT&&last->stage.reason==PRODUCT_CONFLICT&&r->calculation_valid){
        /* A lower load is a second response level, not a correction. Keep
         * both the projected signal and the difference from the old load
         * above the observed variation. Offer at most one guided reduction. */
        bool reduced=false;
        for(unsigned i=1;i<g->count;++i){
            if(g->attempts[i].source==TRIAL_PLAN_RESPONSE&&g->attempts[i-1].stage.reason==PRODUCT_CONFLICT&&
               g->attempts[i].mass_g<g->attempts[i-1].mass_g)reduced=true;
        }
        const double lighter=round(.5*last->mass_g/g->piece_mass_g)*g->piece_mass_g;
        const double target=fmax(1.01e-5,4*r->mean_noise_scale);
        if(!reduced&&r->delta_coherence>=.65&&mass(lighter)&&lighter<last->mass_g&&
           !already_tested(g,s,lighter,degrees)&&r->delta_peak*fmin(lighter,last->mass_g-lighter)/last->mass_g>=target){
            suggest(out,lighter,degrees,TRIAL_NEXT_LIGHTER_COMPARISON);return;
        }
        const double opposite=fmod(degrees+180,360);
        if(g->schema>=4&&r->delta_coherence>=.65&&r->delta_peak>1.5*r->mean_noise_scale&&
           !already_tested(g,s,last->mass_g,opposite)){
            suggest(out,last->mass_g,opposite,TRIAL_NEXT_OPPOSITE);return;
        }
        if(g->schema>=4&&g->source!=TRIAL_PLAN_MANUAL)out->retry_available=out->editable=false;
        return;
    }
    /* Project an increase only from a coherent, weak difference. The
     * measured variation describes this recording's noise, not a load limit. */
    if(last->stage.reason!=PRODUCT_WEAK_RESPONSE)return;
    const double target=fmax(1.01e-5,2*r->mean_noise_scale);
    if(r->delta_coherence>=.65&&r->delta_peak>0){
        const double projected=last->mass_g*fmax(1.25,target/r->delta_peak);
        const double candidate=ceil((projected/g->piece_mass_g)-1e-10)*g->piece_mass_g;
        if(increase_allowed(g,s,last,candidate,degrees)){
            suggest(out,candidate,degrees,TRIAL_NEXT_RESPONSE);return;
        }
    }else if(g->schema>=4&&g->count==1&&g->source==TRIAL_PLAN_STARTER){
        const double candidate=last->mass_g+g->piece_mass_g;
        if(increase_allowed(g,s,last,candidate,degrees)){
            suggest(out,candidate,degrees,TRIAL_NEXT_STEP_UP);return;
        }
    }
    if(g->schema>=4)out->reason=TRIAL_NEXT_CLEAN_RECORD;
}
void product_trial_apply_advice(product_trial_guide_t *g,product_snapshot_t *s)
{
    if(!g||g->schema<4||g->source==TRIAL_PLAN_MANUAL)return;
    product_trial_advice_t advice;
    product_trial_advice(g,s,&advice);
    if(advice.suggested){
        s->config.mass_g=advice.mass_g;
        s->config.trial_angle_deg=advice.angle_deg;
        g->source=TRIAL_PLAN_RESPONSE;
    }
}
bool product_trial_contradicts_previous(const product_trial_guide_t *g,const product_snapshot_t *s)
{
    if(!product_trial_valid(g,s)||g->source!=TRIAL_PLAN_RESPONSE||!s->solution.calculation_valid||
       s->solution.reason!=PRODUCT_REASON_NONE)return false;
    const product_solution_t *current=&s->solution;
    const double unbalance=s->config.mass_g*s->config.trial_radius_mm;
    for(unsigned i=0;i<g->count;++i){
        const product_trial_attempt_t *a=&g->attempts[i];
        const product_solution_t *previous=&a->solution;
        if(!a->stage.integrity_valid||!previous->calculation_valid||a->stage.epoch!=s->stages[1].epoch||
           a->axis!=s->config.axis||!same(previous->lower_rpm,current->lower_rpm)||
           (same(a->mass_g,s->config.mass_g)&&same(attempt_angle(g,s,i),s->config.trial_angle_deg)))continue;
        const balance_complex_t h=current->calculation.influence,old=previous->calculation.influence;
        const double change=hypot(h.real-old.real,h.imag-old.imag);
        const double typical=fmax(hypot(h.real,h.imag),hypot(old.real,old.imag));
        const double noise=current->mean_noise_scale/unbalance+previous->mean_noise_scale/(a->mass_g*s->config.trial_radius_mm);
        if(change>fmax(.5*typical,4*noise))return true;
    }
    return false;
}
bool product_trial_set_plan(product_trial_guide_t *g,product_snapshot_t *s,double value,product_trial_source_t source)
{
    product_trial_advice_t advice;product_trial_advice(g,s,&advice);
    if(!advice.editable||!mass(value)||(unsigned)source>TRIAL_PLAN_RESPONSE)return false;
    if(source==TRIAL_PLAN_STARTER&&(g->count||!same(value,g->initial_mass_g)))return false;
    if(source==TRIAL_PLAN_RESPONSE&&(!advice.suggested||!same(value,advice.mass_g)))return false;
    s->config.mass_g=value;
    if(source==TRIAL_PLAN_RESPONSE)s->config.trial_angle_deg=advice.angle_deg;
    g->source=source;return true;
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
    g->attempt_angle_deg[g->count]=s->config.trial_angle_deg;
    product_trial_attempt_t *a=&g->attempts[g->count++];a->mass_g=s->config.mass_g;a->source=g->source;
    a->axis=s->config.axis;a->stage=*stage;a->solution=s->solution;return true;
}
