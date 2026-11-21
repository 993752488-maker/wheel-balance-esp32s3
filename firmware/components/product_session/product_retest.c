#include "product_retest.h"
#include <math.h>
#include <string.h>

_Static_assert(sizeof(product_snapshot_t)==752,"saved snapshot ABI changed");
_Static_assert(sizeof(product_retest_log_t)<=2304,"retest sidecar exceeds bounded storage budget");
const char *product_retest_condition_name(product_retest_condition_t condition)
{
    static const char *names[]={"not_recorded","first","unchanged","correction_changed","legacy_unknown"};
    return (unsigned)condition<5?names[condition]:"not_recorded";
}
static bool config_same(const product_config_t *a,const product_config_t *b)
{
    return a->mass_g==b->mass_g&&a->trial_radius_mm==b->trial_radius_mm&&a->trial_angle_deg==b->trial_angle_deg&&
        a->correction_radius_mm==b->correction_radius_mm&&a->axis==b->axis&&a->direction==b->direction&&
        !strncmp(a->vehicle,b->vehicle,sizeof(a->vehicle));
}
void product_retest_init(product_retest_log_t *log,const product_snapshot_t *s)
{
    if(!log)return;
    memset(log,0,sizeof(*log));if(!s)return;
    log->schema=1;log->session_id=s->id;log->snapshot_revision=s->revision;
    memcpy(log->boot_id,s->boot_id,sizeof(log->boot_id));log->config=s->config;
}
bool product_retest_valid(const product_retest_log_t *log,const product_snapshot_t *s)
{
    if(!log||!s||log->schema!=1||!s->id||log->session_id!=s->id||log->snapshot_revision!=s->revision||
        !memchr(log->boot_id,0,sizeof(log->boot_id))||strcmp(log->boot_id,s->boot_id)||
        log->count>PRODUCT_RETEST_HISTORY||log->total_count<log->count||log->dropped_count!=log->total_count-log->count)return false;
    if(log->count&&(!config_same(&log->config,&s->config)||log->original_capture_id!=s->stages[0].capture_id||
        log->trial_capture_id!=s->stages[1].capture_id))return false;
    for(unsigned i=0;i<log->count;++i){const product_retest_entry_t *e=&log->entries[i];
        if(!e->stage.capture_id||(e->stage.end_us&&e->stage.start_us>e->stage.end_us)||!e->correction_group||
            (unsigned)e->condition>RETEST_CONDITION_LEGACY||(unsigned)e->repeat.state>PRODUCT_REPEAT_VARIABLE||
            (unsigned)e->stage.reason>PRODUCT_CAPTURE_LIMIT)return false;
        if(e->comparison.available&&(!isfinite(e->comparison.improvement_percent)||
            !isfinite(e->comparison.original_peak)||!isfinite(e->comparison.retest_peak)))return false;
        if(e->repeat.state>=PRODUCT_REPEAT_OBSERVATION_CLOSE&&(!isfinite(e->repeat.amplitude_change_percent)||
            !isfinite(e->repeat.previous_peak)||!isfinite(e->repeat.current_peak)||!isfinite(e->repeat.phase_change_deg)))return false;
        if(i&&(e->stage.capture_id==log->entries[i-1].stage.capture_id||
            e->stage.start_us<log->entries[i-1].stage.end_us||e->correction_group<log->entries[i-1].correction_group))return false;
    }
    return true;
}
bool product_retest_can_start(const product_retest_log_t *log,product_retest_condition_t condition)
{
    if(!log||log->schema!=1)return false;
    return !log->count?condition==RETEST_CONDITION_FIRST:
        condition==RETEST_CONDITION_SAME||condition==RETEST_CONDITION_CHANGED;
}
bool product_retest_available(const product_snapshot_t *s)
{
    if(!s||s->historical||!s->stages[0].captured||!s->stages[1].captured||
       !s->stages[0].integrity_valid||!s->stages[1].integrity_valid||
       product_correction_status(&s->solution)!=PRODUCT_REASON_NONE)return false;
    if(s->stage==PRODUCT_TRIAL)return s->state==PRODUCT_COMPLETE;
    if(s->stage!=PRODUCT_RETEST)return false;
    if(s->state==PRODUCT_COMPLETE||s->state==PRODUCT_CANCELLED||s->state==PRODUCT_TIMEOUT)return true;
    if(s->state!=PRODUCT_NO_RESULT)return false;
    return s->reason==PRODUCT_NO_COMMON_SPEED||s->reason==PRODUCT_TOO_FEW_TURNS||
           s->reason==PRODUCT_BASELINE_ZERO||s->reason==PRODUCT_BUSY;
}
bool product_retest_record(product_retest_log_t *log,const product_snapshot_t *s,product_retest_condition_t condition,
    const product_repeat_observation_t *repeat)
{
    if(!product_retest_valid(log,s)||!s->stages[2].capture_id||s->stage!=PRODUCT_RETEST||
        !product_retest_can_start(log,condition)||log->total_count==UINT32_MAX)return false;
    const product_stage_summary_t *stage=&s->stages[2];
    if(stage->end_us&&stage->start_us>stage->end_us)return false;
    if(s->comparison.available&&(!isfinite(s->comparison.improvement_percent)||
        !isfinite(s->comparison.original_peak)||!isfinite(s->comparison.retest_peak)))return false;
    if(repeat&&condition==RETEST_CONDITION_SAME&&repeat->state>=PRODUCT_REPEAT_OBSERVATION_CLOSE&&
        (!isfinite(repeat->amplitude_change_percent)||!isfinite(repeat->previous_peak)||
         !isfinite(repeat->current_peak)||!isfinite(repeat->phase_change_deg)))return false;
    uint32_t group=log->count?log->entries[log->count-1].correction_group:1;
    if(log->count){const product_retest_entry_t *last=&log->entries[log->count-1];
        if(stage->capture_id==last->stage.capture_id||stage->start_us<last->stage.end_us)return false;
        if(condition==RETEST_CONDITION_CHANGED){if(group==UINT32_MAX)return false;++group;}}
    if(!log->count){log->config=s->config;log->original_capture_id=s->stages[0].capture_id;log->trial_capture_id=s->stages[1].capture_id;}
    if(log->count==PRODUCT_RETEST_HISTORY){memmove(log->entries,log->entries+1,(PRODUCT_RETEST_HISTORY-1)*sizeof(log->entries[0]));
        --log->count;++log->dropped_count;}
    product_retest_entry_t *e=&log->entries[log->count++];memset(e,0,sizeof(*e));
    e->ordinal=++log->total_count;e->correction_group=group;e->condition=condition;e->stage=*stage;e->comparison=s->comparison;
    if(repeat&&condition==RETEST_CONDITION_SAME)e->repeat=*repeat;
    if(condition==RETEST_CONDITION_CHANGED)log->current_group_variable=false;
    if(e->repeat.state==PRODUCT_REPEAT_VARIABLE)log->current_group_variable=true;
    return true;
}
void product_retest_import_latest(product_retest_log_t *log,const product_snapshot_t *s)
{
    product_retest_init(log,s);
    if(!s||!s->stages[2].capture_id)return;
    if(!product_retest_record(log,s,RETEST_CONDITION_FIRST,NULL))return;
    log->legacy_count_unknown=true;log->entries[0].ordinal=0;log->entries[0].legacy=true;
    log->entries[0].condition=RETEST_CONDITION_LEGACY;
}
