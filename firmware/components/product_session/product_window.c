#include "product_window.h"
#include <math.h>
#include <string.h>

_Static_assert(sizeof(product_window_t)==472,"window sidecar ABI changed");

static bool steady_neighborhood(const selection_turn_t *t,size_t end)
{
    if(end<4)return false;
    double mean=0,minimum=1e9,maximum=0,slope=0;unsigned up=0,down=0;
    for(unsigned k=0;k<5;++k){const coast_result_t *f=&t[end-4+k].fit;
        if(!f->valid||!isfinite(f->rpm)||f->rpm<=0)return false;
        mean+=f->rpm/5;minimum=fmin(minimum,f->rpm);maximum=fmax(maximum,f->rpm);
        slope+=((int)k-2)*f->rpm/10;
        if(k){const double d=f->rpm-t[end-5+k].fit.rpm;if(d>.01)++up;else if(d<-.01)++down;}
    }
    /* A bounded alternating ripple can be steady running. A sustained ramp
     * cannot acquire this exception just by having a small single-step rise. */
    return (up&&down)&&maximum-minimum<=.03*mean&&fabs(slope)<=.003*mean;
}
void product_window_configure_fit(selection_config_t *fit,product_capture_mode_t mode)
{
    fit->fit.acceleration_tolerance=.03;
    if(mode==CAPTURE_TIMED){
        fit->fit.acceleration_tolerance=PRODUCT_STEADY_SPEED_TOLERANCE;
        fit->fit.max_speed_change=PRODUCT_STEADY_SPEED_TOLERANCE;
    }
}
static void select_window(selection_turn_t *turns,size_t n,product_window_stage_t *out,
    product_capture_mode_t mode,double target_rpm)
{
    if(!out)return;
    memset(out,0,sizeof(*out));out->phase=WINDOW_SETTLING;
    if(!turns||n>COAST_SELECTION_MAX_REFS)return;
    const bool timed=mode==CAPTURE_TIMED;
    const unsigned max_rpm=product_capture_max_rpm(mode);
    const double tolerance=target_rpm*PRODUCT_STEADY_SPEED_TOLERANCE;
    unsigned quiet[3]={0},steady=0,coast=0;
    uint64_t quiet_from[3]={0},previous_reference=0;
    for(size_t i=0;i<n;++i){selection_turn_t *t=&turns[i];const coast_result_t *f=&t->fit;
        ++out->total_turns;
        const bool steady_like=!timed&&steady_neighborhood(turns,i);
        const bool accelerating=f->status==COAST_ERR_ACCELERATING||
            (!timed&&f->valid&&f->rpm_previous>0&&f->rpm>f->rpm_previous*1.005&&!steady_like);
        if(accelerating)++out->accelerating_turns;
        const bool on_target=!timed||(target_rpm>=PRODUCT_MIN_RPM&&target_rpm<max_rpm&&
            f->rpm_previous>=PRODUCT_MIN_RPM&&f->rpm_previous<max_rpm&&
            fabs(f->rpm-target_rpm)<=tolerance&&fabs(f->rpm_previous-target_rpm)<=tolerance&&
            fabs(f->rpm-f->rpm_previous)<=PRODUCT_STEADY_SPEED_TOLERANCE*fmax(f->rpm,f->rpm_previous));
        const bool model=f->valid&&f->rpm>=PRODUCT_MIN_RPM&&f->rpm<max_rpm&&
            isfinite(f->rpm_previous)&&f->rpm_previous>0&&f->end_us>f->start_us&&on_target;
        const bool connected=!previous_reference||f->ref_last==previous_reference+1;
        previous_reference=f->ref_last;
        if(model&&!accelerating){if(timed||steady_like||fabs(f->rpm-f->rpm_previous)<=.005*fmax(f->rpm,f->rpm_previous))++steady;else ++coast;}
        for(unsigned axis=0;axis<3;++axis){
            t->reasons[axis]&=~(SELECT_SETTLING|SELECT_WINDOW_LIMIT);
            const uint32_t structural=t->reasons[axis]&~SELECT_WEAK_PHASE;
            const bool bad=!model||accelerating||!connected||structural;
            if(bad){quiet[axis]=0;quiet_from[axis]=0;
                if(structural&(SELECT_IMPULSE|SELECT_RESIDUAL_DECAY|SELECT_DC_CHANGE))++out->transient[axis];
            }else{
                if(!quiet[axis])quiet_from[axis]=f->start_us;
                ++quiet[axis];
            }
            const bool ready=!bad&&quiet[axis]>=3&&f->end_us>=quiet_from[axis]&&
                f->end_us-quiet_from[axis]>=1500000;
            if(ready){++out->usable[axis];if(!out->first_usable_us[axis])out->first_usable_us[axis]=f->start_us;
                out->last_usable_us[axis]=f->end_us;out->phase=WINDOW_COLLECTING;
                out->collecting_mask|=1U<<axis;
            }else{t->reasons[axis]|=SELECT_SETTLING;++out->skipped[axis];out->collecting_mask&=~(1U<<axis);}
            t->candidate[axis]=t->reasons[axis]==0;
        }
    }
    /* Preserve coverage of the WHOLE coast. Divide the bounded working set
     * across occupied 20-RPM bands, then sample each band across time. Long
     * low-speed tails must not evict the earlier higher-speed information.
     * No amplitude, H/Uc or desired improvement influences this selection. */
    for(unsigned axis=0;axis<3;++axis){
        unsigned counts[PRODUCT_SPEED_BAND_COUNT]={0},quota[PRODUCT_SPEED_BAND_COUNT]={0};
        unsigned seen[PRODUCT_SPEED_BAND_COUNT]={0},kept[PRODUCT_SPEED_BAND_COUNT]={0};
        for(size_t i=0;i<n;++i)if(!(turns[i].reasons[axis]&~SELECT_WEAK_PHASE))
            ++counts[((unsigned)turns[i].fit.rpm-PRODUCT_MIN_RPM)/PRODUCT_SPEED_WIDTH];
        unsigned remaining=out->usable[axis]<PRODUCT_WINDOW_KEEP?out->usable[axis]:PRODUCT_WINDOW_KEEP;
        while(remaining)for(unsigned b=PRODUCT_SPEED_BAND_COUNT;b--&&remaining;)if(quota[b]<counts[b]){++quota[b];--remaining;}
        out->first_usable_us[axis]=out->last_usable_us[axis]=0;
        for(size_t i=0;i<n;++i){selection_turn_t *t=&turns[i];
            if(t->reasons[axis]&~SELECT_WEAK_PHASE)continue;
            const unsigned b=((unsigned)t->fit.rpm-PRODUCT_MIN_RPM)/PRODUCT_SPEED_WIDTH,ordinal=seen[b]++;
            const unsigned next=quota[b]>1?kept[b]*(counts[b]-1)/(quota[b]-1):counts[b]/2;
            if(!quota[b]||ordinal!=next){--out->usable[axis];++out->skipped[axis];
                t->reasons[axis]|=SELECT_WINDOW_LIMIT;t->candidate[axis]=false;continue;}
            ++kept[b];
            if(!out->first_usable_us[axis])out->first_usable_us[axis]=t->fit.start_us;
            out->last_usable_us[axis]=t->fit.end_us;
        }
    }
    out->phase=out->collecting_mask?WINDOW_COLLECTING:WINDOW_SETTLING;
    out->mode=steady&&coast?WINDOW_MIXED:steady?WINDOW_STEADY:coast?WINDOW_COAST:WINDOW_UNDETERMINED;
}
void product_window_select(selection_turn_t *turns,size_t n,product_window_stage_t *out)
{
    select_window(turns,n,out,CAPTURE_COAST,0);
}
void product_window_select_timed(selection_turn_t *turns,size_t n,double target_rpm,
    product_window_stage_t *out)
{
    select_window(turns,n,out,CAPTURE_TIMED,target_rpm);
}
void product_window_bind(product_window_t *p,const product_snapshot_t *s)
{if(p&&s){p->schema=2;p->session_id=s->id;p->snapshot_revision=s->revision;p->simulated=s->simulated;memcpy(p->boot_id,s->boot_id,sizeof(p->boot_id));}}
void product_window_reset(product_window_t *p,const product_snapshot_t *s,product_capture_config_t capture)
{if(p){memset(p,0,sizeof(*p));p->capture=capture;product_window_bind(p,s);}}
void product_window_clear(product_window_t *p,unsigned stage)
{if(p&&stage<3){memset(&p->stages[stage],0,sizeof(p->stages[stage]));++p->generation;}}
static bool window_matches(const product_window_t *p,const product_snapshot_t *s)
{
    return p&&s&&p->session_id==s->id&&p->snapshot_revision==s->revision&&
        p->simulated==s->simulated&&!memcmp(p->boot_id,s->boot_id,sizeof(p->boot_id));
}
bool product_window_has_capture_settings(const product_window_t *p,const product_snapshot_t *s)
{
    return window_matches(p,s)&&p->schema==2&&product_capture_config_valid(&p->capture);
}
bool product_window_valid(const product_window_t *p,const product_snapshot_t *s,bool stored)
{
    if(!window_matches(p,s)||(p->schema!=1&&p->schema!=2))return false;
    if(p->schema==2&&!product_capture_config_valid(&p->capture))return false;
    for(unsigned i=0;i<3;++i){const product_window_stage_t *v=&p->stages[i];
        if(v->phase>WINDOW_COMPLETE||v->mode>WINDOW_MIXED||v->collecting_mask>7||v->common_pairs>PRODUCT_MAX_TURNS||v->total_turns>COAST_SELECTION_MAX_REFS)return false;
        if(p->schema==2&&v->end_reason>CAPTURE_END_TIMED)return false;
        if(v->capture_id){
            const bool live=!stored&&!s->historical&&s->stage==i&&(s->state==PRODUCT_RECORDING||s->state==PRODUCT_PROCESSING);
            if(!v->epoch||(!live&&(v->capture_id!=s->stages[i].capture_id||v->epoch!=s->stages[i].epoch||
               v->start_us!=s->stages[i].start_us||v->end_us!=s->stages[i].end_us)))return false;
        }
        for(unsigned a=0;a<3;++a){if(v->usable[a]>PRODUCT_WINDOW_KEEP||v->usable[a]+v->skipped[a]!=v->total_turns||
            v->transient[a]>v->skipped[a]||(v->usable[a]&&(!v->capture_id||v->first_usable_us[a]<v->start_us||
            v->last_usable_us[a]>v->end_us||v->last_usable_us[a]<=v->first_usable_us[a])))return false;}
    }
    return true;
}
