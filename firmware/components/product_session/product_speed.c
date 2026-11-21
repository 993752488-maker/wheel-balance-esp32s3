#include "product_speed.h"
#include <math.h>
#include <string.h>

void product_speed_bind(product_speed_t *v,const product_snapshot_t *s)
{
    if(!v||!s)return;
    v->schema=1;v->session_id=s->id;v->snapshot_revision=s->revision;
    v->axis=s->config.axis;v->simulated=s->simulated;memcpy(v->boot_id,s->boot_id,sizeof(v->boot_id));
}
void product_speed_reset(product_speed_t *v,const product_snapshot_t *s)
{if(v){memset(v,0,sizeof(*v));product_speed_bind(v,s);}}
void product_speed_clear(product_speed_t *v,unsigned stage)
{if(v&&stage<3){memset(&v->stages[stage],0,sizeof(v->stages[stage]));++v->generation;}}
bool product_speed_set(product_speed_t *v,unsigned stage,const product_run_t *r,bool preview,uint64_t updated)
{
    if(!v||!r||stage>2)return false;
    product_speed_clear(v,stage);product_speed_stage_t *o=&v->stages[stage];
    o->mode=SPEED_INVALID;o->capture_id=r->summary.capture_id;o->epoch=r->summary.epoch;
    o->start_us=r->summary.start_us;o->end_us=r->summary.end_us;o->updated_us=updated;
    o->rejected_turns=r->summary.rejected_turns;
    if(!r->summary.captured||!r->summary.integrity_valid||!o->capture_id||!o->epoch||r->count>PRODUCT_MAX_TURNS)return false;
    double energy[PRODUCT_SPEED_BINS]={0};
    for(size_t i=0;i<r->count;++i){const product_turn_t *t=&r->turns[i];
        if(!isfinite(t->rpm)||t->rpm<=0||!isfinite(t->real)||!isfinite(t->imag))goto invalid;
        const double amplitude=hypot(t->real,t->imag);
        if(!isfinite(amplitude)||amplitude>1e6)goto invalid;
        if(t->rpm<PRODUCT_SPEED_WIDTH||t->rpm>=PRODUCT_SPEED_WIDTH*(PRODUCT_SPEED_BINS+1)){++o->outside_turns;continue;}
        const unsigned index=(unsigned)(t->rpm/PRODUCT_SPEED_WIDTH)-1;
        product_speed_bin_t *b=&o->bins[index];
        if(!b->count){b->minimum=(float)amplitude;b->maximum=(float)amplitude;}
        else{if(amplitude<b->minimum)b->minimum=(float)amplitude;if(amplitude>b->maximum)b->maximum=(float)amplitude;}
        energy[index]+=amplitude*amplitude;++b->count;++o->used_turns;
        if(t->reference>o->last_reference)o->last_reference=t->reference;
    }
    for(unsigned i=0;i<PRODUCT_SPEED_BINS;++i)if(o->bins[i].count)
        o->bins[i].rms=(float)sqrt(energy[i]/o->bins[i].count);
    o->mode=preview?SPEED_PREVIEW:SPEED_FROZEN;return true;
invalid:
    memset(o->bins,0,sizeof(o->bins));o->used_turns=0;o->mode=SPEED_INVALID;return false;
}
bool product_speed_valid(const product_speed_t *v,const product_snapshot_t *s,bool stored)
{
    if(!v||!s||v->schema!=1||v->session_id!=s->id||v->snapshot_revision!=s->revision||
       v->axis!=s->config.axis||v->simulated!=s->simulated||memcmp(v->boot_id,s->boot_id,sizeof(v->boot_id)))return false;
    for(unsigned stage=0;stage<3;++stage){const product_speed_stage_t *a=&v->stages[stage];
        if(a->mode>SPEED_INVALID)return false;
        if(a->mode==SPEED_PREVIEW&&(stored||s->historical||
           (s->state!=PRODUCT_RECORDING&&s->state!=PRODUCT_PROCESSING)||s->stage!=stage))return false;
        if(a->mode==SPEED_FROZEN&&(!s->stages[stage].captured||!s->stages[stage].integrity_valid||
           a->capture_id!=s->stages[stage].capture_id||a->epoch!=s->stages[stage].epoch||
           a->start_us!=s->stages[stage].start_us||a->end_us!=s->stages[stage].end_us))return false;
        if(stage&&v->stages[0].mode==SPEED_FROZEN&&(a->mode==SPEED_FROZEN||a->mode==SPEED_PREVIEW)&&a->epoch!=v->stages[0].epoch)return false;
        unsigned count=0;
        for(unsigned i=0;i<PRODUCT_SPEED_BINS;++i){const product_speed_bin_t *b=&a->bins[i];
            if(b->count>PRODUCT_MAX_TURNS||!isfinite(b->rms)||!isfinite(b->minimum)||!isfinite(b->maximum)||
               b->minimum<0||b->maximum<b->minimum||b->rms<0||b->rms>1e6||
               b->rms<b->minimum-1e-5f||b->rms>b->maximum+1e-5f)return false;
            if(b->count&&(a->mode!=SPEED_FROZEN&&a->mode!=SPEED_PREVIEW))return false;
            count+=b->count;
        }
        if(count!=a->used_turns||count>PRODUCT_MAX_TURNS)return false;
        if((a->mode==SPEED_PREVIEW||a->mode==SPEED_FROZEN)&&(!a->capture_id||!a->epoch))return false;
    }
    return true;
}
