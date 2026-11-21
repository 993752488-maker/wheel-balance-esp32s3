#include "raw_pose_core.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static void increment(uint32_t *v){if(*v<UINT32_MAX)++*v;}
void raw_pose_init(raw_pose_core_t *c,raw_pose_t *storage,size_t capacity)
{
    if(!c)return;
    memset(c,0,sizeof(*c));c->storage=storage;
    c->status.enabled=storage&&capacity;c->status.capacity=c->status.enabled?capacity:0;
}
void raw_pose_begin(raw_pose_core_t *c,uint32_t id)
{
    if(!c)return;
    const size_t capacity=c->status.capacity;
    c->status=(raw_pose_status_t){.enabled=c->storage&&capacity,.capacity=capacity,.capture_id=id};
}
bool raw_pose_add(raw_pose_core_t *c,const raw_status_t *raw,uint32_t id,const raw_pose_t *v)
{
    if(!c||!raw||!c->status.enabled||c->status.frozen)return false;
    raw_pose_status_t *s=&c->status;
    if(!id||id!=s->capture_id||id!=raw->id){increment(&s->stale_events);return false;}
    if(raw->state!=RAW_RECORDING&&raw->state!=RAW_CLOSING)return false;
    if(!v){s->faults|=RAW_POSE_METADATA;return false;}
    uint64_t end=raw->deadline_us;
    if(raw->state==RAW_CLOSING&&raw->stop_requested_us<end)end=raw->stop_requested_us;
    if(!v->read_begin_us||v->read_end_us<v->read_begin_us||v->ingest_us<v->read_end_us){
        s->faults|=RAW_POSE_TIME;return false;
    }
    /* A register transaction straddling capture start/stop is not a member of
     * either window. Count it explicitly rather than borrowing adjacent data. */
    if(v->read_begin_us<raw->start_us||v->read_end_us>end){increment(&s->boundary_excluded);return false;}
    if(s->count==s->capacity){s->faults|=RAW_POSE_CAPACITY;return false;}
    if(!v->epoch||!v->fifo_sequence)s->faults|=RAW_POSE_METADATA;
    if(!v->frame_consistent)s->faults|=RAW_POSE_FRAME;
    for(unsigned i=0;i<3;++i)
        if(v->accel[i]>=32760||v->accel[i]<=-32760||v->gyro[i]>=32760||v->gyro[i]<=-32760)
            s->faults|=RAW_POSE_SATURATION;
    if(s->count){
        const raw_pose_t *last=c->storage+s->count-1;
        if(v->epoch!=last->epoch)s->faults|=RAW_POSE_EPOCH;
        if(v->read_begin_us<=last->read_begin_us||v->read_begin_us<last->read_end_us||
           v->ingest_us<last->ingest_us||v->fifo_sequence<last->fifo_sequence)s->faults|=RAW_POSE_TIME;
    }else s->first_epoch=v->epoch;
    c->storage[s->count++]=*v;s->last_epoch=v->epoch;return true;
}
void raw_pose_finish(raw_pose_core_t *c,const raw_status_t *raw,uint32_t drops)
{
    if(!c||!raw||raw->state!=RAW_READY||c->status.capture_id!=raw->id||c->status.frozen)return;
    c->status.queue_drops=drops;c->status.frozen=true;
    if(c->status.count&&(c->status.first_epoch!=raw->first_epoch||c->status.last_epoch!=raw->last_epoch))
        c->status.faults|=RAW_POSE_EPOCH;
}
size_t raw_pose_csv(char *out,size_t cap,uint32_t id,size_t index,const raw_pose_t *v)
{
    if(!out||!cap)return 0;
    out[0]=0;if(!id||!v)return 0;
    int n=snprintf(out,cap,"%" PRIu32 ",%u,%" PRIu32 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
        ",%" PRIu64 ",%d,%d,%d,%d,%d,%d,%d,%d\n",id,(unsigned)index,v->epoch,v->ingest_us,
        v->read_begin_us,v->read_end_us,v->fifo_sequence,v->accel[0],v->accel[1],v->accel[2],
        v->temperature,v->gyro[0],v->gyro[1],v->gyro[2],v->frame_consistent);
    if(n<0||(size_t)n>=cap){out[0]=0;return 0;}return (size_t)n;
}
