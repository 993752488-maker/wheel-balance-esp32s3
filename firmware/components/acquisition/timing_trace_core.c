#include "timing_trace_core.h"
#include <limits.h>
#include <string.h>

void timing_trace_init(timing_trace_core_t *c,uint64_t *storage,size_t capacity)
{
    if(!c)return;
    memset(c,0,sizeof(*c));c->storage=storage;c->capacity=capacity;
    c->status.state=storage && capacity ? TIMING_IDLE : TIMING_DISABLED;
}
bool timing_trace_begin(timing_trace_core_t *c,uint32_t epoch,uint64_t now,uint32_t ms)
{
    if(!c || !epoch || !now || ms<250 || ms>3000 || c->leases ||
       c->status.state==TIMING_DISABLED || c->status.state==TIMING_RECORDING ||
       c->status.id==UINT32_MAX || now>UINT64_MAX-(uint64_t)ms*1000)return false;
    c->status=(timing_trace_status_t){.state=TIMING_RECORDING,.id=c->status.id+1,
        .epoch=epoch,.start_us=now,.deadline_us=now+(uint64_t)ms*1000};
    return true;
}
void timing_trace_tick(timing_trace_core_t *c,uint64_t now)
{
    if(!c || c->status.state!=TIMING_RECORDING)return;
    if(now<c->status.start_us) {c->status.faults|=TIMING_FAULT_CLOCK;c->status.state=TIMING_READY;}
    else if(now>=c->status.deadline_us)c->status.state=TIMING_READY;
}
void timing_trace_event(timing_trace_core_t *c,uint32_t epoch,uint64_t t)
{
    if(!c || c->status.state!=TIMING_RECORDING || t<c->status.start_us)return;
    if(epoch!=c->status.epoch){c->status.faults|=TIMING_FAULT_EPOCH;c->status.state=TIMING_READY;return;}
    if(c->status.events && t<=c->storage[c->status.events-1]) {
        c->status.faults|=TIMING_FAULT_CLOCK;c->status.state=TIMING_READY;return;
    }
    timing_trace_tick(c,t);if(c->status.state!=TIMING_RECORDING)return;
    if(c->status.events==c->capacity){c->status.faults|=TIMING_FAULT_OVERFLOW;c->status.state=TIMING_READY;return;}
    c->storage[c->status.events++]=t;
}
bool timing_trace_lease(timing_trace_core_t *c,uint32_t id)
{
    if(!c || !id || id!=c->status.id || c->status.state!=TIMING_READY || c->leases==UINT_MAX)return false;
    ++c->leases;return true;
}
void timing_trace_release(timing_trace_core_t *c,uint32_t id)
{ if(c && id==c->status.id && c->leases)--c->leases; }
size_t timing_trace_read(const timing_trace_core_t *c,uint32_t id,size_t offset,uint64_t *out,size_t capacity)
{
    if(!c || !out || !c->leases || !id || id!=c->status.id || offset>=c->status.events)return 0;
    size_t n=c->status.events-offset;if(n>capacity)n=capacity;
    memcpy(out,c->storage+offset,n*sizeof(*out));return n;
}
