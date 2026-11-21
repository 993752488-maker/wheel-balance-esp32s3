#include "raw_capture_core.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

const char *raw_state_name(raw_state_t s)
{
    switch(s){case RAW_IDLE:return "IDLE";case RAW_RECORDING:return "RECORDING";
    case RAW_CLOSING:return "CLOSING";case RAW_READY:return "READY";default:return "DISABLED";}
}
const char *raw_context_name(raw_context_t s)
{
    switch(s){case RAW_CONTEXT_USER:return "user_capture";case RAW_CONTEXT_BOOT_CHECK:return "boot_raw_selfcheck";
    case RAW_CONTEXT_USB:return "usb_diagnostic_capture";default:return "unknown";}
}
const char *raw_end_name(raw_end_t s)
{
    switch(s){case RAW_END_NONE:return "none";case RAW_END_DURATION:return "duration";
    case RAW_END_MANUAL:return "manual";case RAW_END_CAPACITY:return "capacity";default:return "unknown";}
}

void raw_core_init(raw_core_t *c,raw_sample_t *samples,size_t ns,raw_reference_t *refs,size_t nr)
{
    if(!c)return;
    memset(c,0,sizeof(*c));
    c->sample_storage=samples;c->reference_storage=refs;
    c->status.enabled=samples && refs && ns && nr;
    c->status.sample_capacity=ns;c->status.reference_capacity=nr;
}

bool raw_core_begin(raw_core_t *c,uint32_t id,uint64_t now,uint64_t duration,raw_context_t context)
{
    if(!c || !c->status.enabled || !id || id<=c->status.id || !now ||
       duration<250000 || duration>120000000 || duration>UINT64_MAX-now ||
       context>RAW_CONTEXT_USB || context<RAW_CONTEXT_USER ||
       c->status.state==RAW_RECORDING || c->status.state==RAW_CLOSING)return false;
    const size_t ns=c->status.sample_capacity,nr=c->status.reference_capacity;
    c->status=(raw_status_t){.enabled=true,.state=RAW_RECORDING,.context=context,.id=id,
        .start_us=now,.deadline_us=now+duration,.sample_capacity=ns,.reference_capacity=nr};
    return true;
}

static bool accepts(raw_core_t *c,uint32_t id)
{
    if(!c || (c->status.state!=RAW_RECORDING && c->status.state!=RAW_CLOSING))return false;
    if(id!=c->status.id){if(c->status.stale_events<UINT32_MAX)++c->status.stale_events;return false;}
    return true;
}

void raw_core_close(raw_core_t *c,uint64_t now,raw_end_t reason)
{
    if(!c || c->status.state!=RAW_RECORDING)return;
    c->status.state=RAW_CLOSING;c->status.end_reason=reason;c->status.stop_requested_us=now;
    if(now<c->status.start_us)c->status.faults|=RAW_FAULT_TIME;
}

bool raw_core_sample(raw_core_t *c,uint32_t id,const raw_sample_t *s)
{
    if(!accepts(c,id))return false;
    if(!s){c->status.faults|=RAW_FAULT_METADATA;return false;}
    raw_status_t *st=&c->status;
    if(st->samples==st->sample_capacity){st->faults|=RAW_FAULT_CAPACITY;raw_core_close(c,s->ingest_us,RAW_END_CAPACITY);return false;}
    if(!s->sequence || !s->epoch || !s->pending_frames || s->batch_index>=s->pending_frames)
        st->faults|=RAW_FAULT_METADATA;
    if(s->readout_us<=0 || s->count_begin_us<=0 || s->count_end_us<s->count_begin_us ||
       s->readout_us<s->count_end_us || s->ingest_us<st->start_us ||
       (s->readout_us>0 && s->ingest_us<(uint64_t)s->readout_us))st->faults|=RAW_FAULT_TIME;
    if(st->samples){
        const raw_sample_t *last=&c->sample_storage[st->samples-1];
        if(last->sequence==UINT64_MAX || s->sequence!=last->sequence+1)st->faults|=RAW_FAULT_SAMPLE_SEQUENCE;
        if(s->epoch!=last->epoch)st->faults|=RAW_FAULT_EPOCH;
        if(s->ingest_us<last->ingest_us || s->readout_us<last->readout_us)st->faults|=RAW_FAULT_TIME;
    }else{st->first_sequence=s->sequence;st->first_epoch=s->epoch;}
    for(unsigned a=0;a<3;++a)if(s->raw[a]>=32760 || s->raw[a]<=-32760)st->faults|=RAW_FAULT_SATURATION;
    c->sample_storage[st->samples++]=*s;st->last_sequence=s->sequence;st->last_epoch=s->epoch;
    return true;
}

bool raw_core_reference(raw_core_t *c,uint32_t id,const raw_reference_t *r)
{
    if(!accepts(c,id))return false;
    if(!r){c->status.faults|=RAW_FAULT_METADATA;return false;}
    raw_status_t *st=&c->status;
    if(st->references==st->reference_capacity){st->faults|=RAW_FAULT_CAPACITY;raw_core_close(c,r->ingest_us,RAW_END_CAPACITY);return false;}
    if(!r->sequence || !r->reference_us)st->faults|=RAW_FAULT_METADATA;
    if(r->ingest_us<st->start_us || r->reference_us>r->ingest_us)st->faults|=RAW_FAULT_TIME;
    if(st->references){
        const raw_reference_t *last=&c->reference_storage[st->references-1];
        if(last->sequence==UINT64_MAX || r->sequence!=last->sequence+1)st->faults|=RAW_FAULT_REFERENCE;
        if(r->reference_us<=last->reference_us || r->ingest_us<last->ingest_us)st->faults|=RAW_FAULT_TIME;
    }else st->first_reference=r->sequence;
    c->reference_storage[st->references++]=*r;st->last_reference=r->sequence;
    return true;
}

void raw_core_finish(raw_core_t *c,uint64_t now,uint32_t faults,uint32_t drops)
{
    if(!c || c->status.state!=RAW_CLOSING)return;
    raw_status_t *st=&c->status;
    st->faults|=faults;st->queue_drops=drops;
    if(drops)st->faults|=RAW_FAULT_QUEUE;
    if(!st->samples)st->faults|=RAW_FAULT_EMPTY;
    if(now<st->start_us || now<st->stop_requested_us ||
       (st->samples && now<c->sample_storage[st->samples-1].ingest_us) ||
       (st->references && now<c->reference_storage[st->references-1].ingest_us))st->faults|=RAW_FAULT_TIME;
    st->end_us=now;st->state=RAW_READY;st->integrity_ok=st->faults==0;
    st->measurement_qualified=false;
}

static size_t formatted(char *out,size_t cap,int n)
{
    if(n<0 || (size_t)n>=cap){if(out && cap)out[0]=0;return 0;}
    return (size_t)n;
}
size_t raw_csv_sample(char *out,size_t cap,uint32_t id,const raw_sample_t *s)
{
    if(!out || !cap)return 0;
    out[0]=0;if(!s || !id)return 0;
    return formatted(out,cap,snprintf(out,cap,"SAMPLE,%" PRIu32 ",%" PRIu64 ",%" PRIu64 ",%" PRIu32
        ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%u,%u,%d,%d,%d,,,\n",id,s->ingest_us,s->sequence,s->epoch,
        s->readout_us,s->count_begin_us,s->count_end_us,(unsigned)s->pending_frames,(unsigned)s->batch_index,
        s->raw[0],s->raw[1],s->raw[2]));
}
size_t raw_csv_reference(char *out,size_t cap,uint32_t id,const raw_reference_t *r)
{
    if(!out || !cap)return 0;
    out[0]=0;if(!r || !id)return 0;
    return formatted(out,cap,snprintf(out,cap,"REFERENCE,%" PRIu32 ",%" PRIu64 ",%" PRIu64
        ",,,,,,,,,,%" PRIu64 ",%d,%d\n",id,r->ingest_us,r->sequence,r->reference_us,r->has_period,r->timestamp_resynced));
}

bool raw_gate_open(raw_gate_t *g,uint32_t id)
{
    if(!g || !id || g->active_id || g->inflight || id<=g->generation)return false;
    *g=(raw_gate_t){.generation=id,.active_id=id};return true;
}
uint32_t raw_gate_reserve(raw_gate_t *g)
{
    if(!g || !g->active_id || g->inflight==UINT32_MAX)return 0;
    ++g->inflight;return g->active_id;
}
void raw_gate_release(raw_gate_t *g,uint32_t token,bool queued)
{
    if(!g || token!=g->generation || !g->inflight)return;
    --g->inflight;
    if(!queued){g->faults|=RAW_FAULT_QUEUE;if(g->queue_drops<UINT32_MAX)++g->queue_drops;}
}
void raw_gate_close(raw_gate_t *g){if(g)g->active_id=0;}
void raw_gate_fault(raw_gate_t *g,uint32_t faults){if(g && g->active_id)g->faults|=faults;}
bool raw_gate_quiet(const raw_gate_t *g){return g && !g->active_id && !g->inflight;}
