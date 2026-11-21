#include "raw_capture.h"
#include "sdkconfig.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if CONFIG_RAW_CAPTURE_ENABLED
enum { SAMPLE_CAPACITY=64000, REFERENCE_CAPACITY=1024, EVENT_CAPACITY=256 };
typedef struct { uint32_t id; bool reference; union { raw_sample_t sample; raw_reference_t ref; }; } event_t;
typedef struct { uint32_t id; raw_pose_t value; } pose_event_t;
static raw_core_t s_core;
static raw_pose_core_t s_pose;
static raw_gate_t s_gate;
static portMUX_TYPE s_gate_lock=portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_lock;
static QueueHandle_t s_events;
static QueueHandle_t s_pose_events;
static uint32_t s_pose_drops; /* protected by s_gate_lock */
static unsigned s_leases;

static void close_gate(void)
{
    portENTER_CRITICAL(&s_gate_lock);raw_gate_close(&s_gate);portEXIT_CRITICAL(&s_gate_lock);
}
static void worker(void *arg)
{
    (void)arg;
    for (;;) {
        event_t e;
        const bool got=xQueueReceive(s_events,&e,pdMS_TO_TICKS(5))==pdTRUE;
        xSemaphoreTake(s_lock,portMAX_DELAY);
        if(got) {
            if(e.reference) (void)raw_core_reference(&s_core,e.id,&e.ref);
            else (void)raw_core_sample(&s_core,e.id,&e.sample);
        }
        pose_event_t pose;
        for(unsigned i=0;s_pose_events&&i<4&&xQueueReceive(s_pose_events,&pose,0)==pdTRUE;++i)
            (void)raw_pose_add(&s_pose,&s_core.status,pose.id,&pose.value);
        const uint64_t now=(uint64_t)esp_timer_get_time();
        if(s_core.status.state==RAW_RECORDING && now>=s_core.status.deadline_us)
            raw_core_close(&s_core,now,RAW_END_DURATION);
        if(s_core.status.state==RAW_CLOSING) {
            close_gate();
            portENTER_CRITICAL(&s_gate_lock);
            const bool quiet=raw_gate_quiet(&s_gate);
            const uint32_t faults=s_gate.faults,drops=s_gate.queue_drops,pose_drops=s_pose_drops;
            portEXIT_CRITICAL(&s_gate_lock);
            /* A reserved producer may be preempted before it queues. Seal only
             * after every reservation is released AND the queue is drained. */
            if(quiet&&!uxQueueMessagesWaiting(s_events)&&(!s_pose_events||!uxQueueMessagesWaiting(s_pose_events))){
                raw_core_finish(&s_core,now,faults,drops);
                raw_pose_finish(&s_pose,&s_core.status,pose_drops);
            }
        }
        xSemaphoreGive(s_lock);
    }
}
esp_err_t raw_capture_init(void)
{
    if(s_lock)return ESP_ERR_INVALID_STATE;
    raw_sample_t *samples=heap_caps_malloc(sizeof(*samples)*SAMPLE_CAPACITY,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    raw_reference_t *refs=heap_caps_malloc(sizeof(*refs)*REFERENCE_CAPACITY,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    s_lock=xSemaphoreCreateMutex();s_events=xQueueCreate(EVENT_CAPACITY,sizeof(event_t));
    if(!samples || !refs || !s_lock || !s_events)goto fail;
    raw_core_init(&s_core,samples,SAMPLE_CAPACITY,refs,REFERENCE_CAPACITY);
#if CONFIG_MPU_POSE_DIAG_ENABLED
    raw_pose_t *poses=heap_caps_malloc(sizeof(*poses)*4096,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    s_pose_events=xQueueCreate(32,sizeof(pose_event_t));
    if(!poses||!s_pose_events){heap_caps_free(poses);poses=NULL;if(s_pose_events)vQueueDelete(s_pose_events);s_pose_events=NULL;}
    raw_pose_init(&s_pose,poses,poses?4096:0);
#else
    raw_pose_init(&s_pose,NULL,0);
#endif
    if(xTaskCreatePinnedToCore(worker,"raw_capture",4096,NULL,6,NULL,1)!=pdPASS)goto fail;
    return ESP_OK;
fail:
    heap_caps_free(samples);heap_caps_free(refs);
    heap_caps_free(s_pose.storage);memset(&s_pose,0,sizeof(s_pose));
    if(s_pose_events)vQueueDelete(s_pose_events);
    s_pose_events=NULL;
    if(s_events)vQueueDelete(s_events);
    if(s_lock)vSemaphoreDelete(s_lock);
    s_events=NULL;s_lock=NULL;memset(&s_core,0,sizeof(s_core));return ESP_ERR_NO_MEM;
}
static void publish(event_t *e)
{
    if(!s_events)return;
    portENTER_CRITICAL(&s_gate_lock);e->id=raw_gate_reserve(&s_gate);portEXIT_CRITICAL(&s_gate_lock);
    if(!e->id)return;
    if(e->reference)e->ref.ingest_us=(uint64_t)esp_timer_get_time();
    else e->sample.ingest_us=(uint64_t)esp_timer_get_time();
    const bool queued=xQueueSend(s_events,e,0)==pdTRUE;
    portENTER_CRITICAL(&s_gate_lock);raw_gate_release(&s_gate,e->id,queued);portEXIT_CRITICAL(&s_gate_lock);
}
void raw_capture_publish_sample(const raw_sample_t *s){if(s){event_t e={.sample=*s};publish(&e);}}
void raw_capture_publish_reference(const raw_reference_t *r){if(r){event_t e={.reference=true,.ref=*r};publish(&e);}}
void raw_capture_publish_pose(const raw_pose_t *value)
{
    if(!value||!s_pose_events)return;
    pose_event_t e={.value=*value};
    portENTER_CRITICAL(&s_gate_lock);e.id=raw_gate_reserve(&s_gate);portEXIT_CRITICAL(&s_gate_lock);
    if(!e.id)return;
    e.value.ingest_us=(uint64_t)esp_timer_get_time();
    const bool queued=xQueueSend(s_pose_events,&e,0)==pdTRUE;
    portENTER_CRITICAL(&s_gate_lock);
    if(!queued&&s_pose_drops<UINT32_MAX)++s_pose_drops;
    /* Optional diagnostics never fill the acceleration queue or turn their own
     * loss into a false acceleration fault. The shared fence still prevents
     * sealing/replacing a capture with an outstanding pose producer. */
    raw_gate_release(&s_gate,e.id,true);
    portEXIT_CRITICAL(&s_gate_lock);
}
void raw_capture_signal_fault(uint32_t faults)
{
    portENTER_CRITICAL(&s_gate_lock);raw_gate_fault(&s_gate,faults);portEXIT_CRITICAL(&s_gate_lock);
}
void raw_capture_get_status(raw_status_t *out)
{
    if(!out)return;
    if(!s_lock){memset(out,0,sizeof(*out));return;}
    xSemaphoreTake(s_lock,portMAX_DELAY);*out=s_core.status;
    portENTER_CRITICAL(&s_gate_lock);
    out->faults|=s_gate.faults;out->queue_drops=s_gate.queue_drops;
    portEXIT_CRITICAL(&s_gate_lock);xSemaphoreGive(s_lock);
}
bool raw_capture_get_pose_status(uint32_t id,raw_pose_status_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));if(!s_lock)return false;
    xSemaphoreTake(s_lock,portMAX_DELAY);
    const bool ok=id&&id==s_core.status.id&&id==s_pose.status.capture_id;
    if(ok){*out=s_pose.status;portENTER_CRITICAL(&s_gate_lock);out->queue_drops=s_pose_drops;portEXIT_CRITICAL(&s_gate_lock);}
    xSemaphoreGive(s_lock);return ok;
}
esp_err_t raw_capture_begin(uint32_t ms,raw_context_t context,bool only_unused)
{
    if(!s_lock)return ESP_ERR_NOT_SUPPORTED;
    xSemaphoreTake(s_lock,portMAX_DELAY);
    bool ok=!s_leases && (!only_unused || !s_core.status.id) && s_core.status.id<UINT32_MAX;
    if(ok)ok=raw_core_begin(&s_core,s_core.status.id+1,(uint64_t)esp_timer_get_time(),(uint64_t)ms*1000,context);
    if(ok) {
        /* Previous READY required a closed, quiet gate and an empty queue. */
        raw_pose_begin(&s_pose,s_core.status.id);
        portENTER_CRITICAL(&s_gate_lock);s_pose_drops=0;ok=raw_gate_open(&s_gate,s_core.status.id);portEXIT_CRITICAL(&s_gate_lock);
    }
    xSemaphoreGive(s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
}
esp_err_t raw_capture_stop(uint32_t id)
{
    if(!s_lock)return ESP_ERR_NOT_SUPPORTED;
    xSemaphoreTake(s_lock,portMAX_DELAY);
    const bool ok=id && id==s_core.status.id && s_core.status.state==RAW_RECORDING;
    if(ok){close_gate();raw_core_close(&s_core,(uint64_t)esp_timer_get_time(),RAW_END_MANUAL);}
    xSemaphoreGive(s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
}
esp_err_t raw_capture_export_begin(uint32_t id,raw_status_t *out)
{
    if(!s_lock || !out)return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock,portMAX_DELAY);
    const bool ok=id && id==s_core.status.id && s_core.status.state==RAW_READY && s_leases<UINT32_MAX;
    if(ok){++s_leases;*out=s_core.status;}
    xSemaphoreGive(s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
}
esp_err_t raw_capture_maintenance_begin(void)
{
    if(!s_lock)return ESP_ERR_NOT_SUPPORTED;
    xSemaphoreTake(s_lock,portMAX_DELAY);
    const bool ok=!s_leases&&(s_core.status.state==RAW_IDLE||s_core.status.state==RAW_READY);
    if(ok)++s_leases;
    xSemaphoreGive(s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
}
esp_err_t raw_capture_preview_begin(uint32_t id,raw_status_t *out)
{
    if(!s_lock||!out||xSemaphoreTake(s_lock,0)!=pdTRUE)return ESP_ERR_INVALID_STATE;
    const bool ok=id&&id==s_core.status.id&&s_core.status.state==RAW_RECORDING&&s_leases<UINT32_MAX;
    if(ok){++s_leases;*out=s_core.status;
        portENTER_CRITICAL(&s_gate_lock);out->faults|=s_gate.faults;out->queue_drops=s_gate.queue_drops;portEXIT_CRITICAL(&s_gate_lock);}
    xSemaphoreGive(s_lock);return ok?ESP_OK:ESP_ERR_INVALID_STATE;
}
void raw_capture_maintenance_end(void)
{
    if(!s_lock)return;
    xSemaphoreTake(s_lock,portMAX_DELAY);if(s_leases)--s_leases;xSemaphoreGive(s_lock);
}
void raw_capture_export_end(uint32_t id)
{
    if(!s_lock)return;
    xSemaphoreTake(s_lock,portMAX_DELAY);
    if(id==s_core.status.id && s_leases)--s_leases;
    xSemaphoreGive(s_lock);
}
size_t raw_capture_read_samples(uint32_t id,size_t offset,raw_sample_t *out,size_t cap)
{
    if(!s_lock || !out)return 0;
    xSemaphoreTake(s_lock,portMAX_DELAY);size_t n=0;
    if(s_leases && id==s_core.status.id && offset<s_core.status.samples) {
        n=s_core.status.samples-offset;if(n>cap)n=cap;if(n>32)n=32;
        memcpy(out,s_core.sample_storage+offset,n*sizeof(*out));
    }
    xSemaphoreGive(s_lock);return n;
}
size_t raw_capture_read_references(uint32_t id,size_t offset,raw_reference_t *out,size_t cap)
{
    if(!s_lock || !out)return 0;
    xSemaphoreTake(s_lock,portMAX_DELAY);size_t n=0;
    if(s_leases && id==s_core.status.id && offset<s_core.status.references) {
        n=s_core.status.references-offset;if(n>cap)n=cap;if(n>32)n=32;
        memcpy(out,s_core.reference_storage+offset,n*sizeof(*out));
    }
    xSemaphoreGive(s_lock);return n;
}
size_t raw_capture_read_poses(uint32_t id,size_t offset,raw_pose_t *out,size_t cap)
{
    if(!s_lock||!out)return 0;
    xSemaphoreTake(s_lock,portMAX_DELAY);size_t n=0;
    if(s_leases&&id==s_core.status.id&&id==s_pose.status.capture_id&&s_pose.status.frozen&&offset<s_pose.status.count){
        n=s_pose.status.count-offset;if(n>cap)n=cap;if(n>32)n=32;
        memcpy(out,s_pose.storage+offset,n*sizeof(*out));
    }
    xSemaphoreGive(s_lock);return n;
}
#else
esp_err_t raw_capture_init(void){return ESP_ERR_NOT_SUPPORTED;}
esp_err_t raw_capture_preview_begin(uint32_t id,raw_status_t *s){(void)id;(void)s;return ESP_ERR_NOT_SUPPORTED;}
void raw_capture_publish_sample(const raw_sample_t *s){(void)s;}
void raw_capture_publish_reference(const raw_reference_t *r){(void)r;}
void raw_capture_publish_pose(const raw_pose_t *p){(void)p;}
void raw_capture_signal_fault(uint32_t f){(void)f;}
void raw_capture_get_status(raw_status_t *s){if(s)memset(s,0,sizeof(*s));}
bool raw_capture_get_pose_status(uint32_t id,raw_pose_status_t *s){(void)id;if(s)memset(s,0,sizeof(*s));return false;}
esp_err_t raw_capture_begin(uint32_t m,raw_context_t c,bool u){(void)m;(void)c;(void)u;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t raw_capture_stop(uint32_t i){(void)i;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t raw_capture_export_begin(uint32_t i,raw_status_t *s){(void)i;(void)s;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t raw_capture_maintenance_begin(void){return ESP_ERR_NOT_SUPPORTED;}
void raw_capture_maintenance_end(void){}
void raw_capture_export_end(uint32_t i){(void)i;}
size_t raw_capture_read_samples(uint32_t i,size_t o,raw_sample_t *s,size_t n){(void)i;(void)o;(void)s;(void)n;return 0;}
size_t raw_capture_read_references(uint32_t i,size_t o,raw_reference_t *s,size_t n){(void)i;(void)o;(void)s;(void)n;return 0;}
size_t raw_capture_read_poses(uint32_t i,size_t o,raw_pose_t *s,size_t n){(void)i;(void)o;(void)s;(void)n;return 0;}
#endif
