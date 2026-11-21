#include "capture_export.h"
#include "capture_export_http.h"
#include "sdkconfig.h"
#include <string.h>

#if CONFIG_DIAGNOSTIC_WEB_ENABLED
#include "raw_capture.h"
#include "serial_console.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <stdio.h>
#include <inttypes.h>

typedef struct {uint32_t id;bool pose;httpd_req_t *request;} job_t;
typedef struct {uint32_t id,index,crc,bytes;size_t used;bool pose;char line[512];} usb_sink_t;
static QueueHandle_t queue;
static TaskHandle_t task;
static char boot_id[17],firmware[65];
static atomic_bool busy;
static atomic_uint completed,failed,stack_min;

static bool usb_record(const char *line,size_t n)
{
    const int64_t deadline=esp_timer_get_time()+2000000;
    do{if(serial_console_try_record(line,n))return true;vTaskDelay(1);}while(esp_timer_get_time()<deadline);
    return false;
}
static bool usb_sink(const char *data,size_t n,void *arg)
{
    usb_sink_t *s=arg;char record[640];
    for(size_t i=0;i<n;++i){const unsigned char ch=(unsigned char)data[i];s->crc^=ch;
        for(unsigned b=0;b<8;++b)s->crc=(s->crc>>1)^((s->crc&1)?UINT32_C(0xedb88320):0);
        ++s->bytes;
        if(ch=='\n'){
            s->line[s->used]=0;
            const int len=snprintf(record,sizeof(record),"%s_CSV id=%" PRIu32 " line=%" PRIu32 " data=%s\n",s->pose?"POSE":"RAW",s->id,s->index++,s->line);
            s->used=0;if(len<0||(size_t)len>=sizeof(record)||!usb_record(record,(size_t)len))return false;
            if(!(s->index%16))vTaskDelay(2);
        }else{if(s->used>=sizeof(s->line)-1)return false;s->line[s->used++]=(char)ch;}
    }
    return true;
}
static bool http_sink(const char *s,size_t n,void *arg){return httpd_resp_send_chunk(arg,s,(ssize_t)n)==ESP_OK;}
static esp_err_t usb_export(const job_t *job)
{
    raw_status_t raw;raw_capture_get_status(&raw);raw_pose_status_t pose={0};
    if(job->id!=raw.id||(job->pose&&!raw_capture_get_pose_status(job->id,&pose)))return ESP_ERR_INVALID_STATE;
    char record[256];
    int n=job->pose?snprintf(record,sizeof(record),"POSE_EXPORT_BEGIN id=%" PRIu32 " boot_id=%s observations=%u transport=direct_frozen_writer\n",job->id,boot_id,(unsigned)pose.count):
        snprintf(record,sizeof(record),"RAW_EXPORT_BEGIN id=%" PRIu32 " boot_id=%s samples=%u references=%u transport=direct_frozen_writer\n",job->id,boot_id,(unsigned)raw.samples,(unsigned)raw.references);
    if(n<0||(size_t)n>=sizeof(record)||!usb_record(record,(size_t)n))return ESP_ERR_TIMEOUT;
    usb_sink_t sink={.id=job->id,.pose=job->pose,.crc=UINT32_MAX};
    esp_err_t err=job->pose?raw_capture_write_pose_csv(job->id,boot_id,firmware,usb_sink,&sink):
        raw_capture_write_csv(job->id,boot_id,firmware,usb_sink,&sink);
    if(err==ESP_OK&&(sink.used||!sink.bytes))err=ESP_ERR_INVALID_RESPONSE;
    n=snprintf(record,sizeof(record),"%s_EXPORT_END id=%" PRIu32 " lines=%" PRIu32 " bytes=%" PRIu32 " crc32=%08" PRIx32 " result=%s\n",
        job->pose?"POSE":"RAW",job->id,sink.index,sink.bytes,sink.crc^UINT32_MAX,esp_err_to_name(err));
    if(n<0||(size_t)n>=sizeof(record)||!usb_record(record,(size_t)n))return ESP_ERR_TIMEOUT;
    return err;
}
static esp_err_t http_export(const job_t *job)
{
    httpd_req_t *r=job->request;char id_text[12];snprintf(id_text,sizeof(id_text),"%" PRIu32,job->id);
    esp_err_t err=httpd_resp_set_type(r,"text/csv; charset=utf-8");
    if(err==ESP_OK)err=httpd_resp_set_hdr(r,"Cache-Control","no-store, max-age=0");
    if(err==ESP_OK)err=httpd_resp_set_hdr(r,"X-Content-Type-Options","nosniff");
    if(err==ESP_OK)err=httpd_resp_set_hdr(r,"Content-Disposition",job->pose?"attachment; filename=wheelbalance_pose.csv":"attachment; filename=wheelbalance_raw.csv");
    if(err==ESP_OK)err=httpd_resp_set_hdr(r,"X-Capture-Id",id_text);
    if(err==ESP_OK)err=httpd_resp_set_hdr(r,"X-Boot-Id",boot_id);
    if(err==ESP_OK)err=job->pose?raw_capture_write_pose_csv(job->id,boot_id,firmware,http_sink,r):
        raw_capture_write_csv(job->id,boot_id,firmware,http_sink,r);
    if(err==ESP_OK)err=httpd_resp_send_chunk(r,NULL,0);
    return err;
}
static void execute(const job_t *job)
{
    esp_err_t err=job->request?http_export(job):usb_export(job);
    raw_capture_export_end(job->id);
    if(job->request){
        httpd_handle_t server=job->request->handle;const int fd=httpd_req_to_sockfd(job->request);
        const esp_err_t done=httpd_req_async_handler_complete(job->request);
        if(done!=ESP_OK)err=done;
        if(err!=ESP_OK)(void)httpd_sess_trigger_close(server,fd);
    }
    atomic_fetch_add(&completed,1);if(err!=ESP_OK)atomic_fetch_add(&failed,1);
    atomic_store(&stack_min,(unsigned)(uxTaskGetStackHighWaterMark(NULL)*sizeof(StackType_t)));
    atomic_store(&busy,false);
    serial_console_printf("CAPTURE_EXPORT_DONE id=%" PRIu32 " kind=%s destination=%s result=%s internal_free=%u internal_largest=%u stack_free_bytes=%u\n",
        job->id,job->pose?"pose":"raw",job->request?"http_async":"usb_direct",esp_err_to_name(err),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)atomic_load(&stack_min));
}
static void worker(void *arg)
{
    (void)arg;job_t job;
    for(;;)if(xQueueReceive(queue,&job,portMAX_DELAY)==pdTRUE)execute(&job);
}
static esp_err_t reserve(uint32_t id)
{
    if(!queue||!task)return ESP_ERR_NOT_SUPPORTED;
    if(atomic_exchange(&busy,true))return ESP_ERR_INVALID_STATE;
    raw_status_t status;esp_err_t err=raw_capture_export_begin(id,&status);
    if(err!=ESP_OK)atomic_store(&busy,false);
    return err;
}
static void release(uint32_t id){raw_capture_export_end(id);atomic_store(&busy,false);}
static esp_err_t unavailable(httpd_req_t *r)
{httpd_resp_set_status(r,"503 Service Unavailable");return httpd_resp_sendstr(r,"capture exporter busy or unavailable");}
static esp_err_t download(httpd_req_t *r)
{
    char query[128],text[12],boot[18];
    if(httpd_req_get_url_query_str(r,query,sizeof(query))!=ESP_OK||
       httpd_query_key_value(query,"boot_id",boot,sizeof(boot))!=ESP_OK||strcmp(boot,boot_id)||
       httpd_query_key_value(query,"id",text,sizeof(text))!=ESP_OK||!*text)
        return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"current capture identity required");
    uint64_t id=0;
    for(const char *p=text;*p;++p){if(*p<'0'||*p>'9')return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"invalid id");id=id*10+(unsigned)(*p-'0');}
    if(!id||id>UINT32_MAX)return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"invalid id");
    if(atomic_load(&busy)||!queue||!task)return unavailable(r);
    if(reserve((uint32_t)id)!=ESP_OK)return atomic_load(&busy)?unavailable(r):httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"capture is not frozen");
    httpd_req_t *copy=NULL;
    if(httpd_req_async_handler_begin(r,&copy)!=ESP_OK){release((uint32_t)id);return httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"async request allocation failed");}
    const job_t job={.id=(uint32_t)id,.pose=!strncmp(r->uri,"/api/raw-pose.csv",17),.request=copy};
    if(xQueueSend(queue,&job,0)!=pdTRUE){
        (void)unavailable(copy);(void)httpd_req_async_handler_complete(copy);release(job.id);
    }
    return ESP_OK; /* HTTP server is free to handle live telemetry immediately. */
}
esp_err_t capture_export_start(const char *boot,const char *version)
{
    if(!boot||strlen(boot)!=16||strspn(boot,"0123456789abcdef")!=16||!version||strlen(version)>64||strpbrk(version,"\r\n"))return ESP_ERR_INVALID_ARG;
    if(queue)return !strcmp(boot,boot_id)&&!strcmp(version,firmware)?ESP_OK:ESP_ERR_INVALID_STATE;
    strcpy(boot_id,boot);strcpy(firmware,version);queue=xQueueCreate(1,sizeof(job_t));
    if(!queue)return ESP_ERR_NO_MEM;
    if(xTaskCreatePinnedToCore(worker,"capture_export",8192,NULL,2,&task,1)!=pdPASS){vQueueDelete(queue);queue=NULL;task=NULL;return ESP_ERR_NO_MEM;}
    return ESP_OK;
}
esp_err_t capture_export_register(httpd_handle_t server)
{
    if(!server||!queue||!task)return ESP_ERR_INVALID_STATE;
    const httpd_uri_t raw={.uri="/api/raw.csv",.method=HTTP_GET,.handler=download};
    const httpd_uri_t pose={.uri="/api/raw-pose.csv",.method=HTTP_GET,.handler=download};
    esp_err_t err=httpd_register_uri_handler(server,&raw);if(err!=ESP_OK)return err;
    err=httpd_register_uri_handler(server,&pose);
    if(err!=ESP_OK)(void)httpd_unregister_uri_handler(server,raw.uri,HTTP_GET);
    return err;
}
esp_err_t capture_export_usb(uint32_t id,bool pose)
{
    esp_err_t err=reserve(id);if(err!=ESP_OK)return err;
    const job_t job={.id=id,.pose=pose};
    if(xQueueSend(queue,&job,0)!=pdTRUE){release(id);return ESP_FAIL;}
    return ESP_OK;
}
void capture_export_get_status(capture_export_status_t *out)
{
    if(out)*out=(capture_export_status_t){.ready=queue&&task,.busy=atomic_load(&busy),
        .completed=atomic_load(&completed),.failed=atomic_load(&failed),.stack_min_free_bytes=atomic_load(&stack_min)};
}
#else
esp_err_t capture_export_start(const char *b,const char *v){(void)b;(void)v;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t capture_export_register(httpd_handle_t s){(void)s;return ESP_ERR_NOT_SUPPORTED;}
esp_err_t capture_export_usb(uint32_t id,bool pose){(void)id;(void)pose;return ESP_ERR_NOT_SUPPORTED;}
void capture_export_get_status(capture_export_status_t *out){if(out)memset(out,0,sizeof(*out));}
#endif
