#include "timing_console.h"
#include "sdkconfig.h"
#if CONFIG_MPU_TIMING_TRACE_ENABLED
#include "acquisition.h"
#include "diagnostic_web.h"
#include "serial_console.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

static atomic_bool s_exporting;
static bool record(const char *text,size_t n)
{
    const int64_t deadline=esp_timer_get_time()+2000000;
    do {if(serial_console_try_record(text,n))return true;vTaskDelay(1);}while(esp_timer_get_time()<deadline);
    return false;
}
static void export_task(void *arg)
{
    const uint32_t id=(uint32_t)(uintptr_t)arg;
    timing_trace_status_t status;esp_err_t err=acquisition_timing_lease(id,&status);
    char line[512],payload[96];uint64_t times[32];size_t offset=0,bytes=0;uint32_t crc=UINT32_MAX;
    const char *boot=diagnostic_web_boot_id();
    const bool leased=err==ESP_OK;
    if(leased && !boot)err=ESP_ERR_INVALID_STATE;
    if(err==ESP_OK) {
        const int n=snprintf(line,sizeof(line),"TIME_EXPORT_BEGIN schema=wheelbalance_drdy_v1 source=live_device id=%" PRIu32 " boot_id=%s firmware=%s epoch=%" PRIu32 " start_us=%" PRIu64 " deadline_us=%" PRIu64 " events=%u faults=%" PRIu32 " physical_timing_qualified=0\n",
            id,boot,esp_app_get_description()->version,status.epoch,status.start_us,status.deadline_us,(unsigned)status.events,status.faults);
        if(n<0 || (size_t)n>=sizeof(line) || !record(line,(size_t)n))err=ESP_ERR_TIMEOUT;
    }
    const int64_t deadline=esp_timer_get_time()+20000000;
    while(err==ESP_OK && offset<status.events) {
        if(esp_timer_get_time()>deadline){err=ESP_ERR_TIMEOUT;break;}
        const size_t n=acquisition_timing_read(id,offset,times,32);
        if(!n){err=ESP_ERR_INVALID_STATE;break;}
        for(size_t i=0;i<n;++i) {
            const int len=snprintf(payload,sizeof(payload),"%u,%" PRIu64 "\n",(unsigned)offset,times[i]);
            if(len<0 || (size_t)len>=sizeof(payload)){err=ESP_ERR_INVALID_SIZE;break;}
            for(int p=0;p<len;++p) {
                crc^=(unsigned char)payload[p];
                for(unsigned b=0;b<8;++b)crc=(crc>>1)^((crc&1)?UINT32_C(0xedb88320):0);
            }
            bytes+=(size_t)len;payload[len-1]=0;
            const int wrote=snprintf(line,sizeof(line),"TIME_DATA id=%" PRIu32 " line=%u data=%s\n",id,(unsigned)offset,payload);
            if(wrote<0 || (size_t)wrote>=sizeof(line) || !record(line,(size_t)wrote)){err=ESP_ERR_TIMEOUT;break;}
            ++offset;
        }
        if(err==ESP_OK)vTaskDelay(2);
    }
    if(leased)acquisition_timing_release(id);
    const int n=snprintf(line,sizeof(line),"TIME_EXPORT_END id=%" PRIu32 " lines=%u bytes=%u crc32=%08" PRIx32 " result=%s\n",id,(unsigned)offset,(unsigned)bytes,crc^UINT32_MAX,esp_err_to_name(err));
    if(n>0 && (size_t)n<sizeof(line))(void)record(line,(size_t)n);
    atomic_store(&s_exporting,false);vTaskDelete(NULL);
}
bool timing_console_command(const char *line)
{
    if(strncmp(line,"TIME_",5))return false;
    timing_trace_status_t status;acquisition_timing_status(&status);
    if(!strcmp(line,"TIME_STATUS")) {
        serial_console_printf("TIME_STATUS id=%" PRIu32 " state=%u epoch=%" PRIu32 " events=%u faults=%" PRIu32 " start_us=%" PRIu64 " deadline_us=%" PRIu64 " physical_timing_qualified=0\n",
            status.id,(unsigned)status.state,status.epoch,(unsigned)status.events,status.faults,status.start_us,status.deadline_us);return true;
    }
    esp_err_t err=ESP_ERR_INVALID_ARG;uint32_t value=0;
    const char *digits=strchr(line,' ');
    if(digits && digits[1]) {
        bool valid=true;
        for(const char *p=digits+1;*p;++p) {
            if(*p<'0'||*p>'9'||value>(UINT32_MAX-(unsigned)(*p-'0'))/10){valid=false;break;}
            value=value*10+(unsigned)(*p-'0');
        }
        if(valid) {
            if(!strncmp(line,"TIME_START ",11))err=acquisition_timing_begin(value);
            else if(!strncmp(line,"TIME_EXPORT ",12) && value && !atomic_exchange(&s_exporting,true)) {
                if(xTaskCreatePinnedToCore(export_task,"timing_export",4096,(void *)(uintptr_t)value,2,NULL,1)==pdPASS)err=ESP_OK;
                else {atomic_store(&s_exporting,false);err=ESP_ERR_NO_MEM;}
            }
        }
    }
    serial_console_printf("TIME_COMMAND command=%s result=%s\n",line,esp_err_to_name(err));return true;
}
#else
bool timing_console_command(const char *line){(void)line;return false;}
#endif
