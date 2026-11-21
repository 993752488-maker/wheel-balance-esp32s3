#include "pose_stream.h"
#include "sdkconfig.h"
#if CONFIG_MPU_POSE_DIAG_ENABLED
#include "serial_console.h"
#include "esp_app_desc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
typedef struct { uint64_t sequence;pose_observation_t value; } event_t;
static QueueHandle_t s_queue;
static char s_boot[17];
static uint64_t s_sequence;
static atomic_uint s_queue_drops,s_tx_drops;
static void worker(void *arg)
{
    (void)arg;event_t event;char line[512];
    serial_console_printf("POSE_META source=live_device boot_id=%s firmware=%s dlpf_cfg=6 nominal_bandwidth_hz=5 accel_lsb_g=8192 gyro_lsb_dps=131 timing=register_read_bracket qualified=0\n",s_boot,esp_app_get_description()->version);
    for(;;) {
        if(xQueueReceive(s_queue,&event,portMAX_DELAY)!=pdTRUE)continue;
        const pose_observation_t *v=&event.value;
        const int n=snprintf(line,sizeof(line),"POSE source=live_device boot_id=%s cfg=6 seq=%" PRIu64 " epoch=%" PRIu32 " begin_us=%" PRIu64 " end_us=%" PRIu64 " ax=%d ay=%d az=%d temp=%d gx=%d gy=%d gz=%d fifo_seq=%" PRIu64 " frame_ok=%d q_drop=%u tx_drop=%u qualified=0\n",
            s_boot,event.sequence,v->epoch,v->read_begin_us,v->read_end_us,
            (int)v->accel[0],(int)v->accel[1],(int)v->accel[2],(int)v->temperature,
            (int)v->gyro[0],(int)v->gyro[1],(int)v->gyro[2],v->fifo_sequence,v->frame_consistent,
            (unsigned)atomic_load(&s_queue_drops),(unsigned)atomic_load(&s_tx_drops));
        if(n<0 || (size_t)n>=sizeof(line) || !serial_console_try_record(line,(size_t)n))atomic_fetch_add(&s_tx_drops,1);
    }
}
esp_err_t pose_stream_start(const char *boot)
{
    if(s_queue || !boot || strlen(boot)!=16)return ESP_ERR_INVALID_STATE;
    memcpy(s_boot,boot,17);s_queue=xQueueCreate(128,sizeof(event_t));
    if(!s_queue)return ESP_ERR_NO_MEM;
    if(xTaskCreatePinnedToCore(worker,"pose_log",3072,NULL,2,NULL,1)!=pdPASS){vQueueDelete(s_queue);s_queue=NULL;return ESP_ERR_NO_MEM;}
    return ESP_OK;
}
void pose_stream_publish(const pose_observation_t *v)
{
    if(!v || !s_queue)return;
    const event_t event={.sequence=++s_sequence,.value=*v};
    if(xQueueSend(s_queue,&event,0)!=pdTRUE)atomic_fetch_add(&s_queue_drops,1);
}
#else
esp_err_t pose_stream_start(const char *boot){(void)boot;return ESP_ERR_NOT_SUPPORTED;}
void pose_stream_publish(const pose_observation_t *v){(void)v;}
#endif
