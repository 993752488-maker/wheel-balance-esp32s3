#include "raw_capture.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t raw_capture_write_csv(uint32_t id,const char *boot,const char *firmware,raw_csv_sink_t sink,void *arg)
{
    if(!boot || !firmware || !sink || strlen(boot)!=16 || strlen(firmware)>64 ||
       strpbrk(boot,"\r\n") || strpbrk(firmware,"\r\n"))return ESP_ERR_INVALID_ARG;
    raw_status_t s;
    esp_err_t err=raw_capture_export_begin(id,&s);if(err!=ESP_OK)return err;
    char line[1024];
    int n=snprintf(line,sizeof(line),
        "# schema=wheelbalance_raw_v1\n# source=live_device\n# firmware=%s\n# boot_id=%s\n"
        "# capture_id=%" PRIu32 "\n# context=%s\n# start_us=%" PRIu64 "\n# deadline_us=%" PRIu64
        "\n# end_us=%" PRIu64 "\n# end_reason=%s\n# samples=%u\n# references=%u\n# faults=%" PRIu32
        "\n# queue_drops=%" PRIu32 "\n# integrity_ok=%u\n# measurement_qualified=0\n"
        "# nominal_sample_hz=500\n# accel_lsb_per_g=8192\n"
        "# time_basis=host_fifo_observations_not_exact_sample_times\n"
        "# window_basis=producer_admission_may_contain_partial_fifo_batches\n"
        "# row_order=samples_then_references\n"
        "record_type,capture_id,ingest_us,sequence,epoch,readout_us,count_begin_us,count_end_us,"
        "pending_frames,batch_index,raw_x,raw_y,raw_z,reference_us,has_period,resynced\n",
        firmware,boot,id,raw_context_name(s.context),s.start_us,s.deadline_us,s.end_us,
        raw_end_name(s.end_reason),(unsigned)s.samples,(unsigned)s.references,s.faults,s.queue_drops,s.integrity_ok);
    if(n<0 || (size_t)n>=sizeof(line) || !sink(line,(size_t)n,arg)){err=ESP_FAIL;goto done;}
    raw_sample_t samples[8];
    for(size_t offset=0;offset<s.samples;) {
        const size_t count=raw_capture_read_samples(id,offset,samples,8);
        if(!count){err=ESP_FAIL;goto done;}
        for(size_t i=0;i<count;++i){const size_t len=raw_csv_sample(line,sizeof(line),id,samples+i);
            if(!len || !sink(line,len,arg)){err=ESP_FAIL;goto done;}}
        offset+=count;
        /* A loopback client and USB bridge can stay runnable together. A real
         * blocked interval lets the CPU's idle/watchdog task run, even with a
         * fast consumer; taskYIELD alone would not schedule lower priorities. */
        vTaskDelay(2);
    }
    raw_reference_t refs[8];
    for(size_t offset=0;offset<s.references;) {
        const size_t count=raw_capture_read_references(id,offset,refs,8);
        if(!count){err=ESP_FAIL;goto done;}
        for(size_t i=0;i<count;++i){const size_t len=raw_csv_reference(line,sizeof(line),id,refs+i);
            if(!len || !sink(line,len,arg)){err=ESP_FAIL;goto done;}}
        offset+=count;
        vTaskDelay(2);
    }
    n=snprintf(line,sizeof(line),"# exported_samples=%u\n# exported_references=%u\n# end=1\n",(unsigned)s.samples,(unsigned)s.references);
    if(n<0 || (size_t)n>=sizeof(line) || !sink(line,(size_t)n,arg))err=ESP_FAIL;
done:
    raw_capture_export_end(id);return err;
}

esp_err_t raw_capture_write_pose_csv(uint32_t id,const char *boot,const char *firmware,raw_csv_sink_t sink,void *arg)
{
    if(!boot||!firmware||!sink||strlen(boot)!=16||strlen(firmware)>64||
       strpbrk(boot,"\r\n")||strpbrk(firmware,"\r\n"))return ESP_ERR_INVALID_ARG;
    raw_status_t raw;raw_pose_status_t pose;
    esp_err_t err=raw_capture_export_begin(id,&raw);if(err!=ESP_OK)return err;
    if(!raw_capture_get_pose_status(id,&pose)||!pose.frozen){err=ESP_ERR_INVALID_STATE;goto done;}
    uint64_t window_end=raw.deadline_us;
    if(raw.stop_requested_us<window_end)window_end=raw.stop_requested_us;
    char line[1536];
    int n=snprintf(line,sizeof(line),
        "# schema=wheelbalance_pose_v1\n# source=live_device\n# firmware=%s\n# boot_id=%s\n"
        "# capture_id=%" PRIu32 "\n# context=%s\n# start_us=%" PRIu64 "\n# window_end_us=%" PRIu64
        "\n# end_us=%" PRIu64 "\n# enabled=%u\n# observations=%u\n# capacity=%u\n# faults=%" PRIu32
        "\n# queue_drops=%" PRIu32 "\n# boundary_excluded=%" PRIu32 "\n# stale_events=%" PRIu32
        "\n# first_epoch=%" PRIu32 "\n# last_epoch=%" PRIu32 "\n# raw_first_epoch=%" PRIu32 "\n# raw_last_epoch=%" PRIu32
        "\n# raw_integrity_ok=%u\n# raw_faults=%" PRIu32 "\n# measurement_qualified=0\n"
        "# accel_lsb_per_g=8192\n# gyro_lsb_per_dps=131\n"
        "# time_basis=register_read_bracket_not_exact_ADC_times\n"
        "# role=optional_pose_diagnostic_not_balance_input\n"
        "capture_id,pose_index,epoch,ingest_us,read_begin_us,read_end_us,fifo_sequence,"
        "raw_ax,raw_ay,raw_az,raw_temperature,raw_gx,raw_gy,raw_gz,frame_consistent\n",
        firmware,boot,id,raw_context_name(raw.context),raw.start_us,window_end,raw.end_us,
        pose.enabled,(unsigned)pose.count,(unsigned)pose.capacity,pose.faults,pose.queue_drops,
        pose.boundary_excluded,pose.stale_events,pose.first_epoch,pose.last_epoch,raw.first_epoch,raw.last_epoch,
        raw.integrity_ok,raw.faults);
    if(n<0||(size_t)n>=sizeof(line)||!sink(line,(size_t)n,arg)){err=ESP_FAIL;goto done;}
    raw_pose_t observations[8];
    for(size_t offset=0;offset<pose.count;){
        const size_t count=raw_capture_read_poses(id,offset,observations,8);
        if(!count){err=ESP_FAIL;goto done;}
        for(size_t i=0;i<count;++i){const size_t len=raw_pose_csv(line,sizeof(line),id,offset+i,observations+i);
            if(!len||!sink(line,len,arg)){err=ESP_FAIL;goto done;}}
        offset+=count;vTaskDelay(2);
    }
    n=snprintf(line,sizeof(line),"# exported_observations=%u\n# end=1\n",(unsigned)pose.count);
    if(n<0||(size_t)n>=sizeof(line)||!sink(line,(size_t)n,arg))err=ESP_FAIL;
done:
    raw_capture_export_end(id);return err;
}
