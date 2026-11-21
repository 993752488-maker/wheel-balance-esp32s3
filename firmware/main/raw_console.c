#include "raw_console.h"
#include "timing_console.h"
#include "raw_capture.h"
#include "serial_console.h"
#include "diagnostic_web.h"
#include "acquisition.h"
#include "optical_ref.h"
#include "sdkconfig.h"
#include "product_session.h"
#include "capture_export.h"
#include "esp_heap_caps.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_RAW_CAPTURE_ENABLED
static bool product_command(const char *line)
{
    static product_snapshot_t status;
    static product_trial_guide_t guide;
    static product_retest_log_t retests;
    if(strncmp(line,"PRODUCT_",8))return false;
    product_session_get_full_view(&status,&guide,&retests);
    if(!strcmp(line,"PRODUCT_STATUS")) {
        serial_console_printf("PRODUCT_SESSION id=%" PRIu32 " revision=%" PRIu32 " state=%s stage=%u source=%s historical=%d storage_ok=%d original=%d trial=%d retest=%d calculation_valid=%d engineering_available=%d mass_g=%.6f angle_deg=%.6f improvement_available=%d improvement_pct=%.6f history=%u reason=%s accuracy_validated=0 firmware=%s axis_mode=%s axis=%u axis_resolved=%d trial_mass_g=%.3f trial_radius_mm=%.1f trial_angle_deg=%.1f correction_radius_mm=%.1f trial_attempts=%" PRIu32 "\n",
            status.id,status.revision,product_state_name(status.state),(unsigned)status.stage,status.simulated?"synthetic":"device",
            status.historical,status.storage_ok,status.stages[0].captured,status.stages[1].captured,status.stages[2].captured,
            status.solution.calculation_valid,status.solution.engineering_available,status.solution.calculation.mass_g,
            status.solution.calculation.angle_deg,status.comparison.available,status.comparison.improvement_percent,
            (unsigned)product_session_history(NULL,PRODUCT_HISTORY),product_reason_name(status.reason),status.firmware,
            guide.automatic_axis?"auto":"manual",status.config.axis,guide.axis_resolved,status.config.mass_g,
            status.config.trial_radius_mm,status.config.trial_angle_deg,status.config.correction_radius_mm,guide.count);return true;
    }
    if(!strcmp(line,"PRODUCT_RETESTS")){
        const bool valid=product_retest_valid(&retests,&status);
        serial_console_printf("RETEST_STATUS session=%" PRIu32 " revision=%" PRIu32 " valid=%d retained=%" PRIu32 " total=%" PRIu32 " dropped=%" PRIu32 " legacy_unknown=%d group_variable=%d capacity=%u\n",
            status.id,status.revision,valid,valid?retests.count:0,valid?retests.total_count:0,valid?retests.dropped_count:0,
            valid&&retests.legacy_count_unknown,valid&&retests.current_group_variable,PRODUCT_RETEST_HISTORY);
        for(unsigned i=0;valid&&i<retests.count;++i){const product_retest_entry_t *e=&retests.entries[i];
            serial_console_printf("RETEST_ENTRY session=%" PRIu32 " ordinal=%" PRIu32 " capture=%" PRIu32 " group=%" PRIu32 " condition=%s legacy=%d comparison=%d improvement_pct=%.6f repeat=%s previous=%" PRIu32 " pairs=%u amplitude_change_pct=%.6f\n",
                status.id,e->ordinal,e->stage.capture_id,e->correction_group,product_retest_condition_name(e->condition),e->legacy,
                e->comparison.available,e->comparison.improvement_percent,product_repeat_state_name(e->repeat.state),
                e->repeat.previous_capture_id,e->repeat.matched_turns,e->repeat.amplitude_change_percent);}
        return true;
    }
    if(!strcmp(line,"PRODUCT_WEB_CHECK")){(void)diagnostic_web_run_selfcheck();return true;}
    product_request_t r={.id=status.id,.revision=status.revision};
    if(!strcmp(line,"PRODUCT_DEMO"))r.action=PRODUCT_DEMO;
    else if(!strcmp(line,"PRODUCT_DEMO_TRIAL"))r.action=PRODUCT_DEMO_TRIAL;
    else if(!strcmp(line,"PRODUCT_DEMO_RETEST"))r.action=PRODUCT_DEMO_RETEST;
    else if(!strcmp(line,"PRODUCT_NEW")){r.action=PRODUCT_NEW;product_session_get_real_config(&r.config);}
    else return true;
    const bool ok=product_session_submit(&r);serial_console_printf("PRODUCT_COMMAND command=%s queued=%d\n",line,ok);return true;
}
static void command(const char *line)
{
    if(product_command(line))return;
    if(timing_console_command(line))return;
    if(!strcmp(line,"WEB_TRACE")){(void)diagnostic_web_trace_to_serial();return;}
    if(!strcmp(line,"WEB_LINK_STATUS")){(void)diagnostic_web_link_status_to_serial();return;}
    if(!strcmp(line,"EXPORT_STATUS")){
        capture_export_status_t e;capture_export_get_status(&e);
        serial_console_printf("EXPORT_STATUS ready=%d busy=%d completed=%" PRIu32 " failed=%" PRIu32 " stack_free_bytes=%" PRIu32 " internal_free=%u internal_largest=%u psram_free=%u psram_largest=%u\n",
            e.ready,e.busy,e.completed,e.failed,e.stack_min_free_bytes,
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));return;
    }
    raw_status_t status;raw_capture_get_status(&status);
    uint32_t value=0;esp_err_t err=ESP_ERR_INVALID_ARG;
    if(!strcmp(line,"RAW_STATUS")) {
        raw_pose_status_t pose={0};(void)raw_capture_get_pose_status(status.id,&pose);
        serial_console_printf("RAW_STATUS id=%" PRIu32 " state=%s samples=%u references=%u faults=%" PRIu32 " queue_drops=%" PRIu32 " integrity_ok=%d measurement_qualified=0 pose_enabled=%d pose_count=%u pose_faults=%" PRIu32 " pose_drops=%" PRIu32 "\n",status.id,raw_state_name(status.state),(unsigned)status.samples,(unsigned)status.references,status.faults,status.queue_drops,status.integrity_ok,pose.enabled,(unsigned)pose.count,pose.faults,pose.queue_drops);return;
    }
    const char *digits=strchr(line,' ');
    if(!digits || !digits[1])return;
    for(const char *p=digits+1;*p;++p) {
        if(*p<'0'||*p>'9'||value>(UINT32_MAX-(unsigned)(*p-'0'))/10)return;
        value=value*10+(unsigned)(*p-'0');
    }
    if(!strncmp(line,"RAW_START ",10) && value>=250 && value<=60000) {
        acquisition_status_t imu;acquisition_get_status(&imu);optical_ref_status_t ref;optical_ref_get_status(&ref);
        if(imu.online && ref.enabled && !ref.input_queue_drops && !ref.clock_faults && !ref.sample_gaps)
            err=raw_capture_begin(value,RAW_CONTEXT_USB,false);
        else err=ESP_ERR_INVALID_STATE;
    }else if(!strncmp(line,"RAW_STOP ",9))err=raw_capture_stop(value);
    else if(!strncmp(line,"RAW_EXPORT ",11)&&value)err=diagnostic_web_raw_export_to_serial(value);
    else if(!strncmp(line,"POSE_EXPORT ",12)&&value)err=diagnostic_web_pose_export_to_serial(value);
    else if(!strncmp(line,"CSV_HTTP_CHECK ",15)&&value)err=diagnostic_web_csv_selfcheck(value,0);
    else if(!strncmp(line,"POSE_HTTP_CHECK ",16)&&value)err=diagnostic_web_csv_selfcheck(value,1);
    else if(!strncmp(line,"WEB_LOAD_CHECK ",15)&&value&&value<=1200)err=diagnostic_web_load_selfcheck(value);
    serial_console_printf("RAW_COMMAND command=%s result=%s\n",line,esp_err_to_name(err));
}
#endif
void raw_console_poll(void)
{
#if CONFIG_RAW_CAPTURE_ENABLED
    static char pending[80];static size_t used;static bool overflow;
    char chunk[64];const size_t n=serial_console_read(chunk,sizeof(chunk));
    for(size_t i=0;i<n;++i) {
        if(chunk[i]=='\n') {if(!overflow){pending[used]=0;command(pending);}used=0;overflow=false;}
        else if(chunk[i]!='\r') {if(used<sizeof(pending)-1)pending[used++]=chunk[i];else overflow=true;}
    }
#endif
}
