#include "product_session.h"
#include "product_job.h"
#include "product_auto_axis.h"
#include "raw_capture.h"
#include "live_analysis_core.h"
#include "acquisition.h"
#include "optical_ref.h"
#include "esp_app_desc.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { REPLAY_CAPACITY=64064 };
static product_snapshot_t current,published,history[PRODUCT_HISTORY];
static product_trial_guide_t guide,published_guide,history_guides[PRODUCT_HISTORY];
static product_retest_log_t retests,published_retests,history_retests[PRODUCT_HISTORY];
static product_run_t *previous_retest;
static uint32_t previous_retest_group,synthetic_sequence;
static product_retest_condition_t pending_retest_condition;
static product_config_t real_config;
static product_capture_config_t real_capture_config={.mode=CAPTURE_COAST,.vehicle_type=CAPTURE_BICYCLE};
/* Worker-owned analysis windows need no DMA or ISR access. Keep the network's
 * internal heap available rather than reserving these large arrays in DRAM. */
static product_run_t *runs;
static product_run_t *baseline_axes,*trial_axes;
static product_assessment_t *assessment,*published_assessment,*history_assessments;
static bool axis_finalized;
static uint32_t retry_required_axes;
static product_speed_t *speed,*published_speed,*history_speeds;
static product_window_t *windows,*published_windows,*history_windows;
static uint64_t preview_last_us;
static size_t history_count;
static product_job_t job;
static QueueHandle_t commands;
static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static uint32_t active_capture;
static bool closing;
static sync_sample_t *replay_samples;
static selection_turn_t *selection;
static selection_reference_t *references;
static live_analysis_core_t *clock_core;
static uint32_t i2c_at_arm,fifo_at_arm;
static bool save_pending;
static uint64_t save_retry_us;
static void publish(void)
{++current.revision;guide.snapshot_revision=current.revision;retests.snapshot_revision=current.revision;product_speed_bind(speed,&current);product_window_bind(windows,&current);product_assessment_bind(assessment,&current);
 portENTER_CRITICAL(&lock);published=current;published_guide=guide;published_retests=retests;*published_speed=*speed;*published_windows=*windows;*published_assessment=*assessment;portEXIT_CRITICAL(&lock);}
void product_session_get(product_snapshot_t *out)
{product_session_get_view(out,NULL);}
void product_session_get_view(product_snapshot_t *out,product_trial_guide_t *trial)
{product_session_get_full_view(out,trial,NULL);}
void product_session_get_full_view(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests)
{product_session_get_speed_view(out,trial,saved_retests,NULL);}
void product_session_get_speed_view(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,product_speed_t *profile)
{product_session_get_product_view(out,trial,saved_retests,profile,NULL);}
void product_session_get_product_view(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,product_speed_t *profile,product_window_t *window)
{product_session_get_assessment_view(out,trial,saved_retests,profile,window,NULL);}
void product_session_get_assessment_view(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,product_speed_t *profile,product_window_t *window,product_assessment_t *quality)
{
    portENTER_CRITICAL(&lock);
    if(out)*out=published;
    if(trial)*trial=published_guide;
    if(saved_retests)*saved_retests=published_retests;
    if(profile)*profile=*published_speed;
    if(window){
        if(published_windows)*window=*published_windows;
        else memset(window,0,sizeof(*window));
    }
    if(quality)*quality=*published_assessment;
    portEXIT_CRITICAL(&lock);
}
void product_session_get_real_config(product_config_t *out)
{if(!out)return;portENTER_CRITICAL(&lock);*out=real_config;portEXIT_CRITICAL(&lock);}
size_t product_session_history(product_snapshot_t *out,size_t cap)
{return product_session_history_views(out,NULL,cap);}
size_t product_session_history_views(product_snapshot_t *out,product_trial_guide_t *trial,size_t cap)
{return product_session_history_full_views(out,trial,NULL,cap);}
size_t product_session_history_full_views(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,size_t cap)
{return product_session_history_speed_views(out,trial,saved_retests,NULL,cap);}
size_t product_session_history_speed_views(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,product_speed_t *profiles,size_t cap)
{return product_session_history_product_views(out,trial,saved_retests,profiles,NULL,cap);}
size_t product_session_history_product_views(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,product_speed_t *profiles,product_window_t *window,size_t cap)
{return product_session_history_assessment_views(out,trial,saved_retests,profiles,window,NULL,cap);}
size_t product_session_history_assessment_views(product_snapshot_t *out,product_trial_guide_t *trial,product_retest_log_t *saved_retests,product_speed_t *profiles,product_window_t *window,product_assessment_t *quality,size_t cap)
{
    portENTER_CRITICAL(&lock);
    size_t n=history_count<cap?history_count:cap;
    if(out)memcpy(out,history,n*sizeof(*out));
    if(trial)memcpy(trial,history_guides,n*sizeof(*trial));
    if(saved_retests)memcpy(saved_retests,history_retests,n*sizeof(*saved_retests));
    if(profiles)memcpy(profiles,history_speeds,n*sizeof(*profiles));
    if(window)memcpy(window,history_windows,n*sizeof(*window));
    if(quality)memcpy(quality,history_assessments,n*sizeof(*quality));
    portEXIT_CRITICAL(&lock);
    return n;
}
static void store_history(void)
{
    const bool idle=raw_capture_maintenance_begin()==ESP_OK;
    /* Closing a cancelled/faulted recorder is asynchronous. Retry only after
     * its gate is quiet (also excludes concurrent CSV export leases). */
    save_pending=!idle;save_retry_us=(uint64_t)esp_timer_get_time()+250000;
    const bool ref_paused=idle&&optical_ref_maintenance_begin()==ESP_OK;
    const bool paused=ref_paused&&acquisition_maintenance_begin()==ESP_OK;
    if(idle&&!paused){save_pending=true;save_retry_us=(uint64_t)esp_timer_get_time()+1000000;}
    nvs_handle_t handle;bool ok=paused&&nvs_open("wb_product",NVS_READWRITE,&handle)==ESP_OK;
    if(ok){for(size_t i=0;i<history_count;++i){char key[16];snprintf(key,sizeof(key),"h%u",(unsigned)i);
            if(nvs_set_blob(handle,key,&history[i],sizeof(history[i]))!=ESP_OK)ok=false;
            snprintf(key,sizeof(key),"g%u",(unsigned)i);
            if(product_trial_valid(&history_guides[i],&history[i])){
                if(nvs_set_blob(handle,key,&history_guides[i],sizeof(history_guides[i]))!=ESP_OK)ok=false;
            }else{esp_err_t erased=nvs_erase_key(handle,key);if(erased!=ESP_OK&&erased!=ESP_ERR_NVS_NOT_FOUND)ok=false;}
            snprintf(key,sizeof(key),"r%u",(unsigned)i);
            if(product_retest_valid(&history_retests[i],&history[i])){
                if(nvs_set_blob(handle,key,&history_retests[i],sizeof(history_retests[i]))!=ESP_OK)ok=false;
            }else{esp_err_t erased=nvs_erase_key(handle,key);if(erased!=ESP_OK&&erased!=ESP_ERR_NVS_NOT_FOUND)ok=false;}
            snprintf(key,sizeof(key),"s%u",(unsigned)i);
            if(product_speed_valid(&history_speeds[i],&history[i],true)){
                if(nvs_set_blob(handle,key,&history_speeds[i],sizeof(history_speeds[i]))!=ESP_OK)ok=false;
            }else{esp_err_t erased=nvs_erase_key(handle,key);if(erased!=ESP_OK&&erased!=ESP_ERR_NVS_NOT_FOUND)ok=false;}
            snprintf(key,sizeof(key),"w%u",(unsigned)i);
            if(product_window_valid(&history_windows[i],&history[i],true)){
                if(nvs_set_blob(handle,key,&history_windows[i],sizeof(history_windows[i]))!=ESP_OK)ok=false;
            }else{esp_err_t erased=nvs_erase_key(handle,key);if(erased!=ESP_OK&&erased!=ESP_ERR_NVS_NOT_FOUND)ok=false;}}
        if(!current.simulated){
            product_config_t preferences=current.config;
            if(guide.schema>=4)preferences.trial_angle_deg=guide.initial_angle_deg;
            if(nvs_set_blob(handle,"config",&preferences,sizeof(preferences))!=ESP_OK)ok=false;
            if(nvs_set_blob(handle,"trial_piece_g",&guide.piece_mass_g,sizeof(guide.piece_mass_g))!=ESP_OK)ok=false;
            if(nvs_set_blob(handle,"capture_config",&real_capture_config,sizeof(real_capture_config))!=ESP_OK)ok=false;
        }
        if(nvs_commit(handle)!=ESP_OK)ok=false;
        nvs_close(handle);}
    if(paused&&ok){
        nvs_handle_t metadata;
        bool saved=nvs_flash_init_partition("wb_meta")==ESP_OK&&nvs_open_from_partition("wb_meta","assessment",NVS_READWRITE,&metadata)==ESP_OK;
        if(saved){for(size_t i=0;i<history_count;++i){char key[16];snprintf(key,sizeof(key),"q%u",(unsigned)i);
                if(product_assessment_valid(&history_assessments[i],&history[i])){
                    if(nvs_set_blob(metadata,key,&history_assessments[i],sizeof(history_assessments[i]))!=ESP_OK)saved=false;
                }else{esp_err_t erased=nvs_erase_key(metadata,key);if(erased!=ESP_OK&&erased!=ESP_ERR_NVS_NOT_FOUND)saved=false;}}
            if(nvs_commit(metadata)!=ESP_OK)saved=false;
            nvs_close(metadata);}
        if(!saved)ok=false;
    }
    if(paused&&acquisition_maintenance_end()!=ESP_OK)ok=false;
    if(ref_paused&&optical_ref_maintenance_end()!=ESP_OK)ok=false;
    if(idle)raw_capture_maintenance_end();
    current.storage_ok=ok;portENTER_CRITICAL(&lock);history[0].storage_ok=ok;portEXIT_CRITICAL(&lock);publish();
}
static void persist(void)
{
    /* Give each saved snapshot/sidecar pair a new revision before either NVS
     * key is written. Interrupted writes cannot pair new data with old advice. */
    ++current.revision;guide.snapshot_revision=current.revision;retests.snapshot_revision=current.revision;product_speed_bind(speed,&current);product_window_bind(windows,&current);product_assessment_bind(assessment,&current);
    product_snapshot_t copy=current;copy.historical=true;copy.storage_ok=true;
    portENTER_CRITICAL(&lock);
    size_t at=history_count;for(size_t i=0;i<history_count;++i)if(history[i].id==copy.id){at=i;break;}
    if(at==history_count){if(history_count<PRODUCT_HISTORY)++history_count;at=history_count-1;}
    for(size_t i=at;i>0;--i){history[i]=history[i-1];history_guides[i]=history_guides[i-1];history_retests[i]=history_retests[i-1];history_speeds[i]=history_speeds[i-1];history_windows[i]=history_windows[i-1];history_assessments[i]=history_assessments[i-1];}
    history[0]=copy;history_guides[0]=guide;history_retests[0]=retests;history_speeds[0]=*speed;history_windows[0]=*windows;history_assessments[0]=*assessment;
    portEXIT_CRITICAL(&lock);
    store_history();
}
static void fresh(const product_config_t *config,bool simulated)
{
    char boot[17],fw[32];memcpy(boot,current.boot_id,sizeof(boot));memcpy(fw,current.firmware,sizeof(fw));
    uint32_t revision=current.revision;
    memset(&current,0,sizeof(current));memset(runs,0,3*sizeof(*runs));memset(&job,0,sizeof(job));
    current.schema=1;current.id=esp_random();if(!current.id)current.id=1;current.revision=revision;
    memcpy(current.boot_id,boot,sizeof(boot));memcpy(current.firmware,fw,sizeof(fw));
    current.config=*config;current.simulated=simulated;current.state=PRODUCT_IDLE;current.storage_ok=true;
    product_speed_reset(speed,&current);preview_last_us=0;
    const product_capture_config_t capture=simulated?product_capture_default():real_capture_config;
    product_window_reset(windows,&current,capture);product_assessment_reset(assessment,&current);axis_finalized=false;retry_required_axes=0;
    memset(baseline_axes,0,3*sizeof(*baseline_axes));memset(trial_axes,0,3*sizeof(*trial_axes));
    if(!simulated){portENTER_CRITICAL(&lock);real_config=*config;portEXIT_CRITICAL(&lock);}
    memset(&guide,0,sizeof(guide));
    if(!simulated)(void)product_trial_init(&guide,&current,5,false);
    product_retest_init(&retests,&current);memset(previous_retest,0,sizeof(*previous_retest));
    previous_retest_group=0;pending_retest_condition=RETEST_CONDITION_NONE;synthetic_sequence=0;
    active_capture=0;closing=false;
}
static void terminal(product_state_t state,product_reason_t reason)
{
    current.state=state;current.reason=reason;job.state=state;current.deadline_us=0;current.stages[current.stage].reason=reason;
    if(speed->stages[current.stage].mode==SPEED_PREVIEW)product_speed_clear(speed,current.stage);
    if(active_capture&&!current.stages[current.stage].capture_id){raw_status_t raw;raw_capture_get_status(&raw);
        if(raw.id==active_capture){product_stage_summary_t *s=&current.stages[current.stage];
            s->capture_id=raw.id;s->epoch=raw.first_epoch;s->samples=(uint32_t)raw.samples;s->references=(uint32_t)raw.references;
            s->start_us=raw.start_us;s->end_us=raw.end_us;}}
    if(reason==PRODUCT_SENSOR||reason==PRODUCT_DATA_FAULT||reason==PRODUCT_REFERENCE||reason==PRODUCT_CONTEXT||reason==PRODUCT_NUMERIC||reason==PRODUCT_CAPTURE_LIMIT)
        current.solution.engineering_available=false;
    product_window_stage_t *w=&windows->stages[current.stage];const product_stage_summary_t *source=&current.stages[current.stage];
    if(w->capture_id&&source->capture_id==w->capture_id&&source->end_us){w->start_us=source->start_us;w->end_us=source->end_us;w->phase=WINDOW_COMPLETE;w->end_reason=job.end_reason;}
    else if(w->capture_id)product_window_clear(windows,current.stage);
    if(product_trial_record(&guide,&current))product_trial_apply_advice(&guide,&current);
    if(current.stage==PRODUCT_RETEST&&current.stages[2].capture_id){
        product_repeat_observation_t observation={0};
        const uint32_t group=retests.count?retests.entries[retests.count-1].correction_group:0;
        if(pending_retest_condition==RETEST_CONDITION_SAME){
            observation.state=PRODUCT_REPEAT_UNCOMPARABLE;observation.reason=PRODUCT_CONTEXT;
            if(previous_retest->summary.captured&&previous_retest_group==group)
                product_observe_repeat(&current.config,previous_retest,&runs[2],&current.solution,&observation);
        }
        if(product_retest_record(&retests,&current,pending_retest_condition,&observation)&&runs[2].summary.captured){
            *previous_retest=runs[2];previous_retest_group=retests.entries[retests.count-1].correction_group;}
    }
    persist();
}
static bool reference_ok(const optical_ref_status_t *r)
{return r->enabled&&!r->input_queue_drops&&!r->clock_faults&&!r->sample_gaps;}
static void yield_analysis(void *context){(void)context;vTaskDelay(1);}
/* Replay the frozen record after the selected capture policy ends it. */
static bool replay_window(const raw_status_t *raw,size_t *selected)
{
    bool ok=raw->samples<=REPLAY_CAPACITY&&raw->references<=COAST_SELECTION_MAX_REFS;
    live_core_init(clock_core,replay_samples,REPLAY_CAPACITY);
    raw_sample_t batch[32];size_t offset=0;
    while(ok&&offset<raw->samples){size_t cap=raw->samples-offset;if(cap>32)cap=32;
        size_t n=raw_capture_read_samples(raw->id,offset,batch,cap);if(!n){ok=false;break;}
        for(size_t i=0;i<n;++i){const raw_sample_t *v=&batch[i];
            const live_input_sample_t in={.sequence=v->sequence,.epoch=v->epoch,.count_begin_us=v->count_begin_us,
                .count_end_us=v->count_end_us,.pending_frames=v->pending_frames,.batch_index=v->batch_index,
                .raw={v->raw[0],v->raw[1],v->raw[2]}};live_core_sample(clock_core,&in);}
        offset+=n;if(offset%512==0)vTaskDelay(1);
    }
    size_t nr=0;offset=0;raw_reference_t refs[32];
    while(ok&&offset<raw->references){size_t cap=raw->references-offset;if(cap>32)cap=32;
        size_t n=raw_capture_read_references(raw->id,offset,refs,cap);if(!n){ok=false;break;}
        for(size_t i=0;i<n;++i)references[nr++]=(selection_reference_t){{refs[i].sequence,refs[i].reference_us},refs[i].has_period};
        offset+=n;
    }
    ok=ok&&clock_core->count&&clock_core->count<=raw->samples&&raw->samples-clock_core->count<32&&
        !clock_core->status.timing_resets&&clock_core->last_sequence==raw->last_sequence&&live_core_retime(clock_core);
    *selected=0;
    if(ok){selection_config_t fit=selection_default_config();
        product_window_configure_fit(&fit,job.capture.mode);
        ok=coast_select_progress(&fit,clock_core->samples,clock_core->count,references,nr,
            (sync_quality_t){SYNC_TIME_ESTIMATED,0},selection,COAST_SELECTION_MAX_REFS-2,selected,yield_analysis,NULL)==SELECTION_OK;}
    return ok;
}
static void select_window(const raw_status_t *raw,size_t count,bool final)
{
    product_window_stage_t *w=&windows->stages[current.stage];
    if(job.capture.mode==CAPTURE_TIMED)product_window_select_timed(selection,count,job.target_rpm,w);
    else product_window_select(selection,count,w);
    w->capture_id=raw->id;w->epoch=raw->first_epoch;w->start_us=raw->start_us;
    w->planned_seconds=job.capture.mode==CAPTURE_TIMED?job.capture.seconds:0;
    w->end_us=final?raw->end_us:clock_core->samples[clock_core->count-1].time_us;
    w->updated_us=(uint64_t)esp_timer_get_time();if(final){w->phase=WINDOW_COMPLETE;w->end_reason=job.end_reason;}
    ++windows->generation;
}
static void preview_capture(uint64_t now,const raw_status_t *seen)
{
    /* Cheap status only while rotating: no prefix leases, fitting or Flash.
     * The time shown is elapsed recording time, never a completion target. */
    if(closing||seen->id!=active_capture||!seen->first_epoch||now-preview_last_us<500000)return;
    preview_last_us=now;
    product_window_stage_t *w=&windows->stages[current.stage];memset(w,0,sizeof(*w));
    w->capture_id=seen->id;w->epoch=seen->first_epoch;w->start_us=seen->start_us;
    w->planned_seconds=job.capture.mode==CAPTURE_TIMED?job.capture.seconds:0;
    w->end_us=now;w->updated_us=now;w->phase=WINDOW_COLLECTING;++windows->generation;
    /* Progress updates do not invalidate queued finish/cancel commands. */
    product_speed_bind(speed,&current);product_window_bind(windows,&current);
    portENTER_CRITICAL(&lock);*published_speed=*speed;*published_windows=*windows;portEXIT_CRITICAL(&lock);
}
static void process_capture(void)
{
    raw_status_t raw;
    if(raw_capture_export_begin(active_capture,&raw)!=ESP_OK){terminal(PRODUCT_NO_RESULT,PRODUCT_DATA_FAULT);return;}
    product_run_t *run=&runs[current.stage];memset(run,0,sizeof(*run));
    run->summary=(product_stage_summary_t){.capture_id=raw.id,.epoch=raw.first_epoch,.samples=(uint32_t)raw.samples,
        .references=(uint32_t)raw.references,.start_us=raw.start_us,.end_us=raw.end_us,
        .integrity_valid=raw.integrity_ok&&!raw.faults&&!raw.queue_drops&&raw.first_epoch==raw.last_epoch};
    size_t selected=0;bool solved_here=false,solved_ok=false;
    bool ok=run->summary.integrity_valid&&replay_window(&raw,&selected);
    raw_capture_export_end(raw.id);
    if(ok)select_window(&raw,selected,true);
    if(ok&&current.stage==PRODUCT_ORIGINAL&&guide.automatic_axis){
        for(unsigned axis=0;axis<3;++axis){memset(&baseline_axes[axis],0,sizeof(baseline_axes[axis]));
            baseline_axes[axis].summary=run->summary;
            (void)product_extract_run(selection,selected,axis,&baseline_axes[axis]);}
        product_axis_choice_t choice;ok=product_choose_axis(baseline_axes,&choice);
        if(ok){current.config.axis=choice.axis;*run=baseline_axes[choice.axis];guide.axis_resolved=true;
            memcpy(guide.axis_score,choice.score,sizeof(guide.axis_score));}
        else run->summary.reason=choice.reason;
    }else if(ok&&current.stage==PRODUCT_TRIAL&&guide.automatic_axis&&!axis_finalized){
        for(unsigned axis=0;axis<3;++axis){memset(&trial_axes[axis],0,sizeof(trial_axes[axis]));trial_axes[axis].summary=run->summary;
            (void)product_extract_run(selection,selected,axis,&trial_axes[axis]);}
        unsigned chosen=current.config.axis;
        solved_ok=product_assess_axes(&current.config,baseline_axes,trial_axes,&chosen,&current.solution,assessment);solved_here=true;
        const uint32_t comparable=assessment->axis_available_mask&~assessment->axis_conflict_mask;
        if(solved_ok&&current.solution.reason==PRODUCT_REASON_NONE&&(comparable&retry_required_axes)!=retry_required_axes){
            /* A lighter trial must not resolve a conflict by losing axes
             * that had measurable responses before the load was reduced. */
            current.solution.engineering_available=false;
            current.solution.reason=PRODUCT_CONFLICT;
            assessment->state=ASSESS_VARIABLE;
            assessment->flags|=ASSESS_AXES_DIFFERENT;
        }
        if(current.solution.reason==PRODUCT_CONTEXT||current.solution.reason==PRODUCT_NUMERIC||current.solution.reason==PRODUCT_DATA_FAULT){
            ok=false;run->summary.reason=current.solution.reason;
        }else{
            axis_finalized=solved_ok&&current.solution.reason==PRODUCT_REASON_NONE;
            current.config.axis=chosen;*run=trial_axes[chosen];runs[0]=baseline_axes[chosen];current.stages[0]=runs[0].summary;
            memcpy(guide.axis_score,assessment->axis_scores,sizeof(guide.axis_score));guide.axis_resolved=true;
            (void)product_speed_set(speed,0,&runs[0],false,(uint64_t)esp_timer_get_time());
            portENTER_CRITICAL(&lock);real_config=current.config;real_config.trial_angle_deg=guide.initial_angle_deg;portEXIT_CRITICAL(&lock);
            ok=run->summary.captured;
        }
    }else if(ok)ok=product_extract_run(selection,selected,current.config.axis,run);
    if(ok&&current.stage==PRODUCT_ORIGINAL)guide.axis_resolved=true;
    if(!ok && run->summary.reason==PRODUCT_REASON_NONE)run->summary.reason=PRODUCT_DATA_FAULT;
    current.stages[current.stage]=run->summary;
    if(ok&&current.stage)windows->stages[current.stage].common_pairs=product_common_turns(&runs[0],run,
        current.stage==PRODUCT_RETEST?current.solution.lower_rpm:0,current.stage==PRODUCT_RETEST?current.solution.upper_rpm:0);
    (void)product_speed_set(speed,current.stage,run,false,(uint64_t)esp_timer_get_time());
    if(!ok){terminal(PRODUCT_NO_RESULT,run->summary.reason);return;}
    if(current.stage==PRODUCT_TRIAL){
        ok=solved_here?solved_ok:product_assess_runs(&current.config,&runs[0],&runs[1],&current.solution,assessment);
        if(ok&&product_trial_contradicts_previous(&guide,&current)){
            current.solution.engineering_available=false;
            current.solution.reason=PRODUCT_CONFLICT;
            assessment->state=ASSESS_VARIABLE;
            axis_finalized=false;
        }
        assessment->automatic_axis=guide.automatic_axis;assessment->axis_finalized=ok&&current.solution.reason!=PRODUCT_CONFLICT;
        if(guide.automatic_axis)memcpy(guide.axis_score,assessment->axis_scores,sizeof(guide.axis_score));
        product_assessment_bind(assessment,&current);
        terminal(ok && current.solution.reason!=PRODUCT_CONFLICT?PRODUCT_COMPLETE:PRODUCT_NO_RESULT,current.solution.reason);
    }else if(current.stage==PRODUCT_RETEST){
        ok=product_compare_runs(&current.config,&runs[0],&runs[2],&current.solution,&current.comparison);
        terminal(ok?PRODUCT_COMPLETE:PRODUCT_NO_RESULT,current.comparison.reason);
    }else {
        (void)product_trial_plan_original(&guide,&current,product_capture_baseline_rpm(&current.stages[0]));
        product_assessment_reset(assessment,&current);terminal(PRODUCT_COMPLETE,PRODUCT_REASON_NONE);
    }
}
static void synthetic_run(unsigned stage)
{
    product_run_t *r=&runs[stage];memset(r,0,sizeof(*r));
    const uint32_t sequence=++synthetic_sequence;
    r->summary=(product_stage_summary_t){.capture_id=sequence,.epoch=1,.samples=4000,.references=10,
        .start_us=1000000+(uint64_t)(sequence-1)*10000000,.end_us=9000000+(uint64_t)(sequence-1)*10000000,.integrity_valid=true};
    selection_turn_t points[8];memset(points,0,sizeof(points));
    const double a=.05358983848622454,b=-.8928203230275509;
    for(unsigned i=0;i<8;++i){coast_result_t *f=&points[i].fit;f->valid=true;f->status=COAST_OK;f->epoch=1;
        f->rpm=116-i*2;f->rpm_previous=f->rpm+2;f->ref_last=i+2+stage*10;f->end_us=r->summary.start_us+(i+1)*900000;f->start_us=f->end_us-900000;
        f->axis[current.config.axis].real=stage==2?a*.2:a+(stage==1?1:0);
        f->axis[current.config.axis].imag=stage==2?b*.2:b+(stage==1?-.5:0);}
    product_window_stage_t *w=&windows->stages[stage];product_window_select(points,8,w);
    w->capture_id=r->summary.capture_id;w->epoch=1;w->start_us=r->summary.start_us;w->end_us=r->summary.end_us;
    w->updated_us=(uint64_t)esp_timer_get_time();w->phase=WINDOW_COMPLETE;++windows->generation;
    (void)product_extract_run(points,8,current.config.axis,r);current.stages[stage]=r->summary;
    if(stage)w->common_pairs=product_common_turns(&runs[0],r,stage==2?current.solution.lower_rpm:0,stage==2?current.solution.upper_rpm:0);
    (void)product_speed_set(speed,stage,r,false,(uint64_t)esp_timer_get_time());
}
static void command(const product_request_t *request)
{
    if(request->id!=current.id || request->revision!=current.revision)return;
    const bool busy=current.state==PRODUCT_ARMED||current.state==PRODUCT_RECORDING||current.state==PRODUCT_PROCESSING;
    if(request->action==PRODUCT_CANCEL){if(!busy)return;
        if(current.state==PRODUCT_RECORDING)(void)raw_capture_stop(active_capture);
        terminal(PRODUCT_CANCELLED,PRODUCT_REASON_NONE);return;}
    if(request->action==PRODUCT_FINISH){
        if(current.state==PRODUCT_RECORDING&&job.state==PRODUCT_RECORDING&&!closing&&raw_capture_stop(active_capture)==ESP_OK){
            (void)product_job_finish(&job);closing=true;}
        return;}
    if(busy)return;
    if(request->action==PRODUCT_SET_CAPTURE){
        if(current.state==PRODUCT_IDLE&&!current.simulated&&!current.historical&&
           product_capture_config_valid(&request->capture)){
            real_capture_config=request->capture;
            product_capture_apply_preset(&real_capture_config);
            windows->capture=real_capture_config;
            store_history();
        }
        return;
    }
    if(request->action==PRODUCT_NEW){if(product_config_valid(&request->config)){
        if(request->capture_config_present){
            if(!product_capture_config_valid(&request->capture))return;
            real_capture_config=request->capture;product_capture_apply_preset(&real_capture_config);
        }
        fresh(&request->config,false);
        (void)product_trial_init(&guide,&current,request->piece_mass_g>0?request->piece_mass_g:5,request->guided);
        guide.automatic_axis=!request->manual_axis;guide.axis_resolved=false;
        portENTER_CRITICAL(&lock);real_config=current.config;portEXIT_CRITICAL(&lock);
        persist();}return;}
    if(request->action==PRODUCT_DEMO){product_config_t c=product_default_config();c.mass_g=5;c.trial_radius_mm=c.correction_radius_mm=100;
        strcpy(c.vehicle,"合成演示");fresh(&c,true);current.stage=PRODUCT_ORIGINAL;synthetic_run(0);terminal(PRODUCT_COMPLETE,PRODUCT_REASON_NONE);return;}
    if(request->action==PRODUCT_DEMO_TRIAL||request->action==PRODUCT_DEMO_RETEST){
        if(!current.simulated||!current.stages[0].captured)return;
        unsigned stage=request->action==PRODUCT_DEMO_TRIAL?1:2;
        if(stage==2&&!product_retest_available(&current))return;
        if(stage==2)pending_retest_condition=retests.count?RETEST_CONDITION_SAME:RETEST_CONDITION_FIRST;
        current.stage=(product_stage_t)stage;synthetic_run(stage);
        if(stage==1){(void)product_assess_runs(&current.config,&runs[0],&runs[1],&current.solution,assessment);product_assessment_bind(assessment,&current);current.solution.calculation.simulated=true;}
        else (void)product_compare_runs(&current.config,&runs[0],&runs[2],&current.solution,&current.comparison);
        terminal(PRODUCT_COMPLETE,PRODUCT_REASON_NONE);return;}
    if(current.simulated||current.historical)return;
    if(request->action==PRODUCT_TRIAL_PLAN){
        acquisition_status_t imu;acquisition_get_status(&imu);
        if(!runs[0].summary.captured)return;
        if(imu.epoch!=runs[0].summary.epoch||!imu.online){
            active_capture=0;current.stage=PRODUCT_TRIAL;
            terminal(PRODUCT_NO_RESULT,!imu.online?PRODUCT_SENSOR:PRODUCT_CONTEXT);return;}
        if(product_trial_set_plan(&guide,&current,request->confirmed_mass_g,request->plan_source))persist();
        return;}
    unsigned stage=request->action==PRODUCT_START_ORIGINAL?0:request->action==PRODUCT_START_TRIAL?1:2;
    /* Restarting after trial captures requires a fresh session: otherwise the
     * attempt log and the physical meaning of V0 would be silently replaced. */
    if(stage==0&&guide.count)return;
    if(stage && !runs[0].summary.captured)return;
    if(stage==1&&(!request->trial_installed||!product_trial_can_start(&guide,&current,request->confirmed_mass_g)))return;
    if(stage==2&&(!request->trial_removed||!product_retest_available(&current)||
                 !product_assessment_valid(assessment,&current)))return;
    if(stage==2){product_retest_condition_t condition=request->retest_condition;
        if(!retests.count&&condition==RETEST_CONDITION_NONE)condition=RETEST_CONDITION_FIRST;
        if(!product_retest_can_start(&retests,condition))return;
        pending_retest_condition=condition;}
    acquisition_status_t imu;optical_ref_status_t ref;raw_status_t raw;
    acquisition_get_status(&imu);optical_ref_get_status(&ref);raw_capture_get_status(&raw);
    const bool extend_record=current.stage==stage&&(current.reason==PRODUCT_TOO_FEW_TURNS||
        current.reason==PRODUCT_INCONSISTENT||current.reason==PRODUCT_LOW_DIRECTION_COHERENCE);
    const unsigned previous_seconds=windows->stages[stage].planned_seconds;
    if(stage==1&&guide.count&&current.reason==PRODUCT_CONFLICT&&
       (current.config.mass_g<guide.attempts[guide.count-1].mass_g||current.config.trial_angle_deg!=guide.attempt_angle_deg[guide.count-1])&&
       product_assessment_valid(assessment,&current)&&
       (assessment->flags&ASSESS_AXES_DIFFERENT)){
        retry_required_axes|=assessment->axis_available_mask&~assessment->axis_conflict_mask;
    }
    current.stage=(product_stage_t)stage;
    if(stage==0){memset(runs,0,3*sizeof(*runs));memset(current.stages,0,sizeof(current.stages));product_speed_reset(speed,&current);
        product_window_reset(windows,&current,windows->capture);
        product_retest_init(&retests,&current);memset(previous_retest,0,sizeof(*previous_retest));previous_retest_group=0;}
    if(stage==0){axis_finalized=false;retry_required_axes=0;memset(baseline_axes,0,3*sizeof(*baseline_axes));memset(trial_axes,0,3*sizeof(*trial_axes));guide.axis_resolved=false;memset(guide.axis_score,0,sizeof(guide.axis_score));}
    else{memset(&runs[stage],0,sizeof(runs[stage]));memset(&current.stages[stage],0,sizeof(current.stages[stage]));product_speed_clear(speed,stage);product_window_clear(windows,stage);}
    if(stage==2)memset(&current.comparison,0,sizeof(current.comparison));
    active_capture=0;
    if(stage<2){product_assessment_reset(assessment,&current);memset(&current.solution,0,sizeof(current.solution));memset(&current.comparison,0,sizeof(current.comparison));
        if(stage==1){memset(&runs[2],0,sizeof(runs[2]));memset(&current.stages[2],0,sizeof(current.stages[2]));product_speed_clear(speed,2);product_window_clear(windows,2);}}
    if(!imu.online||!reference_ok(&ref)){terminal(PRODUCT_NO_RESULT,!imu.online?PRODUCT_SENSOR:PRODUCT_REFERENCE);return;}
    if(stage&&imu.epoch!=runs[0].summary.epoch){terminal(PRODUCT_NO_RESULT,PRODUCT_CONTEXT);return;}
    if(raw.state==RAW_RECORDING||raw.state==RAW_CLOSING){terminal(PRODUCT_NO_RESULT,PRODUCT_BUSY);return;}
    product_job_arm(&job,(uint64_t)esp_timer_get_time(),ref.rpm.accepted_edges,imu.epoch,windows->capture);
    if(job.capture.mode==CAPTURE_TIMED&&!job.capture.seconds){
        if(stage)job.reference_rpm=stage==PRODUCT_RETEST?current.solution.mean_rpm:product_capture_baseline_rpm(&current.stages[0]);
        if(extend_record)job.minimum_seconds=(unsigned)fmin(60,fmax(previous_seconds,product_capture_auto_seconds(job.reference_rpm))+10);
    }
    i2c_at_arm=imu.i2c_errors;fifo_at_arm=imu.fifo_loss_resets;
    current.state=PRODUCT_ARMED;current.reason=PRODUCT_REASON_NONE;current.armed_us=(uint64_t)esp_timer_get_time();current.deadline_us=job.deadline_us;publish();
}
static void worker(void *unused)
{
    (void)unused;product_request_t request;
    for(;;){if(xQueueReceive(commands,&request,pdMS_TO_TICKS(25))==pdTRUE)command(&request);
        if(current.state!=PRODUCT_ARMED&&current.state!=PRODUCT_RECORDING){
            if(save_pending&&(uint64_t)esp_timer_get_time()>=save_retry_us){
                portENTER_CRITICAL(&lock);history[0].storage_ok=true;portEXIT_CRITICAL(&lock);
                store_history();
            }
            continue;
        }
        acquisition_status_t imu;optical_ref_status_t ref;raw_status_t raw;
        acquisition_get_status(&imu);optical_ref_get_status(&ref);raw_capture_get_status(&raw);
        uint64_t now=(uint64_t)esp_timer_get_time();
        const product_job_action_t action=product_job_tick(&job,now,ref.rpm.accepted_edges,imu.epoch,imu.online,
            reference_ok(&ref),imu.i2c_errors!=i2c_at_arm||imu.fifo_loss_resets!=fifo_at_arm,
            ref.rpm.rpm_valid,ref.rpm.rpm);
        if(action==JOB_FAIL){if(current.state==PRODUCT_RECORDING)(void)raw_capture_stop(active_capture);
            if(current.state==PRODUCT_RECORDING&&job.end_reason==CAPTURE_END_LIMIT)closing=true;
            else{terminal(job.state,job.reason);continue;}}
        if(action==JOB_START){const uint32_t duration=job.capture.mode==CAPTURE_TIMED?job.capture.seconds*1000:PRODUCT_CAPTURE_MAX_MS;
            if(raw_capture_begin(duration,RAW_CONTEXT_USER,false)!=ESP_OK){terminal(PRODUCT_NO_RESULT,PRODUCT_BUSY);continue;}
            raw_capture_get_status(&raw);active_capture=raw.id;closing=false;preview_last_us=0;
            product_job_recording(&job,now,ref.rpm.accepted_edges);current.state=PRODUCT_RECORDING;current.deadline_us=job.deadline_us;publish();}
        if(current.state==PRODUCT_RECORDING){
            if(raw.id!=active_capture){terminal(PRODUCT_NO_RESULT,PRODUCT_DATA_FAULT);continue;}
            if(raw.faults||raw.queue_drops){(void)raw_capture_stop(active_capture);terminal(PRODUCT_NO_RESULT,PRODUCT_DATA_FAULT);continue;}
            if(action==JOB_STOP&&!closing){(void)raw_capture_stop(active_capture);closing=true;}
            if(raw.state==RAW_READY){
                if(raw.end_reason==RAW_END_DURATION&&job.capture.mode==CAPTURE_TIMED)job.end_reason=CAPTURE_END_TIMED;
                if((raw.end_reason==RAW_END_DURATION&&job.capture.mode!=CAPTURE_TIMED)||job.end_reason==CAPTURE_END_LIMIT){job.end_reason=CAPTURE_END_LIMIT;
                    terminal(PRODUCT_NO_RESULT,PRODUCT_CAPTURE_LIMIT);continue;}
                if(job.end_reason==CAPTURE_END_UNKNOWN)job.end_reason=CAPTURE_END_OPERATOR;
                current.state=PRODUCT_PROCESSING;current.deadline_us=0;publish();process_capture();}
            else if(raw.state==RAW_RECORDING)preview_capture(now,&raw);
        }
    }
}
bool product_session_submit(const product_request_t *r)
{if(!r||!commands)return false;product_snapshot_t s;product_session_get(&s);
    if(r->id!=s.id||r->revision!=s.revision)return false;
    if(r->action==PRODUCT_DEMO_RETEST&&(!s.simulated||!product_retest_available(&s)))return false;
    if(r->action==PRODUCT_START_RETEST){
        if(s.simulated||!r->trial_removed||!product_retest_available(&s))return false;
        product_retest_condition_t condition=r->retest_condition;
        portENTER_CRITICAL(&lock);
        if(!published_retests.count&&condition==RETEST_CONDITION_NONE)condition=RETEST_CONDITION_FIRST;
        const bool confirmed=product_retest_can_start(&published_retests,condition);
        portEXIT_CRITICAL(&lock);if(!confirmed)return false;}
    return xQueueSend(commands,r,0)==pdTRUE;}
esp_err_t product_session_start(const char *boot)
{
    if(commands||!boot||strlen(boot)!=16)return ESP_ERR_INVALID_STATE;
    replay_samples=heap_caps_calloc(REPLAY_CAPACITY,sizeof(*replay_samples),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    selection=heap_caps_calloc(COAST_SELECTION_MAX_REFS,sizeof(*selection),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    references=heap_caps_calloc(COAST_SELECTION_MAX_REFS,sizeof(*references),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    clock_core=heap_caps_calloc(1,sizeof(*clock_core),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    baseline_axes=heap_caps_calloc(3,sizeof(*baseline_axes),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    trial_axes=heap_caps_calloc(3,sizeof(*trial_axes),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    assessment=heap_caps_calloc(2+PRODUCT_HISTORY,sizeof(*assessment),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    previous_retest=heap_caps_calloc(1,sizeof(*previous_retest),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    runs=heap_caps_calloc(3,sizeof(*runs),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    speed=heap_caps_calloc(2+PRODUCT_HISTORY,sizeof(*speed),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    windows=heap_caps_calloc(2+PRODUCT_HISTORY,sizeof(*windows),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!replay_samples||!selection||!references||!clock_core||!baseline_axes||!trial_axes||!assessment||!previous_retest||!runs||!speed||!windows)return ESP_ERR_NO_MEM;
    published_speed=speed+1;history_speeds=speed+2;published_windows=windows+1;history_windows=windows+2;published_assessment=assessment+1;history_assessments=assessment+2;
    memset(&current,0,sizeof(current));snprintf(current.boot_id,sizeof(current.boot_id),"%s",boot);
    snprintf(current.firmware,sizeof(current.firmware),"%s",esp_app_get_description()->version);
    product_config_t config=product_default_config();fresh(&config,false);
    (void)product_trial_init(&guide,&current,5,true);publish();
    nvs_handle_t handle;
    if(nvs_open("wb_product",NVS_READONLY,&handle)==ESP_OK){
        for(unsigned i=0;i<PRODUCT_HISTORY;++i){char key[8];snprintf(key,sizeof(key),"h%u",i);product_snapshot_t saved;size_t size=sizeof(saved);
            if(nvs_get_blob(handle,key,&saved,&size)==ESP_OK&&size==sizeof(saved)&&saved.schema==1&&
                product_config_valid(&saved.config)&&saved.id&&memchr(saved.boot_id,0,sizeof(saved.boot_id))&&
                memchr(saved.firmware,0,sizeof(saved.firmware))){saved.historical=true;history[history_count]=saved;
                snprintf(key,sizeof(key),"g%u",i);product_trial_guide_t saved_guide;size_t guide_size=sizeof(saved_guide);
                if(nvs_get_blob(handle,key,&saved_guide,&guide_size)==ESP_OK&&product_trial_restore(&saved_guide,&saved,guide_size))
                    history_guides[history_count]=saved_guide;
                snprintf(key,sizeof(key),"r%u",i);product_retest_log_t *saved_retests=&history_retests[history_count];
                size_t retest_size=sizeof(*saved_retests);
                if(nvs_get_blob(handle,key,saved_retests,&retest_size)!=ESP_OK||retest_size!=sizeof(*saved_retests)||
                    !product_retest_valid(saved_retests,&saved))product_retest_import_latest(saved_retests,&saved);
                snprintf(key,sizeof(key),"s%u",i);size_t profile_size=sizeof(history_speeds[history_count]);
                if(nvs_get_blob(handle,key,&history_speeds[history_count],&profile_size)!=ESP_OK||profile_size!=sizeof(history_speeds[history_count])||
                    !product_speed_valid(&history_speeds[history_count],&saved,true))memset(&history_speeds[history_count],0,sizeof(history_speeds[history_count]));
                snprintf(key,sizeof(key),"w%u",i);size_t window_size=sizeof(history_windows[history_count]);
                if(nvs_get_blob(handle,key,&history_windows[history_count],&window_size)!=ESP_OK||window_size!=sizeof(history_windows[history_count])||
                    !product_window_valid(&history_windows[history_count],&saved,true))memset(&history_windows[history_count],0,sizeof(history_windows[history_count]));
                ++history_count;}}
        double piece=5,saved_piece;size_t piece_size=sizeof(saved_piece);
        if(nvs_get_blob(handle,"trial_piece_g",&saved_piece,&piece_size)==ESP_OK&&piece_size==sizeof(saved_piece)&&
           isfinite(saved_piece)&&saved_piece>=.1&&saved_piece<=PRODUCT_GUIDED_TRIAL_MAX_G)piece=saved_piece;
        product_config_t saved_config;size_t config_size=sizeof(saved_config);
        if(nvs_get_blob(handle,"config",&saved_config,&config_size)==ESP_OK && config_size==sizeof(saved_config)&&product_config_valid(&saved_config)){
            current.config=saved_config;real_config=saved_config;
            (void)product_trial_init(&guide,&current,piece,true);publish();}
        product_capture_config_t saved_capture;size_t capture_size=sizeof(saved_capture);
        if(nvs_get_blob(handle,"capture_config",&saved_capture,&capture_size)==ESP_OK&&capture_size==sizeof(saved_capture)&&product_capture_config_valid(&saved_capture)){
            product_capture_apply_preset(&saved_capture);
            saved_capture.seconds=0;
            real_capture_config=saved_capture;windows->capture=saved_capture;publish();}
        nvs_close(handle);}
    if(nvs_open_from_partition("wb_meta","assessment",NVS_READONLY,&handle)==ESP_OK){
        for(unsigned i=0;i<history_count;++i){char key[16];snprintf(key,sizeof(key),"q%u",i);size_t bytes=sizeof(history_assessments[i]);
            if(nvs_get_blob(handle,key,&history_assessments[i],&bytes)!=ESP_OK||bytes!=sizeof(history_assessments[i])||
                !product_assessment_valid(&history_assessments[i],&history[i]))memset(&history_assessments[i],0,sizeof(history_assessments[i]));}
        nvs_close(handle);}
    commands=xQueueCreate(4,sizeof(product_request_t));if(!commands)return ESP_ERR_NO_MEM;
    if(xTaskCreatePinnedToCore(worker,"product_session",32768,NULL,2,NULL,1)!=pdPASS)return ESP_ERR_NO_MEM;
    return ESP_OK;
}
