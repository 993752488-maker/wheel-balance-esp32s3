#include "product_http.h"
#include "product_session.h"
#include "raw_capture.h"
#include "acquisition.h"
#include "optical_ref.h"
#include "capture_export.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char boot_id[17];
/* Sole HTTP task owns these copies; neither sampling nor the async CSV worker
 * accesses them. Their lifetime is the server lifetime, with no per-poll heap
 * fragmentation and no change to the persisted snapshot format. */
static struct {
    product_snapshot_t snapshot,stored[PRODUCT_HISTORY];
    product_trial_guide_t trial_view,stored_trials[PRODUCT_HISTORY];
    product_retest_log_t retest_view,stored_retests[PRODUCT_HISTORY];
    product_speed_t speed_view,stored_speeds[PRODUCT_HISTORY];
    product_window_t window_view,stored_windows[PRODUCT_HISTORY];
    product_assessment_t assessment_view,stored_assessments[PRODUCT_HISTORY];
} *http_view;
#define snapshot (http_view->snapshot)
#define stored (http_view->stored)
#define trial_view (http_view->trial_view)
#define stored_trials (http_view->stored_trials)
#define retest_view (http_view->retest_view)
#define stored_retests (http_view->stored_retests)
#define speed_view (http_view->speed_view)
#define stored_speeds (http_view->stored_speeds)
#define window_view (http_view->window_view)
#define stored_windows (http_view->stored_windows)
#define assessment_view (http_view->assessment_view)
#define stored_assessments (http_view->stored_assessments)
static bool product_protocol(const char *version,unsigned first_patch)
{
    unsigned major,minor,patch;char suffix;
    return sscanf(version,"%u.%u.%u%c",&major,&minor,&patch,&suffix)==3&&
        (major>0||minor>5||(minor==5&&patch>=first_patch));
}
static bool product_06_protocol(const char *version,unsigned first_patch)
{
    unsigned major,minor,patch;char suffix;
    return sscanf(version,"%u.%u.%u%c",&major,&minor,&patch,&suffix)==3&&
        (major>0||minor>6||(minor==6&&patch>=first_patch));
}
static void number(cJSON *o,const char *k,double v,bool valid)
{if(valid&&isfinite(v))cJSON_AddNumberToObject(o,k,v);else cJSON_AddNullToObject(o,k);}
static bool assessment_protocol(const char *version)
{
    return product_06_protocol(version,2);
}
static const char *axis_policy(const product_snapshot_t *s,const product_trial_guide_t *trial,const product_assessment_t *quality)
{
    if(!trial->automatic_axis)return "operator_selected";
    if(!assessment_protocol(s->firmware))return "baseline_only_median_band_coherence_v1_frozen_before_trial";
    if(product_06_protocol(s->firmware,6)){
        if(s->solution.reason==PRODUCT_CONFLICT)return "no_axis_guidance_due_to_conflict";
        if(s->solution.calculation_valid)return product_06_protocol(s->firmware,7)?
            "trial_pairwise_axes_v5_frozen_on_usable_result":"trial_consistent_axes_v4_frozen_on_usable_result";
        if(trial->count)return "reselect_after_failed_trial";
    }
    if(!product_assessment_valid(quality,s)||!quality->axis_finalized)return "baseline_provisional_before_trial_response";
    return quality->policy>=2?"trial_compatible_support_v3_frozen":"trial_response_quality_v2_frozen_after_first_completed_trial";
}
static void serialize_assessment(cJSON *parent,const product_snapshot_t *s,const product_assessment_t *q,bool detailed)
{
    const bool valid=product_assessment_valid(q,s),available=valid&&q->valid_bands;
    cJSON *o=cJSON_AddObjectToObject(parent,"assessment");cJSON_AddBoolToObject(o,"available",available);
    cJSON_AddStringToObject(o,"state",available?product_assessment_state_name((product_assessment_state_t)q->state):
        s->solution.calculation_valid?(assessment_protocol(s->firmware)?"ASSESSMENT_MISSING":"LEGACY_NOT_ASSESSED"):"NOT_ASSESSED");
    cJSON_AddStringToObject(o,"policy",!valid?"not_available":q->policy>=4?"pairwise_consensus_normalized_fit_v4":q->policy==3?"compatible_axes_normalized_fit_v3":
        q->policy==2?"compatible_support_normalized_complex_fit_v2":"response_quality_split_review_consistent_band_complex_fit_v1");
    cJSON_AddBoolToObject(o,"accuracy_interval",false);cJSON_AddBoolToObject(o,"physical_repeatability_verified",false);
    if(!valid)return;
    cJSON_AddNumberToObject(o,"schema",q->schema);cJSON_AddNumberToObject(o,"snapshot_revision",q->snapshot_revision);
    cJSON_AddBoolToObject(o,"axis_finalized",q->axis_finalized);cJSON_AddNumberToObject(o,"axis",q->axis);
    cJSON_AddNumberToObject(o,"warning_flags",q->flags);cJSON_AddNumberToObject(o,"valid_bands",q->valid_bands);
    cJSON_AddNumberToObject(o,"joint_bands",q->joint_bands);cJSON_AddNumberToObject(o,"joint_band_mask",q->joint_band_mask);
    cJSON_AddBoolToObject(o,"joint_used",q->joint_used);
    number(o,"observed_mass_min_g",q->mass_min_g,available);number(o,"observed_mass_max_g",q->mass_max_g,available);
    number(o,"max_relative_vector_difference",q->max_relative_difference,available);
    number(o,"max_phase_difference_deg",q->max_phase_difference,available);
    cJSON_AddStringToObject(o,"range_definition","range_of_individual_eligible_band_estimates_not_accuracy_or_confidence");
    if(!detailed)return;
    cJSON_AddStringToObject(o,"fit_formula",q->policy>=2?"Uc=sum(score_k^2*(-V0_k/H_k))/sum(score_k^2)":"Uc=-sum(w*conj(H_k)*V0_k)/sum(w*abs(H_k)^2); w=empirical_quality_score^2");
    cJSON_AddStringToObject(o,"fit_neighbourhood",q->policy>=4?"all_pairs_vector035_phase30_shared_support080_of_best_group":
        q->policy>=2?"compatible_group_max_sum_squared_scores_vector035_phase30_rival080":"relative_complex_difference_le035_phase_le30deg_from_quality_anchor_not_validation_target");
    number(o,"model_residual_ratio",q->model_residual_ratio,available&&q->joint_used);
    cJSON_AddStringToObject(o,"model_residual_definition","calibration_model_residual_not_measured_improvement");
    number(o,"correction_real_g_mm",q->correction_real_g_mm,available);number(o,"correction_imag_g_mm",q->correction_imag_g_mm,available);
    cJSON *axes=cJSON_AddArrayToObject(o,"axes");for(unsigned i=0;i<3;++i){cJSON *a=cJSON_CreateObject();cJSON_AddItemToArray(axes,a);
        const bool present=(q->axis_available_mask&(1U<<i))!=0;cJSON_AddNumberToObject(a,"axis",i);cJSON_AddBoolToObject(a,"calculation_available",present);
        if(q->policy>=3)cJSON_AddBoolToObject(a,"band_conflict",(q->axis_conflict_mask&(1U<<i))!=0);
        number(a,"response_score",q->axis_scores[i],present);number(a,"mass_g",q->axis_mass_g[i],present);number(a,"angle_deg",q->axis_angle_deg[i],present);}
    cJSON *bands=cJSON_AddArrayToObject(o,"bands");for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){const product_assessment_band_t *v=&q->bands[i];if(!v->pairs)continue;
        cJSON *b=cJSON_CreateObject();cJSON_AddItemToArray(bands,b);cJSON_AddNumberToObject(b,"lower_rpm",v->lower_rpm);cJSON_AddNumberToObject(b,"upper_rpm",v->lower_rpm+20);
        cJSON_AddNumberToObject(b,"matched_turns",v->pairs);cJSON_AddBoolToObject(b,"available",v->available);cJSON_AddStringToObject(b,"reason",product_reason_name(v->reason));
        cJSON_AddBoolToObject(b,"used_for_joint_fit",(q->joint_band_mask&(1U<<i))!=0);
        if(q->policy>=2 && q->joint_band_mask){
            double total=0;
            for(unsigned k=0;k<PRODUCT_ASSESSMENT_BANDS;++k)
                if(q->joint_band_mask&(1U<<k))total+=q->bands[k].score*q->bands[k].score;
            number(b,"fit_share",(q->joint_band_mask&(1U<<i)) && total>0?v->score*v->score/total:0,total>0);
        }
        number(b,"mass_g",v->mass_g,v->available);number(b,"angle_deg",v->angle_deg,v->available);number(b,"score",v->score,true);
        number(b,"h_real",v->influence_real,v->available);number(b,"h_imag",v->influence_imag,v->available);
        number(b,"v0_real",v->original_real,v->available);number(b,"v0_imag",v->original_imag,v->available);
        cJSON_AddBoolToObject(b,"split_available",v->split_available);cJSON_AddBoolToObject(b,"split_valid",v->split_valid);
        number(b,"split_relative_vector_difference",v->split_relative_difference,v->split_valid);
        number(b,"split_phase_difference_deg",v->split_phase_difference,v->split_valid);}
}
static void serialize_retests(cJSON *o,const product_snapshot_t *s,const product_retest_log_t *log)
{
    cJSON *h=cJSON_AddObjectToObject(o,"retest_history");
    const bool valid=product_retest_valid(log,s);
    cJSON_AddBoolToObject(h,"available",valid);cJSON_AddNumberToObject(h,"schema_version",1);
    cJSON_AddNumberToObject(h,"capacity",PRODUCT_RETEST_HISTORY);cJSON_AddNumberToObject(h,"snapshot_revision",s->revision);
    cJSON_AddStringToObject(h,"retention","latest6_summaries_latest_raw_only");
    cJSON_AddStringToObject(h,"comparison_policy",product_06_protocol(s->firmware,6)?
        "chronological_max_pairs_in_frozen_band_current_and_previous_RPM_3pct":"frozen_calibration_band_current_and_previous_RPM_3pct");
    cJSON_AddStringToObject(h,"warning_policy","prototype_only_magnitude_ratio_over1.25_or_coherent_phase_over30deg_not_accuracy");
    cJSON_AddBoolToObject(h,"accuracy_validated",false);
    cJSON_AddNumberToObject(h,"retained_count",valid?log->count:0);
    cJSON_AddBoolToObject(h,"legacy_count_unknown",valid&&log->legacy_count_unknown);
    cJSON_AddBoolToObject(h,"group_variable_observed",valid&&log->current_group_variable);
    number(h,"total_count",valid?log->total_count:0,valid&&!log->legacy_count_unknown);
    cJSON_AddNumberToObject(h,"dropped_count",valid?log->dropped_count:0);
    cJSON *rows=cJSON_AddArrayToObject(h,"entries");
    if(!valid)return;
    for(unsigned i=0;i<log->count;++i){const product_retest_entry_t *v=&log->entries[i];
        cJSON *e=cJSON_CreateObject();cJSON_AddItemToArray(rows,e);
        number(e,"ordinal",v->ordinal,!v->legacy);cJSON_AddBoolToObject(e,"legacy",v->legacy);
        cJSON_AddNumberToObject(e,"correction_group",v->correction_group);
        cJSON_AddStringToObject(e,"condition",product_retest_condition_name(v->condition));
        cJSON_AddNumberToObject(e,"capture_id",v->stage.capture_id);cJSON_AddNumberToObject(e,"epoch",v->stage.epoch);
        cJSON_AddNumberToObject(e,"start_us",(double)v->stage.start_us);number(e,"end_us",(double)v->stage.end_us,v->stage.end_us>0);
        cJSON_AddNumberToObject(e,"samples",v->stage.samples);cJSON_AddNumberToObject(e,"references",v->stage.references);
        cJSON_AddBoolToObject(e,"data_integrity_valid",v->stage.integrity_valid);cJSON_AddBoolToObject(e,"captured",v->stage.captured);
        cJSON_AddBoolToObject(e,"comparison_available",v->comparison.available);
        cJSON_AddStringToObject(e,"reason",product_reason_name(v->stage.reason));
        cJSON_AddStringToObject(e,"comparison_reason",product_reason_name(v->comparison.reason));
        cJSON_AddNumberToObject(e,"matched_turns",v->comparison.matched_turns);
        number(e,"improvement_percent",v->comparison.improvement_percent,v->comparison.available);
        number(e,"original_peak_mps2",v->comparison.original_peak,v->comparison.available);
        number(e,"retest_peak_mps2",v->comparison.retest_peak,v->comparison.available);
        number(e,"lower_rpm",v->comparison.lower_rpm,v->comparison.matched_turns>0);
        number(e,"upper_rpm",v->comparison.upper_rpm,v->comparison.matched_turns>0);
        cJSON *r=cJSON_AddObjectToObject(e,"repeat");const product_repeat_observation_t *p=&v->repeat;
        const bool compared=p->state>=PRODUCT_REPEAT_OBSERVATION_CLOSE;
        cJSON_AddStringToObject(r,"state",product_repeat_state_name(p->state));
        cJSON_AddNumberToObject(r,"previous_capture_id",p->previous_capture_id);cJSON_AddNumberToObject(r,"matched_turns",p->matched_turns);
        cJSON_AddStringToObject(r,"reason",product_reason_name(p->reason));
        number(r,"amplitude_change_percent",p->amplitude_change_percent,compared);
        number(r,"previous_peak_mps2",p->previous_peak,compared);number(r,"current_peak_mps2",p->current_peak,compared);
        number(r,"phase_change_deg",p->phase_change_deg,compared&&p->phase_available);
    }
}
static void serialize_speed(cJSON *parent,const product_snapshot_t *s,const product_speed_t *v)
{
    cJSON *o=cJSON_AddObjectToObject(parent,"speed_profile");
    const bool valid=product_speed_valid(v,s,s->historical);
    cJSON_AddNumberToObject(o,"schema_version",1);cJSON_AddBoolToObject(o,"available",valid);
    cJSON_AddBoolToObject(o,"display_only",true);cJSON_AddBoolToObject(o,"accuracy_validated",false);
    cJSON_AddStringToObject(o,"metric","RMS_of_per_turn_1x_peak_magnitudes_mps2");
    cJSON_AddStringToObject(o,"spread","per_turn_min_max_not_noise_or_confidence");
    cJSON_AddNumberToObject(o,"bin_width_rpm",PRODUCT_SPEED_WIDTH);
    cJSON_AddStringToObject(o,"reason",valid?"NONE":"NOT_RECORDED_OR_CONTEXT_MISMATCH");
    if(!valid)return;
    cJSON_AddNumberToObject(o,"session_id",v->session_id);cJSON_AddNumberToObject(o,"snapshot_revision",v->snapshot_revision);
    cJSON_AddNumberToObject(o,"generation",v->generation);cJSON_AddNumberToObject(o,"axis",v->axis);
    cJSON_AddStringToObject(o,"boot_id",v->boot_id);cJSON_AddStringToObject(o,"source",v->simulated?"synthetic":"device");
    cJSON *stages=cJSON_AddArrayToObject(o,"stages");const char *names[]={"empty","preview","frozen","invalid"};
    for(unsigned stage=0;stage<3;++stage){const product_speed_stage_t *a=&v->stages[stage];
        cJSON *p=cJSON_CreateObject();cJSON_AddItemToArray(stages,p);
        cJSON_AddStringToObject(p,"state",names[a->mode]);cJSON_AddNumberToObject(p,"capture_id",a->capture_id);
        cJSON_AddNumberToObject(p,"epoch",a->epoch);cJSON_AddNumberToObject(p,"start_us",(double)a->start_us);
        cJSON_AddNumberToObject(p,"end_us",(double)a->end_us);cJSON_AddNumberToObject(p,"updated_us",(double)a->updated_us);
        cJSON_AddNumberToObject(p,"used_turns",a->used_turns);cJSON_AddNumberToObject(p,"rejected_turns",a->rejected_turns);
        cJSON_AddNumberToObject(p,"outside_speed_turns",a->outside_turns);
        cJSON *points=cJSON_AddArrayToObject(p,"points");
        for(unsigned i=0;i<PRODUCT_SPEED_BINS;++i){const product_speed_bin_t *b=&a->bins[i];if(!b->count)continue;
            cJSON *row=cJSON_CreateObject();cJSON_AddItemToArray(points,row);
            cJSON_AddNumberToObject(row,"lower_rpm",(i+1)*PRODUCT_SPEED_WIDTH);
            cJSON_AddNumberToObject(row,"upper_rpm",(i+2)*PRODUCT_SPEED_WIDTH);
            cJSON_AddNumberToObject(row,"count",b->count);cJSON_AddNumberToObject(row,"rms",b->rms);
            cJSON_AddNumberToObject(row,"low",b->minimum);cJSON_AddNumberToObject(row,"high",b->maximum);
        }
    }
}
static void serialize_window(cJSON *parent,const product_snapshot_t *s,const product_window_t *w)
{
    cJSON *o=cJSON_AddObjectToObject(parent,"capture_flow");const bool valid=product_window_valid(w,s,s->historical);
    const bool whole=valid&&w->schema==2;
    cJSON_AddBoolToObject(o,"available",valid);cJSON_AddNumberToObject(o,"schema_version",valid?w->schema:0);
    cJSON_AddBoolToObject(o,"hand_release_confirmed",false);cJSON_AddBoolToObject(o,"motor_state_verified",false);
    cJSON_AddStringToObject(o,"policy",!valid?"unavailable":whole?"whole_record_settling_then20RPM_stratified_time_coverage96_v2":"per_axis_3_structural_turns_1500ms_after_detected_disturbance_recent64_v1");
    cJSON_AddStringToObject(o,"steady_policy","5_turns_bounded3pct_alternating_ripple_slope0.3pct_not_drive_state_proof");
    cJSON_AddStringToObject(o,"stop_policy",!valid?"unavailable":whole?
        (w->capture.mode==CAPTURE_TIMED?"fixed_record_duration_or_explicit_operator":"reference_silence8to12seconds_or_explicit_operator_no_count_target_120second_limit_no_result"):
        "12_structural_turns_6seconds_3_common_pairs_or_stop_or60seconds");
    if(valid&&!whole)cJSON_AddNumberToObject(o,"target_turns",12);
    cJSON_AddBoolToObject(o,"stop_observation_required",whole&&w->capture.mode==CAPTURE_COAST);
    cJSON_AddNumberToObject(o,"max_record_seconds",whole?(w->capture.mode==CAPTURE_TIMED?w->capture.seconds:PRODUCT_CAPTURE_MAX_MS/1000):60);
    cJSON_AddNumberToObject(o,"retained_turn_limit",whole?PRODUCT_WINDOW_KEEP:64);
    if(whole){const char *types[]={"bicycle","electric","other"};
        cJSON_AddStringToObject(o,"vehicle_type",types[w->capture.vehicle_type]);
        cJSON_AddStringToObject(o,"mode",w->capture.mode==CAPTURE_TIMED?"timed":"coast");
        cJSON_AddNumberToObject(o,"duration_seconds",w->capture.seconds);
        cJSON_AddStringToObject(o,"pairing_policy",product_06_protocol(s->firmware,6)?
            "chronological_max_pairs_3pct_original_band":"common_RPM_tail_first_then_chronological_complex_average");}
    if(!valid)return;
    cJSON_AddNumberToObject(o,"session_id",w->session_id);cJSON_AddNumberToObject(o,"snapshot_revision",w->snapshot_revision);
    cJSON_AddNumberToObject(o,"generation",w->generation);cJSON_AddStringToObject(o,"boot_id",w->boot_id);
    cJSON_AddStringToObject(o,"source",w->simulated?"synthetic":"device");
    const char *phases[]={"waiting","settling","collecting","complete"},*modes[]={"unknown","steady","coast","mixed"};
    cJSON *rows=cJSON_AddArrayToObject(o,"stages");
    for(unsigned i=0;i<3;++i){const product_window_stage_t *v=&w->stages[i];cJSON *r=cJSON_CreateObject();cJSON_AddItemToArray(rows,r);
        unsigned axis=s->config.axis;if(i==0&&!s->stages[0].captured)for(unsigned a=0;a<3;++a)if(v->usable[a]>v->usable[axis])axis=a;
        unsigned phase=v->phase;if(!whole&&phase==WINDOW_COLLECTING&&!(v->collecting_mask&(1U<<axis)))phase=WINDOW_SETTLING;
        cJSON_AddStringToObject(r,"phase",phases[phase]);cJSON_AddStringToObject(r,"motion",modes[v->mode]);
        const char *ends[]={"not_recorded","reference_silence","operator_finish","capture_limit","timed_duration"};
        cJSON_AddStringToObject(r,"end_reason",whole?ends[v->end_reason]:"not_recorded");
        cJSON_AddNumberToObject(r,"capture_id",v->capture_id);cJSON_AddNumberToObject(r,"epoch",v->epoch);
        cJSON_AddNumberToObject(r,"start_us",(double)v->start_us);cJSON_AddNumberToObject(r,"end_us",(double)v->end_us);
        cJSON_AddNumberToObject(r,"updated_us",(double)v->updated_us);cJSON_AddNumberToObject(r,"total_turns",v->total_turns);
        cJSON_AddNumberToObject(r,"accelerating_turns",v->accelerating_turns);cJSON_AddNumberToObject(r,"progress_axis",axis);
        cJSON_AddNumberToObject(r,"usable_turns",v->usable[axis]);cJSON_AddNumberToObject(r,"skipped_turns",v->skipped[axis]);
        cJSON_AddNumberToObject(r,"transient_turns",v->transient[axis]);
        cJSON_AddNumberToObject(r,"common_pairs",v->common_pairs);
        cJSON_AddNumberToObject(r,"first_usable_us",(double)v->first_usable_us[axis]);cJSON_AddNumberToObject(r,"last_usable_us",(double)v->last_usable_us[axis]);
    }
}
static cJSON *serialize(const product_snapshot_t *s,const product_trial_guide_t *trial,const product_retest_log_t *log,const product_speed_t *profile,const product_window_t *window,const product_assessment_t *quality,bool detailed)
{
    cJSON *o=cJSON_CreateObject();if(!o)return NULL;
    cJSON_AddNumberToObject(o,"id",s->id);cJSON_AddNumberToObject(o,"revision",s->revision);
    cJSON_AddStringToObject(o,"boot_id",s->boot_id);cJSON_AddStringToObject(o,"firmware",s->firmware);
    cJSON_AddStringToObject(o,"source",s->simulated?"synthetic":"device");
    cJSON_AddBoolToObject(o,"historical",s->historical);cJSON_AddBoolToObject(o,"storage_ok",s->storage_ok);
    cJSON_AddStringToObject(o,"state",product_state_name(s->state));cJSON_AddNumberToObject(o,"stage",s->stage);
    cJSON_AddStringToObject(o,"reason",product_reason_name(s->reason));
    cJSON_AddBoolToObject(o,"accuracy_validated",false);cJSON_AddBoolToObject(o,"recommendation_valid",false);
    cJSON_AddStringToObject(o,"angle_convention","white_mark_zero_along_rotation_same_view");
    cJSON_AddBoolToObject(o,"trial_removal_required",true);
    cJSON_AddStringToObject(o,"retest_weight_confirmation",!s->simulated&&s->stages[2].capture_id&&product_protocol(s->firmware,1)?"operator_confirmed_trial_removed":"not_recorded");
    cJSON_AddNumberToObject(o,"armed_us",(double)s->armed_us);cJSON_AddNumberToObject(o,"deadline_us",(double)s->deadline_us);
    cJSON *config=cJSON_AddObjectToObject(o,"config");
    cJSON_AddStringToObject(config,"vehicle",s->config.vehicle);cJSON_AddNumberToObject(config,"trial_mass_g",s->config.mass_g);
    cJSON_AddNumberToObject(config,"trial_radius_mm",s->config.trial_radius_mm);cJSON_AddNumberToObject(config,"trial_angle_deg",s->config.trial_angle_deg);
    cJSON_AddNumberToObject(config,"correction_radius_mm",s->config.correction_radius_mm);cJSON_AddNumberToObject(config,"axis",s->config.axis);
    cJSON_AddStringToObject(config,"direction",s->config.direction==1?"cw":"ccw");
    cJSON *guide=cJSON_AddObjectToObject(o,"trial_guide");
    const bool guide_ok=product_trial_valid(trial,s);
    cJSON_AddNumberToObject(guide,"schema_version",guide_ok?trial->schema:0);cJSON_AddBoolToObject(guide,"available",guide_ok);
    if(guide_ok){product_trial_advice_t advice;product_trial_advice(trial,s,&advice);
        cJSON_AddStringToObject(guide,"policy","baseline_first_two_pieces_max3_attempts_projection2x_variation_bounded_not_safety_limit");
        cJSON_AddNumberToObject(guide,"snapshot_revision",trial->snapshot_revision);
        cJSON *axis=cJSON_AddObjectToObject(guide,"axis_selection");
        cJSON_AddStringToObject(axis,"mode",trial->automatic_axis?"auto":"manual");
        cJSON_AddBoolToObject(axis,"resolved",trial->axis_resolved&&s->stages[0].captured);
        number(axis,"selected_axis",s->config.axis,trial->axis_resolved&&s->stages[0].captured);
        cJSON_AddStringToObject(axis,"policy",axis_policy(s,trial,quality));
        cJSON *scores=cJSON_AddArrayToObject(axis,"baseline_scores");
        for(unsigned i=0;i<3;++i){if(trial->automatic_axis&&trial->axis_resolved)cJSON_AddItemToArray(scores,cJSON_CreateNumber(trial->axis_score[i]));else cJSON_AddItemToArray(scores,cJSON_CreateNull());}
        cJSON_AddStringToObject(guide,"plan_source",product_trial_source_name(trial->source));
        cJSON_AddNumberToObject(guide,"piece_mass_g",trial->piece_mass_g);cJSON_AddNumberToObject(guide,"planned_mass_g",s->config.mass_g);
        cJSON_AddNumberToObject(guide,"initial_mass_g",trial->initial_mass_g);
        cJSON_AddNumberToObject(guide,"attempt_count",trial->count);cJSON_AddNumberToObject(guide,"max_attempts",PRODUCT_TRIAL_ATTEMPTS);
        cJSON_AddBoolToObject(guide,"editable",advice.editable);cJSON_AddBoolToObject(guide,"retry_available",advice.retry_available);
        cJSON_AddStringToObject(guide,"next_reason",product_trial_next_name(advice.reason));number(guide,"suggested_mass_g",advice.mass_g,advice.suggested);
        cJSON_AddStringToObject(guide,"mass_confirmation","operator_confirms_total_added_trial_mass_before_each_capture");
        cJSON *attempts=cJSON_AddArrayToObject(guide,"attempts");
        for(unsigned i=0;i<trial->count;++i){const product_trial_attempt_t *a=&trial->attempts[i];cJSON *t=cJSON_CreateObject();cJSON_AddItemToArray(attempts,t);
            cJSON_AddNumberToObject(t,"index",i+1);cJSON_AddNumberToObject(t,"mass_g",a->mass_g);
            number(t,"axis",a->axis,trial->schema>=3);
            cJSON_AddNumberToObject(t,"radius_mm",s->config.trial_radius_mm);cJSON_AddNumberToObject(t,"angle_deg",s->config.trial_angle_deg);
            cJSON_AddStringToObject(t,"plan_source",product_trial_source_name(a->source));cJSON_AddBoolToObject(t,"operator_confirmed_installed",true);
            cJSON_AddNumberToObject(t,"capture_id",a->stage.capture_id);cJSON_AddNumberToObject(t,"epoch",a->stage.epoch);
            cJSON_AddNumberToObject(t,"baseline_capture_id",s->stages[0].capture_id);
            cJSON_AddNumberToObject(t,"start_us",(double)a->stage.start_us);cJSON_AddNumberToObject(t,"end_us",(double)a->stage.end_us);
            cJSON_AddNumberToObject(t,"samples",a->stage.samples);cJSON_AddNumberToObject(t,"references",a->stage.references);
            cJSON_AddNumberToObject(t,"usable_turns",a->stage.usable_turns);cJSON_AddBoolToObject(t,"data_integrity_valid",a->stage.integrity_valid);
            cJSON_AddBoolToObject(t,"captured",a->stage.captured);cJSON_AddStringToObject(t,"reason",product_reason_name(a->stage.reason));
            number(t,"real",a->stage.real,a->stage.captured);number(t,"imag",a->stage.imag,a->stage.captured);
            cJSON_AddBoolToObject(t,"calculation_valid",a->solution.calculation_valid);
            cJSON_AddNumberToObject(t,"matched_turns",a->solution.matched_turns);
            number(t,"lower_rpm",a->solution.lower_rpm,a->solution.matched_turns>0);number(t,"upper_rpm",a->solution.upper_rpm,a->solution.matched_turns>0);
            number(t,"delta_peak",a->solution.delta_peak,a->solution.matched_turns>0);
            number(t,"delta_coherence",a->solution.delta_coherence,a->solution.matched_turns>0);
            number(t,"mean_variation_scale",a->solution.mean_noise_scale,a->solution.matched_turns>0);}
    }
    cJSON *stages=cJSON_AddArrayToObject(o,"stages");
    for(unsigned i=0;i<3;++i){const product_stage_summary_t *v=&s->stages[i];cJSON *t=cJSON_CreateObject();cJSON_AddItemToArray(stages,t);
        cJSON_AddBoolToObject(t,"captured",v->captured);cJSON_AddBoolToObject(t,"data_integrity_valid",v->integrity_valid);
        cJSON_AddStringToObject(t,"reason",product_reason_name(v->reason));cJSON_AddNumberToObject(t,"capture_id",v->capture_id);
        cJSON_AddNumberToObject(t,"epoch",v->epoch);cJSON_AddNumberToObject(t,"samples",v->samples);cJSON_AddNumberToObject(t,"references",v->references);
        cJSON_AddNumberToObject(t,"turn_count",v->turn_count);cJSON_AddNumberToObject(t,"usable_turns",v->usable_turns);
        cJSON_AddNumberToObject(t,"rejected_turns",v->rejected_turns);cJSON_AddNumberToObject(t,"start_us",(double)v->start_us);cJSON_AddNumberToObject(t,"end_us",(double)v->end_us);
        number(t,"min_rpm",v->min_rpm,v->captured);number(t,"max_rpm",v->max_rpm,v->captured);
        number(t,"real",v->real,v->captured);number(t,"imag",v->imag,v->captured);number(t,"peak_mps2",v->peak,v->captured);
        number(t,"vector_scatter",v->scatter,v->captured);number(t,"direction_coherence",v->coherence,v->captured);
    }
    const bool reviewed=product_assessment_valid(quality,s)&&quality->valid_bands;
    const bool guide_allowed=!assessment_protocol(s->firmware)||reviewed;
    const product_solution_t *v=&s->solution;cJSON *r=cJSON_AddObjectToObject(o,"result");
    cJSON_AddBoolToObject(r,"calculation_valid",v->calculation_valid);cJSON_AddBoolToObject(r,"engineering_available",v->engineering_available&&guide_allowed);
    cJSON_AddBoolToObject(r,"can_start_retest",guide_allowed&&product_retest_available(s));
    cJSON_AddBoolToObject(r,"no_correction_required",guide_allowed&&v->calculation_valid&&v->reason==PRODUCT_REASON_NONE&&v->calculation.mass_g==0);
    cJSON_AddBoolToObject(r,"accuracy_validated",false);cJSON_AddStringToObject(r,"reason",product_reason_name(v->reason));
    cJSON_AddNumberToObject(r,"matched_turns",v->matched_turns);number(r,"lower_rpm",v->lower_rpm,v->matched_turns>0);
    number(r,"upper_rpm",v->upper_rpm,v->matched_turns>0);number(r,"mean_rpm",v->mean_rpm,v->calculation_valid);
    number(r,"mass_g",v->calculation.mass_g,v->calculation_valid);number(r,"angle_deg",v->calculation.angle_deg,v->engineering_available&&guide_allowed);
    number(r,"h_real",v->calculation.influence.real,v->calculation_valid);number(r,"h_imag",v->calculation.influence.imag,v->calculation_valid);
    number(r,"delta_peak",v->delta_peak,v->matched_turns>0);number(r,"delta_coherence",v->delta_coherence,v->matched_turns>0);
    number(r,"mean_variation_scale",v->mean_noise_scale,v->matched_turns>0);
    cJSON_AddStringToObject(r,"estimator",reviewed&&quality->joint_used?(quality->policy>=2?"response_normalized_multispeed_complex_fit":"consistent_multispeed_complex_fit"):"single_band_complex_influence_ratio");
    cJSON_AddStringToObject(r,"h_definition",reviewed&&quality->joint_used?"measured_reference_band_H_not_an_aggregate_coefficient":"selected_band_influence");
    serialize_assessment(r,s,quality,detailed);
    cJSON_AddStringToObject(r,"quality_policy","prototype_v1_min3_pairs_3pct_profile_20rpm_band_coherence065_response15x_scatter_scale");
    cJSON_AddStringToObject(r,"speed_selection_policy",product_06_protocol(s->firmware,7)?"max_chronological_pairs_then_pairwise_consensus":product_06_protocol(s->firmware,6)?"max_chronological_pairs_then_compatible_support":
        reviewed&&quality->policy==2?"compatible_support_normalized_fit":assessment_protocol(s->firmware)?"response_quality_anchor_consistency_bounded_multispeed_fit":product_06_protocol(s->firmware,1)?
        "eligible_bands_tail_first_then_pairs_low_rpm_tie":product_protocol(s->firmware,4)?
        "eligible_bands_then_most_pairs_low_rpm_tie":"single_most_pairs_band_low_rpm_tie");
    const product_comparison_t *b=&s->comparison;cJSON *c=cJSON_AddObjectToObject(o,"comparison");
    cJSON_AddBoolToObject(c,"available",b->available);cJSON_AddStringToObject(c,"reason",product_reason_name(b->reason));
    cJSON_AddNumberToObject(c,"matched_turns",b->matched_turns);number(c,"improvement_percent",b->improvement_percent,b->available);
    number(c,"original_peak_mps2",b->original_peak,b->available);number(c,"retest_peak_mps2",b->retest_peak,b->available);
    cJSON_AddStringToObject(c,"amplitude_definition","RMS_of_per_turn_1x_peak_magnitudes_on_matched_RPM_mps2");
    cJSON_AddStringToObject(c,"phase_estimate_quality",!b->available?"UNAVAILABLE":!product_protocol(s->firmware,3)?"NOT_RECORDED":
        b->reason==PRODUCT_LOW_DIRECTION_COHERENCE?"WEAK_OR_INCOHERENT":"WITHIN_PROTOTYPE_COHERENCE_RULE");
    cJSON_AddBoolToObject(c,"correction_angle_validated",false);
    cJSON_AddStringToObject(c,"interpretation","measured_axis_1x_indicator_only_not_static_balance_or_total_vehicle_vibration");
    serialize_retests(o,s,log);
    serialize_speed(o,s,profile);
    serialize_window(o,s,window);
    return o;
}
static esp_err_t reply(httpd_req_t *req,bool queued,bool report)
{
    product_session_get_assessment_view(&snapshot,&trial_view,&retest_view,&speed_view,&window_view,&assessment_view);const uint32_t current_id=snapshot.id;
    size_t count=product_session_history_assessment_views(stored,stored_trials,stored_retests,stored_speeds,stored_windows,stored_assessments,PRODUCT_HISTORY);
    char query[64],text[16];
    if(httpd_req_get_url_query_str(req,query,sizeof(query))==ESP_OK &&
       httpd_query_key_value(query,"id",text,sizeof(text))==ESP_OK){char *end=NULL;unsigned long id=strtoul(text,&end,10);
        bool found=id==snapshot.id&&*text&&!(*end);
        if(!found)for(size_t i=0;i<count;++i)if(id==stored[i].id && *text && !(*end)){snapshot=stored[i];trial_view=stored_trials[i];retest_view=stored_retests[i];speed_view=stored_speeds[i];window_view=stored_windows[i];assessment_view=stored_assessments[i];found=true;break;}
        if(!found)return httpd_resp_send_err(req,HTTPD_404_NOT_FOUND,"session not found");}
    cJSON *root=cJSON_CreateObject();if(!root)return ESP_ERR_NO_MEM;
    capture_export_status_t exporter;capture_export_get_status(&exporter);
    cJSON_AddBoolToObject(root,"export_busy",exporter.busy);
    cJSON_AddNumberToObject(root,"schema_version",2);cJSON_AddStringToObject(root,"boot_id",boot_id);
    acquisition_status_t imu;optical_ref_status_t ref;acquisition_get_status(&imu);optical_ref_get_status(&ref);
    cJSON_AddBoolToObject(root,"device_online",imu.online);cJSON_AddBoolToObject(root,"reference_ready",ref.enabled&&!ref.input_queue_drops&&!ref.clock_faults&&!ref.sample_gaps);
    cJSON_AddNumberToObject(root,"uptime_us",(double)esp_timer_get_time());
    product_config_t defaults;product_session_get_real_config(&defaults);cJSON *d=cJSON_AddObjectToObject(root,"real_config");
    cJSON_AddStringToObject(d,"vehicle",defaults.vehicle);cJSON_AddNumberToObject(d,"trial_mass_g",defaults.mass_g);
    cJSON_AddNumberToObject(d,"trial_radius_mm",defaults.trial_radius_mm);cJSON_AddNumberToObject(d,"trial_angle_deg",defaults.trial_angle_deg);
    cJSON_AddNumberToObject(d,"correction_radius_mm",defaults.correction_radius_mm);cJSON_AddNumberToObject(d,"axis",defaults.axis);
    cJSON_AddStringToObject(d,"direction",defaults.direction==1?"cw":"ccw");
    cJSON_AddBoolToObject(root,"request_queued",queued);cJSON_AddNumberToObject(root,"current_id",current_id);
    cJSON_AddItemToObject(root,"session",serialize(&snapshot,&trial_view,&retest_view,&speed_view,&window_view,&assessment_view,report));cJSON *list=cJSON_AddArrayToObject(root,"history");
    for(size_t i=0;i<count;++i){cJSON *h=cJSON_CreateObject();cJSON_AddItemToArray(list,h);
        cJSON_AddNumberToObject(h,"id",stored[i].id);cJSON_AddStringToObject(h,"vehicle",stored[i].config.vehicle);
        cJSON_AddStringToObject(h,"source",stored[i].simulated?"synthetic":"device");cJSON_AddStringToObject(h,"state",product_state_name(stored[i].state));}
    raw_status_t raw;raw_capture_get_status(&raw);cJSON_AddNumberToObject(root,"latest_raw_id",raw.id);
    cJSON_AddBoolToObject(root,"latest_raw_ready",raw.state==RAW_READY);
    if(report)cJSON_AddStringToObject(root,"report_kind",snapshot.simulated?"synthetic_product_session":"engineering_product_session");
    char *body=cJSON_PrintUnformatted(root);cJSON_Delete(root);if(!body)return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req,"application/json; charset=utf-8");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    if(report)httpd_resp_set_hdr(req,"Content-Disposition","attachment; filename=wheelbalance_session.json");
    esp_err_t result=httpd_resp_sendstr(req,body);cJSON_free(body);return result;
}
static esp_err_t get(httpd_req_t *r){return reply(r,false,false);}
static esp_err_t report(httpd_req_t *r){return reply(r,false,true);}
extern const unsigned char product_js_start[] asm("_binary_product_js_start");
extern const unsigned char product_js_end[] asm("_binary_product_js_end");
extern const unsigned char product_css_start[] asm("_binary_product_css_start");
extern const unsigned char product_css_end[] asm("_binary_product_css_end");
extern const unsigned char speed_js_start[] asm("_binary_speed_js_start");
extern const unsigned char speed_js_end[] asm("_binary_speed_js_end");
static esp_err_t javascript(httpd_req_t *r){httpd_resp_set_type(r,"application/javascript; charset=utf-8");httpd_resp_set_hdr(r,"Cache-Control","no-store");return httpd_resp_send(r,(const char *)product_js_start,product_js_end-product_js_start-1);}
static esp_err_t stylesheet(httpd_req_t *r){httpd_resp_set_type(r,"text/css; charset=utf-8");httpd_resp_set_hdr(r,"Cache-Control","no-store");return httpd_resp_send(r,(const char *)product_css_start,product_css_end-product_css_start-1);}
static esp_err_t speed_script(httpd_req_t *r){httpd_resp_set_type(r,"application/javascript; charset=utf-8");httpd_resp_set_hdr(r,"Cache-Control","no-store");return httpd_resp_send(r,(const char *)speed_js_start,speed_js_end-speed_js_start-1);}
static bool numeric(const cJSON *o,const char *key,double *out)
{const cJSON *v=cJSON_GetObjectItemCaseSensitive(o,key);if(!cJSON_IsNumber(v)||!isfinite(v->valuedouble))return false;*out=v->valuedouble;return true;}
static esp_err_t post(httpd_req_t *req)
{
    char body[1024];if(req->content_len<2||req->content_len>=sizeof(body))return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"bounded JSON required");
    size_t used=0;while(used<req->content_len){int n=httpd_req_recv(req,body+used,req->content_len-used);if(n<=0)return ESP_FAIL;used+=(size_t)n;}body[used]=0;
    cJSON *root=cJSON_ParseWithLengthOpts(body,used+1,NULL,true);
    const cJSON *boot=root?cJSON_GetObjectItemCaseSensitive(root,"boot_id"):NULL;
    const cJSON *action=root?cJSON_GetObjectItemCaseSensitive(root,"action"):NULL;
    double id,revision;product_request_t r={0};bool valid=cJSON_IsString(boot)&&!strcmp(boot->valuestring,boot_id)&&cJSON_IsString(action)&&
        numeric(root,"id",&id)&&id>=1&&id<=UINT32_MAX&&floor(id)==id&&numeric(root,"revision",&revision)&&revision>=1&&revision<=UINT32_MAX&&floor(revision)==revision;
    if(valid){r.id=(uint32_t)id;r.revision=(uint32_t)revision;
        const char *names[]={"new","original","trial","retest","cancel","demo","demo_trial","demo_retest","trial_plan","finish"};bool found=false;
        for(unsigned i=0;i<10;++i)if(!strcmp(action->valuestring,names[i])){r.action=(product_command_t)i;found=true;break;}
        valid=found;
        if(valid&&r.action==PRODUCT_START_RETEST){r.trial_removed=cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root,"trial_removed"));valid=r.trial_removed;
            const cJSON *condition=cJSON_GetObjectItemCaseSensitive(root,"retest_condition");
            if(condition){valid=valid&&cJSON_IsString(condition);bool found_condition=false;
                for(unsigned i=RETEST_CONDITION_FIRST;valid&&i<=RETEST_CONDITION_CHANGED;++i)
                    if(!strcmp(condition->valuestring,product_retest_condition_name((product_retest_condition_t)i))){r.retest_condition=(product_retest_condition_t)i;found_condition=true;break;}
                valid=valid&&found_condition;}}
        if(valid&&r.action==PRODUCT_START_TRIAL){r.trial_installed=cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root,"trial_installed"));
            valid=r.trial_installed&&numeric(root,"confirmed_mass_g",&r.confirmed_mass_g)&&r.confirmed_mass_g>=.1&&r.confirmed_mass_g<=1000;}
        if(valid&&r.action==PRODUCT_TRIAL_PLAN){cJSON *source=cJSON_GetObjectItemCaseSensitive(root,"plan_source");
            valid=numeric(root,"mass_g",&r.confirmed_mass_g)&&r.confirmed_mass_g>=.1&&r.confirmed_mass_g<=1000&&cJSON_IsString(source);
            if(valid){bool found_source=false;for(unsigned i=0;i<3;++i)if(!strcmp(source->valuestring,product_trial_source_name((product_trial_source_t)i))){r.plan_source=(product_trial_source_t)i;found_source=true;break;}valid=found_source;}}
        if(valid&&r.action==PRODUCT_NEW){cJSON *c=cJSON_GetObjectItemCaseSensitive(root,"config");double axis;
            const cJSON *capture=cJSON_GetObjectItemCaseSensitive(root,"capture_settings");
            if(capture){const cJSON *type=cJSON_GetObjectItemCaseSensitive(capture,"vehicle_type"),*mode=cJSON_GetObjectItemCaseSensitive(capture,"mode");double seconds;
                valid=cJSON_IsObject(capture)&&cJSON_IsString(type)&&cJSON_IsString(mode)&&numeric(capture,"duration_seconds",&seconds)&&seconds>=10&&seconds<=60&&floor(seconds)==seconds;
                const char *types[]={"bicycle","electric","other"};bool type_found=false;
                if(valid)for(unsigned i=0;i<3;++i)if(!strcmp(type->valuestring,types[i])){r.capture.vehicle_type=i;type_found=true;break;}
                valid=valid&&type_found&&(!strcmp(mode->valuestring,"coast")||!strcmp(mode->valuestring,"timed"));
                if(valid){r.capture.mode=!strcmp(mode->valuestring,"timed")?CAPTURE_TIMED:CAPTURE_COAST;r.capture.seconds=(uint16_t)seconds;r.capture_config_present=true;}}
            cJSON *vehicle=cJSON_GetObjectItemCaseSensitive(c,"vehicle"),*direction=cJSON_GetObjectItemCaseSensitive(c,"direction");
            valid=valid&&numeric(c,"trial_mass_g",&r.config.mass_g)&&numeric(c,"trial_radius_mm",&r.config.trial_radius_mm)&&
                numeric(c,"trial_angle_deg",&r.config.trial_angle_deg)&&numeric(c,"correction_radius_mm",&r.config.correction_radius_mm)&&
                numeric(c,"axis",&axis)&&axis>=0&&axis<3&&floor(axis)==axis&&cJSON_IsString(vehicle)&&strlen(vehicle->valuestring)<sizeof(r.config.vehicle)&&
                cJSON_IsString(direction)&&(!strcmp(direction->valuestring,"cw")||!strcmp(direction->valuestring,"ccw"));
            if(valid){r.config.axis=(unsigned)axis;r.config.direction=!strcmp(direction->valuestring,"cw")?1:-1;
                strcpy(r.config.vehicle,vehicle->valuestring);valid=product_config_valid(&r.config);}
            /* Old clients sent an explicit numeric axis without this field. */
            r.manual_axis=true;
            cJSON *axis_mode=cJSON_GetObjectItemCaseSensitive(root,"axis_mode");
            if(valid&&axis_mode){valid=cJSON_IsString(axis_mode)&&(!strcmp(axis_mode->valuestring,"auto")||!strcmp(axis_mode->valuestring,"manual"));
                if(valid)r.manual_axis=!strcmp(axis_mode->valuestring,"manual");}
            cJSON *guide=cJSON_GetObjectItemCaseSensitive(root,"trial_guide");r.piece_mass_g=5;
            if(valid&&guide){cJSON *mode=cJSON_GetObjectItemCaseSensitive(guide,"mode");
                valid=cJSON_IsString(mode)&&(!strcmp(mode->valuestring,"guided")||!strcmp(mode->valuestring,"manual"))&&
                    numeric(guide,"piece_mass_g",&r.piece_mass_g)&&r.piece_mass_g>=.1&&r.piece_mass_g<=500;
                if(valid)r.guided=!strcmp(mode->valuestring,"guided");}}
    }
    cJSON_Delete(root);if(!valid)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"invalid session command");
    return reply(req,product_session_submit(&r),false);
}
esp_err_t product_http_register(httpd_handle_t server,const char *boot)
{
    if(!http_view)http_view=heap_caps_calloc(1,sizeof(*http_view),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!http_view)return ESP_ERR_NO_MEM;
    snprintf(boot_id,sizeof(boot_id),"%s",boot);esp_err_t err=product_session_start(boot);if(err!=ESP_OK)return err;
    const httpd_uri_t routes[]={
        {.uri="/api/session",.method=HTTP_GET,.handler=get},
        {.uri="/api/session",.method=HTTP_POST,.handler=post},
        {.uri="/api/session/report",.method=HTTP_GET,.handler=report},
        {.uri="/product.js",.method=HTTP_GET,.handler=javascript},
        {.uri="/product.css",.method=HTTP_GET,.handler=stylesheet},
        {.uri="/speed.js",.method=HTTP_GET,.handler=speed_script}};
    for(unsigned i=0;i<6;++i){err=httpd_register_uri_handler(server,&routes[i]);if(err!=ESP_OK)return err;}return ESP_OK;
}
