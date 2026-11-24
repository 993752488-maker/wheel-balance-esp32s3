/* Actual product engine plus the same FIFO/selection path as the device.
 * Host transport only; --demo is explicitly synthetic. */
#include "product_engine.h"
#include "product_speed.h"
#include "product_window.h"
#include "product_auto_axis.h"
#include "product_assessment.h"
#include "live_analysis_core.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static sync_sample_t samples[SYNC_MAX_SAMPLES];
static selection_reference_t refs[COAST_SELECTION_MAX_REFS];
static selection_turn_t turns[COAST_SELECTION_MAX_REFS];
static live_analysis_core_t clock_core;
static product_run_t runs[4];
static product_run_t all_axes[4][3];
static product_assessment_t assessment;
static product_config_t config;
static product_solution_t result;
static product_comparison_t comparison;
static size_t selected_count;
static bool settling,input_synthetic,auto_solve,timed;
static product_window_stage_t window_stages[4];
static void assessment_json(void)
{
    const product_assessment_t *q=&assessment;
    printf("{\"available\":%s,\"state\":\"%s\",\"policy\":\"pairwise_consensus_normalized_fit_v4\",\"axis\":%u,\"axis_finalized\":%s,\"warning_flags\":%u,\"axis_band_conflict_mask\":%u,\"valid_bands\":%u,\"joint_used\":%s,\"joint_bands\":%u,\"joint_band_mask\":%u,\"observed_mass_min_g\":%.17g,\"observed_mass_max_g\":%.17g,\"model_residual_ratio\":%.17g,\"accuracy_interval\":false,\"physical_repeatability_verified\":false,\"bands\":[",
        q->valid_bands?"true":"false",product_assessment_state_name((product_assessment_state_t)q->state),q->axis,q->axis_finalized?"true":"false",q->flags,q->axis_conflict_mask,q->valid_bands,q->joint_used?"true":"false",q->joint_bands,q->joint_band_mask,q->mass_min_g,q->mass_max_g,q->model_residual_ratio);
    bool comma=false;for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){const product_assessment_band_t *r=&q->bands[i];if(!r->pairs)continue;
        printf("%s{\"lower_rpm\":%u,\"matched_turns\":%u,\"available\":%s,\"reason\":\"%s\",\"mass_g\":%.17g,\"angle_deg\":%.17g,\"score\":%.17g,\"h_real\":%.17g,\"h_imag\":%.17g,\"v0_real\":%.17g,\"v0_imag\":%.17g,\"used_for_joint_fit\":%s,\"split_available\":%s,\"split_valid\":%s,\"split_relative_vector_difference\":%.17g,\"split_phase_difference_deg\":%.17g}",
            comma?",":"",r->lower_rpm,r->pairs,r->available?"true":"false",product_reason_name(r->reason),r->mass_g,r->angle_deg,r->score,r->influence_real,r->influence_imag,r->original_real,r->original_imag,
            q->joint_band_mask&(1U<<i)?"true":"false",r->split_available?"true":"false",r->split_valid?"true":"false",r->split_relative_difference,r->split_phase_difference);comma=true;}
    printf("]}");
}
static void speed_stage_json(unsigned stage)
{
    product_speed_t profile={0};(void)product_speed_set(&profile,stage,&runs[stage],false,0);
    const product_speed_stage_t *v=&profile.stages[stage];const char *names[]={"empty","preview","frozen","invalid"};
    printf("{\"state\":\"%s\",\"capture_id\":%u,\"epoch\":%u,\"start_us\":%" PRIu64 ",\"end_us\":%" PRIu64 ",\"updated_us\":0,\"used_turns\":%u,\"rejected_turns\":%u,\"outside_speed_turns\":%u,\"points\":[",names[v->mode],v->capture_id,v->epoch,v->start_us,v->end_us,v->used_turns,v->rejected_turns,v->outside_turns);
    bool comma=false;for(unsigned i=0;i<PRODUCT_SPEED_BINS;++i){const product_speed_bin_t *b=&v->bins[i];if(!b->count)continue;
        const unsigned lower=product_speed_lower_rpm(v,i);
        printf("%s{\"lower_rpm\":%u,\"upper_rpm\":%u,\"count\":%u,\"rms\":%.9g,\"low\":%.9g,\"high\":%.9g}",comma?",":"",lower,lower+PRODUCT_SPEED_WIDTH,b->count,(double)b->rms,(double)b->minimum,(double)b->maximum);comma=true;}
    printf("]}");
}
static void window_stage_json(unsigned stage)
{
    const product_window_stage_t *w=&window_stages[stage];const char *modes[]={"unknown","steady","coast","mixed"};
    printf("{\"capture_id\":%u,\"epoch\":%u,\"start_us\":%" PRIu64 ",\"end_us\":%" PRIu64 ",\"phase\":\"complete\",\"motion\":\"%s\",\"total_turns\":%u,\"accelerating_turns\":%u,\"usable_turns\":%u,\"skipped_turns\":%u,\"transient_turns\":%u,\"first_usable_us\":%" PRIu64 ",\"last_usable_us\":%" PRIu64 ",\"progress_axis\":%u,\"common_pairs\":0}",w->capture_id,w->epoch,w->start_us,w->end_us,modes[w->mode],w->total_turns,w->accelerating_turns,w->usable[config.axis],w->skipped[config.axis],w->transient[config.axis],w->first_usable_us[config.axis],w->last_usable_us[config.axis],config.axis);
}
static void summary(const product_stage_summary_t *s)
{
    printf("{\"captured\":%s,\"data_integrity_valid\":%s,\"capture_id\":%u,\"epoch\":%u,\"samples\":%u,\"references\":%u,\"turn_count\":%u,\"usable_turns\":%u,\"rejected_turns\":%u,\"start_us\":%" PRIu64 ",\"end_us\":%" PRIu64 ",\"reason\":\"%s\",\"min_rpm\":%.17g,\"max_rpm\":%.17g,\"real\":%.17g,\"imag\":%.17g,\"peak_mps2\":%.17g,\"vector_scatter\":%.17g,\"direction_coherence\":%.17g}",
        s->captured?"true":"false",s->integrity_valid?"true":"false",s->capture_id,s->epoch,s->samples,s->references,s->turn_count,s->usable_turns,s->rejected_turns,s->start_us,s->end_us,product_reason_name(s->reason),s->min_rpm,s->max_rpm,s->real,s->imag,s->peak,s->scatter,s->coherence);
}
static void output(bool simulated,unsigned datasets)
{
    printf("{\"source\":\"%s\",\"speed_selection_policy\":\"max_chronological_pairs_then_pairwise_consensus\",\"stages\":[",simulated?"synthetic":"device");
    for(unsigned i=0;i<datasets;++i){if(i)printf(",");summary(&runs[i].summary);}
    printf("],\"capture_window_stages\":[");for(unsigned stage=0;stage<datasets;++stage){if(stage)printf(",");window_stage_json(stage);}printf("],\"speed_profile_stages\":[");for(unsigned stage=0;stage<datasets;++stage){if(stage)printf(",");speed_stage_json(stage);}printf("],\"result\":{\"calculation_valid\":%s,\"engineering_available\":%s,\"accuracy_validated\":false,\"reason\":\"%s\",\"mass_g\":%.17g,\"angle_deg\":%.17g,\"matched_turns\":%u,\"lower_rpm\":%.17g,\"upper_rpm\":%.17g,\"mean_rpm\":%.17g,\"h_real\":%.17g,\"h_imag\":%.17g,\"delta_peak\":%.17g,\"delta_coherence\":%.17g,\"mean_variation_scale\":%.17g,\"original_peak\":%.17g,\"trial_peak\":%.17g,\"core_status\":\"%s\"},",
        result.calculation_valid?"true":"false",result.engineering_available?"true":"false",product_reason_name(result.reason),result.calculation.mass_g,result.calculation.angle_deg,result.matched_turns,result.lower_rpm,result.upper_rpm,result.mean_rpm,result.calculation.influence.real,result.calculation.influence.imag,result.delta_peak,result.delta_coherence,result.mean_noise_scale,result.original_peak,result.trial_peak,result.mean_rpm>0?balance_status_name(result.calculation.status):"NOT_CALLED");
    printf("\"assessment\":");assessment_json();printf(",");
    printf("\"comparison\":{\"available\":%s,\"reason\":\"%s\",\"matched_turns\":%u,\"original_peak_mps2\":%.17g,\"retest_peak_mps2\":%.17g,\"improvement_percent\":%.17g},\"recommendation_valid\":false}\n",
        comparison.available?"true":"false",product_reason_name(comparison.reason),comparison.matched_turns,comparison.original_peak,comparison.retest_peak,comparison.improvement_percent);
}
static void demo(void)
{
    config=product_default_config();config.mass_g=5;config.trial_radius_mm=config.correction_radius_mm=100;
    for(unsigned stage=0;stage<3;++stage){memset(turns,0,8*sizeof(*turns));
        runs[stage].summary=(product_stage_summary_t){.integrity_valid=true,.epoch=1,.capture_id=stage+1,.samples=4000,.references=10,
            .start_us=1000000+stage*10000000,.end_us=9000000+stage*10000000};
        for(unsigned i=0;i<8;++i){coast_result_t *f=&turns[i].fit;f->valid=true;f->epoch=1;f->rpm=116-i*2;f->rpm_previous=f->rpm+2;
            f->ref_last=stage*10+i+2;f->end_us=runs[stage].summary.start_us+(i+1)*900000;f->start_us=f->end_us-900000;
            f->axis[1].real=stage==2?.05358983848622454*.2:.05358983848622454+(stage==1?1:0);
            f->axis[1].imag=stage==2?-.8928203230275509*.2:-.8928203230275509+(stage==1?-.5:0);}
        product_window_select(turns,8,&window_stages[stage]);window_stages[stage].capture_id=stage+1;window_stages[stage].epoch=1;
        window_stages[stage].start_us=runs[stage].summary.start_us;window_stages[stage].end_us=runs[stage].summary.end_us;
        (void)product_extract_run(turns,8,1,&runs[stage]);}
}
static int dataset(unsigned stage)
{
    char line[512];product_stage_summary_t *s=&runs[stage].summary;unsigned integrity=0;double target_rpm=0;
    if(!fgets(line,sizeof(line),stdin)||sscanf(line,"D,%u,%u,%u,%u,%" SCNu64 ",%" SCNu64 ",%u,%lf",
        &s->capture_id,&s->epoch,&s->samples,&s->references,&s->start_us,&s->end_us,&integrity,&target_rpm)!=(timed?8:7))return 2;
    if(timed&&(!isfinite(target_rpm)||target_rpm<PRODUCT_MIN_RPM||target_rpm>=PRODUCT_TIMED_MAX_RPM_EXCLUSIVE))return 2;
    s->integrity_valid=integrity==1;size_t nr=0,rows=0;bool ended=false;live_core_init(&clock_core,samples,SYNC_MAX_SAMPLES);
    while(fgets(line,sizeof(line),stdin)){
        if(!strcmp(line,"E\n")||!strcmp(line,"E\r\n")){ended=true;break;}
        if(line[0]=='S'){live_input_sample_t v={0};
            if(rows++>=64000||sscanf(line,"S,%" SCNu64 ",%" SCNu32 ",%" SCNd64 ",%" SCNd64 ",%" SCNu16 ",%" SCNu16 ",%" SCNd16 ",%" SCNd16 ",%" SCNd16,
                &v.sequence,&v.epoch,&v.count_begin_us,&v.count_end_us,&v.pending_frames,&v.batch_index,&v.raw[0],&v.raw[1],&v.raw[2])!=9)return 2;
            size_t before=clock_core.count;live_core_sample(&clock_core,&v);
            if(clock_core.status.timing_resets||(clock_core.have_anchor&&clock_core.count!=before+1))return 3;
        }else if(line[0]=='R'){unsigned period;
            if(nr==COAST_SELECTION_MAX_REFS||sscanf(line,"R,%" SCNu64 ",%" SCNu64 ",%u",&refs[nr].ref.sequence,&refs[nr].ref.time_us,&period)!=3||period>1)return 2;
            refs[nr++].has_period=period!=0;
        }else return 2;
    }
    if(!ended||!rows||!live_core_retime(&clock_core))return 3;
    if(nr<3){s->reason=PRODUCT_TOO_FEW_TURNS;selected_count=0;return 0;}
    selection_config_t cfg=selection_default_config();size_t count=0;
    if(settling)product_window_configure_fit(&cfg,timed?CAPTURE_TIMED:CAPTURE_COAST);
    if(coast_select(&cfg,clock_core.samples,clock_core.count,refs,nr,(sync_quality_t){SYNC_TIME_ESTIMATED,0},turns,COAST_SELECTION_MAX_REFS,&count)!=SELECTION_OK)return 3;
    selected_count=count;
    if(settling){
        if(timed)product_window_select_timed(turns,count,target_rpm,&window_stages[stage]);
        else product_window_select(turns,count,&window_stages[stage]);
        window_stages[stage].capture_id=s->capture_id;window_stages[stage].epoch=s->epoch;window_stages[stage].start_us=s->start_us;window_stages[stage].end_us=s->end_us;}
    (void)product_extract_run(turns,count,config.axis,&runs[stage]);
    for(unsigned k=0;k<3;++k){all_axes[stage][k].summary=*s;(void)product_extract_run(turns,count,k,&all_axes[stage][k]);}return 0;
}
static void profile(void)
{
    printf("{\"kind\":\"shared_device_C_profile_replay\",\"axis\":%u,\"summary\":",config.axis);summary(&runs[0].summary);
    printf(",\"turns\":[");
    for(size_t i=0;i<runs[0].count;++i){const product_turn_t *t=&runs[0].turns[i];
        printf("%s{\"rpm\":%.17g,\"previous_rpm\":%.17g,\"real\":%.17g,\"imag\":%.17g,\"reference\":%" PRIu64 ",\"time_us\":%" PRIu64 "}",i?",":"",t->rpm,t->previous_rpm,t->real,t->imag,t->reference,t->time_us);}
    printf("],\"all_turn_selection\":[");
    for(size_t i=0;i<selected_count;++i){const coast_result_t *f=&turns[i].fit;
        printf("%s{\"index\":%u,\"fit_valid\":%s,\"reason_bits\":%u,\"rpm\":%.17g,\"previous_rpm\":%.17g,\"real\":%.17g,\"imag\":%.17g,\"reference\":%" PRIu64 ",\"time_us\":%" PRIu64 "}",i?",":"",(unsigned)i,f->valid?"true":"false",(unsigned)turns[i].reasons[config.axis],f->valid?f->rpm:0,f->valid?f->rpm_previous:0,f->valid?f->axis[config.axis].real:0,f->valid?f->axis[config.axis].imag:0,f->ref_last,f->end_us);}
    printf("],\"capture_window\":");window_stage_json(0);printf(",\"speed_profile_stage\":");speed_stage_json(0);printf(",\"settling_applied\":%s,\"mass_angle_output\":false}\n",settling?"true":"false");
}

static bool assess_pair(unsigned datasets)
{
    if(!auto_solve)return product_assess_runs(&config,&runs[0],&runs[1],&result,&assessment);
    unsigned chosen=config.axis;
    const bool ok=product_assess_axes(&config,all_axes[0],all_axes[1],&chosen,&result,&assessment);
    config.axis=chosen;
    for(unsigned i=0;i<datasets;++i)runs[i]=all_axes[i][chosen];
    return ok;
}

int main(int argc,char **argv)
{
    if(argc>1&&!strcmp(argv[1],"--timed")){timed=true;settling=true;--argc;++argv;}
    if(argc==2&&!strcmp(argv[1],"--auto-pair-window")){auto_solve=true;argv[1]="--pair-window";}
    else if(argc==2&&!strcmp(argv[1],"--auto-window")){auto_solve=true;argv[1]="--window";}
    else if(timed&&argc==2&&!strcmp(argv[1],"--auto-repeat")){auto_solve=true;argv[1]="--repeat";}
    if(argc==2&&!strcmp(argv[1],"--synthetic-window")){settling=true;input_synthetic=true;argc=1;}
    if(argc==2&&!strcmp(argv[1],"--synthetic-timed")){timed=true;settling=true;input_synthetic=true;auto_solve=true;argc=1;}
    if(argc==2&&strstr(argv[1],"window")){settling=true;
        if(!strcmp(argv[1],"--profile-all-window"))argv[1]="--profile-all";
        else if(!strcmp(argv[1],"--profile-window"))argv[1]="--profile";
        else if(!strcmp(argv[1],"--pair-window"))argv[1]="--pair";
        else if(!strcmp(argv[1],"--window"))argc=1;
        else return 2;
    }
    if(argc==2&&!strcmp(argv[1],"--repeat")){
        char line[256];config=product_default_config();
        if(!fgets(line,sizeof(line),stdin)||sscanf(line,"C,%lf,%lf,%lf,%lf,%u,%d",&config.mass_g,&config.trial_radius_mm,&config.trial_angle_deg,&config.correction_radius_mm,&config.axis,&config.direction)!=6||!product_config_valid(&config))return 2;
        for(unsigned i=0;i<4;++i){int err=dataset(i);if(err)return err;}
        if(!(timed?assess_pair(4):product_solve_runs(&config,&runs[0],&runs[1],&result)))return 4;
        product_repeat_observation_t o;product_observe_repeat(&config,&runs[2],&runs[3],&result,&o);
        printf("{\"kind\":\"shared_C_same_condition_repeat_observation\",\"state\":\"%s\",\"previous_capture_id\":%u,\"current_capture_id\":%u,\"matched_turns\":%u,\"phase_available\":%s,\"previous_peak_mps2\":%.17g,\"current_peak_mps2\":%.17g,\"amplitude_change_percent\":%.17g,\"phase_change_deg\":%.17g,\"reason\":\"%s\",\"lower_rpm\":%.17g,\"upper_rpm\":%.17g,\"accuracy_validated\":false}\n",
            product_repeat_state_name(o.state),o.previous_capture_id,runs[3].summary.capture_id,o.matched_turns,o.phase_available?"true":"false",o.previous_peak,o.current_peak,o.amplitude_change_percent,o.phase_change_deg,product_reason_name(o.reason),result.lower_rpm,result.upper_rpm);return 0;
    }
    if(argc==2&&!strcmp(argv[1],"--profile-all")){char line[64];config=product_default_config();
        if(!fgets(line,sizeof(line),stdin)||sscanf(line,"P,%u",&config.axis)!=1||config.axis>2)return 2;
        int err=dataset(0);if(err)return err;
        static product_run_t axes[3];
        for(unsigned axis=0;axis<3;++axis){axes[axis].summary=runs[0].summary;(void)product_extract_run(turns,selected_count,axis,&axes[axis]);}
        product_axis_choice_t choice;const bool ok=product_choose_axis(axes,&choice);
        printf("{\"kind\":\"shared_device_C_automatic_axis_baseline_only\",\"valid\":%s,\"selected_axis\":%u,\"reason\":\"%s\",\"scores\":[%.17g,%.17g,%.17g],\"summaries\":[",ok?"true":"false",choice.axis,product_reason_name(choice.reason),choice.score[0],choice.score[1],choice.score[2]);
        for(unsigned axis=0;axis<3;++axis){if(axis)printf(",");summary(&axes[axis].summary);}printf("],\"trial_or_retest_used_for_selection\":false,\"mass_angle_output\":false}\n");return 0;}
    if(argc==2&&!strcmp(argv[1],"--profile")){char line[64];config=product_default_config();
        if(!fgets(line,sizeof(line),stdin)||sscanf(line,"P,%u",&config.axis)!=1||config.axis>2)return 2;
        int err=dataset(0);if(err)return err;profile();return 0;}
    bool simulated=argc==2&&!strcmp(argv[1],"--demo");
    const bool pair_only=argc==2&&!strcmp(argv[1],"--pair");
    if(simulated)demo();else{char line[256];config=product_default_config();
        if(!fgets(line,sizeof(line),stdin)||sscanf(line,"C,%lf,%lf,%lf,%lf,%u,%d",&config.mass_g,&config.trial_radius_mm,&config.trial_angle_deg,&config.correction_radius_mm,&config.axis,&config.direction)!=6||!product_config_valid(&config))return 2;
        for(unsigned i=0;i<(pair_only?2U:3U);++i){int err=dataset(i);if(err)return err;}}
    (void)assess_pair(pair_only?2:3);
    if(result.calculation_valid&&!pair_only)(void)product_compare_runs(&config,&runs[0],&runs[2],&result,&comparison);
    output(simulated||input_synthetic,pair_only?2:3);return 0;
}
