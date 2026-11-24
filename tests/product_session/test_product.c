#include "product_engine.h"
#include "product_assessment.h"
#include "product_job.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(v) do{if(!(v)){fprintf(stderr,"FAIL %s:%d %s\n",__func__,__LINE__,#v);return false;}}while(0)
static unsigned passed;
static product_config_t config;
static product_run_t a,b,c;
static product_solution_t result;
static void run(product_run_t *out,unsigned stage,double re,double im)
{
    memset(out,0,sizeof(*out));out->summary=(product_stage_summary_t){.integrity_valid=true,.epoch=1,.capture_id=stage+1,
        .start_us=1000000+stage*10000000,.end_us=9000000+stage*10000000};
    selection_turn_t t[8];memset(t,0,sizeof(t));
    for(unsigned i=0;i<8;++i){t[i].fit.valid=true;t[i].fit.epoch=1;t[i].fit.rpm=116-i*2;t[i].fit.rpm_previous=118-i*2;
        t[i].fit.ref_last=stage*10+i+2;t[i].fit.end_us=out->summary.start_us+(i+1)*900000;
        t[i].fit.axis[1].real=re;t[i].fit.axis[1].imag=im;t[i].reasons[1]=SELECT_WEAK_PHASE;}
    (void)product_extract_run(t,8,1,out);
}
static void setup(void){config=product_default_config();config.mass_g=5;config.trial_radius_mm=config.correction_radius_mm=100;
    run(&a,0,.05358983848622454,-.8928203230275509);run(&b,1,1.0535898384862245,-1.3928203230275509);run(&c,2,.0107179676972449,-.17856406460551);}
static bool full_estimated_flow(void){CHECK(product_solve_runs(&config,&a,&b,&result));CHECK(result.engineering_available);
    CHECK(fabs(result.calculation.mass_g-4)<1e-9&&fabs(result.calculation.angle_deg-120)<1e-9);
    CHECK(!result.accuracy_validated&&!result.calculation.recommendation_valid&&!result.calculation.simulated);
    product_comparison_t comparison;CHECK(product_compare_runs(&config,&a,&c,&result,&comparison));CHECK(fabs(comparison.improvement_percent-80)<1e-9);return true;}
static bool negative_improvement(void){CHECK(product_solve_runs(&config,&a,&b,&result));run(&c,2,a.summary.real*1.2,a.summary.imag*1.2);
    product_comparison_t r;CHECK(product_compare_runs(&config,&a,&c,&result,&r));CHECK(fabs(r.improvement_percent+20)<1e-9);return true;}
static bool phase_cancellation_is_not_false_improvement(void){CHECK(product_solve_runs(&config,&a,&b,&result));
    for(size_t i=0;i<c.count;++i){c.turns[i].real=a.turns[i].real*(i%2?1:-1);c.turns[i].imag=a.turns[i].imag*(i%2?1:-1);}
    product_comparison_t r;CHECK(product_compare_runs(&config,&a,&c,&result,&r));CHECK(fabs(r.improvement_percent)<1e-9);CHECK(r.reason==PRODUCT_LOW_DIRECTION_COHERENCE);return true;}
static bool weak_residual_keeps_amplitude_result(void){CHECK(product_solve_runs(&config,&a,&b,&result));
    for(size_t i=0;i<c.count;++i){c.turns[i].real=a.turns[i].real*.2*(i%2?1:-1);c.turns[i].imag=a.turns[i].imag*.2*(i%2?1:-1);}
    product_comparison_t r;CHECK(product_compare_runs(&config,&a,&c,&result,&r));CHECK(r.available);
    CHECK(fabs(r.improvement_percent-80)<1e-9);CHECK(r.reason==PRODUCT_LOW_DIRECTION_COHERENCE);return true;}
static bool real_speed_mismatch_is_preserved(void){for(size_t i=0;i<b.count;++i){b.turns[i].rpm*=1.015;b.turns[i].previous_rpm*=1.015;}
    CHECK(product_solve_runs(&config,&a,&b,&result));CHECK(result.calculation.rpm_mismatch>0);CHECK(fabs(result.calculation.mass_g-4)<1e-8);return true;}
static bool no_trial_response(void){run(&b,1,a.summary.real,a.summary.imag);CHECK(!product_solve_runs(&config,&a,&b,&result));CHECK(result.reason==PRODUCT_WEAK_RESPONSE&&!result.calculation_valid);return true;}
static bool epoch_and_overlap(void){b.summary.epoch=2;CHECK(!product_solve_runs(&config,&a,&b,&result));CHECK(result.reason==PRODUCT_CONTEXT);
    b.summary.epoch=1;b.summary.start_us=a.summary.start_us;CHECK(!product_solve_runs(&config,&a,&b,&result));return true;}
static bool faults_and_no_band(void){b.summary.integrity_valid=false;CHECK(!product_solve_runs(&config,&a,&b,&result));b.summary.integrity_valid=true;
    for(size_t i=0;i<b.count;++i){b.turns[i].rpm+=30;b.turns[i].previous_rpm+=30;}CHECK(!product_solve_runs(&config,&a,&b,&result));CHECK(result.reason==PRODUCT_NO_COMMON_SPEED);return true;}
static bool zero_baseline(void){run(&a,0,0,0);run(&b,1,1,-.5);CHECK(product_solve_runs(&config,&a,&b,&result));CHECK(!result.engineering_available);
    product_comparison_t r;CHECK(!product_compare_runs(&config,&a,&c,&result,&r));CHECK(r.reason==PRODUCT_BASELINE_ZERO);return true;}
static bool near_zero_mean_with_large_turns_is_inconsistent(void)
{
    for(size_t i=0;i<a.count;++i){
        a.turns[i].real=i%2?.1:-.1;
        a.turns[i].imag=0;
        b.turns[i].real=a.turns[i].real+1;
        b.turns[i].imag=0;
    }
    CHECK(!product_solve_runs(&config,&a,&b,&result));
    CHECK(result.reason==PRODUCT_INCONSISTENT&&!result.calculation_valid);
    for(size_t i=0;i<a.count;++i){
        a.turns[i].real=1;
        b.turns[i].real=i%2?.1:-.1;
    }
    CHECK(!product_solve_runs(&config,&a,&b,&result));
    CHECK(result.reason==PRODUCT_INCONSISTENT&&!result.calculation_valid);
    return true;
}
static bool matching_maximizes_pairs_before_speed_error(void)
{
    const double original[]={112,111.5,110,109};
    const double measured[]={112,111.5,109.9,106.6};
    a.count=b.count=3;
    for(size_t i=0;i<a.count;++i){
        a.turns[i].previous_rpm=original[i];a.turns[i].rpm=original[i+1];
        b.turns[i].previous_rpm=measured[i];b.turns[i].rpm=measured[i+1];
    }
    product_pairing_t pairs;
    CHECK(product_match_runs(&a,&b,20,300,&pairs)==PRODUCT_REASON_NONE&&pairs.count==3);
    for(size_t i=0;i<pairs.count;++i)CHECK(pairs.turns[i].original==i&&pairs.turns[i].measured==i);
    CHECK(product_solve_runs(&config,&a,&b,&result));
    CHECK(result.matched_turns==3&&fabs(result.calculation.mass_g-4)<1e-9);
    return true;
}
static bool matches_across_speed_band_boundary_without_reuse(void)
{
    a.count=b.count=c.count=6;
    for(size_t i=0;i<a.count;++i){
        a.turns[i].rpm=a.turns[i].previous_rpm=119;
        b.turns[i].rpm=b.turns[i].previous_rpm=121;
        c.turns[i].rpm=c.turns[i].previous_rpm=121;
    }
    CHECK(product_common_turns(&a,&b,100,120)==6);
    CHECK(product_common_turns(&a,&b,120,140)==0);
    CHECK(product_solve_runs(&config,&a,&b,&result)&&result.matched_turns==6);
    CHECK(result.lower_rpm==100&&result.upper_rpm==120);
    product_comparison_t comparison;
    CHECK(product_compare_runs(&config,&a,&c,&result,&comparison)&&comparison.matched_turns==6);
    CHECK(fabs(comparison.improvement_percent-80)<1e-9);
    return true;
}
static bool weak_single_turn_is_not_automatic_failure(void){CHECK(a.summary.captured&&a.summary.usable_turns==8);CHECK(product_solve_runs(&config,&a,&b,&result));return true;}
static bool invalid_run_not_reused(void){selection_turn_t t[3];memset(t,0,sizeof(t));a.summary.integrity_valid=false;
    CHECK(!product_extract_run(t,3,1,&a));CHECK(!a.summary.captured&&a.count==0);return true;}
static bool bad_config(void){config.mass_g=NAN;CHECK(!product_solve_runs(&config,&a,&b,&result));CHECK(!result.engineering_available);return true;}
static void move_to_lower_band(unsigned count,bool remove_response)
{
    for(unsigned i=0;i<count;++i){a.turns[i].rpm=b.turns[i].rpm=58-i*2;
        a.turns[i].previous_rpm=b.turns[i].previous_rpm=60-i*2;
        if(remove_response){b.turns[i].real=a.turns[i].real;b.turns[i].imag=a.turns[i].imag;}}
}
static bool failed_larger_band_does_not_hide_usable_band(void)
{
    move_to_lower_band(5,true);
    CHECK(product_solve_runs(&config,&a,&b,&result));CHECK(result.lower_rpm==100&&result.matched_turns==3);
    CHECK(fabs(result.calculation.mass_g-4)<1e-9&&fabs(result.calculation.angle_deg-120)<1e-9);return true;
}
static bool band_ranking_does_not_target_a_mass(void)
{
    move_to_lower_band(3,false);
    for(unsigned i=0;i<3;++i){b.turns[i].real=a.turns[i].real+2;b.turns[i].imag=a.turns[i].imag-1;}
    CHECK(product_solve_runs(&config,&a,&b,&result));CHECK(result.lower_rpm==100&&result.matched_turns==5);
    CHECK(fabs(result.calculation.mass_g-4)<1e-9);return true;
}
static bool equally_usable_bands_choose_tailward_usable_band(void)
{
    move_to_lower_band(4,false);CHECK(product_solve_runs(&config,&a,&b,&result));
    CHECK(result.lower_rpm==100&&result.matched_turns==4);return true;
}
static bool all_failed_bands_stay_failed(void)
{
    move_to_lower_band(5,true);
    for(unsigned i=5;i<8;++i){b.turns[i].real=a.turns[i].real;b.turns[i].imag=a.turns[i].imag;}
    CHECK(!product_solve_runs(&config,&a,&b,&result));CHECK(result.reason==PRODUCT_WEAK_RESPONSE);
    CHECK(result.lower_rpm==40&&!result.engineering_available);return true;
}
static bool unselected_nonfinite_input_is_still_a_hard_fault(void)
{
    move_to_lower_band(5,true);a.turns[0].real=NAN;
    CHECK(!product_solve_runs(&config,&a,&b,&result));CHECK(result.reason==PRODUCT_NUMERIC&&!result.calculation_valid);return true;
}
static void arm_coast(product_job_t *job,uint64_t now,uint64_t edges,uint32_t epoch)
{product_job_arm(job,now,edges,epoch,product_capture_default());}
static bool arming_and_stop(void){product_job_t job;arm_coast(&job,1000000,10,1);
    CHECK(product_job_tick(&job,2000000,10,1,true,true,false,true,100)==JOB_NONE);
    CHECK(product_job_tick(&job,3000000,11,1,true,true,false,true,100)==JOB_NONE);
    CHECK(product_job_tick(&job,4000000,12,1,true,true,false,true,100)==JOB_START);
    product_job_recording(&job,4000000,12);CHECK(product_job_tick(&job,6000000,13,1,true,true,false,true,90)==JOB_NONE);
    CHECK(product_job_tick(&job,11000000,13,1,true,true,false,true,90)==JOB_NONE);
    CHECK(product_job_tick(&job,14000000,13,1,true,true,false,false,0)==JOB_STOP&&job.end_reason==CAPTURE_END_REFERENCE_SILENCE);return true;}
static bool whole_coast_not_count_or_old_deadline(void){product_job_t j;arm_coast(&j,1,0,1);product_job_recording(&j,1000000,2);
    for(unsigned sec=1;sec<=100;++sec)CHECK(product_job_tick(&j,1000000+(uint64_t)sec*1000000,2+sec*3,1,true,true,false,true,180)==JOB_NONE);
    CHECK(j.state==PRODUCT_RECORDING&&j.end_reason==CAPTURE_END_UNKNOWN);
    CHECK(product_job_tick(&j,108999999,302,1,true,true,false,false,0)==JOB_NONE);
    CHECK(product_job_tick(&j,109000000,302,1,true,true,false,false,0)==JOB_STOP);return true;}
static bool silence_remembers_slow_period_and_new_edge(void){product_job_t j;
    arm_coast(&j,1,0,1);product_job_recording(&j,1000000,2);
    CHECK(product_job_tick(&j,4000000,3,1,true,true,false,true,20)==JOB_NONE);
    CHECK(product_job_tick(&j,12000000,3,1,true,true,false,false,0)==JOB_NONE);
    CHECK(product_job_tick(&j,12500000,4,1,true,true,false,false,0)==JOB_NONE);
    CHECK(product_job_tick(&j,21000000,4,1,true,true,false,false,0)==JOB_NONE);
    CHECK(product_job_tick(&j,21500000,4,1,true,true,false,false,0)==JOB_STOP);return true;}
static bool explicit_finish_and_incomplete_limit(void){product_job_t j;arm_coast(&j,1,0,1);
    CHECK(!product_job_finish(&j));product_job_recording(&j,1000000,2);CHECK(product_job_finish(&j));
    CHECK(j.state==PRODUCT_PROCESSING&&j.end_reason==CAPTURE_END_OPERATOR&&!product_job_finish(&j));
    arm_coast(&j,1,0,1);product_job_recording(&j,1000000,2);
    CHECK(product_job_tick(&j,121000000,300,1,true,true,false,true,150)==JOB_FAIL);
    CHECK(j.state==PRODUCT_NO_RESULT&&j.reason==PRODUCT_CAPTURE_LIMIT&&j.end_reason==CAPTURE_END_LIMIT);return true;}
static bool timed_vehicle_mode_has_exact_duration(void){product_job_t j;
    product_capture_config_t capture={.mode=CAPTURE_COAST,.vehicle_type=CAPTURE_ELECTRIC,.seconds=30};
    product_capture_apply_preset(&capture);product_job_arm(&j,1,100,1,capture);
    CHECK(product_capture_config_valid(&j.capture)&&j.capture.mode==CAPTURE_TIMED);
    CHECK(product_job_tick(&j,100000,100,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,200000,101,1,true,true,false,true,640)==JOB_NONE);
    CHECK(product_job_tick(&j,400000,102,1,true,true,false,true,400)==JOB_NONE);
    CHECK(product_job_tick(&j,760000,103,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,800000,103,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,880000,104,1,true,true,false,true,504)==JOB_NONE);
    CHECK(product_job_tick(&j,1000000,105,1,true,true,false,true,500)==JOB_START);
    CHECK(fabs(j.target_rpm-502)<1e-9);
    product_job_recording(&j,1000000,105);
    for(unsigned i=1;i<250;++i)
        CHECK(product_job_tick(&j,1000000+(uint64_t)i*120000,105+i,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,30999999,354,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,31000000,355,1,true,true,false,true,500)==JOB_STOP&&j.end_reason==CAPTURE_END_TIMED);
    j.capture.seconds=0;CHECK(product_capture_config_valid(&j.capture));
    j.capture.seconds=61;CHECK(!product_capture_config_valid(&j.capture));return true;}
static bool automatic_duration_and_baseline_speed(void){product_job_t j;
    const product_capture_config_t automatic={.mode=CAPTURE_TIMED,.vehicle_type=CAPTURE_ELECTRIC};
    product_job_arm(&j,1,100,1,automatic);j.reference_rpm=555;
    for(unsigned i=101;i<105;++i)
        CHECK(product_job_tick(&j,i*1000, i,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,105000,105,1,true,true,false,true,552)==JOB_NONE);
    CHECK(product_job_tick(&j,106000,106,1,true,true,false,true,555)==JOB_NONE);
    CHECK(product_job_tick(&j,107000,107,1,true,true,false,true,558)==JOB_START);
    CHECK(j.capture.seconds==20&&product_capture_auto_seconds(180)==51&&product_capture_auto_seconds(20)==60);
    product_job_recording(&j,107000,107);
    CHECK(j.deadline_us==20107000);
    product_job_arm(&j,1,100,1,automatic);j.minimum_seconds=30;
    CHECK(product_job_tick(&j,105000,101,1,true,true,false,true,555)==JOB_NONE);
    CHECK(product_job_tick(&j,106000,102,1,true,true,false,true,555)==JOB_NONE);
    CHECK(product_job_tick(&j,107000,103,1,true,true,false,true,555)==JOB_START);
    CHECK(j.capture.seconds==30);
    return true;}
static bool bicycle_retains_coast_range(void){product_job_t j;arm_coast(&j,1,0,1);
    j.capture.mode=CAPTURE_TIMED;product_capture_apply_preset(&j.capture);
    CHECK(j.capture.mode==CAPTURE_COAST);
    CHECK(product_job_tick(&j,400000,2,1,true,true,false,true,500)==JOB_NONE);
    CHECK(product_job_tick(&j,600000,3,1,true,true,false,true,300)==JOB_NONE);
    CHECK(product_job_tick(&j,900000,4,1,true,true,false,true,299)==JOB_START);
    return true;}
static bool tail_matching_avoids_unmatched_startup_prefix(void){
    for(unsigned i=0;i<8;++i){a.turns[i].rpm=a.turns[i].previous_rpm=110;a.turns[i].real=i<5?1:2;a.turns[i].imag=0;}
    b.count=3;for(unsigned i=0;i<3;++i){b.turns[i].rpm=b.turns[i].previous_rpm=110;b.turns[i].real=3;b.turns[i].imag=0;}
    CHECK(product_solve_runs(&config,&a,&b,&result));CHECK(result.matched_turns==3);
    CHECK(fabs(result.original_peak-2)<1e-10&&fabs(result.trial_peak-3)<1e-10);return true;}
static bool timeout_sensor_epoch(void){product_job_t j;arm_coast(&j,1,10,1);
    CHECK(product_job_tick(&j,120000002,10,1,true,true,false,false,0)==JOB_FAIL&&j.state==PRODUCT_TIMEOUT);
    arm_coast(&j,1,10,1);CHECK(product_job_tick(&j,2,10,2,true,true,false,false,0)==JOB_FAIL&&j.reason==PRODUCT_CONTEXT);
    arm_coast(&j,1,10,1);CHECK(product_job_tick(&j,2,10,1,false,true,false,false,0)==JOB_FAIL&&j.reason==PRODUCT_SENSOR);return true;}
static bool bad_reference_and_loss(void){product_job_t j;arm_coast(&j,1,10,1);
    CHECK(product_job_tick(&j,2,9,1,true,true,false,false,0)==JOB_FAIL&&j.reason==PRODUCT_REFERENCE);
    arm_coast(&j,1,10,1);CHECK(product_job_tick(&j,2,10,1,true,true,true,false,0)==JOB_FAIL&&j.reason==PRODUCT_DATA_FAULT);return true;}
int main(void){
#define RUN(f) do{setup();if(!f())return 1;++passed;}while(0)
    RUN(full_estimated_flow);RUN(negative_improvement);RUN(phase_cancellation_is_not_false_improvement);RUN(weak_residual_keeps_amplitude_result);RUN(real_speed_mismatch_is_preserved);RUN(no_trial_response);RUN(epoch_and_overlap);RUN(faults_and_no_band);
    RUN(zero_baseline);RUN(near_zero_mean_with_large_turns_is_inconsistent);
    RUN(matching_maximizes_pairs_before_speed_error);RUN(matches_across_speed_band_boundary_without_reuse);
    RUN(weak_single_turn_is_not_automatic_failure);RUN(invalid_run_not_reused);RUN(bad_config);
    RUN(failed_larger_band_does_not_hide_usable_band);RUN(band_ranking_does_not_target_a_mass);RUN(equally_usable_bands_choose_tailward_usable_band);RUN(tail_matching_avoids_unmatched_startup_prefix);
    RUN(all_failed_bands_stay_failed);RUN(unselected_nonfinite_input_is_still_a_hard_fault);
    RUN(arming_and_stop);RUN(whole_coast_not_count_or_old_deadline);RUN(silence_remembers_slow_period_and_new_edge);RUN(explicit_finish_and_incomplete_limit);RUN(timed_vehicle_mode_has_exact_duration);RUN(bicycle_retains_coast_range);RUN(timeout_sensor_epoch);RUN(bad_reference_and_loss);
    RUN(automatic_duration_and_baseline_speed);
    printf("PASS product_engine_and_job cases=%u snapshot_bytes=%zu synthetic_only=1\n",passed,sizeof(product_snapshot_t));return 0;}
