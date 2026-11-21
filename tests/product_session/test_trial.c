/* Synthetic state transitions for the production trial guide, no hardware. */
#include "product_trial.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(v) do{if(!(v)){fprintf(stderr,"FAIL %s:%d %s\n",__func__,__LINE__,#v);return false;}}while(0)
static product_snapshot_t s;
static product_trial_guide_t g;
static unsigned passed;
static void setup(void)
{
    memset(&s,0,sizeof(s));s.schema=1;s.id=42;s.revision=7;strcpy(s.boot_id,"0000000000000001");strcpy(s.firmware,"0.5.6");
    s.config=product_default_config();s.config.mass_g=50;s.state=PRODUCT_COMPLETE;
    s.stages[0]=(product_stage_summary_t){.captured=true,.integrity_valid=true,.capture_id=1,.epoch=2,.start_us=1000000,.end_us=9000000};
    (void)product_trial_init(&g,&s,5,true);
}
static void trial(unsigned capture,product_reason_t reason)
{
    s.stage=PRODUCT_TRIAL;s.reason=reason;s.state=reason==PRODUCT_REASON_NONE?PRODUCT_COMPLETE:PRODUCT_NO_RESULT;
    s.stages[1]=(product_stage_summary_t){.captured=true,.integrity_valid=true,.capture_id=capture,.epoch=2,
        .start_us=1000000+(capture-1)*10000000ULL,.end_us=9000000+(capture-1)*10000000ULL,.reason=reason};
    memset(&s.solution,0,sizeof(s.solution));s.solution.reason=reason;
}
static bool fresh_mass_is_not_previous_50g(void)
{CHECK(s.config.mass_g==10&&g.initial_mass_g==10&&g.piece_mass_g==5);CHECK(g.source==TRIAL_PLAN_STARTER);CHECK(product_trial_valid(&g,&s));return true;}
static bool baseline_first(void)
{product_trial_advice_t a;s.stages[0].captured=false;product_trial_advice(&g,&s,&a);CHECK(!a.editable&&a.reason==TRIAL_NEXT_NO_BASELINE);CHECK(!product_trial_set_plan(&g,&s,50,TRIAL_PLAN_MANUAL));return true;}
static bool actual_confirmation_matches_plan(void)
{CHECK(product_trial_can_start(&g,&s,10));CHECK(!product_trial_can_start(&g,&s,50));CHECK(!product_trial_can_start(&g,&s,NAN));return true;}
static bool manual_change_keeps_baseline(void)
{product_stage_summary_t baseline=s.stages[0];CHECK(product_trial_set_plan(&g,&s,50,TRIAL_PLAN_MANUAL));CHECK(s.config.mass_g==50&&g.source==TRIAL_PLAN_MANUAL);
 CHECK(s.id==42&&s.revision==7&&!memcmp(&baseline,&s.stages[0],sizeof(baseline)));CHECK(product_trial_can_start(&g,&s,50));return true;}
static bool failed_trial_is_not_overwritten(void)
{trial(2,PRODUCT_WEAK_RESPONSE);s.solution.matched_turns=3;s.solution.delta_peak=.01;s.solution.delta_coherence=.65;s.solution.mean_noise_scale=.0068;
 CHECK(product_trial_record(&g,&s));CHECK(!product_trial_record(&g,&s));CHECK(product_trial_set_plan(&g,&s,50,TRIAL_PLAN_MANUAL));
 CHECK(g.count==1&&g.attempts[0].mass_g==10&&g.attempts[0].stage.reason==PRODUCT_WEAK_RESPONSE);CHECK(s.stages[0].capture_id==1);return true;}
static bool supported_projection_is_quantized_and_bounded(void)
{trial(2,PRODUCT_WEAK_RESPONSE);s.solution.matched_turns=3;s.solution.delta_peak=.01;s.solution.delta_coherence=.65;s.solution.mean_noise_scale=.0068;
 CHECK(product_trial_record(&g,&s));product_trial_advice_t a;product_trial_advice(&g,&s,&a);CHECK(a.retry_available&&a.suggested&&a.mass_g==15);
 CHECK(!product_trial_set_plan(&g,&s,20,TRIAL_PLAN_RESPONSE));CHECK(product_trial_set_plan(&g,&s,15,TRIAL_PLAN_RESPONSE));CHECK(s.config.mass_g==15);return true;}
static bool inconsistent_direction_has_no_mass_guess(void)
{trial(2,PRODUCT_INCONSISTENT);s.solution.matched_turns=8;s.solution.delta_peak=.01;s.solution.delta_coherence=.2;s.solution.mean_noise_scale=.0068;
 CHECK(product_trial_record(&g,&s));product_trial_advice_t a;product_trial_advice(&g,&s,&a);CHECK(a.retry_available&&!a.suggested&&a.reason==TRIAL_NEXT_NO_PREDICTION);return true;}
static bool no_common_speed_has_no_mass_guess(void)
{trial(2,PRODUCT_NO_COMMON_SPEED);CHECK(product_trial_record(&g,&s));product_trial_advice_t a;product_trial_advice(&g,&s,&a);CHECK(a.retry_available&&!a.suggested);return true;}
static bool hard_fault_is_not_a_mass_problem(void)
{trial(2,PRODUCT_DATA_FAULT);s.stages[1].integrity_valid=false;CHECK(product_trial_record(&g,&s));product_trial_advice_t a;product_trial_advice(&g,&s,&a);
 CHECK(!a.editable&&!a.suggested&&a.reason==TRIAL_NEXT_HARD_FAULT);CHECK(!product_trial_set_plan(&g,&s,100,TRIAL_PLAN_MANUAL));return true;}
static bool success_stops_escalation(void)
{trial(2,PRODUCT_REASON_NONE);s.solution.calculation_valid=true;CHECK(product_trial_record(&g,&s));product_trial_advice_t a;product_trial_advice(&g,&s,&a);CHECK(!a.editable&&!a.suggested&&a.reason==TRIAL_NEXT_FINISHED);return true;}
static bool attempt_limit_preserves_all_sources(void)
{for(unsigned i=0;i<3;++i){CHECK(product_trial_set_plan(&g,&s,10*(i+1),TRIAL_PLAN_MANUAL));trial(i+2,PRODUCT_WEAK_RESPONSE);CHECK(product_trial_record(&g,&s));}
 product_trial_advice_t a;product_trial_advice(&g,&s,&a);CHECK(!a.editable&&a.reason==TRIAL_NEXT_LIMIT);CHECK(g.attempts[0].mass_g==10&&g.attempts[1].mass_g==20&&g.attempts[2].mass_g==30);CHECK(product_trial_valid(&g,&s));return true;}
static bool excessive_or_nonfinite_projection_is_withheld(void)
{trial(2,PRODUCT_WEAK_RESPONSE);s.solution.matched_turns=3;s.solution.delta_peak=.00001;s.solution.delta_coherence=.8;s.solution.mean_noise_scale=.1;
 CHECK(product_trial_record(&g,&s));product_trial_advice_t a;product_trial_advice(&g,&s,&a);CHECK(!a.suggested);g.attempts[0].solution.mean_noise_scale=NAN;product_trial_advice(&g,&s,&a);CHECK(!a.suggested);return true;}
static bool stale_sidecar_rejected(void)
{CHECK(product_trial_valid(&g,&s));g.snapshot_revision++;CHECK(!product_trial_valid(&g,&s));g.snapshot_revision=s.revision;g.session_id++;CHECK(!product_trial_valid(&g,&s));g.session_id=s.id;strcpy(g.boot_id,"0000000000000002");CHECK(!product_trial_valid(&g,&s));return true;}
static bool attempt_axis_is_retained_and_legacy_padding_is_ignored(void)
{
    s.config.axis=0;trial(2,PRODUCT_WEAK_RESPONSE);CHECK(product_trial_record(&g,&s));
    s.config.axis=1;trial(3,PRODUCT_REASON_NONE);CHECK(product_trial_record(&g,&s));
    CHECK(g.schema==3&&g.attempts[0].axis==0&&g.attempts[1].axis==1);
    CHECK(product_trial_restore(&g,&s,sizeof(g)));
    g.attempts[0].axis=UINT32_MAX;CHECK(!product_trial_restore(&g,&s,sizeof(g)));
    g.schema=2;CHECK(product_trial_restore(&g,&s,sizeof(g)));
    return true;
}
static bool history_and_synthetic_are_readonly(void)
{product_trial_advice_t a;s.historical=true;product_trial_advice(&g,&s,&a);CHECK(!a.editable);s.historical=false;s.simulated=true;CHECK(!product_trial_set_plan(&g,&s,20,TRIAL_PLAN_MANUAL));trial(2,PRODUCT_WEAK_RESPONSE);CHECK(!product_trial_record(&g,&s));return true;}
static bool running_plan_is_locked(void)
{s.state=PRODUCT_ARMED;CHECK(!product_trial_set_plan(&g,&s,20,TRIAL_PLAN_MANUAL));s.state=PRODUCT_RECORDING;CHECK(!product_trial_can_start(&g,&s,10));return true;}
static bool cancel_before_capture_does_not_spend_attempt(void)
{s.stage=PRODUCT_TRIAL;s.state=PRODUCT_CANCELLED;s.reason=PRODUCT_REASON_NONE;CHECK(!product_trial_record(&g,&s));CHECK(g.count==0&&product_trial_can_start(&g,&s,10));return true;}
static bool explicit_manual_initial_plan(void)
{s.config.mass_g=50;CHECK(product_trial_init(&g,&s,5,false));CHECK(s.config.mass_g==50&&g.initial_mass_g==50&&g.source==TRIAL_PLAN_MANUAL);CHECK(!product_trial_init(&g,&s,NAN,true));return true;}
int main(void)
{
    _Static_assert(sizeof(product_snapshot_t)==752,"Do not break old saved summaries");
#define RUN(f) do{setup();if(!f())return 1;++passed;}while(0)
    RUN(fresh_mass_is_not_previous_50g);RUN(baseline_first);RUN(actual_confirmation_matches_plan);RUN(manual_change_keeps_baseline);
    RUN(failed_trial_is_not_overwritten);RUN(supported_projection_is_quantized_and_bounded);RUN(inconsistent_direction_has_no_mass_guess);
    RUN(no_common_speed_has_no_mass_guess);RUN(hard_fault_is_not_a_mass_problem);RUN(success_stops_escalation);
    RUN(attempt_limit_preserves_all_sources);RUN(excessive_or_nonfinite_projection_is_withheld);RUN(stale_sidecar_rejected);
    RUN(attempt_axis_is_retained_and_legacy_padding_is_ignored);
    RUN(history_and_synthetic_are_readonly);RUN(running_plan_is_locked);RUN(cancel_before_capture_does_not_spend_attempt);RUN(explicit_manual_initial_plan);
    printf("PASS product_trial cases=%u snapshot_bytes=%zu guide_bytes=%zu synthetic_state_tests=1\n",passed,sizeof(s),sizeof(g));return 0;
}
