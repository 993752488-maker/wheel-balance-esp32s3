#include "product_retest.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL %s:%d %s\n",__func__,__LINE__,#c);return false;}}while(0)
static product_run_t a,b;
static product_config_t config;
static product_solution_t solution;
static product_snapshot_t snapshot;
static product_retest_log_t logbook;
static void setup(double scale,double phase)
{
    memset(&a,0,sizeof(a));memset(&b,0,sizeof(b));config=product_default_config();
    solution=(product_solution_t){.calculation_valid=true,.engineering_available=true,.lower_rpm=100,.upper_rpm=120,
        .calculation={.mass_g=5,.angle_valid=true,.angle_deg=90}};
    a.summary=(product_stage_summary_t){.captured=true,.integrity_valid=true,.epoch=1,.capture_id=3,.start_us=1,.end_us=9000000};
    b.summary=a.summary;b.summary.capture_id=4;b.summary.start_us=10000001;b.summary.end_us=19000000;
    a.count=b.count=8;
    for(unsigned i=0;i<8;++i){a.turns[i]=(product_turn_t){116-i*2,118-i*2,1,0,i+2,1000000+i*800000};
        b.turns[i]=a.turns[i];b.turns[i].reference+=10;b.turns[i].time_us+=10000000;
        b.turns[i].real=scale*cos(phase);b.turns[i].imag=scale*sin(phase);}
    memset(&snapshot,0,sizeof(snapshot));snapshot.schema=1;snapshot.id=42;snapshot.revision=7;
    strcpy(snapshot.boot_id,"1234567890abcdef");strcpy(snapshot.firmware,"0.5.8-test");snapshot.config=config;
    snapshot.stage=PRODUCT_RETEST;snapshot.state=PRODUCT_COMPLETE;
    snapshot.stages[0].capture_id=1;snapshot.stages[1].capture_id=2;snapshot.stages[2]=a.summary;
    snapshot.comparison=(product_comparison_t){.available=true,.matched_turns=8,.original_peak=1,.retest_peak=.5,.improvement_percent=50,.lower_rpm=100,.upper_rpm=120};
    product_retest_init(&logbook,&snapshot);
}
static product_repeat_observation_t observe(void)
{product_repeat_observation_t o;product_observe_repeat(&config,&a,&b,&solution,&o);return o;}
static bool identical(void){setup(1,0);product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_OBSERVATION_CLOSE&&o.matched_turns==8);CHECK(fabs(o.amplitude_change_percent)<1e-12);return true;}
static bool large_amplitude(void){setup(1.7,0);product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_VARIABLE);CHECK(fabs(o.amplitude_change_percent-70)<1e-10);return true;}
static bool phase_change(void){setup(1,1.5707963267948966);product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_VARIABLE&&o.phase_available);CHECK(fabs(o.phase_change_deg-90)<1e-10);return true;}
static bool warning_is_symmetric(void){setup(.7,0);product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_VARIABLE);CHECK(o.amplitude_change_percent<0);return true;}
static bool near_floor(void){setup(1e-7,0);product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_UNCOMPARABLE&&o.reason==PRODUCT_BASELINE_ZERO);return true;}
static bool previous_rpm_needed(void){setup(1,0);for(unsigned i=0;i<8;++i)b.turns[i].previous_rpm*=1.3;product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_UNCOMPARABLE&&o.reason==PRODUCT_NO_COMMON_SPEED);return true;}
static bool epoch_guard(void){setup(1,0);b.summary.epoch=2;CHECK(observe().state==PRODUCT_REPEAT_UNCOMPARABLE);return true;}
static bool overlap_guard(void){setup(1,0);b.summary.start_us=a.summary.end_us-1;CHECK(observe().state==PRODUCT_REPEAT_UNCOMPARABLE);return true;}
static bool nonfinite_guard(void){setup(1,0);b.turns[2].real=NAN;CHECK(observe().reason==PRODUCT_NUMERIC);return true;}
static bool no_phase_certificate(void){setup(1,0);for(unsigned i=0;i<8;++i)b.turns[i].real=i%2?1:-1;product_repeat_observation_t o=observe();CHECK(o.state==PRODUCT_REPEAT_OBSERVATION_CLOSE&&!o.phase_available);CHECK(o.reason==PRODUCT_LOW_DIRECTION_COHERENCE);return true;}
static bool frozen_band(void){setup(1,0);solution.lower_rpm=200;solution.upper_rpm=220;CHECK(observe().reason==PRODUCT_NO_COMMON_SPEED);return true;}
static bool only_comparable_corrections_can_start_or_retry(void)
{
    setup(1,0);
    snapshot.solution=solution;snapshot.stages[0]=a.summary;snapshot.stages[1]=b.summary;
    snapshot.stage=PRODUCT_TRIAL;CHECK(product_retest_available(&snapshot));
    snapshot.stage=PRODUCT_RETEST;snapshot.state=PRODUCT_RECORDING;CHECK(!product_retest_available(&snapshot));
    snapshot.state=PRODUCT_NO_RESULT;snapshot.reason=PRODUCT_NO_COMMON_SPEED;CHECK(product_retest_available(&snapshot));
    snapshot.reason=PRODUCT_TOO_FEW_TURNS;CHECK(product_retest_available(&snapshot));
    snapshot.reason=PRODUCT_SENSOR;CHECK(!product_retest_available(&snapshot));
    snapshot.reason=PRODUCT_DATA_FAULT;CHECK(!product_retest_available(&snapshot));
    snapshot.reason=PRODUCT_CONTEXT;CHECK(!product_retest_available(&snapshot));
    snapshot.state=PRODUCT_TIMEOUT;CHECK(product_retest_available(&snapshot));
    snapshot.historical=true;CHECK(!product_retest_available(&snapshot));snapshot.historical=false;
    snapshot.solution.engineering_available=false;CHECK(!product_retest_available(&snapshot));
    snapshot.solution=solution;snapshot.solution.calculation.mass_g=0;CHECK(!product_retest_available(&snapshot));
    snapshot.solution=solution;snapshot.solution.calculation.angle_deg=NAN;CHECK(!product_retest_available(&snapshot));
    snapshot.solution=solution;snapshot.stages[1].integrity_valid=false;CHECK(!product_retest_available(&snapshot));
    product_comparison_t comparison;
    solution.reason=PRODUCT_CONFLICT;solution.engineering_available=false;snapshot.solution=solution;
    CHECK(!product_compare_runs(&config,&a,&b,&solution,&comparison)&&comparison.reason==PRODUCT_CONFLICT&&!comparison.available);
    const product_repeat_observation_t repeat=observe();
    CHECK(repeat.state==PRODUCT_REPEAT_UNCOMPARABLE&&repeat.reason==PRODUCT_CONFLICT);
    return true;
}
static bool first_record(void){setup(1,0);CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));CHECK(logbook.count==1&&logbook.entries[0].ordinal==1);CHECK(product_retest_valid(&logbook,&snapshot));return true;}
static bool explicit_condition(void){setup(1,0);CHECK(!product_retest_can_start(&logbook,RETEST_CONDITION_SAME));CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));CHECK(!product_retest_can_start(&logbook,RETEST_CONDITION_NONE));CHECK(!product_retest_can_start(&logbook,RETEST_CONDITION_FIRST));CHECK(product_retest_can_start(&logbook,RETEST_CONDITION_SAME));return true;}
static bool changed_correction_separates_groups(void){setup(1.7,0);CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));snapshot.stages[2]=b.summary;product_repeat_observation_t o=observe();CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_CHANGED,&o));CHECK(logbook.entries[1].correction_group==2&&logbook.entries[1].repeat.state==PRODUCT_REPEAT_UNASSESSED);return true;}
static bool retain_bounded_rows(void){setup(1,0);for(unsigned i=0;i<8;++i){snapshot.stages[2].capture_id=3+i;snapshot.stages[2].start_us=1+i*10000000ULL;snapshot.stages[2].end_us=9000000+i*10000000ULL;CHECK(product_retest_record(&logbook,&snapshot,i?RETEST_CONDITION_SAME:RETEST_CONDITION_FIRST,NULL));}CHECK(logbook.count==6&&logbook.total_count==8&&logbook.dropped_count==2);CHECK(logbook.entries[0].ordinal==3&&logbook.entries[5].ordinal==8);CHECK(product_retest_valid(&logbook,&snapshot));return true;}
static bool failed_capture_preserved(void){setup(1,0);snapshot.stages[2].captured=false;snapshot.stages[2].integrity_valid=false;snapshot.stages[2].end_us=0;snapshot.stages[2].reason=PRODUCT_DATA_FAULT;snapshot.comparison.available=false;CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));CHECK(!logbook.entries[0].stage.captured&&product_retest_valid(&logbook,&snapshot));return true;}
static bool waiting_cancel_not_a_capture(void){setup(1,0);snapshot.stages[2].capture_id=0;CHECK(!product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));CHECK(logbook.count==0);return true;}
static bool duplicate_not_appended(void){setup(1,0);CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));CHECK(!product_retest_record(&logbook,&snapshot,RETEST_CONDITION_SAME,NULL));CHECK(logbook.count==1);return true;}
static bool binding_guards(void){setup(1,0);CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));++snapshot.revision;CHECK(!product_retest_valid(&logbook,&snapshot));--snapshot.revision;snapshot.config.axis=2;CHECK(!product_retest_valid(&logbook,&snapshot));snapshot.config.axis=1;snapshot.boot_id[0]='a';CHECK(!product_retest_valid(&logbook,&snapshot));return true;}
static bool legacy_only_latest(void){setup(1,0);product_retest_import_latest(&logbook,&snapshot);CHECK(product_retest_valid(&logbook,&snapshot));CHECK(logbook.count==1&&logbook.legacy_count_unknown&&logbook.entries[0].legacy&&logbook.entries[0].ordinal==0);CHECK(logbook.entries[0].repeat.state==PRODUCT_REPEAT_UNASSESSED);return true;}
static bool reject_nonfinite_without_mutation(void){setup(1,0);snapshot.comparison.improvement_percent=NAN;CHECK(!product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));CHECK(logbook.count==0&&product_retest_valid(&logbook,&snapshot));return true;}
static bool variable_group_is_not_erased_by_later_close_pair(void){setup(1.7,0);CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_FIRST,NULL));snapshot.stages[2]=b.summary;product_repeat_observation_t o=observe();CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_SAME,&o));CHECK(logbook.current_group_variable);for(unsigned i=0;i<7;++i){snapshot.stages[2].capture_id=5+i;snapshot.stages[2].start_us=20000001+i*10000000ULL;snapshot.stages[2].end_us=29000000+i*10000000ULL;o.state=PRODUCT_REPEAT_OBSERVATION_CLOSE;CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_SAME,&o));}CHECK(logbook.current_group_variable);snapshot.stages[2].capture_id=20;snapshot.stages[2].start_us=100000001;snapshot.stages[2].end_us=109000000;CHECK(product_retest_record(&logbook,&snapshot,RETEST_CONDITION_CHANGED,NULL));CHECK(!logbook.current_group_variable);return true;}
int main(void)
{
    bool (*tests[])(void)={only_comparable_corrections_can_start_or_retry,identical,large_amplitude,phase_change,warning_is_symmetric,near_floor,previous_rpm_needed,epoch_guard,overlap_guard,nonfinite_guard,no_phase_certificate,frozen_band,first_record,explicit_condition,changed_correction_separates_groups,retain_bounded_rows,failed_capture_preserved,waiting_cancel_not_a_capture,duplicate_not_appended,binding_guards,legacy_only_latest,reject_nonfinite_without_mutation,variable_group_is_not_erased_by_later_close_pair};
    for(unsigned i=0;i<sizeof(tests)/sizeof(tests[0]);++i)if(!tests[i]())return 1;
    printf("PASS retest cases=%u snapshot_bytes=%zu sidecar_bytes=%zu synthetic_only=1\n",(unsigned)(sizeof(tests)/sizeof(tests[0])),sizeof(snapshot),sizeof(logbook));return 0;
}
