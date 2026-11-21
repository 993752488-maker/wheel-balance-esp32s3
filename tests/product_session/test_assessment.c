#include "product_assessment.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(v) do{if(!(v)){fprintf(stderr,"FAIL %s:%d %s\n",__func__,__LINE__,#v);return false;}}while(0)
static unsigned passed;
static product_run_t a[3],b[3];
static product_config_t cfg;
static product_solution_t result;
static product_assessment_t review;
static void empty(product_run_t *r,unsigned stage)
{
    memset(r,0,sizeof(*r));r->summary=(product_stage_summary_t){.captured=true,.integrity_valid=true,.epoch=1,
        .capture_id=stage+1,.start_us=1000000+(uint64_t)stage*30000000,.end_us=29000000+(uint64_t)stage*30000000};
}
static void band(product_run_t *original,product_run_t *trial,double rpm,unsigned n,double mass,double angle,double dr,double di)
{
    const double phase=angle*3.14159265358979323846/180;
    const double vr=-(mass/cfg.mass_g)*(dr*cos(phase)-di*sin(phase));
    const double vi=-(mass/cfg.mass_g)*(dr*sin(phase)+di*cos(phase));
    for(unsigned i=0;i<n;++i){size_t j=original->count++;
        original->turns[j]=(product_turn_t){.rpm=rpm-i*.1,.previous_rpm=rpm+.1-i*.1,.real=vr,.imag=vi,
            .reference=j+2,.time_us=original->summary.start_us+(j+1)*800000};
        trial->turns[trial->count]=original->turns[j];trial->turns[trial->count].real+=dr;trial->turns[trial->count].imag+=di;
        trial->turns[trial->count].reference+=100;trial->turns[trial->count].time_us=trial->summary.start_us+(j+1)*800000;++trial->count;
    }
}
static void setup(void)
{
    cfg=product_default_config();cfg.mass_g=10;cfg.axis=0;
    for(unsigned k=0;k<3;++k){empty(&a[k],0);empty(&b[k],1);}
    memset(&review,0,sizeof(review));memset(&result,0,sizeof(result));
}
static uint32_t noise_seed=0x9625ad71U;
static double noise(void)
{noise_seed^=noise_seed<<13;noise_seed^=noise_seed>>17;noise_seed^=noise_seed<<5;return 2*(noise_seed>>8)/16777216.0-1;}
static void model_band(double rpm,unsigned n,double hr,double hi,double scatter)
{
    const size_t offset=a[0].count;band(&a[0],&b[0],rpm,n,4,120,1,0);
    const double p=3.14159265358979323846/180,ur=4*cfg.correction_radius_mm*cos(120*p),ui=4*cfg.correction_radius_mm*sin(120*p);
    const double tr=cfg.mass_g*cfg.trial_radius_mm*cos(cfg.trial_angle_deg*p),ti=cfg.mass_g*cfg.trial_radius_mm*sin(cfg.trial_angle_deg*p);
    for(size_t j=offset;j<a[0].count;++j){
        a[0].turns[j].real=-hr*ur+hi*ui+scatter*noise();a[0].turns[j].imag=-hr*ui-hi*ur+scatter*noise();
        b[0].turns[j].real=-hr*ur+hi*ui+hr*tr-hi*ti+scatter*noise();b[0].turns[j].imag=-hr*ui-hi*ur+hr*ti+hi*tr+scatter*noise();
    }
}
static bool joint_fit_respects_trial_angle_radius_and_direction(void)
{
    cfg.mass_g=15;cfg.trial_radius_mm=300;cfg.correction_radius_mm=125;cfg.trial_angle_deg=87;cfg.direction=-1;
    model_band(210,8,.002,.001,0);model_band(150,8,-.003,.004,0);model_band(90,8,.001,-.0005,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));CHECK(review.joint_used&&review.joint_bands==3);
    CHECK(fabs(result.calculation.mass_g-4)<1e-8&&fabs(result.calculation.angle_deg-120)<1e-8);return true;
}
static bool noisy_responses_are_less_sensitive_than_weak_tail_ratio(void)
{
    /* Controlled seeded numerical comparison, not measured sensor noise or
     * a physical precision claim. Truth is used ONLY by this test oracle. */
    double old_error=0,new_error=0;unsigned compared=0;noise_seed=0x9625ad71U;
    for(unsigned experiment=0;experiment<80;++experiment){
        setup();model_band(210,8,.005,.001,.15);model_band(130,8,.001,-.0005,.15);model_band(50,8,.0003,.0001,.15);
        product_solution_t tail;product_band_detail_t detail;
        if(!product_evaluate_band(&cfg,&a[0],&b[0],40,&tail,&detail))continue;
        CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
        const double ur=4*cfg.correction_radius_mm*cos(120*3.14159265358979323846/180),ui=4*cfg.correction_radius_mm*sin(120*3.14159265358979323846/180);
        old_error+=pow(hypot(tail.calculation.correction_g_mm.real-ur,tail.calculation.correction_g_mm.imag-ui)/cfg.correction_radius_mm,2);
        new_error+=pow(hypot(result.calculation.correction_g_mm.real-ur,result.calculation.correction_g_mm.imag-ui)/cfg.correction_radius_mm,2);++compared;
    }
    CHECK(compared>=60&&new_error<old_error*.5625);
    printf("SYNTHETIC seeded_noise cases=%u weak_tail_vector_RMSE_g=%.6f new_fit_vector_RMSE_g=%.6f physical_accuracy_verified=0\n",compared,sqrt(old_error/compared),sqrt(new_error/compared));return true;
}
static bool complex_joint_fit_reduces_band_bias(void)
{
    band(&a[0],&b[0],150,8,4.3,120,1,.3);band(&a[0],&b[0],110,8,3.7,120,-.3,1);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(review.joint_used&&review.joint_bands==2);
    CHECK(fabs(result.calculation.mass_g-4)<1e-8&&fabs(result.calculation.angle_deg-120)<1e-8);
    CHECK(fabs(review.bands[review.selected_band].mass_g-4)>.29);
    CHECK(!result.accuracy_validated&&!result.calculation.recommendation_valid&&!result.calculation.simulated);
    CHECK(review.state==ASSESS_CLOSE);return true;
}
static bool rpm_dependent_phase_and_gain_cancel(void)
{
    band(&a[0],&b[0],210,9,4,120,.7,1);band(&a[0],&b[0],150,7,4,120,-.4,.5);band(&a[0],&b[0],90,6,4,120,.2,-.1);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(review.joint_bands==3&&fabs(result.calculation.mass_g-4)<1e-8&&fabs(result.calculation.angle_deg-120)<1e-8);
    CHECK(review.model_residual_ratio<1e-10);return true;
}
static bool contradictory_band_is_not_averaged_silently(void)
{
    band(&a[0],&b[0],190,8,5,0,1,0);band(&a[0],&b[0],130,8,15,0,.3,0);band(&a[0],&b[0],70,8,20,0,.2,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(review.state==ASSESS_VARIABLE&&(review.flags&ASSESS_BANDS_DIFFERENT));
    CHECK(review.mass_min_g<5.000001&&review.mass_max_g>19.99999);
    /* Two agreeing bands (15/20) now outweigh the isolated 5g band. */
    CHECK(review.joint_used&&review.joint_bands==2&&result.engineering_available);
    CHECK(fabs(result.calculation.mass_g-17.5)<1e-8);return true;
}
static bool stronger_reproducible_band_beats_late_weak_tail(void)
{
    band(&a[0],&b[0],150,8,4,120,1,0);band(&a[0],&b[0],50,3,4,120,.05,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));CHECK(result.lower_rpm==140);return true;
}
static bool split_detects_a_collinear_but_variable_response(void)
{
    band(&a[0],&b[0],150,8,15,0,1,0);
    for(size_t i=4;i<8;++i){a[0].turns[i].real=-4;b[0].turns[i].real=-3;}
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(review.state==ASSESS_VARIABLE&&(review.flags&ASSESS_SPLIT_DIFFERENT));
    CHECK(result.engineering_available);return true;
}
static bool three_turns_remain_an_explicit_limited_estimate(void)
{
    band(&a[0],&b[0],110,3,4,120,1,0);CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(review.state==ASSESS_LIMITED&&(review.flags&ASSESS_NO_SPLIT)&&result.engineering_available);return true;
}
static bool trial_response_resolves_axis_in_one_capture(void)
{
    band(&a[0],&b[0],110,8,4,120,1,0);
    band(&a[1],&b[1],110,8,4,120,.2,.1);
    band(&a[2],&b[2],110,8,4,120,.02,.01);
    for(size_t i=0;i<b[1].count;++i){b[1].turns[i].real+=i%2?.08:-.08;}
    for(size_t i=0;i<b[2].count;++i){b[2].turns[i].real=a[2].turns[i].real;b[2].turns[i].imag=a[2].turns[i].imag;}
    unsigned axis=2;CHECK(product_assess_axes(&cfg,a,b,&axis,&result,&review));CHECK(axis==0&&review.axis_finalized&&review.automatic_axis);
    CHECK(review.axis_scores[0]>review.axis_scores[1]);CHECK(fabs(result.calculation.mass_g-4)<1e-8);return true;
}
static bool incompatible_axes_are_visible_not_blended(void)
{
    band(&a[0],&b[0],110,8,4,120,1,0);band(&a[1],&b[1],110,8,16,120,.5,0);
    a[2].summary.captured=b[2].summary.captured=false;unsigned axis;
    CHECK(product_assess_axes(&cfg,a,b,&axis,&result,&review));CHECK(review.state==ASSESS_VARIABLE&&(review.flags&ASSESS_AXES_DIFFERENT));
    CHECK(result.calculation_valid && !result.engineering_available && result.reason==PRODUCT_CONFLICT);
    return true;
}
static bool internally_conflicted_axis_cannot_break_a_tie(void)
{
    band(&a[0],&b[0],130,6,5,0,.5,0);
    band(&a[0],&b[0],110,6,18,0,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(result.reason==PRODUCT_CONFLICT&&review.axis_conflict_mask==1);
    band(&a[1],&b[1],110,8,18,0,1,0);
    band(&a[2],&b[2],110,8,5,0,1,0);
    unsigned axis;
    CHECK(product_assess_axes(&cfg,a,b,&axis,&result,&review));
    CHECK(result.calculation_valid&&!result.engineering_available&&result.reason==PRODUCT_CONFLICT);
    CHECK(review.axis_available_mask==7&&review.axis_conflict_mask==1);
    CHECK(review.flags&ASSESS_AXES_DIFFERENT);
    a[2].summary.captured=b[2].summary.captured=false;
    CHECK(product_assess_axes(&cfg,a,b,&axis,&result,&review));
    CHECK(axis==1&&result.engineering_available&&review.axis_conflict_mask==1);
    CHECK(review.flags&ASSESS_AXES_DIFFERENT);
    return true;
}
static bool near_zero_original_stays_zero_across_bands(void)
{
    band(&a[0],&b[0],130,6,2.5,0,2e-5,0);
    band(&a[0],&b[0],110,6,2.5,0,2e-5,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(review.valid_bands==2&&review.joint_bands==1&&!review.joint_used);
    CHECK(result.calculation.mass_g==0&&!result.calculation.angle_valid&&!result.engineering_available);
    CHECK(result.reason==PRODUCT_REASON_NONE&&review.correction_real_g_mm==0&&review.correction_imag_g_mm==0);
    return true;
}
static bool high_gain_cannot_dominate_and_band_tie_cannot_direct(void)
{
    band(&a[0],&b[0],110,8,10,90,.01,0);
    band(&a[0],&b[0],150,8,12,90,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(result.engineering_available && fabs(result.calculation.mass_g-11)<1e-8);
    setup();
    band(&a[0],&b[0],110,8,18,120,1,0);
    band(&a[0],&b[0],150,8,6,120,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(result.calculation_valid && !result.engineering_available && result.reason==PRODUCT_CONFLICT);
    return true;
}
static bool unselected_axis_fault_still_blocks_everything(void)
{
    band(&a[0],&b[0],110,8,4,120,1,0);band(&a[1],&b[1],110,8,4,120,.5,0);band(&a[2],&b[2],110,8,4,120,.1,0);
    b[2].turns[0].real=NAN;CHECK(!product_assess_axes(&cfg,a,b,NULL,&result,&review));CHECK(result.reason==PRODUCT_NUMERIC&&!result.engineering_available);return true;
}
static bool intermediate_mass_cannot_join_incompatible_groups(void)
{
    const double masses[]={6,9,13};
    for(unsigned i=0;i<3;++i)band(&a[0],&b[0],170-20*i,8,masses[i],25,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    printf("SYNTHETIC bridge_6_9_13 mass_g=%.4f eligible=%u fit_bands=%u\n",
           result.calculation.mass_g,(unsigned)result.engineering_available,review.joint_bands);
    CHECK(result.calculation_valid&&!result.engineering_available&&result.reason==PRODUCT_CONFLICT);
    CHECK(review.joint_bands<=2&&(review.flags&ASSESS_BANDS_DIFFERENT)&&!review.axis_finalized);
    product_snapshot_t saved={.id=10,.revision=3,.config=cfg,.solution=result};
    strcpy(saved.boot_id,"saved-bridge");saved.stages[0]=a[0].summary;saved.stages[1]=b[0].summary;
    product_assessment_bind(&review,&saved);CHECK(product_assessment_valid(&review,&saved));
    review.axis_finalized=1;CHECK(!product_assessment_valid(&review,&saved));
    review.policy=3;CHECK(product_assessment_valid(&review,&saved)); /* Older software froze the diagnostic axis. */
    for(unsigned shift=0;shift<3;++shift){
        setup();
        for(unsigned i=0;i<3;++i)band(&a[i],&b[i],110,8,masses[(i+shift)%3],25,1,0);
        CHECK(product_assess_axes(&cfg,a,b,NULL,&result,&review));
        CHECK(result.reason==PRODUCT_CONFLICT&&!result.engineering_available&&!review.axis_finalized);
        CHECK(review.axis_available_mask==7&&(review.flags&ASSESS_AXES_DIFFERENT));
    }
    return true;
}
static bool decisive_group_or_strong_common_core_keeps_guidance(void)
{
    const double dominant[]={6,6,6,9,13};
    for(unsigned i=0;i<5;++i)band(&a[0],&b[0],190-20*i,8,dominant[i],25,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(result.engineering_available&&review.joint_bands==4&&fabs(result.calculation.mass_g-6.75)<1e-8);
    CHECK(!(review.joint_band_mask&(1U<<4))); /* 110 RPM belongs to band index 4. */
    setup();
    const double common[]={6,9,9,9,13};
    for(unsigned i=0;i<5;++i)band(&a[0],&b[0],190-20*i,i==0||i==4?3:8,common[i],25,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(result.engineering_available&&review.joint_bands==3&&fabs(result.calculation.mass_g-9)<1e-8);
    CHECK(review.flags&ASSESS_BANDS_DIFFERENT);
    return true;
}
static bool wrapped_angles_and_full_band_capacity(void)
{
    for(unsigned i=0;i<3;++i)band(&a[i],&b[i],110,8,6,(345+15*i)%360,1,0);
    CHECK(product_assess_axes(&cfg,a,b,NULL,&result,&review));
    CHECK(result.reason==PRODUCT_CONFLICT&&!result.engineering_available);
    setup();
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i)
        band(&a[0],&b[0],290-20*i,6,6,i%2?359:1,1,0);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    CHECK(result.engineering_available&&review.joint_bands==PRODUCT_ASSESSMENT_BANDS);
    CHECK(fabs(result.calculation.mass_g-6*cos(3.14159265358979323846/180))<1e-8);
    return true;
}
static bool bounded_groups_match_exhaustive_reference(void)
{
    /* Independent seven-band subset enumeration checks both search passes,
     * the competing-group intersection and the actual fit mask. */
    enum { count=7, sets=1<<count };
    unsigned seed=0x70616972U;
    const double rad=3.14159265358979323846/180;
    for(unsigned run=0;run<96;++run){
        setup();
        for(unsigned i=0;i<count;++i){
            seed=1664525U*seed+1013904223U;
            band(&a[0],&b[0],150-20*i,3+(seed>>24)%7,4+(seed>>11)%10,(335+(seed>>5)%50)%360,1,.2);
        }
        CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review)&&review.valid_bands==count);
        bool agrees[count][count]={0},clique[sets]={0};
        double weight[sets]={0},best=0;
        for(unsigned i=0;i<count;++i)for(unsigned j=0;j<i;++j){
            const product_assessment_band_t x=review.bands[i],y=review.bands[j];
            const double dx=x.mass_g*cos(rad*x.angle_deg)-y.mass_g*cos(rad*y.angle_deg);
            const double dy=x.mass_g*sin(rad*x.angle_deg)-y.mass_g*sin(rad*y.angle_deg);
            agrees[i][j]=hypot(dx,dy)<=.35*fmax(x.mass_g,y.mass_g)&&fabs(remainder(x.angle_deg-y.angle_deg,360))<=30;
        }
        for(unsigned mask=1;mask<sets;++mask){
            clique[mask]=true;
            for(unsigned i=0;i<count;++i)if(mask&(1U<<i)){
                weight[mask]+=review.bands[i].score*review.bands[i].score;
                for(unsigned j=0;j<i;++j)if((mask&(1U<<j))&&!agrees[i][j])clique[mask]=false;
            }
            if(clique[mask])best=fmax(best,weight[mask]);
        }
        unsigned common=sets-1;
        for(unsigned mask=1;mask<sets;++mask)if(clique[mask]&&weight[mask]+1e-12>=.8*best){
            bool maximal=true;
            for(unsigned i=0;i<count;++i)if(!(mask&(1U<<i))&&clique[mask|(1U<<i)])maximal=false;
            if(maximal)common&=mask;
        }
        const bool useful=common!=0&&weight[common]+1e-12>=.8*best;
        CHECK(result.engineering_available==useful);
        CHECK(clique[review.joint_band_mask]);
        if(useful)CHECK(review.joint_band_mask==common);
        else CHECK(result.reason==PRODUCT_CONFLICT&&fabs(weight[review.joint_band_mask]-best)<1e-10);
    }
    return true;
}
static bool mismatched_axis_capture_is_not_reused(void)
{
    for(unsigned k=0;k<3;++k)band(&a[k],&b[k],110,8,4,120,1,0);
    a[1].summary.capture_id=99;CHECK(!product_assess_axes(&cfg,a,b,NULL,&result,&review));CHECK(result.reason==PRODUCT_CONTEXT);return true;
}
static bool sidecar_is_bound_to_exact_source_and_revision(void)
{
    band(&a[0],&b[0],150,8,4.3,120,1,.3);band(&a[0],&b[0],110,8,3.7,120,-.3,1);
    CHECK(product_assess_runs(&cfg,&a[0],&b[0],&result,&review));
    product_snapshot_t s={.schema=1,.id=55,.revision=3,.config=cfg,.solution=result};strcpy(s.boot_id,"testboot");
    s.stages[0]=a[0].summary;s.stages[1]=b[0].summary;product_assessment_bind(&review,&s);
    CHECK(product_assessment_valid(&review,&s));++s.revision;CHECK(!product_assessment_valid(&review,&s));product_assessment_bind(&review,&s);CHECK(product_assessment_valid(&review,&s));
    ++s.stages[1].capture_id;CHECK(!product_assessment_valid(&review,&s));--s.stages[1].capture_id;
    ++s.stages[1].epoch;CHECK(!product_assessment_valid(&review,&s));--s.stages[1].epoch;
    s.simulated=true;CHECK(!product_assessment_valid(&review,&s));return true;
}
static bool changed_context_cannot_rebind_old_review(void)
{
    product_snapshot_t s={.schema=1,.id=55,.revision=3,.config=cfg};strcpy(s.boot_id,"testboot");product_assessment_reset(&review,&s);
    CHECK(product_assessment_valid(&review,&s));s.id=56;product_assessment_bind(&review,&s);CHECK(!product_assessment_valid(&review,&s));return true;
}
int main(void)
{
#define RUN(f) do{setup();if(!f())return 1;++passed;}while(0)
    RUN(intermediate_mass_cannot_join_incompatible_groups);RUN(decisive_group_or_strong_common_core_keeps_guidance);
    RUN(wrapped_angles_and_full_band_capacity);RUN(bounded_groups_match_exhaustive_reference);
    RUN(complex_joint_fit_reduces_band_bias);RUN(rpm_dependent_phase_and_gain_cancel);RUN(contradictory_band_is_not_averaged_silently);
    RUN(joint_fit_respects_trial_angle_radius_and_direction);RUN(noisy_responses_are_less_sensitive_than_weak_tail_ratio);
    RUN(stronger_reproducible_band_beats_late_weak_tail);RUN(split_detects_a_collinear_but_variable_response);
    RUN(three_turns_remain_an_explicit_limited_estimate);RUN(trial_response_resolves_axis_in_one_capture);
    RUN(high_gain_cannot_dominate_and_band_tie_cannot_direct);RUN(incompatible_axes_are_visible_not_blended);RUN(unselected_axis_fault_still_blocks_everything);RUN(mismatched_axis_capture_is_not_reused);
    RUN(internally_conflicted_axis_cannot_break_a_tie);RUN(near_zero_original_stays_zero_across_bands);
    RUN(sidecar_is_bound_to_exact_source_and_revision);RUN(changed_context_cannot_rebind_old_review);
    printf("PASS estimation_assessment cases=%u snapshot_bytes=%zu assessment_bytes=%zu synthetic_only=1\n",passed,sizeof(product_snapshot_t),sizeof(product_assessment_t));return 0;
}
