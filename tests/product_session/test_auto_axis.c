#include "product_auto_axis.h"
#include "product_trial.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(v) do{if(!(v)){fprintf(stderr,"FAIL %s:%d %s\n",__func__,__LINE__,#v);return false;}}while(0)
static product_run_t axes[3];static product_axis_choice_t choice;static unsigned passed;
static void setup(void)
{
    memset(axes,0,sizeof(axes));
    for(unsigned axis=0;axis<3;++axis){product_run_t *r=&axes[axis];r->summary=(product_stage_summary_t){.captured=true,.integrity_valid=true,.capture_id=1,.epoch=1,.start_us=1,.end_us=9000000};r->count=8;
        for(unsigned i=0;i<8;++i)r->turns[i]=(product_turn_t){.rpm=116-i*2,.previous_rpm=118-i*2,.real=(axis==1?1:(i%2?1:-1)),.imag=0,.reference=i+1,.time_us=1000+i*1000};}
}
static bool uses_same_capture_three_axes_once(void)
{CHECK(product_choose_axis(axes,&choice));CHECK(choice.axis==1&&choice.score[1]==1);CHECK(axes[0].summary.capture_id==axes[2].summary.capture_id);return true;}
static bool loud_incoherent_axis_does_not_win(void)
{for(unsigned i=0;i<8;++i)axes[0].turns[i].real*=100000;CHECK(product_choose_axis(axes,&choice));CHECK(choice.axis==1);return true;}
static bool deterministic_tie(void)
{axes[0]=axes[1];axes[2]=axes[1];CHECK(product_choose_axis(axes,&choice));CHECK(choice.axis==0);return true;}
static bool no_qualified_axis_is_not_a_fake_success(void)
{for(unsigned a=0;a<3;++a)axes[a].summary.captured=false;CHECK(!product_choose_axis(axes,&choice));CHECK(choice.reason==PRODUCT_TOO_FEW_TURNS);return true;}
static bool corrupt_unselected_axis_blocks_selection(void)
{axes[2].turns[0].real=NAN;CHECK(!product_choose_axis(axes,&choice));CHECK(choice.reason==PRODUCT_NUMERIC);return true;}
static bool different_capture_or_epoch_is_rejected(void)
{axes[2].summary.capture_id=2;CHECK(!product_choose_axis(axes,&choice));CHECK(choice.reason==PRODUCT_CONTEXT);setup();axes[0].summary.epoch=2;CHECK(!product_choose_axis(axes,&choice));return true;}
static bool single_good_band_does_not_dominate_selection(void)
{for(unsigned a=0;a<3;++a){axes[a].count=9;for(unsigned i=0;i<9;++i){product_turn_t *t=&axes[a].turns[i];memset(t,0,sizeof(*t));t->rpm=60+(i/3)*40+i%3;
 t->real=a==1?(i%3==2?-.2:1):a==0&&i<3?1:i%3==0?1:i%3==1?-1:0;}}
 CHECK(product_choose_axis(axes,&choice));CHECK(choice.axis==1);return true;}
static bool legacy_guide_is_preserved_as_manual(void)
{product_snapshot_t s={.id=42,.revision=7};strcpy(s.boot_id,"0000000000000001");s.config=product_default_config();s.stages[0].captured=true;
 product_trial_guide_t g;CHECK(product_trial_init(&g,&s,5,true));g.schema=1;g.automatic_axis=true;g.axis_resolved=false;
 CHECK(product_trial_restore(&g,&s,920));CHECK(g.schema==2&&!g.automatic_axis&&g.axis_resolved&&g.initial_mass_g==10);return true;}
static bool wrong_revision_or_size_not_migrated(void)
{product_snapshot_t s={.id=42,.revision=7};strcpy(s.boot_id,"0000000000000001");s.config=product_default_config();product_trial_guide_t g;
 CHECK(product_trial_init(&g,&s,5,true));CHECK(!product_trial_restore(&g,&s,924));g.snapshot_revision++;CHECK(!product_trial_restore(&g,&s,sizeof(g)));return true;}
static bool unresolved_auto_channel_cannot_start_trial(void)
{product_snapshot_t s={.id=42,.revision=7,.state=PRODUCT_COMPLETE};strcpy(s.boot_id,"0000000000000001");s.config=product_default_config();product_trial_guide_t g;
 CHECK(product_trial_init(&g,&s,5,true));s.stages[0].captured=s.stages[0].integrity_valid=true;
 CHECK(!product_trial_can_start(&g,&s,10));g.axis_resolved=true;CHECK(product_trial_can_start(&g,&s,10));return true;}
static bool mass_change_preserves_frozen_axis(void)
{product_snapshot_t s={.id=42,.revision=7,.state=PRODUCT_COMPLETE};strcpy(s.boot_id,"0000000000000001");s.config=product_default_config();s.config.axis=2;
 s.stages[0].captured=s.stages[0].integrity_valid=true;product_trial_guide_t g;CHECK(product_trial_init(&g,&s,5,true));
 CHECK(product_trial_set_plan(&g,&s,50,TRIAL_PLAN_MANUAL));CHECK(s.config.axis==2&&g.axis_resolved);return true;}
int main(void)
{
    _Static_assert(sizeof(product_snapshot_t)==752,"historical snapshot ABI changed");
    _Static_assert(offsetof(product_trial_guide_t,automatic_axis)==920,"legacy guide prefix changed");
#define RUN(f) do{setup();if(!f())return 1;++passed;}while(0)
    RUN(uses_same_capture_three_axes_once);RUN(loud_incoherent_axis_does_not_win);RUN(deterministic_tie);RUN(no_qualified_axis_is_not_a_fake_success);
    RUN(corrupt_unselected_axis_blocks_selection);RUN(different_capture_or_epoch_is_rejected);RUN(single_good_band_does_not_dominate_selection);
    RUN(legacy_guide_is_preserved_as_manual);RUN(wrong_revision_or_size_not_migrated);RUN(unresolved_auto_channel_cannot_start_trial);RUN(mass_change_preserves_frozen_axis);
    printf("PASS automatic_axis cases=%u snapshot_bytes=%zu guide_bytes=%zu synthetic_only=1\n",passed,sizeof(product_snapshot_t),sizeof(product_trial_guide_t));return 0;
}
