#include "product_window.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(v) do{if(!(v)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#v);return 1;}}while(0)
static selection_turn_t turns[400];
static void fixture(unsigned n,double rpm,bool coast)
{
    memset(turns,0,sizeof(turns));
    for(unsigned i=0;i<n;++i){coast_result_t *f=&turns[i].fit;f->valid=true;f->status=COAST_OK;f->epoch=1;
        f->rpm=coast?rpm-i*1.5:rpm;f->rpm_previous=f->rpm+(coast?1.5:0);f->ref_first=i+2;f->ref_last=i+3;
        f->start_us=1000000+(uint64_t)i*600000;f->end_us=f->start_us+600000;
        for(unsigned a=0;a<3;++a){f->axis[a].real=.01;f->axis[a].imag=.005;f->axis[a].peak=hypot(.01,.005);f->axis[a].phase_available=true;}}
}
int main(void)
{
    product_window_stage_t w;fixture(20,100,false);product_window_select(turns,20,&w);
    CHECK(w.mode==WINDOW_STEADY&&w.usable[0]==18&&w.skipped[0]==2);
    CHECK(turns[0].reasons[0]&SELECT_SETTLING);CHECK(!(turns[2].reasons[0]&SELECT_SETTLING));
    fixture(20,160,true);product_window_select(turns,20,&w);CHECK(w.mode==WINDOW_COAST&&w.usable[1]==18);
    fixture(20,100,false);for(unsigned i=1;i<20;++i){turns[i].fit.rpm=100+(i%3==1?1:i%3==2?-1:0);turns[i].fit.rpm_previous=turns[i-1].fit.rpm;}
    product_window_select(turns,20,&w);CHECK(w.usable[0]>=10);
    fixture(20,100,false);for(unsigned i=0;i<20;++i){turns[i].fit.rpm=100+i;turns[i].fit.rpm_previous=99+i;}
    product_window_select(turns,20,&w);CHECK(!w.usable[0]&&w.accelerating_turns==20);
    fixture(20,100,false);for(unsigned i=0;i<5;++i){turns[i].fit.valid=false;turns[i].fit.status=COAST_ERR_ACCELERATING;}
    product_window_select(turns,20,&w);CHECK(w.accelerating_turns==5&&w.usable[0]==13&&w.skipped[0]==7);
    fixture(20,100,false);turns[8].reasons[0]=SELECT_IMPULSE;product_window_select(turns,20,&w);
    CHECK(w.usable[0]==15&&w.usable[1]==18&&w.transient[0]==1);CHECK(turns[10].reasons[0]&SELECT_SETTLING);CHECK(!(turns[11].reasons[0]&SELECT_SETTLING));
    fixture(20,100,false);for(unsigned i=0;i<20;++i)turns[i].reasons[2]=SELECT_WEAK_PHASE;
    product_window_select(turns,20,&w);CHECK(w.usable[2]==18);CHECK(turns[2].reasons[2]==SELECT_WEAK_PHASE);
    fixture(20,100,false);turns[8].fit.ref_last+=10;product_window_select(turns,20,&w);CHECK(turns[9].reasons[0]&SELECT_SETTLING);
    fixture(2,100,false);product_window_select(turns,2,&w);CHECK(!w.usable[0]&&w.phase==WINDOW_SETTLING);
    fixture(20,100,false);turns[4].fit.rpm=NAN;turns[5].fit.rpm=400;product_window_select(turns,20,&w);
    CHECK((turns[4].reasons[0]&SELECT_SETTLING)&&(turns[5].reasons[0]&SELECT_SETTLING));
    fixture(100,100,false);product_window_select(turns,100,&w);CHECK(w.usable[0]==96&&w.skipped[0]==4);
    /* Per-axis mask is the actual input to the unchanged product extractor. */
    product_run_t r={.summary={.integrity_valid=true,.epoch=1}};CHECK(product_extract_run(turns,100,0,&r)&&r.count==96);
    /* A long slow tail must not evict early high-speed data. Time coverage
     * within each occupied band is retained without using vibration values. */
    fixture(400,100,false);
    for(unsigned i=0;i<400;++i){turns[i].fit.rpm=turns[i].fit.rpm_previous=i<100?180:60;}
    product_window_select(turns,400,&w);unsigned high=0,low=0;
    for(unsigned i=0;i<400;++i)if(!(turns[i].reasons[0]&~SELECT_WEAK_PHASE)){if(i<100)++high;else ++low;}
    CHECK(w.usable[0]==96&&high==48&&low==48);
    CHECK(!(turns[2].reasons[0]&SELECT_WINDOW_LIMIT)&&!(turns[99].reasons[0]&SELECT_WINDOW_LIMIT)&&!(turns[399].reasons[0]&SELECT_WINDOW_LIMIT));
    fixture(24,500,false);
    for(unsigned i=12;i<24;++i)turns[i].fit.rpm=turns[i].fit.rpm_previous=280;
    product_window_select_timed(turns,24,500,&w);
    CHECK(w.mode==WINDOW_STEADY&&w.usable[0]==10);
    CHECK(!(turns[2].reasons[0]&SELECT_SETTLING)&&(turns[20].reasons[0]&SELECT_SETTLING));
    product_window_select_timed(turns,24,600,&w);CHECK(!w.usable[0]);
    product_snapshot_t s={.id=42,.revision=8,.state=PRODUCT_RECORDING,.stage=PRODUCT_ORIGINAL};strcpy(s.boot_id,"0123456789abcdef");
    const product_capture_config_t capture={.mode=CAPTURE_TIMED,.vehicle_type=CAPTURE_ELECTRIC,.seconds=30};
    product_window_t p;product_window_reset(&p,&s,capture);CHECK(product_window_valid(&p,&s,false));
    CHECK(product_window_has_capture_settings(&p,&s));
    fixture(20,100,false);product_window_select(turns,20,&p.stages[0]);product_window_stage_t *v=&p.stages[0];
    v->capture_id=7;v->epoch=1;v->start_us=1000000;v->end_us=13000000;
    CHECK(product_window_valid(&p,&s,false)&&!product_window_valid(&p,&s,true));
    s.stages[0]=(product_stage_summary_t){.capture_id=7,.epoch=1,.start_us=1000000,.end_us=13000000};s.state=PRODUCT_COMPLETE;
    CHECK(product_window_valid(&p,&s,true));s.revision++;CHECK(!product_window_valid(&p,&s,true));product_window_bind(&p,&s);CHECK(product_window_valid(&p,&s,true));
    s.id++;CHECK(!product_window_valid(&p,&s,true)&&!product_window_has_capture_settings(&p,&s));s.id--;
    s.stages[0].epoch++;CHECK(!product_window_valid(&p,&s,true)&&product_window_has_capture_settings(&p,&s));s.stages[0].epoch--;
    product_window_clear(&p,0);CHECK(!p.stages[0].capture_id&&product_window_valid(&p,&s,true));
    p.schema=1;CHECK(product_window_valid(&p,&s,true)&&!product_window_has_capture_settings(&p,&s)); /* released sidecar remains readable */
    printf("PASS whole-record window: settling, all-speed stratification/time coverage, no recent-tail eviction, actual extractor, legacy schema, provenance; sidecar_bytes=%zu\n",sizeof(p));return 0;
}
