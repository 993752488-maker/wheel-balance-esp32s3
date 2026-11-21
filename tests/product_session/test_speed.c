#include "product_speed.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(v) do{if(!(v)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#v);return 1;}}while(0)
int main(void)
{
    product_snapshot_t s={.id=42,.revision=7,.config={.axis=1},.state=PRODUCT_COMPLETE};strcpy(s.boot_id,"0123456789abcdef");
    product_run_t run={.summary={.captured=true,.integrity_valid=true,.capture_id=3,.epoch=1,.start_us=100,.end_us=900},.count=4};
    run.turns[0]=(product_turn_t){.rpm=100,.real=3,.imag=4,.reference=1};
    run.turns[1]=(product_turn_t){.rpm=105,.real=-3,.imag=-4,.reference=2};
    run.turns[2]=(product_turn_t){.rpm=119,.real=0,.imag=10,.reference=3};
    run.turns[3]=(product_turn_t){.rpm=140,.real=2,.reference=4};
    s.stages[0]=run.summary;product_speed_t p;product_speed_reset(&p,&s);
    CHECK(product_speed_valid(&p,&s,false));CHECK(product_speed_set(&p,0,&run,false,900));
    CHECK(fabs(p.stages[0].bins[4].rms-sqrt(50))<1e-5);CHECK(p.stages[0].bins[4].minimum==5&&p.stages[0].bins[4].maximum==10);
    CHECK(p.stages[0].bins[4].count==3&&p.stages[0].bins[5].count==0&&p.stages[0].bins[6].count==1);
    CHECK(p.stages[0].used_turns==4&&product_speed_valid(&p,&s,true));
    const product_speed_stage_t original=p.stages[0];s.stage=PRODUCT_TRIAL;s.state=PRODUCT_RECORDING;
    run.summary.capture_id=4;run.summary.start_us=1000;run.summary.end_us=2000;
    CHECK(product_speed_set(&p,1,&run,true,2000));CHECK(!memcmp(&original,&p.stages[0],sizeof(original)));
    CHECK(product_speed_valid(&p,&s,false)&&!product_speed_valid(&p,&s,true));
    s.historical=true;CHECK(!product_speed_valid(&p,&s,false));s.historical=false;
    s.state=PRODUCT_PROCESSING;CHECK(product_speed_valid(&p,&s,false));
    s.state=PRODUCT_COMPLETE;CHECK(!product_speed_valid(&p,&s,false));
    s.stages[1]=run.summary;CHECK(product_speed_set(&p,1,&run,false,2100));CHECK(product_speed_valid(&p,&s,true));
    s.revision++;CHECK(!product_speed_valid(&p,&s,true));product_speed_bind(&p,&s);CHECK(product_speed_valid(&p,&s,true));
    s.id++;CHECK(!product_speed_valid(&p,&s,true));s.id--;s.config.axis=2;CHECK(!product_speed_valid(&p,&s,true));s.config.axis=1;
    s.boot_id[0]='f';CHECK(!product_speed_valid(&p,&s,true));s.boot_id[0]='0';
    s.stages[1].capture_id++;CHECK(!product_speed_valid(&p,&s,true));s.stages[1].capture_id--;
    run.turns[0].real=NAN;CHECK(!product_speed_set(&p,1,&run,false,2200));CHECK(p.stages[1].mode==SPEED_INVALID&&p.stages[1].used_turns==0);
    CHECK(product_speed_valid(&p,&s,true));CHECK(!memcmp(&original,&p.stages[0],sizeof(original)));
    product_speed_clear(&p,1);CHECK(p.stages[1].mode==SPEED_EMPTY);product_speed_reset(&p,&s);CHECK(p.stages[0].used_turns==0);
    run.turns[0].real=1;run.turns[0].rpm=20;run.turns[1].rpm=39.9;run.turns[2].rpm=40;run.turns[3].rpm=300;
    CHECK(product_speed_set(&p,0,&run,false,2300));CHECK(p.stages[0].bins[0].count==2&&p.stages[0].bins[1].count==1&&p.stages[0].outside_turns==1);
    run.summary.integrity_valid=false;CHECK(!product_speed_set(&p,0,&run,false,2400));CHECK(p.stages[0].used_turns==0);
    CHECK(sizeof(product_speed_t)<=1200);printf("PASS speed profile: RMS, phase cancellation, holes, boundaries, frozen baseline, preview, revision/boot/axis/capture isolation, nonfinite/fault rejection; bytes=%zu\n",sizeof(product_speed_t));return 0;
}
