#include "product_auto_axis.h"
#include <math.h>
#include <string.h>

static bool score_axis(const product_run_t *run,double *score)
{
    if(!run->summary.captured||!run->summary.integrity_valid||!run->summary.epoch||run->count<3||run->count>PRODUCT_MAX_TURNS)return false;
    for(size_t i=0;i<run->count;++i)if(!isfinite(run->turns[i].real)||!isfinite(run->turns[i].imag)||
        !isfinite(run->turns[i].rpm)||run->turns[i].rpm<=0)return false;
    double scores[14];unsigned count=0;
    for(unsigned band=1;band<15;++band){
        double re=0,im=0,magnitude=0;unsigned n=0;
        for(size_t i=0;i<run->count;++i){const product_turn_t *t=&run->turns[i];
            if(t->rpm<band*20||t->rpm>=(band+1)*20)continue;
            re+=t->real;im+=t->imag;magnitude+=hypot(t->real,t->imag);++n;}
        if(n<3)continue;
        const double value=magnitude>0?fmin(1,hypot(re,im)/magnitude):0;
        if(!isfinite(value))return false;
        unsigned at=count;while(at&&scores[at-1]>value){scores[at]=scores[at-1];--at;}scores[at]=value;++count;
    }
    if(count)*score=count%2?scores[count/2]:(scores[count/2-1]+scores[count/2])/2;
    else *score=isfinite(run->summary.coherence)?run->summary.coherence:0;
    return *score>=0&&*score<=1;
}
bool product_choose_axis(const product_run_t baseline[3],product_axis_choice_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    out->reason=PRODUCT_TOO_FEW_TURNS;
    if(!baseline)return false;
    unsigned turns=0;
    for(unsigned axis=0;axis<3;++axis){const product_run_t *r=&baseline[axis];
        if(!r->summary.integrity_valid){out->valid=false;out->reason=PRODUCT_DATA_FAULT;return false;}
        if(r->count>PRODUCT_MAX_TURNS){out->valid=false;out->reason=PRODUCT_DATA_FAULT;return false;}
        for(size_t i=0;i<r->count;++i)if(!isfinite(r->turns[i].real)||!isfinite(r->turns[i].imag)||
            !isfinite(r->turns[i].rpm)||r->turns[i].rpm<=0){out->valid=false;out->reason=PRODUCT_NUMERIC;return false;}
        /* All three channels must come from the same captured window. */
        if(axis&&(r->summary.capture_id!=baseline[0].summary.capture_id||r->summary.epoch!=baseline[0].summary.epoch||
            r->summary.start_us!=baseline[0].summary.start_us||r->summary.end_us!=baseline[0].summary.end_us)){out->valid=false;out->reason=PRODUCT_CONTEXT;return false;}
        if(!score_axis(r,&out->score[axis]))continue;
        if(!out->valid||out->score[axis]>out->score[out->axis]+1e-12||
            (fabs(out->score[axis]-out->score[out->axis])<=1e-12&&r->count>turns)){
            out->valid=true;out->axis=axis;turns=(unsigned)r->count;}
    }
    if(out->valid)out->reason=PRODUCT_REASON_NONE;
    return out->valid;
}
