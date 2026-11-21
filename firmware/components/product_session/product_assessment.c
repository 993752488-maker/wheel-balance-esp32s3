#include "product_assessment.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(product_assessment_t)==1720,"assessment sidecar ABI changed");
_Static_assert(offsetof(product_assessment_t,start_us)==88,"assessment timestamps moved");
static const double pi=3.14159265358979323846;
static bool finite_nonnegative(double v){return isfinite(v)&&v>=0;}
static double phase_difference(double a,double b)
{double p=(a-b)*pi/180;return fabs(atan2(sin(p),cos(p)))*180/pi;}
static double vector_difference(double m,double a,double n,double b)
{return hypot(m*cos(a*pi/180)-n*cos(b*pi/180),m*sin(a*pi/180)-n*sin(b*pi/180))/fmax(fmax(m,n),1e-12);}
const char *product_assessment_state_name(product_assessment_state_t state)
{static const char *names[]={"NOT_ASSESSED","LIMITED_SUPPORT","OBSERVATIONS_CLOSE","VARIABLE_ESTIMATES"};
 return (unsigned)state<4?names[state]:"NOT_ASSESSED";}
static double tail_distance(const product_run_t *run,double lower)
{
    uint64_t latest=0;for(size_t i=0;i<run->count;++i)
        if(run->turns[i].rpm>=lower&&run->turns[i].rpm<lower+20&&run->turns[i].time_us>latest)latest=run->turns[i].time_us;
    return latest&&run->summary.end_us>latest?(double)(run->summary.end_us-latest):0;
}
static void source(product_assessment_t *q,const product_config_t *cfg,const product_run_t *a,const product_run_t *b)
{
    memset(q,0,sizeof(*q));q->schema=1;q->policy=PRODUCT_ASSESSMENT_POLICY;q->selected_band=PRODUCT_ASSESSMENT_BANDS;
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){q->bands[i].lower_rpm=(i+1)*20;q->bands[i].reason=PRODUCT_NO_COMMON_SPEED;}
    if(cfg)q->axis=cfg->axis;
    if(a){q->capture_id[0]=a->summary.capture_id;q->epoch=a->summary.epoch;q->start_us[0]=a->summary.start_us;q->end_us[0]=a->summary.end_us;}
    if(b){q->capture_id[1]=b->summary.capture_id;q->start_us[1]=b->summary.start_us;q->end_us[1]=b->summary.end_us;}
}
/* Per-band and per-axis evidence in wheel correction coordinates.
 * Quality is a relative ranking. A rival retaining >=80% of the winning
 * group's support makes an incompatible point estimate ambiguous. */
typedef struct {
    double mass, angle, score;
    bool available;
} observation_t;

static bool compatible(observation_t a, observation_t b)
{
    return vector_difference(a.mass,a.angle,b.mass,b.angle)<=.35 &&
           (fmin(a.mass,b.mass)<=1e-9 || phase_difference(a.angle,b.angle)<=30);
}

typedef struct {
    uint32_t neighbours[PRODUCT_ASSESSMENT_BANDS],preferred_bit,best,common;
    double weight[PRODUCT_ASSESSMENT_BANDS],best_weight;
    bool intersect;
} group_search_t;

typedef struct {
    uint32_t members;
    unsigned anchor;
    bool conflict,has_excluded;
} group_result_t;

/* Enumerate maximal pairwise-compatible groups (Bron-Kerbosch with a pivot).
 * At most 14 speed bands or 3 axes, one stack frame per member, no heap.
 * The second pass intersects every contender, including equal-weight ties. */
static void visit_groups(group_search_t *s,uint32_t members,uint32_t possible,
                         uint32_t excluded,double weight)
{
    if(!possible){
        if(excluded)return;
        if(s->intersect){
            if(weight+1e-12>=.80*s->best_weight)s->common&=members;
        }else if(!s->best||weight>s->best_weight+1e-12||
                 (fabs(weight-s->best_weight)<=1e-12&&(members&s->preferred_bit)&&!(s->best&s->preferred_bit))){
            s->best=members;
            s->best_weight=weight;
        }
        return;
    }
    const unsigned pivot=(unsigned)__builtin_ctz(possible|excluded);
    uint32_t choices=possible&~s->neighbours[pivot];
    while(choices){
        const unsigned i=(unsigned)__builtin_ctz(choices);
        const uint32_t bit=1U<<i;
        visit_groups(s,members|bit,possible&s->neighbours[i],excluded&s->neighbours[i],weight+s->weight[i]);
        possible&=~bit;
        excluded|=bit;
        choices&=~bit;
    }
}

static group_result_t supported_group(const observation_t rows[],unsigned count,unsigned preferred)
{
    group_search_t search={.preferred_bit=1U<<preferred};
    uint32_t available=0;
    for(unsigned i=0;i<count;++i){
        if(!rows[i].available)continue;
        available|=1U<<i;
        search.weight[i]=rows[i].score*rows[i].score;
        for(unsigned j=0;j<i;++j)if(rows[j].available&&compatible(rows[i],rows[j])){
            search.neighbours[i]|=1U<<j;
            search.neighbours[j]|=1U<<i;
        }
    }
    visit_groups(&search,0,available,0,0);
    search.common=search.best;
    search.intersect=true;
    visit_groups(&search,0,available,0,0);
    double common_weight=0;
    for(unsigned i=0;i<count;++i)if(search.common&(1U<<i))common_weight+=search.weight[i];
    /* If two credible groups overlap, their shared measurements may still
     * suffice by themselves. Otherwise retain one fit for diagnostics only. */
    const bool conflict=!search.common||common_weight+1e-12<.80*search.best_weight;
    const uint32_t members=conflict?search.best:search.common;
    unsigned anchor=preferred;
    for(unsigned i=0;i<count;++i)if(members&(1U<<i)){
        if(!(members&(1U<<anchor))||rows[i].score>rows[anchor].score+1e-12)anchor=i;
    }
    return (group_result_t){members,anchor,conflict,members!=available};
}

static void withhold(product_solution_t *solution)
{
    solution->reason=PRODUCT_CONFLICT;
    solution->engineering_available=false;
}

static void joint_fit(const product_config_t *cfg,product_solution_t *out,product_assessment_t *q,uint32_t mask)
{
    q->correction_real_g_mm=out->calculation.correction_g_mm.real;
    q->correction_imag_g_mm=out->calculation.correction_g_mm.imag;
    q->joint_band_mask=1U<<q->selected_band;
    q->joint_bands=1;
    /* A near-zero original response has no correction direction. Keep the
     * band's zero correction instead of recomputing a ratio from its noise. */
    if(!out->calculation.angle_valid)return;
    const unsigned count=(unsigned)__builtin_popcount(mask);
    if(count<2)return;
    double nr=0,ni=0,den=0;
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i)if(mask&(1U<<i)){
        const product_assessment_band_t *r=&q->bands[i];const double w=r->score*r->score;
        const double gain=hypot(r->influence_real,r->influence_imag);
        if(!isfinite(gain)||gain<=0)return;
        const double hr=r->influence_real/gain,hi=r->influence_imag/gain;
        const double vr=r->original_real/gain,vi=r->original_imag/gain;
        nr+=w*(hr*vr+hi*vi);ni+=w*(hr*vi-hi*vr);den+=w;
    }
    if(!isfinite(den)||den<=0)return;
    const double ur=-nr/den,ui=-ni/den,mass=hypot(ur,ui)/cfg->correction_radius_mm;
    if(!isfinite(ur)||!isfinite(ui)||!isfinite(mass)||mass>balance_default_config().max_mass_g)return;
    double residual=0,initial=0;
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i)if(mask&(1U<<i)){
        const product_assessment_band_t *r=&q->bands[i];const double w=r->score*r->score;
        const double er=r->original_real+r->influence_real*ur-r->influence_imag*ui;
        const double ei=r->original_imag+r->influence_real*ui+r->influence_imag*ur;
        const double gain=hypot(r->influence_real,r->influence_imag);
        residual+=w*pow(hypot(er,ei)/gain,2);
        initial+=w*pow(hypot(r->original_real,r->original_imag)/gain,2);
    }
    const double fit=initial>0?sqrt(residual/initial):0;if(!isfinite(fit))return;
    q->joint_used=1;q->joint_band_mask=mask;q->joint_bands=count;q->model_residual_ratio=fit;
    q->correction_real_g_mm=ur;q->correction_imag_g_mm=ui;
    out->calculation.correction_g_mm=(balance_complex_t){ur,ui};out->calculation.mass_g=mass;
    out->calculation.angle_deg=fmod(atan2(ui,ur)*180/pi+360,360);
    out->calculation.angle_valid=mass>1e-12;out->engineering_available=out->calculation.angle_valid;
    /* influence/delta/SNR remain the measured ANCHOR band's diagnostics.
     * Correction combines -V0_k/H_k using squared quality weights, not -V0/H
     * of that one band. The separate sidecar preserves every H_k and V0_k. */
}
bool product_assess_runs(const product_config_t *cfg,const product_run_t *a,const product_run_t *b,
    product_solution_t *out,product_assessment_t *assessment)
{
    if(!out)return false;
    product_assessment_t local;product_assessment_t *q=assessment?assessment:&local;source(q,cfg,a,b);
    memset(out,0,sizeof(*out));product_solution_t failure={.reason=PRODUCT_NO_COMMON_SPEED};
    product_pairing_t pairing;
    out->reason=product_match_runs(a,b,20,300,&pairing);
    if(out->reason!=PRODUCT_REASON_NONE)return false;
    double best_score=-1,best_response=-1,best_tail=INFINITY;unsigned failed_pairs=0;
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){
        product_solution_t candidate;product_band_detail_t detail;
        const double lower=(i+1)*20;
        const bool ok=product_evaluate_matched_band(cfg,a,b,&pairing,lower,&candidate,&detail);
        product_assessment_band_t *row=&q->bands[i];
        *row=(product_assessment_band_t){.lower_rpm=(i+1)*20,.pairs=candidate.matched_turns,.available=ok,
            .reason=candidate.reason,.split_available=detail.split_available,.split_valid=detail.split_valid,
            .mass_g=candidate.calculation.mass_g,.angle_deg=candidate.calculation.angle_deg,
            .score=detail.score,.response_snr=candidate.calculation.response_snr,
            .split_relative_difference=detail.split_relative_difference,.split_phase_difference=detail.split_phase_difference,
            .influence_real=candidate.calculation.influence.real,.influence_imag=candidate.calculation.influence.imag,
            .original_real=detail.original_real,.original_imag=detail.original_imag};
        if(candidate.reason==PRODUCT_CONTEXT||candidate.reason==PRODUCT_NUMERIC||candidate.reason==PRODUCT_DATA_FAULT){
            memset(out,0,sizeof(*out));out->reason=candidate.reason;q->valid_bands=0;q->selected_band=PRODUCT_ASSESSMENT_BANDS;return false;}
        if(!ok){if(candidate.matched_turns>failed_pairs){failure=candidate;failed_pairs=candidate.matched_turns;}continue;}
        ++q->valid_bands;
        const double tail=tail_distance(a,lower)+tail_distance(b,lower);
        const bool better=detail.score>best_score+1e-12||
            (fabs(detail.score-best_score)<=1e-12&&(candidate.delta_peak>best_response+1e-12||
            (fabs(candidate.delta_peak-best_response)<=1e-12&&tail<best_tail)));
        if(better){*out=candidate;q->selected_band=i;best_score=detail.score;best_response=candidate.delta_peak;best_tail=tail;}
    }
    if(!q->valid_bands){*out=failure;return false;}
    observation_t bands[PRODUCT_ASSESSMENT_BANDS];
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){
        const product_assessment_band_t *b=&q->bands[i];
        bands[i]=(observation_t){b->mass_g,b->angle_deg,b->score,b->available};
    }
    const group_result_t group=supported_group(bands,PRODUCT_ASSESSMENT_BANDS,q->selected_band);
    const unsigned anchor=group.anchor;
    if(anchor!=q->selected_band){
        if(!product_evaluate_matched_band(cfg,a,b,&pairing,q->bands[anchor].lower_rpm,out,NULL))return false;
        q->selected_band=anchor;
        best_score=q->bands[anchor].score;
    }
    joint_fit(cfg,out,q,group.members);
    if(group.conflict)withhold(out);
    q->axis_finalized=!group.conflict;q->axis_available_mask=1U<<cfg->axis;q->axis_scores[cfg->axis]=best_score;
    if(group.conflict)q->axis_conflict_mask=1U<<cfg->axis;
    q->axis_mass_g[cfg->axis]=out->calculation.mass_g;q->axis_angle_deg[cfg->axis]=out->calculation.angle_deg;
    q->mass_min_g=q->mass_max_g=out->calculation.mass_g;
    for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){const product_assessment_band_t *r=&q->bands[i];if(!r->available)continue;
        q->mass_min_g=fmin(q->mass_min_g,r->mass_g);q->mass_max_g=fmax(q->mass_max_g,r->mass_g);
        q->max_relative_difference=fmax(q->max_relative_difference,vector_difference(r->mass_g,r->angle_deg,out->calculation.mass_g,out->calculation.angle_deg));
        if(r->mass_g>1e-9&&out->calculation.mass_g>1e-9)q->max_phase_difference=fmax(q->max_phase_difference,phase_difference(r->angle_deg,out->calculation.angle_deg));
    }
    const product_assessment_band_t *selected=&q->bands[q->selected_band];
    if(q->valid_bands<2)q->flags|=ASSESS_SINGLE_BAND;
    if(!selected->split_available)q->flags|=ASSESS_NO_SPLIT;
    else if(!selected->split_valid)q->flags|=ASSESS_SPLIT_FAILED;
    else if(selected->split_relative_difference>.35||selected->split_phase_difference>30)q->flags|=ASSESS_SPLIT_DIFFERENT;
    if(group.has_excluded||q->max_relative_difference>.35||q->max_phase_difference>30)q->flags|=ASSESS_BANDS_DIFFERENT;
    q->state=(q->flags&(ASSESS_SPLIT_FAILED|ASSESS_SPLIT_DIFFERENT|ASSESS_BANDS_DIFFERENT))?ASSESS_VARIABLE:
        q->flags?ASSESS_LIMITED:ASSESS_CLOSE;
    return true;
}
static bool same_capture(const product_run_t *a,const product_run_t *b)
{return a->summary.capture_id==b->summary.capture_id&&a->summary.epoch==b->summary.epoch&&
    a->summary.start_us==b->summary.start_us&&a->summary.end_us==b->summary.end_us;}
bool product_assess_axes(const product_config_t *cfg,const product_run_t original[3],const product_run_t trial[3],
    unsigned *axis,product_solution_t *out,product_assessment_t *assessment)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));product_assessment_t best_review;memset(&best_review,0,sizeof(best_review));
    if(assessment)memset(assessment,0,sizeof(*assessment));
    if(!product_config_valid(cfg)||!original||!trial){out->reason=PRODUCT_CONTEXT;return false;}
    for(unsigned k=0;k<3;++k){
        if(!original[k].summary.integrity_valid||!trial[k].summary.integrity_valid||
           original[k].count>PRODUCT_MAX_TURNS||trial[k].count>PRODUCT_MAX_TURNS){out->reason=PRODUCT_DATA_FAULT;return false;}
        if(!same_capture(&original[0],&original[k])||!same_capture(&trial[0],&trial[k])){out->reason=PRODUCT_CONTEXT;return false;}
        const product_run_t *rows[]={&original[k],&trial[k]};
        for(unsigned r=0;r<2;++r)for(size_t j=0;j<rows[r]->count;++j){const product_turn_t *t=&rows[r]->turns[j];
            if(!isfinite(t->real)||!isfinite(t->imag)||!isfinite(t->rpm)||!isfinite(t->previous_rpm)||t->rpm<=0||t->previous_rpm<=0){out->reason=PRODUCT_NUMERIC;return false;}}
    }
    double scores[3]={0},masses[3]={0},angles[3]={0},best_score=-1;
    unsigned mask=0,conflicts=0,best_axis=cfg->axis,failed_pairs=0;bool found=false;
    product_solution_t failure={.reason=PRODUCT_NO_COMMON_SPEED};
    for(unsigned k=0;k<3;++k){product_config_t c=*cfg;c.axis=k;
        product_solution_t candidate;product_assessment_t review;
        /* An axis with no usable turns can be skipped; a global data/context
         * fault was checked above and can never be hidden by another channel. */
        if(!original[k].summary.captured||!trial[k].summary.captured)continue;
        const bool ok=product_assess_runs(&c,&original[k],&trial[k],&candidate,&review);
        if(candidate.reason==PRODUCT_CONTEXT||candidate.reason==PRODUCT_NUMERIC||candidate.reason==PRODUCT_DATA_FAULT){*out=candidate;return false;}
        if(!ok){if(candidate.matched_turns>failed_pairs){failure=candidate;failed_pairs=candidate.matched_turns;if(!found){best_review=review;best_axis=k;}}continue;}
        scores[k]=review.axis_scores[k];masses[k]=candidate.calculation.mass_g;angles[k]=candidate.calculation.angle_deg;mask|=1U<<k;
        conflicts|=review.axis_conflict_mask;
        if(!found||scores[k]>best_score+1e-12){found=true;best_score=scores[k];best_axis=k;*out=candidate;best_review=review;}
    }
    const unsigned supported=mask&~conflicts;
    if(supported){
        observation_t axes[3];
        unsigned preferred=best_axis;
        for(unsigned k=0;k<3;++k){
            const bool eligible=(supported&(1U<<k))!=0;
            axes[k]=(observation_t){masses[k],angles[k],scores[k],eligible};
            if(eligible&&(!(supported&(1U<<preferred))||scores[k]>scores[preferred]+1e-12))preferred=k;
        }
        const group_result_t group=supported_group(axes,3,preferred);
        const unsigned choice=group.anchor;
        if(choice!=best_axis){
            product_config_t selected=*cfg;
            selected.axis=choice;
            if(!product_assess_runs(&selected,&original[choice],&trial[choice],out,&best_review))return false;
            best_axis=choice;
        }
        if(group.conflict)withhold(out);
        if(group.has_excluded){best_review.flags|=ASSESS_AXES_DIFFERENT;best_review.state=ASSESS_VARIABLE;}
    }
    if(!found){*out=failure;if(!failed_pairs)source(&best_review,cfg,&original[cfg->axis],&trial[cfg->axis]);}
    best_review.automatic_axis=1;best_review.axis_finalized=found&&out->reason!=PRODUCT_CONFLICT;best_review.axis=best_axis;
    best_review.axis_available_mask=mask;best_review.axis_conflict_mask=conflicts;
    memcpy(best_review.axis_scores,scores,sizeof(scores));memcpy(best_review.axis_mass_g,masses,sizeof(masses));memcpy(best_review.axis_angle_deg,angles,sizeof(angles));
    if(found)for(unsigned k=0;k<3;++k)if(mask&(1U<<k)){
        if((conflicts&(1U<<k))||vector_difference(masses[k],angles[k],out->calculation.mass_g,out->calculation.angle_deg)>.35||
            (masses[k]>1e-9&&out->calculation.mass_g>1e-9&&phase_difference(angles[k],out->calculation.angle_deg)>30)){
            best_review.flags|=ASSESS_AXES_DIFFERENT;best_review.state=ASSESS_VARIABLE;}
    }
    if(axis)*axis=best_axis;
    if(assessment)*assessment=best_review;
    return found;
}
void product_assessment_bind(product_assessment_t *q,const product_snapshot_t *s)
{
    if(!q||!s)return;
    if(q->session_id&&(q->session_id!=s->id||!memchr(q->boot_id,0,sizeof(q->boot_id))||strcmp(q->boot_id,s->boot_id)))return;
    q->schema=1;if(!q->policy)q->policy=PRODUCT_ASSESSMENT_POLICY;q->session_id=s->id;q->snapshot_revision=s->revision;
    memcpy(q->boot_id,s->boot_id,sizeof(q->boot_id));q->simulated=s->simulated;
}
void product_assessment_reset(product_assessment_t *q,const product_snapshot_t *s)
{if(!q)return;memset(q,0,sizeof(*q));q->selected_band=PRODUCT_ASSESSMENT_BANDS;if(s){q->axis=s->config.axis;product_assessment_bind(q,s);}}
bool product_assessment_valid(const product_assessment_t *q,const product_snapshot_t *s)
{
    if(!q||!s||q->schema!=1||q->policy<1||q->policy>PRODUCT_ASSESSMENT_POLICY||!q->session_id||q->session_id!=s->id||q->snapshot_revision!=s->revision||
       !memchr(q->boot_id,0,sizeof(q->boot_id))||strcmp(q->boot_id,s->boot_id)||q->simulated!=(unsigned)s->simulated||q->axis!=s->config.axis||
       q->axis>2||q->state>ASSESS_VARIABLE||q->flags>63||q->automatic_axis>1||q->axis_finalized>1||q->valid_bands>PRODUCT_ASSESSMENT_BANDS||q->axis_available_mask>7)return false;
    if(q->policy>=3&&(q->axis_conflict_mask&~q->axis_available_mask))return false;
    if(!q->capture_id[0]&&!q->capture_id[1])return !s->solution.calculation_valid&&q->state==ASSESS_NONE&&q->valid_bands==0;
    for(unsigned i=0;i<2;++i)if(q->capture_id[i]!=s->stages[i].capture_id||q->epoch!=s->stages[i].epoch||
        q->start_us[i]!=s->stages[i].start_us||q->end_us[i]!=s->stages[i].end_us)return false;
    if(!q->epoch||q->capture_id[0]==q->capture_id[1]||q->end_us[0]>q->start_us[1])return false;
    unsigned valid=0;for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i){const product_assessment_band_t *b=&q->bands[i];
        if(b->lower_rpm!=(i+1)*20||b->pairs>PRODUCT_MAX_TURNS||b->available>1||b->reason>PRODUCT_CAPTURE_LIMIT||b->split_available>1||b->split_valid>1||
            !finite_nonnegative(b->mass_g)||!finite_nonnegative(b->angle_deg)||b->angle_deg>=360||!finite_nonnegative(b->score)||b->score>1||
            !finite_nonnegative(b->response_snr)||!finite_nonnegative(b->split_relative_difference)||!finite_nonnegative(b->split_phase_difference)||b->split_phase_difference>180.00001||
            !isfinite(b->influence_real)||!isfinite(b->influence_imag)||!isfinite(b->original_real)||!isfinite(b->original_imag))return false;
        if(b->available){++valid;if(b->reason!=PRODUCT_REASON_NONE||b->pairs<3)return false;}
    }
    for(unsigned i=0;i<3;++i)if(!finite_nonnegative(q->axis_scores[i])||q->axis_scores[i]>1||!finite_nonnegative(q->axis_mass_g[i])||
        !finite_nonnegative(q->axis_angle_deg[i])||q->axis_angle_deg[i]>=360)return false;
    if(!finite_nonnegative(q->mass_min_g)||!finite_nonnegative(q->mass_max_g)||q->mass_min_g>q->mass_max_g||
        !finite_nonnegative(q->max_relative_difference)||!finite_nonnegative(q->max_phase_difference)||q->max_phase_difference>180.00001||valid!=q->valid_bands)return false;
    if(q->joint_used>1||q->joint_band_mask>16383||q->joint_bands>PRODUCT_ASSESSMENT_BANDS||
        !isfinite(q->correction_real_g_mm)||!isfinite(q->correction_imag_g_mm)||!finite_nonnegative(q->model_residual_ratio))return false;
    if(!valid)return !s->solution.calculation_valid&&q->state==ASSESS_NONE;
    const bool should_be_final=q->policy<4||s->solution.reason!=PRODUCT_CONFLICT;
    if(q->selected_band>=PRODUCT_ASSESSMENT_BANDS||!s->solution.calculation_valid||q->axis_finalized!=(unsigned)should_be_final||
       q->state==ASSESS_NONE||!(q->axis_available_mask&(1U<<q->axis)))return false;
    const product_assessment_band_t *b=&q->bands[q->selected_band];
    unsigned joint=0;for(unsigned i=0;i<PRODUCT_ASSESSMENT_BANDS;++i)if(q->joint_band_mask&(1U<<i)){if(!q->bands[i].available)return false;++joint;}
    return b->available&&b->pairs==s->solution.matched_turns&&b->lower_rpm==s->solution.lower_rpm&&
        (q->joint_band_mask&(1U<<q->selected_band))&&joint==q->joint_bands&&(!q->joint_used||joint>=2)&&
        fabs(q->correction_real_g_mm-s->solution.calculation.correction_g_mm.real)<1e-8&&
        fabs(q->correction_imag_g_mm-s->solution.calculation.correction_g_mm.imag)<1e-8;
}
