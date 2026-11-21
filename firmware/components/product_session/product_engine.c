#include "product_engine.h"
#include "product_assessment.h"
#include <math.h>
#include <string.h>

enum { MIN_MATCHES=3 };
static const double speed_limit=.03;
static const double phase_coherence_limit=.65;
static bool positive(double x){return isfinite(x)&&x>0;}
product_config_t product_default_config(void)
{return (product_config_t){.mass_g=10,.trial_radius_mm=270,.trial_angle_deg=0,
    .correction_radius_mm=270,.axis=1,.direction=1,.vehicle="自行车 · 后轮"};}
bool product_config_valid(const product_config_t *c)
{return c && positive(c->mass_g)&&c->mass_g<=1000 && positive(c->trial_radius_mm)&&c->trial_radius_mm<=2000 &&
    positive(c->correction_radius_mm)&&c->correction_radius_mm<=2000 && isfinite(c->trial_angle_deg)&&
    c->trial_angle_deg>=0&&c->trial_angle_deg<360 && c->axis<3&&(c->direction==1||c->direction==-1)&&
    memchr(c->vehicle,0,sizeof(c->vehicle));}
const char *product_state_name(product_state_t s)
{static const char *v[]={"IDLE","ARMED","RECORDING","PROCESSING","COMPLETE","NO_RESULT","CANCELLED","TIMEOUT"};return (unsigned)s<8?v[s]:"NO_RESULT";}
const char *product_reason_name(product_reason_t s)
{static const char *v[]={"NONE","SENSOR_OFFLINE","DATA_FAULT","REFERENCE_INVALID","TOO_FEW_TURNS","NO_COMMON_SPEED",
    "WEAK_TRIAL_RESPONSE","INCONSISTENT_RESPONSE","SETUP_CHANGED","NUMERIC_ERROR","DEVICE_BUSY","WAIT_TIMEOUT","BASELINE_NEAR_ZERO","STORAGE_ERROR","LOW_1X_DIRECTION_COHERENCE","CAPTURE_LIMIT","ESTIMATE_CONFLICT"};
return (unsigned)s<17?v[s]:"NUMERIC_ERROR";}
static coherent_result_t average(const product_turn_t *turns,size_t n,uint32_t epoch)
{
    coherent_average_t core;coherent_result_t out;
    const coherent_config_t config={0,10000,MIN_MATCHES,epoch,SYNC_TIME_ESTIMATED};
    (void)coherent_average_init(&core,&config);
    for(size_t i=0;i<n;++i){coherent_input_t v={.reference_sequence=turns[i].reference,
        .time_us=turns[i].time_us,.epoch=epoch,.rpm=turns[i].rpm,.real=turns[i].real,.imag=turns[i].imag};
        (void)coherent_average_push(&core,&v);}
    coherent_average_result(&core,&out);return out;
}
bool product_extract_run(const selection_turn_t *turns,size_t count,unsigned axis,product_run_t *out)
{
    if(!out)return false;
    out->count=0;out->summary.captured=false;out->summary.turn_count=(uint32_t)count;
    if(!turns||axis>2||!out->summary.integrity_valid||!out->summary.epoch){out->summary.reason=PRODUCT_DATA_FAULT;return false;}
    for(size_t i=0;i<count;++i){const selection_turn_t *t=&turns[i];const coast_result_t *f=&t->fit;
        if(!f->valid||(t->reasons[axis]&~SELECT_WEAK_PHASE)||f->epoch!=out->summary.epoch||
           f->rpm<20||f->rpm>=300||!positive(f->rpm_previous)||
           !isfinite(f->axis[axis].real)||!isfinite(f->axis[axis].imag))continue;
        if(out->count==PRODUCT_MAX_TURNS){out->summary.reason=PRODUCT_DATA_FAULT;return false;}
        out->turns[out->count++]=(product_turn_t){f->rpm,f->rpm_previous,f->axis[axis].real,f->axis[axis].imag,f->ref_last,f->end_us};
    }
    out->summary.usable_turns=(uint32_t)out->count;out->summary.rejected_turns=(uint32_t)(count-out->count);
    if(out->count<MIN_MATCHES){out->summary.reason=PRODUCT_TOO_FEW_TURNS;return false;}
    coherent_result_t r=average(out->turns,out->count,out->summary.epoch);
    if(!r.mathematical_valid){out->summary.reason=PRODUCT_NUMERIC;return false;}
    out->summary.min_rpm=r.min_rpm;out->summary.max_rpm=r.max_rpm;
    out->summary.real=r.real;out->summary.imag=r.imag;out->summary.peak=r.peak;
    out->summary.scatter=r.vector_scatter_rms;out->summary.coherence=r.direction_coherence;
    out->summary.captured=true;out->summary.reason=PRODUCT_REASON_NONE;return true;
}
static bool context(const product_run_t *a,const product_run_t *b)
{return a&&b&&a->count<=PRODUCT_MAX_TURNS&&b->count<=PRODUCT_MAX_TURNS&&
    a->summary.captured&&b->summary.captured&&a->summary.integrity_valid&&b->summary.integrity_valid&&
    a->summary.epoch && a->summary.epoch==b->summary.epoch &&
    a->summary.end_us<=b->summary.start_us && a->summary.capture_id!=b->summary.capture_id;}
static bool run_finite(const product_run_t *run)
{
    for(size_t i=0;i<run->count;++i){const product_turn_t *t=&run->turns[i];
        if(!positive(t->rpm)||!positive(t->previous_rpm)||!isfinite(t->real)||!isfinite(t->imag))return false;}
    return true;
}
typedef struct {
    double mismatch;
    unsigned count;
} pair_score_t;

static bool better_pairing(pair_score_t candidate,pair_score_t current)
{
    return candidate.count>current.count ||
        (candidate.count==current.count&&candidate.mismatch<current.mismatch);
}

static bool pair_mismatch(const product_turn_t *a,const product_turn_t *b,double *error)
{
    const double current=fabs(a->rpm-b->rpm);
    const double previous=fabs(a->previous_rpm-b->previous_rpm);
    const double current_scale=fmax(a->rpm,b->rpm);
    const double previous_scale=fmax(a->previous_rpm,b->previous_rpm);
    if(current>speed_limit*current_scale||previous>speed_limit*previous_scale)return false;
    *error=fmax(current/current_scale,previous/previous_scale);
    return true;
}

product_reason_t product_match_runs(const product_run_t *a,const product_run_t *b,
    double lower,double upper,product_pairing_t *out)
{
    if(!out)return PRODUCT_CONTEXT;
    out->count=0;
    if(!context(a,b)||!isfinite(lower)||!isfinite(upper)||lower<20||upper>300||upper<=lower)
        return PRODUCT_CONTEXT;
    if(!run_finite(a)||!run_finite(b))return PRODUCT_NUMERIC;
    _Static_assert(PRODUCT_MAX_TURNS<=UINT8_MAX+1U,"pair index does not fit");

    /* Align the two chronological sequences. Prefer the most pairs, then
     * the lowest total speed mismatch. In a tie, match the later pair.
     * Two score rows and two bits per cell keep the working set bounded. */
    enum { SKIP_ORIGINAL,SKIP_MEASURED,MATCH };
    pair_score_t rolling[2][PRODUCT_MAX_TURNS+1]={0};
    uint8_t trace[(PRODUCT_MAX_TURNS*PRODUCT_MAX_TURNS+3)/4]={0};
    pair_score_t *previous=rolling[0],*current=rolling[1];
    for(size_t i=1;i<=a->count;++i){
        const product_turn_t *x=&a->turns[i-1];
        const bool within_band=x->rpm>=lower&&x->rpm<upper;
        for(size_t j=1;j<=b->count;++j){
            const product_turn_t *y=&b->turns[j-1];
            pair_score_t best=previous[j];
            unsigned move=SKIP_ORIGINAL;
            if(better_pairing(current[j-1],best)){
                best=current[j-1];move=SKIP_MEASURED;
            }
            double mismatch;
            if(within_band&&y->rpm>=20&&y->rpm<300&&pair_mismatch(x,y,&mismatch)){
                pair_score_t matched=previous[j-1];
                ++matched.count;
                matched.mismatch+=mismatch;
                if(!better_pairing(best,matched)){
                    best=matched;move=MATCH;
                }
            }
            current[j]=best;
            const size_t cell=(i-1)*b->count+j-1;
            trace[cell/4]|=(uint8_t)(move<<(2*(cell%4)));
        }
        pair_score_t *swap=previous;previous=current;current=swap;
    }

    out->count=previous[b->count].count;
    size_t remaining=out->count,i=a->count,j=b->count;
    while(remaining&&i&&j){
        const size_t cell=(i-1)*b->count+j-1;
        const unsigned move=(trace[cell/4]>>(2*(cell%4)))&3U;
        if(move==MATCH){
            --remaining;
            out->turns[remaining].original=(uint8_t)--i;
            out->turns[remaining].measured=(uint8_t)--j;
        }else if(move==SKIP_ORIGINAL)--i;
        else --j;
    }
    return PRODUCT_REASON_NONE;
}

static size_t copy_band_pairs(const product_run_t *a,const product_run_t *b,
    const product_pairing_t *pairs,double lower,double upper,
    product_turn_t *aa,product_turn_t *bb,product_turn_t *dd)
{
    size_t count=0;
    for(size_t i=0;i<pairs->count;++i){
        const product_turn_t *x=&a->turns[pairs->turns[i].original];
        const product_turn_t *y=&b->turns[pairs->turns[i].measured];
        if(x->rpm<lower||x->rpm>=upper)continue;
        aa[count]=*x;
        bb[count]=*y;
        if(dd){
            dd[count]=*x;
            dd[count].real=y->real-x->real;
            dd[count].imag=y->imag-x->imag;
        }
        ++count;
    }
    return count;
}
static bool solve_pairs(const product_config_t *cfg,uint32_t epoch,
    const product_turn_t *aa,const product_turn_t *bb,const product_turn_t *dd,
    size_t n,double lower,product_solution_t *out,double *score)
{
    memset(out,0,sizeof(*out));if(score)*score=0;
    out->matched_turns=(unsigned)n;out->lower_rpm=lower;out->upper_rpm=lower+20;
    if(n<MIN_MATCHES){out->reason=PRODUCT_NO_COMMON_SPEED;return false;}
    coherent_result_t x=average(aa,n,epoch),y=average(bb,n,epoch),d=average(dd,n,epoch);
    out->original_peak=x.peak;out->trial_peak=y.peak;out->delta_peak=d.peak;out->delta_coherence=d.direction_coherence;
    out->mean_noise_scale=d.vector_scatter_rms/sqrt((double)n);
    if(!x.mathematical_valid||!y.mathematical_valid||!d.mathematical_valid){out->reason=PRODUCT_NUMERIC;return false;}
    /* Empirical ranking, not a calibrated SNR or confidence interval. Include
     * numerator variation as well as trial-difference variation. */
    if(score){
        const double relative=out->mean_noise_scale/fmax(d.peak,1e-5)+
            x.vector_scatter_rms/sqrt((double)n)/fmax(x.peak,1e-5);
        *score=d.direction_coherence*sqrt((double)n/(n+6.0))/(1+3*relative);
        if(!isfinite(*score))*score=0;
    }
    if(!d.phase_defined||d.peak<1e-5||d.peak<=1.5*out->mean_noise_scale){out->reason=PRODUCT_WEAK_RESPONSE;return false;}
    if(d.direction_coherence<phase_coherence_limit){out->reason=PRODUCT_INCONSISTENT;return false;}
    double rpm=0,rpm1=0,variance=0,variance1=0;
    for(size_t i=0;i<n;++i){rpm+=aa[i].rpm/n;rpm1+=bb[i].rpm/n;}
    for(size_t i=0;i<n;++i){variance+=(aa[i].rpm-rpm)*(aa[i].rpm-rpm)/n;variance1+=(bb[i].rpm-rpm1)*(bb[i].rpm-rpm1)/n;}
    out->mean_rpm=(rpm+rpm1)/2;
    balance_config_t c=balance_default_config();c.trial_mass_g=cfg->mass_g;c.trial_radius_mm=cfg->trial_radius_mm;
    c.trial_angle_deg=cfg->trial_angle_deg;c.correction_radius_mm=cfg->correction_radius_mm;
    c.max_rpm_cv=1;c.max_rpm_mismatch=speed_limit;c.min_coherence=phase_coherence_limit;c.min_response=1e-5;c.min_response_snr=1.5;c.zero_vibration=1e-5;
    /* An almost-zero vector mean can also come from substantial per-turn
     * vibration pointing in different directions. Only waive the phase check
     * for a genuinely small signal at the existing near-zero threshold. */
    if((x.peak<=c.zero_vibration&&hypot(x.peak,x.vector_scatter_rms)>c.zero_vibration)||
       (y.peak<=c.zero_vibration&&hypot(y.peak,y.vector_scatter_rms)>c.zero_vibration)){
        out->reason=PRODUCT_INCONSISTENT;return false;
    }
    balance_measurement_t v0={.vibration={x.real,x.imag},.valid=true,.phase_valid=x.phase_defined,.qualified=false,
        .noise_rms=out->mean_noise_scale/sqrt(2),.rpm=rpm,.rpm_cv=sqrt(variance)/rpm,.coherence=x.direction_coherence,
        .setup_id=1,.epoch=epoch,.axis=cfg->axis,.direction=cfg->direction};
    balance_measurement_t v1=v0;v1.vibration=(balance_complex_t){y.real,y.imag};v1.phase_valid=y.phase_defined;v1.coherence=y.direction_coherence;
    v1.rpm=rpm1;v1.rpm_cv=sqrt(variance1)/rpm1;
    const balance_status_t status=balance_estimate(&c,&v0,&v1,&out->calculation);
    out->calculation_valid=out->calculation.calculation_valid;
    out->engineering_available=out->calculation_valid && out->calculation.angle_valid;
    out->reason=(status==BALANCE_OK||status==BALANCE_NO_CORRECTION)?PRODUCT_REASON_NONE:
        status==BALANCE_ERR_PHASE?PRODUCT_INCONSISTENT:status==BALANCE_ERR_WEAK_RESPONSE?PRODUCT_WEAK_RESPONSE:PRODUCT_NUMERIC;
    return out->calculation_valid;
}
bool product_evaluate_matched_band(const product_config_t *cfg,const product_run_t *a,const product_run_t *b,
    const product_pairing_t *pairs,double lower,product_solution_t *out,product_band_detail_t *detail)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));if(detail)memset(detail,0,sizeof(*detail));
    if(!product_config_valid(cfg)||!context(a,b)||!pairs||pairs->count>PRODUCT_MAX_TURNS||
        !isfinite(lower)||lower<20||lower>280||fmod(lower,20)!=0){out->reason=PRODUCT_CONTEXT;return false;}
    for(size_t i=0;i<pairs->count;++i)
        if(pairs->turns[i].original>=a->count||pairs->turns[i].measured>=b->count){
            out->reason=PRODUCT_CONTEXT;return false;}
    product_turn_t aa[PRODUCT_MAX_TURNS],bb[PRODUCT_MAX_TURNS],dd[PRODUCT_MAX_TURNS];
    const size_t n=copy_band_pairs(a,b,pairs,lower,lower+20,aa,bb,dd);
    double score=0;const bool ok=solve_pairs(cfg,a->summary.epoch,aa,bb,dd,n,lower,out,&score);
    if(detail){detail->score=score*.75;
        if(n>=MIN_MATCHES){const coherent_result_t original=average(aa,n,a->summary.epoch);
            if(original.mathematical_valid){detail->original_real=original.real;detail->original_imag=original.imag;}}
        if(ok&&n>=6){
            product_solution_t first,second;const size_t half=n/2;detail->split_available=1;
            detail->split_valid=solve_pairs(cfg,a->summary.epoch,aa,bb,dd,half,lower,&first,NULL)&&
                solve_pairs(cfg,a->summary.epoch,aa+half,bb+half,dd+half,n-half,lower,&second,NULL);
            if(detail->split_valid){
                const balance_complex_t u=first.calculation.correction_g_mm,v=second.calculation.correction_g_mm;
                const double magnitude=fmax(hypot(u.real,u.imag),hypot(v.real,v.imag));
                detail->split_relative_difference=hypot(u.real-v.real,u.imag-v.imag)/fmax(magnitude,1e-9);
                if(hypot(u.real,u.imag)>1e-9&&hypot(v.real,v.imag)>1e-9){
                    const double phase=atan2(u.imag,u.real)-atan2(v.imag,v.real);
                    detail->split_phase_difference=fabs(atan2(sin(phase),cos(phase)))*180/3.14159265358979323846;
                }
                detail->score=score*((detail->split_relative_difference>.35||detail->split_phase_difference>30)?.35:1);
            }else detail->score=score*.5;
        }
    }
    return ok;
}
bool product_evaluate_band(const product_config_t *cfg,const product_run_t *a,const product_run_t *b,
    double lower,product_solution_t *out,product_band_detail_t *detail)
{
    if(!out)return false;
    product_pairing_t pairs;
    const product_reason_t reason=product_match_runs(a,b,20,300,&pairs);
    if(reason!=PRODUCT_REASON_NONE){
        memset(out,0,sizeof(*out));out->reason=reason;
        if(detail)memset(detail,0,sizeof(*detail));
        return false;
    }
    return product_evaluate_matched_band(cfg,a,b,&pairs,lower,out,detail);
}
unsigned product_common_turns(const product_run_t *a,const product_run_t *b,double lower,double upper)
{
    product_pairing_t pairs;
    if(upper>lower)return product_match_runs(a,b,lower,upper,&pairs)==PRODUCT_REASON_NONE?(unsigned)pairs.count:0;
    if(product_match_runs(a,b,20,300,&pairs)!=PRODUCT_REASON_NONE)return 0;
    unsigned counts[PRODUCT_ASSESSMENT_BANDS]={0},maximum=0;
    for(size_t i=0;i<pairs.count;++i){
        const unsigned band=(unsigned)(a->turns[pairs.turns[i].original].rpm/20)-1U;
        if(++counts[band]>maximum)maximum=counts[band];
    }
    return maximum;
}
bool product_solve_runs(const product_config_t *cfg,const product_run_t *a,const product_run_t *b,product_solution_t *out)
{return product_assess_runs(cfg,a,b,out,NULL);}

product_reason_t product_correction_status(const product_solution_t *s)
{
    if(!s||!s->calculation_valid)return PRODUCT_CONTEXT;
    if(s->reason!=PRODUCT_REASON_NONE)return s->reason;
    if(!isfinite(s->calculation.mass_g)||s->calculation.mass_g<0)return PRODUCT_NUMERIC;
    if(s->calculation.mass_g==0)return s->calculation.angle_valid?PRODUCT_NUMERIC:PRODUCT_BASELINE_ZERO;
    if(!s->engineering_available||!s->calculation.angle_valid)return PRODUCT_CONTEXT;
    const double angle=s->calculation.angle_deg;
    return isfinite(angle)&&angle>=0&&angle<360?PRODUCT_REASON_NONE:PRODUCT_NUMERIC;
}
bool product_compare_runs(const product_config_t *cfg,const product_run_t *a,const product_run_t *b,
    const product_solution_t *solution,product_comparison_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(!product_config_valid(cfg)||!context(a,b)){out->reason=PRODUCT_CONTEXT;return false;}
    out->reason=product_correction_status(solution);
    if(out->reason!=PRODUCT_REASON_NONE)return false;
    product_pairing_t pairs;
    out->reason=product_match_runs(a,b,solution->lower_rpm,solution->upper_rpm,&pairs);
    if(out->reason!=PRODUCT_REASON_NONE)return false;
    product_turn_t aa[PRODUCT_MAX_TURNS],bb[PRODUCT_MAX_TURNS];
    const size_t n=copy_band_pairs(a,b,&pairs,solution->lower_rpm,solution->upper_rpm,aa,bb,NULL);
    out->matched_turns=(unsigned)n;out->lower_rpm=solution->lower_rpm;out->upper_rpm=solution->upper_rpm;
    if(n<MIN_MATCHES){out->reason=PRODUCT_NO_COMMON_SPEED;return false;}
    coherent_result_t x=average(aa,n,a->summary.epoch),y=average(bb,n,a->summary.epoch);
    double energy0=0,energy1=0;
    for(size_t i=0;i<n;++i){energy0+=(aa[i].real*aa[i].real+aa[i].imag*aa[i].imag)/n;energy1+=(bb[i].real*bb[i].real+bb[i].imag*bb[i].imag)/n;}
    out->original_peak=sqrt(energy0);out->retest_peak=sqrt(energy1);
    if(!x.mathematical_valid||!y.mathematical_valid){out->reason=PRODUCT_NUMERIC;return false;}
    if(out->original_peak<=1e-5){out->reason=PRODUCT_BASELINE_ZERO;return false;}
    out->improvement_percent=100*(1-out->retest_peak/out->original_peak);
    out->available=isfinite(out->improvement_percent);
    /* Residual direction can be weak after a useful correction. Keep the
     * magnitude comparison, but never present it as angle validation. */
    out->reason=!out->available?PRODUCT_NUMERIC:
        (!x.phase_defined||!y.phase_defined||x.direction_coherence<phase_coherence_limit||y.direction_coherence<phase_coherence_limit)?
        PRODUCT_LOW_DIRECTION_COHERENCE:PRODUCT_REASON_NONE;
    return out->available;
}

const char *product_repeat_state_name(product_repeat_state_t state)
{
    static const char *names[]={"NOT_CHECKED","NOT_COMPARABLE","OBSERVATIONS_CLOSE","VARIABLE_RESPONSE"};
    return (unsigned)state<4?names[state]:"NOT_COMPARABLE";
}
void product_observe_repeat(const product_config_t *cfg,const product_run_t *a,const product_run_t *b,
    const product_solution_t *solution,product_repeat_observation_t *out)
{
    if(!out)return;
    memset(out,0,sizeof(*out));out->state=PRODUCT_REPEAT_UNCOMPARABLE;out->reason=PRODUCT_CONTEXT;
    if(a)out->previous_capture_id=a->summary.capture_id;
    if(!product_config_valid(cfg)||!context(a,b))return;
    out->reason=product_correction_status(solution);
    if(out->reason!=PRODUCT_REASON_NONE)return;
    product_pairing_t pairs;
    out->reason=product_match_runs(a,b,solution->lower_rpm,solution->upper_rpm,&pairs);
    if(out->reason!=PRODUCT_REASON_NONE)return;
    product_turn_t aa[PRODUCT_MAX_TURNS],bb[PRODUCT_MAX_TURNS];
    const size_t n=copy_band_pairs(a,b,&pairs,solution->lower_rpm,solution->upper_rpm,aa,bb,NULL);
    out->matched_turns=(unsigned)n;
    if(n<MIN_MATCHES){out->reason=PRODUCT_NO_COMMON_SPEED;return;}
    coherent_result_t x=average(aa,n,a->summary.epoch),y=average(bb,n,a->summary.epoch);
    double e0=0,e1=0;
    for(size_t i=0;i<n;++i){e0+=(aa[i].real*aa[i].real+aa[i].imag*aa[i].imag)/n;
        e1+=(bb[i].real*bb[i].real+bb[i].imag*bb[i].imag)/n;}
    out->previous_peak=sqrt(e0);out->current_peak=sqrt(e1);
    if(!x.mathematical_valid||!y.mathematical_valid||!isfinite(out->previous_peak)||!isfinite(out->current_peak)){
        out->reason=PRODUCT_NUMERIC;return;}
    if(fmin(out->previous_peak,out->current_peak)<=1e-5){out->reason=PRODUCT_BASELINE_ZERO;return;}
    out->amplitude_change_percent=100*(out->current_peak/out->previous_peak-1);
    out->phase_available=x.phase_defined&&y.phase_defined&&
        x.direction_coherence>=phase_coherence_limit&&y.direction_coherence>=phase_coherence_limit;
    if(out->phase_available){
        const double change=atan2(y.imag,y.real)-atan2(x.imag,x.real);
        out->phase_change_deg=atan2(sin(change),cos(change))*180/3.14159265358979323846;
    }
    if(!isfinite(out->amplitude_change_percent)||!isfinite(out->phase_change_deg)){out->reason=PRODUCT_NUMERIC;return;}
    const bool variable=fmax(out->previous_peak,out->current_peak)>1.25*fmin(out->previous_peak,out->current_peak)||
        (out->phase_available&&fabs(out->phase_change_deg)>30);
    out->state=variable?PRODUCT_REPEAT_VARIABLE:PRODUCT_REPEAT_OBSERVATION_CLOSE;
    out->reason=out->phase_available?PRODUCT_REASON_NONE:PRODUCT_LOW_DIRECTION_COHERENCE;
}
