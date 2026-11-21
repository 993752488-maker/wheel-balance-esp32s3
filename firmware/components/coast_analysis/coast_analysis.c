#include "coast_analysis.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
#define N 7U

coast_config_t coast_default_config(void)
{
    return (coast_config_t){.min_samples=24,.max_sample_gap_us=4000,
        .max_speed_change=.15,.acceleration_tolerance=.005,.max_angular_gap_fraction=.2,
        .min_phase_peak=1e-5,.min_fit_snr=5};
}

const char *coast_status_name(coast_status_t status)
{
    switch(status) {
    case COAST_WAIT_INPUT:return "WAIT_INPUT";
    case COAST_OK:return "FIT_ESTIMATED";
    case COAST_ERR_ARGUMENT:return "INVALID_PARAMETER";
    case COAST_ERR_INPUT:return "INPUT_FAULT";
    case COAST_ERR_REFERENCE:return "REFERENCE_ERROR";
    case COAST_ERR_SPEED_CHANGE:return "SPEED_CHANGE_TOO_LARGE";
    case COAST_ERR_ACCELERATING:return "NOT_COASTING_TREND";
    case COAST_ERR_SAMPLES:return "INSUFFICIENT_SAMPLES";
    case COAST_ERR_NUMERIC:return "NUMERIC_ERROR";
    default:return "INVALID_PARAMETER";
    }
}

static coast_status_t fail(coast_result_t *r,coast_status_t error)
{ memset(r,0,sizeof(*r)); r->status=error; return error; }

static bool fraction(double x)
{ return isfinite(x) && x>=0 && x<1; }

static void basis(double theta,double b[N])
{
    b[0]=1;
    for(unsigned h=1;h<=3;++h) { b[2*h-1]=cos(h*theta); b[2*h]=sin(h*theta); }
}

static bool solve(double a[N][N+3],size_t count)
{
    for(unsigned k=0;k<N;++k) {
        unsigned pivot=k;
        for(unsigned row=k+1;row<N;++row)if(fabs(a[row][k])>fabs(a[pivot][k]))pivot=row;
        if(!isfinite(a[pivot][k]) || fabs(a[pivot][k]) <= (double)count*1e-10)return false;
        if(pivot!=k)for(unsigned j=0;j<N+3;++j){double t=a[k][j];a[k][j]=a[pivot][j];a[pivot][j]=t;}
        const double d=a[k][k];
        for(unsigned j=k;j<N+3;++j)a[k][j]/=d;
        for(unsigned row=0;row<N;++row)if(row!=k){double f=a[row][k];for(unsigned j=k;j<N+3;++j)a[row][j]-=f*a[k][j];}
    }
    for(unsigned row=0;row<N;++row)for(unsigned axis=0;axis<3;++axis)
        if(!isfinite(a[row][N+axis]))return false;
    return true;
}

coast_status_t coast_analyze(const coast_config_t *c,const sync_sample_t *s,size_t count,
    const sync_reference_t ref[3],sync_quality_t quality,coast_result_t *r)
{
    if(!r)return COAST_ERR_ARGUMENT;
    memset(r,0,sizeof(*r));
    if(!c || !s || !ref || c->min_samples<N || !c->max_sample_gap_us ||
        !fraction(c->max_speed_change) || !fraction(c->acceleration_tolerance) ||
        !fraction(c->max_angular_gap_fraction) || c->max_angular_gap_fraction<=0 ||
        !isfinite(c->min_phase_peak) || c->min_phase_peak<=0 ||
        !isfinite(c->min_fit_snr) || c->min_fit_snr<=0 || count>SYNC_MAX_SAMPLES)
        return fail(r,COAST_ERR_ARGUMENT);
    if(quality.faults || (quality.time_quality!=SYNC_TIME_ESTIMATED && quality.time_quality!=SYNC_TIME_SYNTHETIC))
        return fail(r,COAST_ERR_INPUT);
    for(unsigned i=0;i<2;++i)
        if(!ref[i].sequence || ref[i].sequence==UINT64_MAX ||
            ref[i+1].sequence!=ref[i].sequence+1 || ref[i+1].time_us<=ref[i].time_us)
            return fail(r,COAST_ERR_REFERENCE);
    const uint64_t previous=ref[1].time_us-ref[0].time_us,period=ref[2].time_us-ref[1].time_us;
    const double rpm_previous=60000000.0/(double)previous,rpm=60000000.0/(double)period;
    const double change=fabs(rpm-rpm_previous)/fmax(rpm,rpm_previous);
    if(change>c->max_speed_change)return fail(r,COAST_ERR_SPEED_CHANGE);
    if(rpm>rpm_previous*(1+c->acceleration_tolerance))return fail(r,COAST_ERR_ACCELERATING);
    const double ratio=(double)period/(double)previous;
    const double warp=ratio*(1-ratio)/(1+ratio);
    if(!isfinite(warp) || fabs(warp)>=1)return fail(r,COAST_ERR_NUMERIC);
    if(count<c->min_samples)return fail(r,COAST_ERR_SAMPLES);
    const uint32_t epoch=s[0].epoch;
    if(!epoch)return fail(r,COAST_ERR_INPUT);
    double first_fraction=0,previous_fraction=0,largest_angle_gap=0;
    for(size_t i=0;i<count;++i) {
        if(!s[i].sequence || s[i].epoch!=epoch ||
            s[i].time_us<ref[1].time_us || s[i].time_us>=ref[2].time_us ||
            (i && (s[i-1].sequence==UINT64_MAX || s[i].sequence!=s[i-1].sequence+1 ||
                   s[i].time_us<=s[i-1].time_us || s[i].time_us-s[i-1].time_us>c->max_sample_gap_us)))
            return fail(r,COAST_ERR_INPUT);
        const double x=(double)(s[i].time_us-ref[1].time_us)/(double)period;
        const double fraction=x+warp*x*(x-1);
        if(!i)first_fraction=fraction;
        else largest_angle_gap=fmax(largest_angle_gap,fraction-previous_fraction);
        previous_fraction=fraction;
        for(unsigned axis=0;axis<3;++axis)if(!isfinite(s[i].acceleration[axis]))return fail(r,COAST_ERR_NUMERIC);
    }
    largest_angle_gap=fmax(largest_angle_gap,first_fraction+1-previous_fraction);
    if(largest_angle_gap>c->max_angular_gap_fraction)return fail(r,COAST_ERR_SAMPLES);
    double a[N][N+3]={{0}},rhs[N][3]={{0}},yy[3]={0};
    for(size_t i=0;i<count;++i) {
        const double x=(double)(s[i].time_us-ref[1].time_us)/(double)period;
        double b[N];basis(2*PI*(x+warp*x*(x-1)),b);
        for(unsigned row=0;row<N;++row) {
            for(unsigned col=0;col<N;++col)a[row][col]+=b[row]*b[col];
            for(unsigned axis=0;axis<3;++axis)rhs[row][axis]+=b[row]*s[i].acceleration[axis];
        }
        for(unsigned axis=0;axis<3;++axis)yy[axis]+=s[i].acceleration[axis]*s[i].acceleration[axis];
    }
    for(unsigned row=0;row<N;++row)for(unsigned axis=0;axis<3;++axis)a[row][N+axis]=rhs[row][axis];
    if(!solve(a,count))return fail(r,COAST_ERR_NUMERIC);
    for(unsigned axis=0;axis<3;++axis) {
        coast_axis_t *v=&r->axis[axis];
        v->dc=a[0][N+axis];v->real=a[1][N+axis];v->imag=-a[2][N+axis];
        v->peak=hypot(v->real,v->imag);v->rms=v->peak/sqrt(2.0);
        double sse=yy[axis];
        for(unsigned row=0;row<N;++row)sse-=a[row][N+axis]*rhs[row][axis];
        if(!isfinite(sse) || sse < -1e-7*fmax(yy[axis],1) || !isfinite(v->peak))return fail(r,COAST_ERR_NUMERIC);
        v->residual_rms=sqrt(fmax(sse,0)/(double)count);
        v->fit_snr=v->peak/fmax(sqrt(2.0)*v->residual_rms,c->min_phase_peak/c->min_fit_snr);
        if(!isfinite(v->fit_snr))return fail(r,COAST_ERR_NUMERIC);
        v->phase_available=v->peak>=c->min_phase_peak && v->fit_snr>=c->min_fit_snr;
        if(v->phase_available) {
            v->phase_estimate_deg=fmod(atan2(v->imag,v->real)*180/PI+360,360);
        }
    }
    double half_energy[2][3]={{0}};
    size_t half_count[2]={0};
    /* Evaluate the complete fitted waveform: a real 2x/3x component must not
     * be mislabelled as a transient by subtracting only the 1x fit. */
    for(size_t i=0;i<count;++i) {
        const double x=(double)(s[i].time_us-ref[1].time_us)/(double)period;
        double b[N];basis(2*PI*(x+warp*x*(x-1)),b);
        for(unsigned axis=0;axis<3;++axis) {
            const unsigned half=x<.5?0:1;
            double predicted=0;for(unsigned j=0;j<N;++j)predicted+=a[j][N+axis]*b[j];
            const double residual=fabs(s[i].acceleration[axis]-predicted);
            if(!isfinite(residual))return fail(r,COAST_ERR_NUMERIC);
            half_energy[half][axis]+=residual*residual;
            r->axis[axis].residual_peak=fmax(r->axis[axis].residual_peak,residual);
        }
        ++half_count[x<.5?0:1];
    }
    if(!half_count[0] || !half_count[1])return fail(r,COAST_ERR_SAMPLES);
    for(unsigned axis=0;axis<3;++axis) {
        r->axis[axis].residual_crest=r->axis[axis].residual_peak/fmax(r->axis[axis].residual_rms,1e-12);
        r->axis[axis].residual_early_rms=sqrt(half_energy[0][axis]/(double)half_count[0]);
        r->axis[axis].residual_late_rms=sqrt(half_energy[1][axis]/(double)half_count[1]);
        if(!isfinite(r->axis[axis].residual_early_rms) || !isfinite(r->axis[axis].residual_late_rms))
            return fail(r,COAST_ERR_NUMERIC);
    }
    r->status=COAST_OK;r->valid=true;r->simulated=quality.time_quality==SYNC_TIME_SYNTHETIC;
    r->measurement_qualified=false;r->samples=count;r->epoch=epoch;
    r->previous_ref_sequence=ref[0].sequence;r->ref_first=ref[1].sequence;r->ref_last=ref[2].sequence;
    r->previous_ref_us=ref[0].time_us;r->start_us=ref[1].time_us;r->end_us=ref[2].time_us;
    r->rpm_previous=rpm_previous;r->rpm=rpm;r->speed_change=change;r->phase_warp=warp;
    return COAST_OK;
}
