#include "coast_precision.h"
#include <math.h>
#include <string.h>

#define P 7U
#define PI 3.14159265358979323846
precision_config_t precision_default_config(void)
{
    return (precision_config_t){.fit=coast_default_config(),.min_samples=64,.alpha=.025};
}
static void basis(const sync_sample_t *s,const sync_reference_t ref[3],double b[P])
{
    const double ratio=(double)(ref[2].time_us-ref[1].time_us)/(double)(ref[1].time_us-ref[0].time_us);
    const double warp=ratio*(1-ratio)/(1+ratio);
    const double x=(double)(s->time_us-ref[1].time_us)/(double)(ref[2].time_us-ref[1].time_us);
    const double theta=2*PI*(x+warp*x*(x-1));b[0]=1;
    for(unsigned k=1;k<=3;++k){b[2*k-1]=cos(k*theta);b[2*k]=sin(k*theta);}
}
/* The independent regression implementation also yields the design inverse.
 * Input continuity is validated once by coast_analyze. */
static bool regress(const sync_sample_t *s,size_t n,const sync_reference_t ref[3],
    double beta[P][3],double inverse[P][P])
{
    double m[P][P+3+P]={{0}};
    for(size_t i=0;i<n;++i) {
        double b[P];basis(s+i,ref,b);
        for(unsigned j=0;j<P;++j){
            for(unsigned k=0;k<P;++k)m[j][k]+=b[j]*b[k];
            for(unsigned a=0;a<3;++a)m[j][P+a]+=b[j]*s[i].acceleration[a];
        }
    }
    for(unsigned j=0;j<P;++j)m[j][P+3+j]=1;
    for(unsigned j=0;j<P;++j) {
        unsigned pivot=j;
        for(unsigned k=j+1;k<P;++k)if(fabs(m[k][j])>fabs(m[pivot][j]))pivot=k;
        if(!isfinite(m[pivot][j]) || fabs(m[pivot][j])<n*1e-10)return false;
        if(pivot!=j)for(unsigned k=0;k<P+3+P;++k){double t=m[j][k];m[j][k]=m[pivot][k];m[pivot][k]=t;}
        const double d=m[j][j];for(unsigned k=0;k<P+3+P;++k)m[j][k]/=d;
        for(unsigned row=0;row<P;++row)if(row!=j){const double f=m[row][j];for(unsigned k=0;k<P+3+P;++k)m[row][k]-=f*m[j][k];}
    }
    for(unsigned j=0;j<P;++j)for(unsigned a=0;a<3;++a){beta[j][a]=m[j][P+a];if(!isfinite(beta[j][a]))return false;}
    for(unsigned j=0;j<P;++j)for(unsigned k=0;k<P;++k){inverse[j][k]=m[j][P+3+k];if(!isfinite(inverse[j][k]))return false;}
    return true;
}
static double largest_eigenvalue(double xx,double xy,double yy)
{
    return fmax(0,(xx+yy+hypot(xx-yy,2*xy))/2);
}
static bool polar(double real,double imag,double radius,double *peak,double *phase,double *halfwidth)
{
    *peak=hypot(real,imag);*phase=0;*halfwidth=180;
    if(*peak<=radius || *peak<=0)return false;
    *phase=fmod(atan2(imag,real)*180/PI+360,360);
    *halfwidth=asin(fmin(1,radius/(*peak)))*180/PI;return true;
}
static precision_status_t fail(precision_result_t *r,precision_status_t status)
{
    memset(r,0,sizeof(*r));r->status=status;return status;
}
precision_status_t coast_precision(const precision_config_t *cfg,const sync_sample_t *s,size_t n,
    const sync_reference_t ref[3],sync_quality_t quality,precision_result_t *r)
{
    if(!r)return PRECISION_ARGUMENT;
    memset(r,0,sizeof(*r));
    if(!cfg || !isfinite(cfg->alpha) || cfg->alpha<=0 || cfg->alpha>=.5 || cfg->min_samples<32)
        return fail(r,PRECISION_ARGUMENT);
    coast_result_t fitted;
    const coast_status_t checked=coast_analyze(&cfg->fit,s,n,ref,quality,&fitted);
    if(checked!=COAST_OK)return fail(r,checked==COAST_ERR_ARGUMENT?PRECISION_ARGUMENT:PRECISION_INPUT);
    if(n<cfg->min_samples)return fail(r,PRECISION_SAMPLES);
    double beta[P][3],inverse[P][P],sse[3]={0};
    if(!regress(s,n,ref,beta,inverse))return fail(r,PRECISION_NUMERIC);
    for(size_t i=0;i<n;++i) {
        double b[P];basis(s+i,ref,b);
        for(unsigned a=0;a<3;++a){double y=0;for(unsigned j=0;j<P;++j)y+=beta[j][a]*b[j];const double e=s[i].acceleration[a]-y;sse[a]+=e*e;}
    }
    const double df=(double)(n-P);
    /* 2*F(2,df) quantile, exact only for fixed-design IID Gaussian errors. */
    const double iid_critical=df*expm1(-2*log(cfg->alpha)/df);
    for(unsigned a=0;a<3;++a) {
        precision_axis_t *v=r->axis+a;v->real=beta[1][a];v->imag=-beta[2][a];
        const double variance=sse[a]/df;
        v->iid_radius=sqrt(iid_critical*variance*largest_eigenvalue(inverse[1][1],-inverse[1][2],inverse[2][2]));
        v->radius=v->iid_radius;
    }
    const unsigned groups[3]={4,8,16};double score[16][3][2];
    for(unsigned scale=0;scale<3;++scale) {
        const unsigned g=groups[scale];memset(score,0,sizeof(score));
        for(size_t i=0;i<n;++i) {
            double b[P],influence[2]={0},leverage=0;basis(s+i,ref,b);
            for(unsigned j=0;j<P;++j) {
                influence[0]+=inverse[1][j]*b[j];influence[1]-=inverse[2][j]*b[j];
                for(unsigned k=0;k<P;++k)leverage+=b[j]*inverse[j][k]*b[k];
            }
            if(!isfinite(leverage) || leverage>=1-1e-10)return fail(r,PRECISION_NUMERIC);
            const double hc2=1/sqrt(fmax(1-leverage,1e-10));
            const size_t block=i*g/n;
            for(unsigned a=0;a<3;++a){double predicted=0;for(unsigned j=0;j<P;++j)predicted+=b[j]*beta[j][a];const double e=(s[i].acceleration[a]-predicted)*hc2;score[block][a][0]+=influence[0]*e;score[block][a][1]+=influence[1]*e;}
        }
        /* Fixed-design cluster scores preserve angular coverage. Small-group
         * inflation is heuristic: dependent time blocks are not an exact
         * Hotelling sample. Never label this as certified physical coverage. */
        const double inflation=(g-1)*expm1(-2*log(cfg->alpha)/(g-2));
        for(unsigned a=0;a<3;++a) {
            double xx=0,xy=0,yy=0;
            for(unsigned block=0;block<g;++block){const double x=score[block][a][0],y=score[block][a][1];xx+=x*x;xy+=x*y;yy+=y*y;}
            const double factor=(double)g/(g-1)*(n-1)/(n-P);
            precision_axis_t *v=r->axis+a;
            v->block_radius[scale]=sqrt(inflation*factor*largest_eigenvalue(xx,xy,yy));
            v->radius=fmax(v->radius,v->block_radius[scale]);
        }
    }
    for(unsigned a=0;a<3;++a) {
        precision_axis_t *v=r->axis+a;
        if(!isfinite(v->radius) || !isfinite(v->real) || !isfinite(v->imag))return fail(r,PRECISION_NUMERIC);
        v->phase_bounded=polar(v->real,v->imag,v->radius,&v->peak,&v->phase_deg,&v->phase_halfwidth_deg);
    }
    r->status=PRECISION_OK;r->valid=true;r->fit=fitted;r->alpha=cfg->alpha;
    r->simulated=quality.time_quality==SYNC_TIME_SYNTHETIC;r->measurement_qualified=false;return PRECISION_OK;
}
bool coast_precision_delta(const precision_result_t *a,const precision_result_t *b,unsigned axis,
    double maximum,bool same_setup,precision_delta_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(!same_setup || !a || !b || axis>2 || !isfinite(maximum) || maximum<=0 || maximum>=1 ||
       !a->valid || !b->valid || !a->fit.valid || !b->fit.valid || a->status!=PRECISION_OK || b->status!=PRECISION_OK || a->simulated!=b->simulated ||
       !isfinite(a->fit.rpm) || !isfinite(b->fit.rpm) || !isfinite(a->fit.rpm_previous) || !isfinite(b->fit.rpm_previous) ||
       a->fit.epoch!=b->fit.epoch || !a->fit.epoch || a->fit.rpm<=0 || b->fit.rpm<=0 ||
       a->fit.rpm_previous<=0 || b->fit.rpm_previous<=0)return false;
    const double m=fabs(a->fit.rpm-b->fit.rpm)/fmax(a->fit.rpm,b->fit.rpm);
    const double previous=fabs(a->fit.rpm_previous-b->fit.rpm_previous)/fmax(a->fit.rpm_previous,b->fit.rpm_previous);
    if(m>maximum || previous>maximum)return false;
    const precision_axis_t *x=a->axis+axis,*y=b->axis+axis;
    if(!isfinite(x->radius) || !isfinite(y->radius) || x->radius<0 || y->radius<0)return false;
    out->real=y->real-x->real;out->imag=y->imag-x->imag;out->radius=x->radius+y->radius;
    if(!isfinite(out->real)||!isfinite(out->imag)||!isfinite(out->radius)||out->radius<0){memset(out,0,sizeof(*out));return false;}
    out->phase_bounded=polar(out->real,out->imag,out->radius,&out->peak,&out->phase_deg,&out->phase_halfwidth_deg);
    out->resolved=out->phase_bounded;out->valid=true;out->rpm_mismatch=m;
    out->measurement_qualified=false;return true;
}
