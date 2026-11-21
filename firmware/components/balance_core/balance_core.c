#include "balance_core.h"
#include <math.h>
#include <string.h>

#define PI 3.1415926535897932384626433832795

balance_config_t balance_default_config(void)
{
    return (balance_config_t){.trial_mass_g=5, .trial_radius_mm=100,
        .trial_angle_deg=0, .correction_radius_mm=100, .max_rpm_cv=0.01,
        .max_rpm_mismatch=0.01, .min_coherence=0.8, .min_response=0.01,
        .min_response_snr=5, .zero_vibration=0.001, .max_mass_g=50};
}

const char *balance_status_name(balance_status_t s)
{
    switch (s) {
    case BALANCE_OK:return "OK";
    case BALANCE_NO_CORRECTION:return "NO_CORRECTION";
    case BALANCE_ERR_ARGUMENT:return "INVALID_PARAMETER";
    case BALANCE_ERR_UNQUALIFIED:return "UNQUALIFIED_TIMING";
    case BALANCE_ERR_SOURCE:return "MIXED_SOURCE";
    case BALANCE_ERR_CONTEXT:return "SETUP_CHANGED";
    case BALANCE_ERR_UNSTABLE:return "UNSTABLE_RPM";
    case BALANCE_ERR_SPEED:return "SPEED_MISMATCH";
    case BALANCE_ERR_PHASE:return "PHASE_UNRELIABLE";
    case BALANCE_ERR_WEAK_RESPONSE:return "TRIAL_RESPONSE_TOO_SMALL";
    case BALANCE_ERR_LIMIT:return "MASS_LIMIT";
    case BALANCE_ERR_NUMERIC:return "NUMERIC_ERROR";
    default:return "UNKNOWN_ERROR";
    }
}

static bool positive(double v) { return isfinite(v) && v>0; }
static bool fraction(double v) { return isfinite(v) && v>=0 && v<=1; }
static bool measurement_ok(const balance_measurement_t *m)
{
    return m && m->valid && isfinite(m->vibration.real) && isfinite(m->vibration.imag) &&
        isfinite(m->noise_rms) && m->noise_rms>=0 && positive(m->rpm) &&
        fraction(m->rpm_cv) && fraction(m->coherence) && m->axis<3 &&
        (m->direction==1 || m->direction==-1) && m->setup_id && m->epoch;
}
static balance_status_t fail(balance_result_t *r, balance_status_t status)
{
    memset(r,0,sizeof(*r)); r->status=status; return status;
}
static bool divide(balance_complex_t a, balance_complex_t b, balance_complex_t *out)
{
    /* Scale denominator first; no squared tiny/huge denominator. */
    const double scale=fmax(fabs(b.real),fabs(b.imag));
    if (!positive(scale)) return false;
    const double x=b.real/scale,y=b.imag/scale,d=x*x+y*y;
    const double ar=a.real/scale,ai=a.imag/scale;
    *out=(balance_complex_t){(ar*x+ai*y)/d,(ai*x-ar*y)/d};
    return isfinite(out->real) && isfinite(out->imag);
}

static balance_status_t solve(const balance_config_t *c,
    const balance_measurement_t *v0, const balance_measurement_t *v1,
    balance_result_t *r,bool certified)
{
    if (!r) return BALANCE_ERR_ARGUMENT;
    memset(r,0,sizeof(*r));
    if (!c || !measurement_ok(v0) || !measurement_ok(v1) ||
        !positive(c->trial_mass_g) || !positive(c->trial_radius_mm) ||
        !positive(c->correction_radius_mm) || !isfinite(c->trial_angle_deg) ||
        !fraction(c->max_rpm_cv) || !fraction(c->max_rpm_mismatch) ||
        !fraction(c->min_coherence) || !positive(c->min_response) ||
        !positive(c->min_response_snr) || !positive(c->zero_vibration) ||
        !positive(c->max_mass_g)) return fail(r,BALANCE_ERR_ARGUMENT);
    if (v0->simulated != v1->simulated) return fail(r,BALANCE_ERR_SOURCE);
    if (certified && !v0->simulated && (!v0->qualified || !v1->qualified))
        return fail(r,BALANCE_ERR_UNQUALIFIED);
    if (v0->setup_id!=v1->setup_id || v0->epoch!=v1->epoch ||
        v0->axis!=v1->axis || v0->direction!=v1->direction)
        return fail(r,BALANCE_ERR_CONTEXT);
    if (v0->rpm_cv>c->max_rpm_cv || v1->rpm_cv>c->max_rpm_cv)
        return fail(r,BALANCE_ERR_UNSTABLE);
    const double speed=fabs(v0->rpm-v1->rpm)/fmax(v0->rpm,v1->rpm);
    if (speed>c->max_rpm_mismatch) return fail(r,BALANCE_ERR_SPEED);
    const double original=hypot(v0->vibration.real,v0->vibration.imag);
    const double trial=hypot(v1->vibration.real,v1->vibration.imag);
    if (!isfinite(original) || !isfinite(trial)) return fail(r,BALANCE_ERR_NUMERIC);
    if (original>c->zero_vibration && (!v0->phase_valid || v0->coherence<c->min_coherence))
        return fail(r,BALANCE_ERR_PHASE);
    if (trial>c->zero_vibration && (!v1->phase_valid || v1->coherence<c->min_coherence))
        return fail(r,BALANCE_ERR_PHASE);
    const double ut=c->trial_mass_g*c->trial_radius_mm;
    const double angle=fmod(c->trial_angle_deg,360.0)*PI/180.0;
    const balance_complex_t delta={v1->vibration.real-v0->vibration.real,
                                   v1->vibration.imag-v0->vibration.imag};
    const double response=hypot(delta.real,delta.imag),noise=hypot(v0->noise_rms,v1->noise_rms);
    if (!positive(ut) || !isfinite(response) || !isfinite(noise)) return fail(r,BALANCE_ERR_NUMERIC);
    if (response<c->min_response || response<=c->min_response_snr*noise)
        return fail(r,BALANCE_ERR_WEAK_RESPONSE);
    const balance_complex_t h={(delta.real*cos(angle)+delta.imag*sin(angle))/ut,
                               (delta.imag*cos(angle)-delta.real*sin(angle))/ut};
    balance_complex_t correction;
    const balance_complex_t negative={-v0->vibration.real,-v0->vibration.imag};
    if (!divide(negative,h,&correction)) return fail(r,BALANCE_ERR_NUMERIC);
    const double mass=hypot(correction.real,correction.imag)/c->correction_radius_mm;
    if (!isfinite(mass) || (original>c->zero_vibration && mass<=0))
        return fail(r,BALANCE_ERR_NUMERIC);
    if (mass>c->max_mass_g) return fail(r,BALANCE_ERR_LIMIT);
    r->influence=h; r->correction_g_mm=correction; r->mass_g=mass;
    r->angle_deg=fmod(atan2(correction.imag,correction.real)*180.0/PI+360.0,360.0);
    r->delta_peak=response;
    /* No Infinity is exported: zero residual noise uses a finite ratio scale. */
    r->response_snr=response/fmax(noise,c->min_response/c->min_response_snr);
    if (!isfinite(r->response_snr)) return fail(r,BALANCE_ERR_NUMERIC);
    r->rpm_mismatch=speed; r->simulated=v0->simulated;
    r->calculation_valid=true; r->recommendation_valid=certified && !v0->simulated;
    r->angle_valid=original>c->zero_vibration;
    r->status=r->angle_valid?BALANCE_OK:BALANCE_NO_CORRECTION;
    if (!r->angle_valid) { r->mass_g=0; r->angle_deg=0; r->correction_g_mm=(balance_complex_t){0,0}; }
    return r->status;
}
balance_status_t balance_solve(const balance_config_t *c,const balance_measurement_t *a,
    const balance_measurement_t *b,balance_result_t *r){return solve(c,a,b,r,true);}
balance_status_t balance_estimate(const balance_config_t *c,const balance_measurement_t *a,
    const balance_measurement_t *b,balance_result_t *r){return solve(c,a,b,r,false);}
