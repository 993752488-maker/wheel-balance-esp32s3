#include "sync_analysis.h"
#include <math.h>
#include <string.h>

#define PI 3.1415926535897932384626433832795
#define BASIS 7

sync_config_t sync_default_config(void)
{
    return (sync_config_t){.min_turns=4, .min_samples_per_turn=16, .max_harmonic=3,
        .max_sample_gap_us=4000, .max_angular_gap_fraction=0.2,
        .max_rpm_cv=0.01, .max_rpm_deviation=0.02,
        .min_phase_amplitude=1e-5, .min_coherence=0.8};
}

static sync_status_t fail(sync_result_t *r, sync_status_t status)
{
    r->valid = false;
    r->status = status;
    /* An incomplete fit cannot leave apparently usable phase values. */
    memset(r->axis, 0, sizeof(r->axis));
    return status;
}

static bool config_ok(const sync_config_t *c)
{
    return c && c->min_turns >= 1 && c->min_turns <= SYNC_MAX_TURNS &&
        c->max_harmonic >= 1 && c->max_harmonic <= 3 &&
        c->min_samples_per_turn >= 2*c->max_harmonic+2 &&
        c->min_samples_per_turn <= SYNC_MAX_SAMPLES && c->max_sample_gap_us > 0 &&
        isfinite(c->max_angular_gap_fraction) && c->max_angular_gap_fraction > 0 && c->max_angular_gap_fraction <= 1 &&
        isfinite(c->max_rpm_cv) && c->max_rpm_cv >= 0 && c->max_rpm_cv <= 1 &&
        isfinite(c->max_rpm_deviation) && c->max_rpm_deviation >= 0 && c->max_rpm_deviation <= 1 &&
        isfinite(c->min_phase_amplitude) && c->min_phase_amplitude > 0 &&
        isfinite(c->min_coherence) && c->min_coherence >= 0 && c->min_coherence <= 1;
}

static void basis(double theta, unsigned harmonics, double *b)
{
    b[0] = 1;
    const double c = cos(theta), s = sin(theta);
    double ch = c, sh = s;
    for (unsigned h=1; h<=harmonics; ++h) {
        b[2*h-1] = ch; b[2*h] = sh;
        const double next = ch*c - sh*s;
        sh = sh*c + ch*s; ch = next;
    }
}

static bool solve(double a[BASIS][BASIS+3], unsigned p, size_t n)
{
    for (unsigned k=0; k<p; ++k) {
        unsigned pivot = k;
        for (unsigned row=k+1; row<p; ++row)
            if (fabs(a[row][k]) > fabs(a[pivot][k])) pivot = row;
        if (!isfinite(a[pivot][k]) || fabs(a[pivot][k]) <= (double)n*1e-10) return false;
        if (pivot != k) for (unsigned j=0; j<p+3; ++j) {
            const double t=a[k][j]; a[k][j]=a[pivot][j]; a[pivot][j]=t;
        }
        const double d=a[k][k];
        for (unsigned j=k; j<p+3; ++j) a[k][j] /= d;
        for (unsigned row=0; row<p; ++row) if (row != k) {
            const double f=a[row][k];
            for (unsigned j=k; j<p+3; ++j) a[row][j] -= f*a[k][j];
        }
    }
    for (unsigned row=0; row<p; ++row)
        for (unsigned axis=0; axis<3; ++axis)
            if (!isfinite(a[row][p+axis])) return false;
    return true;
}

sync_status_t sync_analyze(const sync_config_t *c, const sync_sample_t *s, size_t ns,
    const sync_reference_t *ref, size_t nr, sync_quality_t quality, sync_result_t *r)
{
    if (!r) return SYNC_ERR_ARGUMENT;
    memset(r, 0, sizeof(*r));
    if (!config_ok(c) || !s || !ref || ns == 0 || ns > SYNC_MAX_SAMPLES ||
        nr < 2 || nr > SYNC_MAX_TURNS+1)
        return fail(r, SYNC_ERR_ARGUMENT);
    if (nr-1 < c->min_turns) return fail(r, SYNC_ERR_INSUFFICIENT);
    if (quality.time_quality != SYNC_TIME_SYNTHETIC && quality.time_quality != SYNC_TIME_VALIDATED &&
        quality.time_quality != SYNC_TIME_ESTIMATED)
        return fail(r, SYNC_ERR_TIME_QUALITY);
    r->simulated = quality.time_quality == SYNC_TIME_SYNTHETIC;
    if (quality.faults) return fail(r, SYNC_ERR_INPUT_FAULT);
    r->turns = nr-1; r->samples = ns;
    double rpm_m2=0;
    r->rpm_min=INFINITY;
    for (size_t turn=0; turn<nr-1; ++turn) {
        if (!ref[turn].sequence || ref[turn].sequence == UINT64_MAX ||
            ref[turn+1].sequence != ref[turn].sequence+1 || ref[turn+1].time_us <= ref[turn].time_us)
            return fail(r, SYNC_ERR_REFERENCE);
        const double rpm=60000000.0/(double)(ref[turn+1].time_us-ref[turn].time_us);
        if (rpm < r->rpm_min) r->rpm_min=rpm;
        if (rpm > r->rpm_max) r->rpm_max=rpm;
        const double delta=rpm-r->rpm_mean;
        r->rpm_mean += delta/(double)(turn+1);
        rpm_m2 += delta*(rpm-r->rpm_mean);
    }
    r->rpm_std=sqrt(fmax(0, rpm_m2/(double)r->turns));
    r->rpm_cv=r->rpm_std/r->rpm_mean;
    r->max_rpm_deviation=fmax(r->rpm_max-r->rpm_mean, r->rpm_mean-r->rpm_min)/r->rpm_mean;
    for (size_t i=0; i<ns; ++i) {
        if (!s[i].sequence || (i && (s[i-1].sequence == UINT64_MAX || s[i].sequence != s[i-1].sequence+1)))
            return fail(r, SYNC_ERR_SAMPLE_GAP);
        if (s[i].epoch != s[0].epoch) return fail(r, SYNC_ERR_EPOCH);
        if (s[i].time_us < ref[0].time_us || s[i].time_us >= ref[nr-1].time_us ||
            (i && s[i].time_us <= s[i-1].time_us)) return fail(r, SYNC_ERR_SAMPLE_TIME);
        if (i && s[i].time_us-s[i-1].time_us > c->max_sample_gap_us) return fail(r, SYNC_ERR_SAMPLE_GAP);
        for (unsigned axis=0; axis<3; ++axis)
            if (!isfinite(s[i].acceleration[axis])) return fail(r, SYNC_ERR_NUMERIC);
    }
    if (r->rpm_cv > c->max_rpm_cv || r->max_rpm_deviation > c->max_rpm_deviation)
        return fail(r, SYNC_ERR_UNSTABLE_RPM);
    size_t first=0;
    const unsigned p=1+2*c->max_harmonic;
    for (size_t turn=0; turn<r->turns; ++turn) {
        size_t end=first;
        while (end<ns && s[end].time_us < ref[turn+1].time_us) ++end;
        const size_t n=end-first;
        if (n < c->min_samples_per_turn) return fail(r, SYNC_ERR_INSUFFICIENT);
        const uint64_t period=ref[turn+1].time_us-ref[turn].time_us;
        uint64_t largest_gap=(s[first].time_us-ref[turn].time_us)+(ref[turn+1].time_us-s[end-1].time_us);
        for (size_t i=first+1; i<end; ++i) {
            const uint64_t gap=s[i].time_us-s[i-1].time_us;
            if (gap>largest_gap) largest_gap=gap;
        }
        if ((double)largest_gap/(double)period > c->max_angular_gap_fraction)
            return fail(r, SYNC_ERR_INSUFFICIENT);
        double a[BASIS][BASIS+3]={{0}}, rhs[BASIS][3]={{0}}, yy[3]={0};
        for (size_t i=first; i<end; ++i) {
            double b[BASIS];
            basis(2*PI*(double)(s[i].time_us-ref[turn].time_us)/(double)period, c->max_harmonic, b);
            for (unsigned row=0; row<p; ++row) {
                for (unsigned col=0; col<p; ++col) a[row][col] += b[row]*b[col];
                for (unsigned axis=0; axis<3; ++axis) rhs[row][axis] += b[row]*s[i].acceleration[axis];
            }
            for (unsigned axis=0; axis<3; ++axis) yy[axis] += s[i].acceleration[axis]*s[i].acceleration[axis];
        }
        for (unsigned row=0; row<p; ++row)
            for (unsigned axis=0; axis<3; ++axis) a[row][p+axis]=rhs[row][axis];
        if (!solve(a, p, n)) return fail(r, SYNC_ERR_SINGULAR);
        for (unsigned axis=0; axis<3; ++axis) {
            sync_axis_t *v=&r->axis[axis];
            const double real=a[1][p+axis], imag=-a[2][p+axis];
            double sse=yy[axis];
            for (unsigned row=0; row<p; ++row) sse -= a[row][p+axis]*rhs[row][axis];
            if (!isfinite(sse) || sse < -1e-7*fmax(yy[axis], 1)) return fail(r, SYNC_ERR_NUMERIC);
            const double count=(double)(turn+1);
            v->real += (real-v->real)/count;
            v->imag += (imag-v->imag)/count;
            v->dc += (a[0][p+axis]-v->dc)/count;
            v->mean_turn_peak += (hypot(real, imag)-v->mean_turn_peak)/count;
            v->residual_rms += fmax(0, sse);
        }
        first=end;
    }
    for (unsigned axis=0; axis<3; ++axis) {
        sync_axis_t *v=&r->axis[axis];
        v->peak=hypot(v->real, v->imag); v->rms=v->peak/sqrt(2.0);
        v->coherence=v->mean_turn_peak > 0 ? fmin(1, v->peak/v->mean_turn_peak) : 0;
        v->residual_rms=sqrt(v->residual_rms/(double)ns);
        v->phase_valid=v->peak >= c->min_phase_amplitude && v->coherence >= c->min_coherence;
        if (v->phase_valid) {
            v->phase_deg=atan2(v->imag, v->real)*180/PI;
            if (v->phase_deg < 0) v->phase_deg += 360;
        }
        v->measurement_qualified=v->phase_valid && quality.time_quality == SYNC_TIME_VALIDATED;
    }
    r->status=SYNC_OK; r->valid=true;
    return SYNC_OK;
}
