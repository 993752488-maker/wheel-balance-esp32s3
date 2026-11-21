#include "balance_workflow.h"
#include "live_analysis.h"
#include "acquisition.h"
#include <math.h>
#include <string.h>

static balance_workflow_status_t s;
static void clear_result(void) { memset(&s.result,0,sizeof(s.result)); }
void balance_workflow_init(void)
{
    memset(&s,0,sizeof(s)); s.config=balance_default_config();
    s.direction=1; s.generation=s.revision=1; s.state="WAIT_ORIGINAL";
}
static void clear_captures(void)
{
    s.original_captured=s.trial_captured=s.simulated=false;
    memset(&s.original,0,sizeof(s.original)); memset(&s.trial,0,sizeof(s.trial));
    clear_result(); s.state="WAIT_ORIGINAL";
}
bool balance_workflow_configure(double mass,double radius,double angle,
    double correction_radius,unsigned axis,int direction)
{
    if (!isfinite(mass) || mass<=0 || mass>1000 || !isfinite(radius) || radius<=0 || radius>2000 ||
        !isfinite(angle) || fabs(angle)>360000 || !isfinite(correction_radius) ||
        correction_radius<=0 || correction_radius>2000 || axis>2 ||
        (direction!=1 && direction!=-1) || s.generation==UINT32_MAX || s.revision==UINT32_MAX) return false;
    s.config.trial_mass_g=mass; s.config.trial_radius_mm=radius;
    s.config.trial_angle_deg=fmod(fmod(angle,360)+360,360);
    s.config.correction_radius_mm=correction_radius;
    s.axis=axis; s.direction=direction; ++s.generation; ++s.revision; clear_captures(); return true;
}
static bool current_measurement(balance_measurement_t *m)
{
    acquisition_status_t imu; acquisition_get_status(&imu);
    live_analysis_status_t live; live_analysis_get_status(&live);
    const sync_axis_t *a=&live.result.axis[s.axis];
    if (live.state!=LIVE_ANALYSIS_READY_ESTIMATED || !live.result.valid || !a->phase_valid ||
        !imu.epoch || live.window_epoch!=imu.epoch || !live.result_sequence ||
        !live.window_ref_start_sequence || live.result.turns<4 ||
        live.window_ref_end_sequence<=live.window_ref_start_sequence ||
        live.window_ref_end_sequence-live.window_ref_start_sequence!=live.result.turns ||
        !live.window_start_us || live.window_end_us<=live.window_start_us ||
        !isfinite(a->real) || !isfinite(a->imag) ||
        !isfinite(a->residual_rms) || a->residual_rms<0 ||
        !isfinite(live.result.rpm_mean) || live.result.rpm_mean<=0 ||
        !isfinite(live.result.rpm_cv) || !isfinite(a->coherence)) return false;
    *m=(balance_measurement_t){.vibration={a->real,a->imag},
        .valid=true,.phase_valid=a->phase_valid,
        .qualified=live.measurement_qualified && a->measurement_qualified,
        .simulated=false,.noise_rms=a->residual_rms,.rpm=live.result.rpm_mean,
        .rpm_cv=live.result.rpm_cv,.coherence=a->coherence,
        .setup_id=s.generation,.epoch=live.window_epoch,.axis=s.axis,.direction=s.direction,
        .result_sequence=live.result_sequence,
        .window_ref_start_sequence=live.window_ref_start_sequence,
        .window_ref_end_sequence=live.window_ref_end_sequence,
        .window_start_us=live.window_start_us,.window_end_us=live.window_end_us};
    /* This release never grants calibration by changing a request parameter.
     * Future verified timing needs a real calibration provenance and epoch. */
    m->qualified=false;
    return true;
}
static balance_complex_t multiply(balance_complex_t a,balance_complex_t b)
{ return (balance_complex_t){a.real*b.real-a.imag*b.imag,a.real*b.imag+a.imag*b.real}; }
static void synthetic_measurements(void)
{
    /* Forward model: H=(.002,-.001), original U=400 at 300 degrees.
     * Ground-truth correction is 400 g*mm at 120 degrees. Purely synthetic. */
    const double pi=3.14159265358979323846;
    const balance_complex_t h={0.002,-0.001},u={200,-346.41016151377546};
    const double t=s.config.trial_angle_deg*pi/180,ut=s.config.trial_mass_g*s.config.trial_radius_mm;
    const balance_complex_t delta=multiply(h,(balance_complex_t){ut*cos(t),ut*sin(t)});
    s.original=(balance_measurement_t){.vibration=multiply(h,u),.valid=true,.phase_valid=true,
        .simulated=true,.qualified=false,.noise_rms=0.005,.rpm=300,.rpm_cv=0.001,
        .coherence=0.99,.setup_id=s.generation,.epoch=1,.axis=s.axis,.direction=s.direction};
    s.trial=s.original; s.trial.vibration.real+=delta.real; s.trial.vibration.imag+=delta.imag;
    s.original_captured=s.trial_captured=s.simulated=true;
}
bool balance_workflow_command(const char *cmd)
{
    if (!cmd || s.revision==UINT32_MAX) return false;
    if (strcmp(cmd,"reset") && strcmp(cmd,"demo") && strcmp(cmd,"capture_original") &&
        strcmp(cmd,"capture_trial") && strcmp(cmd,"solve")) return false;
    ++s.revision;
    if (!strcmp(cmd,"reset")) { clear_captures(); return true; }
    if (!strcmp(cmd,"demo")) {
        clear_result(); synthetic_measurements();
        balance_solve(&s.config,&s.original,&s.trial,&s.result);
        s.state=balance_status_name(s.result.status); return true;
    }
    if (!strcmp(cmd,"capture_original") || !strcmp(cmd,"capture_trial")) {
        clear_result();
        const bool original=!strcmp(cmd,"capture_original");
        /* A failed remeasurement must not leave an old pair ready to solve. */
        if (original) clear_captures();
        else { s.trial_captured=false; memset(&s.trial,0,sizeof(s.trial)); }
        if (!original && (!s.original_captured || s.simulated)) { s.state="CAPTURE_ORIGINAL_FIRST"; return false; }
        balance_measurement_t m;
        if (!current_measurement(&m)) { s.state="WAIT_VALID_1X"; return false; }
        if (!original && m.epoch!=s.original.epoch) { s.state="SETUP_CHANGED"; return false; }
        /* Windows are half-open. Sharing their boundary reference is fine;
         * sharing any turn or reusing an old result is not. This does not
         * prove that a trial weight was actually installed. */
        if (!original && (m.result_sequence<=s.original.result_sequence ||
            m.window_ref_start_sequence<s.original.window_ref_end_sequence ||
            m.window_start_us<s.original.window_end_us)) {
            s.state="WINDOW_OVERLAP"; return false;
        }
        if (original) { s.original=m; s.original_captured=true; s.state="WAIT_TRIAL"; }
        else { s.trial=m; s.trial_captured=true; s.state="READY_TO_SOLVE"; }
        return true;
    }
    if (!strcmp(cmd,"solve")) {
        clear_result();
        if (!s.original_captured || !s.trial_captured) { s.state="NEED_BOTH_MEASUREMENTS"; return false; }
        balance_solve(&s.config,&s.original,&s.trial,&s.result);
        s.state=balance_status_name(s.result.status); return true;
    }
    return false;
}
void balance_workflow_get_status(balance_workflow_status_t *out)
{
    if (!out) return;
    *out=s; balance_measurement_t m;
    out->can_capture=current_measurement(&m);
    out->live_qualified=out->can_capture && m.qualified;
}
