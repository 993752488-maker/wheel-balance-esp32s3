#include "balance_core.h"
#include <math.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
static unsigned passed,failed;
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);++failed;return;} }while(0)
static balance_config_t cfg;
static balance_measurement_t a,b;
static balance_result_t out;
static const double pi=3.14159265358979323846;
static balance_complex_t mul(balance_complex_t x,balance_complex_t y){return (balance_complex_t){x.real*y.real-x.imag*y.imag,x.real*y.imag+x.imag*y.real};}
static void setup(void){cfg=balance_default_config();a=(balance_measurement_t){.vibration={0.05358983848622454,-0.8928203230275509},.valid=true,.phase_valid=true,.simulated=true,.noise_rms=.005,.rpm=300,.rpm_cv=.001,.coherence=.99,.setup_id=1,.epoch=1,.axis=0,.direction=1};b=a;b.vibration.real+=1;b.vibration.imag-=.5;memset(&out,0,sizeof(out));}
static balance_status_t solve(void){return balance_solve(&cfg,&a,&b,&out);}
static void cleared(void){CHECK(!out.calculation_valid&&!out.recommendation_valid&&!out.angle_valid);CHECK(out.mass_g==0&&out.angle_deg==0&&out.correction_g_mm.real==0&&out.correction_g_mm.imag==0);}
static void forward(void){CHECK(solve()==BALANCE_OK);CHECK(fabs(out.mass_g-4)<1e-10);CHECK(fabs(out.angle_deg-120)<1e-10);CHECK(fabs(out.influence.real-.002)<1e-12&&fabs(out.influence.imag+.001)<1e-12);CHECK(out.simulated&&!out.recommendation_valid);printf("SIM,known_model,4,120,%.12g,%.12g\n",out.mass_g,out.angle_deg);balance_complex_t change=mul(out.influence,out.correction_g_mm);CHECK(hypot(change.real+a.vibration.real,change.imag+a.vibration.imag)<1e-12);}
static void arbitrary_trial(void){cfg.trial_angle_deg=217;cfg.trial_mass_g=3;cfg.trial_radius_mm=130;double t=217*pi/180;balance_complex_t d=mul((balance_complex_t){.002,-.001},(balance_complex_t){390*cos(t),390*sin(t)});b.vibration.real=a.vibration.real+d.real;b.vibration.imag=a.vibration.imag+d.imag;CHECK(solve()==BALANCE_OK);CHECK(fabs(out.mass_g-4)<1e-10&&fabs(out.angle_deg-120)<1e-10);}
static void radius(void){cfg.correction_radius_mm=200;CHECK(solve()==BALANCE_OK);CHECK(fabs(out.mass_g-2)<1e-10);}
static void direction(void){a.direction=b.direction=-1;CHECK(solve()==BALANCE_OK);CHECK(fabs(out.angle_deg-120)<1e-10);}
static void validated(void){a.simulated=b.simulated=false;a.qualified=b.qualified=true;CHECK(solve()==BALANCE_OK);CHECK(out.recommendation_valid&&!out.simulated);}
static void estimated(void){a.simulated=b.simulated=false;CHECK(solve()==BALANCE_ERR_UNQUALIFIED);cleared();}
static void half_qualified(void){a.simulated=b.simulated=false;a.qualified=true;CHECK(solve()==BALANCE_ERR_UNQUALIFIED);cleared();}
static void mixed(void){a.simulated=false;CHECK(solve()==BALANCE_ERR_SOURCE);cleared();}
static void context(void){b.setup_id=2;CHECK(solve()==BALANCE_ERR_CONTEXT);cleared();}
static void epoch(void){b.epoch=2;CHECK(solve()==BALANCE_ERR_CONTEXT);cleared();}
static void axis(void){b.axis=1;CHECK(solve()==BALANCE_ERR_CONTEXT);cleared();}
static void reverse(void){b.direction=-1;CHECK(solve()==BALANCE_ERR_CONTEXT);cleared();}
static void unstable(void){a.rpm_cv=.02;CHECK(solve()==BALANCE_ERR_UNSTABLE);cleared();}
static void mismatch(void){b.rpm=305;CHECK(solve()==BALANCE_ERR_SPEED);cleared();}
static void incoherent(void){a.coherence=.2;CHECK(solve()==BALANCE_ERR_PHASE);cleared();}
static void bad_phase(void){b.phase_valid=false;CHECK(solve()==BALANCE_ERR_PHASE);cleared();}
static void weak(void){b.vibration=a.vibration;CHECK(solve()==BALANCE_ERR_WEAK_RESPONSE);cleared();}
static void noisy(void){a.noise_rms=b.noise_rms=1;CHECK(solve()==BALANCE_ERR_WEAK_RESPONSE);cleared();}
static void max_mass(void){cfg.max_mass_g=3;CHECK(solve()==BALANCE_ERR_LIMIT);cleared();}
static void zero_mass(void){cfg.trial_mass_g=0;CHECK(solve()==BALANCE_ERR_ARGUMENT);cleared();}
static void zero_radius(void){cfg.correction_radius_mm=0;CHECK(solve()==BALANCE_ERR_ARGUMENT);cleared();}
static void reject_nan(void){a.vibration.real=NAN;CHECK(solve()==BALANCE_ERR_ARGUMENT);cleared();}
static void reject_infinity(void){cfg.trial_angle_deg=INFINITY;CHECK(solve()==BALANCE_ERR_ARGUMENT);cleared();}
static void overflow(void){cfg.trial_mass_g=DBL_MAX;cfg.trial_radius_mm=DBL_MAX;CHECK(solve()==BALANCE_ERR_NUMERIC);cleared();}
static void underflow_mass(void){cfg.correction_radius_mm=DBL_MAX;a.noise_rms=b.noise_rms=1e306;b.vibration=(balance_complex_t){1e308,0};CHECK(solve()==BALANCE_ERR_NUMERIC);cleared();}
static void nulls(void){CHECK(balance_solve(NULL,&a,&b,&out)==BALANCE_ERR_ARGUMENT);cleared();CHECK(balance_solve(&cfg,NULL,&b,&out)==BALANCE_ERR_ARGUMENT);CHECK(balance_solve(&cfg,&a,&b,NULL)==BALANCE_ERR_ARGUMENT);}
static void no_correction(void){a.vibration=(balance_complex_t){0,0};a.phase_valid=false;a.coherence=0;b.vibration=(balance_complex_t){1,-.5};CHECK(solve()==BALANCE_NO_CORRECTION);CHECK(out.calculation_valid&&!out.angle_valid&&out.mass_g==0);}
static void trial_cancels_original(void)
{
    cfg.trial_angle_deg=63;
    b.vibration=(balance_complex_t){0,0};
    b.phase_valid=false;
    b.coherence=0;
    CHECK(solve()==BALANCE_OK);
    CHECK(out.angle_valid&&fabs(out.mass_g-cfg.trial_mass_g*cfg.trial_radius_mm/cfg.correction_radius_mm)<1e-9);
    CHECK(fabs(out.angle_deg-63)<1e-9);
}
static void stale_result(void){CHECK(solve()==BALANCE_OK);b.valid=false;CHECK(solve()==BALANCE_ERR_ARGUMENT);cleared();}
static void shifted_trial(void){cfg.trial_angle_deg=-360;CHECK(solve()==BALANCE_OK);CHECK(fabs(out.angle_deg-120)<1e-10);}
static void polarity(void){a.vibration.real=-a.vibration.real;a.vibration.imag=-a.vibration.imag;b.vibration.real=-b.vibration.real;b.vibration.imag=-b.vibration.imag;CHECK(solve()==BALANCE_OK);CHECK(fabs(out.angle_deg-120)<1e-10);}
static void finite_noise(void){a.noise_rms=b.noise_rms=0;CHECK(solve()==BALANCE_OK);CHECK(isfinite(out.response_snr));}
static void retained_trial(void){CHECK(solve()==BALANCE_OK);balance_complex_t d=mul(out.influence,(balance_complex_t){cfg.trial_mass_g*cfg.trial_radius_mm,0});CHECK(hypot(d.real,d.imag)>.1);/* Retaining trial after applying Uc leaves this residual. */}
static void known_5g_10g(void)
{
    const balance_complex_t h={.002,-.001};
    const balance_complex_t original=a.vibration;
    balance_complex_t delta5={0,0};
    cfg.trial_angle_deg=37;
    for(unsigned i=0;i<2;++i) {
        cfg.trial_mass_g=i?10:5;
        const double theta=cfg.trial_angle_deg*pi/180;
        const double moment=cfg.trial_mass_g*cfg.trial_radius_mm;
        const balance_complex_t delta=mul(h,(balance_complex_t){moment*cos(theta),moment*sin(theta)});
        b.vibration=(balance_complex_t){original.real+delta.real,original.imag+delta.imag};
        CHECK(solve()==BALANCE_OK);
        CHECK(fabs(out.mass_g-4)<1e-10 && fabs(out.angle_deg-120)<1e-10);
        CHECK(fabs(out.influence.real-h.real)<1e-12 && fabs(out.influence.imag-h.imag)<1e-12);
        CHECK(out.simulated && !out.recommendation_valid);
        if(!i)delta5=delta;
        else CHECK(hypot(delta.real-2*delta5.real,delta.imag-2*delta5.imag)<1e-12);
        printf("SIM,trial_%ug,4,120,%.12g,%.12g\n",i?10:5,out.mass_g,out.angle_deg);
    }
}
static void common_complex_gain(void)
{
    /* A stable common gain/delay cancels in the ratio. This is not a model
     * of clock jitter, speed-dependent delay or a changing mount. */
    const double theta=-73*pi/180;
    const balance_complex_t gain={.73*cos(theta),.73*sin(theta)};
    const balance_complex_t expected=mul(gain,(balance_complex_t){.002,-.001});
    a.vibration=mul(gain,a.vibration);b.vibration=mul(gain,b.vibration);
    a.noise_rms*=.73;b.noise_rms*=.73;
    CHECK(solve()==BALANCE_OK);
    CHECK(fabs(out.mass_g-4)<1e-10 && fabs(out.angle_deg-120)<1e-10);
    CHECK(fabs(out.influence.real-expected.real)<1e-12 && fabs(out.influence.imag-expected.imag)<1e-12);
    CHECK(out.simulated && !out.recommendation_valid);
}
static void different_wheels_and_mounts(void)
{
    /* Independent forward models: radii and complex mechanical transfer vary.
     * No bicycle/motorcycle coefficient or inference of two-plane balance. */
    const double radii[]={80,170,300,450};
    const balance_complex_t h[]={{.002,-.001},{-.0004,.003},{.01,.02},{-.004,-.002}};
    for(unsigned j=0;j<4;++j)for(unsigned k=0;k<4;++k) {
        cfg.trial_radius_mm=radii[j];cfg.correction_radius_mm=radii[k];
        cfg.trial_mass_g=5;cfg.trial_angle_deg=73;
        const double phase=241*pi/180,trial=73*pi/180;
        const balance_complex_t desired={1200*cos(phase),1200*sin(phase)};
        a.vibration=mul(h[j],(balance_complex_t){-desired.real,-desired.imag});
        const balance_complex_t delta=mul(h[j],(balance_complex_t){5*radii[j]*cos(trial),5*radii[j]*sin(trial)});
        b=a;b.vibration.real+=delta.real;b.vibration.imag+=delta.imag;
        a.noise_rms=b.noise_rms=1e-5;
        CHECK(solve()==BALANCE_OK);
        CHECK(fabs(out.mass_g-1200/radii[k])<1e-8 && fabs(out.angle_deg-241)<1e-8);
        CHECK(fabs(out.influence.real-h[j].real)<1e-10 && fabs(out.influence.imag-h[j].imag)<1e-10);
        CHECK(!out.recommendation_valid && out.simulated);
    }
}
static void published_bk_single_plane_example(void)
{
    /* External numerical reference, NOT a measurement by this project.
     * B&K BO0276, Appendix 1 Example One, printed p15:
     * V0=3.4 mm/s at 116 deg; V1=1.8 mm/s at 42 deg; trial=2 g.
     * The original uses a vector diagram and prints ~2.03 g, -31 deg.
     * Direct complex arithmetic on the printed inputs yields 2.011675957 g,
     * 329.211248769 deg. The graphical/printed example is not a precision
     * calibration target; compare it within 0.03 g and 1 deg only.
     * Equal arbitrary 100mm radii cancel; the source does not supply a radius.
     * https://www.bksv.com/doc/BO0276.pdf
     */
    cfg.trial_mass_g=2;
    cfg.trial_radius_mm=cfg.correction_radius_mm=100;
    cfg.trial_angle_deg=0;
    a.vibration=(balance_complex_t){3.4*cos(116*pi/180),3.4*sin(116*pi/180)};
    b.vibration=(balance_complex_t){1.8*cos(42*pi/180),1.8*sin(42*pi/180)};
    a.rpm=b.rpm=1490;
    CHECK(solve()==BALANCE_OK);
    CHECK(fabs(out.mass_g-2.011675957184712)<1e-10);
    CHECK(fabs(out.angle_deg-329.21124876914655)<1e-9);
    CHECK(fabs(out.mass_g-2.03)<.03 && fabs(out.angle_deg-329)<1);
    CHECK(out.simulated && !out.recommendation_valid);
    printf("REF,BK_BO0276_example_one,%.12g,%.12g,source_published_not_our_measurement\n",
        out.mass_g,out.angle_deg);
}
static void run(void (*f)(void),const char *name){unsigned before=failed;setup();f();if(before==failed){++passed;printf("PASS %s\n",name);}}
int main(void){
#define RUN(f) run(f,#f)
RUN(forward);RUN(arbitrary_trial);RUN(radius);RUN(direction);RUN(validated);RUN(estimated);RUN(half_qualified);RUN(mixed);RUN(context);RUN(epoch);RUN(axis);RUN(reverse);RUN(unstable);RUN(mismatch);RUN(incoherent);RUN(bad_phase);RUN(weak);RUN(noisy);RUN(max_mass);RUN(zero_mass);RUN(zero_radius);RUN(reject_nan);RUN(reject_infinity);RUN(overflow);RUN(underflow_mass);RUN(nulls);RUN(no_correction);RUN(stale_result);RUN(shifted_trial);RUN(polarity);RUN(finite_noise);RUN(retained_trial);RUN(known_5g_10g);RUN(common_complex_gain);
RUN(different_wheels_and_mounts);RUN(trial_cancels_original);
RUN(published_bk_single_plane_example);
printf("SUMMARY %u passed, %u failed; synthetic/published inputs only, no physical measurements.\n",passed,failed);return failed?1:0;}
