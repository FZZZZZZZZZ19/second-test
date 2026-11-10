#include "control_api.h"
#include "control_tuning.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    AppParam p = {0};
    control_param_defaults(&p);
    PathControlState state;
    MotionRequest motion;
    path_control_reset(&state, 7);
    VisionOutput v = {0};
    v.frame_id = 1;
    v.capture_ms = 100;
    v.session_id = 7;
    p.lateral_kp=2; p.lateral_kd=.1f; p.lateral_tau_s=0;
    v.lateral_m=.03f; v.forward_m=.2f; v.curvature=2;
    v.speed_cap_mps = .2f;
    v.confidence = 100;
    v.valid = true;
    assert(control_path_step(&state, &v, &p, &motion) && motion.omega_raw < 0);
    assert(fabsf(motion.omega_raw-(-.46f))<1e-5f);
    assert(!control_path_step(&state, &v, &p, &motion)); /* repeated frame */
    v.frame_id++;
    v.session_id = 8;
    assert(!control_path_step(&state, &v, &p, &motion));
    v.session_id = 7;
    v.lateral_m = NAN;
    assert(!control_path_step(&state, &v, &p, &motion));
    v.valid = false;
    v.stop_requested = true;
    assert(!control_path_step(&state, &v, &p, &motion) && motion.stop_requested);
    v.valid=true; v.stop_requested=false; v.lateral_m=-.03f; v.curvature=-2;
    v.frame_id++; v.capture_ms+=20; v.reset_derivative=true;
    assert(control_path_step(&state,&v,&p,&motion));
    assert(fabsf(motion.omega_raw-.46f)<1e-5f);
    v.frame_id++; v.capture_ms+=20; v.reset_derivative=false;
    v.lateral_m=-.01f; v.curvature=0;
    assert(control_path_step(&state,&v,&p,&motion));
    assert(fabsf(motion.omega_raw-(-.08f))<1e-5f);
    v.frame_id++; v.capture_ms+=300;
    assert(control_path_step(&state,&v,&p,&motion));
    assert(fabsf(motion.omega_raw-.02f)<1e-5f);
    v.frame_id++; v.forward_m=0;
    assert(!control_path_step(&state,&v,&p,&motion));
    v.forward_m=.2f; v.curvature=INFINITY;
    assert(!control_path_step(&state,&v,&p,&motion));
    v.curvature=2; v.lateral_m=0; v.speed_cap_mps=0;
    assert(control_path_step(&state,&v,&p,&motion) && motion.omega_raw==0);
    puts("PASS: standalone control, synthetic VisionOutput, no vision or board objects linked");
    return 0;
}
