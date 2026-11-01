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
    v.direction_rad = .3f;
    v.distance_px = 50;
    v.speed_cap_mps = .2f;
    v.confidence = 100;
    v.valid = true;
    assert(control_path_step(&state, &v, &p, &motion) && motion.omega_raw < 0);
    assert(!control_path_step(&state, &v, &p, &motion)); /* repeated frame */
    v.frame_id++;
    v.session_id = 8;
    assert(!control_path_step(&state, &v, &p, &motion));
    v.session_id = 7;
    v.direction_rad = NAN;
    assert(!control_path_step(&state, &v, &p, &motion));
    v.valid = false;
    v.stop_requested = true;
    assert(!control_path_step(&state, &v, &p, &motion) && motion.stop_requested);
    puts("PASS: standalone control, synthetic VisionOutput, no vision or board objects linked");
    return 0;
}
