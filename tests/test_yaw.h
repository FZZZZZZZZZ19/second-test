#include "yaw_control.h"
static void test_yaw(void) {
    AppParam p = {0};
    control_param_defaults(&p);
    YawState s = {0};
    float out;
    assert(gyro_measure(.3f, .1f, 1, &out) && fabsf(out - .2f) < 1e-6f);
    assert(gyro_measure(.3f, .1f, -1, &out) && fabsf(out + .2f) < 1e-6f);
    assert(!gyro_measure(NAN, 0, 1, &out));
    assert(!gyro_measure(0, 0, 0, &out));
    p.yaw_kp = 2;
    p.yaw_ki = 1;
    p.yaw_i_limit = .1f;
    p.yaw_output_limit = 5;
    assert(yaw_control_step(&s, &p, 1, .5f, .005f, false, &out));
    assert(fabsf(out - 2.0025f) < 1e-5f);
    for (unsigned i = 0; i < 1000; i++)
        assert(yaw_control_step(&s, &p, 1, .5f, .005f, false, &out));
    assert(fabsf(s.integral - .1f) < 1e-5f);
    s.integral = 0;
    assert(yaw_control_step(&s, &p, 10, 0, .005f, false, &out) && out == 5 && s.integral == 0);
    assert(yaw_control_step(&s, &p, -10, 0, .005f, false, &out) && out == -5 && s.integral == 0);
    s.integral = .1f;
    assert(yaw_control_step(&s, &p, 10, 0, .005f, true, &out) && out == 10 && s.integral == 0);
    assert(!yaw_control_step(&s, &p, NAN, 0, .005f, false, &out) && out == 0);
    assert(!yaw_control_step(&s, &p, 0, NAN, .005f, false, &out));
    assert(!yaw_control_step(&s, &p, 0, 0, NAN, false, &out));
    p.yaw_kp = NAN;
    assert(!yaw_control_step(&s, &p, 0, 0, .005f, false, &out));
    control_param_defaults(&p);
    GyroCalibration c = {0};
    for (unsigned i = 0; i < 799; i++)
        assert(!gyro_calibrate_step(&c, &p, .02f, .02f, 1, true, i * 5));
    assert(!c.ready);
    assert(gyro_calibrate_step(&c, &p, .02f, .02f, 1, true, 3995));
    assert(fabsf(c.bias - .02f) < 1e-5f);
    c = (GyroCalibration){0};
    (void)gyro_calibrate_step(&c, &p, 0, 0, 1, true, 0);
    (void)gyro_calibrate_step(&c, &p, 0, 0, 1, true, 0);
    assert(c.count == 1); /* no duplicate sample accumulation */
    (void)gyro_calibrate_step(&c, &p, 1, 1, 1, true, 5);
    assert(c.count == 0);
    (void)gyro_calibrate_step(&c, &p, 0, 0, 2, true, 10);
    assert(c.count == 0);
    (void)gyro_calibrate_step(&c, &p, 0, 0, 1, false, 15);
    assert(c.count == 0);
    (void)gyro_calibrate_step(&c, &p, NAN, 0, 1, true, 20);
    assert(c.failed && !c.ready);
    c = (GyroCalibration){0};
    for (unsigned i = 0; i < 800; i++)
        (void)gyro_calibrate_step(&c, &p, i % 2 ? .04f : -.04f, .04f, 1, true, i * 5);
    assert(!c.ready && c.count == 0); /* moving/noisy calibration rejected */
}
