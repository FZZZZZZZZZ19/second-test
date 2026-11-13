/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "maintenance.h"
#include <math.h>
MaintenanceCheck maintenance_stop_check(const ControlSnapshot *s) {
    if (!s || s->mode!=RUN_STATIC_DEBUG) return MAINT_MODE;
    if (s->duty[0]!=0 || s->duty[1]!=0) return MAINT_OUTPUT;
    /* Uncalibrated raw counts are not a speed measurement. Saving cannot arm or drive motors. */
    if (s->feedback.valid && (s->feedback.delta[0]!=0 || s->feedback.delta[1]!=0 ||
        !isfinite(s->feedback.speed_mps[0]) || !isfinite(s->feedback.speed_mps[1]) ||
        fabsf(s->feedback.speed_mps[0])>=.01f || fabsf(s->feedback.speed_mps[1])>=.01f)) return MAINT_MOTION;
    return MAINT_OK;
}
MaintenanceCheck maintenance_save_check(const ControlSnapshot *s,const AppParam *p) {
    MaintenanceCheck c=maintenance_stop_check(s);
    if(c!=MAINT_OK) return c;
    return param_storable(p)?MAINT_OK:MAINT_PARAM;
}
