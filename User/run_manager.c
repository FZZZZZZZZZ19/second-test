/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
bool run_manager_step(RunMode *m, StopReason *reason, uint32_t r, bool armable, bool fresh,
                      uint32_t *session) {
    bool reset = false;
    if (r & REQ_STOP) {
        if (*m == RUN_RUNNING)
            *m = RUN_STOPPING;
        else if (*m == RUN_READY) {
            *m = RUN_STATIC_DEBUG;
            (*session)++;
        }
        if (*m != RUN_FAULT)
            *reason = STOP_LOCAL;
        return false;
    }
    if ((r & REQ_RESET) && *m == RUN_FAULT) {
        *m = RUN_STATIC_DEBUG;
        *reason = STOP_NONE;
        (*session)++;
        return true;
    }
    if ((r & REQ_READY) && *m == RUN_STATIC_DEBUG && armable) {
        *m = RUN_READY;
        *reason = STOP_NONE;
        (*session)++;
        reset = true;
    } else if ((r & REQ_START) && *m == RUN_READY && armable && fresh) {
        *m = RUN_RUNNING;
        *reason = STOP_NONE;
        reset = true;
    }
    return reset;
}
