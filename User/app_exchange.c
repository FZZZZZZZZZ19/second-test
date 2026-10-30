/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
static MotionRequest motion;
static ControlSnapshot snapshot;
static uint32_t requests;
void app_exchange_publish_motion(const MotionRequest *r) {
    BoardIrqToken t = board_irq_save();
    motion = *r;
    board_memory_barrier();
    board_irq_restore(t);
}
void app_exchange_read_motion(MotionRequest *r) {
    BoardIrqToken t = board_irq_save();
    *r = motion;
    board_irq_restore(t);
}
void app_exchange_publish_control(const ControlSnapshot *s) {
    BoardIrqToken t = board_irq_save();
    snapshot = *s;
    board_memory_barrier();
    board_irq_restore(t);
}
void app_exchange_read_control(ControlSnapshot *s) {
    BoardIrqToken t = board_irq_save();
    *s = snapshot;
    board_irq_restore(t);
}
void app_exchange_request(uint32_t r) {
    BoardIrqToken t = board_irq_save();
    requests |= r;
    board_irq_restore(t);
}
uint32_t app_exchange_take_requests(void) {
    BoardIrqToken t = board_irq_save();
    uint32_t r = requests;
    requests = 0;
    board_irq_restore(t);
    return r;
}
