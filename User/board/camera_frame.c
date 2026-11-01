/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "zf_common_headfile.h"
enum { FRAME_CAPTURE, FRAME_READY, FRAME_PROCESSING, FRAME_FAILED };
static volatile uint8_t state = FRAME_CAPTURE;
static volatile bool enabled, discard;
static volatile uint32_t frame_id, capture_ms;
bool camera_frame_init(void) {
    enabled = false;
    state = FRAME_CAPTURE;
    frame_id = 0;
    discard = true;
    if (mt9v03x_init() != 0) {
        state = FRAME_FAILED;
        return false;
    }
    BoardIrqToken t = board_irq_save();
    enabled = true;
    (void)board_camera_hold_after_frame_isr();
    bool ok = board_camera_restart_full_frame();
    if (!ok)
        state = FRAME_FAILED;
    board_irq_restore(t);
    return ok;
}
void camera_frame_on_done_isr(void) {
    if (!enabled || state != FRAME_CAPTURE)
        return;
    if (discard) {
        discard = false;
        return;
    } /* Discard the first frame after reset/restart. */
    if (!board_camera_hold_after_frame_isr()) {
        state = FRAME_FAILED;
        return;
    }
    capture_ms = board_time_ms();
    frame_id++;
    board_memory_barrier();
    state = FRAME_READY;
}
bool camera_frame_acquire(FrameView *f) {
    BoardIrqToken t = board_irq_save();
    if (state != FRAME_READY) {
        board_irq_restore(t);
        return false;
    }
    state = FRAME_PROCESSING;
    f->pixels = &mt9v03x_image[0][0];
    f->width = APP_WIDTH;
    f->height = APP_HEIGHT;
    f->stride = APP_WIDTH;
    f->frame_id = frame_id;
    f->capture_ms = capture_ms;
    board_irq_restore(t);
    return true;
}
void camera_frame_release(uint32_t id) {
    BoardIrqToken t = board_irq_save();
    if (state == FRAME_PROCESSING && frame_id == id) {
        discard = true;
        state = FRAME_CAPTURE;
        if (!board_camera_restart_full_frame())
            state = FRAME_FAILED;
    }
    board_irq_restore(t);
}
