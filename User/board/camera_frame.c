/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "zf_common_headfile.h"
#include <string.h>
enum { FRAME_CAPTURE, FRAME_READY, FRAME_PROCESSING, FRAME_FAILED };
static volatile uint8_t state;
static volatile bool enabled;
static bool camera_initialized;
static volatile uint8_t init_status;
static volatile uint32_t frame_id, capture_ms;
/* Native DVP runs continuously. Separate application-owned image uses the
 * same 64 KiB SRAM configuration as the working example, without DMA resets. */
static uint8_t stable_image[APP_HEIGHT][APP_WIDTH] __attribute__((aligned(4)));
bool camera_frame_init(void) {
    camera_initialized = false; enabled = false;
    state = FRAME_CAPTURE; frame_id = 0; mt9v03x_finish_flag = 0;
    init_status = mt9v03x_init();
    if (init_status) { state = FRAME_FAILED; return false; }
    camera_initialized = true; enabled = true;
    return true;
}
uint8_t camera_frame_init_status(void) { return init_status; }
void camera_frame_on_done_isr(void) {
    if (!enabled || state == FRAME_PROCESSING || state == FRAME_FAILED) return;
    _Static_assert(APP_WIDTH == MT9V03X_W && APP_HEIGHT == MT9V03X_H,
                   "vision and calibration require the full native frame");
    memcpy(stable_image, mt9v03x_image, sizeof stable_image);
    capture_ms = board_time_ms(); frame_id++;
    board_memory_barrier(); state = FRAME_READY;
}
bool camera_frame_acquire(FrameView *f) {
    BoardIrqToken t = board_irq_save();
    if (!enabled || state != FRAME_READY || !mt9v03x_finish_flag) {
        board_irq_restore(t); return false;
    }
    mt9v03x_finish_flag = 0; state = FRAME_PROCESSING;
    f->pixels = &stable_image[0][0];
    f->width = APP_WIDTH; f->height = APP_HEIGHT; f->stride = APP_WIDTH;
    f->frame_id = frame_id; f->capture_ms = capture_ms;
    board_irq_restore(t); return true;
}
void camera_frame_release(uint32_t id) {
    BoardIrqToken t = board_irq_save();
    if (enabled && state == FRAME_PROCESSING && frame_id == id) {
        mt9v03x_finish_flag = 0; state = FRAME_CAPTURE;
    }
    board_irq_restore(t);
}
bool camera_frame_set_running(bool running) {
    if (running && !camera_initialized) return camera_frame_init();
    BoardIrqToken t = board_irq_save();
    enabled = false; mt9v03x_finish_flag = 0; state = FRAME_CAPTURE;
    bool ok = true;
    if (running) {
        ok = camera_initialized && board_camera_start(); enabled = ok;
        if (!ok) state = FRAME_FAILED;
    } else ok = board_camera_stop();
    board_irq_restore(t); return ok;
}
