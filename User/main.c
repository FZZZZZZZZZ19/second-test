/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "zf_common_headfile.h"
int main(void) {
    clock_init(SYSTEM_CLOCK_144M);
    motor_init_zero();
    debug_init();
    (void)app_boot_init();
    for (;;)
        app_main_poll();
}
