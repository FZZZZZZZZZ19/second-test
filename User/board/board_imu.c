/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_port.h"
#include "zf_common_headfile.h"
#include "zf_device_icm20602_checked.h"
bool board_imu_init(void) {
    return icm20602_checked_init(board_time_us);
}
int board_imu_read(bool calibration, float *z, float *norm, float *accel) {
    return icm20602_checked_sample(calibration, z, norm, accel);
}
