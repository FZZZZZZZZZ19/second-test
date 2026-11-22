#ifndef ZF_DEVICE_ICM20602_CHECKED_H
#define ZF_DEVICE_ICM20602_CHECKED_H
#include <stdbool.h>
#include <stdint.h>
/* ACK-aware companion to zf_device_icm20602. Exclusive owner of B3/B5. */
bool icm20602_checked_init(uint32_t (*micros)(void));
/* -1 error, 0 no fresh sample, 1 fresh. Full=true for stationary calibration. */
int icm20602_checked_sample(bool full, float *z, float *norm, float *accel);
#endif
