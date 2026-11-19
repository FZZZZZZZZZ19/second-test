#ifndef VEHICLE_CALIBRATION_H
#define VEHICLE_CALIBRATION_H
/* Measured 2026-11-17: diameter 33.7 mm, track 166.6 mm, wheelbase 35.5 mm.
 * SI units below. Remaining zeros intentionally prevent arming.
 * Saved Flash parameters override these boot defaults; update hardware menu too. */
#define CAL_WHEEL_LEFT_M 0.0337f
#define CAL_WHEEL_RIGHT_M 0.0337f
#define CAL_TRACK_M 0.1666f
#define CAL_AXLE_M 0.0355f
/* MT6701CT-ACD: 1024 PPR, existing TIM TI2 x2 counting, wheel shaft 1:1.
 * Do not use 16384 (absolute angle resolution) or 4096 (TI12 x4).
 * Wiring direction still needs confirmation; CAL_MASK remains unchanged. */
#define CAL_COUNTS_LEFT 2048.0f
#define CAL_COUNTS_RIGHT 2048.0f
#define CAL_ROW_NEAR_M 0.0f
#define CAL_ROW_FAR_M 0.0f
#define CAL_CAMERA_FORWARD_M 0.0f

/* Ground calibration uses native image coordinates: image[v][u],
 * 120 rows by 188 columns, origin at the top left of the full frame. */
#define CAL_GROUND_H0 0.0045366634f
#define CAL_GROUND_H1 -0.000223055189f
#define CAL_GROUND_H2 -0.439942106f
#define CAL_GROUND_H3 -0.000210106974f
#define CAL_GROUND_H4 -0.0030013693f
#define CAL_GROUND_H5 0.561728764f
#define CAL_GROUND_H6 0.000546354536f
#define CAL_GROUND_H7 0.0137539169f
#define CAL_MASK 0u
#define CAL_CAPTURE_VALIDATED 0
#define CAL_FULL_RACE 0
#define CAL_VISION_V2_VALIDATED 0 /* revalidate Otsu, contour timing and steering gains */
#endif
