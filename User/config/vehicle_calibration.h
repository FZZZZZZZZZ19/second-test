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

/* User calibration 2026-11-19 was fitted in the V1.50.13 128x120 PREVIEW,
 * with its own top-left (0,0), excluding the LCD's 16-pixel title.
 * Vision still uses full 188x120 pixels: preview_u = raw_u - 30, preview_v = raw_v.
 * These are H_raw = H_preview * translate(-30,0), normalized to H_raw[2][2]=1.
 * Original supplied coefficients and Flash input values are in
 * docs/2026-11-19相机标定记录.md. Saved Flash remains authoritative. */
#define CAL_GROUND_H0 0.004612260964f
#define CAL_GROUND_H1 -0.0002267721284f
#define CAL_GROUND_H2 -0.5856410265f
#define CAL_GROUND_H3 -0.0002136081457f
#define CAL_GROUND_H4 -0.003051383421f
#define CAL_GROUND_H5 0.5774975419f
#define CAL_GROUND_H6 0.0005554588279f
#define CAL_GROUND_H7 0.01398310903f
#define CAL_MASK 0u
#define CAL_CAPTURE_VALIDATED 0
#define CAL_FULL_RACE 0
#define CAL_VISION_V2_VALIDATED 0 /* revalidate Otsu, contour timing and steering gains */
#endif
