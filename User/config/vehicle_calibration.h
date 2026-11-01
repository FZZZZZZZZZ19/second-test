#ifndef VEHICLE_CALIBRATION_H
#define VEHICLE_CALIBRATION_H
/* Fill measured SI values. Zeros intentionally prevent arming. */
#define CAL_WHEEL_LEFT_M 0.0f
#define CAL_WHEEL_RIGHT_M 0.0f
#define CAL_TRACK_M 0.0f
#define CAL_AXLE_M 0.0f
#define CAL_COUNTS_LEFT 0.0f
#define CAL_COUNTS_RIGHT 0.0f
#define CAL_ROW_NEAR_M 0.0f
#define CAL_ROW_FAR_M 0.0f
#define CAL_CAMERA_FORWARD_M 0.0f
#define CAL_MASK 0u
#define CAL_CAPTURE_VALIDATED 0
#define CAL_FULL_RACE 0
#define CAL_VISION_V2_VALIDATED 0 /* revalidate Otsu, contour timing and steering gains */
#endif
