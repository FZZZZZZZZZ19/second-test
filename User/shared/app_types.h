#ifndef APP_TYPES_H
#define APP_TYPES_H
#include "app_config.h"
#include "vehicle_param.h"
#include <stdbool.h>
#include <stdint.h>
typedef enum { SIDE_LEFT, SIDE_RIGHT, SIDE_COUNT } SideId;
typedef enum {
    RUN_BOOT,
    RUN_STATIC_DEBUG,
    RUN_READY,
    RUN_RUNNING,
    RUN_STOPPING,
    RUN_FAULT
} RunMode;
typedef enum {
    TRACK_NORMAL,
    TRACK_CROSS_CONFIRM,
    TRACK_CROSS_PASS,
    TRACK_CROSS_CLEAR,
    TRACK_Y_CONFIRM,
    TRACK_Y_COMMITTED,
    TRACK_Y_MERGE,
    TRACK_Y_CLEAR,
    TRACK_ISLAND_CONFIRM,
    TRACK_ISLAND_ENTER,
    TRACK_ISLAND_FOLLOW,
    TRACK_ISLAND_EXIT,
    TRACK_ISLAND_CLEAR,
    TRACK_DEGRADED
} TrackState;
typedef enum {
    STOP_NONE,
    STOP_LOCAL,
    STOP_VISION,
    STOP_ENCODER,
    STOP_NONFINITE,
    STOP_TIMING,
    STOP_TRACK,
    STOP_CONFIG
} StopReason;
typedef enum { REQ_NONE = 0, REQ_READY = 1, REQ_START = 2, REQ_STOP = 4, REQ_RESET = 8 } AppRequest;
typedef struct {
    uint16_t left, right;
} WhiteRun;
typedef struct {
    WhiteRun runs[APP_RUNS];
    uint8_t count;
    bool overflow;
} RowRuns;
typedef struct {
    float x[APP_ROWS];
    uint32_t valid_rows;
    uint8_t count;
    float trend;
    uint8_t target_x, target_y;
    bool target_valid;
} PathCandidate;

typedef struct {
    uint8_t x, y;
} VisionPoint;
typedef struct {
    VisionPoint midpoint;
    uint8_t sides;
} BorderPort;
typedef struct {
    BorderPort ports[8];
    VisionPoint corners[16];
    uint8_t corner_side[16]; /* 0: image left half, 1: image right half */
    uint8_t port_count, corner_count, threshold;
    uint16_t contour_steps;
    bool valid, overflow, island_candidate;
} BoundaryFeatures;
typedef struct {
    uint32_t frame_id, capture_ms;
    RowRuns rows[APP_ROWS];
    PathCandidate paths[APP_PATHS];
    uint8_t path_count, confidence;
    bool cross, split, merge, overflow;
    BoundaryFeatures boundary;
} VisionFeatures;
typedef struct {
    const uint8_t *pixels;
    uint16_t width, height, stride;
    uint32_t frame_id, capture_ms;
} FrameView;
typedef struct {
    uint32_t stamp_ms;
    int32_t delta[2];
    int64_t count[2];
    float speed_mps[2], rolling_m[2];
    bool valid;
} SideFeedback;
typedef struct {
    float left_mps, right_mps, model_v_mps, model_omega_radps;
    bool limited;
} SideTargets;
typedef struct {
    uint32_t frame_id, capture_ms, session_id;
    TrackState state;
    float speed_cap_mps, omega_raw;
    float direction_rad, distance_px;
    uint8_t confidence;
    bool valid, stop_requested;
} MotionRequest;
typedef struct {
    uint32_t stamp_ms, session_id;
    RunMode mode;
    StopReason reason;
    SideFeedback feedback;
    SideTargets targets;
    float duty[2];
    uint32_t vision_age_ms, overruns;
    bool camera_ok, config_ok;
} ControlSnapshot;
typedef struct {
    TrackState state;
    uint8_t path_index;
    bool valid, reset_d, stop;
    float speed_cap;
} TrackDecision;
typedef struct {
    float integral, last_output;
} SpeedPiState;
typedef struct {
    float left, right;
} MotionPlanState;
typedef struct {
    bool valid;
    float x_exit_m, s_anchor_m;
    uint32_t stamp_ms;
} ElementAnchor;
typedef struct {
    float error, derivative;
    uint32_t previous_ms, previous_frame, session;
    bool initialized;
} PathControlState;
typedef struct {
    uint32_t last_frame, entered_ms, confirm_ms, clear_ms, session;
    TrackState state;
    float entry_s, entry_heading, locked_x, locked_trend;
    uint8_t visits[APP_Y_CAPACITY], passed_mask[APP_Y_CAPACITY], y_id, choice;
    uint8_t expected_y_count;
    uint16_t event_index;
    bool finishing;
    float finish_s;
    bool configured, race, entry_seen;
    ElementAnchor anchor;
} TrackContext;
#endif
