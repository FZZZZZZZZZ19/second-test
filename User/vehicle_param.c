/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
#include <string.h>
void param_defaults(AppParam *p) {
    memset(p, 0, sizeof(*p));
    p->encoder_sign[0] = 1;
    p->encoder_sign[1] = 1;
    p->encoder_tau_s = .025f;
    p->kp_left = .2f;
    p->kp_right = .2f;
    p->ki_left = .4f;
    p->ki_right = .4f;
    p->integral_limit = .2f;
    p->duty_limit = .25f;
    p->speed_limit_mps = .4f;
    p->accel_mps2 = .3f;
    p->decel_mps2 = .6f;
    p->inner_ratio = .15f;
    p->visual_kp = 1.5f;
    p->visual_kd = .05f;
    p->visual_tau_s = .05f;
    p->center_x_px = 93.5f;
    p->speed_normal = .2f;
    p->speed_corner = .12f;
    p->speed_element = .1f;
    p->soft_timeout_ms = 150;
    p->hard_timeout_ms = 600;
    p->element_timeout_s = 20;
    p->confirm_ms = 80;
    p->release_ms = 120;
    p->clear_distance_m = .12f;
    p->island_min_distance_m = .5f;
    p->island_min_turn_rad = 5.4f;
    p->width_near_px = 20;
    p->width_far_px = 5;
    p->row_jump_px = 28;
    p->threshold = 130;
}
bool param_valid(const AppParam *p) {
    if (!p)
        return false;
#define CHECK_FINITE(n)                                                                            \
    if (!isfinite(p->n))                                                                           \
        return false;
    APP_FLOAT_FIELDS(CHECK_FINITE)
#undef CHECK_FINITE
    if (p->wheel_left_m < .01f || p->wheel_left_m > .3f || p->wheel_right_m < .01f ||
        p->wheel_right_m > .3f)
        return false;
    if (p->track_m < .03f || p->track_m > .6f || p->axle_m < .01f || p->axle_m > .6f)
        return false;
    if (p->effective_track_m != 0 && (p->effective_track_m < .03f || p->effective_track_m > 1.5f))
        return false;
    if (p->counts_left < 1 || p->counts_right < 1 || p->counts_left > 1000000 ||
        p->counts_right > 1000000)
        return false;
    if (p->encoder_tau_s < 0 || p->encoder_tau_s > 1 || p->integral_limit < 0 ||
        p->integral_limit > 1)
        return false;
    if (p->kp_left < 0 || p->kp_right < 0 || p->ki_left < 0 || p->ki_right < 0 || p->ff_left < 0 ||
        p->ff_right < 0)
        return false;
    if (p->duty_limit <= 0 || p->duty_limit > .8f || p->speed_limit_mps <= 0 ||
        p->speed_limit_mps > 5)
        return false;
    if (p->accel_mps2 <= 0 || p->decel_mps2 <= 0 || p->inner_ratio < 0 || p->inner_ratio >= 1)
        return false;
    if (p->visual_kp < 0 || p->visual_kd < 0 || p->visual_tau_s < 0 || p->center_x_px < 0 ||
        p->center_x_px >= APP_WIDTH)
        return false;
    if (p->speed_normal < 0 || p->speed_corner < 0 || p->speed_element < 0 ||
        p->speed_normal > p->speed_limit_mps || p->speed_corner > p->speed_limit_mps ||
        p->speed_element > p->speed_limit_mps)
        return false;
    if (p->soft_timeout_ms < 20 || p->hard_timeout_ms <= p->soft_timeout_ms ||
        p->hard_timeout_ms > 2000)
        return false;
    if (p->confirm_ms < 20 || p->release_ms < 20 || p->element_timeout_s < 1 ||
        p->element_timeout_s > 120)
        return false;
    if (p->clear_distance_m < 0 || p->clear_distance_m > 1 || p->island_min_distance_m <= 0 ||
        p->island_min_turn_rad < 3 || p->island_min_turn_rad > 10)
        return false;
    if (p->row_jump_px < 1 || p->row_jump_px > APP_WIDTH || p->width_near_px < 1 ||
        p->width_far_px < 1 || p->width_near_px > APP_WIDTH || p->width_far_px > APP_WIDTH)
        return false;
    return (p->encoder_sign[0] == 1 || p->encoder_sign[0] == -1) &&
           (p->encoder_sign[1] == 1 || p->encoder_sign[1] == -1);
}
bool param_can_arm(const AppParam *p, bool race) {
    return param_valid(p) && p->row_distance_near_m > 0 &&
           p->row_distance_far_m > p->row_distance_near_m &&
           ((p->calibration_mask & APP_CAL_REQUIRED) == APP_CAL_REQUIRED) &&
           ((p->flags & 2u) != 0) && (!race || (p->flags & 1u));
}
float param_track(const AppParam *p) {
    return p->effective_track_m > 0 ? p->effective_track_m : p->track_m;
}
static void put32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
bool param_encode(const AppParam *p, uint32_t seq, uint8_t out[APP_PARAM_RECORD_BYTES]) {
    if (!param_valid(p) || !out)
        return false;
    _Static_assert(sizeof(float) == 4, "IEEE float32 required");
    memset(out, 0xff, APP_PARAM_RECORD_BYTES);
    put32(out, 0x53545631u);
    put32(out + 4, 1);
    put32(out + 8, seq);
    put32(out + 16, p->calibration_mask);
    put32(out + 20, p->flags);
    put32(out + 24, p->threshold);
    put32(out + 28, (p->encoder_sign[0] < 0 ? 1u : 0u) | (p->encoder_sign[1] < 0 ? 2u : 0u));
    unsigned offset = 32, n = 0;
#define ENCODE(nm)                                                                                 \
    {                                                                                              \
        uint32_t bits;                                                                             \
        memcpy(&bits, &p->nm, 4);                                                                  \
        put32(out + offset, bits);                                                                 \
        offset += 4;                                                                               \
        n++;                                                                                       \
    }
    APP_FLOAT_FIELDS(ENCODE)
#undef ENCODE
    if (offset > 248)
        return false;
    put32(out + 12, n);
    put32(out + 248, app_crc32(out, 248));
    put32(out + 252, 0x434F4D54u);
    return true;
}
bool param_decode(const uint8_t in[APP_PARAM_RECORD_BYTES], AppParam *p, uint32_t *seq) {
    if (!in || !p || !seq || get32(in) != 0x53545631u || get32(in + 4) != 1 ||
        get32(in + 252) != 0x434F4D54u || get32(in + 248) != app_crc32(in, 248))
        return false;
    AppParam candidate;
    memset(&candidate, 0, sizeof(candidate));
    unsigned offset = 32, n = 0;
#define DECODE(nm)                                                                                 \
    {                                                                                              \
        uint32_t bits = get32(in + offset);                                                        \
        memcpy(&candidate.nm, &bits, 4);                                                           \
        offset += 4;                                                                               \
        n++;                                                                                       \
    }
    APP_FLOAT_FIELDS(DECODE)
#undef DECODE
    if (get32(in + 12) != n || get32(in + 24) > 255 || get32(in + 28) > 3)
        return false;
    candidate.calibration_mask = get32(in + 16);
    candidate.flags = get32(in + 20);
    candidate.threshold = (uint8_t)get32(in + 24);
    candidate.encoder_sign[0] = (get32(in + 28) & 1) ? -1 : 1;
    candidate.encoder_sign[1] = (get32(in + 28) & 2) ? -1 : 1;
    if (!param_valid(&candidate))
        return false;
    *p = candidate;
    *seq = get32(in + 8);
    return true;
}
