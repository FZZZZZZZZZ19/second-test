/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include "track_profile.h"
#include <math.h>
#include <string.h>
static const uint8_t route[] = APP_ROUTE_SEQUENCE;
_Static_assert(APP_ROUTE_LENGTH <= 32, "Route supports up to 32 events per lap");
_Static_assert(APP_ROUTE_LENGTH == 0 || sizeof(route) == APP_ROUTE_LENGTH, "Route length mismatch");
static float endpoint(const PathCandidate *q) {
    for (int i = (int)APP_ROWS - 1; i >= 0; i--)
        if (q->valid_rows & (1u << i))
            return q->x[i];
    return APP_WIDTH * .5f;
}
static unsigned select_path(const VisionFeatures *f, float x, float trend) {
    unsigned best = 0;
    float cost = 1e9f;
    for (unsigned j = 0; j < f->path_count; j++) {
        float c = fabsf(endpoint(&f->paths[j]) - x) + 3 * fabsf(f->paths[j].trend - trend) -
                  f->paths[j].count;
        if (c < cost) {
            cost = c;
            best = j;
        }
    }
    return best;
}
static unsigned select_branch(const VisionFeatures *f, bool right) {
    unsigned k = 0;
    for (unsigned j = 1; j < f->path_count; j++)
        if (right ? (endpoint(&f->paths[j]) > endpoint(&f->paths[k]))
                  : (endpoint(&f->paths[j]) < endpoint(&f->paths[k])))
            k = j;
    return k;
}
void track_element_reset(TrackContext *s, uint32_t session, bool race, uint8_t y_count) {
    memset(s, 0, sizeof(*s));
    s->session = session;
    s->state = TRACK_NORMAL;
    s->race = race;
    s->expected_y_count = y_count;
    s->configured = APP_ROUTE_LENGTH > 0;
    s->locked_x = (APP_WIDTH - 1) * .5f;
}
static void enter(TrackContext *s, TrackState state, const VisionFeatures *f, float dist) {
    s->state = state;
    s->entered_ms = f->capture_ms;
    s->entry_s = dist;
    s->confirm_ms = 0;
    s->clear_ms = 0;
}
static void anchor_exit(TrackContext *s, const AppParam *p, float dist, uint32_t now) {
    s->anchor.valid = p->row_distance_near_m > 0;
    s->anchor.x_exit_m = p->camera_forward_m + p->row_distance_near_m;
    s->anchor.s_anchor_m = dist;
    s->anchor.stamp_ms = now;
}
bool track_element_step(TrackContext *s, const VisionFeatures *f, const AppParam *p,
                        const SideFeedback *fb, bool commit, bool yaw_valid, float yaw,
                        TrackDecision *o) {
    if (!s || !f || !p || !fb || !o)
        return false;
    memset(o, 0, sizeof(*o));
    o->state = s->state;
    if (f->frame_id == s->last_frame)
        return false;
    s->last_frame = f->frame_id;
    if (f->path_count == 0 || f->confidence < 40 || f->overflow)
        return true;
    float dist = (fb->rolling_m[0] + fb->rolling_m[1]) * .5f;
    uint32_t now = f->capture_ms;
    unsigned selected = select_path(f, s->locked_x, s->locked_trend);
    TrackState old = s->state;
    o->valid = true;
    o->speed_cap = p->speed_normal;
    if (!commit) {
        s->locked_x = endpoint(&f->paths[selected]);
        s->locked_trend = f->paths[selected].trend;
        o->path_index = (uint8_t)selected;
        return true;
    }
    if (s->state != TRACK_NORMAL &&
        (uint32_t)(now - s->entered_ms) > (uint32_t)(p->element_timeout_s * 1000))
        s->state = TRACK_DEGRADED;
    switch (s->state) {
    case TRACK_NORMAL:
        if (f->cross)
            enter(s, TRACK_CROSS_CONFIRM, f, dist);
        else if (f->split) {
            if (!s->configured) {
                enter(s, TRACK_DEGRADED, f, dist);
                break;
            }
            uint8_t code = route[s->event_index % (APP_ROUTE_LENGTH ? APP_ROUTE_LENGTH : 1u)];
            if (code == 128) {
                enter(s, TRACK_ISLAND_CONFIRM, f, dist);
                s->entry_heading = yaw;
                s->entry_seen = false;
            } else if (code < APP_Y_CAPACITY) {
                s->y_id = code;
                s->choice = (s->passed_mask[code] & 1u) ? 1u : 0u;
                enter(s, TRACK_Y_CONFIRM, f, dist);
            } else
                enter(s, TRACK_DEGRADED, f, dist);
        }
        break;
    case TRACK_CROSS_CONFIRM:
        if (!f->cross) {
            enter(s, TRACK_NORMAL, f, dist);
            break;
        }
        if (now - s->entered_ms >= (uint32_t)p->confirm_ms)
            enter(s, TRACK_CROSS_PASS, f, dist);
        break;
    case TRACK_CROSS_PASS:
        /* Keep the entry heading: broad horizontal runs do not select a turn. */
        if (!f->cross && !f->split) {
            if (!s->clear_ms)
                s->clear_ms = now;
            if (now - s->clear_ms >= (uint32_t)p->release_ms) {
                enter(s, TRACK_CROSS_CLEAR, f, dist);
                anchor_exit(s, p, dist, now);
            }
        } else
            s->clear_ms = 0;
        break;
    case TRACK_CROSS_CLEAR:
        if (vehicle_progress_rear_clear(&s->anchor, p, fb, !f->split && !f->cross, now))
            enter(s, TRACK_NORMAL, f, dist);
        break;
    case TRACK_Y_CONFIRM:
        if (!f->split) {
            enter(s, TRACK_NORMAL, f, dist);
            break;
        }
        selected = select_branch(f, s->choice != 0);
        if (now - s->entered_ms >= (uint32_t)p->confirm_ms)
            enter(s, TRACK_Y_COMMITTED, f, dist);
        break;
    case TRACK_Y_COMMITTED:
        if (!f->split) {
            if (!s->clear_ms)
                s->clear_ms = now;
            if (now - s->clear_ms >= (uint32_t)p->release_ms)
                enter(s, TRACK_Y_MERGE, f, dist);
        } else
            s->clear_ms = 0;
        break;
    case TRACK_Y_MERGE:
        if (!f->split) {
            enter(s, TRACK_Y_CLEAR, f, dist);
            anchor_exit(s, p, dist, now);
        }
        break;
    case TRACK_Y_CLEAR:
        if (vehicle_progress_rear_clear(&s->anchor, p, fb, !f->split, now)) {
            s->passed_mask[s->y_id] |= (uint8_t)(1u << s->choice);
            if (s->visits[s->y_id] < 255)
                s->visits[s->y_id]++;
            s->event_index++;
            enter(s, TRACK_NORMAL, f, dist);
        }
        break;
    case TRACK_ISLAND_CONFIRM:
        selected = select_branch(f, APP_ISLAND_ENTER_BRANCH != 0);
        if (!f->split) {
            enter(s, TRACK_NORMAL, f, dist);
            break;
        }
        if (now - s->entered_ms >= (uint32_t)p->confirm_ms) {
            enter(s, TRACK_ISLAND_ENTER, f, dist);
            s->entry_heading = yaw;
        }
        break;
    case TRACK_ISLAND_ENTER:
        if (!f->split) {
            s->entry_seen = true;
            enter(s, TRACK_ISLAND_FOLLOW, f, dist);
        }
        break;
    case TRACK_ISLAND_FOLLOW:
        if (f->split) {
            bool enough = dist - s->entry_s >= p->island_min_distance_m;
            bool heading_ok = !yaw_valid || fabsf(yaw - s->entry_heading) >= p->island_min_turn_rad;
            /* Requires a calibrated route profile, visual re-encounter AND distance.
             * No claim that encoder integration alone measures a full physical lap. */
            if (enough && heading_ok && s->entry_seen) {
                if (!s->confirm_ms)
                    s->confirm_ms = now;
                if (now - s->confirm_ms >= (uint32_t)p->confirm_ms) {
                    selected = select_branch(f, APP_ISLAND_ENTER_BRANCH == 0);
                    enter(s, TRACK_ISLAND_EXIT, f, dist);
                }
            } else
                selected = select_branch(f, APP_ISLAND_ENTER_BRANCH != 0);
        } else
            s->confirm_ms = 0;
        break;
    case TRACK_ISLAND_EXIT:
        if (!f->split) {
            if (!s->clear_ms)
                s->clear_ms = now;
            if (now - s->clear_ms >= (uint32_t)p->release_ms) {
                enter(s, TRACK_ISLAND_CLEAR, f, dist);
                anchor_exit(s, p, dist, now);
            }
        } else
            s->clear_ms = 0;
        break;
    case TRACK_ISLAND_CLEAR:
        if (vehicle_progress_rear_clear(&s->anchor, p, fb, !f->split, now)) {
            s->event_index++;
            enter(s, TRACK_NORMAL, f, dist);
        }
        break;
    case TRACK_DEGRADED:
        o->valid = false;
        o->stop = true;
        break;
    default:
        s->state = TRACK_DEGRADED;
        o->valid = false;
        o->stop = true;
        break;
    }
    if (s->state == TRACK_DEGRADED) {
        o->valid = false;
        o->stop = true;
    }
#if APP_ROUTE_LENGTH > 0
    if (s->race && s->event_index >= 2u * APP_ROUTE_LENGTH) {
        if (!s->finishing) {
            s->finishing = true;
            s->finish_s = dist;
        }
        bool all_y_done = true;
        for (unsigned i = 0; i < APP_ROUTE_LENGTH; i++) {
            if (route[i] < APP_Y_CAPACITY && s->passed_mask[route[i]] != 3u)
                all_y_done = false;
        }
        if (!all_y_done || APP_FINISH_DISTANCE_M <= 0) {
            s->state = TRACK_DEGRADED;
            o->valid = false;
            o->stop = true;
        } else if (dist - s->finish_s >= APP_FINISH_DISTANCE_M) {
            o->stop = true;
            o->speed_cap = 0;
        }
    }
#endif
    if (s->state != TRACK_NORMAL)
        o->speed_cap = p->speed_element;
    else if (fabsf(f->paths[selected].trend) > 2 ||
             (f->paths[selected].target_valid && f->paths[selected].target_y > 8))
        o->speed_cap = p->speed_corner;
    s->locked_x = endpoint(&f->paths[selected]);
    s->locked_trend = f->paths[selected].trend;
    o->state = s->state;
    o->path_index = (uint8_t)selected;
    o->reset_d = old != s->state;
    return true;
}
