/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
#include <string.h>
static float center(WhiteRun r) {
    return ((float)r.left + (float)r.right) * .5f;
}
static bool path_at(const PathCandidate *p, unsigned row) {
    return (p->valid_rows & (1u << row)) != 0;
}
bool vision_line_process(const FrameView *f, const AppParam *p, VisionFeatures *o) {
    if (!f || !p || !o || !f->pixels || f->width != APP_WIDTH || f->height != APP_HEIGHT ||
        f->stride < APP_WIDTH)
        return false;
    memset(o, 0, sizeof(*o));
    o->frame_id = f->frame_id;
    o->capture_ms = f->capture_ms;
    unsigned wide = 0, splits = 0, merge_rows = 0;
    for (unsigned row = 0; row < APP_ROWS; row++) {
        unsigned y = APP_HEIGHT - 3u - row * APP_ROW_STEP;
        RowRuns *r = &o->rows[row];
        const uint8_t *pix = f->pixels + y * f->stride;
        unsigned x = 0;
        float expected =
            p->width_near_px + (p->width_far_px - p->width_near_px) * (float)row / (APP_ROWS - 1u);
        while (x < APP_WIDTH) {
            while (x < APP_WIDTH && pix[x] <= p->threshold)
                x++;
            if (x == APP_WIDTH)
                break;
            unsigned left = x;
            while (x < APP_WIDTH && pix[x] > p->threshold)
                x++;
            unsigned width = x - left;
            if (width < 2)
                continue;
            if (width > APP_WIDTH * 3u / 4u)
                wide++;
            if (r->count < APP_RUNS) {
                r->runs[r->count].left = (uint16_t)left;
                r->runs[r->count].right = (uint16_t)(x - 1);
                r->count++;
            } else {
                r->overflow = true;
                o->overflow = true;
            }
        }
        if (r->count >= 2) {
            float span = center(r->runs[r->count - 1]) - center(r->runs[0]);
            if (span > expected * 1.5f)
                splits++;
        }
        if (row > APP_ROWS / 2 && r->count == 1 && splits >= 2)
            merge_rows++;
    }
    /* Seed near the chassis; never use the centroid of multiple branches. */
    unsigned seed = 0;
    while (seed < 3 && o->rows[seed].count == 0)
        seed++;
    if (seed >= 3 || o->overflow)
        return true;
    RowRuns *base = &o->rows[seed];
    for (unsigned j = 0; j < base->count && j < APP_PATHS; j++) {
        PathCandidate *q = &o->paths[o->path_count++];
        q->x[seed] = center(base->runs[j]);
        q->valid_rows = 1u << seed;
        q->count = 1;
    }
    for (unsigned row = seed + 1; row < APP_ROWS; row++) {
        RowRuns *runs = &o->rows[row];
        uint8_t original = o->path_count;
        bool used[APP_RUNS] = {false};
        for (unsigned k = 0; k < original; k++) {
            PathCandidate *q = &o->paths[k];
            unsigned prev = row;
            while (prev > seed && !path_at(q, prev - 1))
                prev--;
            if (prev == 0 || !path_at(q, prev - 1) || row - (prev - 1) > 3)
                continue;
            float prediction = q->x[prev - 1] + q->trend;
            int best = -1;
            float distance = p->row_jump_px;
            for (unsigned j = 0; j < runs->count; j++) {
                float d = fabsf(center(runs->runs[j]) - prediction);
                if (d < distance) {
                    distance = d;
                    best = (int)j;
                }
            }
            if (best < 0)
                continue;
            /* Clone the previous trajectory at a branch while capacity remains. */
            for (unsigned j = 0; j < runs->count && o->path_count < APP_PATHS; j++) {
                if ((int)j == best || used[j] ||
                    fabsf(center(runs->runs[j]) - prediction) > p->row_jump_px)
                    continue;
                PathCandidate *copy = &o->paths[o->path_count++];
                *copy = *q;
                copy->x[row] = center(runs->runs[j]);
                copy->valid_rows |= 1u << row;
                copy->count++;
                copy->trend = copy->x[row] - q->x[prev - 1];
                used[j] = true;
            }
            float next = center(runs->runs[best]);
            q->trend = .7f * q->trend + .3f * (next - q->x[prev - 1]);
            q->x[row] = next;
            q->valid_rows |= 1u << row;
            q->count++;
            used[best] = true;
        }
    }
    unsigned best_count = 0;
    for (unsigned j = 0; j < o->path_count; j++)
        if (o->paths[j].count > best_count)
            best_count = o->paths[j].count;
    o->confidence = (uint8_t)(100u * best_count / APP_ROWS);
    o->split = splits >= 3;
    o->merge = merge_rows >= 3;
    o->cross = wide >= 2 && !o->split;
    if (wide > APP_ROWS / 2)
        o->confidence = 0;
    return true;
}
