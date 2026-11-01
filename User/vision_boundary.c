/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
#include <string.h>
/* One bit per pixel, no second grayscale frame. Non-reentrant: main-loop owner. */
static uint8_t component[(APP_WIDTH * APP_HEIGHT + 7u) / 8u];
static union {
    uint16_t histogram[256];
    VisionPoint seeds[128];
} scratch;
#define histogram scratch.histogram
#define seeds scratch.seeds
static bool inside(int x, int y) {
    return x >= 0 && y >= 0 && x < (int)APP_WIDTH && y < (int)APP_HEIGHT;
}
bool vision_component_pixel(unsigned x, unsigned y) {
    if (x >= APP_WIDTH || y >= APP_HEIGHT)
        return false;
    unsigned i = y * APP_WIDTH + x;
    return (component[i / 8] & (1u << (i % 8))) != 0;
}
static void mark(unsigned x, unsigned y) {
    unsigned i = y * APP_WIDTH + x;
    component[i / 8] |= (uint8_t)(1u << (i % 8));
}
static bool white(const FrameView *f, int x, int y, unsigned threshold) {
    return inside(x, y) && f->pixels[(unsigned)y * f->stride + (unsigned)x] > threshold;
}
static bool otsu(const FrameView *f, uint8_t *threshold) {
    memset(histogram, 0, sizeof(histogram));
    uint32_t sum = 0;
    for (unsigned y = 0; y < APP_HEIGHT; y++)
        for (unsigned x = 0; x < APP_WIDTH; x++) {
            unsigned v = f->pixels[y * f->stride + x];
            histogram[v]++;
            sum += v;
        }
    uint32_t count = 0, partial = 0, total = APP_WIDTH * APP_HEIGHT;
    double best = -1;
    unsigned cut = 0;
    for (unsigned t = 0; t < 255; t++) {
        count += histogram[t];
        partial += t * histogram[t];
        if (!count || count == total)
            continue;
        double delta = (double)partial * total - (double)sum * count;
        double variance = delta * delta / ((double)count * (total - count));
        if (variance > best) {
            best = variance;
            cut = t;
        }
    }
    *threshold = (uint8_t)cut;
    /* Uniform or almost-uniform images are not roads. */
    return best > 0 && best / ((double)total * total) >= 16.0;
}
static bool flood(const FrameView *f, unsigned sx, unsigned sy, unsigned threshold) {
    unsigned top = 0;
    seeds[top++] = (VisionPoint){(uint8_t)sx, (uint8_t)sy};
    while (top) {
        VisionPoint s = seeds[--top];
        if (vision_component_pixel(s.x, s.y))
            continue;
        int left = s.x, right = s.x;
        while (white(f, left - 1, s.y, threshold) &&
               !vision_component_pixel((unsigned)(left - 1), s.y))
            left--;
        while (white(f, right + 1, s.y, threshold) &&
               !vision_component_pixel((unsigned)(right + 1), s.y))
            right++;
        for (int x = left; x <= right; x++)
            mark((unsigned)x, s.y);
        for (int dy = -1; dy <= 1; dy += 2) {
            int y = (int)s.y + dy;
            if (!inside(0, y))
                continue;
            bool run = false;
            for (int x = left; x <= right; x++) {
                bool fresh =
                    white(f, x, y, threshold) && !vision_component_pixel((unsigned)x, (unsigned)y);
                if (fresh && !run) {
                    if (top >= 128)
                        return false;
                    seeds[top++] = (VisionPoint){(uint8_t)x, (uint8_t)y};
                }
                run = fresh;
            }
        }
    }
    return true;
}
/* Clockwise unique perimeter: bottom -> left -> top -> right. Corners occur once. */
#define PERIMETER (2u * APP_WIDTH + 2u * APP_HEIGHT - 4u)
static VisionPoint perimeter(unsigned i, uint8_t *side) {
    i %= PERIMETER;
    if (i < APP_WIDTH) {
        *side = 1;
        return (VisionPoint){(uint8_t)(APP_WIDTH - 1 - i), APP_HEIGHT - 1};
    }
    i -= APP_WIDTH;
    if (i < APP_HEIGHT - 1) {
        *side = 2;
        return (VisionPoint){0, (uint8_t)(APP_HEIGHT - 2 - i)};
    }
    i -= APP_HEIGHT - 1;
    if (i < APP_WIDTH - 1) {
        *side = 4;
        return (VisionPoint){(uint8_t)(i + 1), 0};
    }
    i -= APP_WIDTH - 1;
    *side = 8;
    return (VisionPoint){APP_WIDTH - 1, (uint8_t)(i + 1)};
}
static bool trace(unsigned sx, unsigned sy, BoundaryFeatures *o) {
    static const int8_t dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1}, dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};
    int x = (int)sx, y = (int)sy, bx = x - 1, by = y, firstx = -1, firsty = -1;
    VisionPoint a = {(uint8_t)x, (uint8_t)y}, b = a;
    for (unsigned step = 0; step < APP_WIDTH * APP_HEIGHT * 2u; step++) {
        int start = 0;
        for (int k = 0; k < 8; k++)
            if (x + dx[k] == bx && y + dy[k] == by) {
                start = k;
                break;
            }
        int chosen = -1;
        for (int k = 1; k <= 8; k++) {
            int n = (start + k) % 8;
            if (inside(x + dx[n], y + dy[n]) &&
                vision_component_pixel((unsigned)(x + dx[n]), (unsigned)(y + dy[n]))) {
                chosen = n;
                break;
            }
        }
        if (chosen < 0)
            return false;
        int nx = x + dx[chosen], ny = y + dy[chosen];
        if (step > 0 && x == (int)sx && y == (int)sy && nx == firstx && ny == firsty) {
            o->contour_steps = (uint16_t)step;
            return true;
        }
        if (step == 0) {
            firstx = nx;
            firsty = ny;
        }
        if (step && step % 8 == 0) {
            int ux = (int)b.x - a.x, uy = (int)b.y - a.y, vx = x - b.x, vy = y - b.y;
            int dot = ux * vx + uy * vy, cross = ux * vy - uy * vx;
            if ((cross > 20 || cross < -20) && dot < 40 && o->corner_count < 16) {
                unsigned j = o->corner_count++;
                o->corners[j] = b;
                o->corner_side[j] = b.x < APP_WIDTH / 2 ? 0 : 1;
            }
            a = b;
            b = (VisionPoint){(uint8_t)x, (uint8_t)y};
        }
        int previous = (chosen + 7) % 8;
        bx = x + dx[previous];
        by = y + dy[previous];
        x = nx;
        y = ny;
    }
    return false;
}
bool vision_boundary_process(const FrameView *f, BoundaryFeatures *o) {
    if (!f || !o || !f->pixels || f->width != APP_WIDTH || f->height != APP_HEIGHT ||
        f->stride < APP_WIDTH)
        return false;
    memset(o, 0, sizeof(*o));
    memset(component, 0, sizeof(component));
    if (!otsu(f, &o->threshold))
        return true;
    int sx = -1, sy = -1;
    for (int y = (int)APP_HEIGHT - 1; y >= (int)APP_HEIGHT - 9 && sx < 0; y--) {
        for (unsigned d = 0; d < APP_WIDTH / 2; d++) {
            int left = (int)APP_WIDTH / 2 - (int)d, right = (int)APP_WIDTH / 2 + (int)d;
            if (white(f, left, y, o->threshold)) {
                sx = left;
                sy = y;
                break;
            }
            if (white(f, right, y, o->threshold)) {
                sx = right;
                sy = y;
                break;
            }
        }
    }
    if (sx < 0)
        return true;
    if (!flood(f, (unsigned)sx, (unsigned)sy, o->threshold)) {
        o->overflow = true;
        return true;
    }
    unsigned black = PERIMETER;
    uint8_t side;
    for (unsigned i = 0; i < PERIMETER; i++) {
        VisionPoint p = perimeter(i, &side);
        if (!vision_component_pixel(p.x, p.y)) {
            black = i;
            break;
        }
    }
    if (black == PERIMETER)
        return true;
    unsigned length = 0, start = 0;
    uint8_t sides = 0;
    for (unsigned k = 1; k <= PERIMETER; k++) {
        unsigned i = (black + k) % PERIMETER;
        VisionPoint p = perimeter(i, &side);
        if (vision_component_pixel(p.x, p.y)) {
            if (!length)
                start = i;
            length++;
            sides |= side;
        } else if (length) {
            if (length >= 2) {
                if (o->port_count == 8) {
                    o->overflow = true;
                    return true;
                }
                BorderPort *port = &o->ports[o->port_count++];
                port->midpoint = perimeter(start + length / 2, &side);
                port->sides = sides;
            }
            length = 0;
            sides = 0;
        }
    }
    while (sx > 0 && vision_component_pixel((unsigned)(sx - 1), (unsigned)sy))
        sx--;
    o->valid = trace((unsigned)sx, (unsigned)sy, o) && o->port_count >= 2;
    unsigned left = 0, right = 0;
    for (unsigned i = 0; i < o->port_count; i++) {
        left += o->ports[i].midpoint.x < APP_WIDTH / 2;
        right += o->ports[i].midpoint.x >= APP_WIDTH / 2;
    }
    o->island_candidate = o->port_count == 4 && (left >= 3 || right >= 3);
    return true;
}
