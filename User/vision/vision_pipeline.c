/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "vision_api.h"
#include <string.h>
bool vision_pipeline_step(TrackContext *track, const FrameView *frame, const AppParam *p,
                          const SideFeedback *feedback, bool commit, uint32_t session, bool race,
                          VisionFeatures *features, VisionOutput *out) {
    if (!track || !frame || !p || !feedback || !features || !out)
        return false;
    memset(out, 0, sizeof(*out));
    out->session_id = session;
    out->capture_ms = frame->capture_ms;
    out->frame_id = frame->frame_id;
    if (track->session != session)
        track_element_reset(track, session, race, 0);
    if (!vision_line_process(frame, p, features))
        return false;
    TrackDecision d;
    if (!track_element_step(track, features, p, feedback, commit, false, 0, &d))
        return false;
    bool ok = vision_target_step(features, &d, p, out);
    out->session_id = session;
    return ok;
}
