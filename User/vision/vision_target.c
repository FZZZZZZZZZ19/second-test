/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "vision_api.h"
#include <math.h>
#include <string.h>
/* Project selected path samples only, not the full image. Calibration origin is
 * the vehicle reference point, with x right and z forward, both in metres. */
static bool project(const AppParam *p,float x,float y,float *right,float *forward) {
    float w=p->ground_h6*x+p->ground_h7*y+1;
    if(!isfinite(w)||fabsf(w)<1e-6f) return false;
    *right=(p->ground_h0*x+p->ground_h1*y+p->ground_h2)/w;
    *forward=(p->ground_h3*x+p->ground_h4*y+p->ground_h5)/w;
    return isfinite(*right)&&isfinite(*forward)&&*forward>0;
}
bool vision_target_step(const VisionFeatures *f,const TrackDecision *d,const AppParam *p,VisionOutput *o) {
    if(!f||!d||!p||!o)return false;
    memset(o,0,sizeof(*o));
    o->frame_id=f->frame_id; o->capture_ms=f->capture_ms;
    o->state=d->state; o->confidence=f->confidence;
    o->stop_requested=d->stop; o->reset_derivative=d->reset_d; o->speed_cap_mps=d->speed_cap;
    if (!d->valid || d->path_index>=f->path_count) return false;
    const PathCandidate *q=&f->paths[d->path_index];
    unsigned rows[APP_ROWS],n=0;
    for(unsigned i=0;i<APP_ROWS;i++)if(q->valid_rows&(1u<<i))rows[n++]=i;
    if(n<3)return false;
    unsigned chosen[]={rows[0],rows[n/2],rows[n-1]};
    float u[3],v[3];
    for(unsigned i=0;i<3;i++) {
        unsigned row=chosen[i];
        u[i]=q->x[row];
        v[i]=APP_HEIGHT-3u-row*APP_ROW_STEP;
    }
    return vision_project_path(p,u,v,0,o);
}

bool vision_project_path(const AppParam *p,const float u[3],const float v[3],float forward_offset_m,VisionOutput *o) {
    o->valid=false;
    o->near_transverse=false;
    if (!isfinite(o->speed_cap_mps) || o->speed_cap_mps < 0)
        return false;
    float h[]={p->ground_h0,p->ground_h1,p->ground_h2,p->ground_h3,p->ground_h4,p->ground_h5,p->ground_h6,p->ground_h7};
    for(unsigned i=0;i<8;i++)if(!isfinite(h[i]))return false;
    float det=h[0]*(h[4]-h[5]*h[7])-h[1]*(h[3]-h[5]*h[6])+h[2]*(h[3]*h[7]-h[4]*h[6]);
    if(!isfinite(det)||fabsf(det)<1e-12f)return false;
    float x[3],z[3];
    for (unsigned i=0;i<3;i++) {
        if (!isfinite(u[i]) || !isfinite(v[i]) || u[i]<0 || u[i]>=APP_WIDTH ||
            v[i]<0 || v[i]>=APP_HEIGHT || !project(p,u[i],v[i],&x[i],&z[i]))
            return false;
        z[i] -= forward_offset_m;
    }
    const float min_point_distance_m=.005f;
    const float transverse_forward_tolerance_m=.030f;
    float dx1=x[1]-x[0], dz1=z[1]-z[0];
    float dx2=x[2]-x[1], dz2=z[2]-z[1];
    bool near_transverse=fabsf(dz1)<=transverse_forward_tolerance_m &&
                         fabsf(dx1)>=fmaxf(min_point_distance_m,fabsf(dz1));
    bool far_transverse=fabsf(dz2)<=transverse_forward_tolerance_m &&
                        fabsf(dx2)>=fmaxf(min_point_distance_m,fabsf(dz2));
    if ((dz1<=0 && !near_transverse) || (dz2<=0 && !far_transverse))
        return false;
    float a=hypotf(dz1,dx1), b=hypotf(dz2,dx2), c=hypotf(dz1+dz2,dx1+dx2);
    float denominator=a*b*c;
    if (!isfinite(denominator) || a<min_point_distance_m || b<min_point_distance_m ||
        c<min_point_distance_m) return false;
    /* The samples stay in contour order. Reject a fold-back, but keep a right angle. */
    if ((near_transverse || far_transverse) && dx1*dx2+dz1*dz2 < -.5f*a*b)
        return false;
    o->lateral_m=x[0]; o->forward_m=z[0];
    o->heading_rad=atan2f(dx1,fmaxf(dz1,0));
    /* Signed circumcircle curvature; forward then right gives right-positive. */
    o->curvature=near_transverse ? 0 : 2*(dz1*dx2-dx1*dz2)/denominator;
    if (near_transverse) {
        if (!isfinite(p->speed_corner) || p->speed_corner < 0)
            return false;
        o->speed_cap_mps=fminf(o->speed_cap_mps,p->speed_corner);
        o->near_transverse=true;
    }
    o->valid=isfinite(o->curvature)&&isfinite(o->heading_rad);
    return o->valid;
}
