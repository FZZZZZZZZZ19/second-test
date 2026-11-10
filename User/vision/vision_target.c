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
    float h[]={p->ground_h0,p->ground_h1,p->ground_h2,p->ground_h3,p->ground_h4,p->ground_h5,p->ground_h6,p->ground_h7};
    for(unsigned i=0;i<8;i++)if(!isfinite(h[i]))return false;
    float det=h[0]*(h[4]-h[5]*h[7])-h[1]*(h[3]-h[5]*h[6])+h[2]*(h[3]*h[7]-h[4]*h[6]);
    if(!isfinite(det)||fabsf(det)<1e-12f||!d->valid||d->path_index>=f->path_count)return false;
    const PathCandidate *q=&f->paths[d->path_index];
    unsigned rows[APP_ROWS],n=0;
    for(unsigned i=0;i<APP_ROWS;i++)if(q->valid_rows&(1u<<i))rows[n++]=i;
    if(n<3)return false;
    unsigned chosen[]={rows[0],rows[n/2],rows[n-1]};
    float x[3],z[3];
    for(unsigned i=0;i<3;i++) {
        unsigned row=chosen[i];
        if(!isfinite(q->x[row])||q->x[row]<0||q->x[row]>=APP_WIDTH)return false;
        if(!project(p,q->x[row],APP_HEIGHT-3u-row*APP_ROW_STEP,&x[i],&z[i]))return false;
    }
    if(!(z[0]<z[1]&&z[1]<z[2]))return false;
    float a=hypotf(z[1]-z[0],x[1]-x[0]);
    float b=hypotf(z[2]-z[1],x[2]-x[1]);
    float c=hypotf(z[2]-z[0],x[2]-x[0]);
    float denominator=a*b*c;
    if(!isfinite(denominator)||denominator<1e-12f)return false;
    o->lateral_m=x[0]; o->forward_m=z[0];
    /* Signed circumcircle curvature; forward then right gives right-positive. */
    o->curvature=2*((z[1]-z[0])*(x[2]-x[0])-(x[1]-x[0])*(z[2]-z[0]))/denominator;
    o->valid=isfinite(o->curvature)&&isfinite(o->speed_cap_mps)&&o->speed_cap_mps>=0;
    return o->valid;
}
