/*
 * lensing.h - gravitational black-hole compositor (geodesic ray bending)
 *
 * The 3D scene (stars, bodies, trails, glare) is rendered into an offscreen
 * colour+depth buffer. lensing_resolve() then runs a fullscreen pass that, for
 * every pixel whose view ray passes near a black hole, integrates the bent
 * photon path: the event horizon shows as captured (black) rays, the accretion
 * disk is evaluated analytically along the path (so it correctly wraps over and
 * under the hole), a photon-ring glow is added at the photon sphere, and the
 * star background is sampled in the final deflected direction. Pixels far from
 * every hole pass the captured scene through untouched. UI/text is drawn after
 * the resolve on the default framebuffer and stays crisp.
 *
 * Usage per frame (only when at least one black hole is on screen):
 *   if (lensing_begin_scene()) {  ... render the whole scene ...  }
 *   lensing_resolve(sources, n, bodies, nb, sun_pos, sun_col,
 *                   cam_right, cam_up, cam_fwd, fov_tan);
 */
#pragma once
#include "common.h"

#define LENS_MAX_HOLES 4
#define LENS_MAX_BODIES 32   /* scene spheres ray-traced through the lens */

typedef struct {
    float center[3];   /* hole centre, camera-relative render-AU (pos*RS - cam) */
    float r_horizon;   /* event-horizon radius, AU */
    float r_inner;     /* disk inner radius, AU (0 disables the disk) */
    float r_outer;     /* disk outer radius, AU */
    float disk_u[3];   /* in-plane unit basis (azimuth reference) */
    float disk_v[3];   /* in-plane unit basis, completes the plane */
    float disk_n[3];   /* plane normal (unit) = u x v */
    float color[3];    /* disk / ring base colour */
    float disk_time;   /* swirl phase, rad */
} LensSource;

/* A scene body the geodesic ray-tracer intersects as an analytic sphere, so
 * bodies behind the hole are genuinely lensed and bodies in front correctly
 * occlude it (first hit along the bent path wins). */
typedef struct {
    float center[3];   /* camera-relative render-AU (pos*RS - cam) */
    float radius;      /* render-AU */
    float color[3];    /* base display colour (fallback shading) */
    int   emissive;    /* 1 = star/self-luminous (no lambert term) */
} LensBody;

void lensing_init(void);

/*
 * Ensure the offscreen target matches the current window size and bind it.
 * Returns 1 if the FBO is ready (caller should clear + render into it), or 0
 * if lensing is unavailable (caller renders straight to the default fb).
 */
int  lensing_begin_scene(void);

/*
 * Snapshot the current contents of the offscreen colour buffer into the
 * "background" texture used for bent-ray sampling. Call this once the distant
 * sky (starfield) has been drawn but BEFORE any near geometry (trails, bodies):
 * only sky is at infinity, so only sky may be sampled by direction, otherwise
 * nearby objects with parallax appear as lensed ghost copies.
 */
void lensing_capture_background(void);

/*
 * Resolve the captured scene to the default framebuffer, compositing each black
 * hole via geodesic ray bending. Must be paired with a successful
 * lensing_begin_scene(). cam_* are the camera basis vectors (from view_rot) and
 * fov_tan = tan(0.5*FOV); these reconstruct the per-pixel view ray.
 */
void lensing_resolve(const LensSource *sources, int count,
                     const LensBody *bodies, int nbody,
                     const float sun_pos[3], const float sun_col[3],
                     const float cam_right[3], const float cam_up[3],
                     const float cam_fwd[3], float fov_tan);

void lensing_shutdown(void);
