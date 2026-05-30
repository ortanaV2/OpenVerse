/*
 * blackhole.h — black-hole accretion disk, photon ring, and horizon capture
 *
 * A black hole is stored as a Body with is_star=1 (so it acts as a system root
 * for the physics grouping) and is_black_hole=1. This module owns the extra
 * visuals (a glowing accretion disk and an additive photon ring) and the
 * horizon-capture step that absorbs any body crossing the event horizon.
 *
 * render.c drives init/render/shutdown inside the GL context lifecycle;
 * main.c drives blackhole_step() from the simulation loop.
 */
#pragma once

/* Load shaders and build geometry. Call once after the GL context exists. */
void blackhole_init(void);

/* Advance disk swirl animation and swallow bodies inside any event horizon. */
void blackhole_step(double dt);

/* Draw accretion disks and photon rings. cam_* are the camera basis vectors
 * used by render.c; vp is the camera-relative view-projection matrix. */
void blackhole_render(const float vp[16], const float cam_right[3],
                      const float cam_up[3], const float cam_fwd[3]);

/* Free GL resources. */
void blackhole_shutdown(void);
