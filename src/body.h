/*
 * body.h — Body data structure and orbital mechanics
 */
#pragma once
#include "common.h"

typedef struct {
    char   name[32];
    double mass;           /* kg                              */
    double radius;         /* m (physical)                    */
    double pos[3];         /* m, simulation frame             */
    double vel[3];         /* m/s                             */
    double acc[3];         /* m/s^2 (recomputed each step)    */
    double fast_acc[3];    /* m/s^2 dominant parent force, RESPA inner step */
    float  col[3];         /* RGB display colour              */
    int    is_star;
    int    is_black_hole;   /* 1 = black hole (also has is_star=1 as a system root) */
    int    alive;           /* 0 = removed/absorbed; index kept stable */
    int    parent;         /* index of parent body (-1 = none)                  */
                           /* stars: -1; planets: star idx; moons: planet idx   */
    double dyn_period;     /* s, estimated local orbital/dynamical period       */
    double dyn_dt_outer;   /* s, recommended slow-force timestep ceiling         */
    double dyn_dt_inner;   /* s, recommended parent-force timestep ceiling       */
    int    dyn_bucket;     /* 0=slow .. 3=very fast                             */

    /* Rotation */
    double obliquity;       /* axial tilt in degrees (from ecliptic north)  */
    double rotation_rate;   /* rad/s (positive = prograde)                  */
    double rotation_angle;  /* current rotation phase, rad (0..2π)          */
    double cloud_rotation;  /* continuous cloud angle, rad (never wrapped)  */

    /* Atmosphere (set by universe loader; zero = no atmosphere) */
    float  atm_color[3];    /* RGB atmosphere rim colour                    */
    float  atm_intensity;   /* peak glow strength (0 = no atmosphere)       */
    float  atm_scale;       /* outer atm radius as multiple of planet radius */

    /* Accretion disk (black holes only; disk_outer == 0 means no disk) */
    float  disk_color[3];   /* base emissive colour of the disk             */
    float  disk_inner;      /* inner radius as multiple of event horizon    */
    float  disk_outer;      /* outer radius as multiple of event horizon    */
    double disk_angle;      /* current swirl phase, rad (never wrapped)     */

    /* Orbital trail (circular buffer, TRAIL_LEN samples, positions in AU) */
    double trail_accum;      /* meters accumulated toward next sample      */
    int    trail_head;       /* index of next write slot                   */
    int    trail_count;      /* number of valid samples (0..TRAIL_LEN)     */
    double (*trail)[3];      /* heap-allocated [TRAIL_LEN][3]              */
    double *trail_seg_len;   /* segment length ending at each sample index */
    double trail_total_len;  /* retained trail length in world meters      */
    double trail_fade;       /* 1.0 = full alpha; fades to 0 after death   */
    int    trail_emitting;   /* 1 while the body should keep adding points */
    double trail_prev_pos[3];/* previous trail tick position for interpolation */
    double trail_prev_vel[3];/* previous trail tick velocity for curve reconstruction */
    double trail_frame_accum;     /* meters accumulated at frame start        */
    int    trail_frame_head;      /* trail head snapshot at frame start       */
    int    trail_frame_count;     /* trail count snapshot at frame start      */
    double trail_frame_total_len; /* retained trail length at frame start     */
    double trail_frame_pos[3];    /* body position at frame start             */
    double trail_frame_vel[3];    /* body velocity at frame start             */
    double trail_frame_prev_pos[3];/* previous curve anchor at frame start    */
    double trail_frame_prev_vel[3];/* previous curve velocity at frame start  */
} Body;

/* g_bodies is a heap-allocated array that grows via realloc.
 * g_nbodies is the high-water slot count, not the number of alive bodies.
 * Absorbed slots stay addressable for stable indices and may be reused by
 * universe_add_body() once their Body.alive flag is clear.
 * g_bodies_cap is the current allocated capacity.
 * MAX_BODIES (common.h) is also the compile-time bound for per-frame arrays
 * in render/labels/physics/collision, so runtime-added body indices stay
 * below that value. */
extern Body *g_bodies;
extern int   g_nbodies;
extern int   g_bodies_cap;

/* State from Keplerian elements around a star of given GM (angles in degrees,
 * a in AU, gm_au_day2 in AU³/day² — use GM_SUN for Sol planets). */
void keplerian_to_state(
        double a, double e, double i_deg,
        double Omega_deg, double omega_tilde_deg, double L_deg,
        double gm_au_day2,
        double pos_m[3], double vel_ms[3]);

/* Index of the star body nearest to the camera (camera.h must be included first). */
int nearest_star_idx(void);

/* Walk parent links to find the owning root star for a body. */
int body_root_star(int i);

/* Convert a world-space direction into the body's surface-local frame.
 * This matches the local-space convention used by the planet shader. */
void body_world_to_local_surface_dir(int body_idx, const double world_dir[3],
                                     float out[3]);

/* Planetocentric state from simple moon elements (a in km, angles in degrees). */
void moon_to_state(
        double a_km, double e, double i_deg,
        double Omega_deg, double omega_deg, double M0_deg,
        double gm,
        double pos_m[3], double vel_ms[3]);
