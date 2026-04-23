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

    /* Atmosphere (set by universe loader; zero = no atmosphere) */
    float  atm_color[3];    /* RGB atmosphere rim colour                    */
    float  atm_intensity;   /* peak glow strength (0 = no atmosphere)       */
    float  atm_scale;       /* outer atm radius as multiple of planet radius */

    /* Orbital trail (circular buffer, TRAIL_LEN samples, positions in AU) */
    double trail_interval;   /* sim-seconds between samples (≈ T/200)     */
    double trail_accum;      /* accumulator toward next sample             */
    int    trail_head;       /* index of next write slot                   */
    int    trail_count;      /* number of valid samples (0..TRAIL_LEN)     */
    double (*trail)[3];      /* heap-allocated [TRAIL_LEN][3]              */
} Body;

/* g_bodies is a heap-allocated array that grows via realloc.
 * g_bodies_cap is the current allocated capacity.
 * MAX_BODIES (common.h) is used only as the initial allocation size
 * and as the compile-time bound for per-frame stack arrays in render/labels. */
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
