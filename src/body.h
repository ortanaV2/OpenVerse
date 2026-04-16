/*
 * body.h — Body data structure and solar-system initialisation
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
    int    parent;         /* index of parent body (-1 = none) */

    /* Rotation */
    double obliquity;       /* axial tilt in degrees (from ecliptic north)  */
    double rotation_rate;   /* rad/s (positive = prograde)                  */
    double rotation_angle;  /* current rotation phase, rad (0..2π)          */

    /* Orbital trail (circular buffer, positions in GL/AU units) */
    double trail_interval;     /* sim-seconds between samples (per-body)   */
    double trail_accum;        /* accumulator toward next sample            */
    int    trail_head;
    int    trail_count;
    double trail[TRAIL_LEN][3]; /* AU, camera-relative subtraction done at render time */
} Body;

extern Body g_bodies[MAX_BODIES];
extern int  g_nbodies;

/* Populate g_bodies with real J2000.0 Keplerian elements */
void solar_system_init(void);
