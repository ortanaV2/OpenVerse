/*
 * physics.h — N-body gravitational physics (RESPA hierarchical integrator)
 *
 * Force decomposition:
 *   Slow forces — primary-body interactions (planets among themselves, plus
 *                 tidal perturbations from non-parent primaries on satellites).
 *                 Updated once per outer step (~1 day).
 *   Fast forces — dominant parent→satellite force.
 *                 Updated every inner step (0.02 days) for orbital accuracy.
 *
 * Usage (each frame):
 *   for each outer step:
 *       physics_respa_begin(dt_outer);
 *       for each inner step:
 *           physics_respa_inner(dt_inner);
 *           trails_tick(dt_inner);
 *       physics_respa_end(dt_outer);
 */
#pragma once
#include "common.h"

extern double g_sim_time;
extern double g_sim_speed;
extern int    g_paused;

/* RESPA split-step integrator — call begin/inner.../end per outer step */
void physics_respa_begin(double dt_outer);  /* slow half-kick + prime fast acc  */
void physics_respa_inner(double dt_inner);  /* fast KDK + drift all bodies      */
void physics_respa_end  (double dt_outer);  /* slow half-kick + rotation + time */

/* Legacy single-step KDK (kept for reference / one-off use) */
void physics_step(double dt);

/* Trail helpers */
void trails_sample(void);
void trails_tick(double dt);
