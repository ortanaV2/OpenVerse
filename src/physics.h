/*
 * physics.h — N-body gravitational physics (Leapfrog KDK integrator)
 */
#pragma once
#include "common.h"

extern double g_sim_time;    /* total simulated seconds elapsed */
extern double g_sim_speed;   /* simulated seconds per real second */
extern int    g_paused;

/* Advance the simulation by dt seconds (SI units throughout) */
void physics_step(double dt);

/* Sample all body positions into their trail circular buffers */
void trails_sample(void);
