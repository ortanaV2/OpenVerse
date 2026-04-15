/*
 * physics.c — N-body gravitational physics
 *
 * Integrator: Leapfrog KDK (kick-drift-kick).
 * State: positions in metres, velocities in m/s, G in SI.
 */
#include "physics.h"
#include "body.h"
#include <math.h>

double g_sim_time  = 0.0;
double g_sim_speed = DAY;    /* default: 1 simulated day per real second */
int    g_paused    = 0;

/* ------------------------------------------------------------------ internal */
static void compute_acc(void) {
    int i, j;
    for (i = 0; i < g_nbodies; i++)
        g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;

    for (i = 0; i < g_nbodies; i++) {
        for (j = i + 1; j < g_nbodies; j++) {
            double dx = g_bodies[j].pos[0] - g_bodies[i].pos[0];
            double dy = g_bodies[j].pos[1] - g_bodies[i].pos[1];
            double dz = g_bodies[j].pos[2] - g_bodies[i].pos[2];
            double r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
            double r  = sqrt(r2);
            double f  = G_CONST / (r2 * r);          /* G / r^3 */

            double ai = f * g_bodies[j].mass;
            double aj = f * g_bodies[i].mass;

            g_bodies[i].acc[0] += ai * dx;
            g_bodies[i].acc[1] += ai * dy;
            g_bodies[i].acc[2] += ai * dz;
            g_bodies[j].acc[0] -= aj * dx;
            g_bodies[j].acc[1] -= aj * dy;
            g_bodies[j].acc[2] -= aj * dz;
        }
    }
}

/* ------------------------------------------------------------------ public */
void physics_step(double dt) {
    int i;
    /* KDK: compute acc at t */
    compute_acc();

    /* Half-kick: v(t) → v(t+dt/2) */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    /* Drift: x(t) → x(t+dt) */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt;
        g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt;
        g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt;
    }
    /* Acc at new positions */
    compute_acc();
    /* Half-kick: v(t+dt/2) → v(t+dt) */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    g_sim_time += dt;
}

void trails_sample(void) {
    int i;
    for (i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        b->trail[b->trail_head][0] = (float)(b->pos[0] * RS);
        b->trail[b->trail_head][1] = (float)(b->pos[1] * RS);
        b->trail[b->trail_head][2] = (float)(b->pos[2] * RS);
        b->trail_head = (b->trail_head + 1) % TRAIL_LEN;
        if (b->trail_count < TRAIL_LEN) b->trail_count++;
    }
}
