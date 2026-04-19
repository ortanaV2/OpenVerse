/*
 * physics.c — N-body gravitational physics (RESPA hierarchical integrator)
 *
 * 2R-RESPA split:
 *   Slow forces: primary-primary pairs + non-parent-primary → satellite
 *                (338 pairs for 14 primaries + 19 moons)
 *   Fast forces: parent → satellite dominant force (19 pairs)
 *
 * Each outer step costs  2 × 338  slow pair evaluations.
 * Each inner step costs        19  fast pair evaluations.
 * At 3650 days/s (59 outer × 50 inner):
 *   New:  59×2×338 + 59×51×19  ≈  97 000  pair evaluations / frame
 *   Old:  2920 × 2 × 528       ≈  3 083 000 pair evaluations / frame
 *   Speedup: ~32×
 */
#include "physics.h"
#include "body.h"
#include <math.h>

double g_sim_time  = 0.0;
double g_sim_speed = DAY;
int    g_paused    = 0;

/* ── helpers ─────────────────────────────────────────────────────────── */
/* A body is a satellite (moon) if it has a parent and that parent is not a star.
 * Stars   (parent=-1)          → false  — primary body
 * Planets (parent=star_idx)    → false  — primary body, fast forces not used
 * Moons   (parent=planet_idx)  → true   — handled by fast forces */
static int is_satellite(int i) {
    return g_bodies[i].parent >= 0 && !g_bodies[g_bodies[i].parent].is_star;
}

/* ── slow forces: primary-primary + non-parent tidal on satellites ───── */
static void compute_acc_slow(void) {
    int i, j;
    for (i = 0; i < g_nbodies; i++)
        g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;

    for (i = 0; i < g_nbodies; i++) {
        for (j = i + 1; j < g_nbodies; j++) {
            /* skip satellite-satellite (negligible and expensive) */
            if (is_satellite(i) && is_satellite(j)) continue;
            /* skip parent-satellite pair — handled by fast forces */
            if (is_satellite(j) && g_bodies[j].parent == i) continue;
            if (is_satellite(i) && g_bodies[i].parent == j) continue;

            double dx = g_bodies[j].pos[0] - g_bodies[i].pos[0];
            double dy = g_bodies[j].pos[1] - g_bodies[i].pos[1];
            double dz = g_bodies[j].pos[2] - g_bodies[i].pos[2];
            double r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
            /* Skip negligible cross-system pairs (e.g. Sol's gravity on Alpha Cen planets) */
            if (G_CONST * g_bodies[j].mass / r2 < GRAV_EPSILON &&
                G_CONST * g_bodies[i].mass / r2 < GRAV_EPSILON) continue;
            double r  = sqrt(r2);
            double f  = G_CONST / (r2 * r);

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

/* ── fast forces: dominant parent → satellite (+ Newton 3rd reaction) ── */
static void compute_acc_fast(void) {
    int i;
    for (i = 0; i < g_nbodies; i++)
        g_bodies[i].fast_acc[0] = g_bodies[i].fast_acc[1] =
        g_bodies[i].fast_acc[2] = 0.0;

    for (i = 0; i < g_nbodies; i++) {
        if (!is_satellite(i)) continue;
        int p = g_bodies[i].parent;

        double dx = g_bodies[p].pos[0] - g_bodies[i].pos[0];
        double dy = g_bodies[p].pos[1] - g_bodies[i].pos[1];
        double dz = g_bodies[p].pos[2] - g_bodies[i].pos[2];
        double r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
        if (G_CONST * g_bodies[p].mass / r2 < GRAV_EPSILON) continue;
        double r  = sqrt(r2);
        double f  = G_CONST / (r2 * r);

        /* satellite accelerated toward parent */
        g_bodies[i].fast_acc[0] = f * g_bodies[p].mass * dx;
        g_bodies[i].fast_acc[1] = f * g_bodies[p].mass * dy;
        g_bodies[i].fast_acc[2] = f * g_bodies[p].mass * dz;

        /* reaction on parent (Newton 3rd — small but correct) */
        g_bodies[p].fast_acc[0] -= f * g_bodies[i].mass * dx;
        g_bodies[p].fast_acc[1] -= f * g_bodies[i].mass * dy;
        g_bodies[p].fast_acc[2] -= f * g_bodies[i].mass * dz;
    }
}

/* ── RESPA public API ────────────────────────────────────────────────── */

/*
 * physics_respa_begin — slow half-kick, then pre-compute fast forces
 * for the carry-over into the first inner step.
 */
void physics_respa_begin(double dt_outer) {
    int i;
    compute_acc_slow();
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
    }
    /* Pre-compute fast forces so physics_respa_inner can use them
     * immediately without an extra evaluation (carry-over trick). */
    compute_acc_fast();
}

/*
 * physics_respa_inner — one inner KDK step using fast forces only.
 * fast_acc must already be valid on entry (set by begin or previous inner).
 * Leaves fast_acc valid for the next call (carry-over).
 */
void physics_respa_inner(double dt_inner) {
    int i;
    /* fast half-kick (uses fast_acc from previous compute_acc_fast) */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
    }
    /* drift all bodies */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt_inner;
        g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt_inner;
        g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt_inner;
    }
    /* recompute fast forces at new positions, then fast half-kick */
    compute_acc_fast();
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
    }
}

/*
 * physics_respa_end — slow half-kick at final position, rotation update,
 * sim-time advance.
 */
void physics_respa_end(double dt_outer) {
    int i;
    compute_acc_slow();
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
    }
    /* axial rotation */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].rotation_angle = fmod(
            g_bodies[i].rotation_angle + g_bodies[i].rotation_rate * dt_outer,
            2.0 * PI);
    }
    g_sim_time += dt_outer;
}

/* ── legacy single-step KDK (not used in main loop) ─────────────────── */
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
            double f  = G_CONST / (r2 * r);
            double ai = f * g_bodies[j].mass;
            double aj = f * g_bodies[i].mass;
            g_bodies[i].acc[0] += ai * dx; g_bodies[i].acc[1] += ai * dy; g_bodies[i].acc[2] += ai * dz;
            g_bodies[j].acc[0] -= aj * dx; g_bodies[j].acc[1] -= aj * dy; g_bodies[j].acc[2] -= aj * dz;
        }
    }
}

void physics_step(double dt) {
    int i;
    compute_acc();
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt;
        g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt;
        g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt;
    }
    compute_acc();
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].rotation_angle = fmod(
            g_bodies[i].rotation_angle + g_bodies[i].rotation_rate * dt,
            2.0 * PI);
    }
    g_sim_time += dt;
}

/* ── trail helpers ───────────────────────────────────────────────────── */
static void sample_body(Body *b) {
    b->trail[b->trail_head][0] = b->pos[0] * RS;
    b->trail[b->trail_head][1] = b->pos[1] * RS;
    b->trail[b->trail_head][2] = b->pos[2] * RS;
    b->trail_head = (b->trail_head + 1) % TRAIL_LEN;
    if (b->trail_count < TRAIL_LEN) b->trail_count++;
}

/*
 * trails_tick — time-based trail sampling.
 *
 * Each body accumulates sim-time; once trail_interval is exceeded a sample
 * is recorded.  The while-loop handles multiple samples per tick (fast
 * moons or high sim speeds).  trail_interval ≈ T/200 gives ~200 evenly
 * spaced samples per orbit regardless of sim speed — the trail fills the
 * buffer faster at high speeds, naturally showing more orbital history.
 */
void trails_tick(double dt) {
    int i;
    for (i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        if (!b->trail || b->trail_interval <= 0.0) continue;
        b->trail_accum += dt;
        while (b->trail_accum >= b->trail_interval) {
            b->trail_accum -= b->trail_interval;
            sample_body(b);
        }
    }
}
