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

#define OUTER_PERIOD_DIVISOR 24.0
#define INNER_PERIOD_DIVISOR 96.0
#define OUTER_DT_MIN (DAY * 0.05)
#define INNER_DT_MIN 60.0
#define INNER_DT_MAX (DAY * 0.02)
#define OUTER_DT_DEFAULT DAY

static double s_outer_dt_limit = OUTER_DT_DEFAULT;
static double s_inner_dt_limit = INNER_DT_MAX;
static int    s_system_roots[MAX_BODIES];
static double s_system_outer_dt[MAX_BODIES];
static double s_system_inner_dt[MAX_BODIES];
static int    s_nsystems = 0;

static int in_system(int i, int root)
{
    return root < 0 || body_root_star(i) == root;
}

static int ensure_system_slot(int root)
{
    for (int i = 0; i < s_nsystems; i++)
        if (s_system_roots[i] == root) return i;
    if (s_nsystems >= MAX_BODIES) return -1;
    s_system_roots[s_nsystems] = root;
    s_system_outer_dt[s_nsystems] = OUTER_DT_DEFAULT;
    s_system_inner_dt[s_nsystems] = INNER_DT_MAX;
    return s_nsystems++;
}

static int timestep_anchor(int i)
{
    if (i < 0 || i >= g_nbodies || !g_bodies[i].alive) return -1;
    if (g_bodies[i].parent >= 0 && g_bodies[g_bodies[i].parent].alive)
        return g_bodies[i].parent;

    int best = -1;
    double best_score = 1e300;
    for (int j = 0; j < g_nbodies; j++) {
        if (j == i || !g_bodies[j].alive) continue;
        if (g_bodies[j].mass <= 0.0) continue;
        double dx = g_bodies[j].pos[0] - g_bodies[i].pos[0];
        double dy = g_bodies[j].pos[1] - g_bodies[i].pos[1];
        double dz = g_bodies[j].pos[2] - g_bodies[i].pos[2];
        double r2 = dx*dx + dy*dy + dz*dz;
        if (r2 <= 0.0) continue;
        double score = r2 / g_bodies[j].mass;
        if (score < best_score) {
            best_score = score;
            best = j;
        }
    }
    return best;
}

static double estimate_period_about(int i, int anchor)
{
    if (i < 0 || anchor < 0 || i >= g_nbodies || anchor >= g_nbodies) return 0.0;
    if (!g_bodies[i].alive || !g_bodies[anchor].alive) return 0.0;
    double dx = g_bodies[i].pos[0] - g_bodies[anchor].pos[0];
    double dy = g_bodies[i].pos[1] - g_bodies[anchor].pos[1];
    double dz = g_bodies[i].pos[2] - g_bodies[anchor].pos[2];
    double r = sqrt(dx*dx + dy*dy + dz*dz);
    double gm = G_CONST * (g_bodies[i].mass + g_bodies[anchor].mass);
    if (r <= 0.0 || gm <= 0.0) return 0.0;
    return 2.0 * PI * sqrt(r * r * r / gm);
}

/* ── helpers ─────────────────────────────────────────────────────────── */
/* A body is a satellite (moon) if it has a parent and that parent is not a star.
 * Stars   (parent=-1)          → false  — primary body
 * Planets (parent=star_idx)    → false  — primary body, fast forces not used
 * Moons   (parent=planet_idx)  → true   — handled by fast forces */
static int is_satellite(int i) {
    if (!g_bodies[i].alive) return 0;
    return g_bodies[i].parent >= 0 && !g_bodies[g_bodies[i].parent].is_star;
}

/* Any explicit parent-child pair is integrated at the inner cadence. */
static int has_fast_parent(int i) {
    return is_satellite(i);
}

void physics_refresh_timestep_model(void)
{
    double best_outer = OUTER_DT_DEFAULT;
    double best_inner = INNER_DT_MAX;
    s_nsystems = 0;

    for (int i = 0; i < g_nbodies; i++) {
        if (g_bodies[i].alive && g_bodies[i].is_star)
            ensure_system_slot(i);
    }

    for (int i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        b->dyn_period = 0.0;
        b->dyn_dt_outer = OUTER_DT_DEFAULT;
        b->dyn_dt_inner = INNER_DT_MAX;
        b->dyn_bucket = 0;

        if (!b->alive || b->is_star) continue;

        int anchor = timestep_anchor(i);
        double T = estimate_period_about(i, anchor);
        if (T <= 0.0) continue;

        double dt_outer = T / OUTER_PERIOD_DIVISOR;
        double dt_inner = T / INNER_PERIOD_DIVISOR;
        if (dt_outer < OUTER_DT_MIN) dt_outer = OUTER_DT_MIN;
        if (dt_outer > OUTER_DT_DEFAULT) dt_outer = OUTER_DT_DEFAULT;
        if (dt_inner < INNER_DT_MIN) dt_inner = INNER_DT_MIN;
        if (dt_inner > INNER_DT_MAX) dt_inner = INNER_DT_MAX;

        b->dyn_period = T;
        b->dyn_dt_outer = dt_outer;
        b->dyn_dt_inner = dt_inner;
        if (dt_inner <= 10.0 * 60.0) b->dyn_bucket = 3;
        else if (dt_inner <= 60.0 * 60.0) b->dyn_bucket = 2;
        else if (dt_inner <= 6.0 * 60.0 * 60.0) b->dyn_bucket = 1;

        {
            int root = body_root_star(i);
            int slot = ensure_system_slot(root);
            if (slot >= 0) {
                if (dt_outer < s_system_outer_dt[slot]) s_system_outer_dt[slot] = dt_outer;
                if (is_satellite(i) && dt_inner < s_system_inner_dt[slot]) s_system_inner_dt[slot] = dt_inner;
            }
        }

        if (dt_outer < best_outer) best_outer = dt_outer;
        if (is_satellite(i) && dt_inner < best_inner) best_inner = dt_inner;
    }

    s_outer_dt_limit = best_outer;
    s_inner_dt_limit = best_inner;
}

double physics_outer_dt_limit(void)
{
    return s_outer_dt_limit;
}

double physics_inner_dt_limit(void)
{
    return s_inner_dt_limit;
}

int physics_system_count(void)
{
    return s_nsystems;
}

int physics_system_root(int idx)
{
    if (idx < 0 || idx >= s_nsystems) return -1;
    return s_system_roots[idx];
}

double physics_system_outer_dt_limit(int idx)
{
    if (idx < 0 || idx >= s_nsystems) return OUTER_DT_DEFAULT;
    return s_system_outer_dt[idx];
}

double physics_system_inner_dt_limit(int idx)
{
    if (idx < 0 || idx >= s_nsystems) return INNER_DT_MAX;
    return s_system_inner_dt[idx];
}

void physics_advance_time(double dt)
{
    g_sim_time += dt;
}

static int is_ancestor_of(int ancestor, int child) {
    int p = g_bodies[child].parent;
    while (p >= 0) {
        if (p == ancestor) return 1;
        p = g_bodies[p].parent;
    }
    return 0;
}

/* ── slow forces: primary-primary + non-parent tidal on satellites ───── */
static void compute_acc_slow_system(int root) {
    int i, j;
    for (i = 0; i < g_nbodies; i++)
        if (in_system(i, root))
            g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;

    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        for (j = 0; j < g_nbodies; j++) {
            int same_system;
            double dx, dy, dz, r2, r, f;

            if (j == i || !g_bodies[j].alive) continue;

            same_system = in_system(j, root);
            if (same_system && j < i) continue;

            /* skip satellite-satellite (negligible and expensive) */
            if (is_satellite(i) && is_satellite(j)) continue;
            /* skip only true moon-parent chains here — planet-star stays slow */
            if (same_system &&
                ((is_satellite(j) && is_ancestor_of(i, j)) ||
                 (is_satellite(i) && is_ancestor_of(j, i)))) continue;

            dx = g_bodies[j].pos[0] - g_bodies[i].pos[0];
            dy = g_bodies[j].pos[1] - g_bodies[i].pos[1];
            dz = g_bodies[j].pos[2] - g_bodies[i].pos[2];
            r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
            if (G_CONST * g_bodies[j].mass / r2 < GRAV_EPSILON &&
                G_CONST * g_bodies[i].mass / r2 < GRAV_EPSILON) continue;
            r  = sqrt(r2);
            f  = G_CONST / (r2 * r);

            g_bodies[i].acc[0] += f * g_bodies[j].mass * dx;
            g_bodies[i].acc[1] += f * g_bodies[j].mass * dy;
            g_bodies[i].acc[2] += f * g_bodies[j].mass * dz;

            if (same_system) {
                g_bodies[j].acc[0] -= f * g_bodies[i].mass * dx;
                g_bodies[j].acc[1] -= f * g_bodies[i].mass * dy;
                g_bodies[j].acc[2] -= f * g_bodies[i].mass * dz;
            }
        }
    }
}

/* ── fast forces: dominant parent → satellite (+ Newton 3rd reaction) ── */
static void compute_acc_fast_system(int root) {
    int i;
    for (i = 0; i < g_nbodies; i++)
        if (in_system(i, root))
            g_bodies[i].fast_acc[0] = g_bodies[i].fast_acc[1] =
            g_bodies[i].fast_acc[2] = 0.0;

    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        if (!has_fast_parent(i)) continue;
        for (int p = g_bodies[i].parent; p >= 0; p = g_bodies[p].parent) {
            if (!g_bodies[p].alive || !in_system(p, root)) continue;
            double dx = g_bodies[p].pos[0] - g_bodies[i].pos[0];
            double dy = g_bodies[p].pos[1] - g_bodies[i].pos[1];
            double dz = g_bodies[p].pos[2] - g_bodies[i].pos[2];
            double r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
            if (G_CONST * g_bodies[p].mass / r2 < GRAV_EPSILON) continue;
            double r  = sqrt(r2);
            double f  = G_CONST / (r2 * r);

        /* satellite accelerated toward parent */
            g_bodies[i].fast_acc[0] += f * g_bodies[p].mass * dx;
            g_bodies[i].fast_acc[1] += f * g_bodies[p].mass * dy;
            g_bodies[i].fast_acc[2] += f * g_bodies[p].mass * dz;

        /* reaction on parent (Newton 3rd — small but correct) */
            g_bodies[p].fast_acc[0] -= f * g_bodies[i].mass * dx;
            g_bodies[p].fast_acc[1] -= f * g_bodies[i].mass * dy;
            g_bodies[p].fast_acc[2] -= f * g_bodies[i].mass * dz;
        }
    }
}

/* ── RESPA public API ────────────────────────────────────────────────── */

/*
 * physics_respa_begin — slow half-kick, then pre-compute fast forces
 * for the carry-over into the first inner step.
 */
void physics_respa_begin(double dt_outer) {
    physics_respa_begin_system(-1, dt_outer);
}

void physics_respa_begin_system(int root, double dt_outer) {
    int i;
    compute_acc_slow_system(root);
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
    }
    /* Pre-compute fast forces so physics_respa_inner can use them
     * immediately without an extra evaluation (carry-over trick). */
    compute_acc_fast_system(root);
}

/*
 * physics_respa_inner — one inner KDK step using fast forces only.
 * fast_acc must already be valid on entry (set by begin or previous inner).
 * Leaves fast_acc valid for the next call (carry-over).
 */
void physics_respa_inner(double dt_inner) {
    physics_respa_inner_system(-1, dt_inner);
}

void physics_respa_inner_system(int root, double dt_inner) {
    int i;
    /* fast half-kick (uses fast_acc from previous compute_acc_fast) */
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
    }
    /* drift all bodies */
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt_inner;
        g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt_inner;
        g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt_inner;
    }
    /* recompute fast forces at new positions, then fast half-kick */
    compute_acc_fast_system(root);
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
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
    physics_respa_end_system(-1, dt_outer);
    physics_advance_time(dt_outer);
}

void physics_respa_end_system(int root, double dt_outer) {
    int i;
    compute_acc_slow_system(root);
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
    }
    /* axial rotation */
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !in_system(i, root)) continue;
        g_bodies[i].rotation_angle = fmod(
            g_bodies[i].rotation_angle + g_bodies[i].rotation_rate * dt_outer,
            2.0 * PI);
    }
}

/* ── legacy single-step KDK (not used in main loop) ─────────────────── */
static void compute_acc(void) {
    int i, j;
    for (i = 0; i < g_nbodies; i++)
        g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        for (j = i + 1; j < g_nbodies; j++) {
            if (!g_bodies[j].alive) continue;
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
        if (!g_bodies[i].alive) continue;
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt;
        g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt;
        g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt;
    }
    compute_acc();
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
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
    trails_tick_system(-1, dt);
}

void trails_tick_system(int root, double dt) {
    int i;
    for (i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        if (!b->alive || !in_system(i, root) || !b->trail || b->trail_interval <= 0.0) continue;
        b->trail_accum += dt;
        while (b->trail_accum >= b->trail_interval) {
            b->trail_accum -= b->trail_interval;
            sample_body(b);
        }
    }
}
