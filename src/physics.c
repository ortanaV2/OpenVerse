/*
 * physics.c — N-body gravitational physics and trail system
 *
 * ── Integrator: 2R-RESPA (Reference System Propagator Algorithm) ──────────
 *
 * Forces are split into two classes by timescale:
 *
 *   Slow forces (outer timestep dt_outer):
 *     All primary–primary pairs (star↔planet, star↔star, planet↔planet).
 *     Tidal/perturbation forces on moons from non-parent bodies.
 *     These change slowly compared to the dominant moon–parent orbit.
 *
 *   Fast forces (inner timestep dt_inner, dt_inner << dt_outer):
 *     Dominant parent → satellite force (planet → moon).
 *     Newton 3rd reaction back onto the parent.
 *     These must be integrated at the short orbital period of the moon.
 *
 * Each outer step is a KDK (Kick-Drift-Kick) leapfrog over slow forces,
 * with the "drift" replaced by N inner KDK steps over fast forces:
 *
 *   physics_respa_begin(dt_outer)        — slow half-kick  (outer K/2)
 *     for k in range(N):
 *       physics_respa_inner(dt_inner)    — fast KDK        (inner K-D-K)
 *   physics_respa_end(dt_outer)          — slow half-kick  (outer K/2)
 *
 * This structure is symplectic (area-preserving in phase space) and
 * exactly conserves a modified Hamiltonian.  Long-term orbital energy
 * drift is dramatically lower than a naive integrator at the same cost.
 *
 * ── Timestep model ───────────────────────────────────────────────────────
 *
 * For each non-star body, the orbital period T is estimated from the
 * two-body formula T = 2π√(r³/GM) where r is the distance to its
 * timestep anchor (parent if explicit, otherwise the most gravitationally
 * dominant body by r²/M score).
 *
 *   dt_outer = T / OUTER_PERIOD_DIVISOR   (clamped to [OUTER_DT_MIN, OUTER_DT_DEFAULT])
 *   dt_inner = T / INNER_PERIOD_DIVISOR   (clamped to [INNER_DT_MIN, INNER_DT_MAX])
 *
 * For highly-perturbed moons (satellite_perturbation_ratio() > threshold),
 * dt_outer is tightened toward dt_inner to prevent numerical collapse when
 * the "slow" perturbation is no longer actually slow.
 *
 * Per-system timestep slots store the tightest dt among all bodies in that
 * system, so main.c can schedule systems independently without scanning
 * all bodies to find the limiting timestep each frame.
 *
 * ── Performance notes ────────────────────────────────────────────────────
 *
 * Precomputed s_system_members[slot][] lists replace O(g_nbodies) scans in
 * the force kernels with O(members) iterations — critical since force
 * evaluation is the inner loop of the simulation.
 *
 * Satellite–satellite forces are skipped (moon masses << planet masses;
 * the mutual acceleration is below GRAV_EPSILON).
 *
 * GRAV_EPSILON early-exit: pairs whose gravitational acceleration is below
 * this threshold from both sides are skipped entirely.
 *
 * Speedup vs naive (14 primaries + 19 moons at 3650 days/s sim speed):
 *   RESPA: 59×2×338 + 59×51×19 ≈  97 000 pair evaluations/frame
 *   Naive: 2920 × 2 × 528      ≈ 3 083 000 pair evaluations/frame  (~32×)
 */
#include "physics.h"
#include "body.h"
#include "collision.h"
#include <math.h>

double g_sim_time  = 0.0;
double g_sim_speed = DAY;
int    g_paused    = 0;

/* ── timestep constants ─────────────────────────────────────────────────── */

#define OUTER_PERIOD_DIVISOR 24.0
#define INNER_PERIOD_DIVISOR 96.0
/* OUTER_DT_MIN: prevents a single very-short-period moon from throttling every
 * system in the scene by imposing a global floor on the outer timestep. */
#define OUTER_DT_MIN (DAY * 0.05)
#define INNER_DT_MIN 60.0          /* 1-minute floor; below this physics diverges */
#define INNER_DT_MAX (DAY * 0.02)  /* ~29 min ceiling; sufficient for all planets */
#define OUTER_DT_DEFAULT DAY

/* ── per-system timestep state (computed by physics_refresh_timestep_model) */

static double s_outer_dt_limit = OUTER_DT_DEFAULT;
static double s_inner_dt_limit = INNER_DT_MAX;

/* s_system_roots[slot]        — body index of the root star for this slot
 * s_system_outer/inner_dt[slot] — tightest timestep across all members
 * s_root_to_slot[body_idx]    — inverse map: root star index → slot
 * s_body_system_slot[body_idx]  — which slot does this body belong to
 * s_system_members[slot][k]   — precomputed list of body indices in slot
 * s_system_member_count[slot] — length of the above list */
static int    s_system_roots[MAX_BODIES];
static double s_system_outer_dt[MAX_BODIES];
static double s_system_inner_dt[MAX_BODIES];
static int    s_root_to_slot[MAX_BODIES];
static int    s_body_system_slot[MAX_BODIES];
static int    s_system_members[MAX_BODIES][MAX_BODIES];
static int    s_system_member_count[MAX_BODIES];
static int    s_nsystems = 0;

/* Forward declarations for helpers used before their definition */
static int is_satellite(int i);
static int is_ancestor_of(int ancestor, int child);

/* ── system membership helpers ──────────────────────────────────────────── */

/* Return 1 if body i belongs to the star system rooted at `root`.
 * root < 0 is a wildcard — matches every body (used by non-system code paths). */
static int in_system(int i, int root)
{
    int slot;

    if (root < 0) return 1;
    if (i < 0 || i >= g_nbodies || i >= MAX_BODIES) return 0;
    if (root >= MAX_BODIES) return 0;
    slot = s_root_to_slot[root];
    return slot >= 0 && s_body_system_slot[i] == slot;
}

/* Lazily allocate a system slot for star `root`.
 * Returns the slot index (≥0), or -1 if root is out of range or all slots used. */
static int ensure_system_slot(int root)
{
    if (root < 0 || root >= MAX_BODIES) return -1;
    if (s_root_to_slot[root] >= 0) return s_root_to_slot[root];
    for (int i = 0; i < s_nsystems; i++)
        if (s_system_roots[i] == root) return i;
    if (s_nsystems >= MAX_BODIES) return -1;
    s_system_roots[s_nsystems] = root;
    s_system_outer_dt[s_nsystems] = OUTER_DT_DEFAULT;
    s_system_inner_dt[s_nsystems] = INNER_DT_MAX;
    s_system_member_count[s_nsystems] = 0;
    s_root_to_slot[root] = s_nsystems;
    return s_nsystems++;
}

/* ── timestep estimation helpers ────────────────────────────────────────── */

/*
 * timestep_anchor — find the reference body for period estimation.
 *
 * If body i has an explicit parent, the parent is the dominant gravitational
 * influence and is returned directly.  For orphan bodies (parent < 0), we
 * score every other body by r²/M and return the one with the smallest score
 * (= highest acceleration GM/r²).
 */
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
        double score = r2 / g_bodies[j].mass;  /* ∝ 1 / acceleration */
        if (score < best_score) {
            best_score = score;
            best = j;
        }
    }
    return best;
}

/* Estimate Keplerian orbital period of body i around `anchor` using the
 * vis-viva / circular approximation: T = 2π√(r³ / GM_total). */
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

/*
 * satellite_perturbation_ratio — ratio of max third-body acceleration to
 * dominant parent acceleration for satellite body i.
 *
 * Returns acc_max_other / acc_parent.  Values above ~0.01 indicate that the
 * "slow" perturbation forces are not negligible compared to the fast force,
 * which can cause orbit numerical instability if the outer timestep is too
 * coarse relative to the inner one.
 *
 * Used by physics_refresh_timestep_model() to tighten dt_outer for highly-
 * perturbed moons (e.g. outer moons near the Hill sphere of their planet).
 */
static double satellite_perturbation_ratio(int i)
{
    int parent;
    double pdx, pdy, pdz, pr2, parent_acc;
    double max_other_acc = 0.0;

    if (i < 0 || i >= g_nbodies || !is_satellite(i)) return 0.0;
    parent = g_bodies[i].parent;
    if (parent < 0 || parent >= g_nbodies || !g_bodies[parent].alive) return 0.0;

    pdx = g_bodies[parent].pos[0] - g_bodies[i].pos[0];
    pdy = g_bodies[parent].pos[1] - g_bodies[i].pos[1];
    pdz = g_bodies[parent].pos[2] - g_bodies[i].pos[2];
    pr2 = pdx*pdx + pdy*pdy + pdz*pdz + SOFTENING*SOFTENING;
    if (pr2 <= 0.0) return 0.0;
    parent_acc = G_CONST * g_bodies[parent].mass / pr2;
    if (parent_acc <= 0.0) return 0.0;

    for (int j = 0; j < g_nbodies; j++) {
        double dx, dy, dz, r2, acc;
        if (j == i || j == parent || !g_bodies[j].alive) continue;
        if (is_ancestor_of(i, j)) continue;   /* skip own children */

        dx = g_bodies[j].pos[0] - g_bodies[i].pos[0];
        dy = g_bodies[j].pos[1] - g_bodies[i].pos[1];
        dz = g_bodies[j].pos[2] - g_bodies[i].pos[2];
        r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
        if (r2 <= 0.0) continue;
        acc = G_CONST * g_bodies[j].mass / r2;
        if (acc > max_other_acc) max_other_acc = acc;
    }

    return max_other_acc / parent_acc;
}

/* ── force classification helpers ───────────────────────────────────────── */

/*
 * is_satellite — true if body i is a moon (has a non-star parent).
 *   Stars   (parent=-1)            → false  — primary body
 *   Planets (parent=star_idx)      → false  — primary body; slow forces only
 *   Moons   (parent=planet_idx)    → true   — integrated with fast forces
 */
static int is_satellite(int i) {
    if (!g_bodies[i].alive) return 0;
    return g_bodies[i].parent >= 0 && !g_bodies[g_bodies[i].parent].is_star;
}


/* ── timestep model (re)build ───────────────────────────────────────────── */

/*
 * physics_refresh_timestep_model — rebuild all per-system and per-body
 * timestep data.  Must be called:
 *   - After universe_load() to set initial timesteps.
 *   - After any body is added or removed (build mode, collision merge).
 *
 * Steps:
 *   1. Clear and rebuild s_system_* arrays and s_root_to_slot[]/s_body_system_slot[].
 *   2. For each non-star body: estimate orbital period, compute dt_outer/dt_inner.
 *   3. For moons: compute perturbation ratio and tighten dt_outer if needed.
 *   4. Accumulate per-system and global dt limits.
 */
void physics_refresh_timestep_model(void)
{
    double best_outer = OUTER_DT_DEFAULT;
    double best_inner = INNER_DT_MAX;
    s_nsystems = 0;

    for (int i = 0; i < MAX_BODIES; i++) {
        s_root_to_slot[i] = -1;
        s_body_system_slot[i] = -1;
        s_system_member_count[i] = 0;
    }

    /* Pre-create a slot for every live star so s_root_to_slot is populated
     * before the member assignment loop below. */
    for (int i = 0; i < g_nbodies; i++) {
        if (g_bodies[i].alive && g_bodies[i].is_star)
            ensure_system_slot(i);
    }

    /* Assign every live body to its root star's slot */
    for (int i = 0; i < g_nbodies && i < MAX_BODIES; i++) {
        int root, slot;

        if (!g_bodies[i].alive) continue;
        root = body_root_star(i);
        slot = ensure_system_slot(root);
        if (slot < 0) continue;

        s_body_system_slot[i] = slot;
        if (s_system_member_count[slot] < MAX_BODIES)
            s_system_members[slot][s_system_member_count[slot]++] = i;
    }

    /* Compute per-body timesteps */
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
        double perturb = 0.0;
        if (dt_outer < OUTER_DT_MIN) dt_outer = OUTER_DT_MIN;
        if (dt_outer > OUTER_DT_DEFAULT) dt_outer = OUTER_DT_DEFAULT;
        if (dt_inner < INNER_DT_MIN) dt_inner = INNER_DT_MIN;
        if (dt_inner > INNER_DT_MAX) dt_inner = INNER_DT_MAX;

        if (is_satellite(i)) {
            perturb = satellite_perturbation_ratio(i);

            /* Tighten dt_outer toward dt_inner when third-body acceleration
             * is a significant fraction of the parent acceleration.  Without
             * this, the slow-force integrator becomes inaccurate enough that
             * it can numerically eject weakly-bound outer moons. */
            if (perturb > 0.20) dt_outer = fmin(dt_outer, dt_inner * 1.5);
            else if (perturb > 0.10) dt_outer = fmin(dt_outer, dt_inner * 2.0);
            else if (perturb > 0.05) dt_outer = fmin(dt_outer, dt_inner * 3.0);
            else if (perturb > 0.02) dt_outer = fmin(dt_outer, dt_inner * 4.0);
            else if (perturb > 0.01) dt_outer = fmin(dt_outer, dt_inner * 6.0);

            if (dt_outer < OUTER_DT_MIN) dt_outer = OUTER_DT_MIN;
        }

        b->dyn_period = T;
        b->dyn_dt_outer = dt_outer;
        b->dyn_dt_inner = dt_inner;
        /* Priority bucket for close-approach subdivision in main.c:
         * higher bucket = shorter period = more frequent trail sampling needed */
        if (dt_inner <= 10.0 * 60.0) b->dyn_bucket = 3;
        else if (dt_inner <= 60.0 * 60.0) b->dyn_bucket = 2;
        else if (dt_inner <= 6.0 * 60.0 * 60.0) b->dyn_bucket = 1;

        /* Accumulate per-system and global limits */
        if (i < MAX_BODIES) {
            int slot = s_body_system_slot[i];
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

/* Accessor functions for the main loop to query computed timestep limits */
double physics_outer_dt_limit(void)  { return s_outer_dt_limit; }
double physics_inner_dt_limit(void)  { return s_inner_dt_limit; }
int    physics_system_count(void)    { return s_nsystems; }

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

void physics_advance_time(double dt) { g_sim_time += dt; }

/* Walk the parent chain from child upward; return 1 if ancestor appears. */
static int is_ancestor_of(int ancestor, int child) {
    int p = g_bodies[child].parent;
    while (p >= 0) {
        if (p == ancestor) return 1;
        p = g_bodies[p].parent;
    }
    return 0;
}

/* ── force kernels ──────────────────────────────────────────────────────── */

/*
 * compute_acc_slow_system — accumulate slow gravitational accelerations.
 *
 * Computes all-pairs force for the given star system, with two exclusions:
 *   1. Satellite–satellite pairs: skipped (moon masses are negligible;
 *      the mutual force is below GRAV_EPSILON in all realistic systems).
 *   2. True parent–child chains (moon↔parent): skipped because these are
 *      handled by compute_acc_fast_system at the inner cadence.
 *      Planet↔star pairs are NOT excluded here — planets are primaries.
 *
 * Newton's 3rd law: same_system pairs update both i and j in one evaluation,
 * halving the number of pair evaluations (j < i guard on same-system pairs).
 * Cross-system pairs (j in a different system) update only i, since j will
 * get its contribution when the other system processes the same pair.
 *
 * SOFTENING prevents singularities at very small separations.
 * GRAV_EPSILON early-exit skips pairs too weak to matter this step.
 *
 * root < 0 selects the legacy all-bodies path (used during warmup).
 */
static void compute_acc_slow_system(int root) {
    int i, j;
    int slot = (root >= 0 && root < MAX_BODIES) ? s_root_to_slot[root] : -1;

    /* Zero accumulators */
    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++)
            if (in_system(i, root))
                g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;
    } else {
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;
        }
    }

    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            for (j = 0; j < g_nbodies; j++) {
                int same_system;
                double dx, dy, dz, r2, r, f;

                if (j == i || !g_bodies[j].alive) continue;

                same_system = in_system(j, root);
                if (same_system && j < i) continue;   /* Newton 3rd: avoid double-count */

                if (is_satellite(i) && is_satellite(j)) continue;
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
                f  = G_CONST / (r2 * r);   /* f = G / r³; acceleration = f × M × r̂ */

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
        return;
    }

    /* System-aware path: iterate member list instead of all bodies */
    for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
        i = s_system_members[slot][mi];
        if (!g_bodies[i].alive) continue;
        for (j = 0; j < g_nbodies; j++) {
            int same_system;
            double dx, dy, dz, r2, r, f;

            if (j == i || !g_bodies[j].alive) continue;

            same_system = (j >= 0 && j < MAX_BODIES && s_body_system_slot[j] == slot);
            if (same_system && j < i) continue;

            if (is_satellite(i) && is_satellite(j)) continue;
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

/*
 * compute_acc_fast_system — accumulate fast (parent→satellite) accelerations.
 *
 * For each moon, walk the parent chain and accumulate the gravitational
 * acceleration from each ancestor.  Newton's 3rd law reaction is applied back
 * to each ancestor (the reaction is tiny but keeps total momentum exact).
 *
 * Only bodies satisfying is_satellite() (moons) are processed here.
 * Stores results in Body.fast_acc[], separate from Body.acc[] (slow forces),
 * so both can be combined independently at the KDK half-kick stages.
 */
static void compute_acc_fast_system(int root) {
    int i;
    int slot = (root >= 0 && root < MAX_BODIES) ? s_root_to_slot[root] : -1;

    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++)
            if (in_system(i, root))
                g_bodies[i].fast_acc[0] = g_bodies[i].fast_acc[1] =
                g_bodies[i].fast_acc[2] = 0.0;

        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            if (!is_satellite(i)) continue;
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

                /* Newton 3rd reaction onto parent (small but exact) */
                g_bodies[p].fast_acc[0] -= f * g_bodies[i].mass * dx;
                g_bodies[p].fast_acc[1] -= f * g_bodies[i].mass * dy;
                g_bodies[p].fast_acc[2] -= f * g_bodies[i].mass * dz;
            }
        }
        return;
    }

    for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
        i = s_system_members[slot][mi];
        g_bodies[i].fast_acc[0] = g_bodies[i].fast_acc[1] =
        g_bodies[i].fast_acc[2] = 0.0;
    }

    for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
        i = s_system_members[slot][mi];
        if (!g_bodies[i].alive) continue;
        if (!is_satellite(i)) continue;
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

            /* Newton 3rd reaction onto parent (small but exact) */
            g_bodies[p].fast_acc[0] -= f * g_bodies[i].mass * dx;
            g_bodies[p].fast_acc[1] -= f * g_bodies[i].mass * dy;
            g_bodies[p].fast_acc[2] -= f * g_bodies[i].mass * dz;
        }
    }
}

/* ── RESPA public API ────────────────────────────────────────────────────── */

/*
 * physics_respa_begin — outer slow half-kick (K/2 of the outer KDK).
 *
 * 1. Compute slow (primary–primary + tidal) forces at current positions.
 * 2. Apply half-kick: vel += ½ × acc_slow × dt_outer.
 * 3. Pre-compute fast forces so physics_respa_inner() can use them
 *    immediately without an extra evaluation (carry-over optimisation).
 *
 * The carry-over trick: the fast_acc[] computed here is valid at the same
 * positions as the end of the previous outer step (or t=0 on first call),
 * which is exactly where the first inner step begins.
 */
void physics_respa_begin(double dt_outer) {
    physics_respa_begin_system(-1, dt_outer);
}

void physics_respa_begin_system(int root, double dt_outer) {
    int i;
    int slot = (root >= 0 && root < MAX_BODIES) ? s_root_to_slot[root] : -1;
    compute_acc_slow_system(root);
    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
        }
    } else {
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            if (!g_bodies[i].alive) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
        }
    }
    /* Pre-compute fast forces; physics_respa_inner uses them immediately */
    compute_acc_fast_system(root);
}

/*
 * physics_respa_inner — one inner KDK step using fast forces only.
 *
 * fast_acc[] must be valid on entry (set by respa_begin or a previous inner
 * step).  The KDK structure inside each inner step is:
 *   vel += ½ × fast_acc × dt_inner     (fast half-kick using old forces)
 *   pos += vel × dt_inner               (drift at updated velocity)
 *   recompute fast_acc at new positions
 *   vel += ½ × fast_acc × dt_inner     (fast half-kick using new forces)
 *
 * This leaves fast_acc[] valid for the next inner step (carry-over).
 */
void physics_respa_inner(double dt_inner) {
    physics_respa_inner_system(-1, dt_inner);
}

void physics_respa_inner_system(int root, double dt_inner) {
    int i;
    int slot = (root >= 0 && root < MAX_BODIES) ? s_root_to_slot[root] : -1;

    /* Fast half-kick (uses fast_acc from previous evaluation) */
    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
        }
        /* Drift */
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt_inner;
            g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt_inner;
            g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt_inner;
        }
    } else {
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            if (!g_bodies[i].alive) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
        }
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            if (!g_bodies[i].alive) continue;
            g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt_inner;
            g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt_inner;
            g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt_inner;
        }
    }

    /* Recompute fast forces at new positions, then second fast half-kick */
    compute_acc_fast_system(root);
    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
        }
    } else {
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            if (!g_bodies[i].alive) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].fast_acc[0] * dt_inner;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].fast_acc[1] * dt_inner;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].fast_acc[2] * dt_inner;
        }
    }
}

/*
 * physics_respa_end — outer slow half-kick at final positions + rotation.
 *
 * Closes the outer KDK leapfrog:
 *   1. Recompute slow forces at the positions after all inner steps.
 *   2. Apply outer slow half-kick: vel += ½ × acc_slow × dt_outer.
 *   3. Advance axial rotation: rotation_angle += rotation_rate × dt_outer.
 * Rotation is updated at the outer cadence — angular resolution is sufficient
 * since rotation periods are hours-to-days, far longer than dt_outer.
 */
void physics_respa_end(double dt_outer) {
    physics_respa_end_system(-1, dt_outer);
    physics_advance_time(dt_outer);
}

void physics_respa_end_system(int root, double dt_outer) {
    int i;
    int slot = (root >= 0 && root < MAX_BODIES) ? s_root_to_slot[root] : -1;
    compute_acc_slow_system(root);
    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
        }
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive || !in_system(i, root)) continue;
            g_bodies[i].rotation_angle = fmod(
                g_bodies[i].rotation_angle + g_bodies[i].rotation_rate * dt_outer,
                2.0 * PI);
            g_bodies[i].cloud_rotation += g_bodies[i].rotation_rate * 1.15 * dt_outer;
        }
    } else {
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            if (!g_bodies[i].alive) continue;
            g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt_outer;
            g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt_outer;
            g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt_outer;
        }
        for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
            i = s_system_members[slot][mi];
            if (!g_bodies[i].alive) continue;
            g_bodies[i].rotation_angle = fmod(
                g_bodies[i].rotation_angle + g_bodies[i].rotation_rate * dt_outer,
                2.0 * PI);
            g_bodies[i].cloud_rotation += g_bodies[i].rotation_rate * 1.15 * dt_outer;
        }
    }
}

/* ── trail system ────────────────────────────────────────────────────────── */
/*
 * Trail overview:
 *
 * Each body has a circular buffer of world-space positions sampled at equal
 * arc-length intervals (not equal time intervals).  This gives uniform visual
 * density along the trail regardless of orbital speed variations.
 *
 * Positions are stored in render units (metres × RS = AU-units) to match the
 * coordinate system expected by trails_render().
 *
 * Between physics steps, the body's continuous trajectory is approximated by
 * a cubic Hermite spline through (prev_pos, prev_vel) → (pos, vel).  The
 * spline is adaptively subdivided and walked at the desired arc-length
 * interval to emit samples into the circular buffer.
 *
 * Hermite spline basis functions (t ∈ [0,1]):
 *   h00 =  2t³ − 3t² + 1    (start point weight)
 *   h10 =   t³ − 2t² + t    (start tangent weight; scaled by dt)
 *   h01 = −2t³ + 3t²        (end point weight)
 *   h11 =   t³ − t²         (end tangent weight; scaled by dt)
 *   p(t) = h00·p0 + h10·dt·v0 + h01·p1 + h11·dt·v1
 *
 * Adaptive subdivision (trail_curve_flatten_recursive):
 *   Compute the midpoint of the curve (at t=0.5) and the midpoint of the
 *   chord (p0+p1)/2.  If their distance exceeds max_err, subdivide into
 *   two half-intervals and recurse.  Stops at TRAIL_CURVE_MAX_DEPTH or
 *   when the curve is flat enough.  The result is a polyline approximating
 *   the Hermite curve within the specified error tolerance.
 */

/*
 * sample_body_pos — push one world-space position into the trail buffer.
 *
 * Positions are stored in render units (pos × RS).  When the buffer is full,
 * the oldest sample is evicted and its arc-length contribution is removed from
 * trail_total_len.  A second pruning loop then evicts additional old samples
 * if the total trail length exceeds target_world_len (in metres), keeping the
 * visible trail within a fixed physical length rather than a fixed count.
 *
 * Satellites (moons) use TRAIL_SATELLITE_WORLD_LEN instead of
 * TRAIL_TARGET_WORLD_LEN because their orbits are much shorter than planets'.
 */
static void sample_body_pos(Body *b, const double pos[3]) {
    int write_idx = b->trail_head;
    double seg_len = 0.0;
    double target_world_len = TRAIL_TARGET_WORLD_LEN;
    int body_idx = (int)(b - g_bodies);

    if (body_idx >= 0 && body_idx < g_nbodies && is_satellite(body_idx))
        target_world_len = TRAIL_SATELLITE_WORLD_LEN;

    if (b->trail_count > 0) {
        int prev_idx = (b->trail_head - 1 + TRAIL_LEN) & TRAIL_MASK;
        double dx = pos[0] * RS - b->trail[prev_idx][0];
        double dy = pos[1] * RS - b->trail[prev_idx][1];
        double dz = pos[2] * RS - b->trail[prev_idx][2];
        /* RS converts metres → AU-units; multiply back by AU for metres */
        seg_len = sqrt(dx*dx + dy*dy + dz*dz) * AU;
    }

    b->trail[b->trail_head][0] = pos[0] * RS;
    b->trail[b->trail_head][1] = pos[1] * RS;
    b->trail[b->trail_head][2] = pos[2] * RS;
    if (b->trail_seg_len) b->trail_seg_len[write_idx] = seg_len;
    b->trail_head = (b->trail_head + 1) & TRAIL_MASK;
    if (b->trail_count < TRAIL_LEN) {
        b->trail_count++;
        b->trail_total_len += seg_len;
    } else {
        /* Buffer full: evict oldest, subtract its successor's seg_len from total */
        int oldest_idx = b->trail_head;
        int next_oldest_idx = (oldest_idx + 1) & TRAIL_MASK;
        if (b->trail_seg_len)
            b->trail_total_len += seg_len - b->trail_seg_len[next_oldest_idx];
        else
            b->trail_total_len += seg_len;
    }

    /* Prune excess length by evicting oldest samples */
    while (b->trail_count > 2 && b->trail_total_len > target_world_len) {
        int oldest_idx = (b->trail_head - b->trail_count + TRAIL_LEN) & TRAIL_MASK;
        int next_oldest_idx = (oldest_idx + 1) & TRAIL_MASK;
        double drop_len = b->trail_seg_len ? b->trail_seg_len[next_oldest_idx] : 0.0;
        b->trail_total_len -= drop_len;
        if (b->trail_total_len < 0.0) b->trail_total_len = 0.0;
        b->trail_count--;
    }
}

/*
 * trail_segment_len_for_body — compute the arc-length sampling interval.
 *
 * Base interval is TRAIL_BASE_SEGMENT_LEN (or TRAIL_SATELLITE_SEGMENT_LEN for
 * moons).  During close approaches (collision_body_needs_dense_trail()),
 * the interval is reduced by TRAIL_CLOSE_APPROACH_FACTOR to capture the
 * high-curvature trajectory near impact for accurate Hermite interpolation in
 * trails_cut_body_at_time().
 */
static double trail_segment_len_for_body(const Body *b)
{
    double segment_len = TRAIL_BASE_SEGMENT_LEN;
    int body_idx = (int)(b - g_bodies);

    if (body_idx >= 0 && body_idx < g_nbodies && is_satellite(body_idx))
        segment_len = TRAIL_SATELLITE_SEGMENT_LEN;

    if (body_idx >= 0 && body_idx < g_nbodies &&
        collision_body_needs_dense_trail(body_idx))
        segment_len *= TRAIL_CLOSE_APPROACH_FACTOR;

    if (segment_len < TRAIL_MIN_SEGMENT_LEN) segment_len = TRAIL_MIN_SEGMENT_LEN;
    if (segment_len > TRAIL_MAX_SEGMENT_LEN) segment_len = TRAIL_MAX_SEGMENT_LEN;
    return segment_len;
}

/*
 * trail_curve_eval — evaluate the cubic Hermite spline at parameter t ∈ [0,1].
 *
 * The spline passes through p0 (at t=0) and p1 (at t=1) with tangents
 * v0×dt and v1×dt (velocity scaled by the time interval dt).  The cubic basis
 * functions ensure C1 continuity across adjacent spline segments, which is
 * important for smooth trail rendering and accurate arc-length integration.
 */
static void trail_curve_eval(const double p0[3], const double v0[3],
                             const double p1[3], const double v1[3],
                             double dt, double t, double out[3])
{
    double t2 = t * t;
    double t3 = t2 * t;
    double h00 =  2.0 * t3 - 3.0 * t2 + 1.0;
    double h10 =        t3 - 2.0 * t2 + t;
    double h01 = -2.0 * t3 + 3.0 * t2;
    double h11 =        t3 -       t2;

    out[0] = h00 * p0[0] + h10 * dt * v0[0] + h01 * p1[0] + h11 * dt * v1[0];
    out[1] = h00 * p0[1] + h10 * dt * v0[1] + h01 * p1[1] + h11 * dt * v1[1];
    out[2] = h00 * p0[2] + h10 * dt * v0[2] + h01 * p1[2] + h11 * dt * v1[2];
}

/*
 * trail_curve_eval_vel — evaluate the time-derivative of the Hermite spline.
 *
 * dh/dt of the four basis functions divided by dt gives dp/dt, the velocity
 * along the curve at parameter t.  Used to:
 *   1. Compute the tangent at the midpoint during adaptive subdivision.
 *   2. Estimate the velocity at the impact time in trails_cut_body_at_time().
 */
static void trail_curve_eval_vel(const double p0[3], const double v0[3],
                                 const double p1[3], const double v1[3],
                                 double dt, double t, double out[3])
{
    double t2 = t * t;
    double dh00 = 6.0 * t2 - 6.0 * t;
    double dh10 = 3.0 * t2 - 4.0 * t + 1.0;
    double dh01 = -6.0 * t2 + 6.0 * t;
    double dh11 = 3.0 * t2 - 2.0 * t;
    double inv_dt = (dt > 0.0) ? (1.0 / dt) : 0.0;

    out[0] = (dh00 * p0[0] + dh10 * dt * v0[0] + dh01 * p1[0] + dh11 * dt * v1[0]) * inv_dt;
    out[1] = (dh00 * p0[1] + dh10 * dt * v0[1] + dh01 * p1[1] + dh11 * dt * v1[1]) * inv_dt;
    out[2] = (dh00 * p0[2] + dh10 * dt * v0[2] + dh01 * p1[2] + dh11 * dt * v1[2]) * inv_dt;
}

/* Append a point to the adaptive subdivision output array.
 * The array is sized for 2^TRAIL_CURVE_MAX_DEPTH + 1 points. */
static void trail_curve_append_point(double points[][3], int *count,
                                     const double p[3])
{
    if (*count >= (1 << TRAIL_CURVE_MAX_DEPTH) + 1) return;
    points[*count][0] = p[0];
    points[*count][1] = p[1];
    points[*count][2] = p[2];
    (*count)++;
}

/*
 * trail_curve_flatten_recursive — adaptive Hermite-to-polyline subdivision.
 *
 * Compares the curve midpoint (at t=0.5 via trail_curve_eval) against the
 * chord midpoint (p0+p1)/2.  If the distance between them (the flatness
 * error) exceeds max_err, the interval is split at the midpoint into two
 * halves, each recursed independently.  The midpoint velocity is computed
 * via trail_curve_eval_vel for accurate sub-interval Hermite tangents.
 *
 * Appends endpoint p1 on leaf nodes — the starting point p0 is always the
 * last appended point from the caller, so the result forms a connected chain.
 */
static void trail_curve_flatten_recursive(const double p0[3], const double v0[3],
                                          const double p1[3], const double v1[3],
                                          double dt, double max_err, int depth,
                                          double points[][3], int *count)
{
    double mid_curve[3], mid_line[3], diff[3], err;

    trail_curve_eval(p0, v0, p1, v1, dt, 0.5, mid_curve);
    mid_line[0] = 0.5 * (p0[0] + p1[0]);
    mid_line[1] = 0.5 * (p0[1] + p1[1]);
    mid_line[2] = 0.5 * (p0[2] + p1[2]);
    diff[0] = mid_curve[0] - mid_line[0];
    diff[1] = mid_curve[1] - mid_line[1];
    diff[2] = mid_curve[2] - mid_line[2];
    err = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);

    if (depth >= TRAIL_CURVE_MAX_DEPTH || err <= max_err) {
        trail_curve_append_point(points, count, p1);
        return;
    }

    {
        double mid_vel[3];
        trail_curve_eval_vel(p0, v0, p1, v1, dt, 0.5, mid_vel);
        trail_curve_flatten_recursive(p0, v0, mid_curve, mid_vel,
                                      dt * 0.5, max_err, depth + 1,
                                      points, count);
        trail_curve_flatten_recursive(mid_curve, mid_vel, p1, v1,
                                      dt * 0.5, max_err, depth + 1,
                                      points, count);
    }
}

/*
 * trail_rebuild_segment — emit arc-length-uniform trail samples along the
 * Hermite spline from (start, start_vel) to (end, end_vel) over time dt.
 *
 * Steps:
 *   1. Flatten the Hermite curve to a polyline via adaptive subdivision.
 *   2. Walk each polyline segment, advancing trail_accum by the arc length.
 *   3. Every time trail_accum reaches segment_len, emit a trail sample at
 *      the interpolated position along the current polyline segment and
 *      reset trail_accum.  Allows fractional carry between segments.
 *
 * This arc-length parameterisation ensures samples are spatially uniform
 * regardless of where in the orbit (apoapsis vs periapsis) the step occurs.
 */
static void trail_rebuild_segment(Body *b,
                                  const double start[3], const double start_vel[3],
                                  const double end[3], const double end_vel[3],
                                  double dt, double segment_len, double max_err)
{
    double curve_points[(1 << TRAIL_CURVE_MAX_DEPTH) + 1][3];
    int curve_count = 0;

    trail_curve_append_point(curve_points, &curve_count, start);
    trail_curve_flatten_recursive(start, start_vel, end, end_vel,
                                  dt, max_err, 0,
                                  curve_points, &curve_count);

    for (int k = 1; k < curve_count; k++) {
        double seg_start[3], seg_end[3];
        double dx, dy, dz, seg_dist, traveled;

        seg_start[0] = curve_points[k - 1][0];
        seg_start[1] = curve_points[k - 1][1];
        seg_start[2] = curve_points[k - 1][2];
        seg_end[0] = curve_points[k][0];
        seg_end[1] = curve_points[k][1];
        seg_end[2] = curve_points[k][2];
        dx = seg_end[0] - seg_start[0];
        dy = seg_end[1] - seg_start[1];
        dz = seg_end[2] - seg_start[2];
        seg_dist = sqrt(dx*dx + dy*dy + dz*dz);
        traveled = 0.0;

        /* Emit a sample every segment_len metres of arc walked */
        while (b->trail_accum + (seg_dist - traveled) >= segment_len) {
            double need = segment_len - b->trail_accum;
            double u;
            double sample_pos[3];

            if (need < 0.0) need = 0.0;
            if (seg_dist <= 0.0) break;
            u = (traveled + need) / seg_dist;
            if (u < 0.0) u = 0.0;
            if (u > 1.0) u = 1.0;
            sample_pos[0] = seg_start[0] + (seg_end[0] - seg_start[0]) * u;
            sample_pos[1] = seg_start[1] + (seg_end[1] - seg_start[1]) * u;
            sample_pos[2] = seg_start[2] + (seg_end[2] - seg_start[2]) * u;
            sample_body_pos(b, sample_pos);
            traveled += need;
            if (traveled > seg_dist) traveled = seg_dist;
            b->trail_accum = 0.0;
        }

        b->trail_accum += seg_dist - traveled;
    }
}

/* ── collision rollback support ─────────────────────────────────────────── */

/*
 * trails_begin_frame_snapshot — save full trail state for every body at the
 * start of a physics frame.
 *
 * Called once per frame before physics integration begins.  The snapshot
 * captures every field needed to completely undo any trail emission that
 * happens during the frame: buffer head/count, accumulated partial segment
 * length, total physical trail length, and the previous position/velocity
 * used as the Hermite tangent for the next segment.
 *
 * This data is used by trails_cut_body_at_time() to restore a body's trail
 * to its pre-frame state when a collision is detected mid-frame.
 */
void trails_begin_frame_snapshot(void)
{
    for (int i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        b->trail_frame_accum = b->trail_accum;
        b->trail_frame_head = b->trail_head;
        b->trail_frame_count = b->trail_count;
        b->trail_frame_total_len = b->trail_total_len;
        b->trail_frame_pos[0] = b->pos[0];
        b->trail_frame_pos[1] = b->pos[1];
        b->trail_frame_pos[2] = b->pos[2];
        b->trail_frame_vel[0] = b->vel[0];
        b->trail_frame_vel[1] = b->vel[1];
        b->trail_frame_vel[2] = b->vel[2];
        b->trail_frame_prev_pos[0] = b->trail_prev_pos[0];
        b->trail_frame_prev_pos[1] = b->trail_prev_pos[1];
        b->trail_frame_prev_pos[2] = b->trail_prev_pos[2];
        b->trail_frame_prev_vel[0] = b->trail_prev_vel[0];
        b->trail_frame_prev_vel[1] = b->trail_prev_vel[1];
        b->trail_frame_prev_vel[2] = b->trail_prev_vel[2];
    }
}

/*
 * trails_cut_body_at_time — roll back and re-emit a body's trail up to the
 * collision point, then terminate it there.
 *
 * Called by the collision system when body body_idx collides at time hit_dt
 * into the current frame (which spans frame_dt total).  cut_pos is the impact
 * site in world metres.
 *
 * Steps:
 *   1. Restore trail state to the beginning-of-frame snapshot (undo any trail
 *      emission that occurred during this frame for this body).
 *   2. If hit_dt > 0: reconstruct the trail segment from the snapshot's prev
 *      position to cut_pos by evaluating the Hermite spline at τ = hit_dt/frame_dt
 *      and calling trail_rebuild_segment on the partial interval.
 *   3. Snap the last trail point to exactly cut_pos (or add it if the buffer
 *      was empty).  Uses snap-in-place if the distance is below one min segment
 *      length to avoid a spuriously short terminal segment.
 *   4. Update trail_prev_pos/vel to cut_pos/cut_vel so the next tick (if any)
 *      starts from the correct position.
 *
 * The result is a trail that ends cleanly at the impact site with no visual
 * jump or gap caused by the collision detection stepping past the impact.
 */
void trails_cut_body_at_time(int body_idx, double hit_dt, double frame_dt,
                             const double cut_pos[3])
{
    Body *b;
    double tau, cut_vel[3], segment_len, max_err;
    double dx, dy, dz, dist2;

    if (body_idx < 0 || body_idx >= g_nbodies || !cut_pos) return;
    b = &g_bodies[body_idx];
    if (!b->trail) return;

    /* Restore to beginning-of-frame snapshot */
    b->trail_head = b->trail_frame_head;
    b->trail_count = b->trail_frame_count;
    b->trail_accum = b->trail_frame_accum;
    b->trail_total_len = b->trail_frame_total_len;
    b->trail_prev_pos[0] = b->trail_frame_prev_pos[0];
    b->trail_prev_pos[1] = b->trail_frame_prev_pos[1];
    b->trail_prev_pos[2] = b->trail_frame_prev_pos[2];
    b->trail_prev_vel[0] = b->trail_frame_prev_vel[0];
    b->trail_prev_vel[1] = b->trail_frame_prev_vel[1];
    b->trail_prev_vel[2] = b->trail_frame_prev_vel[2];

    if (frame_dt <= 0.0 || hit_dt <= 0.0) {
        /* Impact at frame start — use the snapshot velocity as cut_vel */
        cut_vel[0] = b->trail_frame_prev_vel[0];
        cut_vel[1] = b->trail_frame_prev_vel[1];
        cut_vel[2] = b->trail_frame_prev_vel[2];
    } else {
        /* τ ∈ [0,1]: fractional time within the frame at which impact occurred */
        tau = hit_dt / frame_dt;
        if (tau < 0.0) tau = 0.0;
        if (tau > 1.0) tau = 1.0;
        /* Interpolate velocity at impact time using Hermite derivative */
        trail_curve_eval_vel(b->trail_frame_pos, b->trail_frame_vel,
                             b->pos, b->vel, frame_dt, tau, cut_vel);
        segment_len = trail_segment_len_for_body(b);
        max_err = segment_len * TRAIL_CURVE_ERROR_RATIO;
        if (max_err < TRAIL_CURVE_MIN_ERROR) max_err = TRAIL_CURVE_MIN_ERROR;
        if (max_err > TRAIL_CURVE_MAX_ERROR) max_err = TRAIL_CURVE_MAX_ERROR;
        /* Re-emit the partial trail from snapshot prev_pos up to cut_pos */
        trail_rebuild_segment(b,
                              b->trail_frame_prev_pos, b->trail_frame_prev_vel,
                              cut_pos, cut_vel, hit_dt, segment_len, max_err);
    }

    /* Snap or append the final point exactly at cut_pos */
    if (b->trail_count > 0) {
        int last_idx = (b->trail_head - 1 + TRAIL_LEN) & TRAIL_MASK;
        dx = b->trail[last_idx][0] - cut_pos[0] * RS;
        dy = b->trail[last_idx][1] - cut_pos[1] * RS;
        dz = b->trail[last_idx][2] - cut_pos[2] * RS;
        dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 <= (TRAIL_MIN_SEGMENT_LEN * RS) * (TRAIL_MIN_SEGMENT_LEN * RS)) {
            /* Close enough — snap in place rather than emitting a micro-segment */
            b->trail[last_idx][0] = cut_pos[0] * RS;
            b->trail[last_idx][1] = cut_pos[1] * RS;
            b->trail[last_idx][2] = cut_pos[2] * RS;
        } else {
            sample_body_pos(b, cut_pos);
        }
    } else {
        sample_body_pos(b, cut_pos);
    }

    /* Update prev_* so the next trail_tick starts from the impact site */
    b->trail_prev_pos[0] = cut_pos[0];
    b->trail_prev_pos[1] = cut_pos[1];
    b->trail_prev_pos[2] = cut_pos[2];
    b->trail_prev_vel[0] = cut_vel[0];
    b->trail_prev_vel[1] = cut_vel[1];
    b->trail_prev_vel[2] = cut_vel[2];
}

/* ── trail tick ─────────────────────────────────────────────────────────── */

/*
 * trails_tick / trails_tick_system — emit trail samples for one physics step.
 *
 * For each live, trail-emitting body, calls trail_rebuild_segment() on the
 * interval from (trail_prev_pos, trail_prev_vel) to the current (pos, vel)
 * over elapsed time dt.  Updates trail_prev_* after emission so the next
 * call continues from the correct state.
 *
 * Called once per outer RESPA step (or once per physics_step in warmup mode),
 * so dt matches the actual simulation timestep that produced the positions.
 */
void trails_tick(double dt) {
    trails_tick_system(-1, dt);
}

void trails_tick_system(int root, double dt) {
    int i;
    int slot = (root >= 0 && root < MAX_BODIES) ? s_root_to_slot[root] : -1;
    if (dt <= 0.0) return;

    if (root < 0 || slot < 0) {
        for (i = 0; i < g_nbodies; i++) {
            Body *b = &g_bodies[i];
            double segment_len, max_err, start[3], start_vel[3], end[3], end_vel[3];

            if (!b->alive || !in_system(i, root) || !b->trail || !b->trail_emitting) {
                continue;
            }
            segment_len = trail_segment_len_for_body(b);
            max_err = segment_len * TRAIL_CURVE_ERROR_RATIO;
            if (max_err < TRAIL_CURVE_MIN_ERROR) max_err = TRAIL_CURVE_MIN_ERROR;
            if (max_err > TRAIL_CURVE_MAX_ERROR) max_err = TRAIL_CURVE_MAX_ERROR;
            start[0] = b->trail_prev_pos[0];
            start[1] = b->trail_prev_pos[1];
            start[2] = b->trail_prev_pos[2];
            start_vel[0] = b->trail_prev_vel[0];
            start_vel[1] = b->trail_prev_vel[1];
            start_vel[2] = b->trail_prev_vel[2];
            end[0] = b->pos[0];
            end[1] = b->pos[1];
            end[2] = b->pos[2];
            end_vel[0] = b->vel[0];
            end_vel[1] = b->vel[1];
            end_vel[2] = b->vel[2];
            trail_rebuild_segment(b, start, start_vel, end, end_vel,
                                  dt, segment_len, max_err);

            b->trail_prev_pos[0] = end[0];
            b->trail_prev_pos[1] = end[1];
            b->trail_prev_pos[2] = end[2];
            b->trail_prev_vel[0] = end_vel[0];
            b->trail_prev_vel[1] = end_vel[1];
            b->trail_prev_vel[2] = end_vel[2];
        }
        return;
    }

    for (int mi = 0; mi < s_system_member_count[slot]; mi++) {
        i = s_system_members[slot][mi];
        Body *b = &g_bodies[i];
        double segment_len, max_err, start[3], start_vel[3], end[3], end_vel[3];

        if (!b->alive || !b->trail || !b->trail_emitting) {
            continue;
        }
        segment_len = trail_segment_len_for_body(b);
        max_err = segment_len * TRAIL_CURVE_ERROR_RATIO;
        if (max_err < TRAIL_CURVE_MIN_ERROR) max_err = TRAIL_CURVE_MIN_ERROR;
        if (max_err > TRAIL_CURVE_MAX_ERROR) max_err = TRAIL_CURVE_MAX_ERROR;
        start[0] = b->trail_prev_pos[0];
        start[1] = b->trail_prev_pos[1];
        start[2] = b->trail_prev_pos[2];
        start_vel[0] = b->trail_prev_vel[0];
        start_vel[1] = b->trail_prev_vel[1];
        start_vel[2] = b->trail_prev_vel[2];
        end[0] = b->pos[0];
        end[1] = b->pos[1];
        end[2] = b->pos[2];
        end_vel[0] = b->vel[0];
        end_vel[1] = b->vel[1];
        end_vel[2] = b->vel[2];
        trail_rebuild_segment(b, start, start_vel, end, end_vel,
                              dt, segment_len, max_err);

        b->trail_prev_pos[0] = end[0];
        b->trail_prev_pos[1] = end[1];
        b->trail_prev_pos[2] = end[2];
        b->trail_prev_vel[0] = end_vel[0];
        b->trail_prev_vel[1] = end_vel[1];
        b->trail_prev_vel[2] = end_vel[2];
    }
}
