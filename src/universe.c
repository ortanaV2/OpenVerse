/*
 * universe.c — flat-body universe loader and runtime management
 *
 * JSON schema (top-level keys):
 *
 *   "bodies"         — flat array of all bodies (stars, planets, moons)
 *   "rings"          — ring-system descriptors (passed through unchanged)
 *   "asteroid_belts" — belt descriptors (passed through unchanged)
 *
 * Body placement rules by "type":
 *
 *   "star"                       — placed at absolute world position given by
 *                                  "pos_ly" ([x,y,z] in light-years from origin).
 *                                  Optional "velocity_km_s" sets bulk proper-motion
 *                                  velocity for the whole system (applied last).
 *
 *   "planet" / "dwarf_planet" /
 *   "asteroid"                   — Keplerian orbit around "parent" star.
 *                                  "parent" must name a star already loaded in Pass 1.
 *
 *   "moon"                       — parent-relative orbit around "parent" planet/moon
 *                                  using "moon_keplerian" elements (a in km, GM of parent).
 *
 * Three-pass load order (why passes matter):
 *   Pass 1 — stars first, so absolute world positions exist before any body tries
 *             to reference a star as its parent.
 *   Pass 2 — planets/dwarf_planets/asteroids: find_body_index() can locate the parent star
 *             because all stars are already in g_bodies[].
 *   Pass 3 — moons: parent planets are fully positioned so moon_to_state() receives
 *             the correct world-space parent GM and can add the parent offset.
 *
 * Post-processing per star (after all bodies loaded):
 *   1. Centre-of-mass velocity correction: the star's velocity is adjusted so the
 *      total linear momentum of the system is zero in the star's frame.  Only the
 *      star is adjusted because M_star >> M_planets, making the correction exact
 *      enough for long-term stability without touching every planet velocity.
 *   2. Apply bulk velocity (proper motion): "velocity_km_s" from the JSON is added
 *      uniformly to every body in the system, translating the whole system through
 *      world space at the correct stellar drift speed.
 *
 * Parent chain convention (Body.parent field):
 *   stars        — parent = -1   (root of the chain)
 *   planets      — parent = star body index
 *   moons        — parent = planet body index
 *   The root_star_of() helper walks this chain upward until parent == -1.
 */
#include "universe.h"
#include "body.h"
#include "json.h"
#include "common.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ------------------------------------------------------------------ helpers */

/*
 * ensure_capacity — grow g_bodies[] to hold at least `needed` entries.
 *
 * Doubling strategy: capacity starts at MAX_BODIES, then doubles each time the
 * limit is reached.  This amortises realloc cost to O(1) per insertion.
 * The process exits on allocation failure — there is no graceful recovery path
 * since a partially-loaded universe is unusable.
 */
static void ensure_capacity(int needed)
{
    if (needed <= g_bodies_cap) return;
    int new_cap = g_bodies_cap ? g_bodies_cap * 2 : MAX_BODIES;
    while (new_cap < needed) new_cap *= 2;
    Body *p = (Body*)realloc(g_bodies, new_cap * sizeof(Body));
    if (!p) { fprintf(stderr, "[universe] out of memory\n"); exit(1); }
    g_bodies     = p;
    g_bodies_cap = new_cap;
}

/*
 * alloc_trail — allocate and zero-initialise all trail state for a body.
 *
 * Two parallel trail systems live side-by-side:
 *
 *   Simulation trail ("trail_*"):
 *     The authoritative, accumulated path.  TRAIL_LEN double[3] positions plus
 *     per-segment arc lengths.  Used by trails_render() for the visible ribbon.
 *
 *   Frame-snapshot trail ("trail_frame_*"):
 *     A snapshot of the trail state as it was at the start of the current
 *     physics frame.  The collision system uses this to roll back and re-emit
 *     the trail from its pre-impact state when two bodies merge, preventing
 *     a visual glitch where the trail suddenly jumps to the collision point.
 *
 * Both systems share the same circular-buffer structure (head, count, accum,
 * total_len) and a "prev" sample for Hermite spline tangent computation.
 * Initialising prev_pos/vel to the body's current position and velocity at
 * load time ensures the first trail segment has a valid tangent.
 */
static void alloc_trail(Body *bo)
{
    if (bo->trail)
        memset(bo->trail, 0, TRAIL_LEN * 3 * sizeof(double));
    else
        bo->trail = (double(*)[3])calloc(TRAIL_LEN, 3 * sizeof(double));
    if (bo->trail_seg_len)
        memset(bo->trail_seg_len, 0, TRAIL_LEN * sizeof(double));
    else
        bo->trail_seg_len = (double*)calloc(TRAIL_LEN, sizeof(double));
    if (!bo->trail || !bo->trail_seg_len) {
        fprintf(stderr, "[universe] trail alloc failed\n");
        exit(1);
    }
    bo->trail_head  = 0;
    bo->trail_count = 0;
    bo->trail_accum = 0.0;
    bo->trail_total_len = 0.0;
    bo->trail_fade  = 1.0;
    bo->trail_emitting = 1;
    bo->trail_prev_pos[0] = bo->pos[0];
    bo->trail_prev_pos[1] = bo->pos[1];
    bo->trail_prev_pos[2] = bo->pos[2];
    bo->trail_prev_vel[0] = bo->vel[0];
    bo->trail_prev_vel[1] = bo->vel[1];
    bo->trail_prev_vel[2] = bo->vel[2];
    bo->trail_frame_accum = 0.0;
    bo->trail_frame_head = 0;
    bo->trail_frame_count = 0;
    bo->trail_frame_total_len = 0.0;
    bo->trail_frame_pos[0] = bo->pos[0];
    bo->trail_frame_pos[1] = bo->pos[1];
    bo->trail_frame_pos[2] = bo->pos[2];
    bo->trail_frame_vel[0] = bo->vel[0];
    bo->trail_frame_vel[1] = bo->vel[1];
    bo->trail_frame_vel[2] = bo->vel[2];
    bo->trail_frame_prev_pos[0] = bo->trail_prev_pos[0];
    bo->trail_frame_prev_pos[1] = bo->trail_prev_pos[1];
    bo->trail_frame_prev_pos[2] = bo->trail_prev_pos[2];
    bo->trail_frame_prev_vel[0] = bo->trail_prev_vel[0];
    bo->trail_frame_prev_vel[1] = bo->trail_prev_vel[1];
    bo->trail_frame_prev_vel[2] = bo->trail_prev_vel[2];
}

/* O(n) linear search through g_bodies[0..n-1] by name.  Only called during
 * loading (Pass 2 and Pass 3), never in the hot path. */
static int find_body_index(const char *name, int n)
{
    int i;
    for (i = 0; i < n; i++)
        if (strcmp(g_bodies[i].name, name) == 0) return i;
    return -1;
}

int universe_live_body_count(void)
{
    int n = 0;
    for (int i = 0; i < g_nbodies; i++) {
        if (g_bodies[i].alive) n++;
    }
    return n;
}

int universe_can_add_body(void)
{
    return universe_live_body_count() < MAX_BODIES;
}

static int find_reusable_body_slot(void)
{
    int n = g_nbodies < MAX_BODIES ? g_nbodies : MAX_BODIES;
    for (int i = 0; i < n; i++) {
        if (!g_bodies[i].alive) return i;
    }
    return -1;
}

/* Read a [r, g, b] JSON array into a float[3].  Missing elements default to 0. */
static void read_color(const JsonNode *arr, float col[3])
{
    col[0] = (float)json_num(json_idx(arr, 0), 0.0);
    col[1] = (float)json_num(json_idx(arr, 1), 0.0);
    col[2] = (float)json_num(json_idx(arr, 2), 0.0);
}

/* Zero a Body struct and set safe scalar defaults: alive=1, parent=-1,
 * atm_scale=1.0 (no atmosphere still renders correctly at unit scale). */
static void body_defaults(Body *bo)
{
    memset(bo, 0, sizeof(*bo));
    bo->alive     = 1;
    bo->parent    = -1;
    bo->atm_scale = 1.0f;
}

/* Read obliquity and rotation period from JSON.
 * rotation_rate is stored in rad/s: 2π / (period_days × DAY). */
static void read_rotation(const JsonNode *bn, Body *bo)
{
    JsonNode *obl = json_get(bn, "obliquity_deg");
    JsonNode *rot = json_get(bn, "rotation_period_days");
    if (obl) bo->obliquity = json_num(obl, 0.0);
    if (rot && json_num(rot, 0.0) != 0.0)
        bo->rotation_rate = (2.0 * PI) / (json_num(rot, 1.0) * DAY);
}

/* Read the optional "atmosphere" sub-object: color, intensity, and scale. */
static void read_atmosphere(const JsonNode *bn, Body *bo)
{
    JsonNode *atm = json_get(bn, "atmosphere");
    if (!atm) return;
    float ac[3]; read_color(json_get(atm, "color"), ac);
    bo->atm_color[0]  = ac[0];
    bo->atm_color[1]  = ac[1];
    bo->atm_color[2]  = ac[2];
    bo->atm_intensity = (float)json_num(json_get(atm, "intensity"), 0.0);
    bo->atm_scale     = (float)json_num(json_get(atm, "scale"),     1.0);
}

/* Read the optional "accretion_disk" sub-object for a black hole: base color,
 * inner and outer radius as multiples of the event-horizon radius. Defaults
 * give a warm disk spanning 2.6–9× the horizon when the block is omitted. */
static void read_accretion_disk(const JsonNode *bn, Body *bo)
{
    JsonNode *disk = json_get(bn, "accretion_disk");
    float dc[3] = { 1.0f, 0.55f, 0.18f };
    if (disk) read_color(json_get(disk, "color"), dc);
    bo->disk_color[0] = dc[0];
    bo->disk_color[1] = dc[1];
    bo->disk_color[2] = dc[2];
    bo->disk_inner = (float)json_num(json_get(disk, "inner"), 2.6);
    bo->disk_outer = (float)json_num(json_get(disk, "outer"), 9.0);
}

/*
 * root_star_of — thin wrapper for body_root_star() (defined in body.c).
 *   Stars (parent=-1) return themselves in zero hops.
 *   Planets (parent=star) return the star in one hop.
 *   Moons (parent=planet, parent=star) return the star in two hops.
 */
static int root_star_of(int i) { return body_root_star(i); }

/* ------------------------------------------------------------------ public */

void universe_load(const char *path)
{
    int i, s;
    fprintf(stdout, "[Boot] Loading universe data from %s\n", path);
    fflush(stdout);
    JsonNode *root = json_parse_file(path);
    if (!root) {
        fprintf(stderr, "[universe] failed to open or parse '%s'\n", path);
        exit(1);
    }

    JsonNode *bodies_arr = json_get(root, "bodies");
    if (!bodies_arr || bodies_arr->type != JSON_ARRAY) {
        fprintf(stderr, "[universe] 'bodies' array not found in '%s'\n", path);
        json_free(root); exit(1);
    }

    g_nbodies = 0;

    /* Bulk velocity per body slot.  Set for star indices during Pass 1;
     * zero everywhere else.  Applied in post-processing after CoM correction. */
    double bv[MAX_BODIES][3];
    for (i = 0; i < MAX_BODIES; i++) bv[i][0] = bv[i][1] = bv[i][2] = 0.0;

    /* ================================================================
     * Pass 1 — Stars
     *
     * Stars are placed at absolute world positions (pos_ly × LY → metres).
     * The bulk velocity vector (velocity_km_s) is stashed in bv[star_idx]
     * and applied after the full system is loaded so that the CoM correction
     * can be computed first against the orbital velocities alone.
     * ================================================================ */
    fprintf(stdout, "[Boot] Universe pass 1/3: stars\n");
    fflush(stdout);
    {
        JsonNode *bn;
        for (bn = bodies_arr->first_child; bn; bn = bn->next) {
            const char *type = json_str(json_get(bn, "type"), "");
            int is_bh = (strcmp(type, "black_hole") == 0);
            if (strcmp(type, "star") != 0 && !is_bh) continue;

            const char *name   = json_str(json_get(bn, "name"),      "unknown");
            double      mass   = json_num(json_get(bn, "mass"),       0.0);
            double      rad_km = json_num(json_get(bn, "radius_km"),  1.0);
            float       col[3];
            read_color(json_get(bn, "color"), col);

            double px = 0.0, py = 0.0, pz = 0.0;
            JsonNode *ply = json_get(bn, "pos_ly");
            if (ply) {
                px = json_num(json_idx(ply, 0), 0.0) * LY;
                py = json_num(json_idx(ply, 1), 0.0) * LY;
                pz = json_num(json_idx(ply, 2), 0.0) * LY;
            }

            ensure_capacity(g_nbodies + 1);
            Body *bo = &g_bodies[g_nbodies];
            body_defaults(bo);
            strncpy(bo->name, name, 31); bo->name[31] = '\0';
            bo->mass           = mass;
            bo->radius         = rad_km * 1000.0;
            bo->pos[0]         = px; bo->pos[1] = py; bo->pos[2] = pz;
            bo->col[0]         = col[0]; bo->col[1] = col[1]; bo->col[2] = col[2];
            bo->is_star        = 1;   /* black holes are system roots, like stars */
            bo->is_black_hole  = is_bh;
            if (is_bh) read_accretion_disk(bn, bo);
            read_rotation(bn, bo);

            /* Stash bulk velocity; convert km/s → m/s */
            JsonNode *vn = json_get(bn, "velocity_km_s");
            if (vn) {
                bv[g_nbodies][0] = json_num(json_idx(vn, 0), 0.0) * 1000.0;
                bv[g_nbodies][1] = json_num(json_idx(vn, 1), 0.0) * 1000.0;
                bv[g_nbodies][2] = json_num(json_idx(vn, 2), 0.0) * 1000.0;
            }

            alloc_trail(bo);
            g_nbodies++;
        }
    }

    /* ================================================================
     * Pass 2 — Planets, dwarf_planets, and asteroids
     *
     * keplerian_to_state() returns a heliocentric position/velocity in SI
     * units (metres, m/s) relative to the parent star.  The star's world
     * position is then added to convert to absolute world coordinates.
     *
     * GM of the parent star must be expressed in AU³/day² to match the
     * JPL table convention used by keplerian_to_state():
     *   gm_star_au2 = G × M_star / AU³ × DAY²
     *
     * All parent data is read before the ensure_capacity() call: while
     * ensure_capacity uses an integer index (par_idx) that survives realloc,
     * taking a pointer to g_bodies[par_idx] before realloc would be UB.
     * ================================================================ */
    fprintf(stdout, "[Boot] Universe pass 2/3: planets, dwarf planets, and asteroids\n");
    fflush(stdout);
    {
        JsonNode *bn;
        for (bn = bodies_arr->first_child; bn; bn = bn->next) {
            const char *type = json_str(json_get(bn, "type"), "");
            if (strcmp(type, "planet") != 0 &&
                strcmp(type, "dwarf_planet") != 0 &&
                strcmp(type, "asteroid") != 0)
                continue;

            const char *name     = json_str(json_get(bn, "name"),      "unknown");
            double      mass     = json_num(json_get(bn, "mass"),       0.0);
            double      rad_km   = json_num(json_get(bn, "radius_km"),  1.0);
            const char *par_name = json_str(json_get(bn, "parent"),     "");
            float       col[3];
            read_color(json_get(bn, "color"), col);

            int par_idx = find_body_index(par_name, g_nbodies);
            if (par_idx < 0) {
                fprintf(stderr, "[universe] orbiting body '%s': parent '%s' not found\n",
                        name, par_name);
                json_free(root); exit(1);
            }

            /* GM of parent star in AU³/day² for keplerian_to_state() */
            double gm_star_au2 = G_CONST * g_bodies[par_idx].mass
                                 / (AU * AU * AU) * (DAY * DAY);

            double p[3] = {0,0,0}, v[3] = {0,0,0};
            double a = 1.0;
            JsonNode *kep = json_get(bn, "keplerian");
            if (kep) {
                a        = json_num(json_get(kep, "a"),            1.0);
                double e = json_num(json_get(kep, "e"),            0.0);
                double ii= json_num(json_get(kep, "i"),            0.0);
                double O = json_num(json_get(kep, "Omega"),        0.0);
                double w = json_num(json_get(kep, "omega_tilde"),  0.0);
                double L = json_num(json_get(kep, "L"),            0.0);
                keplerian_to_state(a, e, ii, O, w, L, gm_star_au2, p, v);
            }
            /* Convert heliocentric → world coordinates */
            p[0] += g_bodies[par_idx].pos[0];
            p[1] += g_bodies[par_idx].pos[1];
            p[2] += g_bodies[par_idx].pos[2];

            ensure_capacity(g_nbodies + 1);
            Body *bo = &g_bodies[g_nbodies++];
            body_defaults(bo);
            strncpy(bo->name, name, 31); bo->name[31] = '\0';
            bo->mass           = mass;
            bo->radius         = rad_km * 1000.0;
            bo->pos[0]         = p[0]; bo->pos[1] = p[1]; bo->pos[2] = p[2];
            bo->vel[0]         = v[0]; bo->vel[1] = v[1]; bo->vel[2] = v[2];
            bo->col[0]         = col[0]; bo->col[1] = col[1]; bo->col[2] = col[2];
            bo->parent         = par_idx;
            read_rotation(bn, bo);
            read_atmosphere(bn, bo);
            alloc_trail(bo);
        }
    }

    /* ================================================================
     * Pass 3 — Moons
     *
     * moon_to_state() returns a parent-relative position/velocity (metres,
     * m/s) in the GL frame.  The parent's world position and velocity are
     * added to get absolute state.
     *
     * The parent's position and velocity are cached in local arrays BEFORE
     * ensure_capacity() is called.  ensure_capacity() may realloc g_bodies[],
     * invalidating any raw pointer taken from it.  Using cached scalar copies
     * avoids any dependency on the old pointer after realloc.
     * ================================================================ */
    fprintf(stdout, "[Boot] Universe pass 3/3: moons\n");
    fflush(stdout);
    {
        JsonNode *bn;
        for (bn = bodies_arr->first_child; bn; bn = bn->next) {
            const char *type = json_str(json_get(bn, "type"), "");
            if (strcmp(type, "moon") != 0) continue;

            const char *name     = json_str(json_get(bn, "name"),     "unknown");
            double      mass     = json_num(json_get(bn, "mass"),      0.0);
            double      rad_km   = json_num(json_get(bn, "radius_km"), 1.0);
            const char *par_name = json_str(json_get(bn, "parent"),    "");
            float       col[3];
            read_color(json_get(bn, "color"), col);

            int par_idx = find_body_index(par_name, g_nbodies);
            if (par_idx < 0) {
                fprintf(stderr, "[universe] moon '%s': parent '%s' not found\n",
                        name, par_name);
                json_free(root); exit(1);
            }

            /* Cache parent state before potential realloc in ensure_capacity() */
            double gm_par   = G_CONST * g_bodies[par_idx].mass;
            double par_p[3] = { g_bodies[par_idx].pos[0],
                                g_bodies[par_idx].pos[1],
                                g_bodies[par_idx].pos[2] };
            double par_v[3] = { g_bodies[par_idx].vel[0],
                                g_bodies[par_idx].vel[1],
                                g_bodies[par_idx].vel[2] };

            JsonNode *mk = json_get(bn, "moon_keplerian");
            double rel_p[3] = {0,0,0}, rel_v[3] = {0,0,0};
            double a_km = 0.0;
            if (mk) {
                a_km             = json_num(json_get(mk, "a_km"),      0.0);
                double e         = json_num(json_get(mk, "e"),          0.0);
                double i_deg     = json_num(json_get(mk, "i_deg"),      0.0);
                double Omega_deg = json_num(json_get(mk, "Omega_deg"),  0.0);
                double omega_deg = json_num(json_get(mk, "omega_deg"),  0.0);
                double M0_deg    = json_num(json_get(mk, "M0_deg"),     0.0);
                moon_to_state(a_km, e, i_deg, Omega_deg, omega_deg,
                              M0_deg, gm_par, rel_p, rel_v);
            }

            ensure_capacity(g_nbodies + 1);
            Body *bo = &g_bodies[g_nbodies++];
            body_defaults(bo);
            strncpy(bo->name, name, 31); bo->name[31] = '\0';
            bo->mass           = mass;
            bo->radius         = rad_km * 1000.0;
            bo->pos[0]         = par_p[0] + rel_p[0];
            bo->pos[1]         = par_p[1] + rel_p[1];
            bo->pos[2]         = par_p[2] + rel_p[2];
            bo->vel[0]         = par_v[0] + rel_v[0];
            bo->vel[1]         = par_v[1] + rel_v[1];
            bo->vel[2]         = par_v[2] + rel_v[2];
            bo->col[0]         = col[0]; bo->col[1] = col[1]; bo->col[2] = col[2];
            bo->parent         = par_idx;
            read_rotation(bn, bo);
            read_atmosphere(bn, bo);
            alloc_trail(bo);
        }
    }

    /* ================================================================
     * Post-processing — per star system:
     *
     * Step 1 — Centre-of-mass velocity correction.
     *   After keplerian_to_state(), each planet has a heliocentric velocity
     *   that assumes the star is stationary.  Summing p·v over all planets
     *   gives a net momentum; this is removed by nudging the star velocity:
     *     v_star -= Σ (M_i / M_star) × v_i
     *   Only the star is adjusted (M_star >> M_planets), so the correction
     *   is negligible for the planets and exact for the total momentum.
     *
     * Step 2 — Bulk velocity (proper motion).
     *   The stellar "velocity_km_s" from JSON is the system's velocity through
     *   the galaxy.  After CoM correction, this is added uniformly to every
     *   body in the system so the system drifts as a rigid unit.
     * ================================================================ */
    fprintf(stdout, "[Boot] Universe post-processing: system velocities\n");
    fflush(stdout);
    int n_stars = 0;
    for (s = 0; s < g_nbodies; s++) {
        if (!g_bodies[s].is_star) continue;
        n_stars++;

        /* CoM correction: zero net internal momentum by adjusting only the star */
        for (i = 0; i < g_nbodies; i++) {
            if (i == s || root_star_of(i) != s) continue;
            g_bodies[s].vel[0] -=
                g_bodies[i].mass * g_bodies[i].vel[0] / g_bodies[s].mass;
            g_bodies[s].vel[1] -=
                g_bodies[i].mass * g_bodies[i].vel[1] / g_bodies[s].mass;
            g_bodies[s].vel[2] -=
                g_bodies[i].mass * g_bodies[i].vel[2] / g_bodies[s].mass;
        }

        /* Apply bulk proper-motion velocity uniformly to the whole system */
        if (bv[s][0] != 0.0 || bv[s][1] != 0.0 || bv[s][2] != 0.0) {
            for (i = 0; i < g_nbodies; i++) {
                if (root_star_of(i) != s) continue;
                g_bodies[i].vel[0] += bv[s][0];
                g_bodies[i].vel[1] += bv[s][1];
                g_bodies[i].vel[2] += bv[s][2];
            }
        }

        int cnt = 0;
        for (i = 0; i < g_nbodies; i++)
            if (root_star_of(i) == s) cnt++;
        fprintf(stdout,
                "[universe] '%s' at (%.3g, %.3g, %.3g) ly  -  %d bod%s\n",
                g_bodies[s].name,
                g_bodies[s].pos[0] / LY,
                g_bodies[s].pos[1] / LY,
                g_bodies[s].pos[2] / LY,
                cnt, cnt == 1 ? "y" : "ies");
    }

    fprintf(stdout, "[universe] total: %d bodies across %d star%s\n",
            g_nbodies, n_stars, n_stars == 1 ? "" : "s");
    fflush(stdout);

    json_free(root);
}

/*
 * universe_add_body — create a body at runtime from a BodyCreateSpec.
 *
 * Used by the build system to add stars, planets, and moons interactively.
 * Mirrors the three-pass loader but operates on a single pre-filled spec.
 * Returns the new body's index in g_bodies[], or -1 on failure.
 */
int universe_add_body(const BodyCreateSpec *spec)
{
    int idx, reused_slot;
    double (*old_trail)[3] = NULL;
    double *old_trail_seg_len = NULL;
    Body *bo;

    if (!spec) return -1;
    if (!universe_can_add_body()) {
        fprintf(stderr, "[universe] cannot add body '%s': live MAX_BODIES reached\n",
                spec->name ? spec->name : "unknown");
        return -1;
    }

    idx = find_reusable_body_slot();
    reused_slot = (idx >= 0);
    if (!reused_slot) {
        if (g_nbodies >= MAX_BODIES) {
            fprintf(stderr, "[universe] cannot add body '%s': no reusable body slots\n",
                    spec->name ? spec->name : "unknown");
            return -1;
        }
        ensure_capacity(g_nbodies + 1);
        idx = g_nbodies++;
    }

    bo = &g_bodies[idx];
    if (reused_slot) {
        old_trail = bo->trail;
        old_trail_seg_len = bo->trail_seg_len;
    }
    body_defaults(bo);
    if (reused_slot) {
        bo->trail = old_trail;
        bo->trail_seg_len = old_trail_seg_len;
    }

    strncpy(bo->name, spec->name ? spec->name : "Body", 31);
    bo->name[31] = '\0';
    bo->mass = spec->mass;
    bo->radius = spec->radius;
    bo->pos[0] = spec->pos[0];
    bo->pos[1] = spec->pos[1];
    bo->pos[2] = spec->pos[2];
    bo->vel[0] = spec->vel[0];
    bo->vel[1] = spec->vel[1];
    bo->vel[2] = spec->vel[2];
    bo->col[0] = spec->col[0];
    bo->col[1] = spec->col[1];
    bo->col[2] = spec->col[2];
    bo->is_star = spec->is_star;
    bo->is_black_hole = spec->is_black_hole;
    bo->parent = spec->parent;
    bo->obliquity = spec->obliquity;
    bo->rotation_rate = spec->rotation_rate;
    bo->atm_color[0] = spec->atm_color[0];
    bo->atm_color[1] = spec->atm_color[1];
    bo->atm_color[2] = spec->atm_color[2];
    bo->atm_intensity = spec->atm_intensity;
    bo->atm_scale = spec->atm_scale > 0.0f ? spec->atm_scale : 1.0f;
    bo->disk_color[0] = spec->disk_color[0];
    bo->disk_color[1] = spec->disk_color[1];
    bo->disk_color[2] = spec->disk_color[2];
    bo->disk_inner = spec->disk_inner;
    bo->disk_outer = spec->disk_outer;
    alloc_trail(bo);

    return idx;
}

/*
 * universe_rebind_to_nearest_stars — reassign planet parent pointers after a
 * star has been added or moved by the build system.
 *
 * Only star-orbiting bodies are eligible for rebinding:
 *   - Skips dead bodies and stars (they manage their own parent = -1).
 *   - Skips moons: if a body's current parent is not a star (i.e. parent is a
 *     planet or another moon), its orbital hierarchy is left intact.
 *   - Parentless non-stars (parent == -1) are also candidates — they adopt the
 *     nearest star.
 *
 * Use case: the user drops a new star near an existing solar system in build
 * mode.  The planets whose nearest star is now the new one should switch their
 * parent pointer so that physics grouping and LOD decisions stay correct.
 */
void universe_rebind_to_nearest_stars(void)
{
    for (int i = 0; i < g_nbodies; i++) {
        int best_star = -1;
        double best_d2 = 1e300;

        if (!g_bodies[i].alive || g_bodies[i].is_star) continue;
        if (g_bodies[i].parent >= 0 && !g_bodies[g_bodies[i].parent].is_star)
            continue;

        for (int s = 0; s < g_nbodies; s++) {
            double dx, dy, dz, d2;
            if (!g_bodies[s].alive || !g_bodies[s].is_star) continue;
            dx = g_bodies[s].pos[0] - g_bodies[i].pos[0];
            dy = g_bodies[s].pos[1] - g_bodies[i].pos[1];
            dz = g_bodies[s].pos[2] - g_bodies[i].pos[2];
            d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_star = s;
            }
        }

        if (best_star >= 0)
            g_bodies[i].parent = best_star;
    }
}

/* Free all trail buffers and the body array itself, then reset globals. */
void universe_shutdown(void)
{
    int i;
    for (i = 0; i < g_nbodies; i++) {
        free(g_bodies[i].trail);
        g_bodies[i].trail = NULL;
        free(g_bodies[i].trail_seg_len);
        g_bodies[i].trail_seg_len = NULL;
    }
    free(g_bodies);
    g_bodies     = NULL;
    g_nbodies    = 0;
    g_bodies_cap = 0;
}
