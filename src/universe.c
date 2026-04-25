/*
 * universe.c — flat-bodies universe loader
 *
 * JSON schema (top-level keys):
 *
 *   "bodies"         — flat array of all bodies (stars, planets, moons)
 *   "rings"          — ring-system descriptors (unchanged)
 *   "asteroid_belts" — belt descriptors (unchanged)
 *
 * Body placement rules by "type":
 *
 *   "star"                       — placed at absolute position given by
 *                                  "pos_ly" (light-years from origin).
 *                                  Optional "velocity_km_s" sets bulk
 *                                  proper-motion velocity for the whole system.
 *
 *   "planet" / "dwarf_planet"    — Keplerian orbit around "parent" star.
 *                                  "parent" must name a star already loaded.
 *
 *   "moon"                       — parent-relative orbit around "parent"
 *                                  planet/moon using "moon_keplerian" elements.
 *
 * Three-pass load order:
 *   Pass 1 — stars          (need absolute positions before anything else)
 *   Pass 2 — planets / dwarf_planets
 *   Pass 3 — moons
 *
 * Post-processing per star (after all bodies loaded):
 *   1. Centre-of-mass velocity correction (zero internal momentum)
 *   2. Apply bulk velocity to all bodies in system
 *
 * Parent chain convention (Body.parent field):
 *   stars        — parent = -1
 *   planets      — parent = star body index
 *   moons        — parent = planet body index
 *
 * The root_star_of() helper walks this chain to find the owning star,
 * which is used for per-system post-processing and physics grouping.
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

static void alloc_trail(Body *bo)
{
    bo->trail = (double(*)[3])calloc(TRAIL_LEN, 3 * sizeof(double));
    if (!bo->trail) { fprintf(stderr, "[universe] trail alloc failed\n"); exit(1); }
    bo->trail_head  = 0;
    bo->trail_count = 0;
    bo->trail_accum = 0.0;
    bo->trail_fade  = 1.0;
}

static int find_body_index(const char *name, int n)
{
    int i;
    for (i = 0; i < n; i++)
        if (strcmp(g_bodies[i].name, name) == 0) return i;
    return -1;
}

static void read_color(const JsonNode *arr, float col[3])
{
    col[0] = (float)json_num(json_idx(arr, 0), 0.0);
    col[1] = (float)json_num(json_idx(arr, 1), 0.0);
    col[2] = (float)json_num(json_idx(arr, 2), 0.0);
}

static void body_defaults(Body *bo)
{
    memset(bo, 0, sizeof(*bo));
    bo->alive     = 1;
    bo->parent    = -1;
    bo->atm_scale = 1.0f;
}

static double trail_interval_for_body(const Body *bo)
{
    if (bo->is_star) return DAY * 25.0;
    if (bo->parent >= 0 && bo->parent < g_nbodies && g_bodies[bo->parent].mass > 0.0) {
        double dx = bo->pos[0] - g_bodies[bo->parent].pos[0];
        double dy = bo->pos[1] - g_bodies[bo->parent].pos[1];
        double dz = bo->pos[2] - g_bodies[bo->parent].pos[2];
        double r = sqrt(dx*dx + dy*dy + dz*dz);
        double gm = G_CONST * g_bodies[bo->parent].mass;
        if (r > 0.0 && gm > 0.0) {
            double T = 2.0 * PI * sqrt(r * r * r / gm);
            double interval = T / 400.0;
            if (interval < 60.0) interval = 60.0;
            return interval;
        }
    }
    return DAY;
}

static void read_rotation(const JsonNode *bn, Body *bo)
{
    JsonNode *obl = json_get(bn, "obliquity_deg");
    JsonNode *rot = json_get(bn, "rotation_period_days");
    if (obl) bo->obliquity = json_num(obl, 0.0);
    if (rot && json_num(rot, 0.0) != 0.0)
        bo->rotation_rate = (2.0 * PI) / (json_num(rot, 1.0) * DAY);
}

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

/*
 * root_star_of — walk the parent chain to find the owning star.
 *   Stars (parent=-1) return themselves immediately.
 *   Planets (parent=star) return the star in one hop.
 *   Moons  (parent=planet, parent=star) return the star in two hops.
 */
static int root_star_of(int i) { return body_root_star(i); }

/* ------------------------------------------------------------------ public */

void universe_load(const char *path)
{
    int i, s;
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

    /* Bulk velocity per body slot (set for stars during Pass 1, zero otherwise) */
    double bv[MAX_BODIES][3];
    for (i = 0; i < MAX_BODIES; i++) bv[i][0] = bv[i][1] = bv[i][2] = 0.0;

    /* ================================================================
     * Pass 1 — Stars
     * Placed at their absolute position (pos_ly → metres).
     * Bulk velocity stashed in bv[]; applied in post-processing.
     * ================================================================ */
    {
        JsonNode *bn;
        for (bn = bodies_arr->first_child; bn; bn = bn->next) {
            const char *type = json_str(json_get(bn, "type"), "");
            if (strcmp(type, "star") != 0) continue;

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
            bo->is_star        = 1;
            bo->trail_interval = DAY * 25.0;
            read_rotation(bn, bo);

            /* Stash bulk velocity for post-processing */
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
     * Pass 2 — Planets and dwarf_planets
     * Keplerian orbit around "parent" star.
     * Body.parent is set to the star's index so root_star_of() works
     * and the physics engine correctly skips planet–star fast forces.
     * ================================================================ */
    {
        JsonNode *bn;
        for (bn = bodies_arr->first_child; bn; bn = bn->next) {
            const char *type = json_str(json_get(bn, "type"), "");
            if (strcmp(type, "planet") != 0 && strcmp(type, "dwarf_planet") != 0)
                continue;

            const char *name     = json_str(json_get(bn, "name"),      "unknown");
            double      mass     = json_num(json_get(bn, "mass"),       0.0);
            double      rad_km   = json_num(json_get(bn, "radius_km"),  1.0);
            const char *par_name = json_str(json_get(bn, "parent"),     "");
            float       col[3];
            read_color(json_get(bn, "color"), col);

            int par_idx = find_body_index(par_name, g_nbodies);
            if (par_idx < 0) {
                fprintf(stderr, "[universe] planet '%s': parent '%s' not found\n",
                        name, par_name);
                json_free(root); exit(1);
            }

            /* GM of parent star in AU³/day² — used for both velocity and trail */
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
            /* Offset by parent star's world position */
            p[0] += g_bodies[par_idx].pos[0];
            p[1] += g_bodies[par_idx].pos[1];
            p[2] += g_bodies[par_idx].pos[2];

            /* Trail interval: T/400 in sim-seconds */
            double T_days = 2.0 * PI * sqrt(a * a * a / gm_star_au2);
            double trail_int   = (T_days / 400.0) * DAY;

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
            bo->trail_interval = trail_int;
            read_rotation(bn, bo);
            read_atmosphere(bn, bo);
            alloc_trail(bo);
        }
    }

    /* ================================================================
     * Pass 3 — Moons
     * Parent-relative orbit; parent must be a planet already loaded.
     * ================================================================ */
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

            /* Cache parent state before potential realloc */
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

            /* Trail interval: T/400, minimum 60 s */
            double trail_int = DAY;
            if (a_km > 0.0) {
                double a_m = a_km * 1000.0;
                double T   = 2.0 * PI * sqrt(a_m * a_m * a_m / gm_par);
                trail_int = T / 400.0;
                if (trail_int < 60.0) trail_int = 60.0;
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
            bo->trail_interval = trail_int;
            read_rotation(bn, bo);
            read_atmosphere(bn, bo);
            alloc_trail(bo);
        }
    }

    /* ================================================================
     * Post-processing — per star:
     *   1. Centre-of-mass velocity correction (zero internal momentum).
     *   2. Apply bulk velocity (proper motion) to all bodies in system.
     * ================================================================ */
    int n_stars = 0;
    for (s = 0; s < g_nbodies; s++) {
        if (!g_bodies[s].is_star) continue;
        n_stars++;

        /* CoM correction: adjust star velocity to zero total system momentum.
         * Star mass >> planet masses so adjusting only the star is sufficient. */
        for (i = 0; i < g_nbodies; i++) {
            if (i == s || root_star_of(i) != s) continue;
            g_bodies[s].vel[0] -=
                g_bodies[i].mass * g_bodies[i].vel[0] / g_bodies[s].mass;
            g_bodies[s].vel[1] -=
                g_bodies[i].mass * g_bodies[i].vel[1] / g_bodies[s].mass;
            g_bodies[s].vel[2] -=
                g_bodies[i].mass * g_bodies[i].vel[2] / g_bodies[s].mass;
        }

        /* Apply bulk velocity to all bodies in this system */
        if (bv[s][0] != 0.0 || bv[s][1] != 0.0 || bv[s][2] != 0.0) {
            for (i = 0; i < g_nbodies; i++) {
                if (root_star_of(i) != s) continue;
                g_bodies[i].vel[0] += bv[s][0];
                g_bodies[i].vel[1] += bv[s][1];
                g_bodies[i].vel[2] += bv[s][2];
            }
        }

        /* Log */
        int cnt = 0;
        for (i = 0; i < g_nbodies; i++)
            if (root_star_of(i) == s) cnt++;
        fprintf(stdout,
                "[universe] '%s' at (%.3g, %.3g, %.3g) ly  —  %d bod%s\n",
                g_bodies[s].name,
                g_bodies[s].pos[0] / LY,
                g_bodies[s].pos[1] / LY,
                g_bodies[s].pos[2] / LY,
                cnt, cnt == 1 ? "y" : "ies");
    }

    fprintf(stdout, "[universe] total: %d bodies across %d star%s\n",
            g_nbodies, n_stars, n_stars == 1 ? "" : "s");

    json_free(root);
}

int universe_add_body(const BodyCreateSpec *spec)
{
    if (!spec) return -1;
    if (g_nbodies >= MAX_BODIES) {
        fprintf(stderr, "[universe] cannot add body '%s': MAX_BODIES reached\n",
                spec->name ? spec->name : "unknown");
        return -1;
    }

    ensure_capacity(g_nbodies + 1);
    int idx = g_nbodies++;
    Body *bo = &g_bodies[idx];
    body_defaults(bo);

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
    bo->parent = spec->parent;
    bo->obliquity = spec->obliquity;
    bo->rotation_rate = spec->rotation_rate;
    bo->atm_color[0] = spec->atm_color[0];
    bo->atm_color[1] = spec->atm_color[1];
    bo->atm_color[2] = spec->atm_color[2];
    bo->atm_intensity = spec->atm_intensity;
    bo->atm_scale = spec->atm_scale > 0.0f ? spec->atm_scale : 1.0f;
    bo->trail_interval = trail_interval_for_body(bo);
    alloc_trail(bo);

    return idx;
}

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

void universe_shutdown(void)
{
    int i;
    for (i = 0; i < g_nbodies; i++) {
        free(g_bodies[i].trail);
        g_bodies[i].trail = NULL;
    }
    free(g_bodies);
    g_bodies     = NULL;
    g_nbodies    = 0;
    g_bodies_cap = 0;
}
