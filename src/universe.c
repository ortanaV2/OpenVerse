/*
 * universe.c — data-driven universe loader
 *
 * Reads a universe.json file and populates g_bodies[] / g_nbodies.
 * The JSON must have the structure:
 *
 *   {
 *     "systems": [
 *       {
 *         "name": "Sol",
 *         "bodies": [ { ... }, ... ]
 *       }
 *     ]
 *   }
 *
 * Body types: "star", "planet", "dwarf_planet", "moon"
 *
 * Orbital elements:
 *   planets / dwarf_planets: "keplerian" { a, e, i, Omega, omega_tilde, L }
 *   moons:                   "moon_keplerian" { a_km, e, i_deg, Omega_deg, omega_deg, M0_deg }
 *   stars:                   no orbital elements — pos/vel = {0,0,0}
 *
 * After all bodies are loaded:
 *   - Trail intervals are computed from orbital periods.
 *   - Centre-of-mass correction is applied to the star's velocity.
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
 * ensure_capacity — grow g_bodies if needed to hold at least `needed` bodies.
 * Uses doubling strategy; initial allocation is MAX_BODIES.
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
 * alloc_trail — allocate the trail buffer for a newly added body.
 */
static void alloc_trail(Body *bo)
{
    bo->trail      = (double(*)[3])calloc(TRAIL_LEN, 3 * sizeof(double));
    if (!bo->trail) { fprintf(stderr, "[universe] trail alloc failed\n"); exit(1); }
    bo->trail_head  = 0;
    bo->trail_count = 0;
    bo->trail_accum = 0.0;
}

/*
 * find_body_index — search g_bodies[0..n-1] for body with given name.
 * Returns -1 if not found.
 */
static int find_body_index(const char *name, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        if (strcmp(g_bodies[i].name, name) == 0) return i;
    }
    return -1;
}

/*
 * read_color — read a [r, g, b] JSON array into a float[3].
 * Falls back to {0,0,0} if node is NULL or wrong type.
 */
static void read_color(const JsonNode *arr, float col[3])
{
    col[0] = (float)json_num(json_idx(arr, 0), 0.0);
    col[1] = (float)json_num(json_idx(arr, 1), 0.0);
    col[2] = (float)json_num(json_idx(arr, 2), 0.0);
}

/* ------------------------------------------------------------------ public */

void universe_load(const char *path)
{
    JsonNode *root = json_parse_file(path);
    if (!root) {
        fprintf(stderr, "[universe] failed to open or parse '%s'\n", path);
        exit(1);
    }

    JsonNode *systems = json_get(root, "systems");
    if (!systems || systems->type != JSON_ARRAY) {
        fprintf(stderr, "[universe] 'systems' array not found in '%s'\n", path);
        json_free(root);
        exit(1);
    }

    /* Use the first system */
    JsonNode *system = systems->first_child;
    if (!system) {
        fprintf(stderr, "[universe] no systems defined in '%s'\n", path);
        json_free(root);
        exit(1);
    }

    JsonNode *bodies_arr = json_get(system, "bodies");
    if (!bodies_arr || bodies_arr->type != JSON_ARRAY) {
        fprintf(stderr, "[universe] 'bodies' array not found in system\n");
        json_free(root);
        exit(1);
    }

    g_nbodies = 0;

    /* ----------------------------------------------------------------
     * First pass: create all bodies.
     * Moons need their parent to already exist in g_bodies, so we
     * process stars first, then planets/dwarf_planets, then moons.
     * Since the JSON is ordered (stars before planets before moons)
     * a single forward pass works fine — but we explicitly sort by
     * type priority to be safe.
     *
     * Priority: star(0) -> planet(1) -> dwarf_planet(2) -> moon(3)
     * ---------------------------------------------------------------- */

    /* We do two passes: non-moons first, then moons. */

    /* Pass 1 — stars, planets, dwarf_planets */
    {
        JsonNode *bnode = bodies_arr->first_child;
        while (bnode) {
            const char *type = json_str(json_get(bnode, "type"), "");

            int is_moon = (strcmp(type, "moon") == 0);
            if (!is_moon) {
                const char *name  = json_str(json_get(bnode, "name"), "unknown");
                double      mass  = json_num(json_get(bnode, "mass"), 0.0);
                double      rad_km = json_num(json_get(bnode, "radius_km"), 1.0);
                float       col[3];
                read_color(json_get(bnode, "color"), col);

                int is_star = (strcmp(type, "star") == 0);

                double p[3] = {0.0, 0.0, 0.0};
                double v[3] = {0.0, 0.0, 0.0};

                if (!is_star) {
                    /* planet or dwarf_planet — use keplerian elements */
                    JsonNode *kep = json_get(bnode, "keplerian");
                    if (kep) {
                        double a            = json_num(json_get(kep, "a"),            1.0);
                        double e            = json_num(json_get(kep, "e"),            0.0);
                        double i_deg        = json_num(json_get(kep, "i"),            0.0);
                        double Omega_deg    = json_num(json_get(kep, "Omega"),        0.0);
                        double omega_tilde  = json_num(json_get(kep, "omega_tilde"),  0.0);
                        double L            = json_num(json_get(kep, "L"),            0.0);
                        keplerian_to_state(a, e, i_deg, Omega_deg, omega_tilde, L, p, v);
                    }
                }

                /* add to g_bodies */
                {
                    ensure_capacity(g_nbodies + 1);
                    Body *bo = &g_bodies[g_nbodies++];
                    strncpy(bo->name, name, 31);
                    bo->name[31] = '\0';
                    bo->mass            = mass;
                    bo->radius          = rad_km * 1000.0;
                    bo->pos[0] = p[0]; bo->pos[1] = p[1]; bo->pos[2] = p[2];
                    bo->vel[0] = v[0]; bo->vel[1] = v[1]; bo->vel[2] = v[2];
                    bo->acc[0] = bo->acc[1] = bo->acc[2] = 0.0;
                    bo->fast_acc[0] = bo->fast_acc[1] = bo->fast_acc[2] = 0.0;
                    bo->col[0] = col[0]; bo->col[1] = col[1]; bo->col[2] = col[2];
                    bo->is_star         = is_star;
                    bo->parent          = -1;
                    bo->obliquity       = 0.0;
                    bo->rotation_rate   = 0.0;
                    bo->rotation_angle  = 0.0;
                    bo->trail_interval = DAY;   /* overwritten below for planets */
                    alloc_trail(bo);

                    /* atmosphere defaults */
                    bo->atm_color[0] = bo->atm_color[1] = bo->atm_color[2] = 0.0f;
                    bo->atm_intensity = 0.0f;
                    bo->atm_scale     = 1.0f;

                    /* obliquity / rotation */
                    JsonNode *obl = json_get(bnode, "obliquity_deg");
                    JsonNode *rot = json_get(bnode, "rotation_period_days");
                    if (obl) bo->obliquity = json_num(obl, 0.0);
                    if (rot && json_num(rot, 0.0) != 0.0)
                        bo->rotation_rate = (2.0 * PI) / (json_num(rot, 1.0) * DAY);

                    /* atmosphere */
                    JsonNode *atm = json_get(bnode, "atmosphere");
                    if (atm) {
                        float acol[3];
                        read_color(json_get(atm, "color"), acol);
                        bo->atm_color[0] = acol[0];
                        bo->atm_color[1] = acol[1];
                        bo->atm_color[2] = acol[2];
                        bo->atm_intensity = (float)json_num(json_get(atm, "intensity"), 0.0);
                        bo->atm_scale     = (float)json_num(json_get(atm, "scale"),     1.0);
                    }
                }
            }

            bnode = bnode->next;
        }
    }

    /* Pass 2 — moons */
    {
        JsonNode *bnode = bodies_arr->first_child;
        while (bnode) {
            const char *type = json_str(json_get(bnode, "type"), "");

            if (strcmp(type, "moon") == 0) {
                const char *name      = json_str(json_get(bnode, "name"), "unknown");
                double      mass      = json_num(json_get(bnode, "mass"), 0.0);
                double      rad_km    = json_num(json_get(bnode, "radius_km"), 1.0);
                const char *par_name  = json_str(json_get(bnode, "parent"), "");
                float       col[3];
                read_color(json_get(bnode, "color"), col);

                int par_idx = find_body_index(par_name, g_nbodies);
                if (par_idx < 0) {
                    fprintf(stderr, "[universe] moon '%s': parent '%s' not found\n",
                            name, par_name);
                    json_free(root);
                    exit(1);
                }

                /* Save parent values now — ensure_capacity below may realloc,
                 * invalidating any Body* pointer into g_bodies.             */
                double gm_parent  = G_CONST * g_bodies[par_idx].mass;
                double par_pos[3] = { g_bodies[par_idx].pos[0],
                                      g_bodies[par_idx].pos[1],
                                      g_bodies[par_idx].pos[2] };
                double par_vel[3] = { g_bodies[par_idx].vel[0],
                                      g_bodies[par_idx].vel[1],
                                      g_bodies[par_idx].vel[2] };

                JsonNode *mk = json_get(bnode, "moon_keplerian");
                double rel_p[3] = {0,0,0}, rel_v[3] = {0,0,0};
                double a_km = 0.0;
                if (mk) {
                    a_km            = json_num(json_get(mk, "a_km"),     0.0);
                    double e        = json_num(json_get(mk, "e"),         0.0);
                    double i_deg    = json_num(json_get(mk, "i_deg"),     0.0);
                    double Omega_deg = json_num(json_get(mk, "Omega_deg"),0.0);
                    double omega_deg = json_num(json_get(mk, "omega_deg"),0.0);
                    double M0_deg   = json_num(json_get(mk, "M0_deg"),    0.0);
                    moon_to_state(a_km, e, i_deg, Omega_deg, omega_deg,
                                  M0_deg, gm_parent, rel_p, rel_v);
                }

                double p[3] = { par_pos[0] + rel_p[0],
                                par_pos[1] + rel_p[1],
                                par_pos[2] + rel_p[2] };
                double v[3] = { par_vel[0] + rel_v[0],
                                par_vel[1] + rel_v[1],
                                par_vel[2] + rel_v[2] };

                ensure_capacity(g_nbodies + 1);
                Body *bo = &g_bodies[g_nbodies++];
                strncpy(bo->name, name, 31);
                bo->name[31] = '\0';
                bo->mass            = mass;
                bo->radius          = rad_km * 1000.0;
                bo->pos[0] = p[0]; bo->pos[1] = p[1]; bo->pos[2] = p[2];
                bo->vel[0] = v[0]; bo->vel[1] = v[1]; bo->vel[2] = v[2];
                bo->acc[0] = bo->acc[1] = bo->acc[2] = 0.0;
                bo->fast_acc[0] = bo->fast_acc[1] = bo->fast_acc[2] = 0.0;
                bo->col[0] = col[0]; bo->col[1] = col[1]; bo->col[2] = col[2];
                bo->is_star         = 0;
                bo->parent          = par_idx;
                bo->obliquity       = 0.0;
                bo->rotation_rate   = 0.0;
                bo->rotation_angle  = 0.0;
                alloc_trail(bo);

                /* atmosphere defaults */
                bo->atm_color[0] = bo->atm_color[1] = bo->atm_color[2] = 0.0f;
                bo->atm_intensity = 0.0f;
                bo->atm_scale     = 1.0f;

                /* Trail interval: T/200, floored at 60 s */
                {
                    double a_m = a_km * 1000.0;
                    double T   = 2.0 * PI * sqrt(a_m * a_m * a_m / gm_parent);
                    bo->trail_interval = T / 200.0;
                    if (bo->trail_interval < 60.0) bo->trail_interval = 60.0;
                }

                /* obliquity / rotation */
                JsonNode *obl = json_get(bnode, "obliquity_deg");
                JsonNode *rot = json_get(bnode, "rotation_period_days");
                if (obl) bo->obliquity = json_num(obl, 0.0);
                if (rot && json_num(rot, 0.0) != 0.0)
                    bo->rotation_rate = (2.0 * PI) / (json_num(rot, 1.0) * DAY);

                /* atmosphere */
                JsonNode *atm = json_get(bnode, "atmosphere");
                if (atm) {
                    float acol[3];
                    read_color(json_get(atm, "color"), acol);
                    bo->atm_color[0] = acol[0];
                    bo->atm_color[1] = acol[1];
                    bo->atm_color[2] = acol[2];
                    bo->atm_intensity = (float)json_num(json_get(atm, "intensity"), 0.0);
                    bo->atm_scale     = (float)json_num(json_get(atm, "scale"),     1.0);
                }
            }

            bnode = bnode->next;
        }
    }

    /* ----------------------------------------------------------------
     * Trail intervals for planets / dwarf_planets.
     * Formula: interval = (T_days / 200) * DAY  →  200 samples/orbit.
     * ---------------------------------------------------------------- */
    {
        JsonNode *bnode = bodies_arr->first_child;
        while (bnode) {
            const char *type = json_str(json_get(bnode, "type"), "");
            int is_planet = (strcmp(type, "planet") == 0 ||
                             strcmp(type, "dwarf_planet") == 0);
            if (is_planet) {
                const char *name = json_str(json_get(bnode, "name"), "");
                int idx = find_body_index(name, g_nbodies);
                if (idx >= 0) {
                    JsonNode *kep = json_get(bnode, "keplerian");
                    if (kep) {
                        double a      = json_num(json_get(kep, "a"), 1.0);
                        double T_days = 2.0 * PI * sqrt(a * a * a / GM_SUN);
                        g_bodies[idx].trail_interval = (T_days / 200.0) * DAY;
                    }
                }
            }
            bnode = bnode->next;
        }
    }

    /* Star: use a slow interval so it barely records */
    if (g_nbodies > 0 && g_bodies[0].is_star) {
        g_bodies[0].trail_interval = DAY * 25.0;
    }

    /* ----------------------------------------------------------------
     * Centre-of-mass correction:
     * Adjust the star's velocity so total system momentum = 0.
     * ---------------------------------------------------------------- */
    if (g_nbodies > 0 && g_bodies[0].is_star) {
        int i;
        for (i = 1; i < g_nbodies; i++) {
            g_bodies[0].vel[0] -= g_bodies[i].mass * g_bodies[i].vel[0] / g_bodies[0].mass;
            g_bodies[0].vel[1] -= g_bodies[i].mass * g_bodies[i].vel[1] / g_bodies[0].mass;
            g_bodies[0].vel[2] -= g_bodies[i].mass * g_bodies[i].vel[2] / g_bodies[0].mass;
        }
    }

    json_free(root);
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
