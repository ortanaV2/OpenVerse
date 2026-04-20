/*
 * build.c — first-pass runtime body placement mode
 */
/*
 * build.c - first-pass runtime body placement mode
 */
#include "build.h"
#include "body.h"
#include "camera.h"
#include "physics.h"
#include "universe.h"
#include "trails.h"
#include "labels.h"
#include <float.h>

int g_build_mode = 0;
int g_build_tab_held = 0;

static int s_selected = 0;
static int s_prev_paused = 0;
static int s_place_serial = 1;

#define BUILD_REF_LOCAL_AU        0.018
#define BUILD_REF_INTERSTELLAR_AU 1200.0
#define BUILD_PREVIEW_TARGET_PX   18.0
#define BUILD_PREVIEW_MIN_AU      0.00035
#define BUILD_PREVIEW_MAX_AU      18.0

static const BuildPreset s_presets[] = {
    { "Rocky",    5.972e24,  6371.0e3,  {0.32f, 0.58f, 0.92f}, 0, 1, 0, {0.45f, 0.65f, 1.00f}, 0.45f, 1.25f },
    { "Gas Giant",1.898e27, 71492.0e3,  {0.86f, 0.68f, 0.46f}, 0, 1, 0, {0.90f, 0.72f, 0.50f}, 0.25f, 1.22f },
    { "Ice World",8.681e25,25559.0e3,  {0.54f, 0.86f, 0.96f}, 0, 1, 0, {0.62f, 0.90f, 1.00f}, 0.22f, 1.22f },
    { "Moon",     7.342e22, 1737.4e3,  {0.72f, 0.72f, 0.68f}, 0, 0, 1, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f },
    { "Dwarf",    1.309e22, 1188.3e3,  {0.68f, 0.63f, 0.58f}, 0, 1, 0, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f },
    { "Star",     1.989e30,696000.0e3, {1.00f, 0.92f, 0.28f}, 1, 0, 0, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f }
};

void build_init(void)
{
    s_selected = 0;
}

int build_preset_count(void)
{
    return (int)(sizeof(s_presets) / sizeof(s_presets[0]));
}

int build_selected_index(void)
{
    return s_selected;
}

const BuildPreset *build_preset_at(int idx)
{
    if (idx < 0 || idx >= build_preset_count()) return NULL;
    return &s_presets[idx];
}

const BuildPreset *build_current_preset(void)
{
    return build_preset_at(s_selected);
}

static int reference_too_similar(int cand, double cand_dist,
                                 const int idx[3], const double best[3],
                                 int mode)
{
    double ratio = (mode == 0) ? 0.08 : 0.015;

    for (int i = 0; i < 3; i++) {
        int other = idx[i];
        if (other < 0) continue;

        double dx = g_bodies[cand].pos[0] - g_bodies[other].pos[0];
        double dy = g_bodies[cand].pos[1] - g_bodies[other].pos[1];
        double dz = g_bodies[cand].pos[2] - g_bodies[other].pos[2];
        double sep = sqrt(dx*dx + dy*dy + dz*dz);
        double ref = cand_dist < best[i] ? cand_dist : best[i];
        if (ref > 0.0 && sep / ref < ratio) return 1;
    }

    return 0;
}

void build_toggle(void)
{
    g_build_mode = !g_build_mode;
    if (g_build_mode) {
        s_prev_paused = g_paused;
        g_paused = 1;
    } else {
        g_build_tab_held = 0;
        g_paused = s_prev_paused;
    }
}

void build_set_tab_held(int held)
{
    g_build_tab_held = held ? 1 : 0;
}

void build_scroll(int wheel_y)
{
    if (!g_build_mode || !g_build_tab_held || wheel_y == 0) return;
    int n = build_preset_count();
    s_selected += (wheel_y > 0) ? -1 : 1;
    while (s_selected < 0) s_selected += n;
    while (s_selected >= n) s_selected -= n;
}

void build_preview_pos_au(double out[3])
{
    float dx, dy, dz;
    const BuildPreset *preset = build_current_preset();
    double radius_au = preset ? preset->radius * RS : 6371.0e3 * RS;
    double dist = ((double)WIN_H * 0.5 * radius_au)
                / (BUILD_PREVIEW_TARGET_PX * tan(FOV * 0.5 * PI / 180.0));
    if (dist < BUILD_PREVIEW_MIN_AU) dist = BUILD_PREVIEW_MIN_AU;
    if (dist > BUILD_PREVIEW_MAX_AU) dist = BUILD_PREVIEW_MAX_AU;

    cam_get_dir(&dx, &dy, &dz);
    out[0] = g_cam.pos[0] + (double)dx * dist;
    out[1] = g_cam.pos[1] + (double)dy * dist;
    out[2] = g_cam.pos[2] + (double)dz * dist;
}

void build_preview_pos_m(double out[3])
{
    double p[3];
    build_preview_pos_au(p);
    out[0] = p[0] * AU;
    out[1] = p[1] * AU;
    out[2] = p[2] * AU;
}

void build_nearest3(const double pos_m[3], int out_idx[3], double out_dist_au[3])
{
    int mode = 1; /* 0=stars, 1=primaries, 2=local bodies */
    int nearest_nonstar = -1;
    double nearest_nonstar_dist = DBL_MAX;
    double nearest_star_dist = DBL_MAX;

    for (int k = 0; k < 3; k++) {
        out_idx[k] = -1;
        out_dist_au[k] = 0.0;
    }

    for (int i = 0; i < g_nbodies; i++) {
        double dx = g_bodies[i].pos[0] - pos_m[0];
        double dy = g_bodies[i].pos[1] - pos_m[1];
        double dz = g_bodies[i].pos[2] - pos_m[2];
        double d2 = dx*dx + dy*dy + dz*dz;
        double d = sqrt(d2);
        if (g_bodies[i].is_star) {
            if (d < nearest_star_dist) nearest_star_dist = d;
        } else if (d < nearest_nonstar_dist) {
            nearest_nonstar_dist = d;
            nearest_nonstar = i;
        }
    }

    if (nearest_star_dist / AU > BUILD_REF_INTERSTELLAR_AU)
        mode = 0;
    else if (nearest_nonstar_dist / AU < BUILD_REF_LOCAL_AU)
        mode = 2;

    for (int pass = 0; pass < 3; pass++) {
        double best[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
        int idx[3] = {-1, -1, -1};

        for (int i = 0; i < g_nbodies; i++) {
            int is_moon = (g_bodies[i].parent >= 0 &&
                           !g_bodies[g_bodies[i].parent].is_star);
            int accept = 0;

            if (mode == 0)
                accept = g_bodies[i].is_star;
            else if (mode == 1)
                accept = !is_moon;
            else {
                if (pass == 0 && nearest_nonstar >= 0) {
                    int parent = g_bodies[nearest_nonstar].parent;
                    accept = (i == nearest_nonstar ||
                              g_bodies[i].parent == nearest_nonstar ||
                              (parent >= 0 && (i == parent || g_bodies[i].parent == parent)));
                } else {
                    accept = !g_bodies[i].is_star;
                }
            }

            if (!accept) continue;

            double dx = g_bodies[i].pos[0] - pos_m[0];
            double dy = g_bodies[i].pos[1] - pos_m[1];
            double dz = g_bodies[i].pos[2] - pos_m[2];
            double d = sqrt(dx*dx + dy*dy + dz*dz);
            if (reference_too_similar(i, d, idx, best, mode)) continue;
            for (int k = 0; k < 3; k++) {
                if (d >= best[k]) continue;
                for (int m = 2; m > k; m--) {
                    best[m] = best[m-1];
                    idx[m] = idx[m-1];
                }
                best[k] = d;
                idx[k] = i;
                break;
            }
        }

        for (int k = 0; k < 3; k++) {
            if (out_idx[k] >= 0 || idx[k] < 0) continue;
            int used = 0;
            for (int u = 0; u < 3; u++)
                if (out_idx[u] == idx[k]) used = 1;
            if (used) continue;
            out_idx[k] = idx[k];
            out_dist_au[k] = best[k] / AU;
        }

        int filled = 1;
        for (int k = 0; k < 3; k++)
            if (out_idx[k] < 0) filled = 0;
        if (filled) break;

        if (mode == 0) mode = 1;
        else if (mode == 1) mode = 2;
        else mode = 1;
        if (pass == 2) break;
    }

    for (int k = 0; k < 3; k++) {
        if (out_idx[k] >= 0) continue;
        double best = DBL_MAX;
        int best_idx = -1;
        for (int i = 0; i < g_nbodies; i++) {
            int used = 0;
            for (int u = 0; u < 3; u++)
                if (out_idx[u] == i) used = 1;
            if (used) continue;
            double dx = g_bodies[i].pos[0] - pos_m[0];
            double dy = g_bodies[i].pos[1] - pos_m[1];
            double dz = g_bodies[i].pos[2] - pos_m[2];
            double d = sqrt(dx*dx + dy*dy + dz*dz);
            if (d < best) {
                best = d;
                best_idx = i;
            }
        }
        if (best_idx >= 0) {
            out_idx[k] = best_idx;
            out_dist_au[k] = best / AU;
        }
    }
}

static int nearest_star(const double pos_m[3])
{
    int best_idx = -1;
    double best_d2 = DBL_MAX;
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].is_star) continue;
        double dx = g_bodies[i].pos[0] - pos_m[0];
        double dy = g_bodies[i].pos[1] - pos_m[1];
        double dz = g_bodies[i].pos[2] - pos_m[2];
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

static int nearest_nonstar(const double pos_m[3])
{
    int best_idx = -1;
    double best_d2 = DBL_MAX;
    for (int i = 0; i < g_nbodies; i++) {
        if (g_bodies[i].is_star) continue;
        double dx = g_bodies[i].pos[0] - pos_m[0];
        double dy = g_bodies[i].pos[1] - pos_m[1];
        double dz = g_bodies[i].pos[2] - pos_m[2];
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

static void add_parent_velocity(BodyCreateSpec *spec, int parent)
{
    if (parent < 0) return;

    spec->vel[0] = g_bodies[parent].vel[0];
    spec->vel[1] = g_bodies[parent].vel[1];
    spec->vel[2] = g_bodies[parent].vel[2];

    double rx = spec->pos[0] - g_bodies[parent].pos[0];
    double ry = spec->pos[1] - g_bodies[parent].pos[1];
    double rz = spec->pos[2] - g_bodies[parent].pos[2];
    double r = sqrt(rx*rx + ry*ry + rz*rz);
    double gm = G_CONST * g_bodies[parent].mass;
    if (r <= 0.0 || gm <= 0.0) return;

    double up[3] = {0.0, 1.0, 0.0};
    double tx = up[1]*rz - up[2]*ry;
    double ty = up[2]*rx - up[0]*rz;
    double tz = up[0]*ry - up[1]*rx;
    double tl = sqrt(tx*tx + ty*ty + tz*tz);
    if (tl < 1e-9) {
        float fdx, fdy, fdz;
        cam_get_dir(&fdx, &fdy, &fdz);
        tx = (double)fdz;
        ty = 0.0;
        tz = -(double)fdx;
        tl = sqrt(tx*tx + ty*ty + tz*tz);
    }
    if (tl <= 0.0) return;

    double v = sqrt(gm / r);
    spec->vel[0] += tx / tl * v;
    spec->vel[1] += ty / tl * v;
    spec->vel[2] += tz / tl * v;
}

int build_place_current(void)
{
    if (!g_build_mode || g_nbodies >= MAX_BODIES) return -1;

    const BuildPreset *p = build_current_preset();
    if (!p) return -1;

    BodyCreateSpec spec;
    memset(&spec, 0, sizeof(spec));

    static char name[32];
    snprintf(name, sizeof(name), "%s %d", p->name, s_place_serial++);
    spec.name = name;
    spec.mass = p->mass;
    spec.radius = p->radius;
    build_preview_pos_m(spec.pos);
    spec.col[0] = p->col[0];
    spec.col[1] = p->col[1];
    spec.col[2] = p->col[2];
    spec.is_star = p->is_star;
    spec.parent = -1;
    spec.obliquity = p->is_star ? 7.25 : 23.4;
    spec.rotation_rate = p->is_star ? (2.0 * PI) / (25.0 * DAY)
                                    : (2.0 * PI) / DAY;
    spec.atm_color[0] = p->atm_color[0];
    spec.atm_color[1] = p->atm_color[1];
    spec.atm_color[2] = p->atm_color[2];
    spec.atm_intensity = p->atm_intensity;
    spec.atm_scale = p->atm_scale;

    if (!p->is_star) {
        if (p->wants_nonstar_parent)
            spec.parent = nearest_nonstar(spec.pos);
        if (spec.parent < 0 && p->wants_planet_parent)
            spec.parent = nearest_star(spec.pos);
        if (spec.parent < 0)
            spec.parent = nearest_star(spec.pos);
        add_parent_velocity(&spec, spec.parent);
    }

    int idx = universe_add_body(&spec);
    if (idx >= 0) {
        trails_add_body(idx);
        labels_add_body(idx);
        fprintf(stdout, "[Build] placed '%s' at %.3f %.3f %.3f AU\n",
                g_bodies[idx].name,
                g_bodies[idx].pos[0] / AU,
                g_bodies[idx].pos[1] / AU,
                g_bodies[idx].pos[2] / AU);
    }
    return idx;
}
