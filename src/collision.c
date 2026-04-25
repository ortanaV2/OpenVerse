/*
 * collision.c - first-pass solid-body impacts.
 *
 * Keeps body indices stable by marking absorbed bodies as !alive. The visual
 * side is deliberately lightweight: an impact creates a local hot spot that
 * the planet shader fades over time.
 */
#include "collision.h"
#include "body.h"
#include "labels.h"
#include "rings.h"
#include "trails.h"
#include <math.h>
#include <stdio.h>

#define MAX_IMPACTS 64
#define MAX_PAIR_DT (DAY * 2.0)
#define MIN_PAIR_DT (60.0 * 5.0)
#define HOT_PAIR_DT (60.0 * 10.0)
#define SYSTEM_HOT_DURATION (DAY * 3.0)
#define MIN_COLLISION_SPEED 1.0
#define SYSTEM_MARGIN_AU 5.0
#define IMPACT_COOL_SECONDS (DAY * 45.0)
#define MAJOR_COOL_SECONDS (DAY * 90.0)
#define MERGE_COOL_SECONDS (DAY * 180.0)
#define MERGE_PHASE_SECONDS (DAY * 8.0)
#define MAX_MERGES 16
#define MAX_PERSISTENT_SCARS 128
#define MAX_COLLISION_PARTICLES 768

typedef struct {
    int active;
    int body;
    int kind;
    double age;
    double duration;
    float dir[3];
    float tangent1[3];
    float radius0;
    float radius1;
    float heat0;
} ImpactEvent;

typedef struct {
    int active;
    int body;
    unsigned int stamp;
    float dir[3];
    float tangent1[3];
    float angular_radius;
    float depth;
    float seed;
} PersistentScar;

typedef struct {
    int active;
    double age;
    double duration;
    double start_radius;
    double target_radius;
} RadiusTransition;

typedef struct {
    int active;
    int target;
    int impactor;
    double age;
    double duration;
    double rel_speed;
    double dir[3];
    double rel_vel[3];
    double target_local_attach_dir[3];
    double target_local_attach_t1[3];
    double start_sep;
    double particle_emit_accum;
    double particle_emit_next;
    double target_rotation_rate;
    double impactor_rotation_rate;
    double target_obliquity;
    int scar_slot;          /* s_impacts index for the target intersection scar */
    int imp_scar_slot;      /* s_impacts index for the impactor intersection scar */
} MergeEvent;

typedef struct {
    int active;
    double age;
    double duration;
    double pos[3];
    double vel[3];
    float color[4];
    float size;
    float fade_start; /* normalized lifetime fraction at which alpha fade begins */
} ImpactParticleState;

static ImpactEvent s_impacts[MAX_IMPACTS];
static PersistentScar s_perm_scars[MAX_PERSISTENT_SCARS];
static RadiusTransition s_radius_fx[MAX_BODIES];
static MergeEvent s_merges[MAX_MERGES];
static ImpactParticleState s_particles[MAX_COLLISION_PARTICLES];
static double s_pair_next[MAX_BODIES][MAX_BODIES];
static unsigned char s_system_dirty[MAX_BODIES];
static double s_system_hot[MAX_BODIES];
static unsigned int s_particle_rng = 0x1234abcdu;
static unsigned int s_perm_scar_stamp = 1u;
static double s_pos_before[MAX_BODIES][3];
static int s_pos_before_valid = 0;

static void mark_system_dirty(int root, double hot_duration)
{
    if (root < 0 || root >= MAX_BODIES) return;
    s_system_dirty[root] = 1;
    if (hot_duration > s_system_hot[root]) s_system_hot[root] = hot_duration;
}

void collision_snapshot_positions(void)
{
    int n = g_nbodies < MAX_BODIES ? g_nbodies : MAX_BODIES;
    for (int i = 0; i < n; i++) {
        s_pos_before[i][0] = g_bodies[i].pos[0];
        s_pos_before[i][1] = g_bodies[i].pos[1];
        s_pos_before[i][2] = g_bodies[i].pos[2];
    }
    s_pos_before_valid = 1;
}

void collision_on_body_added(int body_idx)
{
    int root = body_root_star(body_idx);
    if (root >= 0) mark_system_dirty(root, SYSTEM_HOT_DURATION);
}

static int body_is_descendant_of(int body_idx, int ancestor_idx)
{
    if (body_idx < 0 || ancestor_idx < 0) return 0;
    for (int p = g_bodies[body_idx].parent; p >= 0; p = g_bodies[p].parent)
        if (p == ancestor_idx) return 1;
    return 0;
}

static double dot3d(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static int body_is_in_merge(int idx)
{
    for (int i = 0; i < MAX_MERGES; i++) {
        if (!s_merges[i].active) continue;
        if (s_merges[i].target == idx || s_merges[i].impactor == idx) return 1;
    }
    return 0;
}

static int body_is_merge_target(int idx)
{
    for (int i = 0; i < MAX_MERGES; i++) {
        if (!s_merges[i].active) continue;
        if (s_merges[i].target == idx) return 1;
    }
    return 0;
}

static int body_is_merge_impactor(int idx)
{
    for (int i = 0; i < MAX_MERGES; i++) {
        if (!s_merges[i].active) continue;
        if (s_merges[i].impactor == idx) return 1;
    }
    return 0;
}

static void normalize3f(float v[3])
{
    float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (len <= 1e-8f) {
        v[0] = 1.0f; v[1] = 0.0f; v[2] = 0.0f;
        return;
    }
    v[0] /= len; v[1] /= len; v[2] /= len;
}

static double rand01(void)
{
    s_particle_rng = 1664525u * s_particle_rng + 1013904223u;
    return (double)(s_particle_rng & 0x00ffffffu) / (double)0x01000000u;
}

static float rand01f(void)
{
    return (float)rand01();
}

static void orthonormal_basis(const double n[3], double t1[3], double t2[3])
{
    double ref[3] = {0.0, 1.0, 0.0};
    double len;

    if (fabs(n[1]) > 0.9) {
        ref[0] = 1.0;
        ref[1] = 0.0;
        ref[2] = 0.0;
    }

    t1[0] = n[1]*ref[2] - n[2]*ref[1];
    t1[1] = n[2]*ref[0] - n[0]*ref[2];
    t1[2] = n[0]*ref[1] - n[1]*ref[0];
    len = sqrt(dot3d(t1, t1));
    if (len <= 1e-12) {
        t1[0] = 1.0; t1[1] = 0.0; t1[2] = 0.0;
    } else {
        t1[0] /= len; t1[1] /= len; t1[2] /= len;
    }

    t2[0] = n[1]*t1[2] - n[2]*t1[1];
    t2[1] = n[2]*t1[0] - n[0]*t1[2];
    t2[2] = n[0]*t1[1] - n[1]*t1[0];
    len = sqrt(dot3d(t2, t2));
    if (len <= 1e-12) {
        t2[0] = 0.0; t2[1] = 0.0; t2[2] = 1.0;
    } else {
        t2[0] /= len; t2[1] /= len; t2[2] /= len;
    }
}

static void normalize3d(double v[3])
{
    double len = sqrt(dot3d(v, v));
    if (len <= 1e-12) {
        v[0] = 1.0; v[1] = 0.0; v[2] = 0.0;
        return;
    }
    v[0] /= len; v[1] /= len; v[2] /= len;
}

static void body_local_surface_dir_to_world(int body_idx, const double local_dir[3],
                                            double out[3])
{
    Body *b;
    double cr, sr, co, so;
    double tx, ty, tz;
    double len;

    if (!out || body_idx < 0 || body_idx >= g_nbodies) return;
    b = &g_bodies[body_idx];

    cr = cos(b->rotation_angle);
    sr = sin(b->rotation_angle);
    tx = local_dir[0] * cr - local_dir[2] * sr;
    ty = local_dir[1];
    tz = local_dir[0] * sr + local_dir[2] * cr;

    co = cos(b->obliquity * PI / 180.0);
    so = sin(b->obliquity * PI / 180.0);
    out[0] =  co * tx + so * ty;
    out[1] = -so * tx + co * ty;
    out[2] =  tz;

    len = sqrt(dot3d(out, out));
    if (len <= 1e-12) {
        out[0] = 1.0; out[1] = 0.0; out[2] = 0.0;
        return;
    }
    out[0] /= len; out[1] /= len; out[2] /= len;
}

static void build_local_scar_tangent(int body_idx, const double world_dir[3],
                                     const double rel_vel[3], float out_local_t1[3])
{
    double world_t1[3];
    double fallback_t1[3], fallback_t2[3];
    double dotn;

    if (!out_local_t1) return;
    if (!world_dir) {
        out_local_t1[0] = 1.0f;
        out_local_t1[1] = 0.0f;
        out_local_t1[2] = 0.0f;
        return;
    }

    if (rel_vel) {
        double len;
        dotn = dot3d(rel_vel, world_dir);
        world_t1[0] = rel_vel[0] - world_dir[0] * dotn;
        world_t1[1] = rel_vel[1] - world_dir[1] * dotn;
        world_t1[2] = rel_vel[2] - world_dir[2] * dotn;
        len = sqrt(dot3d(world_t1, world_t1));
        if (len > 1e-12) {
            world_t1[0] /= len;
            world_t1[1] /= len;
            world_t1[2] /= len;
        } else {
            orthonormal_basis(world_dir, fallback_t1, fallback_t2);
            world_t1[0] = fallback_t1[0];
            world_t1[1] = fallback_t1[1];
            world_t1[2] = fallback_t1[2];
        }
    } else {
        orthonormal_basis(world_dir, fallback_t1, fallback_t2);
        world_t1[0] = fallback_t1[0];
        world_t1[1] = fallback_t1[1];
        world_t1[2] = fallback_t1[2];
    }

    body_world_to_local_surface_dir(body_idx, world_t1, out_local_t1);
    normalize3f(out_local_t1);
}

static void add_permanent_crater(int body_idx, const double world_dir[3],
                                 const double rel_vel[3], double impactor_radius,
                                 double strength);

static double body_moment_of_inertia(const Body *b)
{
    if (!b) return 0.0;
    return 0.40 * b->mass * b->radius * b->radius;
}

static void spin_axis_from_obliquity(double obliquity_deg, double axis[3])
{
    double ob = obliquity_deg * (PI / 180.0);
    axis[0] = sin(ob);
    axis[1] = cos(ob);
    axis[2] = 0.0;
}

static double obliquity_from_spin_axis(const double axis[3])
{
    return atan2(axis[0], axis[1]) * (180.0 / PI);
}

static void compute_collision_spin_state(int target, int impactor,
                                         const double contact_dir[3],
                                         const double rel_vel[3],
                                         double *out_obliquity,
                                         double *out_rotation_rate)
{
    const Body *a = &g_bodies[target];
    const Body *b = &g_bodies[impactor];
    double axis_a[3], axis_b[3], merged_axis[3];
    double L_spin_a[3], L_spin_b[3], L_orbit[3], L_total[3];
    double contact_r[3], merged_I, planar_len, axis_dot;

    spin_axis_from_obliquity(a->obliquity, axis_a);
    spin_axis_from_obliquity(b->obliquity, axis_b);

    {
        double Ia = body_moment_of_inertia(a);
        double Ib = body_moment_of_inertia(b);
        L_spin_a[0] = axis_a[0] * Ia * a->rotation_rate;
        L_spin_a[1] = axis_a[1] * Ia * a->rotation_rate;
        L_spin_a[2] = axis_a[2] * Ia * a->rotation_rate;
        L_spin_b[0] = axis_b[0] * Ib * b->rotation_rate;
        L_spin_b[1] = axis_b[1] * Ib * b->rotation_rate;
        L_spin_b[2] = axis_b[2] * Ib * b->rotation_rate;
    }

    contact_r[0] = contact_dir[0] * a->radius;
    contact_r[1] = contact_dir[1] * a->radius;
    contact_r[2] = contact_dir[2] * a->radius;

    L_orbit[0] = b->mass * (contact_r[1] * rel_vel[2] - contact_r[2] * rel_vel[1]);
    L_orbit[1] = b->mass * (contact_r[2] * rel_vel[0] - contact_r[0] * rel_vel[2]);
    L_orbit[2] = b->mass * (contact_r[0] * rel_vel[1] - contact_r[1] * rel_vel[0]);

    L_total[0] = L_spin_a[0] + L_spin_b[0] + L_orbit[0];
    L_total[1] = L_spin_a[1] + L_spin_b[1] + L_orbit[1];
    L_total[2] = L_spin_a[2] + L_spin_b[2] + L_orbit[2];

    planar_len = sqrt(L_total[0]*L_total[0] + L_total[1]*L_total[1]);
    if (planar_len <= 1e-12) {
        merged_axis[0] = axis_a[0];
        merged_axis[1] = axis_a[1];
        merged_axis[2] = 0.0;
        planar_len = sqrt(merged_axis[0]*merged_axis[0] + merged_axis[1]*merged_axis[1]);
        if (planar_len <= 1e-12) {
            merged_axis[0] = 0.0;
            merged_axis[1] = 1.0;
        } else {
            merged_axis[0] /= planar_len;
            merged_axis[1] /= planar_len;
        }
        L_total[0] = merged_axis[0] * body_moment_of_inertia(a) * a->rotation_rate;
        L_total[1] = merged_axis[1] * body_moment_of_inertia(a) * a->rotation_rate;
    } else {
        merged_axis[0] = L_total[0] / planar_len;
        merged_axis[1] = L_total[1] / planar_len;
        merged_axis[2] = 0.0;
    }

    /* Axis direction and spin-rate sign are interchangeable in the current
     * obliquity+rotation_rate model. Keep the chosen axis close to the
     * target's pre-impact axis so the resulting rotation can be signed
     * correctly instead of always collapsing to a positive spin. */
    axis_dot = merged_axis[0] * axis_a[0] + merged_axis[1] * axis_a[1];
    if (axis_dot < 0.0) {
        merged_axis[0] = -merged_axis[0];
        merged_axis[1] = -merged_axis[1];
    }

    merged_I = 0.40 * (a->mass + b->mass)
             * pow(cbrt(a->radius*a->radius*a->radius + b->radius*b->radius*b->radius), 2.0);
    if (merged_I <= 1e-12) merged_I = body_moment_of_inertia(a);

    if (out_obliquity)
        *out_obliquity = obliquity_from_spin_axis(merged_axis);
    if (out_rotation_rate)
        *out_rotation_rate = (L_total[0] * merged_axis[0] + L_total[1] * merged_axis[1]) / fmax(merged_I, 1e-12);
}

static double ease_out_cubic(double t)
{
    double inv;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    inv = 1.0 - t;
    return 1.0 - inv*inv*inv;
}

static double fast_then_slow_spread(double t, double fast_window)
{
    double fast_t, slow_t;

    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    if (fast_window < 1e-6) return ease_out_cubic(t);

    fast_t = t / fast_window;
    if (fast_t > 1.0) fast_t = 1.0;
    slow_t = (t - fast_window) / (1.0 - fast_window);
    if (slow_t < 0.0) slow_t = 0.0;
    if (slow_t > 1.0) slow_t = 1.0;

    /* Large burst during active contact, then a long taper. */
    return 0.82 * ease_out_cubic(fast_t) + 0.18 * ease_out_cubic(slow_t);
}

static double impact_heat_curve(const ImpactEvent *e, double t)
{
    if (!e) return 0.0;
    if (e->kind == COLLISION_VIS_MERGE)
        return pow(1.0 - t, 1.45);
    if (e->kind == COLLISION_VIS_CRATER)
        return pow(1.0 - t, 2.35);
    if (e->kind == COLLISION_VIS_INTERSECT)
        return pow(1.0 - t, 1.65);
    return (1.0 - t) * (1.0 - t);
}

static double impact_spread_curve(const ImpactEvent *e, double t)
{
    if (!e) return 0.0;
    if (e->kind == COLLISION_VIS_MERGE)
        return fast_then_slow_spread(t, 0.09);
    if (e->kind == COLLISION_VIS_INTERSECT)
        return fast_then_slow_spread(t, 0.42);
    if (e->kind == COLLISION_VIS_MAJOR)
        return fast_then_slow_spread(t, 0.14);
    return fast_then_slow_spread(t, 0.18);
}

static double active_merge_glow_progress(int body_idx)
{
    double best = 0.0;
    for (int i = 0; i < MAX_MERGES; i++) {
        double t, grow_t;
        MergeEvent *m = &s_merges[i];
        if (!m->active) continue;
        if (m->target != body_idx && m->impactor != body_idx) continue;
        if (m->duration <= 1e-6) return 1.0;
        t = m->age / m->duration;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        grow_t = 0.78 + 0.22 * ease_out_cubic(t);
        if (grow_t > best) best = grow_t;
    }
    return best;
}

static double current_visual_radius(int body_idx, double physical_radius)
{
    RadiusTransition *fx;
    double t;

    if (body_idx < 0 || body_idx >= MAX_BODIES) return physical_radius;
    fx = &s_radius_fx[body_idx];
    if (!fx->active || fx->duration <= 0.0) return physical_radius;

    t = fx->age / fx->duration;
    if (t >= 1.0) return fx->target_radius;
    return fx->start_radius +
           (fx->target_radius - fx->start_radius) * ease_out_cubic(t);
}

static double current_contact_radius(int body_idx)
{
    if (body_idx < 0 || body_idx >= g_nbodies) return 0.0;
    return current_visual_radius(body_idx, g_bodies[body_idx].radius);
}

static int current_merge_intersection_ring(int body_idx, const double fallback_dir[3],
                                           double out_center[3], double out_normal[3],
                                           double *out_ring_radius)
{
    double best_align = -2.0;
    double fallback_n[3] = {1.0, 0.0, 0.0};
    double fallback_len = sqrt(dot3d(fallback_dir, fallback_dir));

    if (body_idx < 0 || body_idx >= g_nbodies) return 0;
    if (fallback_len > 1e-12) {
        fallback_n[0] = fallback_dir[0] / fallback_len;
        fallback_n[1] = fallback_dir[1] / fallback_len;
        fallback_n[2] = fallback_dir[2] / fallback_len;
    }

    for (int i = 0; i < MAX_MERGES; i++) {
        MergeEvent *m = &s_merges[i];
        int other_idx;
        double c0[3], c1[3], n[3], d, r0, r1, x, rr2, align;

        if (!m->active) continue;
        if (m->target != body_idx && m->impactor != body_idx) continue;

        other_idx = (m->target == body_idx) ? m->impactor : m->target;
        if (other_idx < 0 || other_idx >= g_nbodies) continue;
        if (!g_bodies[body_idx].alive || !g_bodies[other_idx].alive) continue;

        c0[0] = g_bodies[body_idx].pos[0];
        c0[1] = g_bodies[body_idx].pos[1];
        c0[2] = g_bodies[body_idx].pos[2];
        c1[0] = g_bodies[other_idx].pos[0];
        c1[1] = g_bodies[other_idx].pos[1];
        c1[2] = g_bodies[other_idx].pos[2];
        n[0] = c1[0] - c0[0];
        n[1] = c1[1] - c0[1];
        n[2] = c1[2] - c0[2];
        d = sqrt(dot3d(n, n));
        if (d <= 1e-9) continue;

        n[0] /= d; n[1] /= d; n[2] /= d;
        align = dot3d(n, fallback_n);
        if (align < best_align) continue;

        r0 = current_contact_radius(body_idx);
        r1 = current_contact_radius(other_idx);
        if (d >= r0 + r1) continue;

        x = (r0*r0 - r1*r1 + d*d) / (2.0 * d);
        if (x < 0.0) x = 0.0;
        if (x > r0) x = r0;
        rr2 = r0*r0 - x*x;
        if (rr2 <= 1e-6) continue;

        out_center[0] = c0[0] + n[0] * x;
        out_center[1] = c0[1] + n[1] * x;
        out_center[2] = c0[2] + n[2] * x;
        out_normal[0] = n[0];
        out_normal[1] = n[1];
        out_normal[2] = n[2];
        *out_ring_radius = sqrt(rr2);
        best_align = align;
    }

    return best_align > -1.5;
}

static void spawn_impact_particles(int body_idx, int kind, const double world_dir[3],
                                   const double rel_vel[3], double impactor_radius,
                                   double rel_speed, double pack_scale)
{
    double n[3] = {world_dir[0], world_dir[1], world_dir[2]};
    double len = sqrt(dot3d(n, n));
    double t1[3], t2[3];
    double body_r, size_ratio, size_mix;
    double anchor_center[3] = {0.0, 0.0, 0.0};
    double anchor_normal[3] = {1.0, 0.0, 0.0};
    double base_ring_radius = 0.0;
    int use_live_ring = 0;
    int count;
    float base_r, base_g, base_b;

    if (body_idx < 0 || body_idx >= g_nbodies) return;
    if (!g_bodies[body_idx].alive || len <= 1e-12) return;

    n[0] /= len; n[1] /= len; n[2] /= len;
    body_r = current_contact_radius(body_idx);
    size_ratio = impactor_radius / fmax(body_r, 1.0);
    if (size_ratio < 0.015) size_ratio = 0.015;
    if (size_ratio > 1.0) size_ratio = 1.0;
    size_mix = pow(size_ratio, 0.68);
    use_live_ring = current_merge_intersection_ring(body_idx, n, anchor_center,
                                                    anchor_normal, &base_ring_radius);
    if (!use_live_ring) {
        anchor_normal[0] = n[0];
        anchor_normal[1] = n[1];
        anchor_normal[2] = n[2];
        anchor_center[0] = g_bodies[body_idx].pos[0] + n[0] * body_r;
        anchor_center[1] = g_bodies[body_idx].pos[1] + n[1] * body_r;
        anchor_center[2] = g_bodies[body_idx].pos[2] + n[2] * body_r;
        base_ring_radius = impactor_radius * (0.46 + 0.22 * size_mix);
    } else {
        base_ring_radius = fmin(base_ring_radius,
                                impactor_radius * (0.58 + 0.24 * size_mix));
    }
    orthonormal_basis(anchor_normal, t1, t2);

    if (kind == COLLISION_VIS_MERGE) count = 42;
    else if (kind == COLLISION_VIS_MAJOR) count = 26;
    else count = 14;
    count = (int)(count * (0.18 + 0.95 * size_mix));
    count = (int)(count * pack_scale);
    if (count < 4) count = 4;
    if (rel_speed > 16000.0) count += 10;
    if (rel_speed > 28000.0) count += 10;
    if (count > 72) count = 72;

    if (kind == COLLISION_VIS_MAJOR || kind == COLLISION_VIS_MERGE) {
        base_r = 1.00f; base_g = 0.66f; base_b = 0.24f;
    } else {
        base_r = 0.92f; base_g = 0.56f; base_b = 0.20f;
    }

    for (int i = 0; i < count; i++) {
        int slot = -1;
        double phi, ring_radius, tangent_mix, eject_speed, spread_speed;
        double outward_bias;
        double speed_scale;
        double ring_dir[3], ring_t1[3], ring_t2[3];
        double rel_vhat[3], tangent_dir[3], tangent_len, normal_approach;
        double out_plane_angle, in_plane_angle;
        double heat_mix;
        ImpactParticleState *p;

        for (int k = 0; k < MAX_COLLISION_PARTICLES; k++) {
            if (!s_particles[k].active) { slot = k; break; }
        }
        if (slot < 0) {
            slot = 0;
            for (int k = 1; k < MAX_COLLISION_PARTICLES; k++)
                if (s_particles[k].age > s_particles[slot].age) slot = k;
        }

        phi = rand01() * 2.0 * PI;
        if (use_live_ring)
            ring_radius = (0.98 + 0.06 * rand01()) * fmax(base_ring_radius,
                                                          impactor_radius * (0.14 + 0.05 * size_mix));
        else
            ring_radius = (0.92 + 0.16 * rand01()) * base_ring_radius;
        tangent_mix = 0.55 + 0.75 * rand01();
        speed_scale = rel_speed / 42000.0;
        if (speed_scale < 0.0) speed_scale = 0.0;
        if (speed_scale > 1.2) speed_scale = 1.2;
        eject_speed = 180.0
                    + rel_speed * (0.045 + 0.070 * rand01())
                    + rel_speed * speed_scale * (0.010 + 0.016 * rand01());
        rel_vhat[0] = rel_vel ? rel_vel[0] : -anchor_normal[0];
        rel_vhat[1] = rel_vel ? rel_vel[1] : -anchor_normal[1];
        rel_vhat[2] = rel_vel ? rel_vel[2] : -anchor_normal[2];
        normalize3d(rel_vhat);
        normal_approach = -(rel_vhat[0]*anchor_normal[0] +
                            rel_vhat[1]*anchor_normal[1] +
                            rel_vhat[2]*anchor_normal[2]);
        if (normal_approach < 0.0) normal_approach = 0.0;
        if (normal_approach > 1.0) normal_approach = 1.0;
        tangent_dir[0] = rel_vhat[0] + anchor_normal[0] * normal_approach;
        tangent_dir[1] = rel_vhat[1] + anchor_normal[1] * normal_approach;
        tangent_dir[2] = rel_vhat[2] + anchor_normal[2] * normal_approach;
        tangent_len = sqrt(dot3d(tangent_dir, tangent_dir));
        if (tangent_len > 1e-8) {
            tangent_dir[0] /= tangent_len;
            tangent_dir[1] /= tangent_len;
            tangent_dir[2] /= tangent_len;
        } else {
            tangent_dir[0] = t1[0];
            tangent_dir[1] = t1[1];
            tangent_dir[2] = t1[2];
        }
        ring_dir[0] = t1[0] * cos(phi) + t2[0] * sin(phi);
        ring_dir[1] = t1[1] * cos(phi) + t2[1] * sin(phi);
        ring_dir[2] = t1[2] * cos(phi) + t2[2] * sin(phi);
        normalize3d(ring_dir);
        ring_t1[0] = tangent_dir[0];
        ring_t1[1] = tangent_dir[1];
        ring_t1[2] = tangent_dir[2];
        ring_t2[0] = anchor_normal[0];
        ring_t2[1] = anchor_normal[1];
        ring_t2[2] = anchor_normal[2];
        in_plane_angle = ((rand01() * 2.0) - 1.0) * (PI / 12.0);
        out_plane_angle = ((rand01() * 2.0) - 1.0) * ((PI / 18.0) + (PI / 14.0) * normal_approach);
        spread_speed = eject_speed * (0.20 + 0.24 * tangent_mix + 0.08 * (1.0 - normal_approach));
        outward_bias = use_live_ring
                     ? impactor_radius * (0.07 + 0.03 * size_mix)
                     : impactor_radius * (0.14 + 0.05 * size_mix);

        p = &s_particles[slot];
        p->active = 1;
        p->duration = (kind == COLLISION_VIS_MERGE ? 1.2 : kind == COLLISION_VIS_MAJOR ? 0.9 : 0.55) * DAY
                    * (0.65 + 0.55 * rand01());
        p->age = 0.0;
        p->pos[0] = anchor_center[0]
                  + anchor_normal[0] * outward_bias
                  + t1[0] * cos(phi) * ring_radius
                  + t2[0] * sin(phi) * ring_radius;
        p->pos[1] = anchor_center[1]
                  + anchor_normal[1] * outward_bias
                  + t1[1] * cos(phi) * ring_radius
                  + t2[1] * sin(phi) * ring_radius;
        p->pos[2] = anchor_center[2]
                  + anchor_normal[2] * outward_bias
                  + t1[2] * cos(phi) * ring_radius
                  + t2[2] * sin(phi) * ring_radius;
        p->vel[0] = g_bodies[body_idx].vel[0]
                  + ring_dir[0] * eject_speed
                  + ring_t1[0] * (spread_speed * sin(in_plane_angle))
                  + ring_t2[0] * (spread_speed * sin(out_plane_angle));
        p->vel[1] = g_bodies[body_idx].vel[1]
                  + ring_dir[1] * eject_speed
                  + ring_t1[1] * (spread_speed * sin(in_plane_angle))
                  + ring_t2[1] * (spread_speed * sin(out_plane_angle));
        p->vel[2] = g_bodies[body_idx].vel[2]
                  + ring_dir[2] * eject_speed
                  + ring_t1[2] * (spread_speed * sin(in_plane_angle))
                  + ring_t2[2] * (spread_speed * sin(out_plane_angle));
        heat_mix = 0.24 + 0.58 * rand01() + 0.18 * fmin(speed_scale, 1.0);
        (void)heat_mix;
        p->color[0] = base_r;
        p->color[1] = base_g * (0.90f + 0.20f * (float)rand01());
        p->color[2] = base_b * (0.85f + 0.25f * (float)rand01());
        p->color[3] = 1.0f;
        p->size       = (0.5f + 2.5f * (float)rand01()) * (float)size_mix;
        p->fade_start = (float)(0.78 - 0.38 * (1.0 - size_mix));
    }
}

static void finish_radius_transition(int body_idx)
{
    RadiusTransition *fx;

    if (body_idx < 0 || body_idx >= MAX_BODIES) return;
    fx = &s_radius_fx[body_idx];
    if (!fx->active) return;
    fx->age = fx->duration;
    fx->active = 0;
}

static void start_radius_transition(int body_idx, double old_radius,
                                    double new_radius, double duration)
{
    RadiusTransition *fx;
    double start_radius;
    int had_active_transition;
    if (body_idx < 0 || body_idx >= MAX_BODIES) return;
    fx = &s_radius_fx[body_idx];
    had_active_transition = fx->active;
    start_radius = current_visual_radius(body_idx, old_radius);
    fx->active = 1;
    fx->age = 0.0;
    fx->duration = fmax(duration, 1.0);
    if (!had_active_transition &&
        new_radius > 0.0 && start_radius < new_radius * 0.35)
        start_radius = new_radius * 0.35;
    if (!had_active_transition && start_radius < old_radius)
        start_radius = old_radius;
    fx->start_radius = start_radius;
    fx->target_radius = new_radius;
}

static void add_impact(int body_idx, int kind, const double world_dir[3],
                       const double rel_vel[3], double impactor_radius,
                       double rel_speed, double mass_ratio)
{
    int slot = -1;
    Body *b;
    ImpactEvent *e;
    double radius_ratio;

    for (int i = 0; i < MAX_IMPACTS; i++) {
        if (!s_impacts[i].active) { slot = i; break; }
    }
    if (slot < 0) {
        slot = 0;
        for (int i = 1; i < MAX_IMPACTS; i++)
            if (s_impacts[i].age > s_impacts[slot].age) slot = i;
    }

    b = &g_bodies[body_idx];
    e = &s_impacts[slot];
    e->active = 1;
    e->body = body_idx;
    e->kind = kind;
    e->age = 0.0;
    radius_ratio = impactor_radius / fmax(b->radius, 1.0);

    if (kind == COLLISION_VIS_CRATER) {
        /* No upper cap: classification already bounds radius_ratio < ~0.43.
         * Duration scales with radius0 so small craters fade fast, large ones linger. */
        e->radius0 = (float)fmax(0.004, radius_ratio * 0.65);
        e->radius1 = e->radius0 * 1.20f;
        e->duration = IMPACT_COOL_SECONDS * fmax(0.15, (double)e->radius0 / 0.15);
        e->heat0 = (float)fmin(0.95, 0.22 + rel_speed / 32000.0);
    } else if (kind == COLLISION_VIS_MAJOR) {
        /* Same: no upper cap, duration proportional to radius0. */
        e->radius0 = (float)fmax(0.018, radius_ratio * 0.90);
        e->radius1 = e->radius0 * 1.20f;
        e->duration = MAJOR_COOL_SECONDS * fmax(0.30, (double)e->radius0 / 0.50);
        e->heat0 = (float)fmin(1.0, 0.45 + rel_speed / 22000.0 + mass_ratio * 0.4);
    } else {
        e->duration = MERGE_COOL_SECONDS;
        e->heat0 = 1.0f;
        e->radius0 = (float)fmax(0.120, fmin(0.280, 0.12 + mass_ratio * 0.28));
        e->radius1 = (float)fmax(2.30f, fmin(3.20f, 2.30f + mass_ratio * 0.70f));
    }
    fprintf(stderr, "[impact] kind=%s rr=%.3f r0=%.4f r1=%.4f dur=%.1fd\n",
            kind == COLLISION_VIS_CRATER ? "crater" :
            kind == COLLISION_VIS_MAJOR  ? "major"  : "merge",
            radius_ratio, e->radius0, e->radius1, e->duration / DAY);
    body_world_to_local_surface_dir(body_idx, world_dir, e->dir);
    normalize3f(e->dir);
    build_local_scar_tangent(body_idx, world_dir, rel_vel, e->tangent1);
    if (kind == COLLISION_VIS_CRATER || kind == COLLISION_VIS_MAJOR)
        add_permanent_crater(body_idx, world_dir, rel_vel, impactor_radius,
                             kind == COLLISION_VIS_MAJOR ? 1.0 : 0.65 + 0.25 * fmin(mass_ratio, 1.0));
    spawn_impact_particles(body_idx, kind, world_dir, rel_vel,
                           impactor_radius, rel_speed, 1.0);
}

static void add_permanent_crater(int body_idx, const double world_dir[3],
                                 const double rel_vel[3], double impactor_radius,
                                 double strength)
{
    int slot = -1;
    int oldest = 0;
    double radius_limit;
    float local_dir[3];
    PersistentScar *s;

    if (body_idx < 0 || body_idx >= g_nbodies) return;
    if (!g_bodies[body_idx].alive) return;
    if (impactor_radius <= 0.0) return;

    radius_limit = asin(fmin(0.999, impactor_radius / fmax(g_bodies[body_idx].radius, impactor_radius)));
    radius_limit *= 0.92 + 0.10 * fmin(fmax(strength, 0.0), 1.0);
    if (radius_limit < 0.006) radius_limit = 0.006;

    for (int i = 0; i < MAX_PERSISTENT_SCARS; i++) {
        if (!s_perm_scars[i].active) {
            slot = i;
            break;
        }
        if (s_perm_scars[i].stamp < s_perm_scars[oldest].stamp)
            oldest = i;
    }
    if (slot < 0) slot = oldest;

    s = &s_perm_scars[slot];
    s->active = 1;
    s->body = body_idx;
    s->stamp = s_perm_scar_stamp++;
    if (s_perm_scar_stamp == 0u) s_perm_scar_stamp = 1u;
    body_world_to_local_surface_dir(body_idx, world_dir, local_dir);
    normalize3f(local_dir);
    s->dir[0] = local_dir[0];
    s->dir[1] = local_dir[1];
    s->dir[2] = local_dir[2];
    build_local_scar_tangent(body_idx, world_dir, rel_vel, s->tangent1);
    s->angular_radius = (float)radius_limit;
    s->depth = 0.45f + 0.35f * (float)fmin(fmax(strength, 0.0), 1.0);
    s->seed = rand01f() * 100.0f;
}

static double trail_interval_for_body_after_merge(int body_idx)
{
    Body *b;
    double dx, dy, dz, r, gm, T, interval;

    if (body_idx < 0 || body_idx >= g_nbodies) return DAY;
    b = &g_bodies[body_idx];
    if (b->is_star) return DAY * 25.0;
    if (b->parent < 0 || b->parent >= g_nbodies || !g_bodies[b->parent].alive)
        return DAY;

    dx = b->pos[0] - g_bodies[b->parent].pos[0];
    dy = b->pos[1] - g_bodies[b->parent].pos[1];
    dz = b->pos[2] - g_bodies[b->parent].pos[2];
    r = sqrt(dx*dx + dy*dy + dz*dz);
    gm = G_CONST * g_bodies[b->parent].mass;
    if (r <= 0.0 || gm <= 0.0) return DAY;

    T = 2.0 * PI * sqrt(r * r * r / gm);
    interval = T / 400.0;
    if (interval < 60.0) interval = 60.0;
    return interval;
}

static double merge_duration_for_bodies(int target, int impactor, double rel_speed)
{
    double radius_scale;
    double speed_scale;
    double speed_factor;
    double duration;

    radius_scale = (g_bodies[target].radius + g_bodies[impactor].radius)
                 / (2.0 * 6371.0e3);
    if (radius_scale < 0.35) radius_scale = 0.35;

    speed_scale = 1.0 / (1.0 + rel_speed / 18000.0);
    if (speed_scale < 0.40) speed_scale = 0.40;
    /* Faster impacts should finish their penetration phase sooner, but not
     * so fast that the merge loses readability. */
    speed_factor = 0.55 + 0.45 * speed_scale;

    duration = DAY * 1.28 * pow(radius_scale, 0.55) * speed_factor;
    if (duration < DAY * 1.2) duration = DAY * 1.2;
    if (duration > DAY * 14.0) duration = DAY * 14.0;
    return duration;
}

static double merge_speed_boost(double rel_speed)
{
    double t = rel_speed / 12000.0;
    if (t < 0.0) t = 0.0;
    if (t > 2.2) t = 2.2;
    return t;
}

static void finalize_absorb_body(int target, int impactor, double rel_speed,
                                 int outcome, double old_radius)
{
    Body *a = &g_bodies[target];
    Body *b = &g_bodies[impactor];

    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || g_bodies[i].parent != impactor) continue;
        if (target == i || body_is_descendant_of(target, i))
            g_bodies[i].parent = body_root_star(target);
        else
            g_bodies[i].parent = target;
        labels_add_body(i);
    }

    rings_on_body_absorbed(target, impactor);
    b->alive = 0;
    b->mass = 0.0;
    b->trail_count = 0;
    a->trail_interval = trail_interval_for_body_after_merge(target);
    trails_reset_body(target);
    labels_add_body(target);
    labels_remove_body(impactor);
    trails_remove_body(impactor);
    mark_system_dirty(body_root_star(target), SYSTEM_HOT_DURATION);
    fprintf(stderr, "[collision] %s absorbed %s (%.0f m/s, %s, %.0f->%.0f km)\n",
            a->name, b->name, rel_speed,
            outcome == COLLISION_VIS_MERGE ? "merge" :
            outcome == COLLISION_VIS_MAJOR ? "major" : "crater",
            old_radius / 1000.0, a->radius / 1000.0);
}

static void update_merge_events(double dt)
{
    for (int i = 0; i < MAX_MERGES; i++) {
        MergeEvent *m = &s_merges[i];
        Body *target, *impactor;
        double t, overlap_sep;
        double speed_boost;
        double attach_world_dir[3];
        double attach_world_t1[3];

        if (!m->active) continue;
        if (m->target < 0 || m->target >= g_nbodies ||
            m->impactor < 0 || m->impactor >= g_nbodies) {
            m->active = 0;
            continue;
        }

        target   = &g_bodies[m->target];
        impactor = &g_bodies[m->impactor];
        if (!target->alive || !impactor->alive) {
            m->active = 0;
            continue;
        }

        m->age += dt;
        t = m->age / m->duration;
        if (t > 1.0) t = 1.0;
        speed_boost = merge_speed_boost(m->rel_speed);
        m->particle_emit_accum += dt;
        body_local_surface_dir_to_world(m->target, m->target_local_attach_dir,
                                        attach_world_dir);
        body_local_surface_dir_to_world(m->target, m->target_local_attach_t1,
                                        attach_world_t1);

        /* Lock impactor velocity to target. */
        impactor->vel[0] = target->vel[0];
        impactor->vel[1] = target->vel[1];
        impactor->vel[2] = target->vel[2];

        /* Both rotations snap to nearly zero very early in the merge —
         * the deceleration should feel simultaneous with the impact. */
        {
            double spin_window = 0.07 - 0.02 * fmin(speed_boost, 1.0);
            double slow = 1.0 - ease_out_cubic(fmin(1.0, t / spin_window));
            target->obliquity = m->target_obliquity;
            target->rotation_rate = m->target_rotation_rate;
            impactor->rotation_rate = m->impactor_rotation_rate * slow;
        }

        {
            double target_contact_r = current_contact_radius(m->target);
            double impactor_r = g_bodies[m->impactor].radius;
            double depth_boost = fmin(speed_boost, 2.0);
            double final_target_sep = target_contact_r
                                    - impactor_r * (1.06 + 0.42 * depth_boost);
            double pen_t;
            if (final_target_sep < target_contact_r * 0.004)
                final_target_sep = target_contact_r * 0.004;
            {
                double pen_exp = 3.0 + 2.8 * depth_boost;
                double fast_t = 1.0 - pow(1.0 - t, pen_exp);
                double tail_blend = (t - 0.22) / 0.78;
                if (tail_blend < 0.0) tail_blend = 0.0;
                if (tail_blend > 1.0) tail_blend = 1.0;
                tail_blend = tail_blend * tail_blend * (3.0 - 2.0 * tail_blend);
                pen_t = fast_t * (1.0 - 0.12 * tail_blend) + t * (0.12 * tail_blend);
            }
            overlap_sep = m->start_sep + (final_target_sep - m->start_sep) * pen_t;

            /* Despawn the impactor the moment it is fully inside the target. */
            if (overlap_sep + impactor_r <= target_contact_r) {
                double old_radius = target->radius;
                impactor->pos[0] = target->pos[0] + attach_world_dir[0] * overlap_sep;
                impactor->pos[1] = target->pos[1] + attach_world_dir[1] * overlap_sep;
                impactor->pos[2] = target->pos[2] + attach_world_dir[2] * overlap_sep;
                /* Do not finish_radius_transition here — let it run to anim_dur
                 * so the target grows smoothly to its final size. */
                if (m->scar_slot >= 0 && m->scar_slot < MAX_IMPACTS &&
                    s_impacts[m->scar_slot].active) {
                    float r0 = s_impacts[m->scar_slot].radius0;
                    float r1_old = s_impacts[m->scar_slot].radius1;
                    if (r0 <= 0.01f) {
                        /* Geometric tracking never ran (sim dt >= anim_dur or tiny impactor).
                         * Estimate the peak cap angle from the impactor's angular size. */
                        float ov = (float)impactor_r;
                        float sv = (float)target_contact_r;
                        r0 = (float)fmax(0.01, asin(fmin(0.999, ov / fmax(sv, ov))));
                        s_impacts[m->scar_slot].radius0 = r0;
                    }
                    s_impacts[m->scar_slot].radius1 = fminf((float)(PI * 0.88), r0 * 1.20f);
                    s_impacts[m->scar_slot].duration = MERGE_COOL_SECONDS
                        * fmax(0.30, (double)r0 / (PI * 0.50));
                    fprintf(stderr, "[early-despawn] r0=%.4f r1_old=%.4f r1_new=%.4f\n",
                            r0, r1_old, s_impacts[m->scar_slot].radius1);
                }
                m->active = 0;
                finalize_absorb_body(m->target, m->impactor, m->rel_speed,
                                     COLLISION_VIS_MERGE, old_radius);
                continue;
            }
        }
        impactor->pos[0] = target->pos[0] + attach_world_dir[0] * overlap_sep;
        impactor->pos[1] = target->pos[1] + attach_world_dir[1] * overlap_sep;
        impactor->pos[2] = target->pos[2] + attach_world_dir[2] * overlap_sep;

        if (t < 0.04) {
            double emit_t = t / 0.04;
            double emit_decay = 1.0 - emit_t;
            if (emit_t > 1.0) emit_t = 1.0;
            double emit_slice = DAY * (0.003 + 0.010 * emit_t);
            double emit_prob = 0.88 * emit_decay * emit_decay * emit_decay
                             * emit_decay * emit_decay;
            if (emit_slice < DAY * 0.0025) emit_slice = DAY * 0.0025;
            while (m->particle_emit_accum >= emit_slice) {
                m->particle_emit_accum -= emit_slice;
                if (rand01() < emit_prob) {
                    double pack_scale = (0.10 + 0.05 * fmin(speed_boost, 1.4))
                                      * emit_decay * emit_decay * emit_decay
                                      * emit_decay * emit_decay;
                    if (pack_scale < 0.020) pack_scale = 0.020;
                spawn_impact_particles(m->target, COLLISION_VIS_MERGE, attach_world_dir,
                                       m->rel_vel, g_bodies[m->impactor].radius,
                                       m->rel_speed, pack_scale);
                }
            }
        } else {
            m->particle_emit_accum = 0.0;
            m->particle_emit_next = 0.0;
        }

        /* Keep the scar event frozen and growing while the merge is live. */
        if (m->scar_slot >= 0 && m->scar_slot < MAX_IMPACTS &&
            s_impacts[m->scar_slot].active) {
            ImpactEvent *scar = &s_impacts[m->scar_slot];
            double sv = collision_visual_radius(m->target, target->radius);
            double ov = g_bodies[m->impactor].radius;
            double d  = overlap_sep;
            if (d < sv + ov && d > 1e-6) {
                double cos_a = (sv*sv + d*d - ov*ov) / (2.0*sv*d);
                if (cos_a < -1.0) cos_a = -1.0;
                if (cos_a >  1.0) cos_a =  1.0;
                float cap = (float)acos(cos_a);
                if (cap > scar->radius0) {
                    float boost = (float)(0.06 + 0.12 * fmin(speed_boost, 1.2));
                    scar->radius0 = cap;
                    scar->radius1 = fminf((float)(PI * 0.88),
                                          fmaxf(scar->radius1, cap * (1.08f + boost)));
                    scar->dir[0] = (float)m->target_local_attach_dir[0];
                    scar->dir[1] = (float)m->target_local_attach_dir[1];
                    scar->dir[2] = (float)m->target_local_attach_dir[2];
                    scar->tangent1[0] = (float)m->target_local_attach_t1[0];
                    scar->tangent1[1] = (float)m->target_local_attach_t1[1];
                    scar->tangent1[2] = (float)m->target_local_attach_t1[2];
                }
            }
            scar->age = 0.0; /* frozen at full heat until merge ends */
        }

        /* Impactor-side scar: same freeze + peak-tracking, opposite cap formula. */
        if (m->imp_scar_slot >= 0 && m->imp_scar_slot < MAX_IMPACTS &&
            s_impacts[m->imp_scar_slot].active) {
            ImpactEvent *scar = &s_impacts[m->imp_scar_slot];
            double ov = collision_visual_radius(m->impactor, g_bodies[m->impactor].radius);
            double sv = collision_visual_radius(m->target,   target->radius);
            double d  = overlap_sep;
            if (d < sv + ov && d > 1e-6) {
                double cos_a = (ov*ov + d*d - sv*sv) / (2.0*ov*d);
                if (cos_a < -1.0) cos_a = -1.0;
                if (cos_a >  1.0) cos_a =  1.0;
                float cap = (float)acos(cos_a);
                if (cap > scar->radius0) {
                    float local_dir[3];
                    float local_t1[3];
                    double neg_dir[3] = {-attach_world_dir[0], -attach_world_dir[1], -attach_world_dir[2]};
                    float boost = (float)(0.05 + 0.10 * fmin(speed_boost, 1.2));
                    scar->radius0 = cap;
                    scar->radius1 = fminf((float)(PI * 0.88),
                                          fmaxf(scar->radius1, cap * (1.06f + boost)));
                    body_world_to_local_surface_dir(m->impactor, neg_dir, local_dir);
                    body_world_to_local_surface_dir(m->impactor, attach_world_t1, local_t1);
                    normalize3f(local_dir);
                    normalize3f(local_t1);
                    scar->dir[0] = local_dir[0];
                    scar->dir[1] = local_dir[1];
                    scar->dir[2] = local_dir[2];
                    scar->tangent1[0] = local_t1[0];
                    scar->tangent1[1] = local_t1[1];
                    scar->tangent1[2] = local_t1[2];
                }
            }
            scar->age = 0.0;
        }

        /* Keep the merge alive until the full animation duration finishes so
         * radius growth completes during the merge instead of after it. */
        {
            if (m->age >= m->duration) {
                double old_radius = target->radius;
                finish_radius_transition(m->target);

                /* Release target scar: expand 20% beyond the geometric cap reached
                 * during the merge. Duration scales with scar size so growth speed
                 * is proportional (smaller merger = faster fade). */
                if (m->scar_slot >= 0 && m->scar_slot < MAX_IMPACTS &&
                    s_impacts[m->scar_slot].active) {
                    float r0 = s_impacts[m->scar_slot].radius0;
                    if (r0 <= 0.01f) {
                        /* Tracking never ran — estimate from impactor angular size. */
                        float ov = (float)g_bodies[m->impactor].radius;
                        float sv = (float)current_contact_radius(m->target);
                        r0 = (float)fmax(0.01, asin(fmin(0.999, ov / fmax(sv, ov))));
                        s_impacts[m->scar_slot].radius0 = r0;
                    }
                    s_impacts[m->scar_slot].radius1 = fminf((float)(PI * 0.88), r0 * 1.20f);
                    s_impacts[m->scar_slot].duration = MERGE_COOL_SECONDS
                        * fmax(0.30, (double)r0 / (PI * 0.50));
                    fprintf(stderr, "[merge-release] r0=%.4f r1=%.4f dur=%.1fd\n",
                            r0, s_impacts[m->scar_slot].radius1,
                            s_impacts[m->scar_slot].duration / DAY);
                }

                m->active = 0;
                finalize_absorb_body(m->target, m->impactor, m->rel_speed,
                                     COLLISION_VIS_MERGE, old_radius);
            }
        }
    }
}

static void begin_merge_event(int target, int impactor, double rel_speed,
                              const double dir[3], const double rel_vel[3],
                              double old_radius)
{
    int slot = -1;
    Body *a = &g_bodies[target];
    Body *b = &g_bodies[impactor];
    double total = a->mass + b->mass;
    double merged_radius;
    float local_dir[3], local_t1[3];

    for (int i = 0; i < MAX_MERGES; i++) {
        if (!s_merges[i].active) { slot = i; break; }
    }
    if (slot < 0) slot = 0;

    if (total <= 0.0) return;

    a->vel[0] = (a->vel[0] * a->mass + b->vel[0] * b->mass) / total;
    a->vel[1] = (a->vel[1] * a->mass + b->vel[1] * b->mass) / total;
    a->vel[2] = (a->vel[2] * a->mass + b->vel[2] * b->mass) / total;
    merged_radius = cbrt(a->radius*a->radius*a->radius +
                         b->radius*b->radius*b->radius);
    if (merged_radius < old_radius) merged_radius = old_radius;
    if (merged_radius < b->radius) merged_radius = b->radius;
    a->mass = total;
    a->radius = merged_radius;
    b->mass = 0.0;
    b->vel[0] = a->vel[0];
    b->vel[1] = a->vel[1];
    b->vel[2] = a->vel[2];
    compute_collision_spin_state(target, impactor, dir, rel_vel,
                                 &a->obliquity, &a->rotation_rate);

    s_merges[slot].active    = 1;
    s_merges[slot].target    = target;
    s_merges[slot].impactor  = impactor;
    s_merges[slot].age       = 0.0;
    s_merges[slot].rel_speed = rel_speed;
    s_merges[slot].dir[0]    = dir[0];
    s_merges[slot].dir[1]    = dir[1];
    s_merges[slot].dir[2]    = dir[2];
    s_merges[slot].rel_vel[0] = rel_vel[0];
    s_merges[slot].rel_vel[1] = rel_vel[1];
    s_merges[slot].rel_vel[2] = rel_vel[2];
    body_world_to_local_surface_dir(target, dir, local_dir);
    s_merges[slot].target_local_attach_dir[0] = local_dir[0];
    s_merges[slot].target_local_attach_dir[1] = local_dir[1];
    s_merges[slot].target_local_attach_dir[2] = local_dir[2];
    build_local_scar_tangent(target, dir, rel_vel, local_t1);
    s_merges[slot].target_local_attach_t1[0] = local_t1[0];
    s_merges[slot].target_local_attach_t1[1] = local_t1[1];
    s_merges[slot].target_local_attach_t1[2] = local_t1[2];
    s_merges[slot].particle_emit_accum    = 0.0;
    s_merges[slot].particle_emit_next     = 0.0;
    s_merges[slot].target_rotation_rate   = a->rotation_rate;
    s_merges[slot].impactor_rotation_rate = b->rotation_rate;
    s_merges[slot].target_obliquity       = a->obliquity;
    s_merges[slot].scar_slot     = -1;
    s_merges[slot].imp_scar_slot = -1;
    add_permanent_crater(target, dir, rel_vel, b->radius, 1.0);

    {
        double anim_dur = merge_duration_for_bodies(target, impactor, rel_speed);

        /* Start from the actual current separation, but never further apart
         * than just-touching. This avoids glitching when swept-sphere fires
         * before visual contact, while still handling mid-step interpenetration. */
        double sep_actual = sqrt((b->pos[0]-a->pos[0])*(b->pos[0]-a->pos[0]) +
                                 (b->pos[1]-a->pos[1])*(b->pos[1]-a->pos[1]) +
                                 (b->pos[2]-a->pos[2])*(b->pos[2]-a->pos[2]));
        double sep_init = fmin(sep_actual, old_radius + b->radius);

        s_merges[slot].duration  = anim_dur;
        s_merges[slot].start_sep = sep_init;

        start_radius_transition(target, old_radius, a->radius, anim_dur);
        /* Impactor keeps its size throughout — no shrink transition. */
    }

    /* Allocate the intersection scar event immediately so it can be updated
     * each frame during the merge and then cool after the merge ends. */
    {
        int k, scar_slot = -1;
        float local_dir[3];
        ImpactEvent *scar;
        for (k = 0; k < MAX_IMPACTS; k++) {
            if (!s_impacts[k].active) { scar_slot = k; break; }
        }
        if (scar_slot < 0) {
            scar_slot = 0;
            for (k = 1; k < MAX_IMPACTS; k++)
                if (s_impacts[k].age > s_impacts[scar_slot].age) scar_slot = k;
        }
        body_world_to_local_surface_dir(target, dir, local_dir);
        normalize3f(local_dir);
        scar = &s_impacts[scar_slot];
        scar->active   = 1;
        scar->body     = target;
        scar->kind     = COLLISION_VIS_INTERSECT;
        scar->age      = 0.0;
        scar->duration = MERGE_COOL_SECONDS;
        scar->heat0    = 1.0f;
        scar->radius0  = 0.0f;
        scar->radius1  = (float)(0.18 + 0.08 * fmin(merge_speed_boost(rel_speed), 1.5));
        scar->dir[0]   = local_dir[0];
        scar->dir[1]   = local_dir[1];
        scar->dir[2]   = local_dir[2];
        scar->tangent1[0] = (float)s_merges[slot].target_local_attach_t1[0];
        scar->tangent1[1] = (float)s_merges[slot].target_local_attach_t1[1];
        scar->tangent1[2] = (float)s_merges[slot].target_local_attach_t1[2];
        s_merges[slot].scar_slot = scar_slot;
    }

    /* Impactor-side scar: same setup, direction reversed. */
    {
        double neg_dir[3] = {-dir[0], -dir[1], -dir[2]};
        double world_t1[3];
        int k, scar_slot = -1;
        float local_dir[3];
        float local_t1[3];
        ImpactEvent *scar;
        for (k = 0; k < MAX_IMPACTS; k++) {
            if (!s_impacts[k].active) { scar_slot = k; break; }
        }
        if (scar_slot < 0) {
            scar_slot = 0;
            for (k = 1; k < MAX_IMPACTS; k++)
                if (s_impacts[k].age > s_impacts[scar_slot].age) scar_slot = k;
        }
        body_local_surface_dir_to_world(target, s_merges[slot].target_local_attach_t1, world_t1);
        body_world_to_local_surface_dir(impactor, neg_dir, local_dir);
        body_world_to_local_surface_dir(impactor, world_t1, local_t1);
        normalize3f(local_dir);
        normalize3f(local_t1);
        scar = &s_impacts[scar_slot];
        scar->active   = 1;
        scar->body     = impactor;
        scar->kind     = COLLISION_VIS_INTERSECT;
        scar->age      = 0.0;
        scar->duration = MERGE_COOL_SECONDS;
        scar->heat0    = 1.0f;
        scar->radius0  = 0.0f;
        scar->radius1  = (float)(0.15 + 0.07 * fmin(merge_speed_boost(rel_speed), 1.5));
        scar->dir[0]   = local_dir[0];
        scar->dir[1]   = local_dir[1];
        scar->dir[2]   = local_dir[2];
        scar->tangent1[0] = local_t1[0];
        scar->tangent1[1] = local_t1[1];
        scar->tangent1[2] = local_t1[2];
        s_merges[slot].imp_scar_slot = scar_slot;
    }

    spawn_impact_particles(target, COLLISION_VIS_MERGE, dir, s_merges[slot].rel_vel,
                           b->radius, rel_speed, 0.85);
}

static int classify_collision(int a, int b, double rel_speed)
{
    double m_small = g_bodies[a].mass < g_bodies[b].mass ? g_bodies[a].mass : g_bodies[b].mass;
    double m_large = g_bodies[a].mass > g_bodies[b].mass ? g_bodies[a].mass : g_bodies[b].mass;
    double ra = current_contact_radius(a);
    double rb = current_contact_radius(b);
    double r_sum = ra + rb;
    double mass_ratio, v_escape;

    if (m_large <= 0.0) return COLLISION_VIS_CRATER;
    mass_ratio = m_small / m_large;
    v_escape = sqrt(fmax(0.0, 2.0 * G_CONST * (g_bodies[a].mass + g_bodies[b].mass) /
                         fmax(r_sum, 1.0)));

    if (mass_ratio >= 0.45 || ra >= 0.74 * rb ||
        rb >= 0.74 * ra)
        return COLLISION_VIS_MERGE;
    if (mass_ratio >= 0.08 || rel_speed >= v_escape * 1.35)
        return COLLISION_VIS_MAJOR;
    return COLLISION_VIS_CRATER;
}

static int systems_may_interact(int root_a, int root_b, const double system_radius[MAX_BODIES])
{
    if (root_a == root_b) return 1;
    if (root_a < 0 || root_b < 0) return 0;
    if (!g_bodies[root_a].alive || !g_bodies[root_b].alive) return 0;

    double dx = g_bodies[root_b].pos[0] - g_bodies[root_a].pos[0];
    double dy = g_bodies[root_b].pos[1] - g_bodies[root_a].pos[1];
    double dz = g_bodies[root_b].pos[2] - g_bodies[root_a].pos[2];
    double r = system_radius[root_a] + system_radius[root_b];
    return dx*dx + dy*dy + dz*dz <= r*r;
}

static double pair_check_dt(int a, int b)
{
    double rx = g_bodies[b].pos[0] - g_bodies[a].pos[0];
    double ry = g_bodies[b].pos[1] - g_bodies[a].pos[1];
    double rz = g_bodies[b].pos[2] - g_bodies[a].pos[2];
    double dist2 = rx*rx + ry*ry + rz*rz;
    if (dist2 <= 1e-12) return MIN_PAIR_DT;

    double dist = sqrt(dist2);
    double vx = g_bodies[b].vel[0] - g_bodies[a].vel[0];
    double vy = g_bodies[b].vel[1] - g_bodies[a].vel[1];
    double vz = g_bodies[b].vel[2] - g_bodies[a].vel[2];
    double vr = -(rx*vx + ry*vy + rz*vz) / dist;
    double gap = dist - (current_contact_radius(a) + current_contact_radius(b));

    if (gap <= 0.0) return MIN_PAIR_DT;
    if (vr <= 1e-3) {
        double rel_speed = sqrt(vx*vx + vy*vy + vz*vz);
        if (dist > 200.0 * (g_bodies[a].radius + g_bodies[b].radius) &&
            rel_speed < 100.0)
            return MAX_PAIR_DT;
        return fmin(MAX_PAIR_DT, fmax(MIN_PAIR_DT, dist / fmax(rel_speed, 1.0) * 0.2));
    }

    {
        double tau = gap / vr;
        double dt = tau * 0.2;
        if (dt < MIN_PAIR_DT) dt = MIN_PAIR_DT;
        if (dt > MAX_PAIR_DT) dt = MAX_PAIR_DT;
        return dt;
    }
}

static int swept_spheres_collide(int a, int b, double dt, double *out_speed)
{
    double rel_now[3] = {
        g_bodies[b].pos[0] - g_bodies[a].pos[0],
        g_bodies[b].pos[1] - g_bodies[a].pos[1],
        g_bodies[b].pos[2] - g_bodies[a].pos[2]
    };
    /* Use actual pre-physics positions when available so that bodies which
     * tunnel completely through each other in one large timestep are still
     * detected.  The old formula (rel_now - vel*dt) fails when gravity
     * reverses the relative velocity mid-step: the reconstructed "past"
     * position ends up on the wrong side of the target, so the swept path
     * never crosses the contact sphere. */
    double rel_p[3];
    double rel_v[3];
    double vv;
    if (s_pos_before_valid && dt > 0.0) {
        rel_p[0] = s_pos_before[b][0] - s_pos_before[a][0];
        rel_p[1] = s_pos_before[b][1] - s_pos_before[a][1];
        rel_p[2] = s_pos_before[b][2] - s_pos_before[a][2];
        rel_v[0] = (rel_now[0] - rel_p[0]) / dt;
        rel_v[1] = (rel_now[1] - rel_p[1]) / dt;
        rel_v[2] = (rel_now[2] - rel_p[2]) / dt;
    } else {
        rel_v[0] = g_bodies[b].vel[0] - g_bodies[a].vel[0];
        rel_v[1] = g_bodies[b].vel[1] - g_bodies[a].vel[1];
        rel_v[2] = g_bodies[b].vel[2] - g_bodies[a].vel[2];
        rel_p[0] = rel_now[0] - rel_v[0] * dt;
        rel_p[1] = rel_now[1] - rel_v[1] * dt;
        rel_p[2] = rel_now[2] - rel_v[2] * dt;
    }
    vv = dot3d(rel_v, rel_v);
    {
        double t = 0.0;
        double c[3];
        double r = current_contact_radius(a) + current_contact_radius(b);
        if (vv > MIN_COLLISION_SPEED * MIN_COLLISION_SPEED) {
            t = -dot3d(rel_p, rel_v) / vv;
            if (t < 0.0) t = 0.0;
            if (t > dt) t = dt;
        }
        c[0] = rel_p[0] + rel_v[0] * t;
        c[1] = rel_p[1] + rel_v[1] * t;
        c[2] = rel_p[2] + rel_v[2] * t;
        if (out_speed) *out_speed = sqrt(vv);
        return dot3d(c, c) <= r*r;
    }
}

static void impact_dir_for_pair(int target, int impactor, double dt, double out_dir[3])
{
    double rel_now[3] = {
        g_bodies[impactor].pos[0] - g_bodies[target].pos[0],
        g_bodies[impactor].pos[1] - g_bodies[target].pos[1],
        g_bodies[impactor].pos[2] - g_bodies[target].pos[2]
    };
    double rel_v[3] = {
        g_bodies[impactor].vel[0] - g_bodies[target].vel[0],
        g_bodies[impactor].vel[1] - g_bodies[target].vel[1],
        g_bodies[impactor].vel[2] - g_bodies[target].vel[2]
    };
    double rel_p[3] = {
        rel_now[0] - rel_v[0] * dt,
        rel_now[1] - rel_v[1] * dt,
        rel_now[2] - rel_v[2] * dt
    };
    double vv = dot3d(rel_v, rel_v);
    double t = 0.0;
    double c[3];
    double len;

    if (vv > MIN_COLLISION_SPEED * MIN_COLLISION_SPEED) {
        t = -dot3d(rel_p, rel_v) / vv;
        if (t < 0.0) t = 0.0;
        if (t > dt) t = dt;
    }

    c[0] = rel_p[0] + rel_v[0] * t;
    c[1] = rel_p[1] + rel_v[1] * t;
    c[2] = rel_p[2] + rel_v[2] * t;
    len = sqrt(dot3d(c, c));
    if (len <= 1e-12) {
        c[0] = rel_now[0];
        c[1] = rel_now[1];
        c[2] = rel_now[2];
        len = sqrt(dot3d(c, c));
    }
    if (len <= 1e-12) {
        out_dir[0] = 1.0;
        out_dir[1] = 0.0;
        out_dir[2] = 0.0;
        return;
    }

    out_dir[0] = c[0] / len;
    out_dir[1] = c[1] / len;
    out_dir[2] = c[2] / len;
}

static void absorb_body(int target, int impactor, double rel_speed, double collision_dt)
{
    Body *a = &g_bodies[target];
    Body *b = &g_bodies[impactor];
    double old_radius;
    double mass_ratio;
    double rel_vel[3] = {
        g_bodies[impactor].vel[0] - g_bodies[target].vel[0],
        g_bodies[impactor].vel[1] - g_bodies[target].vel[1],
        g_bodies[impactor].vel[2] - g_bodies[target].vel[2]
    };
    double dir[3];
    int outcome;

    if (!a->alive || !b->alive || a->is_star || b->is_star) return;

    old_radius = current_contact_radius(target);
    mass_ratio = fmin(a->mass, b->mass) / fmax(fmax(a->mass, b->mass), 1.0);
    outcome = classify_collision(target, impactor, rel_speed);
    {
        double ra = current_contact_radius(target);
        double rb = current_contact_radius(impactor);
        fprintf(stderr, "[absorb] %s <- %s  mr=%.3f rr=%.3f  outcome=%s\n",
                a->name, b->name, mass_ratio, rb/fmax(ra,1.0),
                outcome==COLLISION_VIS_MERGE?"MERGE":
                outcome==COLLISION_VIS_MAJOR?"MAJOR":"CRATER");
    }
    impact_dir_for_pair(target, impactor, collision_dt, dir);
    if (outcome == COLLISION_VIS_MERGE) {
        begin_merge_event(target, impactor, rel_speed, dir, rel_vel, old_radius);
        return;
    }

    {
        double total = a->mass + b->mass;
    if (total <= 0.0) return;

    add_impact(target, outcome, dir, rel_vel, b->radius, rel_speed, mass_ratio);

    a->vel[0] = (a->vel[0] * a->mass + b->vel[0] * b->mass) / total;
    a->vel[1] = (a->vel[1] * a->mass + b->vel[1] * b->mass) / total;
    a->vel[2] = (a->vel[2] * a->mass + b->vel[2] * b->mass) / total;
    a->radius = old_radius;
    a->mass = total;
    }

    finalize_absorb_body(target, impactor, rel_speed, outcome, old_radius);
}

void collision_step(double dt)
{
    double system_radius[MAX_BODIES];
    int resolved[MAX_BODIES] = {0};
    int members[MAX_BODIES][MAX_BODIES];
    int member_count[MAX_BODIES] = {0};
    int active_roots[MAX_BODIES];
    int active_root_count = 0;
    int any_dirty = 0;

    update_merge_events(dt);

    for (int i = 0; i < MAX_COLLISION_PARTICLES; i++) {
        ImpactParticleState *p = &s_particles[i];
        double drag;
        if (!p->active) continue;
        p->age += dt;
        if (p->age >= p->duration) {
            p->active = 0;
            continue;
        }
        p->pos[0] += p->vel[0] * dt;
        p->pos[1] += p->vel[1] * dt;
        p->pos[2] += p->vel[2] * dt;
        /* Mild time-based drag so debris keeps travelling with the impact
         * instead of appearing to freeze in world space. */
        drag = exp(-dt / (DAY * 18.0));
        p->vel[0] *= drag;
        p->vel[1] *= drag;
        p->vel[2] *= drag;
    }

    for (int i = 0; i < MAX_IMPACTS; i++) {
        if (!s_impacts[i].active) continue;
        s_impacts[i].age += dt;
        if (s_impacts[i].age >= s_impacts[i].duration ||
            s_impacts[i].body < 0 || s_impacts[i].body >= g_nbodies ||
            !g_bodies[s_impacts[i].body].alive)
            s_impacts[i].active = 0;
    }
    for (int i = 0; i < MAX_BODIES; i++) {
        if (!s_radius_fx[i].active) continue;
        if (i >= g_nbodies || !g_bodies[i].alive) {
            s_radius_fx[i].active = 0;
            continue;
        }
        s_radius_fx[i].age += dt;
        if (s_radius_fx[i].age >= s_radius_fx[i].duration) {
            s_radius_fx[i].active = 0;
            s_radius_fx[i].age = s_radius_fx[i].duration;
        }
    }

    if (dt <= 0.0) return;

    for (int i = 0; i < MAX_BODIES; i++) {
        if (s_system_dirty[i]) { any_dirty = 1; break; }
    }
    if (!any_dirty) return;

    for (int i = 0; i < MAX_BODIES; i++) {
        system_radius[i] = SYSTEM_MARGIN_AU * AU;
    }
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        int root = body_root_star(i);
        if (root < 0 || root >= g_nbodies) continue;
        if (member_count[root] == 0 && active_root_count < MAX_BODIES)
            active_roots[active_root_count++] = root;
        if (member_count[root] < MAX_BODIES)
            members[root][member_count[root]++] = i;
        double dx = g_bodies[i].pos[0] - g_bodies[root].pos[0];
        double dy = g_bodies[i].pos[1] - g_bodies[root].pos[1];
        double dz = g_bodies[i].pos[2] - g_bodies[root].pos[2];
        double d = sqrt(dx*dx + dy*dy + dz*dz) + current_contact_radius(i);
        if (d > system_radius[root]) system_radius[root] = d;
    }

    for (int root = 0; root < g_nbodies; root++) {
        if (!s_system_dirty[root]) continue;
        if (!g_bodies[root].alive) {
            s_system_dirty[root] = 0;
            s_system_hot[root] = 0.0;
            continue;
        }

        int root_had_collision = 0;
        for (int rj = 0; rj < active_root_count && !root_had_collision; rj++) {
            int other_root = active_roots[rj];
            int same_root = (other_root == root);
            int na, nb;

            if (other_root < 0 || other_root >= g_nbodies) continue;
            if (!g_bodies[other_root].alive) continue;
            if (other_root < root && s_system_dirty[other_root]) continue;
            if (!systems_may_interact(root, other_root, system_radius)) continue;

            na = member_count[root];
            nb = member_count[other_root];
            for (int ai = 0; ai < na && !root_had_collision; ai++) {
                int a = members[root][ai];
                if (!g_bodies[a].alive || g_bodies[a].is_star || resolved[a] || body_is_merge_impactor(a))
                    continue;
                for (int bi = 0; bi < nb; bi++) {
                    int b = members[other_root][bi];
                    int lo, hi;
                    double speed = 0.0;

                    if (same_root && bi <= ai) continue;
                    if (a == b) continue;
                    if (!g_bodies[b].alive || g_bodies[b].is_star || resolved[b] || body_is_merge_impactor(b)) continue;

                    lo = a < b ? a : b;
                    hi = a < b ? b : a;

                    if (s_system_hot[root] > 0.0) {
                        s_pair_next[lo][hi] = HOT_PAIR_DT;
                    } else {
                        s_pair_next[lo][hi] -= dt;
                        if (s_pair_next[lo][hi] > 0.0) continue;
                    }

                    {
                        int hit = swept_spheres_collide(a, b, dt, &speed);
                        if (!hit) {
                            double pdt = pair_check_dt(a, b);
                            s_pair_next[lo][hi] = pdt;
                            if (!same_root) {
                                /* Cross-system approaching pair: keep both roots
                                 * dirty so the pair continues to be evaluated
                                 * after their individual hot periods expire. */
                                mark_system_dirty(root, pdt);
                                mark_system_dirty(other_root, pdt);
                            }
                            continue;
                        }
                    }

                    {
                        int a_is_merge_target = body_is_merge_target(a);
                        int b_is_merge_target = body_is_merge_target(b);
                        int keep_target_open;
                        int target;
                        int impactor;
                        if (a_is_merge_target && !b_is_merge_target)
                            target = a;
                        else if (b_is_merge_target && !a_is_merge_target)
                            target = b;
                        else
                            target = g_bodies[a].mass >= g_bodies[b].mass ? a : b;
                        impactor = target == a ? b : a;
                        absorb_body(target, impactor, speed, dt);
                        keep_target_open = body_is_merge_target(target);
                        if (!keep_target_open)
                            resolved[target] = 1;
                        resolved[impactor] = 1;
                        s_pair_next[lo][hi] = HOT_PAIR_DT;
                        if (!keep_target_open) {
                            root_had_collision = 1;
                            break;
                        }
                        if (impactor == a) break;
                    }
                }
            }
        }

        (void)root_had_collision;
    }

    for (int i = 0; i < MAX_BODIES; i++) {
        if (s_system_hot[i] > 0.0) {
            s_system_hot[i] -= dt;
            if (s_system_hot[i] < 0.0) s_system_hot[i] = 0.0;
        }
    }
}

int collision_spots_for_body(int body_idx, CollisionSpot spots[COLLISION_MAX_SPOTS])
{
    int n = 0;
    if (!spots || body_idx < 0 || body_idx >= g_nbodies) return 0;

    for (int i = 0; i < MAX_PERSISTENT_SCARS && n < COLLISION_MAX_SPOTS; i++) {
        PersistentScar *s = &s_perm_scars[i];
        if (!s->active || s->body != body_idx) continue;
        spots[n].dir[0] = s->dir[0];
        spots[n].dir[1] = s->dir[1];
        spots[n].dir[2] = s->dir[2];
        spots[n].tangent1[0] = s->tangent1[0];
        spots[n].tangent1[1] = s->tangent1[1];
        spots[n].tangent1[2] = s->tangent1[2];
        spots[n].angular_radius = s->angular_radius;
        spots[n].heat = s->depth;
        spots[n].progress = 0.0f;
        spots[n].seed = s->seed;
        spots[n].kind = COLLISION_VIS_PERM_CRATER;
        n++;
    }

    for (int i = 0; i < MAX_IMPACTS && n < COLLISION_MAX_SPOTS; i++) {
        ImpactEvent *e = &s_impacts[i];
        double t;
        double heat_curve;
        double rad;
        if (!e->active || e->body != body_idx) continue;
        t = e->age / e->duration;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        heat_curve = impact_heat_curve(e, t);
        {
            double spread_t = impact_spread_curve(e, t);
            rad = e->radius0 + (e->radius1 - e->radius0) * ease_out_cubic(spread_t);
        }
        {
        float heat = e->heat0 * (float)heat_curve;
        if (heat <= 0.01f) continue;
        spots[n].dir[0] = e->dir[0];
        spots[n].dir[1] = e->dir[1];
        spots[n].dir[2] = e->dir[2];
        spots[n].tangent1[0] = e->tangent1[0];
        spots[n].tangent1[1] = e->tangent1[1];
        spots[n].tangent1[2] = e->tangent1[2];
        spots[n].angular_radius = (float)rad;
        spots[n].heat = heat;
        spots[n].progress = (float)t;
        spots[n].seed = 0.0f;
        spots[n].kind = e->kind;
        n++;
        }
    }

    /* The intersection boundary scar on the target is handled as an ImpactEvent
     * (COLLISION_VIS_INTERSECT kind) that is updated each physics step while
     * the merge is active.  No separate per-frame recomputation needed here. */

    return n;
}

double collision_visual_radius(int body_idx, double physical_radius)
{
    return current_visual_radius(body_idx, physical_radius);
}

void collision_body_heat_glow(int body_idx, float out_color[3],
                              float *out_intensity, float *out_scale)
{
    float best_heat = 0.0f;
    int has_merge = 0;
    double merge_growth = active_merge_glow_progress(body_idx);

    if (out_color) {
        out_color[0] = 0.0f;
        out_color[1] = 0.0f;
        out_color[2] = 0.0f;
    }
    if (out_intensity) *out_intensity = 0.0f;
    if (out_scale) *out_scale = 1.0f;
    if (body_idx < 0 || body_idx >= g_nbodies) return;

    for (int i = 0; i < MAX_IMPACTS; i++) {
        ImpactEvent *e = &s_impacts[i];
        double t, heat_curve;
        float heat;

        if (!e->active || e->body != body_idx) continue;
        t = e->age / e->duration;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        heat_curve = impact_heat_curve(e, t);
        heat = e->heat0 * (float)heat_curve;
        if (heat > best_heat) best_heat = heat;
        if (e->kind == COLLISION_VIS_MERGE || e->kind == COLLISION_VIS_INTERSECT)
            has_merge = 1;
    }

    if (best_heat <= 0.01f) return;
    best_heat = (best_heat - 0.01f) / 0.99f;
    if (best_heat < 0.0f) best_heat = 0.0f;
    if (best_heat > 1.0f) best_heat = 1.0f;
    best_heat = best_heat * best_heat * (3.0f - 2.0f * best_heat);
    if (merge_growth > 0.0) best_heat *= (float)merge_growth;

    if (out_color) {
        out_color[0] = 0.90f;
        out_color[1] = has_merge ? 0.34f : 0.26f;
        out_color[2] = 0.08f;
    }
    if (out_intensity)
        *out_intensity = best_heat * (has_merge ? 0.42f : 0.24f);
    if (out_scale)
        *out_scale = has_merge ? (1.12f + best_heat * 0.24f)
                               : (1.05f + best_heat * 0.10f);
}

int collision_body_has_active_merge(int body_idx)
{
    if (body_idx < 0 || body_idx >= g_nbodies) return 0;
    return body_is_in_merge(body_idx);
}

int collision_particles(CollisionParticle *out, int max_particles,
                        const double cam_pos[3])
{
    int n = 0;
    if (!out || max_particles <= 0 || !cam_pos) return 0;

    for (int i = 0; i < MAX_COLLISION_PARTICLES && n < max_particles; i++) {
        ImpactParticleState *p = &s_particles[i];
        double t, fade, alpha, fade_t;
        double dx, dy, dz, dist;
        double dist_fade = 1.0;
        if (!p->active) continue;
        t = p->age / p->duration;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        fade = 1.0 - t;
        fade_t = (t - p->fade_start) / (1.0 - p->fade_start);
        if (fade_t < 0.0) fade_t = 0.0;
        if (fade_t > 1.0) fade_t = 1.0;
        alpha = 1.0 - fade_t * fade_t;
        dx = p->pos[0] * RS - cam_pos[0];
        dy = p->pos[1] * RS - cam_pos[1];
        dz = p->pos[2] * RS - cam_pos[2];
        dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist >= 0.30 * AU * RS) continue;
        if (dist > 0.10 * AU * RS) {
            dist_fade = 1.0 - (dist - 0.10 * AU * RS) / (0.20 * AU * RS);
            if (dist_fade < 0.0) dist_fade = 0.0;
        }
        out[n].pos[0] = (float)dx;
        out[n].pos[1] = (float)dy;
        out[n].pos[2] = (float)dz;
        out[n].color[0] = p->color[0] * (0.70f + 0.30f * (float)fade);
        out[n].color[1] = p->color[1] * (0.70f + 0.30f * (float)fade);
        out[n].color[2] = p->color[2] * (0.70f + 0.30f * (float)fade);
        out[n].color[3] = (float)(alpha * dist_fade);
        out[n].size = p->size;
        n++;
    }
    return n;
}
