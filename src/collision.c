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
#define MAX_PAIR_DT (DAY * 20.0)
#define MIN_PAIR_DT (60.0 * 5.0)
#define HOT_PAIR_DT (60.0 * 10.0)
#define SYSTEM_HOT_DURATION (DAY * 3.0)
#define MIN_COLLISION_SPEED 1.0
#define SYSTEM_MARGIN_AU 5.0
#define IMPACT_COOL_SECONDS (DAY * 45.0)
#define MAJOR_COOL_SECONDS (DAY * 90.0)
#define MERGE_COOL_SECONDS (DAY * 180.0)

typedef struct {
    int active;
    int body;
    int kind;
    double age;
    double duration;
    float dir[3];
    float radius0;
    float radius1;
    float heat0;
} ImpactEvent;

typedef struct {
    int active;
    double age;
    double duration;
    double start_radius;
    double target_radius;
} RadiusTransition;

static ImpactEvent s_impacts[MAX_IMPACTS];
static RadiusTransition s_radius_fx[MAX_BODIES];
static double s_pair_next[MAX_BODIES][MAX_BODIES];
static unsigned char s_system_dirty[MAX_BODIES];
static double s_system_hot[MAX_BODIES];

static void mark_system_dirty(int root, double hot_duration)
{
    if (root < 0 || root >= MAX_BODIES) return;
    s_system_dirty[root] = 1;
    if (hot_duration > s_system_hot[root]) s_system_hot[root] = hot_duration;
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

static void normalize3f(float v[3])
{
    float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (len <= 1e-8f) {
        v[0] = 1.0f; v[1] = 0.0f; v[2] = 0.0f;
        return;
    }
    v[0] /= len; v[1] /= len; v[2] /= len;
}

static double ease_out_cubic(double t)
{
    double inv;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    inv = 1.0 - t;
    return 1.0 - inv*inv*inv;
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

static void start_radius_transition(int body_idx, double old_radius,
                                    double new_radius, double duration)
{
    RadiusTransition *fx;
    double start_radius;
    if (body_idx < 0 || body_idx >= MAX_BODIES) return;
    fx = &s_radius_fx[body_idx];
    fx->active = 1;
    fx->age = 0.0;
    fx->duration = fmax(duration, 1.0);
    start_radius = current_visual_radius(body_idx, old_radius);
    if (new_radius > 0.0 && start_radius < new_radius * 0.35)
        start_radius = new_radius * 0.35;
    if (start_radius < old_radius)
        start_radius = old_radius;
    fx->start_radius = start_radius;
    fx->target_radius = new_radius;
}

static void add_impact(int body_idx, int kind, const double world_dir[3],
                       double impactor_radius, double rel_speed,
                       double mass_ratio)
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
        e->duration = IMPACT_COOL_SECONDS;
        e->heat0 = (float)fmin(0.95, 0.22 + rel_speed / 32000.0);
        e->radius0 = (float)fmax(0.020, fmin(0.120, radius_ratio * 1.20));
        e->radius1 = (float)fmax(e->radius0, fmin(0.180, e->radius0 * 1.35f));
    } else if (kind == COLLISION_VIS_MAJOR) {
        e->duration = MAJOR_COOL_SECONDS;
        e->heat0 = (float)fmin(1.0, 0.45 + rel_speed / 22000.0 + mass_ratio * 0.4);
        e->radius0 = (float)fmax(0.060, fmin(0.240, radius_ratio * 1.70));
        e->radius1 = (float)fmax(e->radius0 + 0.040f, fmin(0.420, e->radius0 * 1.85f));
    } else {
        e->duration = MERGE_COOL_SECONDS;
        e->heat0 = 1.0f;
        e->radius0 = (float)fmax(0.120, fmin(0.320, 0.14 + mass_ratio * 0.42));
        e->radius1 = (float)fmax(0.650, fmin(1.180, e->radius0 + 0.58f + mass_ratio * 0.32f));
    }
    body_world_to_local_surface_dir(body_idx, world_dir, e->dir);
    normalize3f(e->dir);
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

static int classify_collision(int a, int b, double rel_speed)
{
    double m_small = g_bodies[a].mass < g_bodies[b].mass ? g_bodies[a].mass : g_bodies[b].mass;
    double m_large = g_bodies[a].mass > g_bodies[b].mass ? g_bodies[a].mass : g_bodies[b].mass;
    double r_sum = g_bodies[a].radius + g_bodies[b].radius;
    double mass_ratio, v_escape;

    if (m_large <= 0.0) return COLLISION_VIS_CRATER;
    mass_ratio = m_small / m_large;
    v_escape = sqrt(fmax(0.0, 2.0 * G_CONST * (g_bodies[a].mass + g_bodies[b].mass) /
                         fmax(r_sum, 1.0)));

    if (mass_ratio >= 0.33 || g_bodies[a].radius >= 0.58 * g_bodies[b].radius ||
        g_bodies[b].radius >= 0.58 * g_bodies[a].radius)
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
    double gap = dist - (g_bodies[a].radius + g_bodies[b].radius);

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
    double rel_v[3] = {
        g_bodies[b].vel[0] - g_bodies[a].vel[0],
        g_bodies[b].vel[1] - g_bodies[a].vel[1],
        g_bodies[b].vel[2] - g_bodies[a].vel[2]
    };
    double rel_p[3] = {
        rel_now[0] - rel_v[0] * dt,
        rel_now[1] - rel_v[1] * dt,
        rel_now[2] - rel_v[2] * dt
    };
    double vv = dot3d(rel_v, rel_v);
    double t = 0.0;
    if (vv > MIN_COLLISION_SPEED * MIN_COLLISION_SPEED) {
        t = -dot3d(rel_p, rel_v) / vv;
        if (t < 0.0) t = 0.0;
        if (t > dt) t = dt;
    }

    double c[3] = {
        rel_p[0] + rel_v[0] * t,
        rel_p[1] + rel_v[1] * t,
        rel_p[2] + rel_v[2] * t
    };
    double r = g_bodies[a].radius + g_bodies[b].radius;
    if (out_speed) *out_speed = sqrt(vv);
    return dot3d(c, c) <= r*r;
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
    double merged_radius;
    double dir[3];
    int outcome;

    if (!a->alive || !b->alive || a->is_star || b->is_star) return;

    old_radius = a->radius;
    mass_ratio = fmin(a->mass, b->mass) / fmax(fmax(a->mass, b->mass), 1.0);
    outcome = classify_collision(target, impactor, rel_speed);
    {
        double total = a->mass + b->mass;
    if (total <= 0.0) return;

    impact_dir_for_pair(target, impactor, collision_dt, dir);
    add_impact(target, outcome, dir, b->radius, rel_speed, mass_ratio);

    a->pos[0] = (a->pos[0] * a->mass + b->pos[0] * b->mass) / total;
    a->pos[1] = (a->pos[1] * a->mass + b->pos[1] * b->mass) / total;
    a->pos[2] = (a->pos[2] * a->mass + b->pos[2] * b->mass) / total;
    a->vel[0] = (a->vel[0] * a->mass + b->vel[0] * b->mass) / total;
    a->vel[1] = (a->vel[1] * a->mass + b->vel[1] * b->mass) / total;
    a->vel[2] = (a->vel[2] * a->mass + b->vel[2] * b->mass) / total;
    merged_radius = cbrt(a->radius*a->radius*a->radius +
                         b->radius*b->radius*b->radius);
    if (merged_radius < old_radius) merged_radius = old_radius;
    if (merged_radius < b->radius) merged_radius = b->radius;
    a->radius = merged_radius;
    a->mass = total;
    }

    if (outcome == COLLISION_VIS_MERGE)
        start_radius_transition(target, old_radius, a->radius, DAY * 12.0);
    else if (outcome == COLLISION_VIS_MAJOR)
        start_radius_transition(target, old_radius, a->radius, DAY * 5.0);
    else if (a->radius > old_radius * 1.015)
        start_radius_transition(target, old_radius, a->radius, DAY * 1.5);

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
    fprintf(stdout, "[collision] %s absorbed %s (%.0f m/s, %s, %.0f->%.0f km)\n",
            a->name, b->name, rel_speed,
            outcome == COLLISION_VIS_MERGE ? "merge" :
            outcome == COLLISION_VIS_MAJOR ? "major" : "crater",
            old_radius / 1000.0, a->radius / 1000.0);
}

void collision_step(double dt)
{
    int root_of[MAX_BODIES];
    double system_radius[MAX_BODIES];
    int resolved[MAX_BODIES] = {0};
    int members[MAX_BODIES][MAX_BODIES];
    int member_count[MAX_BODIES] = {0};
    int any_dirty = 0;

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
        if (s_system_dirty[i]) any_dirty = 1;
        if (s_system_hot[i] > 0.0) {
            s_system_hot[i] -= dt;
            if (s_system_hot[i] < 0.0) s_system_hot[i] = 0.0;
        }
    }
    if (!any_dirty) return;

    for (int i = 0; i < MAX_BODIES; i++) {
        root_of[i] = -1;
        system_radius[i] = SYSTEM_MARGIN_AU * AU;
    }
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        int root = body_root_star(i);
        root_of[i] = root;
        if (root < 0 || root >= g_nbodies) continue;
        if (member_count[root] < MAX_BODIES)
            members[root][member_count[root]++] = i;
        double dx = g_bodies[i].pos[0] - g_bodies[root].pos[0];
        double dy = g_bodies[i].pos[1] - g_bodies[root].pos[1];
        double dz = g_bodies[i].pos[2] - g_bodies[root].pos[2];
        double d = sqrt(dx*dx + dy*dy + dz*dz) + g_bodies[i].radius;
        if (d > system_radius[root]) system_radius[root] = d;
    }

    for (int root = 0; root < g_nbodies; root++) {
        if (!s_system_dirty[root]) continue;
        if (!g_bodies[root].alive) {
            s_system_dirty[root] = 0;
            s_system_hot[root] = 0.0;
            continue;
        }

        int n = member_count[root];
        int root_had_collision = 0;
        for (int ai = 0; ai < n && !root_had_collision; ai++) {
            int a = members[root][ai];
            if (!g_bodies[a].alive || g_bodies[a].is_star || resolved[a]) continue;
            for (int bi = ai + 1; bi < n; bi++) {
                int b = members[root][bi];
                int lo = a < b ? a : b;
                int hi = a < b ? b : a;
                double speed = 0.0;

                if (!g_bodies[b].alive || g_bodies[b].is_star || resolved[b]) continue;
                if (!systems_may_interact(root_of[a], root_of[b], system_radius)) continue;

                if (s_system_hot[root] > 0.0) {
                    s_pair_next[lo][hi] = HOT_PAIR_DT;
                } else {
                    s_pair_next[lo][hi] -= dt;
                    if (s_pair_next[lo][hi] > 0.0) continue;
                }

                if (!swept_spheres_collide(a, b, dt, &speed)) {
                    s_pair_next[lo][hi] = pair_check_dt(a, b);
                    continue;
                }

                {
                    int target = g_bodies[a].mass >= g_bodies[b].mass ? a : b;
                    int impactor = target == a ? b : a;
                    absorb_body(target, impactor, speed, dt);
                    resolved[target] = 1;
                    resolved[impactor] = 1;
                    s_pair_next[lo][hi] = HOT_PAIR_DT;
                    root_had_collision = 1;
                    break;
                }
            }
        }

        (void)root_had_collision;
    }
}

int collision_spots_for_body(int body_idx, CollisionSpot spots[COLLISION_MAX_SPOTS])
{
    int n = 0;
    if (!spots || body_idx < 0 || body_idx >= g_nbodies) return 0;

    for (int i = 0; i < MAX_IMPACTS && n < COLLISION_MAX_SPOTS; i++) {
        ImpactEvent *e = &s_impacts[i];
        double t;
        double heat_curve;
        double rad;
        if (!e->active || e->body != body_idx) continue;
        t = e->age / e->duration;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        heat_curve = (1.0 - t) * (1.0 - t);
        if (e->kind == COLLISION_VIS_MERGE)
            heat_curve = pow(1.0 - t, 1.45);
        else if (e->kind == COLLISION_VIS_CRATER)
            heat_curve = pow(1.0 - t, 2.35);
        rad = e->radius0 + (e->radius1 - e->radius0) * ease_out_cubic(t);
        {
        float heat = e->heat0 * (float)heat_curve;
        if (heat <= 0.01f) continue;
        spots[n].dir[0] = e->dir[0];
        spots[n].dir[1] = e->dir[1];
        spots[n].dir[2] = e->dir[2];
        spots[n].angular_radius = (float)rad;
        spots[n].heat = heat;
        spots[n].progress = (float)t;
        spots[n].kind = e->kind;
        n++;
        }
    }
    return n;
}

double collision_visual_radius(int body_idx, double physical_radius)
{
    return current_visual_radius(body_idx, physical_radius);
}
