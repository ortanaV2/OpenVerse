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
#define MERGE_PHASE_SECONDS (DAY * 8.0)
#define MAX_MERGES 16

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

typedef struct {
    int active;
    int target;
    int impactor;
    double age;
    double duration;
    double rel_speed;
    double dir[3];
    double start_sep;
    double target_rotation_rate;   /* saved initial rates so both can slow together */
    double impactor_rotation_rate;
    int scar_slot;          /* s_impacts index for the target intersection scar */
    int imp_scar_slot;      /* s_impacts index for the impactor intersection scar */
} MergeEvent;

static ImpactEvent s_impacts[MAX_IMPACTS];
static RadiusTransition s_radius_fx[MAX_BODIES];
static MergeEvent s_merges[MAX_MERGES];
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

static int body_is_in_merge(int idx)
{
    for (int i = 0; i < MAX_MERGES; i++) {
        if (!s_merges[i].active) continue;
        if (s_merges[i].target == idx || s_merges[i].impactor == idx) return 1;
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
        e->radius0 = (float)fmax(0.010, fmin(0.070, radius_ratio * 0.70));
        e->radius1 = (float)fmax(e->radius0 + 0.020f, fmin(0.240, e->radius0 * 2.80f));
    } else if (kind == COLLISION_VIS_MAJOR) {
        e->duration = MAJOR_COOL_SECONDS;
        e->heat0 = (float)fmin(1.0, 0.45 + rel_speed / 22000.0 + mass_ratio * 0.4);
        e->radius0 = (float)fmax(0.030, fmin(0.140, radius_ratio * 1.05));
        e->radius1 = (float)fmax(e->radius0 + 0.060f, fmin(0.650, e->radius0 * 3.10f));
    } else {
        e->duration = MERGE_COOL_SECONDS;
        e->heat0 = 1.0f;
        e->radius0 = (float)fmax(0.120, fmin(0.280, 0.12 + mass_ratio * 0.28));
        e->radius1 = (float)fmax(2.30f, fmin(3.20f, 2.30f + mass_ratio * 0.70f));
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

static double merge_duration_for_bodies(int target, int impactor, double rel_speed)
{
    double radius_scale;
    double speed_scale;
    double duration;

    radius_scale = (g_bodies[target].radius + g_bodies[impactor].radius)
                 / (2.0 * 6371.0e3);
    if (radius_scale < 0.35) radius_scale = 0.35;

    speed_scale = 1.0 / (1.0 + rel_speed / 18000.0);
    if (speed_scale < 0.45) speed_scale = 0.45;

    duration = DAY * 1.18 * pow(radius_scale, 0.55) / speed_scale;
    if (duration < DAY * 1.2) duration = DAY * 1.2;
    if (duration > DAY * 14.0) duration = DAY * 14.0;
    return duration;
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
    fprintf(stdout, "[collision] %s absorbed %s (%.0f m/s, %s, %.0f->%.0f km)\n",
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

        /* Lock impactor velocity to target. */
        impactor->vel[0] = target->vel[0];
        impactor->vel[1] = target->vel[1];
        impactor->vel[2] = target->vel[2];

        /* Both rotations snap to nearly zero in the first 15% of the merge —
         * the deceleration should feel simultaneous with the impact. */
        {
            double slow = 1.0 - ease_out_cubic(fmin(1.0, t / 0.15));
            target->rotation_rate   = m->target_rotation_rate   * slow;
            impactor->rotation_rate = m->impactor_rotation_rate * slow;
        }

        {
            double merge_t = 0.78 * ease_out_cubic(fmin(1.0, t / 0.30))
                           + 0.22 * ease_out_cubic(t);
            overlap_sep = m->start_sep + (target->radius * 0.08 - m->start_sep) * merge_t;
        }
        impactor->pos[0] = target->pos[0] + m->dir[0] * overlap_sep;
        impactor->pos[1] = target->pos[1] + m->dir[1] * overlap_sep;
        impactor->pos[2] = target->pos[2] + m->dir[2] * overlap_sep;

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
                    float local_dir[3];
                    scar->radius0 = cap;
                    scar->radius1 = cap;
                    body_world_to_local_surface_dir(m->target, m->dir, local_dir);
                    normalize3f(local_dir);
                    scar->dir[0] = local_dir[0];
                    scar->dir[1] = local_dir[1];
                    scar->dir[2] = local_dir[2];
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
                    double neg_dir[3] = {-m->dir[0], -m->dir[1], -m->dir[2]};
                    scar->radius0 = cap;
                    scar->radius1 = cap;
                    body_world_to_local_surface_dir(m->impactor, neg_dir, local_dir);
                    normalize3f(local_dir);
                    scar->dir[0] = local_dir[0];
                    scar->dir[1] = local_dir[1];
                    scar->dir[2] = local_dir[2];
                }
            }
            scar->age = 0.0;
        }

        /* Despawn when the impactor is fully inside the target, or time runs out. */
        {
            double imp_r     = g_bodies[m->impactor].radius; /* constant, no shrink */
            double tgt_vis_r = collision_visual_radius(m->target, target->radius);
            if (overlap_sep + imp_r <= tgt_vis_r || m->age >= m->duration) {
                double old_radius = target->radius;

                /* Release target scar: set radius1 so it expands outward while cooling. */
                if (m->scar_slot >= 0 && m->scar_slot < MAX_IMPACTS &&
                    s_impacts[m->scar_slot].active && s_impacts[m->scar_slot].radius0 > 0.01f) {
                    float r0 = s_impacts[m->scar_slot].radius0;
                    s_impacts[m->scar_slot].radius1 = fminf(r0 * 2.2f, (float)(PI * 0.88));
                }

                m->active = 0;
                finalize_absorb_body(m->target, m->impactor, m->rel_speed,
                                     COLLISION_VIS_MERGE, old_radius);
            }
        }
    }
}

static void begin_merge_event(int target, int impactor, double rel_speed,
                              const double dir[3], double old_radius)
{
    int slot = -1;
    Body *a = &g_bodies[target];
    Body *b = &g_bodies[impactor];
    double total = a->mass + b->mass;
    double merged_radius;

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

    s_merges[slot].active    = 1;
    s_merges[slot].target    = target;
    s_merges[slot].impactor  = impactor;
    s_merges[slot].age       = 0.0;
    s_merges[slot].rel_speed = rel_speed;
    s_merges[slot].dir[0]    = dir[0];
    s_merges[slot].dir[1]    = dir[1];
    s_merges[slot].dir[2]    = dir[2];
    s_merges[slot].target_rotation_rate   = a->rotation_rate;
    s_merges[slot].impactor_rotation_rate = b->rotation_rate;
    s_merges[slot].scar_slot     = -1;
    s_merges[slot].imp_scar_slot = -1;

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
        scar->radius1  = 0.0f;
        scar->dir[0]   = local_dir[0];
        scar->dir[1]   = local_dir[1];
        scar->dir[2]   = local_dir[2];
        s_merges[slot].scar_slot = scar_slot;
    }

    /* Impactor-side scar: same setup, direction reversed. */
    {
        double neg_dir[3] = {-dir[0], -dir[1], -dir[2]};
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
        body_world_to_local_surface_dir(impactor, neg_dir, local_dir);
        normalize3f(local_dir);
        scar = &s_impacts[scar_slot];
        scar->active   = 1;
        scar->body     = impactor;
        scar->kind     = COLLISION_VIS_INTERSECT;
        scar->age      = 0.0;
        scar->duration = MERGE_COOL_SECONDS;
        scar->heat0    = 1.0f;
        scar->radius0  = 0.0f;
        scar->radius1  = 0.0f;
        scar->dir[0]   = local_dir[0];
        scar->dir[1]   = local_dir[1];
        scar->dir[2]   = local_dir[2];
        s_merges[slot].imp_scar_slot = scar_slot;
    }
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

    if (mass_ratio >= 0.45 || g_bodies[a].radius >= 0.74 * g_bodies[b].radius ||
        g_bodies[b].radius >= 0.74 * g_bodies[a].radius)
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
    double dir[3];
    int outcome;

    if (!a->alive || !b->alive || a->is_star || b->is_star) return;

    old_radius = a->radius;
    mass_ratio = fmin(a->mass, b->mass) / fmax(fmax(a->mass, b->mass), 1.0);
    outcome = classify_collision(target, impactor, rel_speed);
    impact_dir_for_pair(target, impactor, collision_dt, dir);
    if (outcome == COLLISION_VIS_MERGE) {
        begin_merge_event(target, impactor, rel_speed, dir, old_radius);
        return;
    }

    {
        double total = a->mass + b->mass;
    if (total <= 0.0) return;

    add_impact(target, outcome, dir, b->radius, rel_speed, mass_ratio);

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

        int root_had_collision = 0;
        for (int rj = 0; rj < active_root_count && !root_had_collision; rj++) {
            int other_root = active_roots[rj];
            int same_root = (other_root == root);
            int na, nb;

            if (other_root < 0 || other_root >= g_nbodies) continue;
            if (!g_bodies[other_root].alive) continue;
            if (other_root < root) continue;
            if (!systems_may_interact(root, other_root, system_radius)) continue;

            na = member_count[root];
            nb = member_count[other_root];
            for (int ai = 0; ai < na && !root_had_collision; ai++) {
                int a = members[root][ai];
                if (!g_bodies[a].alive || g_bodies[a].is_star || resolved[a] || body_is_in_merge(a)) continue;
                for (int bi = 0; bi < nb; bi++) {
                    int b = members[other_root][bi];
                    int lo, hi;
                    double speed = 0.0;

                    if (same_root && bi <= ai) continue;
                    if (a == b) continue;
                    if (!g_bodies[b].alive || g_bodies[b].is_star || resolved[b] || body_is_in_merge(b)) continue;

                    lo = a < b ? a : b;
                    hi = a < b ? b : a;

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
        {
            double spread_t;
            if (e->kind == COLLISION_VIS_MERGE)
                spread_t = fast_then_slow_spread(t, 0.09);
            else if (e->kind == COLLISION_VIS_INTERSECT)
                spread_t = fast_then_slow_spread(t, 0.07);
            else if (e->kind == COLLISION_VIS_MAJOR)
                spread_t = fast_then_slow_spread(t, 0.14);
            else
                spread_t = fast_then_slow_spread(t, 0.18);
            rad = e->radius0 + (e->radius1 - e->radius0) * ease_out_cubic(spread_t);
        }
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
        heat_curve = (1.0 - t) * (1.0 - t);
        if (e->kind == COLLISION_VIS_MERGE)
            heat_curve = pow(1.0 - t, 1.45);
        else if (e->kind == COLLISION_VIS_CRATER)
            heat_curve = pow(1.0 - t, 2.35);
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
