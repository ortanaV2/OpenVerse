/*
 * rings.c — Keplerian ring particle system, data-driven from universe.json
 *
 * ── Physics model ────────────────────────────────────────────────────────
 *
 * Each ring particle follows a Keplerian elliptical orbit:
 *   - The CPU advances the mean anomaly M each physics step: M += n × dt
 *     where n = √(GM/a³) is the mean motion in rad/s.
 *   - The GPU (ring.vert) evaluates a first-order Kepler approximation each
 *     frame to compute particle positions.  This keeps the O(N) position work
 *     on the GPU while staying cheap enough for particle-only distant rings.
 *
 * ── Per-particle data layout (8 floats per particle) ────────────────────
 *
 *   [0] M0    — current mean anomaly (radians), updated by rings_tick()
 *   [1] a_au  — semi-major axis (AU)
 *   [2] e     — eccentricity
 *   [3] omega — argument of periapsis (radians)
 *   [4] h     — vertical displacement from ring plane (AU), small for thin disc
 *   [5] r     — red channel (particle color)
 *   [6] g     — green channel
 *   [7] b     — blue channel
 *
 * The corresponding n_arr[] stores the pre-computed mean motion (rad/s) for
 * each particle at the same index.
 *
 * ── Semi-major axis distribution ─────────────────────────────────────────
 *
 * Uniform sampling in a² (instead of in a) gives equal particle area density
 * across the disc annulus:
 *   a = √(r²_min + u × (r²_max − r²_min))   where u ~ Uniform[0,1]
 * This mirrors the asteroids.c belt distribution; area-uniform sampling avoids
 * over-density near the inner edge.
 *
 * ── Ring-plane orientation ────────────────────────────────────────────────
 *
 * Ring plane = perpendicular to the planet's rotation pole.
 * The basis {b1, b2, pole} is built by rotating the ecliptic frame around X
 * by the planet's obliquity angle:
 *   b1   = (1, 0, 0)                  — fixed ecliptic X
 *   b2   = (0, sin(obl), −cos(obl))   — 90° from pole in the ring plane
 *   pole = (0, cos(obl),  sin(obl))   — planet's rotation axis
 *
 * ── LOD and distance fade ─────────────────────────────────────────────────
 *
 *   dist ≤ LOD_DIST       → full particle count
 *   dist ∈ LOD fade range → reduced particle count, drawn from a shuffled LOD
 *                           buffer so all ring zones remain represented
 *   dist ∈ fade range     → reduced particles fade out smoothly
 *
 * Sections (search "§"):
 *   § STATE    — constants, structs, static arrays, deterministic RNG
 *   § MATH     — local scalar/vector helpers and planet filtering
 *   § RESPONSE — smoothed visual state for shocks, tides, and merge handoff
 *   § ORBITS   — basis construction, retuning, particle baking, damage passes
 *   § CONTACT  — swept ring/body hitboxes, despawn, tidal perturbation
 *   § GL       — VAO/VBO setup, distance LOD, render path
 *   § API      — public lifecycle, ticking, rendering, collision hooks
 */
#include "rings.h"
#include "common.h"
#include "math3d.h"
#include "body.h"
#include "camera.h"
#include "collision.h"
#include "physics.h"
#include "gl_utils.h"
#include "json.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define LOD_DIST      0.05f   /* AU: full particle count up to this range */
#define LOD_FADE_END  0.24f   /* AU: reduced particle count reached here */
#define RING_FADE_START 0.34f /* AU: distant particles start fading out */
#define RING_FADE_END   0.62f /* AU: rings are fully hidden past this */
#define RING_SHADOW_FULL_DIST 0.08f /* AU: full planet shadow detail up to this range */
#define RING_SHADOW_FADE_END  0.18f /* AU: planet shadow skipped past this range */
#define MAX_ZONES     16      /* maximum annulus zones per ring descriptor */
#define RING_COLLISION_SEGMENTS 32
#define RING_COLLISION_RADIAL_BINS 4
#define DAMAGE_MIN_WIDTH     0.18f
#define DAMAGE_MAX_ECC       0.080f
#define DAMAGE_MIN_A_SCALE   0.88f
#define DAMAGE_MAX_A_SCALE   1.18f
#define HIT_SEGMENT_COOLDOWN_SECONDS (DAY * 0.18)
#define RING_SWEEP_MAX_SAMPLES 5
#define MORPH_PUFF_RELAX_MIN  (60.0f * 4.0f)
#define MORPH_PUFF_RELAX_MAX  ((float)DAY * 0.10f)
#define MORPH_DECAY_MIN       (60.0f * 30.0f)
#define MORPH_DECAY_MAX       ((float)DAY * 1.50f)
#define PARENT_TRACK_MIN      (60.0f * 20.0f)
#define PARENT_TRACK_MAX      ((float)DAY * 0.35f)
#define RESPONSE_VISUAL_DT_MAX (60.0f * 12.0f)
#define TIDAL_MAX_DT          ((double)DAY * 0.015)
#define TIDAL_VISUAL_GAIN     5.5
#define TIDAL_MAX_DA_FRAC     0.0015f
#define TIDAL_MAX_DE          0.0012f
#define TIDAL_WARP_MIN_WIDTH  0.14f
#define TIDAL_WARP_MAX_WIDTH  0.95f
#define RING_TRANSFER_MIN_SECONDS ((float)DAY * 0.75f)
#define RING_TRANSFER_MAX_SECONDS ((float)DAY * 8.0f)

/* ── § STATE — constants, structs, static arrays, deterministic RNG ────── */

/* One concentric annulus band from assets/universe.json. */
typedef struct {
    float r_min, r_max;   /* inner / outer radius in km */
    float density;        /* fraction of total particles allocated to this zone */
    float r, g, b;        /* base particle color */
} Zone;

/* ParticleDisc is an internal, self-contained ring system for one planet.
 *
 * Per-particle data layout in data_full / data_lod (8 floats):
 *   [0] M      — mean anomaly (rad), advanced by rings_tick()
 *   [1] a      — semi-major axis (AU); negative values are despawn tombstones
 *   [2] e      — eccentricity
 *   [3] omega  — argument of periapsis (rad)
 *   [4] height — vertical offset from the ring plane (AU)
 *   [5..7] rgb — particle colour
 */
typedef struct {
    int    parent_idx;    /* g_bodies index of central body               */
    int    n_full;        /* close-range particle count                   */
    int    n_lod;         /* reduced particle count for distant rendering */
    float *data_full;     /* 8 floats × n_full                            */
    float *data_lod;      /* 8 floats × n_lod                             */
    float *n_arr_full;    /* Keplerian mean motion (rad/s), CPU only      */
    float *n_arr_lod;

    float  ring_r_inner_km;
    float  ring_r_outer_km;
    float  parent_radius_ref_km;
    float  base_h_scale;
    float  mean_motion_mid;

    float  b1[3], b2[3], pole[3]; /* parent ring-plane basis */
    float  transfer_b1[3], transfer_b2[3], transfer_pole[3];
    float  transfer_offset[3];
    float  transfer_age;
    float  transfer_duration;
    float  motion_scale_start;    /* old/new mean-motion ratio during retune */
    float  motion_blend_age;
    float  motion_blend_duration;

    GLuint shader;
    GLint  loc_vp, loc_center, loc_b1, loc_b2, loc_pole;
    GLint  loc_morph0, loc_morph1, loc_morph2;
    GLint  loc_tide0, loc_tide1;
    GLint  loc_body0, loc_body1;
    GLint  loc_light0, loc_shadow_strength;
    GLuint vao_full, vbo_full;
    GLuint vao_lod,  vbo_lod;

    float  hit_cooldown[RING_COLLISION_SEGMENTS * RING_COLLISION_RADIAL_BINS];
    float  puff_cur, puff_target;
    float  shock_phase, shock_width, shock_amp, shock_spin;
    float  contact_norm, contact_width, contact_strength;
    float  tide_phase, tide_radius_norm, tide_width, tide_strength;
    float  tide_dir_u, tide_dir_v, tide_dir_n;
    float  body_u_au, body_v_au, body_n_au, body_radius_au, body_strength;

    int    initialized;
} ParticleDisc;

/* Disc instances loaded from the "rings" array in assets/universe.json. */
static ParticleDisc *s_discs   = NULL;
static int           s_n_discs = 0;

/*
 * XorShift32 PRNG.
 *
 * Used for deterministic ring particle generation.  The seed is set per-disc
 * from "seed_full" / "seed_lod" in the JSON so repeated runs keep the same
 * ring layout and LOD shuffle order.
 */
static uint32_t s_rng = 1;
static void  s_seed(uint32_t seed) { s_rng = seed ? seed : 1; }
static float s_randf(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng >> 8) * (1.0f / (float)(1 << 24));
}

/* ── § MATH — local scalar/vector helpers and planet filtering ──────────── */


static float wrap_angle_pi(float a)
{
    const float TWO_PI = 6.28318530718f;
    while (a >  (float)PI) a -= TWO_PI;
    while (a < -(float)PI) a += TWO_PI;
    return a;
}

static float angular_falloff(float dphi, float half_width)
{
    float t;
    dphi = fabsf(dphi);
    if (half_width <= 1e-5f || dphi >= half_width) return 0.0f;
    t = 1.0f - dphi / half_width;
    return t * t * (3.0f - 2.0f * t);
}

static float dot3f_local(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void normalize3f_local(float v[3])
{
    float len = sqrtf(dot3f_local(v, v));
    if (len <= 1e-8f) {
        v[0] = 1.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        return;
    }
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}

static void mix3f_local(const float a[3], const float b[3], float t, float out[3])
{
    out[0] = a[0] + (b[0] - a[0]) * t;
    out[1] = a[1] + (b[1] - a[1]) * t;
    out[2] = a[2] + (b[2] - a[2]) * t;
}

static float smootherstep01f(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static float inv_smootherstep01f(float t)
{
    return 1.0f - smootherstep01f(t);
}

/*
 * body_is_ring_perturber_planet — filter ring collision influences.
 *
 * Ring warping should react to planets, not moons or asteroid-sized debris.
 * Body.type is not preserved after JSON loading, so this uses stable runtime
 * properties: alive, non-star, star-parented, and planet-scale radius/mass.
 */
static int body_is_ring_perturber_planet(int idx)
{
    if (idx < 0 || idx >= g_nbodies) return 0;
    if (!g_bodies[idx].alive || g_bodies[idx].is_star) return 0;
    if (g_bodies[idx].parent < 0 || !g_bodies[g_bodies[idx].parent].is_star) return 0;

    return g_bodies[idx].radius >= 2000.0e3 || g_bodies[idx].mass >= 1.0e23;
}

/* ── § RESPONSE — shocks, tides, and merge handoff state ────────────────── */

static void disc_update_mid_motion(ParticleDisc *d)
{
    double gm;
    double a_mid_m;

    if (!d || d->parent_idx < 0 || d->parent_idx >= g_nbodies) return;
    gm = G_CONST * g_bodies[d->parent_idx].mass;
    a_mid_m = ((double)d->ring_r_inner_km + (double)d->ring_r_outer_km) * 500.0;
    d->mean_motion_mid = (gm > 0.0 && a_mid_m > 0.0)
        ? (float)sqrt(gm / (a_mid_m * a_mid_m * a_mid_m))
        : 0.0f;
}

static float disc_mid_period_seconds(const ParticleDisc *d)
{
    if (!d || d->mean_motion_mid <= 1e-8f) return (float)DAY * 0.5f;
    return (2.0f * (float)PI) / d->mean_motion_mid;
}

static void disc_clear_hit_cooldown(ParticleDisc *d)
{
    if (!d) return;
    for (int i = 0; i < RING_COLLISION_SEGMENTS * RING_COLLISION_RADIAL_BINS; i++)
        d->hit_cooldown[i] = 0.0f;
}

static void disc_reset_response(ParticleDisc *d)
{
    if (!d) return;
    disc_clear_hit_cooldown(d);
    d->puff_cur = 0.0f;
    d->puff_target = 0.0f;
    d->shock_phase = 0.0f;
    d->shock_width = 0.35f;
    d->shock_amp = 0.0f;
    d->shock_spin = 0.0f;
    d->contact_norm = 0.5f;
    d->contact_width = 0.25f;
    d->contact_strength = 0.0f;
    d->tide_phase = 0.0f;
    d->tide_radius_norm = 0.5f;
    d->tide_width = 0.35f;
    d->tide_strength = 0.0f;
    d->tide_dir_u = 1.0f;
    d->tide_dir_v = 0.0f;
    d->tide_dir_n = 0.0f;
    d->transfer_b1[0] = 1.0f; d->transfer_b1[1] = 0.0f; d->transfer_b1[2] = 0.0f;
    d->transfer_b2[0] = 0.0f; d->transfer_b2[1] = 1.0f; d->transfer_b2[2] = 0.0f;
    d->transfer_pole[0] = 0.0f; d->transfer_pole[1] = 0.0f; d->transfer_pole[2] = 1.0f;
    d->transfer_offset[0] = d->transfer_offset[1] = d->transfer_offset[2] = 0.0f;
    d->transfer_age = 0.0f;
    d->transfer_duration = 0.0f;
    d->motion_scale_start = 1.0f;
    d->motion_blend_age = 0.0f;
    d->motion_blend_duration = 0.0f;
    d->body_u_au = 0.0f;
    d->body_v_au = 0.0f;
    d->body_n_au = 0.0f;
    d->body_radius_au = 0.0f;
    d->body_strength = 0.0f;
}

static float smooth_raise(float cur, float target, float rate)
{
    if (target <= cur) return cur;
    return cur + (target - cur) * clampf(rate, 0.0f, 1.0f);
}

/*
 * smooth_raise_capped — smooth_raise with a per-call maximum step.
 *
 * This prevents first-contact samples from injecting too much visual energy in
 * one frame, which previously read as a ring twitch at high simulation speeds.
 */
static float smooth_raise_capped(float cur, float target,
                                 float rate, float max_step)
{
    float next;

    if (target <= cur) return cur;
    next = smooth_raise(cur, target, rate);
    max_step = fmaxf(max_step, 0.0f);
    if (next - cur > max_step) next = cur + max_step;
    return next;
}

/*
 * disc_drive_response — drive local shock/contact warp uniforms.
 *
 * Called from both one-shot collision notifications and swept contact samples.
 * The response is intentionally stateful and damped so repeated samples blend
 * into a continuous warp instead of teleporting the active contact sector.
 */
static void disc_drive_response(ParticleDisc *d, float phase, float half_width,
                                float severity, float overlap,
                                float radial_center, float radial_width)
{
    float shock_target;
    float contact_target;
    int soft_start;

    if (!d) return;
    severity = clampf(severity, 0.0f, 1.0f);
    overlap = clampf(overlap, 0.0f, 1.0f);
    soft_start = (d->shock_amp <= 0.018f && d->contact_strength <= 0.030f);

    if (d->shock_amp <= 0.001f) {
        d->shock_phase = phase;
    } else {
        float phase_blend = 0.035f + 0.080f * overlap;
        d->shock_phase = wrap_angle_pi(d->shock_phase +
                         wrap_angle_pi(phase - d->shock_phase) * phase_blend);
    }
    d->shock_width = fmaxf(d->shock_width, half_width);
    shock_target = (0.050f + 0.135f * severity) * (0.30f + 0.70f * overlap);
    d->shock_amp = smooth_raise_capped(d->shock_amp,
                                       shock_target,
                                       soft_start ? (0.035f + 0.045f * overlap)
                                                  : (0.10f + 0.11f * overlap),
                                       soft_start ? (0.012f + 0.014f * overlap)
                                                  : (0.060f + 0.050f * overlap));
    d->shock_spin = d->mean_motion_mid * (0.22f + 0.60f * severity);
    d->puff_target = fmaxf(d->puff_target,
                           soft_start
                               ? (0.06f + 0.24f * severity * overlap)
                               : (0.13f + 0.48f * severity * overlap));

    radial_center = clampf(radial_center, 0.0f, 1.0f);
    radial_width = clampf(radial_width, 0.06f, 0.80f);
    if (d->contact_strength <= 0.001f) {
        d->contact_norm = radial_center;
        d->contact_width = radial_width;
    } else {
        float blend = 0.08f + 0.16f * overlap;
        d->contact_norm += (radial_center - d->contact_norm) * blend;
        d->contact_width += (radial_width - d->contact_width) * blend;
    }
    contact_target = (0.16f + 0.62f * severity) * overlap;
    d->contact_strength = smooth_raise_capped(d->contact_strength,
                                              contact_target,
                                              soft_start ? (0.030f + 0.055f * overlap)
                                                         : (0.085f + 0.145f * overlap),
                                              soft_start ? (0.014f + 0.018f * overlap)
                                                         : (0.070f + 0.045f * overlap));
}

/*
 * disc_drive_tide_response - smooth 3D visual pull from a nearby planet.
 *
 * The swept hitbox pass already knows the perturber position in the ring's
 * local frame.  Instead of spawning real ring segments, keep one dominant,
 * damped response vector and let the vertex shader bend only the particles
 * under that local influence.  This preserves the original ring rendering
 * path while making close planet encounters read as volumetric, not 2D.
 */
static void disc_drive_tide_response(ParticleDisc *d,
                                     double ru, double rv, double rh,
                                     double proj_r_km, double h_km,
                                     double other_r_km, double reach_km)
{
    double ring_inner_km, ring_outer_km, ring_width_km;
    double radial_gap_km, proximity_km, local_len;
    float phase, radial_norm, width, strength, blend, body_blend;
    float body_u_au, body_v_au, body_n_au, body_radius_au;
    float dir_u, dir_v, dir_n;

    if (!d || reach_km <= 1.0 || other_r_km <= 1.0) return;

    ring_inner_km = (double)d->ring_r_inner_km;
    ring_outer_km = (double)d->ring_r_outer_km;
    ring_width_km = fmax(ring_outer_km - ring_inner_km, 1.0);

    if (proj_r_km < ring_inner_km)
        radial_gap_km = ring_inner_km - proj_r_km;
    else if (proj_r_km > ring_outer_km)
        radial_gap_km = proj_r_km - ring_outer_km;
    else
        radial_gap_km = 0.0;

    proximity_km = sqrt(radial_gap_km * radial_gap_km + h_km * h_km);
    strength = clampf((float)(1.0 - proximity_km / fmax(reach_km, 1.0)), 0.0f, 1.0f);
    if (strength <= 0.0f) return;

    local_len = sqrt(ru*ru + rv*rv + rh*rh);
    if (local_len <= 1.0) return;

    phase = atan2f((float)rv, (float)ru);
    radial_norm = clampf((float)((proj_r_km - ring_inner_km) / ring_width_km), 0.0f, 1.0f);
    width = clampf((float)(other_r_km / ring_width_km) * 2.10f + strength * 0.28f,
                   TIDAL_WARP_MIN_WIDTH, TIDAL_WARP_MAX_WIDTH);

    dir_u = (float)(ru / local_len);
    dir_v = (float)(rv / local_len);
    dir_n = (float)(rh / local_len);

    strength *= clampf((float)(other_r_km / fmax(ring_width_km * 0.14, 1.0)),
                       0.45f, 1.55f);
    strength = clampf(strength * 1.05f, 0.0f, 1.0f);
    body_u_au = (float)(ru * RS);
    body_v_au = (float)(rv * RS);
    body_n_au = (float)(rh * RS);
    body_radius_au = (float)(other_r_km / 1.496e8);

    if (d->tide_strength <= 0.001f) {
        d->tide_phase = phase;
        d->tide_radius_norm = radial_norm;
        d->tide_width = width;
        d->tide_dir_u = dir_u;
        d->tide_dir_v = dir_v;
        d->tide_dir_n = dir_n;
        d->body_u_au = body_u_au;
        d->body_v_au = body_v_au;
        d->body_n_au = body_n_au;
        d->body_radius_au = body_radius_au;
    } else {
        blend = 0.045f + 0.155f * strength;
        d->tide_phase = wrap_angle_pi(d->tide_phase +
                         wrap_angle_pi(phase - d->tide_phase) * blend);
        d->tide_radius_norm += (radial_norm - d->tide_radius_norm) * blend;
        d->tide_width += (width - d->tide_width) * blend;
        d->tide_dir_u += (dir_u - d->tide_dir_u) * blend;
        d->tide_dir_v += (dir_v - d->tide_dir_v) * blend;
        d->tide_dir_n += (dir_n - d->tide_dir_n) * blend;
        body_blend = 0.18f + 0.32f * strength;
        d->body_u_au += (body_u_au - d->body_u_au) * body_blend;
        d->body_v_au += (body_v_au - d->body_v_au) * body_blend;
        d->body_n_au += (body_n_au - d->body_n_au) * body_blend;
        d->body_radius_au += (body_radius_au - d->body_radius_au) * body_blend;

        local_len = sqrt((double)d->tide_dir_u * d->tide_dir_u +
                         (double)d->tide_dir_v * d->tide_dir_v +
                         (double)d->tide_dir_n * d->tide_dir_n);
        if (local_len > 1e-6) {
            d->tide_dir_u = (float)(d->tide_dir_u / local_len);
            d->tide_dir_v = (float)(d->tide_dir_v / local_len);
            d->tide_dir_n = (float)(d->tide_dir_n / local_len);
        }
    }

    d->tide_strength = smooth_raise(d->tide_strength,
                                    strength,
                                    0.10f + 0.24f * strength);
    d->body_strength = smooth_raise(d->body_strength,
                                    clampf(strength * 1.45f, 0.0f, 1.0f),
                                    0.18f + 0.30f * strength);
}

/*
 * disc_drive_transfer_response — soft shock from parent-radius growth.
 *
 * During a merge the target planet can grow under its existing ring.  This
 * produces a broad, gentle response so the handoff reads as gravitational
 * disturbance rather than an instantaneous scale change.
 */
static void disc_drive_transfer_response(ParticleDisc *d,
                                         float old_parent_radius_km,
                                         float new_parent_radius_km,
                                         float severity_bias)
{
    float ratio;
    float severity;

    if (!d || new_parent_radius_km <= 0.0f) return;
    if (old_parent_radius_km <= 0.0f) old_parent_radius_km = new_parent_radius_km;

    ratio = new_parent_radius_km / fmaxf(old_parent_radius_km, 1.0f);
    severity = clampf(fabsf(ratio - 1.0f) * 0.85f + severity_bias, 0.0f, 1.0f);

    d->puff_target = fmaxf(d->puff_target, 0.12f + 0.30f * severity);
    d->shock_amp = fmaxf(d->shock_amp, 0.035f + 0.070f * severity);
    d->shock_width = fmaxf(d->shock_width, 0.65f + 0.35f * severity);
    if (d->mean_motion_mid > 1e-8f)
        d->shock_spin = d->mean_motion_mid * (0.14f + 0.34f * severity);
    d->parent_radius_ref_km = old_parent_radius_km;
}

/*
 * disc_begin_visual_transfer - hide ownership jumps during merge retunes.
 *
 * Ring physics and ownership switch to the new parent immediately, but the
 * rendered frame keeps the old center/basis as a decaying offset.  This makes
 * impactor-ring handoff read as a continuous merge instead of a teleport.
 */
static void disc_begin_visual_transfer(ParticleDisc *d,
                                       const double old_center_m[3],
                                       const float old_b1[3],
                                       const float old_b2[3],
                                       const float old_pole[3],
                                       int new_parent_idx)
{
    double dx, dy, dz, dist_m;
    float duration;

    if (!d || !old_center_m || new_parent_idx < 0 || new_parent_idx >= g_nbodies) return;

    dx = old_center_m[0] - g_bodies[new_parent_idx].pos[0];
    dy = old_center_m[1] - g_bodies[new_parent_idx].pos[1];
    dz = old_center_m[2] - g_bodies[new_parent_idx].pos[2];
    dist_m = sqrt(dx*dx + dy*dy + dz*dz);

    d->transfer_offset[0] = (float)(dx * RS);
    d->transfer_offset[1] = (float)(dy * RS);
    d->transfer_offset[2] = (float)(dz * RS);
    memcpy(d->transfer_b1, old_b1, 3 * sizeof(float));
    memcpy(d->transfer_b2, old_b2, 3 * sizeof(float));
    memcpy(d->transfer_pole, old_pole, 3 * sizeof(float));

    duration = clampf(disc_mid_period_seconds(d) * 0.18f,
                      RING_TRANSFER_MIN_SECONDS,
                      RING_TRANSFER_MAX_SECONDS);
    if (dist_m > 1.0)
        duration = fmaxf(duration,
                         clampf((float)(dist_m / fmax(g_bodies[new_parent_idx].radius, 1.0)) * (float)(DAY * 1.35),
                                RING_TRANSFER_MIN_SECONDS,
                                RING_TRANSFER_MAX_SECONDS));

    d->transfer_age = 0.0f;
    d->transfer_duration = duration;
}

/*
 * disc_begin_motion_transfer - ease orbital speed after a parent mass retune.
 *
 * retune_disc_parent() must immediately rebuild n_arr[] for the new GM so the
 * ring remains physically consistent.  Visually, however, a sudden n jump reads
 * as a speed cut at merge completion.  Store only the old/new mean-motion ratio
 * and apply it as a cheap scalar in rings_tick() until the handoff finishes.
 */
static void disc_begin_motion_transfer(ParticleDisc *d,
                                       float old_mean_motion,
                                       float duration)
{
    float ratio;

    if (!d || old_mean_motion <= 1e-8f || d->mean_motion_mid <= 1e-8f ||
        duration <= 0.0f) {
        if (d) {
            d->motion_scale_start = 1.0f;
            d->motion_blend_age = 0.0f;
            d->motion_blend_duration = 0.0f;
        }
        return;
    }

    ratio = old_mean_motion / d->mean_motion_mid;
    d->motion_scale_start = clampf(ratio, 0.15f, 6.0f);
    d->motion_blend_age = 0.0f;
    d->motion_blend_duration = duration;
}

/*
 * disc_update_response — advance and decay visual response state.
 *
 * Called from rings_tick(), not render, so visual state stays tied to the
 * simulation clock.  Large dt values are capped to avoid harsh decay/phase
 * jumps when the user runs at very high simulation speed.
 */
static void disc_update_response(ParticleDisc *d, float dt)
{
    float period, puff_tau, decay_tau, relax, parent_radius_km;
    if (!d || dt <= 0.0f) return;
    if (dt > RESPONSE_VISUAL_DT_MAX) dt = RESPONSE_VISUAL_DT_MAX;

    period = disc_mid_period_seconds(d);
    puff_tau  = clampf(period * 0.06f, MORPH_PUFF_RELAX_MIN,  MORPH_PUFF_RELAX_MAX);
    decay_tau = clampf(period * 0.65f, MORPH_DECAY_MIN,       MORPH_DECAY_MAX);

    parent_radius_km = (float)(collision_visual_radius(d->parent_idx,
                                                       g_bodies[d->parent_idx].radius) * 0.001);
    if (parent_radius_km > 0.0f) {
        if (d->parent_radius_ref_km <= 0.0f) d->parent_radius_ref_km = parent_radius_km;
        if (parent_radius_km > d->parent_radius_ref_km * 1.002f)
            disc_drive_transfer_response(d, d->parent_radius_ref_km, parent_radius_km, 0.12f);
        {
            float parent_tau = clampf(period * 0.18f, PARENT_TRACK_MIN, PARENT_TRACK_MAX);
            d->parent_radius_ref_km += (parent_radius_km - d->parent_radius_ref_km)
                                     * (1.0f - expf(-dt / parent_tau));
        }
    }

    relax = 1.0f - expf(-dt / puff_tau);
    d->puff_cur += (d->puff_target - d->puff_cur) * relax;
    d->puff_target *= expf(-dt / decay_tau);

    d->shock_phase = wrap_angle_pi(d->shock_phase + d->shock_spin * dt);
    d->shock_width += dt * d->mean_motion_mid * 0.07f;
    if (d->shock_width > (float)PI * 0.95f) d->shock_width = (float)PI * 0.95f;
    d->shock_amp *= expf(-dt / decay_tau);
    d->shock_spin *= expf(-dt / decay_tau);
    d->contact_strength *= expf(-dt / decay_tau);
    d->tide_strength *= expf(-dt / clampf(period * 0.38f, MORPH_DECAY_MIN, MORPH_DECAY_MAX));
    d->body_strength *= expf(-dt / clampf(period * 0.30f, MORPH_DECAY_MIN, MORPH_DECAY_MAX));
    if (d->shock_amp < 0.001f) d->shock_amp = 0.0f;
    if (d->contact_strength < 0.001f) d->contact_strength = 0.0f;
    if (d->tide_strength < 0.001f) d->tide_strength = 0.0f;
    if (d->body_strength < 0.001f) d->body_strength = 0.0f;

    if (d->transfer_duration > 0.0f) {
        d->transfer_age += dt;
        if (d->transfer_age >= d->transfer_duration) {
            d->transfer_age = d->transfer_duration = 0.0f;
            d->transfer_offset[0] = d->transfer_offset[1] = d->transfer_offset[2] = 0.0f;
        }
    }
    if (d->motion_blend_duration > 0.0f) {
        d->motion_blend_age += dt;
        if (d->motion_blend_age >= d->motion_blend_duration) {
            d->motion_scale_start = 1.0f;
            d->motion_blend_age = 0.0f;
            d->motion_blend_duration = 0.0f;
        }
    }
}

/* ── § ORBITS — basis construction, retuning, baking, damage passes ─────── */

/* build_basis — ring-plane orthonormal frame from planet obliquity. */
static void build_basis(ParticleDisc *d, float obl_deg)
{
    float obl = obl_deg * (float)(PI / 180.0);
    d->b1[0] = 1.0f; d->b1[1] = 0.0f;        d->b1[2] = 0.0f;
    d->b2[0] = 0.0f; d->b2[1] =  sinf(obl);  d->b2[2] = -cosf(obl);
    d->pole[0] = 0.0f; d->pole[1] = cosf(obl); d->pole[2] = sinf(obl);
}

/*
 * retune_disc_parent — reattach a disc to a new parent body after absorption.
 *
 * Called by rings_on_body_absorbed() when the impactor carries a ring and the
 * target absorbs it.  Updates the basis vectors from the new parent's obliquity
 * and recomputes every particle's mean motion n = √(GM/a³) from the new GM.
 *
 * Must recompute n for all particles because n ∝ √(GM) — the new parent has
 * a different mass, so all orbital periods change.
 */
static void retune_disc_parent(ParticleDisc *d, int parent_idx)
{
    if (!d || parent_idx < 0 || parent_idx >= g_nbodies) return;
    d->parent_idx = parent_idx;
    build_basis(d, (float)g_bodies[parent_idx].obliquity);
    disc_update_mid_motion(d);

    double gm = G_CONST * g_bodies[parent_idx].mass;
    if (gm <= 0.0) {
        d->initialized = 0;
        return;
    }
    for (int i = 0; i < d->n_full; i++) {
        if (d->data_full[i*8+1] <= 0.0f) {
            d->n_arr_full[i] = 0.0f;
            continue;
        }
        double a_m = (double)d->data_full[i*8+1] * AU;
        d->n_arr_full[i] = (float)sqrt(gm / (a_m * a_m * a_m));
    }
    for (int i = 0; i < d->n_lod; i++) {
        if (d->data_lod[i*8+1] <= 0.0f) {
            d->n_arr_lod[i] = 0.0f;
            continue;
        }
        double a_m = (double)d->data_lod[i*8+1] * AU;
        d->n_arr_lod[i] = (float)sqrt(gm / (a_m * a_m * a_m));
    }
    d->initialized = 1;
}

/*
 * bake_particles — fill a particle data array by sampling orbits within zones.
 *
 * For each zone, particles are distributed by `density` fraction.  The last
 * zone absorbs rounding remainder so all n particles are placed.
 *
 * Per particle:
 *   a_au = √(r²_min_au + u × (r²_max_au − r²_min_au))  [area-uniform in AU]
 *   M0   = uniform in [0, 2π]
 *   e    = uniform in [0, e_max]
 *   omega= uniform in [0, 2π]
 *   h    = uniform in [−h_scale/2, h_scale/2]  (disc thickness in AU)
 *   color= zone base color × brightness jitter [0.85, 1.0]
 *   n    = √(GM / a³)  pre-computed in rad/s
 */
static void bake_particles(float *data, float *n_arr, int n,
                            const Zone *zones, int n_zones,
                            double gm, float e_max, float h_scale)
{
    int idx = 0;
    for (int z = 0; z < n_zones; z++) {
        int cnt = (z == n_zones - 1)
                  ? (n - idx)
                  : (int)(zones[z].density * n);

        float r_min_au = zones[z].r_min / 1.496e8f;   /* km → AU */
        float r_max_au = zones[z].r_max / 1.496e8f;
        float r2_min = r_min_au * r_min_au;
        float r2_max = r_max_au * r_max_au;

        for (int k = 0; k < cnt && idx < n; k++, idx++) {
            float a_au = sqrtf(r2_min + s_randf() * (r2_max - r2_min));
            float M0   = s_randf() * 2.0f * (float)PI;
            float e    = s_randf() * e_max;
            float omega= s_randf() * 2.0f * (float)PI;
            float h    = (s_randf() - 0.5f) * h_scale;

            data[idx*8+0] = M0;
            data[idx*8+1] = a_au;
            data[idx*8+2] = e;
            data[idx*8+3] = omega;
            data[idx*8+4] = h;

            float j = 0.85f + 0.15f * s_randf();   /* brightness jitter */
            data[idx*8+5] = zones[z].r * j;
            data[idx*8+6] = zones[z].g * j;
            data[idx*8+7] = zones[z].b * j;

            double a_m = (double)a_au * 1.496e11;   /* AU → metres */
            n_arr[idx] = (float)sqrt(gm / (a_m * a_m * a_m));
        }
    }
}

/*
 * shuffle_particles - decorrelate draw order for distance-based thinning.
 *
 * The baker writes zones contiguously.  Since distant LOD renders only the
 * first N vertices from the LOD VBO, shuffle once after baking so every partial
 * draw remains a representative sample of all ring bands.
 */
static void shuffle_particles(float *data, float *n_arr, int n)
{
    if (!data || !n_arr || n <= 1) return;
    for (int i = n - 1; i > 0; i--) {
        int j = (int)(s_randf() * (float)(i + 1));
        float tmp_particle[8];
        float tmp_n;
        if (j < 0) j = 0;
        if (j > i) j = i;
        if (j == i) continue;

        memcpy(tmp_particle, data + i * 8, sizeof(tmp_particle));
        memcpy(data + i * 8, data + j * 8, sizeof(tmp_particle));
        memcpy(data + j * 8, tmp_particle, sizeof(tmp_particle));

        tmp_n = n_arr[i];
        n_arr[i] = n_arr[j];
        n_arr[j] = tmp_n;
    }
}

/*
 * damage_phase_from_world_dir — convert world impact direction to ring phase.
 *
 * If relative velocity has a useful in-plane component, prefer it so grazing
 * hits smear along the travel direction.  Otherwise fall back to the collision
 * normal supplied by collision.c.
 */
static float damage_phase_from_world_dir(const ParticleDisc *d,
                                         const double dir[3],
                                         const double rel_vel[3])
{
    float proj[3];
    float pole_dot;

    if (!d || !dir) return 0.0f;
    proj[0] = (float)dir[0];
    proj[1] = (float)dir[1];
    proj[2] = (float)dir[2];
    pole_dot = dot3f_local(proj, d->pole);
    proj[0] -= pole_dot * d->pole[0];
    proj[1] -= pole_dot * d->pole[1];
    proj[2] -= pole_dot * d->pole[2];

    if (dot3f_local(proj, proj) <= 1e-8f && rel_vel) {
        proj[0] = (float)rel_vel[0];
        proj[1] = (float)rel_vel[1];
        proj[2] = (float)rel_vel[2];
        pole_dot = dot3f_local(proj, d->pole);
        proj[0] -= pole_dot * d->pole[0];
        proj[1] -= pole_dot * d->pole[1];
        proj[2] -= pole_dot * d->pole[2];
    }

    normalize3f_local(proj);
    return atan2f(dot3f_local(proj, d->b2), dot3f_local(proj, d->b1));
}

static uint32_t damage_seed_for_disc(const ParticleDisc *d,
                                     int body_idx,
                                     double rel_speed,
                                     float phase)
{
    uint32_t seed = 0x9e3779b9u;
    uint32_t speed_bits = (uint32_t)(rel_speed * 0.25 + 0.5);
    uint32_t phase_bits = (uint32_t)((phase + (float)PI) * 1000.0f);

    seed ^= (uint32_t)(body_idx + 1) * 0x85ebca6bu;
    seed ^= speed_bits * 0xc2b2ae35u;
    seed ^= phase_bits * 0x27d4eb2du;
    if (d) {
        seed ^= (uint32_t)(d->ring_r_inner_km * 7.0f + d->ring_r_outer_km * 3.0f);
        seed ^= (uint32_t)(d->parent_idx + 17) * 0x165667b1u;
    }
    return seed ? seed : 1u;
}

/*
 * apply_damage_to_array — local one-shot orbit shear for a struck ring band.
 *
 * The contact grid decides when to call this.  We then perturb only the
 * particles near that azimuth and let Keplerian drift stretch the scar into
 * arcs, which keeps runtime cost low and avoids per-frame particle collisions.
 */
static void apply_damage_to_array(const ParticleDisc *d,
                                  float *data, float *n_arr, int n,
                                  double gm, float phase, float half_width,
                                  float severity, float tangential_sign,
                                  float vertical_sign, uint32_t seed)
{
    float ring_width_km;
    float width_au;
    float puff_scale;

    if (!d || !data || !n_arr || n <= 0 || gm <= 0.0) return;

    ring_width_km = fmaxf(d->ring_r_outer_km - d->ring_r_inner_km, 1.0f);
    width_au = ring_width_km / 1.496e8f;
    puff_scale = fmaxf(d->base_h_scale * (12.0f + 28.0f * severity),
                       width_au * (0.0012f + 0.0025f * severity));
    s_seed(seed);

    for (int i = 0; i < n; i++) {
        float *p = data + i * 8;
        float a0 = p[1];
        float theta = p[0] + p[3];
        float dphi = wrap_angle_pi(theta - phase);
        float ang_w = angular_falloff(dphi, half_width);
        float r_km, r_norm, radial_w, w, jitter, a_scale;

        if (a0 <= 0.0f || ang_w <= 0.0f) continue;

        r_km = p[1] * 1.496e8f;
        r_norm = clampf((r_km - d->ring_r_inner_km) / ring_width_km, 0.0f, 1.0f);
        radial_w = 0.55f + 0.45f * (1.0f - fabsf(r_norm - 0.56f) * 1.55f);
        radial_w = clampf(radial_w, 0.22f, 1.0f);
        w = severity * ang_w * radial_w;
        if (w <= 0.0f) continue;

        jitter = s_randf() - 0.5f;
        a_scale = 1.0f + tangential_sign * (0.030f + 0.028f * r_norm) * w
                        + 0.012f * jitter * w;
        a_scale = clampf(a_scale, DAMAGE_MIN_A_SCALE, DAMAGE_MAX_A_SCALE);
        p[1] = clampf(p[1] * a_scale, a0 * DAMAGE_MIN_A_SCALE, a0 * DAMAGE_MAX_A_SCALE);

        p[2] = fminf(DAMAGE_MAX_ECC,
                     p[2] + (0.010f + 0.040f * severity) * w
                          + fabsf(jitter) * 0.008f * w);
        p[3] += tangential_sign * (0.050f + 0.115f * severity) * w
              + jitter * 0.055f * w;
        p[4] += (vertical_sign * 0.22f + jitter * 0.28f) * puff_scale * w;

        {
            double a_m = (double)p[1] * AU;
            n_arr[i] = (float)sqrt(gm / (a_m * a_m * a_m));
        }
    }
}

static void apply_disc_damage(ParticleDisc *d, int body_idx, double rel_speed,
                              float phase, float half_width,
                              float severity, const double rel_vel[3])
{
    float tangent_dir[3];
    float tangential = 0.0f;
    float vertical = 0.0f;
    float tangential_sign;
    float vertical_sign;
    double gm;
    uint32_t seed;

    if (!d || !d->initialized || d->parent_idx != body_idx) return;
    if (body_idx < 0 || body_idx >= g_nbodies || !g_bodies[body_idx].alive) return;

    gm = G_CONST * g_bodies[body_idx].mass;
    if (gm <= 0.0) return;

    tangent_dir[0] = -sinf(phase) * d->b1[0] + cosf(phase) * d->b2[0];
    tangent_dir[1] = -sinf(phase) * d->b1[1] + cosf(phase) * d->b2[1];
    tangent_dir[2] = -sinf(phase) * d->b1[2] + cosf(phase) * d->b2[2];
    if (rel_vel) {
        tangential = (float)(rel_vel[0] * tangent_dir[0] +
                             rel_vel[1] * tangent_dir[1] +
                             rel_vel[2] * tangent_dir[2]);
        vertical = (float)(rel_vel[0] * d->pole[0] +
                           rel_vel[1] * d->pole[1] +
                           rel_vel[2] * d->pole[2]);
    }
    tangential_sign = tangential >= 0.0f ? 1.0f : -1.0f;
    vertical_sign = vertical >= 0.0f ? 1.0f : -1.0f;
    seed = damage_seed_for_disc(d, body_idx, rel_speed, phase);

    apply_damage_to_array(d, d->data_full, d->n_arr_full, d->n_full, gm,
                          phase, half_width, severity, tangential_sign,
                          vertical_sign, seed);
    apply_damage_to_array(d, d->data_lod, d->n_arr_lod, d->n_lod, gm,
                          phase, half_width, severity, tangential_sign,
                          vertical_sign, seed ^ 0x7f4a7c15u);
}

/* ── § CONTACT — swept hitboxes, despawn, tidal perturbation ────────────── */

/*
 * despawn_contact_particles_array - remove particles swallowed by a planet.
 *
 * A negative semi-major axis is used as an immutable tombstone.  The test is
 * intentionally conservative: only particles inside the current visual sphere
 * are removed, so swept look-ahead samples cannot erase rings before impact.
 */
static int despawn_contact_particles_array(const ParticleDisc *d,
                                           float *data, float *n_arr, int n,
                                           double ru, double rv, double rh,
                                           double other_r_km,
                                           float severity)
{
    double cu = ru * RS;
    double cv = rv * RS;
    double cn = rh * RS;
    double radius_au = other_r_km / 1.496e8;
    double core_r;
    int killed = 0;

    if (!d || !data || !n_arr || n <= 0 || radius_au <= 0.0) return 0;

    core_r = radius_au * (0.985 + 0.010 * clampf(severity, 0.0f, 1.0f));

    for (int i = 0; i < n; i++) {
        float *p = data + i * 8;
        float a = p[1];
        float sinM, cosM, nu, r, phi;
        double du, dv, dn, dist;
        int remove_particle = 0;

        if (a <= 0.0f) continue;

        sinM = sinf(p[0]);
        cosM = cosf(p[0]);
        nu = p[0] + 2.0f * p[2] * sinM;
        r = a * (1.0f - p[2] * cosM);
        phi = nu + p[3];

        du = (double)(r * cosf(phi)) - cu;
        dv = (double)(r * sinf(phi)) - cv;
        dn = (double)p[4] - cn;
        dist = sqrt(du*du + dv*dv + dn*dn);

        if (dist <= core_r) remove_particle = 1;

        if (!remove_particle) continue;

        p[1] = -fabsf(a);
        p[2] = 0.0f;
        p[4] = 0.0f;
        p[5] = 0.0f;
        p[6] = 0.0f;
        p[7] = 0.0f;
        n_arr[i] = 0.0f;
        killed++;
    }
    return killed;
}

static int despawn_disc_contact_particles(ParticleDisc *d,
                                          int other_idx, double rel_speed,
                                          double ru, double rv, double rh,
                                          double other_r_km, float severity)
{
    int killed = 0;

    if (!d || !d->initialized || other_idx < 0 || other_idx >= g_nbodies) return 0;
    (void)other_idx;
    (void)rel_speed;

    killed += despawn_contact_particles_array(d, d->data_full, d->n_arr_full, d->n_full,
                                              ru, rv, rh, other_r_km, severity);
    killed += despawn_contact_particles_array(d, d->data_lod, d->n_arr_lod, d->n_lod,
                                              ru, rv, rh, other_r_km, severity);
    return killed;
}

/*
 * apply_tidal_gravity_to_array - cheap 3D perturbation from a nearby body.
 *
 * This applies differential acceleration: the pull on each ring particle minus
 * the pull on the parent planet.  The result is projected into radial,
 * tangential, and normal components and folded back into the compact Keplerian
 * particle state.  It is only called for nearby bodies, so the O(N) pass stays
 * out of the normal frame path.
 */
static void apply_tidal_gravity_to_array(ParticleDisc *d,
                                         float *data, float *n_arr, int n,
                                         int other_idx, double dt,
                                         const double rel[3],
                                         double influence_m)
{
    const Body *other;
    double gm_parent;
    double gm_other;
    double rel_len2;
    double rel_inv3;
    double parent_acc[3];
    double dt_eff;
    double ring_width_au;

    if (!d || !data || !n_arr || n <= 0 || other_idx < 0 || other_idx >= g_nbodies) return;
    if (d->parent_idx < 0 || d->parent_idx >= g_nbodies) return;
    other = &g_bodies[other_idx];
    if (!other->alive || other->mass <= 0.0) return;

    gm_parent = G_CONST * g_bodies[d->parent_idx].mass;
    gm_other = G_CONST * other->mass;
    if (gm_parent <= 0.0 || gm_other <= 0.0 || influence_m <= 1.0) return;

    rel_len2 = rel[0]*rel[0] + rel[1]*rel[1] + rel[2]*rel[2]
             + fmax(other->radius * other->radius * 0.04, SOFTENING * SOFTENING);
    rel_inv3 = 1.0 / (sqrt(rel_len2) * rel_len2);
    parent_acc[0] = gm_other * rel[0] * rel_inv3;
    parent_acc[1] = gm_other * rel[1] * rel_inv3;
    parent_acc[2] = gm_other * rel[2] * rel_inv3;

    dt_eff = dt > TIDAL_MAX_DT ? TIDAL_MAX_DT : dt;
    if (dt_eff <= 0.0) return;
    ring_width_au = fmax((double)(d->ring_r_outer_km - d->ring_r_inner_km) / 1.496e8, 1e-9);

    for (int i = 0; i < n; i++) {
        float *p = data + i * 8;
        double theta = (double)p[0] + (double)p[3];
        double c = cos(theta);
        double s = sin(theta);
        double a_m;
        if (p[1] <= 0.0f) continue;
        a_m = (double)p[1] * AU;
        double h_m = (double)p[4] * AU;
        double radial[3] = {
            c * d->b1[0] + s * d->b2[0],
            c * d->b1[1] + s * d->b2[1],
            c * d->b1[2] + s * d->b2[2]
        };
        double tangential[3] = {
            -s * d->b1[0] + c * d->b2[0],
            -s * d->b1[1] + c * d->b2[1],
            -s * d->b1[2] + c * d->b2[2]
        };
        double particle_pos[3] = {
            a_m * radial[0] + h_m * d->pole[0],
            a_m * radial[1] + h_m * d->pole[1],
            a_m * radial[2] + h_m * d->pole[2]
        };
        double q[3] = {
            rel[0] - particle_pos[0],
            rel[1] - particle_pos[1],
            rel[2] - particle_pos[2]
        };
        double q_len2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2]
                      + fmax(other->radius * other->radius * 0.04, SOFTENING * SOFTENING);
        double q_len = sqrt(q_len2);
        double weight = clampf((float)((influence_m - q_len) / fmax(influence_m, 1.0)), 0.0f, 1.0f);
        double q_inv3;
        double tidal[3];
        double ar, at, an;
        double v_orb;
        double parent_grav;
        double accel_cap;
        float da_frac;
        float de;
        float dh_au;

        if (weight <= 0.0) continue;
        weight = weight * weight * (3.0 - 2.0 * weight);
        q_inv3 = 1.0 / (q_len * q_len2);
        tidal[0] = (gm_other * q[0] * q_inv3 - parent_acc[0]) * weight * TIDAL_VISUAL_GAIN;
        tidal[1] = (gm_other * q[1] * q_inv3 - parent_acc[1]) * weight * TIDAL_VISUAL_GAIN;
        tidal[2] = (gm_other * q[2] * q_inv3 - parent_acc[2]) * weight * TIDAL_VISUAL_GAIN;

        parent_grav = gm_parent / fmax(a_m * a_m, 1.0);
        accel_cap = parent_grav * 0.060;
        for (int k = 0; k < 3; k++) {
            if (tidal[k] >  accel_cap) tidal[k] =  accel_cap;
            if (tidal[k] < -accel_cap) tidal[k] = -accel_cap;
        }

        ar = tidal[0]*radial[0] + tidal[1]*radial[1] + tidal[2]*radial[2];
        at = tidal[0]*tangential[0] + tidal[1]*tangential[1] + tidal[2]*tangential[2];
        an = tidal[0]*d->pole[0] + tidal[1]*d->pole[1] + tidal[2]*d->pole[2];
        v_orb = sqrt(gm_parent / fmax(a_m, 1.0));
        if (v_orb <= 1e-6) continue;

        da_frac = clampf((float)(2.0 * at * dt_eff / v_orb), -TIDAL_MAX_DA_FRAC, TIDAL_MAX_DA_FRAC);
        de = clampf((float)(fabs(ar) * dt_eff / v_orb * 0.055), 0.0f, TIDAL_MAX_DE);
        dh_au = clampf((float)(an * dt_eff / v_orb * p[1] * 0.070),
                       (float)(-ring_width_au * 0.004),
                       (float)( ring_width_au * 0.004));

        {
            float old_a = p[1];
            p[1] = clampf(old_a * (1.0f + da_frac),
                          old_a * 0.9985f,
                          old_a * 1.0015f);
        }
        p[2] = fminf(DAMAGE_MAX_ECC, p[2] + de);
        p[3] += clampf((float)(ar * dt_eff / v_orb * 0.010), -0.0025f, 0.0025f);
        p[4] += dh_au;

        {
            double new_a_m = (double)p[1] * AU;
            n_arr[i] = (float)sqrt(gm_parent / (new_a_m * new_a_m * new_a_m));
        }
    }
}

static void apply_disc_tidal_gravity(ParticleDisc *d, int other_idx, double dt,
                                     const double rel[3], double influence_m)
{
    if (!d || !d->initialized) return;
    apply_tidal_gravity_to_array(d, d->data_full, d->n_arr_full, d->n_full,
                                 other_idx, dt, rel, influence_m);
    apply_tidal_gravity_to_array(d, d->data_lod, d->n_arr_lod, d->n_lod,
                                 other_idx, dt, rel, influence_m);
}

static void damage_disc(ParticleDisc *d, int body_idx, double rel_speed,
                        const double dir[3], const double rel_vel[3])
{
    float phase;
    float severity;
    float visual_severity;
    float half_width;
    float overlap;

    if (!d || !d->initialized || d->parent_idx != body_idx) return;
    if (body_idx < 0 || body_idx >= g_nbodies || !g_bodies[body_idx].alive) return;

    phase = damage_phase_from_world_dir(d, dir, rel_vel);
    severity = clampf((float)(rel_speed / 15000.0), 0.18f, 1.0f);
    visual_severity = clampf(severity * 0.55f, 0.10f, 0.72f);
    half_width = clampf(0.28f + 0.34f * visual_severity, DAMAGE_MIN_WIDTH, 0.82f);
    overlap = 0.62f;

    disc_drive_response(d, phase, half_width, visual_severity, overlap, 0.5f, 0.65f);
    apply_disc_damage(d, body_idx, rel_speed, phase, half_width,
                      0.18f + 0.26f * severity, rel_vel);
}

/*
 * disc_probe_contact_sample — evaluate one sphere/ring contact sample.
 *
 * The sample time is relative to the current frame endpoint.  Negative values
 * are swept-history probes used to catch fast crossings; t=0 is the current
 * body position.  Only current/near-current samples may despawn particles.
 */
static int disc_probe_contact_sample(ParticleDisc *d, int other_idx,
                                     double rel_speed, double sample_t)
{
    Body *parent;
    Body *other;
    double rel[3], rel_vel[3];
    double ru, rv, rh;
    double proj_r_km, plane_h_km, other_r_km, cross_r_km;
    double ring_width_km;
    float phase, half_width, inner_norm, outer_norm;
    float radial_overlap, plane_overlap, overlap, severity;
    float sector_cooldown;

    if (!d || !d->initialized || other_idx < 0 || other_idx >= g_nbodies) return 0;
    if (d->parent_idx < 0 || d->parent_idx >= g_nbodies) return 0;
    if (!body_is_ring_perturber_planet(other_idx)) return 0;

    parent = &g_bodies[d->parent_idx];
    other = &g_bodies[other_idx];
    if (!parent->alive || !other->alive || other->is_star) return 0;

    rel_vel[0] = other->vel[0] - parent->vel[0];
    rel_vel[1] = other->vel[1] - parent->vel[1];
    rel_vel[2] = other->vel[2] - parent->vel[2];
    rel[0] = (other->pos[0] - parent->pos[0]) + rel_vel[0] * sample_t;
    rel[1] = (other->pos[1] - parent->pos[1]) + rel_vel[1] * sample_t;
    rel[2] = (other->pos[2] - parent->pos[2]) + rel_vel[2] * sample_t;

    ru = rel[0] * d->b1[0] + rel[1] * d->b1[1] + rel[2] * d->b1[2];
    rv = rel[0] * d->b2[0] + rel[1] * d->b2[1] + rel[2] * d->b2[2];
    rh = rel[0] * d->pole[0] + rel[1] * d->pole[1] + rel[2] * d->pole[2];

    proj_r_km = sqrt(ru*ru + rv*rv) * 0.001;
    plane_h_km = fabs(rh) * 0.001;
    other_r_km = collision_visual_radius(other_idx, other->radius) * 0.001;
    if (other_r_km <= 1e-6) return 0;
    if (plane_h_km >= other_r_km) return 0;

    cross_r_km = sqrt(other_r_km*other_r_km - plane_h_km*plane_h_km);
    if (proj_r_km + cross_r_km < d->ring_r_inner_km) return 0;
    if (proj_r_km - cross_r_km > d->ring_r_outer_km) return 0;

    ring_width_km = fmax(d->ring_r_outer_km - d->ring_r_inner_km, 1.0);
    phase = atan2f((float)rv, (float)ru);
    if (proj_r_km <= cross_r_km + 1.0)
        half_width = (float)PI;
    else
        half_width = asinf(clampf((float)(cross_r_km / proj_r_km), 0.0f, 1.0f));

    inner_norm = clampf((float)((proj_r_km - cross_r_km - d->ring_r_inner_km) / ring_width_km),
                        0.0f, 1.0f);
    outer_norm = clampf((float)((proj_r_km + cross_r_km - d->ring_r_inner_km) / ring_width_km),
                        0.0f, 1.0f);

    sector_cooldown = HIT_SEGMENT_COOLDOWN_SECONDS;
    if (d->mean_motion_mid > 1e-8f) {
        float sector_time = ((2.0f * (float)PI) / (float)RING_COLLISION_SEGMENTS) / d->mean_motion_mid;
        sector_cooldown = clampf(sector_time * 0.28f, 60.0f * 8.0f, (float)(DAY * 0.20));
    }

    {
        const float TWO_PI = 6.28318530718f;
        const float SEG_W = TWO_PI / (float)RING_COLLISION_SEGMENTS;
        const float RADIAL_BIN_W = 1.0f / (float)RING_COLLISION_RADIAL_BINS;
        int radial_first = (int)floorf(inner_norm / RADIAL_BIN_W);
        int radial_last  = (int)floorf((outer_norm - 1e-4f) / RADIAL_BIN_W);
        if (radial_first < 0) radial_first = 0;
        if (radial_last >= RING_COLLISION_RADIAL_BINS) radial_last = RING_COLLISION_RADIAL_BINS - 1;
        if (radial_last < radial_first) radial_last = radial_first;

        for (int i = 0; i < RING_COLLISION_SEGMENTS; i++) {
            float seg_phase = -((float)PI) + ((float)i + 0.5f) * SEG_W;
            float dphi = wrap_angle_pi(seg_phase - phase);
            if (angular_falloff(dphi, half_width + SEG_W * 0.65f) <= 0.0f)
                continue;

            for (int rbin = radial_first; rbin <= radial_last; rbin++) {
                int cell = i * RING_COLLISION_RADIAL_BINS + rbin;
                d->hit_cooldown[cell] = sector_cooldown;
            }
        }
    }

    radial_overlap = clampf((float)((fmin(proj_r_km + cross_r_km, d->ring_r_outer_km) -
                                     fmax(proj_r_km - cross_r_km, d->ring_r_inner_km))
                                    / ring_width_km),
                            0.0f, 1.0f);
    plane_overlap = clampf(1.0f - (float)(plane_h_km / other_r_km), 0.0f, 1.0f);
    overlap = clampf(0.25f + 0.75f * plane_overlap * (0.35f + 0.65f * radial_overlap),
                     0.0f, 1.0f);
    {
        float speed_severity = clampf((float)(rel_speed / 18000.0), 0.08f, 0.85f);
        float size_severity = clampf((float)(cross_r_km / fmax(ring_width_km * 0.20, 1.0)), 0.0f, 0.85f);
        severity = clampf(speed_severity * 0.55f + size_severity * 0.65f, 0.10f, 0.95f);
    }

    {
        float radial_center = clampf((float)((proj_r_km - d->ring_r_inner_km) / ring_width_km),
                                     0.0f, 1.0f);
        float radial_width = clampf((float)(cross_r_km / ring_width_km) * 1.35f,
                                    0.08f, 0.85f);
        int killed = 0;
        /*
         * Do not erase particles on the mathematical first touch.  Waiting for
         * a small real penetration keeps grazing contacts from reading as a
         * one-frame pop while still removing particles once the sphere is
         * visibly inside the ring volume.
         */
        if (sample_t >= -1e-6 && plane_overlap > 0.08f && radial_overlap > 0.015f)
            killed = despawn_disc_contact_particles(d, other_idx, rel_speed,
                                                    ru, rv, rh, other_r_km,
                                                    severity);
        if (killed > 0) {
            severity = clampf(severity + 0.08f, 0.0f, 1.0f);
            overlap = clampf(overlap + 0.10f, 0.0f, 1.0f);
        }
        disc_drive_response(d,
                            phase,
                            fminf(half_width + (2.0f * (float)PI / (float)RING_COLLISION_SEGMENTS) * 0.75f,
                                  (float)PI * 0.92f),
                            severity,
                            overlap,
                            radial_center,
                            radial_width);
    }
    return 1;
}

static void add_contact_time(double *times, int *count, double t, double dt)
{
    if (!times || !count) return;
    if (t < -dt) t = -dt;
    if (t > 0.0) t = 0.0;
    for (int i = 0; i < *count; i++) {
        if (fabs(times[i] - t) < dt * 0.015 + 1e-6) return;
    }
    if (*count < 16) {
        times[*count] = t;
        (*count)++;
    }
}

static void add_radial_crossing_times(double *times, int *count, double dt,
                                      double ru, double rv,
                                      double vu, double vv,
                                      double radius_m)
{
    double a = vu*vu + vv*vv;
    double b = 2.0 * (ru*vu + rv*vv);
    double c = ru*ru + rv*rv - radius_m*radius_m;
    double disc;

    if (radius_m <= 0.0 || a <= 1e-12) return;
    disc = b*b - 4.0*a*c;
    if (disc < 0.0) return;
    disc = sqrt(disc);
    add_contact_time(times, count, (-b - disc) / (2.0*a), dt);
    add_contact_time(times, count, (-b + disc) / (2.0*a), dt);
}

/*
 * update_disc_swept_contact — cheap broadphase + swept samples for one body.
 *
 * This is the ring equivalent of collision.c's swept-sphere thinking, but it
 * runs only at the outer system step and only for planet-scale perturbers.
 * Visual tide can start before physical contact; despawn requires confirmed
 * overlap in disc_probe_contact_sample().
 */
static void update_disc_swept_contact(ParticleDisc *d, int other_idx, double dt)
{
    Body *parent;
    Body *other;
    double rel[3], rel_vel[3];
    double ru, rv, rh, vu, vv, vh;
    double proj_r_km, h_km, other_r_km, v_mps, speed, reach_km;
    double ring_width_km, tidal_reach_km, motion_km;
    double times[16];
    int time_count = 0;

    if (!d || !d->initialized || dt <= 0.0) return;
    if (d->parent_idx < 0 || d->parent_idx >= g_nbodies) return;
    if (other_idx < 0 || other_idx >= g_nbodies || other_idx == d->parent_idx) return;
    if (!body_is_ring_perturber_planet(other_idx)) return;

    parent = &g_bodies[d->parent_idx];
    other = &g_bodies[other_idx];
    if (!parent->alive || !other->alive || other->is_star) return;

    rel[0] = other->pos[0] - parent->pos[0];
    rel[1] = other->pos[1] - parent->pos[1];
    rel[2] = other->pos[2] - parent->pos[2];
    rel_vel[0] = other->vel[0] - parent->vel[0];
    rel_vel[1] = other->vel[1] - parent->vel[1];
    rel_vel[2] = other->vel[2] - parent->vel[2];
    speed = sqrt(rel_vel[0]*rel_vel[0] + rel_vel[1]*rel_vel[1] + rel_vel[2]*rel_vel[2]);

    ru = rel[0] * d->b1[0] + rel[1] * d->b1[1] + rel[2] * d->b1[2];
    rv = rel[0] * d->b2[0] + rel[1] * d->b2[1] + rel[2] * d->b2[2];
    rh = rel[0] * d->pole[0] + rel[1] * d->pole[1] + rel[2] * d->pole[2];
    vu = rel_vel[0] * d->b1[0] + rel_vel[1] * d->b1[1] + rel_vel[2] * d->b1[2];
    vv = rel_vel[0] * d->b2[0] + rel_vel[1] * d->b2[1] + rel_vel[2] * d->b2[2];
    vh = rel_vel[0] * d->pole[0] + rel_vel[1] * d->pole[1] + rel_vel[2] * d->pole[2];
    proj_r_km = sqrt(ru*ru + rv*rv) * 0.001;
    h_km = fabs(rh) * 0.001;
    other_r_km = collision_visual_radius(other_idx, other->radius) * 0.001;
    v_mps = speed;
    motion_km = v_mps * dt * 0.001;
    reach_km = other_r_km + motion_km;
    ring_width_km = fmax(d->ring_r_outer_km - d->ring_r_inner_km, 1.0);
    tidal_reach_km = fmax(other_r_km * 14.0, ring_width_km * 3.10) + motion_km;

    if (h_km > tidal_reach_km) return;
    if (proj_r_km + tidal_reach_km < d->ring_r_inner_km) return;
    if (proj_r_km - tidal_reach_km > d->ring_r_outer_km) return;

    disc_drive_tide_response(d, ru, rv, rh, proj_r_km, h_km,
                             other_r_km, tidal_reach_km);
    apply_disc_tidal_gravity(d, other_idx, dt, rel, tidal_reach_km * 1000.0);

    if (h_km > reach_km) return;
    if (proj_r_km + reach_km < d->ring_r_inner_km) return;
    if (proj_r_km - reach_km > d->ring_r_outer_km) return;

    add_contact_time(times, &time_count, 0.0, dt);
    add_contact_time(times, &time_count, -dt, dt);

    if (fabs(vh) > 1e-9) {
        double r_m = other_r_km * 1000.0;
        add_contact_time(times, &time_count, -rh / vh, dt);
        add_contact_time(times, &time_count, ( r_m - rh) / vh, dt);
        add_contact_time(times, &time_count, (-r_m - rh) / vh, dt);
    }

    {
        double radial_v2 = vu*vu + vv*vv;
        if (radial_v2 > 1e-12)
            add_contact_time(times, &time_count, -(ru*vu + rv*vv) / radial_v2, dt);
    }

    {
        double inner_m = d->ring_r_inner_km * 1000.0;
        double outer_m = d->ring_r_outer_km * 1000.0;
        double body_m = other_r_km * 1000.0;
        add_radial_crossing_times(times, &time_count, dt, ru, rv, vu, vv, inner_m - body_m);
        add_radial_crossing_times(times, &time_count, dt, ru, rv, vu, vv, inner_m + body_m);
        add_radial_crossing_times(times, &time_count, dt, ru, rv, vu, vv, outer_m - body_m);
        add_radial_crossing_times(times, &time_count, dt, ru, rv, vu, vv, outer_m + body_m);
    }

    if (time_count < RING_SWEEP_MAX_SAMPLES) {
        int needed = RING_SWEEP_MAX_SAMPLES - time_count;
        for (int s = 1; s <= needed; s++) {
            double t = -dt + dt * ((double)s / (double)(needed + 1));
            add_contact_time(times, &time_count, t, dt);
        }
    }

    for (int s = 0; s < time_count; s++) {
        disc_probe_contact_sample(d, other_idx, speed, times[s]);
    }
}

/* ── § GL — VAO/VBO setup, distance LOD, render path ───────────────────── */

/* Set up the 8-float-per-particle VAO attribute pointers (loc 0..5).
 * Locations 0..4 are individual floats; loc 5 is vec3 (rgb). */
static void setup_particle_attribs(void) {
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(0*sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(1*sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(4*sizeof(float)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(5*sizeof(float)));
}

/* Compile shaders and upload initial particle data for one disc. */
static void init_disc_gl(ParticleDisc *d)
{
    /* ring.vert evaluates the compact Keplerian particle state on the GPU. */
    d->shader = gl_shader_load("assets/shaders/ring.vert",
                               "assets/shaders/color.frag");
    if (!d->shader) return;
    d->loc_vp     = glGetUniformLocation(d->shader, "u_vp");
    d->loc_center = glGetUniformLocation(d->shader, "u_center");
    d->loc_b1     = glGetUniformLocation(d->shader, "u_b1");
    d->loc_b2     = glGetUniformLocation(d->shader, "u_b2");
    d->loc_pole   = glGetUniformLocation(d->shader, "u_pole");
    d->loc_morph0 = glGetUniformLocation(d->shader, "u_morph0");
    d->loc_morph1 = glGetUniformLocation(d->shader, "u_morph1");
    d->loc_morph2 = glGetUniformLocation(d->shader, "u_morph2");
    d->loc_tide0  = glGetUniformLocation(d->shader, "u_tide0");
    d->loc_tide1  = glGetUniformLocation(d->shader, "u_tide1");
    d->loc_body0  = glGetUniformLocation(d->shader, "u_body0");
    d->loc_body1  = glGetUniformLocation(d->shader, "u_body1");
    d->loc_light0 = glGetUniformLocation(d->shader, "u_light0");
    d->loc_shadow_strength = glGetUniformLocation(d->shader, "u_shadow_strength");

    /* Full-count VAO/VBO */
    d->vao_full = gl_vao_create();
    d->vbo_full = gl_vbo_create(d->n_full * 8 * sizeof(float),
                                d->data_full, GL_DYNAMIC_DRAW);
    setup_particle_attribs();
    glBindVertexArray(0);

    /* LOD VAO/VBO */
    d->vao_lod = gl_vao_create();
    d->vbo_lod = gl_vbo_create(d->n_lod * 8 * sizeof(float),
                               d->data_lod, GL_DYNAMIC_DRAW);
    setup_particle_attribs();
    glBindVertexArray(0);

}

/*
 * render_disc — render one ring disc with distance-thinned particles.
 *
 * The sprite path is intentionally gone: all visible rings are drawn through
 * ring.vert so close and distant collision warps stay visually consistent.
 * Past LOD_DIST we switch to the shuffled LOD buffer and draw a progressively
 * smaller prefix.  Past RING_FADE_START, alpha fades to zero before the ring
 * is skipped entirely.
 */
static void render_disc(const ParticleDisc *d, const float vp_camrel[16])
{
    Body *par = &g_bodies[d->parent_idx];
    float rb1[3], rb2[3], rpole[3];
    float blend = 1.0f;
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    float px, py, pz;
    double cx, cy, cz;
    rb1[0] = d->b1[0]; rb1[1] = d->b1[1]; rb1[2] = d->b1[2];
    rb2[0] = d->b2[0]; rb2[1] = d->b2[1]; rb2[2] = d->b2[2];
    rpole[0] = d->pole[0]; rpole[1] = d->pole[1]; rpole[2] = d->pole[2];
    if (d->transfer_duration > 0.0f) {
        blend = smootherstep01f(d->transfer_age / d->transfer_duration);
        ox = d->transfer_offset[0] * (1.0f - blend);
        oy = d->transfer_offset[1] * (1.0f - blend);
        oz = d->transfer_offset[2] * (1.0f - blend);
        mix3f_local(d->transfer_b1, d->b1, blend, rb1);
        mix3f_local(d->transfer_b2, d->b2, blend, rb2);
        mix3f_local(d->transfer_pole, d->pole, blend, rpole);
        normalize3f_local(rb1);
        normalize3f_local(rb2);
        normalize3f_local(rpole);
    }
    cx = par->pos[0] * RS + (double)ox;
    cy = par->pos[1] * RS + (double)oy;
    cz = par->pos[2] * RS + (double)oz;
    /* Camera-relative centre in double -> float to avoid float32 cancellation. */
    px = (float)(cx - g_cam.pos[0]);
    py = (float)(cy - g_cam.pos[1]);
    pz = (float)(cz - g_cam.pos[2]);
    float dx   = px;
    float dy   = py;
    float dz   = pz;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    float alpha = 1.0f;
    int n;
    float *data;
    GLuint vao;
    GLuint vbo;
    float sun_x = 0.0f, sun_y = 0.0f, sun_z = 0.0f;
    float ring_ambient = 0.35f;
    float shadow_strength = 0.0f;

    if (!d->shader) return;
    if (dist >= RING_FADE_END) return;

    if (dist > RING_FADE_START) {
        alpha = inv_smootherstep01f((dist - RING_FADE_START) /
                                    fmaxf(RING_FADE_END - RING_FADE_START, 1e-5f));
        if (alpha <= 0.001f) return;
    }

    if (dist <= RING_SHADOW_FULL_DIST) {
        shadow_strength = 1.0f;
    } else if (dist < RING_SHADOW_FADE_END) {
        shadow_strength = inv_smootherstep01f((dist - RING_SHADOW_FULL_DIST) /
                                              fmaxf(RING_SHADOW_FADE_END -
                                                    RING_SHADOW_FULL_DIST, 1e-5f));
    }

    if (dist <= LOD_DIST || d->n_lod <= 0) {
        n = d->n_full;
        data = d->data_full;
        vao = d->vao_full;
        vbo = d->vbo_full;
    } else {
        float lod_t = smootherstep01f((dist - LOD_DIST) /
                                      fmaxf(LOD_FADE_END - LOD_DIST, 1e-5f));
        float count_f = (float)d->n_full + ((float)d->n_lod - (float)d->n_full) * lod_t;
        int min_visible = d->n_lod < 24 ? d->n_lod : 24;

        n = (int)(count_f + 0.5f);
        if (n < min_visible) n = min_visible;
        if (n > d->n_full) n = d->n_full;
        if (n > d->n_lod) {
            data = d->data_full;
            vao = d->vao_full;
            vbo = d->vbo_full;
        } else {
            data = d->data_lod;
            vao = d->vao_lod;
            vbo = d->vbo_lod;
        }
    }
    if (n <= 0 || !data || !vao || !vbo) return;

    {
        int star_idx = body_root_star(d->parent_idx);
        if (star_idx >= 0 && star_idx < g_nbodies &&
            g_bodies[star_idx].alive && g_bodies[star_idx].is_star) {
            double sx = g_bodies[star_idx].pos[0] - par->pos[0];
            double sy = g_bodies[star_idx].pos[1] - par->pos[1];
            double sz = g_bodies[star_idx].pos[2] - par->pos[2];
            double sl = sqrt(sx*sx + sy*sy + sz*sz);
            if (sl > 1.0) {
                sun_x = (float)(sx / sl);
                sun_y = (float)(sy / sl);
                sun_z = (float)(sz / sl);
            } else {
                ring_ambient = 1.0f;
                shadow_strength = 0.0f;
            }
        } else {
            ring_ambient = 1.0f;
            shadow_strength = 0.0f;
        }
    }

    glUseProgram(d->shader);
    glUniformMatrix4fv(d->loc_vp,     1, GL_FALSE, vp_camrel);
    glUniform3f       (d->loc_center,  px, py, pz);
    glUniform3fv      (d->loc_b1,    1, rb1);
    glUniform3fv      (d->loc_b2,    1, rb2);
    glUniform3fv      (d->loc_pole,  1, rpole);
    glUniform4f       (d->loc_morph0,
                       alpha,
                       d->puff_cur,
                       d->shock_amp,
                       d->shock_phase);
    glUniform4f       (d->loc_morph1,
                       d->shock_width,
                       d->shock_spin,
                       d->ring_r_inner_km / 1.496e8f,
                       d->ring_r_outer_km / 1.496e8f);
    glUniform4f       (d->loc_morph2,
                       d->contact_norm,
                       d->contact_width,
                       d->contact_strength,
                       0.0f);
    glUniform4f       (d->loc_tide0,
                       d->tide_phase,
                       d->tide_radius_norm,
                       d->tide_width,
                       d->tide_strength);
    glUniform4f       (d->loc_tide1,
                       d->tide_dir_u,
                       d->tide_dir_v,
                       d->tide_dir_n,
                       0.0f);
    glUniform4f       (d->loc_body0,
                       d->body_u_au,
                       d->body_v_au,
                       d->body_n_au,
                       d->body_radius_au);
    glUniform4f       (d->loc_body1,
                       d->body_strength,
                       (float)(collision_visual_radius(d->parent_idx, par->radius) * RS),
                       0.0f,
                       0.0f);
    glUniform4f       (d->loc_light0,
                       sun_x,
                       sun_y,
                       sun_z,
                       ring_ambient);
    glUniform1f       (d->loc_shadow_strength, shadow_strength);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    /* Upload updated M0 values (and unchanged a,e,omega,h,rgb) each frame. */
    glBufferSubData(GL_ARRAY_BUFFER, 0, n * 8 * sizeof(float), data);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDrawArrays(GL_POINTS, 0, n);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

/* ── § API — public lifecycle, ticking, rendering, collision hooks ─────── */

/*
 * rings_init — parse "rings" array from universe.json and build all disc data.
 *
 * For each ring entry:
 *   1. Find the parent body by name.
 *   2. Parse zones (annulus bands with density, color, radii).
 *   3. Allocate particle arrays (full and LOD counts).
 *   4. Build ring-plane basis from parent obliquity.
 *   5. Bake particle orbits (bake_particles) for both full and LOD arrays.
 *   6. Upload to GPU (init_disc_gl).
 */
void rings_init(const char *path)
{
    JsonNode *root = json_parse_file(path);
    if (!root) {
        fprintf(stderr, "[Rings] cannot parse '%s'\n", path);
        return;
    }

    JsonNode *rings_arr = json_get(root, "rings");
    if (!rings_arr || rings_arr->type != JSON_ARRAY) {
        fprintf(stderr, "[Rings] no 'rings' array in '%s' — skipping\n", path);
        json_free(root);
        return;
    }

    int count = 0;
    { JsonNode *n = rings_arr->first_child; while (n) { count++; n = n->next; } }
    if (count == 0) { json_free(root); return; }

    s_discs   = (ParticleDisc*)calloc(count, sizeof(ParticleDisc));
    s_n_discs = 0;
    if (!s_discs) { json_free(root); return; }

    JsonNode *rnode = rings_arr->first_child;
    while (rnode) {
        const char *body_name   = json_str(json_get(rnode, "body"),        "");
        int    n_full      = (int)json_num(json_get(rnode, "n_full"),       1000);
        int    n_lod       = (int)json_num(json_get(rnode, "n_lod"),         200);
        uint32_t seed_full = (uint32_t)json_num(json_get(rnode, "seed_full"), 1234);
        uint32_t seed_lod  = (uint32_t)json_num(json_get(rnode, "seed_lod"),  5678);
        float  e_max       = (float)json_num(json_get(rnode, "e_max"),       0.01);
        float  h_scale     = (float)json_num(json_get(rnode, "h_scale"),     1e-6);

        int par_idx = -1;
        for (int i = 0; i < g_nbodies; i++) {
            if (strcmp(g_bodies[i].name, body_name) == 0) { par_idx = i; break; }
        }
        if (par_idx < 0) {
            fprintf(stderr, "[Rings] body '%s' not found — ring skipped\n", body_name);
            rnode = rnode->next;
            continue;
        }

        Zone zones[MAX_ZONES];
        int  n_zones = 0;
        JsonNode *zones_arr = json_get(rnode, "zones");
        if (zones_arr) {
            JsonNode *zn = zones_arr->first_child;
            while (zn && n_zones < MAX_ZONES) {
                zones[n_zones].r_min    = (float)json_num(json_get(zn, "r_min_km"), 0.0);
                zones[n_zones].r_max    = (float)json_num(json_get(zn, "r_max_km"), 1.0);
                zones[n_zones].density  = (float)json_num(json_get(zn, "density"),  1.0);
                JsonNode *col = json_get(zn, "color");
                zones[n_zones].r = (float)json_num(json_idx(col, 0), 0.7);
                zones[n_zones].g = (float)json_num(json_idx(col, 1), 0.7);
                zones[n_zones].b = (float)json_num(json_idx(col, 2), 0.7);
                n_zones++;
                zn = zn->next;
            }
        }
        if (n_zones == 0) { rnode = rnode->next; continue; }

        ParticleDisc *disc = &s_discs[s_n_discs++];
        disc->parent_idx         = par_idx;
        disc->n_full             = n_full;
        disc->n_lod              = n_lod;
        disc->base_h_scale       = h_scale;
        disc->ring_r_inner_km    = zones[0].r_min;
        disc->ring_r_outer_km    = zones[0].r_max;
        for (int zi = 1; zi < n_zones; zi++) {
            if (zones[zi].r_min < disc->ring_r_inner_km)
                disc->ring_r_inner_km = zones[zi].r_min;
            if (zones[zi].r_max > disc->ring_r_outer_km)
                disc->ring_r_outer_km = zones[zi].r_max;
        }
        disc->parent_radius_ref_km = (float)(g_bodies[par_idx].radius * 0.001);
        disc_reset_response(disc);

        disc->data_full  = (float*)malloc(n_full * 8 * sizeof(float));
        disc->data_lod   = (float*)malloc(n_lod  * 8 * sizeof(float));
        disc->n_arr_full = (float*)malloc(n_full * sizeof(float));
        disc->n_arr_lod  = (float*)malloc(n_lod  * sizeof(float));
        if (!disc->data_full || !disc->data_lod ||
            !disc->n_arr_full || !disc->n_arr_lod) {
            rnode = rnode->next; continue;
        }

        build_basis(disc, (float)g_bodies[par_idx].obliquity);
        disc_update_mid_motion(disc);

        double gm = G_CONST * g_bodies[par_idx].mass;
        s_seed(seed_full);
        bake_particles(disc->data_full, disc->n_arr_full, n_full,
                       zones, n_zones, gm, e_max, h_scale);
        shuffle_particles(disc->data_full, disc->n_arr_full, n_full);

        s_seed(seed_lod);
        bake_particles(disc->data_lod, disc->n_arr_lod, n_lod,
                       zones, n_zones, gm, e_max, h_scale);
        shuffle_particles(disc->data_lod, disc->n_arr_lod, n_lod);

        init_disc_gl(disc);
        disc->initialized = 1;

        rnode = rnode->next;
    }

    json_free(root);
}

/*
 * rings_step_system — swept, low-frequency ring hitbox pass for one system.
 *
 * Called once per outer physics step.  The detector sweeps the impactor over
 * the last dt instead of polling every inner RESPA tick, so ring contacts stay
 * responsive without turning the hot integrator loop into a ring broadphase.
 */
void rings_step_system(int root, double dt)
{
    if (root < 0 || root >= g_nbodies || s_n_discs <= 0 || dt <= 0.0) return;
    if (!g_bodies[root].alive) return;

    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        int parent_idx;

        if (!disc->initialized) continue;
        parent_idx = disc->parent_idx;
        if (parent_idx < 0 || parent_idx >= g_nbodies) continue;
        if (!g_bodies[parent_idx].alive) continue;
        if (body_root_star(parent_idx) != root) continue;

        for (int c = 0; c < RING_COLLISION_SEGMENTS * RING_COLLISION_RADIAL_BINS; c++) {
            if (disc->hit_cooldown[c] > 0.0f) {
                disc->hit_cooldown[c] -= (float)dt;
                if (disc->hit_cooldown[c] < 0.0f) disc->hit_cooldown[c] = 0.0f;
            }
        }

        for (int i = 0; i < g_nbodies; i++) {
            if (i == parent_idx) continue;
            if (!body_is_ring_perturber_planet(i)) continue;
            if (body_root_star(i) != root) continue;
            update_disc_swept_contact(disc, i, dt);
        }
    }
}

/*
 * rings_tick — advance mean anomaly for all ring particles: M += n × dt.
 *
 * M is kept in [0, 2π) by subtracting 2π × floor(M / 2π) rather than using
 * fmod, which can be slow for many particles.  The result is passed to the
 * GPU each frame via glBufferSubData in render_disc().
 */
void rings_tick(double dt)
{
    const float TWO_PI = 6.28318530718f;

    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        float motion_scale = 1.0f;
        if (!disc->initialized) continue;
        if (disc->parent_idx < 0 || disc->parent_idx >= g_nbodies) continue;
        if (!g_bodies[disc->parent_idx].alive) continue;
        disc_update_response(disc, dt > 0.0 ? (float)dt : 0.0f);

        if (disc->motion_blend_duration > 0.0f) {
            float t = smootherstep01f(disc->motion_blend_age /
                                      disc->motion_blend_duration);
            motion_scale = disc->motion_scale_start +
                           (1.0f - disc->motion_scale_start) * t;
        }

        for (int i = 0; i < disc->n_full; i++) {
            if (disc->data_full[i*8+1] <= 0.0f) continue;
            float M = disc->data_full[i*8+0]
                    + (float)((double)disc->n_arr_full[i] *
                              (double)motion_scale * dt);
            if (M >= TWO_PI) M -= TWO_PI * (float)(int)(M / TWO_PI);
            disc->data_full[i*8+0] = M;
        }
        for (int i = 0; i < disc->n_lod; i++) {
            if (disc->data_lod[i*8+1] <= 0.0f) continue;
            float M = disc->data_lod[i*8+0]
                    + (float)((double)disc->n_arr_lod[i] *
                              (double)motion_scale * dt);
            if (M >= TWO_PI) M -= TWO_PI * (float)(int)(M / TWO_PI);
            disc->data_lod[i*8+0] = M;
        }
    }
}

/* rings_render — draw all initialized, alive ring discs at current LOD. */
void rings_render(const float vp_camrel[16])
{
    for (int d = 0; d < s_n_discs; d++) {
        if (s_discs[d].initialized &&
            s_discs[d].parent_idx >= 0 &&
            s_discs[d].parent_idx < g_nbodies &&
            g_bodies[s_discs[d].parent_idx].alive)
            render_disc(&s_discs[d], vp_camrel);
    }
}

/*
 * rings_on_collision — strong one-shot ring response at body impact start.
 */
void rings_on_collision(int target_idx, int impactor_idx, double rel_speed,
                        const double dir[3], const double rel_vel[3])
{
    double inv_dir[3];
    double inv_vel[3];

    if (!dir) return;
    inv_dir[0] = -dir[0];
    inv_dir[1] = -dir[1];
    inv_dir[2] = -dir[2];
    if (rel_vel) {
        inv_vel[0] = -rel_vel[0];
        inv_vel[1] = -rel_vel[1];
        inv_vel[2] = -rel_vel[2];
    } else {
        inv_vel[0] = inv_vel[1] = inv_vel[2] = 0.0;
    }

    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        if (!disc->initialized) continue;
        if (disc->parent_idx == target_idx && body_is_ring_perturber_planet(impactor_idx))
            damage_disc(disc, target_idx, rel_speed, dir, rel_vel);
        else if (disc->parent_idx == impactor_idx && body_is_ring_perturber_planet(target_idx))
            damage_disc(disc, impactor_idx, rel_speed, inv_dir,
                        rel_vel ? inv_vel : NULL);
    }
}

/*
 * rings_on_body_absorbed — handle ring ownership transfer after a collision.
 *
 * Two cases:
 *   - Impactor ring: inherit it if the target has no ring and is not a star.
 *   - Target ring: retune it for the target's updated mass/obliquity.
 *
 * Visual and motion transfer blends hide the otherwise abrupt parent change.
 */
void rings_on_body_absorbed(int target_idx, int impactor_idx)
{
    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        if (!disc->initialized) continue;

        if (disc->parent_idx == impactor_idx) {
            int target_has_ring = 0;
            float old_parent_radius_km = disc->parent_radius_ref_km;
            float old_mean_motion = disc->mean_motion_mid;
            double old_center_m[3] = {
                g_bodies[impactor_idx].pos[0],
                g_bodies[impactor_idx].pos[1],
                g_bodies[impactor_idx].pos[2]
            };
            float old_b1[3], old_b2[3], old_pole[3];
            memcpy(old_b1, disc->b1, 3 * sizeof(float));
            memcpy(old_b2, disc->b2, 3 * sizeof(float));
            memcpy(old_pole, disc->pole, 3 * sizeof(float));
            for (int k = 0; k < s_n_discs; k++) {
                if (k == d || !s_discs[k].initialized) continue;
                if (s_discs[k].parent_idx == target_idx) {
                    target_has_ring = 1;
                    break;
                }
            }
            if (target_has_ring || target_idx < 0 || target_idx >= g_nbodies ||
                !g_bodies[target_idx].alive || g_bodies[target_idx].is_star) {
                disc->initialized = 0;
            } else {
                retune_disc_parent(disc, target_idx);
                disc_begin_visual_transfer(disc, old_center_m, old_b1, old_b2,
                                           old_pole, target_idx);
                disc_begin_motion_transfer(disc, old_mean_motion,
                                           disc->transfer_duration);
                disc_clear_hit_cooldown(disc);
                disc_drive_transfer_response(disc,
                                             old_parent_radius_km,
                                             (float)(g_bodies[target_idx].radius * 0.001),
                                             0.30f);
            }
        } else if (disc->parent_idx == target_idx) {
            float old_parent_radius_km = disc->parent_radius_ref_km;
            float old_mean_motion = disc->mean_motion_mid;
            double old_center_m[3] = {
                g_bodies[target_idx].pos[0],
                g_bodies[target_idx].pos[1],
                g_bodies[target_idx].pos[2]
            };
            float old_b1[3], old_b2[3], old_pole[3];
            memcpy(old_b1, disc->b1, 3 * sizeof(float));
            memcpy(old_b2, disc->b2, 3 * sizeof(float));
            memcpy(old_pole, disc->pole, 3 * sizeof(float));
            /* Target's own ring: retune for updated mass after absorption */
            retune_disc_parent(disc, target_idx);
            disc_begin_visual_transfer(disc, old_center_m, old_b1, old_b2,
                                       old_pole, target_idx);
            disc_begin_motion_transfer(disc, old_mean_motion,
                                       disc->transfer_duration);
            disc_clear_hit_cooldown(disc);
            disc_drive_transfer_response(disc,
                                         old_parent_radius_km,
                                         (float)(g_bodies[target_idx].radius * 0.001),
                                         0.18f);
        }
    }
}

/* rings_shutdown — free CPU-side particle data and GPU resources. */
void rings_shutdown(void)
{
    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        free(disc->data_full);   free(disc->data_lod);
        free(disc->n_arr_full);  free(disc->n_arr_lod);
        if (disc->vbo_full)      glDeleteBuffers(1,      &disc->vbo_full);
        if (disc->vao_full)      glDeleteVertexArrays(1, &disc->vao_full);
        if (disc->vbo_lod)       glDeleteBuffers(1,      &disc->vbo_lod);
        if (disc->vao_lod)       glDeleteVertexArrays(1, &disc->vao_lod);
        if (disc->shader)        glDeleteProgram(disc->shader);
    }
    free(s_discs);
    s_discs   = NULL;
    s_n_discs = 0;
}
