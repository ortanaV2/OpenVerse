/*
 * build.c — runtime body placement ("build mode")
 *
 * Build mode lets the user interactively place bodies into the live simulation.
 * Pressing B enters build mode (pauses simulation), Tab+scroll selects the
 * preset type, left-click places a body in front of the camera.
 *
 * Key subsystems
 * ──────────────
 * Preset table      Six archetypes (rocky, gas giant, ice planet, moon, dwarf,
 *                   star) with canonical mass, radius, colour, and atmospheric
 *                   parameters. Stars use a fixed colour; all other types get a
 *                   procedurally randomised colour from random_color_in_range().
 *
 * Colour palette    Each visual type has four hand-tuned corner colours arranged
 *                   into three segments. A LCG roll picks one segment (t),
 *                   a second roll interpolates within it (u), and two more rolls
 *                   add per-channel noise (v, w). This produces ~4,000 distinct
 *                   hues that all look physically plausible for the type.
 *
 * Placement pos     build_preview_pos_au() projects the preset along the camera
 *                   forward vector at a distance scaled so the body subtends a
 *                   constant ~18 px on screen, clamped to [0.00035, 18] AU.
 *
 * Parent inference  build_nearest3() returns three reference bodies in the
 *                   camera region — the parent is picked from that set based on
 *                   the preset's wants_nonstar_parent / wants_planet_parent flags.
 *                   A moon preset prefers the nearest non-star; planets prefer
 *                   the nearest star within BUILD_PARENT_STAR_MAX_AU.
 *
 * Orbital velocity  add_parent_velocity() gives the new body a circular orbit
 *                   around its inferred parent: v = sqrt(GM/r) along the cross
 *                   product of the separation vector and world-up. If the
 *                   separation is nearly parallel to world-up, the camera
 *                   forward direction provides a fallback tangent.
 *
 * Post-placement    universe_rebind_to_nearest_stars() is called when a star is
 *                   placed so that existing planets re-evaluate their parent star
 *                   (relevant when two stars are placed close together).
 */
#include "build.h"
#include "body.h"
#include "camera.h"
#include "physics.h"
#include "collision.h"
#include "universe.h"
#include "trails.h"
#include "labels.h"
#include <float.h>

int g_build_mode = 0;
int g_build_tab_held = 0;

/* ── static state ─────────────────────────────────────────────────────────── */
static int s_selected = 0;         /* index into s_presets[] */
static int s_prev_paused = 0;      /* pause state before build mode was entered */
static int s_place_serial = 1;     /* monotonic counter appended to generated names */
static unsigned int s_build_rng = 0x51f15eedu; /* LCG state; fixed seed for determinism */

/* Thresholds that govern build_nearest3() mode selection (in AU). */
#define BUILD_REF_LOCAL_AU        0.018   /* inside this → local mode (moon placement) */
#define BUILD_REF_INTERSTELLAR_AU 1200.0  /* outside this → star-only mode */
#define BUILD_PREVIEW_TARGET_PX   18.0    /* desired angular size of preview body on screen */
#define BUILD_PREVIEW_MIN_AU      0.00035 /* minimum placement distance (inside bodies) */
#define BUILD_PREVIEW_MAX_AU      18.0    /* maximum placement distance */
#define BUILD_PARENT_STAR_MAX_AU  250.0   /* star not accepted as parent beyond this */

/* ── preset table ─────────────────────────────────────────────────────────── */
/* Each row: { name, mass_kg, radius_m, col_rgb, visual_type,
 *             is_star, is_black_hole, wants_planet_parent, wants_nonstar_parent,
 *             atm_color_rgb, atm_intensity, atm_scale }              */
static const BuildPreset s_presets[] = {
    { "Rocky Planet", 5.972e24,   6371.0e3,  {0.52f, 0.44f, 0.36f}, BUILD_VIS_ROCKY,        0, 0, 1, 0, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f },
    { "Gas Giant",    1.898e27,  71492.0e3,  {0.84f, 0.70f, 0.50f}, BUILD_VIS_GAS_GIANT,    0, 0, 1, 0, {0.90f, 0.72f, 0.50f}, 0.25f, 1.22f },
    { "Ice Planet",   8.681e25,  25559.0e3,  {0.62f, 0.84f, 0.96f}, BUILD_VIS_ICE_PLANET,   0, 0, 1, 0, {0.62f, 0.90f, 1.00f}, 0.22f, 1.22f },
    { "Moon",         7.342e22,   1737.4e3,  {0.72f, 0.72f, 0.68f}, BUILD_VIS_MOON,         0, 0, 0, 1, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f },
    { "Dwarf Planet", 1.309e22,   1188.3e3,  {0.68f, 0.63f, 0.58f}, BUILD_VIS_DWARF_PLANET, 0, 0, 1, 0, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f },
    { "Star",         1.989e30, 696000.0e3,  {1.00f, 0.92f, 0.28f}, BUILD_VIS_STAR,         1, 0, 0, 0, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f },
    /* Black hole: 10 solar masses, an enlarged visual horizon so it is easy to
     * place and see. is_star=1 makes it a system root; is_black_hole=1 switches
     * on the dark-horizon render path and the accretion disk / photon ring. */
    { "Black Hole",   1.989e31,  50000.0e3,  {0.45f, 0.22f, 0.05f}, BUILD_VIS_BLACK_HOLE,   1, 1, 0, 0, {0.00f, 0.00f, 0.00f}, 0.00f, 1.00f }
};

/* ── PRNG ─────────────────────────────────────────────────────────────────── */
/* Linear congruential generator (LCG) with Knuth constants.
 * Output masked to 24 bits to avoid the low-bit periodicity of LCGs, then
 * mapped to [0, 1). Produces a different colour every call sequence even when
 * placements happen on the same frame. */
static double build_rand01(void)
{
    s_build_rng = 1664525u * s_build_rng + 1013904223u;
    return (double)(s_build_rng & 0x00ffffffu) / (double)0x01000000u;
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* ── colour generation ────────────────────────────────────────────────────── */
/*
 * random_color_in_range — procedurally sample a perceptually natural colour for
 * a body of the given visual type.
 *
 * Algorithm
 * ─────────
 * Each type has four hand-tuned corner colours (c0–c3) arranged into three
 * overlapping segments: [c0,c1], [c1,c2], [c2,c3].
 *
 *   t  — chooses which segment (three equal-probability thirds)
 *   u  — interpolates within the chosen segment (scaled to 0.95 to avoid
 *         landing on the exact corner colours, which look synthetic)
 *   v,w — small per-channel offset noise (±0.05 R, ±0.04 G/B) so that two
 *         bodies placed at the same t/u still differ slightly
 *
 * Result is clamped to [0, 1] after noise. Stars always use the fixed preset
 * colour and skip this function.
 */
static void random_color_in_range(BuildVisualType type, float out[3])
{
    float c0[3], c1[3], c2[3], c3[3];
    double t = build_rand01();
    double u = build_rand01();
    double v = build_rand01();
    double w = build_rand01();

    switch (type) {
    case BUILD_VIS_ROCKY:
        /* basalt grey -> dusty tan -> rusty brown */
        c0[0] = 0.30f; c0[1] = 0.30f; c0[2] = 0.31f;
        c1[0] = 0.64f; c1[1] = 0.54f; c1[2] = 0.40f;
        c2[0] = 0.46f; c2[1] = 0.36f; c2[2] = 0.30f;
        c3[0] = 0.56f; c3[1] = 0.46f; c3[2] = 0.38f;
        break;
    case BUILD_VIS_GAS_GIANT:
        /* warm ochre -> beige -> muted caramel */
        c0[0] = 0.54f; c0[1] = 0.40f; c0[2] = 0.24f;
        c1[0] = 0.92f; c1[1] = 0.84f; c1[2] = 0.66f;
        c2[0] = 0.72f; c2[1] = 0.56f; c2[2] = 0.34f;
        c3[0] = 0.84f; c3[1] = 0.70f; c3[2] = 0.48f;
        break;
    case BUILD_VIS_ICE_PLANET:
        /* pale ice blue -> frosted cyan -> blue-white */
        c0[0] = 0.56f; c0[1] = 0.72f; c0[2] = 0.88f;
        c1[0] = 0.86f; c1[1] = 0.95f; c1[2] = 1.00f;
        c2[0] = 0.66f; c2[1] = 0.82f; c2[2] = 0.94f;
        c3[0] = 0.78f; c3[1] = 0.90f; c3[2] = 0.98f;
        break;
    case BUILD_VIS_MOON:
        /* charcoal grey -> dusty light grey */
        c0[0] = 0.36f; c0[1] = 0.36f; c0[2] = 0.37f;
        c1[0] = 0.78f; c1[1] = 0.77f; c1[2] = 0.74f;
        c2[0] = 0.54f; c2[1] = 0.54f; c2[2] = 0.55f;
        c3[0] = 0.66f; c3[1] = 0.65f; c3[2] = 0.63f;
        break;
    case BUILD_VIS_DWARF_PLANET:
        /* grey-brown dusty dwarf palette */
        c0[0] = 0.38f; c0[1] = 0.38f; c0[2] = 0.39f;
        c1[0] = 0.72f; c1[1] = 0.68f; c1[2] = 0.60f;
        c2[0] = 0.52f; c2[1] = 0.48f; c2[2] = 0.44f;
        c3[0] = 0.60f; c3[1] = 0.54f; c3[2] = 0.48f;
        break;
    default:
        out[0] = out[1] = out[2] = 1.0f;
        return;
    }

    if (t < 0.33) {
        out[0] = lerpf(c0[0], c1[0], (float)(u * 0.95));
        out[1] = lerpf(c0[1], c1[1], (float)(u * 0.95));
        out[2] = lerpf(c0[2], c1[2], (float)(u * 0.95));
    } else if (t < 0.66) {
        out[0] = lerpf(c1[0], c2[0], (float)(u * 0.95));
        out[1] = lerpf(c1[1], c2[1], (float)(u * 0.95));
        out[2] = lerpf(c1[2], c2[2], (float)(u * 0.95));
    } else {
        out[0] = lerpf(c2[0], c3[0], (float)(u * 0.95));
        out[1] = lerpf(c2[1], c3[1], (float)(u * 0.95));
        out[2] = lerpf(c2[2], c3[2], (float)(u * 0.95));
    }

    /* Per-channel noise: v perturbs R (±0.05), w perturbs G (±0.04),
     * u (already consumed above) reused for B noise — low correlation
     * between channels because u and v come from consecutive LCG states. */
    out[0] = fminf(1.0f, fmaxf(0.0f, out[0] + (float)(v - 0.5) * 0.10f));
    out[1] = fminf(1.0f, fmaxf(0.0f, out[1] + (float)(w - 0.5) * 0.08f));
    out[2] = fminf(1.0f, fmaxf(0.0f, out[2] + (float)(u - 0.5) * 0.08f));
}

/* ── rotation / obliquity ─────────────────────────────────────────────────── */
/* Rotation period ranges are physically motivated (Earth~1 d, gas giants faster,
 * moons wide range to accommodate tidal locking, stars slow). */
static double random_rotation_period_days(BuildVisualType type)
{
    double t = build_rand01();
    switch (type) {
    case BUILD_VIS_ROCKY:        return 0.35 + t * 3.4;
    case BUILD_VIS_GAS_GIANT:    return 0.28 + t * 0.55;
    case BUILD_VIS_ICE_PLANET:   return 0.45 + t * 1.6;
    case BUILD_VIS_MOON:         return 0.4  + t * 18.0;
    case BUILD_VIS_DWARF_PLANET: return 0.25 + t * 5.0;
    case BUILD_VIS_STAR:         return 18.0 + t * 18.0;
    case BUILD_VIS_BLACK_HOLE:   return 0.05 + t * 0.25;
    default:                     return 1.0;
    }
}

/* Obliquity ranges: gas giants low (mostly prograde), ice planets high (Uranus-like),
 * stars fixed at 7.25° (solar reference). Sign is handled in random_rotation_rate. */
static double random_obliquity_deg(BuildVisualType type)
{
    double t = build_rand01();
    switch (type) {
    case BUILD_VIS_GAS_GIANT:    return t * 32.0;
    case BUILD_VIS_ICE_PLANET:   return t * 98.0;
    case BUILD_VIS_MOON:         return t * 18.0;
    case BUILD_VIS_DWARF_PLANET: return t * 55.0;
    case BUILD_VIS_ROCKY:        return t * 45.0;
    case BUILD_VIS_STAR:         return 7.25;
    case BUILD_VIS_BLACK_HOLE:   return 8.0 + t * 32.0;
    default:                     return 0.0;
    }
}

/* Random retrograde rotation (50% chance) except for stars which are always prograde. */
static double random_rotation_rate(BuildVisualType type)
{
    double period_days = random_rotation_period_days(type);
    double sign = build_rand01() < 0.5 ? -1.0 : 1.0;
    if (type == BUILD_VIS_STAR) sign = 1.0;
    return sign * (2.0 * PI) / (period_days * DAY);
}

/* ── init / accessors ─────────────────────────────────────────────────────── */
void build_init(void)
{
    g_build_mode = 0;
    g_build_tab_held = 0;
    s_selected = 0;
    s_prev_paused = 0;
    s_place_serial = 1;
    s_build_rng = 0x51f15eedu;
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

/* ── similarity deduplication ─────────────────────────────────────────────── */
/*
 * reference_too_similar — returns 1 if candidate body 'cand' is too close to
 * any already-selected reference body to be a useful independent reference.
 *
 * The check is relative, not absolute: sep / min(d_cand, d_other) < ratio.
 * This means bodies that are physically close together (like a moon and its
 * planet) are filtered at fine scales, while bodies that are far from the
 * camera are only filtered if they are proportionally close to each other.
 *
 *   mode == 0: stars only → ratio 0.08  (wide deduplication; stars are far apart)
 *   mode != 0: others     → ratio 0.015 (tight; we want nearby independent references)
 */
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

/* ── mode toggle / scroll ─────────────────────────────────────────────────── */
void build_toggle(void)
{
    g_build_mode = !g_build_mode;
    if (g_build_mode) {
        s_prev_paused = g_paused;
        g_paused = 1;   /* freeze physics while the user aims */
    } else {
        g_build_tab_held = 0;
        g_paused = s_prev_paused;
    }
}

void build_set_tab_held(int held)
{
    g_build_tab_held = held ? 1 : 0;
}

/* Scroll wheel cycles presets (wrap-around). Only active when Tab is held. */
void build_scroll(int wheel_y)
{
    if (!g_build_mode || !g_build_tab_held || wheel_y == 0) return;
    int n = build_preset_count();
    s_selected += (wheel_y > 0) ? -1 : 1;
    while (s_selected < 0) s_selected += n;
    while (s_selected >= n) s_selected -= n;
}

/* ── preview position ─────────────────────────────────────────────────────── */
/*
 * build_preview_pos_au — compute where the ghost preview should appear.
 *
 * Distance is chosen so the body fills BUILD_PREVIEW_TARGET_PX (18 px) on
 * screen, using the perspective formula:
 *
 *   px_height = WIN_H * radius_au / (dist_au * tan(FOV/2))
 *   → dist = WIN_H * radius / (target_px * tan(FOV/2))
 *
 * This keeps the preview body consistently sized regardless of the preset's
 * physical radius, which spans >3 orders of magnitude (moon to star).
 * Result is clamped to [BUILD_PREVIEW_MIN_AU, BUILD_PREVIEW_MAX_AU].
 */
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

/* ── reference body selection ─────────────────────────────────────────────── */
/*
 * build_nearest3 — find the three most spatially distinct bodies near pos_m.
 *
 * This is used in two places: render_build_preview() draws guide lines to
 * these references, and build_place_current() infers the new body's parent.
 *
 * Mode selection (determined once before the three passes):
 * ──────────────────────────────────────────────────────────
 *   mode 0  "interstellar" — nearest star is > BUILD_REF_INTERSTELLAR_AU (1200 AU)
 *           away; only stars are candidates. Useful when exploring open space.
 *
 *   mode 1  "system scale" — default; considers all primaries (stars and planets,
 *           no moons). Gives the three nearest stars/planets.
 *
 *   mode 2  "local" — nearest non-star body is within BUILD_REF_LOCAL_AU (0.018 AU),
 *           meaning the cursor is very close to a planet or moon.
 *           Pass 0 in mode 2 builds a tight neighbourhood: the nearest non-star,
 *           its moons, its siblings (same parent), and its parent. This gives
 *           reference lines that frame the local environment precisely.
 *
 * Three-pass fallback:
 * ────────────────────
 * Each pass tries to fill up to 3 slots. If a pass doesn't fill all 3 slots
 * (e.g. there's only one star in mode 0), the next pass widens the scope.
 * Mode transitions: 0→1→2→1 (mode 2 falls back to system scale, not interstellar).
 *
 * Similarity deduplication (reference_too_similar):
 * ──────────────────────────────────────────────────
 * Candidate bodies that are proportionally very close to already-selected ones
 * are skipped. This prevents all three references from clustering on the same
 * planet-moon pair when the cursor sits between them.
 *
 * Emergency fallback (last loop):
 * ─────────────────────────────────
 * If after all three passes some slots are still unfilled, a plain distance
 * sort without deduplication fills the remainder. Ensures we always return
 * up to g_nbodies references.
 */
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

    /* First pass over all bodies to determine mode. */
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
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
            if (!g_bodies[i].alive) continue;
            /* is_moon: parent is a non-star (i.e. planet-moon or moon-moon) */
            int is_moon = (g_bodies[i].parent >= 0 &&
                           !g_bodies[g_bodies[i].parent].is_star);
            int accept = 0;

            if (mode == 0)
                accept = g_bodies[i].is_star;
            else if (mode == 1)
                accept = !is_moon;
            else {
                if (pass == 0 && nearest_nonstar >= 0) {
                    /* Accept: the nearest planet itself, its moons,
                     * its parent star, and its siblings (same parent). */
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
            /* Insertion sort into top-3 by distance. */
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

        /* Merge pass results into output, skipping duplicates. */
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

        /* Widen mode for next pass: 0→1, 1→2, 2→1 */
        if (mode == 0) mode = 1;
        else if (mode == 1) mode = 2;
        else mode = 1;
        if (pass == 2) break;
    }

    /* Emergency fallback: fill remaining slots with nearest un-used bodies. */
    for (int k = 0; k < 3; k++) {
        if (out_idx[k] >= 0) continue;
        double best = DBL_MAX;
        int best_idx = -1;
        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive) continue;
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

/* ── parent helpers ───────────────────────────────────────────────────────── */
static int nearest_star(const double pos_m[3])
{
    int best_idx = -1;
    double best_d2 = DBL_MAX;
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
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

static int nearest_star_within(const double pos_m[3], double max_dist_au)
{
    int idx = nearest_star(pos_m);
    if (idx < 0) return -1;

    {
        double dx = g_bodies[idx].pos[0] - pos_m[0];
        double dy = g_bodies[idx].pos[1] - pos_m[1];
        double dz = g_bodies[idx].pos[2] - pos_m[2];
        double d_au = sqrt(dx*dx + dy*dy + dz*dz) / AU;
        if (d_au > max_dist_au) return -1;
    }
    return idx;
}

static int nearest_nonstar(const double pos_m[3])
{
    int best_idx = -1;
    double best_d2 = DBL_MAX;
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
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

/* ── orbital velocity ─────────────────────────────────────────────────────── */
/*
 * add_parent_velocity — give the new body a circular orbit around 'parent'.
 *
 * Step 1: Inherit parent's bulk velocity (parent may itself be orbiting a star).
 * Step 2: Compute the separation vector r from parent to new body.
 * Step 3: Find a tangent vector t = r × Y_up (cross product).
 *         This is perpendicular to both r and world-up, giving a prograde
 *         direction for a near-equatorial orbit.
 * Step 4: If r is nearly parallel to Y_up (|t| ≈ 0), fall back to using the
 *         camera forward direction projected into the XZ plane. This handles
 *         bodies placed directly above/below the parent.
 * Step 5: Add orbital speed v_circ = sqrt(GM / |r|) along the normalised tangent.
 *
 * The result is an exact circular orbit in a plane defined by r and Y_up.
 * Real orbits are rarely in this exact plane, but it avoids the radial
 * trajectory that would send the body crashing into the parent.
 */
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

    /* Tangent = r × world_up. Degenerates when r ≈ Y axis. */
    double up[3] = {0.0, 1.0, 0.0};
    double tx = ry*up[2] - rz*up[1];
    double ty = rz*up[0] - rx*up[2];
    double tz = rx*up[1] - ry*up[0];
    double tl = sqrt(tx*tx + ty*ty + tz*tz);
    if (tl < 1e-9) {
        /* Fallback: use camera forward XZ component as tangent. */
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

/* ── placement ────────────────────────────────────────────────────────────── */
/*
 * build_place_current — spawn the currently selected preset at the preview
 * position and register it with all subsystems.
 *
 * Parent inference order:
 *   1. wants_nonstar_parent (moon): nearest non-star body
 *   2. wants_planet_parent (planet): nearest star within BUILD_PARENT_STAR_MAX_AU
 *   3. Fallback: same star query (ensures no planet is parentless near a star)
 *
 * After placement:
 *   - Stars trigger universe_rebind_to_nearest_stars() so existing planets
 *     adopt the new star if it is closer than their current parent.
 *   - trails_reset_body() seeds the trail ring with the current position so
 *     the trail starts clean (no stale samples from before placement).
 */
int build_place_current(void)
{
    if (!g_build_mode || !universe_can_add_body()) return -1;

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
    if (p->visual_type == BUILD_VIS_STAR || p->visual_type == BUILD_VIS_BLACK_HOLE) {
        spec.col[0] = p->col[0];
        spec.col[1] = p->col[1];
        spec.col[2] = p->col[2];
    } else {
        random_color_in_range(p->visual_type, spec.col);
    }
    spec.is_star = p->is_star;
    spec.is_black_hole = p->is_black_hole;
    if (p->is_black_hole) {
        /* Warm accretion disk spanning 2.6–9× the event horizon. */
        spec.disk_color[0] = 1.00f;
        spec.disk_color[1] = 0.55f;
        spec.disk_color[2] = 0.18f;
        spec.disk_inner = 2.6f;
        spec.disk_outer = 9.0f;
    }
    spec.parent = -1;
    spec.obliquity = random_obliquity_deg(p->visual_type);
    spec.rotation_rate = random_rotation_rate(p->visual_type);
    spec.atm_color[0] = p->atm_color[0];
    spec.atm_color[1] = p->atm_color[1];
    spec.atm_color[2] = p->atm_color[2];
    spec.atm_intensity = p->atm_intensity;
    spec.atm_scale = p->atm_scale;

    if (!p->is_star) {
        if (p->wants_nonstar_parent)
            spec.parent = nearest_nonstar(spec.pos);
        if (spec.parent < 0 && p->wants_planet_parent)
            spec.parent = nearest_star_within(spec.pos, BUILD_PARENT_STAR_MAX_AU);
        if (spec.parent < 0)
            spec.parent = nearest_star_within(spec.pos, BUILD_PARENT_STAR_MAX_AU);
        add_parent_velocity(&spec, spec.parent);
    }

    int idx = universe_add_body(&spec);
    if (idx >= 0) {
        if (p->is_star)
            universe_rebind_to_nearest_stars();
        trails_add_body(idx);
        trails_reset_body(idx);
        labels_add_body(idx);
        collision_on_body_added(idx);
        fprintf(stdout, "[Build] placed '%s' at %.3f %.3f %.3f AU\n",
                g_bodies[idx].name,
                g_bodies[idx].pos[0] / AU,
                g_bodies[idx].pos[1] / AU,
                g_bodies[idx].pos[2] / AU);
    }
    return idx;
}
