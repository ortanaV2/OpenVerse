/*
 * render.c — scene renderer
 *
 * Pipeline each frame:
 *   1. Starfield        — rotation-only VP, GL_POINTS (skybox)
 *   2. Body spheres     — Phong-lit billboard quads, depth-written
 *   2.5. Atmosphere glow — additive billboard pass (GL_SRC_ALPHA / GL_ONE)
 *   3. Collision particles — additive GL_POINTS (impact ejecta)
 *   3. Center dots      — GL_POINTS (for sub-pixel or occluded bodies)
 *   4. Rings + belts    — rings_render() / asteroids_render()
 *   5. Trails           — trails_render()
 *   6. Star glare       — additive billboard (GL_ONE / GL_ONE), drawn after
 *                         trails so orbit lines vanish inside the glow
 *   6.5. Build preview  — ghost dot + guide lines + distance labels
 *   7. Labels           — labels_render()
 *
 * ── Camera-relative rendering ────────────────────────────────────────────
 *
 * All vertex positions are passed to the GPU as (world_pos − cam_pos), i.e.,
 * relative to the camera.  The shader receives vp_camrel = proj × view_rot
 * (no translation column) instead of the full proj × view matrix.
 *
 * Reason: body positions are stored in double precision (metres), but GL
 * uniforms and vertex attributes are float32.  At interstellar distances
 * (4 ly ≈ 250,000 AU) the world-space offset is too large to represent in
 * float32 without losing the last 5 significant digits.  Subtracting the
 * camera position in double before casting to float keeps the relative
 * coordinates small (≤ planetary-system scale) and fully precise.
 *
 * ── Billboard sphere pattern ─────────────────────────────────────────────
 *
 * All body spheres share a single unit quad (UV 0..1, two triangles).
 * phong.vert receives the quad corner UVs and expands the billboard to the
 * correct size in clip space using the camera right/up basis vectors and the
 * projected sphere radius.  No per-body vertex buffer needed.
 *
 * ── Dot–sphere transition ────────────────────────────────────────────────
 *
 * Each body alternates between a dot (GL_POINT) and a sphere (billboard):
 *   px < BODY_SPHERE_APPEAR_PX (1.25)  → dot only (sphere billboard skipped)
 *   px ∈ [DOT_FADE_START, DOT_FADE_END] (0.75..1.75) → dot fades out
 *   px ≥ DOT_FADE_END (1.75)           → dot fully hidden, sphere only
 *
 * eye_z (depth along view axis) is used for px computation instead of
 * Euclidean dcam.  dcam = eye_z / cos(θ) — rotating the camera changes θ
 * and therefore dcam, causing the show flag to flicker.  eye_z is invariant
 * under pure camera rotation, so the transition is stable.
 */
#include "render.h"
#include "body.h"
#include "camera.h"
#include "starfield.h"
#include "trails.h"
#include "rings.h"
#include "asteroids.h"
#include "labels.h"
#include "build.h"
#include "inspect.h"
#include "collision.h"
#include "gl_utils.h"
#include "math3d.h"
#include "supernova.h"
#include "ui_theme.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ sphere */

/* Unit quad billboard (UV 0..1), two triangles, shared by all body spheres */
static GLuint s_sphere_shader  = 0;
static GLuint s_sphere_vao     = 0;
static GLuint s_sphere_vbo     = 0;
static GLuint s_sphere_ebo     = 0;

/* Sphere shader uniform locations */
static GLint  s_sp_vp          = -1;
static GLint  s_sp_center      = -1;   /* cam-relative body centre (= -u_oc) */
static GLint  s_sp_radius      = -1;
static GLint  s_sp_cam_right   = -1;
static GLint  s_sp_cam_up      = -1;
static GLint  s_sp_oc          = -1;   /* cam − centre, double-computed float */
static GLint  s_sp_sun_rel     = -1;   /* star − centre, for Phong lighting   */
static GLint  s_sp_cam_fwd     = -1;
static GLint  s_sp_fov_tan     = -1;
static GLint  s_sp_aspect      = -1;
static GLint  s_sp_screen      = -1;
static GLint  s_sp_color       = -1;
static GLint  s_sp_emission    = -1;
static GLint  s_sp_ambient     = -1;
static GLint  s_sp_sun_world   = -1;
static GLint  s_sp_rotation      = -1;
static GLint  s_sp_cloud_rotation = -1;
static GLint  s_sp_obliquity   = -1;
static GLint  s_sp_ptype       = -1;   /* procedural texture variant index */
static GLint  s_sp_star_heat   = -1;
static GLint  s_sp_impact_count = -1;
static GLint  s_sp_impact_dir   = -1;
static GLint  s_sp_impact_t1    = -1;
static GLint  s_sp_impact_rad   = -1;
static GLint  s_sp_impact_heat  = -1;
static GLint  s_sp_impact_prog  = -1;
static GLint  s_sp_impact_seed  = -1;
static GLint  s_sp_impact_kind  = -1;
static GLint  s_sp_use_fullscreen = -1; /* 1 when billboard would degenerate (camera near body) */

/*
 * get_planet_type — map body name to a procedural texture variant index.
 *
 * The lookup is name-based (not index-based) to survive loader order changes
 * and future body additions.  Build-mode bodies use prefix matching on their
 * generated names ("Rocky Planet", "Gas Giant", etc.).
 *
 *   0=rocky(default)  1=Earth  2=Mars  3=Venus  4=Jupiter  5=Saturn
 *   6=ice-giant       7=Io     8=Titan 9=Europa 10=proc-rocky
 *   11=proc-gas       12=proc-ice 13=Uranus 14=airless moon
 */
static int get_planet_type(const char *name)
{
    static const struct { const char *name; int ptype; } tbl[] = {
        { "Mercury", 14 },
        { "Ceres",   14 }, { "Pallas", 14 }, { "Vesta",  14 },
        { "Hygiea",  14 }, { "Interamnia",14 },
        { "Pluto",   14 }, { "Eris",   14 }, { "Makemake",14 },
        { "Haumea",  14 },
        { "Moon",    14 }, { "Phobos", 14 }, { "Deimos", 14 },
        { "Ganymede",14 }, { "Callisto",14 },
        { "Mimas",   14 }, { "Enceladus",14 }, { "Tethys",14 },
        { "Dione",   14 }, { "Rhea",   14 },
        { "Miranda", 14 }, { "Ariel",  14 }, { "Umbriel",14 },
        { "Titania", 14 }, { "Oberon", 14 }, { "Triton", 14 },
        { "Earth",   1 }, { "Mars",    2 }, { "Venus",   3 },
        { "Jupiter", 4 }, { "Saturn",  5 }, { "Uranus", 13 },
        { "Neptune", 6 }, { "Io",      7 }, { "Titan",   8 },
        { "Europa",  9 }, { NULL,      0 }
    };
    int k;
    if (!name) return 0;
    if (strncmp(name, "Rocky Planet", 12) == 0) return 10;
    if (strncmp(name, "Gas Giant",    9)  == 0) return 11;
    if (strncmp(name, "Ice Planet",   10) == 0) return 12;
    if (strncmp(name, "Dwarf Planet", 12) == 0) return 14;
    if (strncmp(name, "Moon",          4) == 0) return 14;
    for (k = 0; tbl[k].name; k++)
        if (strcmp(name, tbl[k].name) == 0) return tbl[k].ptype;
    return 0;
}

/* ------------------------------------------------------------------ atmosphere */

/* Atmosphere glow shader — additive billboard over the sphere */
static GLuint s_atm_shader   = 0;
static GLint  s_at_vp        = -1;
static GLint  s_at_center    = -1;
static GLint  s_at_radius    = -1;
static GLint  s_at_cam_right = -1;
static GLint  s_at_cam_up    = -1;
static GLint  s_at_cam_fwd   = -1;
static GLint  s_at_oc        = -1;
static GLint  s_at_planet_r  = -1;
static GLint  s_at_sun_rel   = -1;
static GLint  s_at_color     = -1;
static GLint  s_at_intensity = -1;
static GLint  s_at_aspect    = -1;
static GLint  s_at_screen    = -1;

/* Atmosphere color/intensity/scale are stored per-body in g_bodies[i],
 * loaded from assets/universe.json by universe_load(). */

/* ------------------------------------------------------------------ dots */

/* Center-dot shader: color.vert / color.frag, same as starfield */
static GLuint s_dot_shader  = 0;
static GLuint s_dot_vao     = 0;
static GLuint s_dot_vbo     = 0;           /* dynamic: updated each frame */
static GLint  s_dot_vp      = -1;

/* Impact ejecta particles — additive GL_POINTS from collision system */
static GLuint s_impact_particle_shader = 0;
static GLint  s_impact_particle_vp = -1;
static GLuint s_impact_particle_vao = 0;
static GLuint s_impact_particle_vbo = 0;

#define RENDER_MAX_COLLISION_PARTICLES COLLISION_MAX_PARTICLES

/* ------------------------------------------------------------------ star glare */

/* Star glare billboard — additive (GL_ONE, GL_ONE), drawn after trails */
static GLuint s_glare_shader = 0;
static GLint  s_gl_vp        = -1;
static GLint  s_gl_center    = -1;
static GLint  s_gl_radius    = -1;
static GLint  s_gl_right     = -1;
static GLint  s_gl_up        = -1;
static GLint  s_gl_color     = -1;

/* Supernova passes: volumetric cloud and core. */
static GLuint s_supernova_core_shader = 0;
static GLuint s_supernova_cloud_shader = 0;
static GLint  s_sn_core_vp = -1;
static GLint  s_sn_core_center = -1;
static GLint  s_sn_core_radius = -1;
static GLint  s_sn_core_right = -1;
static GLint  s_sn_core_up = -1;
static GLint  s_sn_core_fwd = -1;
static GLint  s_sn_core_oc = -1;
static GLint  s_sn_core_color = -1;
static GLint  s_sn_core_flash = -1;
static GLint  s_sn_core_core = -1;
static GLint  s_sn_core_ratio = -1;
static GLint  s_sn_core_time = -1;
static GLint  s_sn_core_seed = -1;
static GLint  s_sn_core_bill = -1;
static GLint  s_sn_core_fullscreen = -1;
static GLint  s_sn_core_fov_tan = -1;
static GLint  s_sn_core_aspect = -1;
static GLint  s_sn_core_screen = -1;
static GLint  s_sn_cloud_vp = -1;
static GLint  s_sn_cloud_center = -1;
static GLint  s_sn_cloud_radius = -1;
static GLint  s_sn_cloud_right = -1;
static GLint  s_sn_cloud_up = -1;
static GLint  s_sn_cloud_fwd = -1;
static GLint  s_sn_cloud_color = -1;
static GLint  s_sn_cloud_oc = -1;
static GLint  s_sn_cloud_inner = -1;
static GLint  s_sn_cloud_density = -1;
static GLint  s_sn_cloud_hot = -1;
static GLint  s_sn_cloud_time = -1;
static GLint  s_sn_cloud_seed = -1;
static GLint  s_sn_cloud_bill = -1;
static GLint  s_sn_cloud_fullscreen = -1;
static GLint  s_sn_cloud_fov_tan = -1;
static GLint  s_sn_cloud_aspect = -1;
static GLint  s_sn_cloud_screen = -1;

/* Glare billboard is STAR_GLARE_BILL_SCALE × the star's visual radius.
 * This constant is also used to compute when the dot fades as the glare grows. */
static const float STAR_GLARE_BILL_SCALE      = 15.0f;
/* Sphere appears when its projected radius reaches this many pixels */
static const float BODY_SPHERE_APPEAR_PX      = 1.25f;
/* Dot fade range: [start, end] pixels of projected radius */
static const float BODY_DOT_FADE_START_PX     = 0.75f;
static const float BODY_DOT_FADE_END_PX       = 1.75f;
/* Star dot fully visible / starts fading as glare bill grows (in pixels) */
static const float STAR_DOT_FULL_GLARE_PX     = 1.25f;
static const float STAR_DOT_FADE_START_GLARE_PX = 5.00f;

/* ------------------------------------------------------------------ build preview */

/* Build-mode overlay: guide lines (3D) and distance labels (2D screen-space) */
static GLuint s_build_line_shader = 0;
static GLuint s_build_line_vao = 0;
static GLuint s_build_line_vbo = 0;
static GLint  s_build_line_vp = -1;

static GLuint s_build_ui_shader = 0;
static GLuint s_build_ui_vao = 0;
static GLuint s_build_ui_vbo = 0;
static GLint  s_build_ui_screen = -1;
static GLint  s_build_ui_color = -1;
static GLint  s_build_ui_use_tex = -1;
static GLint  s_build_ui_tex = -1;
static TTF_Font *s_build_font = NULL;

#define BUILD_UI_FONT_SIZE 16.0f

/*
 * BuildTextCache — GPU texture for one distance label string.
 *
 * The str field is compared before re-rendering; the SDL_TTF texture is
 * recreated only when the string changes, avoiding a per-frame upload.
 */
typedef struct {
    GLuint tex;
    int w, h;
    char str[96];
} BuildTextCache;

static BuildTextCache s_build_dist_text[3];   /* one per guide line (up to 3) */

/* ------------------------------------------------------------------ helpers */

static float half_fov_tan(void) {
    return tanf(FOV * 0.5f * (float)(PI / 180.0));
}


/* Render `str` into tc->tex via SDL_TTF.  No-op if the string didn't change. */
static void build_update_text(BuildTextCache *tc, const char *str) {
    if (!s_build_font || strcmp(tc->str, str) == 0) return;
    snprintf(tc->str, sizeof(tc->str), "%s", str);
    if (tc->tex) {
        glDeleteTextures(1, &tc->tex);
        tc->tex = 0;
    }
    SDL_Color col = {235, 245, 255, 235};
    SDL_Surface *surf = TTF_RenderText_Blended(s_build_font, str, col);
    if (surf) tc->tex = gl_surf_to_tex(surf, &tc->w, &tc->h);
}

/* Upload a screen-space quad to the bound GL_ARRAY_BUFFER and draw it.
 * Vertices are (x,y,u,v) pairs; the shader reads these as loc0=xy, loc1=uv. */
static void build_draw_ui_quad(float x, float y, float w, float h) {
    float v[24] = {
        x,   y,   0,0,
        x+w, y,   1,0,
        x+w, y+h, 1,1,
        x,   y,   0,0,
        x+w, y+h, 1,1,
        x,   y+h, 0,1,
    };
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(v), v);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

/* Draw a cached text texture at screen position (x, y) at a fixed pixel height. */
static void build_draw_text(BuildTextCache *tc, float x, float y, float h) {
    if (!tc || !tc->tex || !s_build_ui_shader) return;
    float w = h * (float)tc->w / (float)tc->h;  /* preserve texture aspect */
    glUniform1i(s_build_ui_use_tex, 1);
    glUniform4f(s_build_ui_color, 1, 1, 1, 1);
    glBindTexture(GL_TEXTURE_2D, tc->tex);
    build_draw_ui_quad(x, y, w, h);
}

/*
 * draw_ring_2d — inspect-target ring as a screen-space dashed circle.
 *
 * Centre: uses the corrected perspective divisor pz_adj = eye_z − R²/eye_z.
 *   For an off-axis sphere the projected silhouette is an ellipse whose
 *   centre is NOT proj(body_pos) — it shifts toward the screen edge.
 *   Dividing the clip-space x/y by pz_adj instead of eye_z gives the true
 *   visual centre of the sphere disc at any camera angle.
 *
 * Radius: dr × 1.3 projected at Euclidean distance D (angle-stable),
 *   with a 12 px floor so the ring stays visible for any body at any distance.
 */
static void draw_ring_2d(const float rel[3], float dr,
                         float alpha, const float vp[16])
{
    enum { N_DASHES = 8, SEGS = 5 };
    float v[N_DASHES * SEGS * 2 * 4];
    float D, r_px, cx, cy, phase, step;
    float clip_x, clip_y, clip_w, pz_adj;
    int vtx = 0;

    if (!s_build_ui_shader || !s_build_ui_vbo) return;

    D = sqrtf(rel[0]*rel[0] + rel[1]*rel[1] + rel[2]*rel[2]);
    if (D < 1e-6f) return;

    clip_x = vp[0]*rel[0] + vp[4]*rel[1] + vp[8] *rel[2] + vp[12];
    clip_y = vp[1]*rel[0] + vp[5]*rel[1] + vp[9] *rel[2] + vp[13];
    clip_w = vp[3]*rel[0] + vp[7]*rel[1] + vp[11]*rel[2] + vp[15];
    if (clip_w <= 0.0f) return;

    pz_adj = clip_w - dr * dr / clip_w;
    if (pz_adj <= 0.0f) return;

    cx = (clip_x / pz_adj + 1.0f) * 0.5f * (float)WIN_W;
    cy = (float)WIN_H - (clip_y / pz_adj + 1.0f) * 0.5f * (float)WIN_H;

    /* Screen radius using D (Euclidean), not eye_z, so it is angle-stable */
    r_px = dr * 1.3f * (WIN_H * 0.5f) / (D * half_fov_tan());
    if (r_px < 12.0f) r_px = 12.0f;

    phase = (float)SDL_GetTicks() * 0.00055f;
    step  = 2.0f * (float)PI / (float)N_DASHES;

    for (int d = 0; d < N_DASHES; d++) {
        float a0 = phase + (float)d * step;
        float a1 = a0 + step * 0.45f;
        for (int s = 0; s < SEGS; s++) {
            float aa = a0 + (a1 - a0) * (float)s       / (float)SEGS;
            float ab = a0 + (a1 - a0) * (float)(s + 1) / (float)SEGS;
            v[vtx*4+0]=cx+cosf(aa)*r_px; v[vtx*4+1]=cy+sinf(aa)*r_px;
            v[vtx*4+2]=0.0f; v[vtx*4+3]=0.0f; vtx++;
            v[vtx*4+0]=cx+cosf(ab)*r_px; v[vtx*4+1]=cy+sinf(ab)*r_px;
            v[vtx*4+2]=0.0f; v[vtx*4+3]=0.0f; vtx++;
        }
    }

    glUseProgram(s_build_ui_shader);
    glUniform2f(s_build_ui_screen, (float)WIN_W, (float)WIN_H);
    glUniform1i(s_build_ui_use_tex, 0);
    glUniform4f(s_build_ui_color, 1.0f, 1.0f, 1.0f, alpha);
    glBindVertexArray(s_build_ui_vao);
    glBindBuffer(GL_ARRAY_BUFFER, s_build_ui_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vtx * 4 * sizeof(float), v);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(1.5f);
    glDrawArrays(GL_LINES, 0, vtx);
    glLineWidth(1.0f);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glBindVertexArray(0);
}


/*
 * supernova_fullscreen_raster_local - choose whether a volumetric supernova
 * pass should rasterize via a fullscreen quad instead of a world-space
 * billboard.
 *
 * The fragment shader already computes the true camera ray from gl_FragCoord
 * and marches against u_oc, so the draw primitive is only there to generate
 * fragments. When the observer is inside or very near the volume, the 3D
 * billboard can expose its own edges as the camera pans. Switching to a
 * fullscreen quad removes those layer-specific clip boundaries entirely.
 */
static int supernova_fullscreen_raster_local(const float center[3],
                                             const float cam_fwd[3],
                                             float coverage_radius,
                                             float bill_scale)
{
    const float edge_overscan = 2.0f; /* keep in sync with supernova_billboard.vert */
    float eye_z = center[0] * cam_fwd[0]
                + center[1] * cam_fwd[1]
                + center[2] * cam_fwd[2];
    float half_extent = coverage_radius * bill_scale * edge_overscan;
    float min_eye_z = fmaxf(half_extent * 1.05f, 0.18f);
    return eye_z < min_eye_z;
}

static int supernova_far_raster_local(const float center[3],
                                      const float cam_fwd[3],
                                      float coverage_radius,
                                      float bill_scale)
{
    const float edge_overscan = 2.0f; /* keep in sync with supernova_billboard.vert */
    const float far_guard = 1850.0f;
    float eye_z = center[0] * cam_fwd[0]
                + center[1] * cam_fwd[1]
                + center[2] * cam_fwd[2];
    float half_extent = coverage_radius * bill_scale * edge_overscan;
    return eye_z + half_extent >= far_guard;
}

static float supernova_distance_fade_local(float dist, float radius)
{
    float fade_start = 2600.0f + radius * 2.5f;
    float fade_end = 18000.0f + radius * 18.0f;
    if (fade_end <= fade_start + 1.0f) fade_end = fade_start + 1.0f;
    if (dist <= fade_start) return 1.0f;
    if (dist >= fade_end) return 0.0f;
    return 1.0f - (float)smoothstepd(fade_start, fade_end, dist);
}

/* Visual render radius in AU-units (= metres × RS), accounting for collision
 * scaling.  Bodies that have absorbed mass grow their visual radius. */
static float visual_radius(int i, float dcam) {
    (void)dcam;
    return (float)(collision_visual_radius(i, g_bodies[i].radius) * RS);
}

/*
 * body_point_star_glare_visibility — compute how much the dot for body_idx
 * is occluded/dimmed by other stars' glare coronae.
 *
 * For each other live star, casts a ray from the camera toward body_idx and
 * measures the perpendicular distance from that star to the ray.  If the
 * star's physical disc intersects the ray, returns 0 (fully blocked).
 * Otherwise, evaluates the glare shader's exponential falloff at the radial
 * distance and converts it to a visibility fraction via smoothstep.
 *
 * The glare formula mirrors star_glare.frag so dot transparency exactly
 * matches the corona brightness — the dot fades precisely as the glare
 * brightens, with no visible seam.
 *
 * Returns 1.0 (fully visible) if no star intersects.
 */
static float body_point_star_glare_visibility(int body_idx) {
    Body *b = &g_bodies[body_idx];

    double bx = b->pos[0] * RS - g_cam.pos[0];
    double by = b->pos[1] * RS - g_cam.pos[1];
    double bz = b->pos[2] * RS - g_cam.pos[2];
    double bd2 = bx*bx + by*by + bz*bz;
    if (bd2 <= 1e-18) return 0;

    /* Unit direction from camera toward the body */
    double bd = sqrt(bd2);
    double ux = bx / bd;
    double uy = by / bd;
    double uz = bz / bd;
    double visibility = 1.0;

    for (int i = 0; i < g_nbodies; i++) {
        if (i == body_idx) continue;
        Body *s = &g_bodies[i];
        if (!s->alive || !s->is_star) continue;

        /* Project star position onto the ray */
        double sx = s->pos[0] * RS - g_cam.pos[0];
        double sy = s->pos[1] * RS - g_cam.pos[1];
        double sz = s->pos[2] * RS - g_cam.pos[2];
        double along = sx*ux + sy*uy + sz*uz;
        /* Skip if star is behind camera or beyond the target body */
        if (along <= 0.0 || along >= bd) continue;

        /* Perpendicular distance from star to the ray, in star-radius units */
        double sd2 = sx*sx + sy*sy + sz*sz;
        double perp2 = sd2 - along*along;
        double sr = s->radius * RS;
        double r_units = sqrt(perp2 < 0.0 ? 0.0 : perp2) / sr;
        if (r_units <= 1.0) return 0.0f;   /* physical disc blocks the ray */

        /* Glare falloff matching star_glare.frag */
        double r_safe = r_units > 0.05 ? r_units : 0.05;
        double shine = 2.1 * exp(-r_safe * 0.48);
        double outer_fade = 1.0 - smoothstepd(6.0, 15.0, r_units);
        double glare = shine * outer_fade;
        double star_vis = 1.0 - smoothstepd(0.04, 0.38, glare);
        if (star_vis < visibility) visibility = star_vis;
    }

    return (float)visibility;
}

/*
 * system_dot_fade_for_body — LOD fade for planet/moon dots at interstellar range.
 *
 * Planets and moons are rendered as dots even when their system is not the
 * camera's current system.  At interstellar distances individual planet dots
 * become misleading (wrong scale, wrong context), so they fade out as the
 * camera moves far from the body's host star:
 *   distance < SYS_DOT_FADE_START  → full opacity
 *   distance > SYS_DOT_FADE_END    → invisible
 *
 * For bodies with no valid star parent (should not occur in normal operation),
 * the body's own distance to the camera is used instead.
 */
static float system_dot_fade_for_body(int body_idx)
{
    int ref_star;
    double sdx, sdy, sdz, dist_star;
    float dot_fade;
    const double FREE_DOT_FADE_START = 400.0;
    const double FREE_DOT_FADE_END = 800.0;

    if (body_idx < 0 || body_idx >= g_nbodies) return 1.0f;
    if (g_bodies[body_idx].is_star) return 1.0f;

    ref_star = body_root_star(body_idx);
    if (ref_star < 0 || ref_star >= g_nbodies ||
        !g_bodies[ref_star].alive || !g_bodies[ref_star].is_star) {
        /* No host star: use body-to-camera distance */
        double bx = g_bodies[body_idx].pos[0] * RS - g_cam.pos[0];
        double by = g_bodies[body_idx].pos[1] * RS - g_cam.pos[1];
        double bz = g_bodies[body_idx].pos[2] * RS - g_cam.pos[2];
        double dist_body = sqrt(bx*bx + by*by + bz*bz);

        dot_fade = 1.0f - (float)((dist_body - FREE_DOT_FADE_START)
                                / (FREE_DOT_FADE_END - FREE_DOT_FADE_START));
        if (dot_fade > 1.0f) dot_fade = 1.0f;
        if (dot_fade < 0.0f) dot_fade = 0.0f;
        return dot_fade;
    }

    /* Camera-to-star distance drives the fade for the whole system */
    sdx = g_cam.pos[0] - g_bodies[ref_star].pos[0] * RS;
    sdy = g_cam.pos[1] - g_bodies[ref_star].pos[1] * RS;
    sdz = g_cam.pos[2] - g_bodies[ref_star].pos[2] * RS;
    dist_star = sqrt(sdx*sdx + sdy*sdy + sdz*sdz);

    dot_fade = 1.0f - (float)((dist_star - SYS_DOT_FADE_START)
                            / (SYS_DOT_FADE_END - SYS_DOT_FADE_START));
    if (dot_fade > 1.0f) dot_fade = 1.0f;
    if (dot_fade < 0.0f) dot_fade = 0.0f;
    return dot_fade;
}

/*
 * body_point_occluded_by_body — ray-sphere test for dot occlusion.
 *
 * Returns 1 if a nearer, visible-as-sphere (info[i].show == 0) body's disc
 * falls between the camera and body_idx along the look ray.  Stars are never
 * considered occluders (they are always shown as dots or glare).
 */
static int body_point_occluded_by_body(int body_idx, const BodyRenderInfo info[]) {
    double bx = g_bodies[body_idx].pos[0] * RS - g_cam.pos[0];
    double by = g_bodies[body_idx].pos[1] * RS - g_cam.pos[1];
    double bz = g_bodies[body_idx].pos[2] * RS - g_cam.pos[2];
    double bd2 = bx*bx + by*by + bz*bz;
    if (bd2 <= 1e-18) return 0;

    double bd = sqrt(bd2);
    double ux = bx / bd;
    double uy = by / bd;
    double uz = bz / bd;

    for (int i = 0; i < g_nbodies; i++) {
        if (i == body_idx) continue;
        /* Only bodies rendered as spheres (not dots) can occlude */
        if (!g_bodies[i].alive || g_bodies[i].is_star || info[i].show) continue;

        double sx = g_bodies[i].pos[0] * RS - g_cam.pos[0];
        double sy = g_bodies[i].pos[1] * RS - g_cam.pos[1];
        double sz = g_bodies[i].pos[2] * RS - g_cam.pos[2];
        double along = sx*ux + sy*uy + sz*uz;
        if (along <= 0.0 || along >= bd) continue;

        double sd2 = sx*sx + sy*sy + sz*sz;
        double perp2 = sd2 - along*along;
        double r = info[i].dr;
        if (perp2 <= r*r) return 1;
    }

    return 0;
}

/* ------------------------------------------------------------------ init */
void render_init(void) {
    /* GL_DEPTH_CLAMP prevents near-plane clipping of close billboard geometry.
     * Without it, the large sphere billboard quad (which extends well beyond
     * the near plane at very close range) would be clipped and produce a hole
     * at the sphere edge.  With depth clamp, fragments with z < near are kept
     * but clamped to depth 0 rather than discarded. */
    glEnable(GL_DEPTH_CLAMP);

    /* --- Sphere billboard shader --- */
    s_sphere_shader = gl_shader_load("assets/shaders/phong.vert",
                                     "assets/shaders/phong.frag");
    if (!s_sphere_shader) {
        fprintf(stderr, "[Render] phong shader failed\n");
        return;
    }

    s_sp_vp       = glGetUniformLocation(s_sphere_shader, "u_vp");
    s_sp_center   = glGetUniformLocation(s_sphere_shader, "u_center");
    s_sp_radius   = glGetUniformLocation(s_sphere_shader, "u_radius");
    s_sp_cam_right= glGetUniformLocation(s_sphere_shader, "u_cam_right");
    s_sp_cam_up   = glGetUniformLocation(s_sphere_shader, "u_cam_up");
    s_sp_color     = glGetUniformLocation(s_sphere_shader, "u_color");
    s_sp_emission  = glGetUniformLocation(s_sphere_shader, "u_emission");
    s_sp_ambient   = glGetUniformLocation(s_sphere_shader, "u_ambient");
    s_sp_sun_world = glGetUniformLocation(s_sphere_shader, "u_sun_pos_world");
    s_sp_oc        = glGetUniformLocation(s_sphere_shader, "u_oc");
    s_sp_sun_rel   = glGetUniformLocation(s_sphere_shader, "u_sun_rel");
    s_sp_cam_fwd   = glGetUniformLocation(s_sphere_shader, "u_cam_fwd");
    s_sp_fov_tan   = glGetUniformLocation(s_sphere_shader, "u_fov_tan");
    s_sp_aspect    = glGetUniformLocation(s_sphere_shader, "u_aspect");
    s_sp_screen    = glGetUniformLocation(s_sphere_shader, "u_screen");
    s_sp_rotation        = glGetUniformLocation(s_sphere_shader, "u_rotation");
    s_sp_cloud_rotation  = glGetUniformLocation(s_sphere_shader, "u_cloud_rotation");
    s_sp_obliquity = glGetUniformLocation(s_sphere_shader, "u_obliquity");
    s_sp_ptype     = glGetUniformLocation(s_sphere_shader, "u_planet_type");
    s_sp_star_heat = glGetUniformLocation(s_sphere_shader, "u_star_heat");
    s_sp_impact_count = glGetUniformLocation(s_sphere_shader, "u_impact_count");
    s_sp_impact_dir   = glGetUniformLocation(s_sphere_shader, "u_impact_dir[0]");
    s_sp_impact_t1    = glGetUniformLocation(s_sphere_shader, "u_impact_tangent1[0]");
    s_sp_impact_rad   = glGetUniformLocation(s_sphere_shader, "u_impact_radius[0]");
    s_sp_impact_heat  = glGetUniformLocation(s_sphere_shader, "u_impact_heat[0]");
    s_sp_impact_prog  = glGetUniformLocation(s_sphere_shader, "u_impact_progress[0]");
    s_sp_impact_seed  = glGetUniformLocation(s_sphere_shader, "u_impact_seed[0]");
    s_sp_impact_kind  = glGetUniformLocation(s_sphere_shader, "u_impact_kind[0]");
    s_sp_use_fullscreen = glGetUniformLocation(s_sphere_shader, "u_use_fullscreen");

    /* Frame-constant uniforms (fov_tan, aspect, screen do not change at runtime) */
    glUseProgram(s_sphere_shader);
    glUniform1f(s_sp_fov_tan, tanf(FOV * 0.5f * (float)(PI / 180.0)));
    glUniform1f(s_sp_aspect,  (float)WIN_W / (float)WIN_H);
    glUniform2f(s_sp_screen,  (float)WIN_W, (float)WIN_H);
    glUseProgram(0);

    /* Unit quad: corners at UV (0,0)..(1,1), two triangles */
    static const float quad_v[] = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
    };
    static const unsigned int quad_i[] = { 0,1,2, 0,2,3 };

    s_sphere_vao = gl_vao_create();
    s_sphere_vbo = gl_vbo_create(sizeof(quad_v), quad_v, GL_STATIC_DRAW);
    s_sphere_ebo = gl_ebo_create(sizeof(quad_i), quad_i);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glBindVertexArray(0);

    /* --- Star glare shader --- */
    s_glare_shader = gl_shader_load("assets/shaders/star_glare.vert",
                                    "assets/shaders/star_glare.frag");
    if (!s_glare_shader)
        fprintf(stderr, "[Render] star_glare shader failed\n");
    else {
        s_gl_vp     = glGetUniformLocation(s_glare_shader, "u_vp");
        s_gl_center = glGetUniformLocation(s_glare_shader, "u_center");
        s_gl_radius = glGetUniformLocation(s_glare_shader, "u_radius");
        s_gl_right  = glGetUniformLocation(s_glare_shader, "u_cam_right");
        s_gl_up     = glGetUniformLocation(s_glare_shader, "u_cam_up");
        s_gl_color  = glGetUniformLocation(s_glare_shader, "u_color");
    }

    /* --- Supernova shaders --- */
    s_supernova_core_shader = gl_shader_load("assets/shaders/supernova_billboard.vert",
                                             "assets/shaders/supernova_core.frag");
    if (!s_supernova_core_shader)
        fprintf(stderr, "[Render] supernova core shader failed\n");
    else {
        s_sn_core_vp = glGetUniformLocation(s_supernova_core_shader, "u_vp");
        s_sn_core_center = glGetUniformLocation(s_supernova_core_shader, "u_center");
        s_sn_core_radius = glGetUniformLocation(s_supernova_core_shader, "u_radius");
        s_sn_core_right = glGetUniformLocation(s_supernova_core_shader, "u_cam_right");
        s_sn_core_up = glGetUniformLocation(s_supernova_core_shader, "u_cam_up");
        s_sn_core_fwd = glGetUniformLocation(s_supernova_core_shader, "u_cam_fwd");
        s_sn_core_oc = glGetUniformLocation(s_supernova_core_shader, "u_oc");
        s_sn_core_color = glGetUniformLocation(s_supernova_core_shader, "u_color");
        s_sn_core_flash = glGetUniformLocation(s_supernova_core_shader, "u_flash_intensity");
        s_sn_core_core = glGetUniformLocation(s_supernova_core_shader, "u_core_intensity");
        s_sn_core_ratio = glGetUniformLocation(s_supernova_core_shader, "u_core_ratio");
        s_sn_core_time = glGetUniformLocation(s_supernova_core_shader, "u_time");
        s_sn_core_seed = glGetUniformLocation(s_supernova_core_shader, "u_seed");
        s_sn_core_bill = glGetUniformLocation(s_supernova_core_shader, "u_bill_scale");
        s_sn_core_fullscreen = glGetUniformLocation(s_supernova_core_shader, "u_fullscreen");
        s_sn_core_fov_tan = glGetUniformLocation(s_supernova_core_shader, "u_fov_tan");
        s_sn_core_aspect = glGetUniformLocation(s_supernova_core_shader, "u_aspect");
        s_sn_core_screen = glGetUniformLocation(s_supernova_core_shader, "u_screen");
    }

    s_supernova_cloud_shader = gl_shader_load("assets/shaders/supernova_billboard.vert",
                                              "assets/shaders/supernova_cloud.frag");
    if (!s_supernova_cloud_shader)
        fprintf(stderr, "[Render] supernova cloud shader failed\n");
    else {
        s_sn_cloud_vp = glGetUniformLocation(s_supernova_cloud_shader, "u_vp");
        s_sn_cloud_center = glGetUniformLocation(s_supernova_cloud_shader, "u_center");
        s_sn_cloud_radius = glGetUniformLocation(s_supernova_cloud_shader, "u_radius");
        s_sn_cloud_right = glGetUniformLocation(s_supernova_cloud_shader, "u_cam_right");
        s_sn_cloud_up = glGetUniformLocation(s_supernova_cloud_shader, "u_cam_up");
        s_sn_cloud_fwd = glGetUniformLocation(s_supernova_cloud_shader, "u_cam_fwd");
        s_sn_cloud_oc = glGetUniformLocation(s_supernova_cloud_shader, "u_oc");
        s_sn_cloud_color = glGetUniformLocation(s_supernova_cloud_shader, "u_color");
        s_sn_cloud_inner = glGetUniformLocation(s_supernova_cloud_shader, "u_shell_inner");
        s_sn_cloud_density = glGetUniformLocation(s_supernova_cloud_shader, "u_density");
        s_sn_cloud_hot = glGetUniformLocation(s_supernova_cloud_shader, "u_hot_shell");
        s_sn_cloud_time = glGetUniformLocation(s_supernova_cloud_shader, "u_time");
        s_sn_cloud_seed = glGetUniformLocation(s_supernova_cloud_shader, "u_seed");
        s_sn_cloud_bill = glGetUniformLocation(s_supernova_cloud_shader, "u_bill_scale");
        s_sn_cloud_fullscreen = glGetUniformLocation(s_supernova_cloud_shader, "u_fullscreen");
        s_sn_cloud_fov_tan = glGetUniformLocation(s_supernova_cloud_shader, "u_fov_tan");
        s_sn_cloud_aspect = glGetUniformLocation(s_supernova_cloud_shader, "u_aspect");
        s_sn_cloud_screen = glGetUniformLocation(s_supernova_cloud_shader, "u_screen");
    }

    /* --- Atmosphere glow shader --- */
    s_atm_shader = gl_shader_load("assets/shaders/atm.vert",
                                  "assets/shaders/atm.frag");
    if (!s_atm_shader)
        fprintf(stderr, "[Render] atm shader failed\n");
    else {
        s_at_vp        = glGetUniformLocation(s_atm_shader, "u_vp");
        s_at_center    = glGetUniformLocation(s_atm_shader, "u_center");
        s_at_radius    = glGetUniformLocation(s_atm_shader, "u_radius");
        s_at_cam_right = glGetUniformLocation(s_atm_shader, "u_cam_right");
        s_at_cam_up    = glGetUniformLocation(s_atm_shader, "u_cam_up");
        s_at_cam_fwd   = glGetUniformLocation(s_atm_shader, "u_cam_fwd");
        s_at_oc        = glGetUniformLocation(s_atm_shader, "u_oc");
        s_at_planet_r  = glGetUniformLocation(s_atm_shader, "u_planet_radius");
        s_at_sun_rel   = glGetUniformLocation(s_atm_shader, "u_sun_rel");
        s_at_color     = glGetUniformLocation(s_atm_shader, "u_atm_color");
        s_at_intensity = glGetUniformLocation(s_atm_shader, "u_atm_intensity");
        s_at_aspect    = glGetUniformLocation(s_atm_shader, "u_aspect");
        s_at_screen    = glGetUniformLocation(s_atm_shader, "u_screen");
        glUseProgram(s_atm_shader);
        glUniform1f(glGetUniformLocation(s_atm_shader, "u_fov_tan"),
                    tanf(FOV * 0.5f * (float)(PI / 180.0)));
        glUniform1f(s_at_aspect, (float)WIN_W / (float)WIN_H);
        glUniform2f(s_at_screen, (float)WIN_W, (float)WIN_H);
        glUseProgram(0);
    }

    /* --- Dot shader (shared with starfield: color.vert / color.frag) --- */
    s_dot_shader = gl_shader_load("assets/shaders/color.vert",
                                  "assets/shaders/color.frag");
    if (!s_dot_shader) {
        fprintf(stderr, "[Render] color shader failed\n");
        return;
    }
    s_dot_vp = glGetUniformLocation(s_dot_shader, "u_vp");

    /* --- Impact particle shader --- */
    s_impact_particle_shader = gl_shader_load("assets/shaders/impact_particle.vert",
                                              "assets/shaders/impact_particle.frag");
    if (!s_impact_particle_shader) {
        fprintf(stderr, "[Render] impact particle shader failed\n");
        return;
    }
    s_impact_particle_vp = glGetUniformLocation(s_impact_particle_shader, "u_vp");

    /* Dynamic dot VBO: layout = (x,y,z, r,g,b,a) × MAX_BODIES.
     * GL_DYNAMIC_DRAW since it is rebuilt every frame from live body positions. */
    s_dot_vao = gl_vao_create();
    s_dot_vbo = gl_vbo_create(MAX_BODIES * 7 * sizeof(float),
                               NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7*sizeof(float),
                          (void*)(3*sizeof(float)));
    glBindVertexArray(0);

    /* Impact particle VBO: layout = (x,y,z, r,g,b,a, size) per particle */
    s_impact_particle_vao = gl_vao_create();
    s_impact_particle_vbo = gl_vbo_create(RENDER_MAX_COLLISION_PARTICLES * 8 * sizeof(float),
                                          NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8*sizeof(float),
                          (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float),
                          (void*)(7*sizeof(float)));
    glBindVertexArray(0);

    /* --- Build-mode guide lines (world-space line segments) --- */
    s_build_line_shader = gl_shader_load("assets/shaders/build_line.vert",
                                         "assets/shaders/build_line.frag");
    if (s_build_line_shader) {
        s_build_line_vp = glGetUniformLocation(s_build_line_shader, "u_vp");
        s_build_line_vao = gl_vao_create();
        /* 3 lines × 2 endpoints × 7 floats (xyz, rgba) */
        s_build_line_vbo = gl_vbo_create(6 * 7 * sizeof(float),
                                         NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7*sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7*sizeof(float),
                              (void*)(3*sizeof(float)));
        glBindVertexArray(0);
    }

    /* --- Build-mode distance label overlay (screen-space text quads) --- */
    s_build_ui_shader = gl_shader_load("assets/shaders/ui.vert",
                                       "assets/shaders/ui.frag");
    if (s_build_ui_shader) {
        s_build_ui_screen  = glGetUniformLocation(s_build_ui_shader, "u_screen");
        s_build_ui_color   = glGetUniformLocation(s_build_ui_shader, "u_color");
        s_build_ui_use_tex = glGetUniformLocation(s_build_ui_shader, "u_use_tex");
        s_build_ui_tex     = glGetUniformLocation(s_build_ui_shader, "u_tex");
        s_build_ui_vao = gl_vao_create();
        /* Shared overlay buffer: enough for one text quad or an inspect ring. */
        s_build_ui_vbo = gl_vbo_create(96 * 2 * 4 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float),
                              (void*)(2*sizeof(float)));
        glBindVertexArray(0);
        glUseProgram(s_build_ui_shader);
        glUniform2f(s_build_ui_screen, (float)WIN_W, (float)WIN_H);
        glUniform1i(s_build_ui_tex, 0);
        glUseProgram(0);
    }
    TTF_Init();
    s_build_font = ui_theme_open_font((int)BUILD_UI_FONT_SIZE);
}

/*
 * render_build_preview — draw the build-mode ghost body and contextual overlays.
 *
 * Three overlays are drawn:
 *
 *   Ghost dot: a large (36px) GL_POINT at the preview body's world position,
 *   in the preset's color.  Drawn without depth test so it is always visible.
 *
 *   Guide lines: 3D line segments from the preview position to the 3 nearest
 *   existing bodies (build_nearest3).  Alpha decreases for more distant bodies.
 *
 *   Distance text panel: screen-space label list showing "name  distance" for
 *   each guide-line target.  Placement uses a 4-quadrant scoring system:
 *     - Try 4 candidate positions (NE, NW, SE, SW of the preview dot).
 *     - Score penalises overlap with guide lines (dot product along line
 *       direction + perpendicular proximity) and clamping to screen edges.
 *     - Choose the quadrant with the lowest score.
 *   Text cache prevents SDL_TTF texture re-renders when the string is unchanged.
 */
static void render_build_preview(const float vp_camrel[16])
{
    if (!g_build_mode) return;

    const BuildPreset *preset = build_current_preset();
    if (!preset) return;

    double preview_m[3];
    build_preview_pos_m(preview_m);

    int idx[3];
    double dist_au[3];
    build_nearest3(preview_m, idx, dist_au);

    /* Camera-relative position (double → float) for correct depth at any distance */
    float px = (float)(preview_m[0] * RS - g_cam.pos[0]);
    float py = (float)(preview_m[1] * RS - g_cam.pos[1]);
    float pz = (float)(preview_m[2] * RS - g_cam.pos[2]);

    /* Ghost dot */
    if (s_dot_shader) {
        float dot[7] = {
            px, py, pz,
            preset->col[0], preset->col[1], preset->col[2], 0.88f
        };
        glUseProgram(s_dot_shader);
        glUniformMatrix4fv(s_dot_vp, 1, GL_FALSE, vp_camrel);
        glBindVertexArray(s_dot_vao);
        glBindBuffer(GL_ARRAY_BUFFER, s_dot_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(dot), dot);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glPointSize(36.0f);
        glDrawArrays(GL_POINTS, 0, 1);
        glPointSize(1.0f);
        glDisable(GL_PROGRAM_POINT_SIZE);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glBindVertexArray(0);
    }

    /* Guide lines to nearest 3 bodies */
    if (s_build_line_shader) {
        float line_data[6 * 7];
        int v = 0;
        for (int k = 0; k < 3; k++) {
            if (idx[k] < 0) continue;
            Body *b = &g_bodies[idx[k]];
            float bx = (float)(b->pos[0] * RS - g_cam.pos[0]);
            float by = (float)(b->pos[1] * RS - g_cam.pos[1]);
            float bz = (float)(b->pos[2] * RS - g_cam.pos[2]);
            float a = 0.72f;

            line_data[v*7+0] = px; line_data[v*7+1] = py; line_data[v*7+2] = pz;
            line_data[v*7+3] = 1.0f; line_data[v*7+4] = 1.0f;
            line_data[v*7+5] = 1.0f; line_data[v*7+6] = a;
            v++;
            line_data[v*7+0] = bx; line_data[v*7+1] = by; line_data[v*7+2] = bz;
            line_data[v*7+3] = 1.0f; line_data[v*7+4] = 1.0f;
            line_data[v*7+5] = 1.0f; line_data[v*7+6] = a;
            v++;
        }
        if (v > 0) {
            glUseProgram(s_build_line_shader);
            glUniformMatrix4fv(s_build_line_vp, 1, GL_FALSE, vp_camrel);
            glBindVertexArray(s_build_line_vao);
            glBindBuffer(GL_ARRAY_BUFFER, s_build_line_vbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, v * 7 * sizeof(float), line_data);
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glLineWidth(1.5f);
            glDrawArrays(GL_LINES, 0, v);
            glLineWidth(1.0f);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            glBindVertexArray(0);
        }
    }

    /* Screen-space distance labels beside the preview.
     * 4-quadrant placement: try NE, NW, SE, SW; choose lowest-scoring position.
     * Score = clamping penalty + guide-line overlap penalty. */
    if (s_build_ui_shader && s_build_font) {
        glUseProgram(s_build_ui_shader);
        glUniform2f(s_build_ui_screen, (float)WIN_W, (float)WIN_H);
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(s_build_ui_vao);
        glBindBuffer(GL_ARRAY_BUFFER, s_build_ui_vbo);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        float psx = 0.0f, psy = 0.0f;
        int preview_on_screen = mat4_project(vp_camrel, px, py, pz, WIN_W, WIN_H, &psx, &psy);
        float preview_y = (float)WIN_H - psy;
        int active[3] = {0, 0, 0};
        float text_w[3] = {0.0f, 0.0f, 0.0f};
        float dir_x[3] = {0.0f, 0.0f, 0.0f};
        float dir_y[3] = {0.0f, 0.0f, 0.0f};
        float max_w = 0.0f;
        int n_active = 0;

        /* Project guide-line endpoints to screen; compute screen-space directions */
        for (int k = 0; k < 3; k++) {
            if (idx[k] < 0 || !preview_on_screen) continue;
            Body *b = &g_bodies[idx[k]];
            float bx = (float)(b->pos[0] * RS - g_cam.pos[0]);
            float by = (float)(b->pos[1] * RS - g_cam.pos[1]);
            float bz = (float)(b->pos[2] * RS - g_cam.pos[2]);
            float bsx = 0.0f, bsy = 0.0f;
            int body_on_screen = mat4_project(vp_camrel, bx, by, bz, WIN_W, WIN_H, &bsx, &bsy);

            char buf[96];
            char dist_buf[32];
            body_format_dist_au(dist_au[k], dist_buf, sizeof(dist_buf));
            snprintf(buf, sizeof(buf), "%s  %s", b->name, dist_buf);
            build_update_text(&s_build_dist_text[k], buf);
            if (!s_build_dist_text[k].tex) continue;

            if (body_on_screen) {
                float body_y = (float)WIN_H - bsy;
                dir_x[k] = bsx - psx;
                dir_y[k] = body_y - preview_y;
            } else {
                dir_x[k] = 1.0f;
                dir_y[k] = 0.0f;
            }
            {
                float dl = sqrtf(dir_x[k]*dir_x[k] + dir_y[k]*dir_y[k]);
                if (dl < 1.0f) { dir_x[k] = 1.0f; dir_y[k] = 0.0f; dl = 1.0f; }
                dir_x[k] /= dl;
                dir_y[k] /= dl;
            }

            text_w[k] = BUILD_UI_FONT_SIZE * (float)s_build_dist_text[k].w
                      / (float)s_build_dist_text[k].h;
            if (text_w[k] > max_w) max_w = text_w[k];
            active[k] = 1;
            n_active++;
        }

        if (n_active > 0) {
            float row_h = BUILD_UI_FONT_SIZE + 4.0f;
            float list_h = row_h * (float)n_active;
            float margin_x = 42.0f;
            float margin_y = 28.0f;
            float best_score = 1e30f;
            float list_x = psx + margin_x;
            float list_y = preview_y - margin_y - list_h;
            int side = 1;

            /* Test 4 candidate positions (q=0: upper-right, 1: upper-left,
             * 2: lower-right, 3: lower-left) and pick the lowest-scoring one */
            for (int q = 0; q < 4; q++) {
                int sx = (q == 0 || q == 3) ? 1 : -1;
                int sy = (q == 0 || q == 1) ? -1 : 1;
                float cx = (sx > 0) ? psx + margin_x : psx - margin_x - max_w;
                float cy = (sy < 0) ? preview_y - margin_y - list_h
                                     : preview_y + margin_y;
                float clamped_x = clampf(cx, 8.0f, (float)WIN_W - max_w - 8.0f);
                float clamped_y = clampf(cy, 8.0f, (float)WIN_H - list_h - 8.0f);
                float center_x = clamped_x + max_w * 0.5f;
                float center_y = clamped_y + list_h * 0.5f;
                float vx = center_x - psx;
                float vy = center_y - preview_y;
                float vl = sqrtf(vx*vx + vy*vy);
                /* Clamping penalty: 8× the pixel distance we had to move */
                float score = fabsf(clamped_x - cx) * 8.0f
                            + fabsf(clamped_y - cy) * 8.0f;
                if (vl < 1.0f) vl = 1.0f;
                vx /= vl;
                vy /= vl;

                /* Guide-line overlap penalty for each active label */
                for (int k = 0; k < 3; k++) {
                    if (!active[k]) continue;
                    float dot = vx * dir_x[k] + vy * dir_y[k];
                    if (dot > 0.0f) score += dot * dot * 120.0f;
                    {
                        float raw_x = center_x - psx;
                        float raw_y = center_y - preview_y;
                        float along = raw_x * dir_x[k] + raw_y * dir_y[k];
                        float perp = fabsf(raw_x * dir_y[k] - raw_y * dir_x[k]);
                        if (along > 0.0f && perp < 48.0f)
                            score += (48.0f - perp) * 2.5f;
                    }
                }

                if (score < best_score) {
                    best_score = score;
                    list_x = clamped_x;
                    list_y = clamped_y;
                    side = sx;
                }
            }

            int row = 0;
            for (int k = 0; k < 3; k++) {
                if (!active[k]) continue;
                float x = (side > 0) ? list_x : list_x + max_w - text_w[k];
                build_draw_text(&s_build_dist_text[k], x, list_y + row_h * (float)row,
                                BUILD_UI_FONT_SIZE);
                row++;
            }
        }

        glBindVertexArray(0);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }
}

/* ------------------------------------------------------------------ frame */
void render_frame(const float view[16], const float proj[16],
                  const float view_rot[16], float dt) {
    float aspect = (float)WIN_W / (float)WIN_H;

    /* Full VP: proj × view (with translation).  Used for rings (nearby geometry). */
    Mat4 vp;
    mat4_mul(vp, proj, view);

    /* Camera-relative VP: proj × view_rot (rotation only, no translation).
     * Used for all far-field geometry (trails, dots, spheres, labels, glare)
     * to eliminate float32 cancellation at large world-space offsets. */
    Mat4 vp_camrel;
    mat4_mul(vp_camrel, proj, view_rot);

    /* Camera basis vectors extracted from view_rot (not the full view matrix).
     * Using the full view matrix here would reintroduce float cancellation —
     * view_rot is kept as a separate pure-rotation matrix for exactly this reason. */
    Vec3 cam_right, cam_up, cam_fwd;
    mat4_get_right(view_rot, cam_right);
    mat4_get_up   (view_rot, cam_up);
    mat4_get_fwd  (view_rot, cam_fwd);

    SupernovaRenderEvent sn_events[SUPERNOVA_MAX_EVENTS];
    int sn_count = supernova_render_events(sn_events, SUPERNOVA_MAX_EVENTS, g_cam.pos);

    /* Sun world position in AU units — used as lighting reference only */
    float sun_wx = (float)(g_bodies[0].pos[0] * RS);
    float sun_wy = (float)(g_bodies[0].pos[1] * RS);
    float sun_wz = (float)(g_bodies[0].pos[2] * RS);

    /* ------------------------------------------------------------------ 1. Starfield */
    /* Stars are on the unit sphere (depth ~= far plane).  Depth test would
     * Z-fight with the cleared depth buffer, so it is disabled for the skybox. */
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    starfield_render(view_rot, proj);
    glDepthMask(GL_TRUE);

    /* ------------------------------------------------------------------ 2. Spheres */
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    glUseProgram(s_sphere_shader);
    /* vp_camrel: body centre will be passed as camera-relative (u_center = -u_oc)
     * so all vertex positions in phong.vert are free of float32 cancellation. */
    glUniformMatrix4fv(s_sp_vp,       1, GL_FALSE, vp_camrel);
    glUniform1f(s_sp_aspect, aspect);
    glUniform2f(s_sp_screen, (float)WIN_W, (float)WIN_H);
    glUniform3f(s_sp_sun_world,  sun_wx, sun_wy, sun_wz);
    glUniform3f(s_sp_cam_right,  cam_right[0], cam_right[1], cam_right[2]);
    glUniform3f(s_sp_cam_up,     cam_up[0],    cam_up[1],    cam_up[2]);
    glUniform3f(s_sp_cam_fwd,    cam_fwd[0],   cam_fwd[1],   cam_fwd[2]);

    glBindVertexArray(s_sphere_vao);

    /* BodyRenderInfo[] is also consumed by labels_render() at the end of the frame */
    BodyRenderInfo info[MAX_BODIES];
    float body_px[MAX_BODIES];   /* projected radius in pixels; used for fade thresholds */
    memset(body_px, 0, sizeof(body_px));

    for (int i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        if (!b->alive) {
            memset(&info[i], 0, sizeof(info[i]));
            info[i].show = 1;
            continue;
        }

        float wx = (float)(b->pos[0] * RS);
        float wy = (float)(b->pos[1] * RS);
        float wz = (float)(b->pos[2] * RS);

        /* Camera-relative vector in double precision.
         * Subtraction must happen in double (not float) to retain precision
         * at large world-space offsets (Proxima b is ~241,000 AU from Sol). */
        double dxd = b->pos[0] * RS - g_cam.pos[0];
        double dyd = b->pos[1] * RS - g_cam.pos[1];
        double dzd = b->pos[2] * RS - g_cam.pos[2];
        float dcam = (float)sqrt(dxd*dxd + dyd*dyd + dzd*dzd);

        float dr = visual_radius(i, dcam);

        info[i].pos[0] = wx;
        info[i].pos[1] = wy;
        info[i].pos[2] = wz;
        info[i].dr     = dr;
        info[i].dcam   = dcam;

        /* When the camera is close to the body (within 4 radii), the billboard
         * degenerates at oblique view angles — the sphere centre projects near
         * the camera plane (eye_z ≈ 0) and perspective division produces NaN/Inf.
         * Switch to a fullscreen quad in that range; the fragment shader does
         * the per-pixel ray-sphere intersection independently of vertex layout
         * and produces the correct silhouette from any view direction. */
        int use_fullscreen = (dcam < dr * 4.0f);

        /* eye_z: forward-axis depth.  Used for the dot/sphere pixel-size test.
         * Prefer eye_z over dcam because dcam = eye_z/cos(θ) oscillates with
         * camera rotation; eye_z is invariant under pure rotation. */
        float eye_z = (float)(dxd*cam_fwd[0] + dyd*cam_fwd[1] + dzd*cam_fwd[2]);
        float px;
        if (use_fullscreen) {
            /* Suppress the dot (large px) and force the sphere to render below. */
            px = 1000.0f;
        } else {
            px = (eye_z > 0.0f)
                 ? (WIN_H / 2.0f) * dr / (eye_z * half_fov_tan() + 1e-9f)
                 : 0.0f;
        }
        /* Threshold: sphere first appears when its diameter (2×px) equals the dot
         * diameter (2.5px), i.e. px = 1.25 = BODY_SPHERE_APPEAR_PX. */
        body_px[i]   = px;
        info[i].show = (!use_fullscreen && px < BODY_SPHERE_APPEAR_PX) ? 1 : 0;

        if (!g_bodies[i].alive || info[i].show) continue;
        if (b->is_star) continue;   /* stars rendered as glare only, not Phong spheres */

        /* u_oc: camera − body, computed in double then cast to float.
         * u_center = −u_oc so the phong.vert billboard is camera-relative. */
        double cam_mx = (double)g_cam.pos[0] * AU;
        double cam_my = (double)g_cam.pos[1] * AU;
        double cam_mz = (double)g_cam.pos[2] * AU;
        float oc_x = (float)((cam_mx - b->pos[0]) * RS);
        float oc_y = (float)((cam_my - b->pos[1]) * RS);
        float oc_z = (float)((cam_mz - b->pos[2]) * RS);

        /* Lighting: find the root star of this body (its owning stellar system).
         * Walk the parent chain so Proxima b is lit by Proxima Centauri, not Sol. */
        int ls = i;
        while (g_bodies[ls].parent >= 0) ls = g_bodies[ls].parent;
        float sr_x = (float)((g_bodies[ls].pos[0] - b->pos[0]) * RS);
        float sr_y = (float)((g_bodies[ls].pos[1] - b->pos[1]) * RS);
        float sr_z = (float)((g_bodies[ls].pos[2] - b->pos[2]) * RS);

        glUniform3f(s_sp_center,  -oc_x, -oc_y, -oc_z);
        glUniform1f(s_sp_radius,   dr);
        glUniform3f(s_sp_oc,       oc_x, oc_y, oc_z);
        glUniform3f(s_sp_sun_rel,  sr_x, sr_y, sr_z);
        glUniform3f(s_sp_color,    b->col[0], b->col[1], b->col[2]);
        glUniform1f(s_sp_emission, b->is_star ? 1.0f : 0.0f);
        glUniform1f(s_sp_ambient,  b->is_star ? 1.0f : 0.05f);
        glUniform1f(s_sp_rotation,        (float)fmod(b->rotation_angle, 2.0 * PI));
        glUniform1f(s_sp_cloud_rotation,  (float)b->cloud_rotation);
        glUniform1f(s_sp_obliquity, (float)(b->obliquity * (PI / 180.0)));
        glUniform1i(s_sp_ptype,     get_planet_type(b->name));
        glUniform1f(s_sp_star_heat, collision_body_star_heat(i));

        /* Upload per-body collision impact spots (craters/ejecta) for the shader */
        {
            CollisionSpot spots[COLLISION_MAX_SPOTS];
            float dirs[COLLISION_MAX_SPOTS * 3] = {0};
            float tangents[COLLISION_MAX_SPOTS * 3] = {0};
            float radii[COLLISION_MAX_SPOTS] = {0};
            float heats[COLLISION_MAX_SPOTS] = {0};
            float progress[COLLISION_MAX_SPOTS] = {0};
            float seeds[COLLISION_MAX_SPOTS] = {0};
            int kinds[COLLISION_MAX_SPOTS] = {0};
            int nspots = collision_spots_for_body(i, spots);
            for (int k = 0; k < nspots; k++) {
                dirs[k*3+0] = spots[k].dir[0];
                dirs[k*3+1] = spots[k].dir[1];
                dirs[k*3+2] = spots[k].dir[2];
                tangents[k*3+0] = spots[k].tangent1[0];
                tangents[k*3+1] = spots[k].tangent1[1];
                tangents[k*3+2] = spots[k].tangent1[2];
                radii[k] = spots[k].angular_radius;
                heats[k] = spots[k].heat;
                progress[k] = spots[k].progress;
                seeds[k] = spots[k].seed;
                kinds[k] = spots[k].kind;
            }
            glUniform1i(s_sp_impact_count, nspots);
            glUniform3fv(s_sp_impact_dir, nspots, dirs);
            glUniform3fv(s_sp_impact_t1, nspots, tangents);
            glUniform1fv(s_sp_impact_rad, nspots, radii);
            glUniform1fv(s_sp_impact_heat, nspots, heats);
            glUniform1fv(s_sp_impact_prog, nspots, progress);
            glUniform1fv(s_sp_impact_seed, nspots, seeds);
            glUniform1iv(s_sp_impact_kind, nspots, kinds);
        }

        glUniform1i(s_sp_use_fullscreen, use_fullscreen);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }
    glBindVertexArray(0);

    inspect_pick_center(vp_camrel, info);

    /* ------------------------------------------------------------------ 2.5. Atmosphere glow
     * Separate additive pass: GL_SRC_ALPHA / GL_ONE, depth-tested but no depth write.
     * The glow billboard radius is planet_radius × atm_scale.
     * Collision heat glow from the collision system is blended additively with the
     * base atmosphere color using a weighted average (intensity as weight). */
    if (s_atm_shader) {
        glUseProgram(s_atm_shader);
        glUniform1f(s_at_aspect, aspect);
        glUniform2f(s_at_screen, (float)WIN_W, (float)WIN_H);
        glUniformMatrix4fv(s_at_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_at_cam_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_at_cam_up,    cam_up[0],    cam_up[1],    cam_up[2]);
        glUniform3f(s_at_cam_fwd,   cam_fwd[0],   cam_fwd[1],   cam_fwd[2]);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);   /* additive: glows accumulate */
        glDepthMask(GL_FALSE);               /* read depth for behind-planet cull */
        glEnable(GL_DEPTH_TEST);
        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive) continue;
            if (info[i].show) continue;   /* sub-pixel body, skip glow */
            float intensity = g_bodies[i].atm_intensity;
            float scale     = g_bodies[i].atm_scale;
            float glow_color[3];
            float glow_intensity = 0.0f;
            float glow_scale = 1.0f;
            float final_color[3];
            float final_intensity;
            float final_scale;
            collision_body_heat_glow(i, glow_color, &glow_intensity, &glow_scale);
            final_intensity = intensity + glow_intensity;
            final_scale = glow_scale > scale ? glow_scale : scale;
            if (final_intensity <= 0.0f) continue;
            {
                /* Weighted average of base atm color and heat glow color */
                float base_w = intensity;
                float glow_w = glow_intensity;
                float sum_w = base_w + glow_w;
                if (sum_w <= 1e-6f) sum_w = 1.0f;
                final_color[0] = (g_bodies[i].atm_color[0] * base_w + glow_color[0] * glow_w) / sum_w;
                final_color[1] = (g_bodies[i].atm_color[1] * base_w + glow_color[1] * glow_w) / sum_w;
                final_color[2] = (g_bodies[i].atm_color[2] * base_w + glow_color[2] * glow_w) / sum_w;
            }

            Body *b = &g_bodies[i];
            double cam_mx = (double)g_cam.pos[0] * AU;
            double cam_my = (double)g_cam.pos[1] * AU;
            double cam_mz = (double)g_cam.pos[2] * AU;
            float oc_x = (float)((cam_mx - b->pos[0]) * RS);
            float oc_y = (float)((cam_my - b->pos[1]) * RS);
            float oc_z = (float)((cam_mz - b->pos[2]) * RS);
            float sr_x, sr_y, sr_z;
            {
                int ls = i;
                while (g_bodies[ls].parent >= 0) ls = g_bodies[ls].parent;
                sr_x = (float)((g_bodies[ls].pos[0] - b->pos[0]) * RS);
                sr_y = (float)((g_bodies[ls].pos[1] - b->pos[1]) * RS);
                sr_z = (float)((g_bodies[ls].pos[2] - b->pos[2]) * RS);
            }

            float planet_r = info[i].dr;
            float atm_r    = planet_r * final_scale;

            glUniform3f(s_at_center,   -oc_x, -oc_y, -oc_z);
            glUniform1f(s_at_radius,    atm_r);
            glUniform1f(s_at_planet_r,  planet_r);
            glUniform3f(s_at_oc,        oc_x, oc_y, oc_z);
            glUniform3f(s_at_sun_rel,   sr_x, sr_y, sr_z);
            glUniform3f(s_at_color,     final_color[0], final_color[1], final_color[2]);
            glUniform1f(s_at_intensity, final_intensity);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }

        glBindVertexArray(0);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    /* ------------------------------------------------------------------ 2.75. Supernova cloud + core */
    if (sn_count > 0 && s_supernova_cloud_shader) {
        glUseProgram(s_supernova_cloud_shader);
        glUniformMatrix4fv(s_sn_cloud_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_sn_cloud_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_sn_cloud_up, cam_up[0], cam_up[1], cam_up[2]);
        glUniform3f(s_sn_cloud_fwd, cam_fwd[0], cam_fwd[1], cam_fwd[2]);
        glUniform1f(s_sn_cloud_fov_tan, tanf(FOV * 0.5f * (float)(PI / 180.0)));
        glUniform1f(s_sn_cloud_aspect, aspect);
        glUniform2f(s_sn_cloud_screen, (float)WIN_W, (float)WIN_H);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glEnable(GL_DEPTH_TEST);
        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < sn_count; i++) {
            const SupernovaRenderEvent *e = &sn_events[i];
            float cloud_bill_scale = 1.34f;
            float dist;
            float dist_fade;
            float cloud_density;
            int fullscreen_raster;
            if (e->cloud_intensity <= 0.00005f || e->cloud_radius <= 0.0f) continue;

            dist = sqrtf(e->pos[0]*e->pos[0] + e->pos[1]*e->pos[1] + e->pos[2]*e->pos[2]);
            dist_fade = supernova_distance_fade_local(dist, e->cloud_radius);
            cloud_density = e->cloud_intensity * dist_fade;
            if (cloud_density <= 0.00005f) continue;

            fullscreen_raster = supernova_fullscreen_raster_local(e->pos, cam_fwd,
                                                                  e->cloud_radius,
                                                                  cloud_bill_scale)
                             || supernova_far_raster_local(e->pos, cam_fwd,
                                                           e->cloud_radius,
                                                           cloud_bill_scale);
            glUniform1f(s_sn_cloud_fullscreen, fullscreen_raster ? 1.0f : 0.0f);
            glUniform3f(s_sn_cloud_center, e->pos[0], e->pos[1], e->pos[2]);
            glUniform1f(s_sn_cloud_radius, e->cloud_radius);
            glUniform3f(s_sn_cloud_oc, -e->pos[0], -e->pos[1], -e->pos[2]);
            glUniform3f(s_sn_cloud_color, e->color[0], e->color[1], e->color[2]);
            glUniform1f(s_sn_cloud_inner,
                        e->cloud_radius > 1e-6f ? e->cloud_inner_radius / e->cloud_radius : 0.72f);
            glUniform1f(s_sn_cloud_density, cloud_density);
            glUniform1f(s_sn_cloud_hot, e->hot_shell_intensity * dist_fade);
            glUniform1f(s_sn_cloud_time, e->time_days);
            glUniform1f(s_sn_cloud_seed, e->seed);
            glUniform1f(s_sn_cloud_bill, cloud_bill_scale);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }

        glBindVertexArray(0);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    if (sn_count > 0 && s_supernova_core_shader) {
        glUseProgram(s_supernova_core_shader);
        glUniformMatrix4fv(s_sn_core_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_sn_core_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_sn_core_up, cam_up[0], cam_up[1], cam_up[2]);
        glUniform3f(s_sn_core_fwd, cam_fwd[0], cam_fwd[1], cam_fwd[2]);
        glUniform1f(s_sn_core_fov_tan, tanf(FOV * 0.5f * (float)(PI / 180.0)));
        glUniform1f(s_sn_core_aspect, aspect);
        glUniform2f(s_sn_core_screen, (float)WIN_W, (float)WIN_H);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);
        glEnable(GL_DEPTH_TEST);
        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < sn_count; i++) {
            const SupernovaRenderEvent *e = &sn_events[i];
            float dist = sqrtf(e->pos[0]*e->pos[0] + e->pos[1]*e->pos[1] + e->pos[2]*e->pos[2]);
            float dist_fade;
            float core_bill_scale;
            int fullscreen_raster;
            float coverage_radius;
            float flash_intensity;
            float core_intensity;

            if (e->flash_intensity <= 0.00005f && e->core_intensity <= 0.00005f)
                continue;

            dist_fade = supernova_distance_fade_local(dist,
                        e->cloud_radius > e->flash_radius ? e->cloud_radius : e->flash_radius);
            flash_intensity = e->flash_intensity * dist_fade;
            core_intensity = e->core_intensity * dist_fade;
            if (flash_intensity <= 0.00005f && core_intensity <= 0.00005f)
                continue;

            if (dist > e->flash_radius * 1.01f) {
                float denom = sqrtf(fmaxf(dist * dist - e->flash_radius * e->flash_radius, 1e-6f));
                core_bill_scale = dist / denom;
            } else {
                core_bill_scale = 8.0f;
            }
            core_bill_scale = clampf(core_bill_scale * 1.10f, 1.18f, 8.0f);
            coverage_radius = e->cloud_radius > e->flash_radius ? e->cloud_radius : e->flash_radius;
            fullscreen_raster = supernova_fullscreen_raster_local(e->pos, cam_fwd,
                                                                  coverage_radius,
                                                                  core_bill_scale)
                             || supernova_far_raster_local(e->pos, cam_fwd,
                                                           coverage_radius,
                                                           core_bill_scale);
            glUniform1f(s_sn_core_fullscreen, fullscreen_raster ? 1.0f : 0.0f);
            glUniform3f(s_sn_core_center, e->pos[0], e->pos[1], e->pos[2]);
            glUniform1f(s_sn_core_radius, e->flash_radius);
            glUniform3f(s_sn_core_oc, -e->pos[0], -e->pos[1], -e->pos[2]);
            glUniform3f(s_sn_core_color, e->color[0], e->color[1], e->color[2]);
            glUniform1f(s_sn_core_flash, flash_intensity);
            glUniform1f(s_sn_core_core, core_intensity);
            glUniform1f(s_sn_core_ratio,
                        e->flash_radius > 1e-6f ? e->core_radius / e->flash_radius : 0.32f);
            glUniform1f(s_sn_core_time, e->time_days);
            glUniform1f(s_sn_core_seed, e->seed);
            glUniform1f(s_sn_core_bill, core_bill_scale);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }

        glBindVertexArray(0);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    /* ------------------------------------------------------------------ 3. Collision particles */
    {
        CollisionParticle particles[RENDER_MAX_COLLISION_PARTICLES];
        float particle_data[RENDER_MAX_COLLISION_PARTICLES * 8];
        int particle_count = collision_particles(particles, RENDER_MAX_COLLISION_PARTICLES,
                                                 g_cam.pos);

        for (int i = 0; i < particle_count; i++) {
            particle_data[i*8+0] = particles[i].pos[0];
            particle_data[i*8+1] = particles[i].pos[1];
            particle_data[i*8+2] = particles[i].pos[2];
            particle_data[i*8+3] = particles[i].color[0];
            particle_data[i*8+4] = particles[i].color[1];
            particle_data[i*8+5] = particles[i].color[2];
            particle_data[i*8+6] = particles[i].color[3];
            particle_data[i*8+7] = particles[i].size;
        }

        if (particle_count > 0) {
            glUseProgram(s_impact_particle_shader);
            glUniformMatrix4fv(s_impact_particle_vp, 1, GL_FALSE, vp_camrel);
            glBindVertexArray(s_impact_particle_vao);
            glBindBuffer(GL_ARRAY_BUFFER, s_impact_particle_vbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            particle_count * 8 * sizeof(float), particle_data);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);   /* additive ejecta */
            glDepthMask(GL_FALSE);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_PROGRAM_POINT_SIZE);
            glDrawArrays(GL_POINTS, 0, particle_count);
            glDisable(GL_PROGRAM_POINT_SIZE);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glBindVertexArray(0);
        }
    }

    /* ------------------------------------------------------------------ 3. Center dots
     *
     * Priority order for overlap resolution: stars first, then planets (sorted
     * by dcam), then moons (sorted by dcam).  Stars must beat planets so that
     * at interstellar distances (e.g. Proxima b nearly coincides with Proxima
     * Centauri on screen from Sol) the star dot reliably wins the screen slot.
     *
     * Overlap removal is greedy: iterate in priority order; a dot is visible
     * only if its screen position is ≥ DOT_EXCL_PX away from all previously
     * confirmed visible dots.  Between DOT_HIDE_PX and DOT_EXCL_PX the alpha
     * is smoothstep-faded so the handoff is gradual rather than binary.
     *
     * IMPORTANT: positions are computed as camera-relative doubles cast to
     * float, used with vp_camrel (no translation).  Using the full float VP
     * with float world positions loses 4-5 digits at 4+ ly → visible jitter. */
#define DOT_EXCL_PX 6.0f   /* fully separated below this screen distance */
#define DOT_HIDE_PX 2.5f   /* fully hidden when centers are this close */

    int dot_order[MAX_BODIES];
    int dot_ns = 0, dot_np = 0, dot_nm = 0;
    int dot_stars[MAX_BODIES], dot_planets[MAX_BODIES], dot_moons[MAX_BODIES];
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        if      (g_bodies[i].is_star)       dot_stars  [dot_ns++] = i;
        else if (g_bodies[i].parent < 0 ||
                 g_bodies[g_bodies[i].parent].is_star) dot_planets[dot_np++] = i;
        else                                           dot_moons  [dot_nm++] = i;
    }
    /* Insertion sort within each tier by dcam (ascending = nearest first) */
    for (int i = 1; i < dot_ns; i++) {
        int t = dot_stars[i], k = i;
        while (k > 0 && info[dot_stars[k-1]].dcam > info[t].dcam)
            { dot_stars[k] = dot_stars[k-1]; k--; }
        dot_stars[k] = t;
    }
    for (int i = 1; i < dot_np; i++) {
        int t = dot_planets[i], k = i;
        while (k > 0 && info[dot_planets[k-1]].dcam > info[t].dcam)
            { dot_planets[k] = dot_planets[k-1]; k--; }
        dot_planets[k] = t;
    }
    for (int i = 1; i < dot_nm; i++) {
        int t = dot_moons[i], k = i;
        while (k > 0 && info[dot_moons[k-1]].dcam > info[t].dcam)
            { dot_moons[k] = dot_moons[k-1]; k--; }
        dot_moons[k] = t;
    }
    /* Concatenate tiers into priority order */
    for (int i = 0; i < dot_ns; i++) dot_order[i]                  = dot_stars[i];
    for (int i = 0; i < dot_np; i++) dot_order[dot_ns + i]          = dot_planets[i];
    for (int i = 0; i < dot_nm; i++) dot_order[dot_ns + dot_np + i] = dot_moons[i];
    int dot_total = dot_ns + dot_np + dot_nm;

    /* Greedy overlap pass — project each candidate dot and test against confirmed dots */
    float dot_sx[MAX_BODIES], dot_sy[MAX_BODIES];
    float dot_overlap_alpha[MAX_BODIES];
    int   dot_candidate[MAX_BODIES];
    int   dot_vis[MAX_BODIES];
    memset(dot_overlap_alpha, 0, sizeof(dot_overlap_alpha));
    memset(dot_candidate, 0, sizeof(dot_candidate));
    memset(dot_vis, 0, sizeof(dot_vis));

    {
        const float DOT_CLAMP_DIST_OV = 1500.0f;
        double cx2 = g_cam.pos[0];
        double cy2 = g_cam.pos[1];
        double cz2 = g_cam.pos[2];

        for (int oi = 0; oi < dot_total; oi++) {
            int i = dot_order[oi];
            if (!g_bodies[i].alive) continue;
            /* Pre-filter: fully occluded by a star's glare corona */
            if (body_point_star_glare_visibility(i) <= 0.02f) continue;
            /* Pre-filter: star has grown large enough that its glare disc replaces its dot */
            if (g_bodies[i].is_star &&
                body_px[i] * STAR_GLARE_BILL_SCALE >= STAR_DOT_FADE_START_GLARE_PX) continue;
            /* Pre-filter: non-star body is large enough to be rendered as a sphere */
            if (!g_bodies[i].is_star && body_px[i] >= BODY_DOT_FADE_END_PX)
                continue;
            if (body_point_occluded_by_body(i, info)) continue;
            Body *bi = &g_bodies[i];

            float rx = (float)(bi->pos[0] * RS - cx2);
            float ry = (float)(bi->pos[1] * RS - cy2);
            float rz = (float)(bi->pos[2] * RS - cz2);
            /* Clamp distant stars to near sphere so they remain on-screen at warp */
            if (bi->is_star) {
                float d = sqrtf(rx*rx + ry*ry + rz*rz);
                if (d > DOT_CLAMP_DIST_OV && d > 1e-6f) {
                    float s = DOT_CLAMP_DIST_OV / d;
                    rx *= s; ry *= s; rz *= s;
                }
            }

            float sx, sy;
            if (!mat4_project(vp_camrel, rx, ry, rz, WIN_W, WIN_H, &sx, &sy)) continue;
            dot_candidate[i] = 1;

            /* Measure nearest confirmed-visible dot on screen */
            float overlap_alpha = 1.0f;
            float nearest2 = DOT_EXCL_PX * DOT_EXCL_PX;
            for (int oj = 0; oj < oi; oj++) {
                int j = dot_order[oj];
                if (!dot_vis[j]) continue;
                float dx = sx - dot_sx[j], dy = sy - dot_sy[j];
                float d2 = dx*dx + dy*dy;
                if (d2 < nearest2) nearest2 = d2;
            }

            if (nearest2 < DOT_EXCL_PX * DOT_EXCL_PX) {
                float d = sqrtf(nearest2);
                overlap_alpha = (float)smoothstepd(DOT_HIDE_PX, DOT_EXCL_PX, d);
            }

            dot_overlap_alpha[i] = overlap_alpha;
            /* A dot claims its slot only if fully opaque (nearest competitor is far) */
            if (overlap_alpha >= 0.999f) {
                dot_sx[i]  = sx;
                dot_sy[i]  = sy;
                dot_vis[i] = 1;
            }
        }
    }

    /* Build the GPU upload buffer for surviving dots.
     * Stars beyond DOT_CLAMP_DIST are clamped to remain renderable at warp speed.
     * Alpha = system_dot_fade × overlap_fade × glare_visibility × sphere-approach fade. */
    float dot_data[MAX_BODIES * 7];
    int   dot_count = 0;
    {
        double cx = g_cam.pos[0];
        double cy = g_cam.pos[1];
        double cz = g_cam.pos[2];
        const float DOT_CLAMP_DIST = 1500.0f;

        for (int oi = 0; oi < dot_total; oi++) {
            int i = dot_order[oi];
            if (!g_bodies[i].alive) continue;
            if (!dot_candidate[i] || dot_overlap_alpha[i] <= 0.001f) continue;
            Body *b = &g_bodies[i];

            float f = b->is_star ? 1.0f : system_dot_fade_for_body(i);
            f *= dot_overlap_alpha[i];
            f *= body_point_star_glare_visibility(i);

            /* Star: fade dot as glare billboard grows large enough to represent it */
            if (b->is_star) {
                float glare_px = body_px[i] * STAR_GLARE_BILL_SCALE;
                if (glare_px > STAR_DOT_FULL_GLARE_PX) {
                    float t = (glare_px - STAR_DOT_FULL_GLARE_PX)
                            / (STAR_DOT_FADE_START_GLARE_PX - STAR_DOT_FULL_GLARE_PX);
                    if (t > 1.0f) t = 1.0f;
                    f *= 1.0f - t;
                }
            }
            if (f <= 0.0f) continue;

            /* Non-star: overlap the dot with the first sphere pixels during the transition */
            float px_i = body_px[i];
            if (!b->is_star && px_i > BODY_DOT_FADE_START_PX) {
                float t = (px_i - BODY_DOT_FADE_START_PX)
                        / (BODY_DOT_FADE_END_PX - BODY_DOT_FADE_START_PX);
                if (t > 1.0f) t = 1.0f;
                f *= 1.0f - t;
            }
            if (f <= 0.0f) continue;

            float bx = (float)(b->pos[0] * RS - cx);
            float by = (float)(b->pos[1] * RS - cy);
            float bz = (float)(b->pos[2] * RS - cz);
            /* Clamp star position so it stays within the far plane */
            if (b->is_star) {
                float d = sqrtf(bx*bx + by*by + bz*bz);
                if (d > DOT_CLAMP_DIST && d > 1e-6f) {
                    float s = DOT_CLAMP_DIST / d;
                    bx *= s; by *= s; bz *= s;
                }
            }
            dot_data[dot_count*7+0] = bx;
            dot_data[dot_count*7+1] = by;
            dot_data[dot_count*7+2] = bz;
            dot_data[dot_count*7+3] = b->col[0];
            dot_data[dot_count*7+4] = b->col[1];
            dot_data[dot_count*7+5] = b->col[2];
            dot_data[dot_count*7+6] = f;
            dot_count++;
        }
    }

    if (dot_count > 0) {
        glUseProgram(s_dot_shader);
        glUniformMatrix4fv(s_dot_vp, 1, GL_FALSE, vp_camrel);
        glBindVertexArray(s_dot_vao);
        glBindBuffer(GL_ARRAY_BUFFER, s_dot_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        dot_count * 7 * sizeof(float), dot_data);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glPointSize(2.5f);
        glDrawArrays(GL_POINTS, 0, dot_count);
        glPointSize(1.0f);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glBindVertexArray(0);
    }

    /* ------------------------------------------------------------------ 4. Rings + Asteroid belts */
    rings_render(vp_camrel);
    asteroids_render(vp_camrel);

    /* ------------------------------------------------------------------ 5. Trails */
    trails_render(vp_camrel);

    /* ------------------------------------------------------------------ 6. Star glare
     * Drawn after trails so stellar corona covers orbit lines inside the glow disc.
     * Additive blend (GL_ONE / GL_ONE) accumulates glow from multiple stars.
     * Depth-tested (GL_LEQUAL) so glare is eclipsed by foreground opaque bodies.
     * Stars beyond GLARE_MAX_DIST are scaled toward the camera so the billboards
     * remain within the view frustum and glare remains visible at warp distances. */
    if (s_glare_shader) {
        const float GLARE_MAX_DIST = 1500.0f;

        glUseProgram(s_glare_shader);
        glUniformMatrix4fv(s_gl_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_gl_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_gl_up,    cam_up[0],    cam_up[1],    cam_up[2]);

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);   /* purely additive */
        glDepthMask(GL_FALSE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive) continue;
            if (!g_bodies[i].is_star) continue;

            float rx = (float)(g_bodies[i].pos[0] * RS - g_cam.pos[0]);
            float ry = (float)(g_bodies[i].pos[1] * RS - g_cam.pos[1]);
            float rz = (float)(g_bodies[i].pos[2] * RS - g_cam.pos[2]);

            /* Skip stars behind the camera */
            if (rx*cam_fwd[0] + ry*cam_fwd[1] + rz*cam_fwd[2] < 0.0f) continue;

            float dist   = sqrtf(rx*rx + ry*ry + rz*rz);
            float radius = (float)(g_bodies[i].radius * RS);
            /* Scale down glare for very distant stars so it stays on-screen */
            if (dist > GLARE_MAX_DIST && dist > 1e-6f) {
                float s = GLARE_MAX_DIST / dist;
                rx *= s; ry *= s; rz *= s;
                radius *= s;
            }

            glUniform3f(s_gl_center, rx, ry, rz);
            glUniform1f(s_gl_radius, radius);
            glUniform3f(s_gl_color,
                        g_bodies[i].col[0], g_bodies[i].col[1], g_bodies[i].col[2]);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }

        glBindVertexArray(0);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    /* ------------------------------------------------------------------ 6.5. Build preview */
    render_build_preview(vp_camrel);

    /* ------------------------------------------------------------------ 6.6. Inspect target ring */
    {
        float rel[3], dr, aa;
        if (inspect_ring_params(info, rel, &dr, &aa))
            draw_ring_2d(rel, dr, aa, vp_camrel);
    }

    /* ------------------------------------------------------------------ 7. Labels */
    labels_render(view_rot, proj, vp_camrel, info, dt);

    /* ------------------------------------------------------------------ 7.5. Screen flash
     *
     * Disabled on purpose: the camera-space glare/wash was reading as too
     * artificial, so we keep the volumetric supernova itself but omit the
     * fullscreen exposure pass entirely.
     */
}

/* ------------------------------------------------------------------ shutdown */
void render_shutdown(void) {
    for (int i = 0; i < 3; i++) {
        if (s_build_dist_text[i].tex) glDeleteTextures(1, &s_build_dist_text[i].tex);
        s_build_dist_text[i].tex = 0;
        s_build_dist_text[i].str[0] = '\0';
    }
    if (s_build_font) TTF_CloseFont(s_build_font);
    TTF_Quit();
    glDeleteProgram(s_sphere_shader);
    glDeleteProgram(s_atm_shader);
    glDeleteProgram(s_dot_shader);
    glDeleteProgram(s_impact_particle_shader);
    glDeleteProgram(s_glare_shader);
    glDeleteProgram(s_supernova_core_shader);
    glDeleteProgram(s_supernova_cloud_shader);
    glDeleteProgram(s_build_line_shader);
    glDeleteProgram(s_build_ui_shader);
    glDeleteBuffers(1, &s_sphere_vbo);
    glDeleteBuffers(1, &s_sphere_ebo);
    glDeleteVertexArrays(1, &s_sphere_vao);
    glDeleteBuffers(1, &s_dot_vbo);
    glDeleteVertexArrays(1, &s_dot_vao);
    glDeleteBuffers(1, &s_impact_particle_vbo);
    glDeleteVertexArrays(1, &s_impact_particle_vao);
    glDeleteBuffers(1, &s_build_line_vbo);
    glDeleteVertexArrays(1, &s_build_line_vao);
    glDeleteBuffers(1, &s_build_ui_vbo);
    glDeleteVertexArrays(1, &s_build_ui_vao);
    s_sphere_shader = s_dot_shader = 0;
    s_impact_particle_shader = 0;
    s_supernova_core_shader = s_supernova_cloud_shader = 0;
    s_build_line_shader = s_build_ui_shader = 0;
    s_build_font = NULL;
}
