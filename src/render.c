/*
 * render.c — scene renderer
 *
 * Pipeline each frame:
 *   1. Starfield  — rotation-only VP, GL_POINTS
 *   2. Body spheres — Phong-lit billboard quads (one per body)
 *   3. Center dots  — GL_POINTS at body centers (depth-tested)
 *   4. Trails       — trails_render()
 *   5. Labels       — labels_render()
 */
#include "render.h"
#include "body.h"
#include "camera.h"
#include "starfield.h"
#include "trails.h"
#include "rings.h"
#include "asteroids.h"
#include "labels.h"
#include "gl_utils.h"
#include "math3d.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ sphere */
/* Billboard quad shared by all spheres (unit quad, GL_TRIANGLE_FAN) */
static GLuint s_sphere_shader  = 0;
static GLuint s_sphere_vao     = 0;
static GLuint s_sphere_vbo     = 0;
static GLuint s_sphere_ebo     = 0;

static GLint  s_sp_vp          = -1;
static GLint  s_sp_center      = -1;
static GLint  s_sp_radius      = -1;
static GLint  s_sp_cam_right   = -1;
static GLint  s_sp_cam_up      = -1;
static GLint  s_sp_oc          = -1;   /* cam - center, double-precision diff */
static GLint  s_sp_sun_rel     = -1;   /* sun - center, double-precision diff */
static GLint  s_sp_cam_fwd     = -1;
static GLint  s_sp_fov_tan     = -1;
static GLint  s_sp_aspect      = -1;
static GLint  s_sp_color       = -1;
static GLint  s_sp_emission    = -1;
static GLint  s_sp_ambient     = -1;
static GLint  s_sp_sun_world   = -1;
static GLint  s_sp_rotation    = -1;
static GLint  s_sp_obliquity   = -1;
static GLint  s_sp_ptype       = -1;

/* Planet-type lookup by body name — robust against loader order changes.
 * 0=rocky(default)  1=Earth  2=Mars  3=Venus  4=Jupiter  5=Saturn
 * 6=ice-giant       7=Io     8=Titan 9=Europa                       */
static int get_planet_type(const char *name)
{
    static const struct { const char *name; int ptype; } tbl[] = {
        { "Earth",   1 }, { "Mars",    2 }, { "Venus",   3 },
        { "Jupiter", 4 }, { "Saturn",  5 }, { "Uranus",  6 },
        { "Neptune", 6 }, { "Io",      7 }, { "Titan",   8 },
        { "Europa",  9 }, { NULL,      0 }
    };
    int k;
    for (k = 0; tbl[k].name; k++)
        if (strcmp(name, tbl[k].name) == 0) return tbl[k].ptype;
    return 0;   /* rocky / default for everything else */
}

/* ------------------------------------------------------------------ atmosphere */
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

/* Atmosphere data is now stored per-body in g_bodies[i].atm_color/intensity/scale,
 * populated by universe_load() from assets/universe.json.
 * No hardcoded table needed here. */

/* ------------------------------------------------------------------ dots */
static GLuint s_dot_shader  = 0;
static GLuint s_dot_vao     = 0;
static GLuint s_dot_vbo     = 0;
static GLint  s_dot_vp      = -1;

/* ------------------------------------------------------------------ star glare */
static GLuint s_glare_shader = 0;
static GLint  s_gl_vp        = -1;
static GLint  s_gl_center    = -1;
static GLint  s_gl_radius    = -1;
static GLint  s_gl_right     = -1;
static GLint  s_gl_up        = -1;
static GLint  s_gl_color     = -1;

/* ------------------------------------------------------------------ helpers */
static float half_fov_tan(void) {
    return tanf(FOV * 0.5f * (float)(PI / 180.0));
}

/* Natural visual radius for a body (AU) — no boost.
 * Bodies too small to see as a disc are rendered as dots instead. */
static float visual_radius(int i, float dcam) {
    (void)dcam;
    return (float)(g_bodies[i].radius * RS);
}

/* ------------------------------------------------------------------ init */
void render_init(void) {
    /* Prevent near-plane clipping of close billboard geometry.
     * Instead of discarding triangles that cross the near plane,
     * OpenGL clamps their depth to [0,1] — no geometry disappears. */
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
    s_sp_rotation  = glGetUniformLocation(s_sphere_shader, "u_rotation");
    s_sp_obliquity = glGetUniformLocation(s_sphere_shader, "u_obliquity");
    s_sp_ptype     = glGetUniformLocation(s_sphere_shader, "u_planet_type");

    /* Frame-constant camera parameters */
    glUseProgram(s_sphere_shader);
    glUniform1f(s_sp_fov_tan, tanf(FOV * 0.5f * (float)(PI / 180.0)));
    glUniform1f(s_sp_aspect,  (float)WIN_W / (float)WIN_H);
    glUseProgram(0);

    /* Unit quad: UV (0,0)..(1,1) */
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

    /* --- Atmosphere shader --- */
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
        /* Frame-constant uniforms */
        glUseProgram(s_atm_shader);
        glUniform1f(glGetUniformLocation(s_atm_shader, "u_fov_tan"),
                    tanf(FOV * 0.5f * (float)(PI / 180.0)));
        glUniform1f(glGetUniformLocation(s_atm_shader, "u_aspect"),
                    (float)WIN_W / (float)WIN_H);
        glUseProgram(0);
    }

    /* --- Dot shader (reuses color.vert / color.frag) --- */
    s_dot_shader = gl_shader_load("assets/shaders/color.vert",
                                  "assets/shaders/color.frag");
    if (!s_dot_shader) {
        fprintf(stderr, "[Render] color shader failed\n");
        return;
    }
    s_dot_vp = glGetUniformLocation(s_dot_shader, "u_vp");

    /* Dynamic VBO for dot positions + colors */
    s_dot_vao = gl_vao_create();
    s_dot_vbo = gl_vbo_create(MAX_BODIES * 6 * sizeof(float),
                               NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                          (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

/* ------------------------------------------------------------------ frame */
void render_frame(const float view[16], const float proj[16],
                  const float view_rot[16], float dt) {
    /* Build combined VP */
    Mat4 vp;
    mat4_mul(vp, proj, view);

    /* Camera-relative VP: proj × view_rot (no translation).
     * Used for trails and labels whose vertex positions are expressed
     * relative to the camera origin to avoid float32 precision loss. */
    Mat4 vp_camrel;
    mat4_mul(vp_camrel, proj, view_rot);

    /* Camera basis vectors in world space.
     * Use view_rot, not the full float-eye view matrix.  The full view is kept
     * only for rings near Sol; extracting axes from it reintroduces the same
     * large-coordinate float cancellation that view_rot was created to avoid. */
    Vec3 cam_right, cam_up, cam_fwd;
    mat4_get_right(view_rot, cam_right);
    mat4_get_up   (view_rot, cam_up);
    mat4_get_fwd  (view_rot, cam_fwd);

    /* Sun position in world space (AU) — used directly for lighting */
    float sun_wx = (float)(g_bodies[0].pos[0] * RS);
    float sun_wy = (float)(g_bodies[0].pos[1] * RS);
    float sun_wz = (float)(g_bodies[0].pos[2] * RS);

    /* ------------------------------------------------------------------ 1. Starfield */
    /* Stars live at unit-sphere radius → depth ≈ far plane → Z-fight with
     * cleared depth buffer.  Disable depth test entirely for the skybox. */
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    starfield_render(view_rot, proj);
    glDepthMask(GL_TRUE);

    /* ------------------------------------------------------------------ 2. Spheres */
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    glUseProgram(s_sphere_shader);
    /* Camera-relative VP (proj × view_rot, no translation).
     * u_center is passed as camera-relative below, so float32 cancellation
     * at large world-space distances is avoided — same pattern as dots/trails. */
    glUniformMatrix4fv(s_sp_vp,       1, GL_FALSE, vp_camrel);
    glUniform3f(s_sp_sun_world,  sun_wx, sun_wy, sun_wz);
    glUniform3f(s_sp_cam_right,  cam_right[0], cam_right[1], cam_right[2]);
    glUniform3f(s_sp_cam_up,     cam_up[0],    cam_up[1],    cam_up[2]);
    glUniform3f(s_sp_cam_fwd,    cam_fwd[0],   cam_fwd[1],   cam_fwd[2]);

    glBindVertexArray(s_sphere_vao);

    /* Per-body render info also filled for labels */
    BodyRenderInfo info[MAX_BODIES];
    /* Per-body projected radius in pixels — used for dot fade near sphere pop-in */
    float body_px[MAX_BODIES];
    memset(body_px, 0, sizeof(body_px));

    for (int i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];

        float wx = (float)(b->pos[0] * RS);
        float wy = (float)(b->pos[1] * RS);
        float wz = (float)(b->pos[2] * RS);

        /* Camera-relative position in double precision.
         * Float32 world coords lose ~5 digits for bodies at large offsets
         * (e.g. Proxima b at 241 000 AU), causing dcam to be imprecise and
         * the px threshold to flicker → sphere pops in and out randomly. */
        double dxd = b->pos[0] * RS - g_cam.pos[0];
        double dyd = b->pos[1] * RS - g_cam.pos[1];
        double dzd = b->pos[2] * RS - g_cam.pos[2];
        float dcam = (float)sqrt(dxd*dxd + dyd*dyd + dzd*dzd);

        float dr = visual_radius(i, dcam);

        /* Fill BodyRenderInfo */
        info[i].pos[0] = wx;
        info[i].pos[1] = wy;
        info[i].pos[2] = wz;
        info[i].dr     = dr;
        info[i].dcam   = dcam;

        /* Use eye_z (depth along view axis) for the pixel-size threshold.
         * dcam (Euclidean) = eye_z / cos(θ) — rotating the camera changes θ
         * and therefore dcam, making the show flag oscillate at the boundary.
         * eye_z is constant under pure rotation, so the transition is stable. */
        float eye_z = (float)(dxd*cam_fwd[0] + dyd*cam_fwd[1] + dzd*cam_fwd[2]);
        float px = (eye_z > 0.0f)
                   ? (WIN_H / 2.0f) * dr / (eye_z * half_fov_tan() + 1e-9f)
                   : 0.0f;
        /* Dot–sphere transition threshold.
         * glPointSize is 2.5px, which is a dot of DIAMETER 2.5px.
         * A sphere at px=R has projected DIAMETER = 2*R px.
         * Setting the threshold at px < 1.25 means the sphere first appears
         * at diameter 2*1.25 = 2.5px — exactly matching the dot size.
         * Using px < 2.5 (old value) made the sphere pop in at 5px diameter,
         * which was 2× bigger than the dot and visually jarring. */
        body_px[i]   = px;
        info[i].show = (px < 1.25f) ? 1 : 0;

        /* Sub-pixel bodies are shown as dots only — skip billboard render. */
        if (info[i].show) continue;

        /* Compute oc = cam - center in double to avoid float cancellation. */
        double cam_mx = (double)g_cam.pos[0] * AU;
        double cam_my = (double)g_cam.pos[1] * AU;
        double cam_mz = (double)g_cam.pos[2] * AU;
        float oc_x = (float)((cam_mx - b->pos[0]) * RS);
        float oc_y = (float)((cam_my - b->pos[1]) * RS);
        float oc_z = (float)((cam_mz - b->pos[2]) * RS);

        /* Lighting direction: sun_rel = (root star) - body.
         * Walk parent chain to find the star this body orbits so Proxima b
         * is lit by Proxima Cen, not Sol. */
        int ls = i;
        while (g_bodies[ls].parent >= 0) ls = g_bodies[ls].parent;
        float sr_x = (float)((g_bodies[ls].pos[0] - b->pos[0]) * RS);
        float sr_y = (float)((g_bodies[ls].pos[1] - b->pos[1]) * RS);
        float sr_z = (float)((g_bodies[ls].pos[2] - b->pos[2]) * RS);

        /* u_center is camera-relative (= -u_oc) so the billboard vertex
         * positions computed in phong.vert are free of float32 world-space
         * cancellation for nearby bodies (Phobos, Deimos, etc.).           */
        glUniform3f(s_sp_center,  -oc_x, -oc_y, -oc_z);
        glUniform1f(s_sp_radius,   dr);
        glUniform3f(s_sp_oc,       oc_x, oc_y, oc_z);
        glUniform3f(s_sp_sun_rel,  sr_x, sr_y, sr_z);
        glUniform3f(s_sp_color,    b->col[0], b->col[1], b->col[2]);
        glUniform1f(s_sp_emission, b->is_star ? 1.0f : 0.0f);
        glUniform1f(s_sp_ambient,  b->is_star ? 1.0f : 0.05f);

        /* Procedural surface texture — name-based lookup, index-independent */
        glUniform1f(s_sp_rotation,  (float)b->rotation_angle);
        glUniform1f(s_sp_obliquity, (float)(b->obliquity * (PI / 180.0)));
        glUniform1i(s_sp_ptype,     get_planet_type(b->name));

        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }
    glBindVertexArray(0);

    /* ------------------------------------------------------------------ 2.5. Atmosphere glow
     * Additive pass (GL_SRC_ALPHA / GL_ONE), depth-tested but no depth writes.
     * Uses the same sphere billboard VAO and camera-relative VP as the sphere pass. */
    if (s_atm_shader) {
        glUseProgram(s_atm_shader);
        glUniformMatrix4fv(s_at_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_at_cam_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_at_cam_up,    cam_up[0],    cam_up[1],    cam_up[2]);
        glUniform3f(s_at_cam_fwd,   cam_fwd[0],   cam_fwd[1],   cam_fwd[2]);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);  /* additive */
        glDepthMask(GL_FALSE);              /* read depth, don't write */
        glEnable(GL_DEPTH_TEST);
        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < g_nbodies; i++) {
            if (info[i].show) continue;     /* sub-pixel body — skip */
            float intensity = g_bodies[i].atm_intensity;
            float scale     = g_bodies[i].atm_scale;
            if (intensity <= 0.0f) continue;

            Body *b = &g_bodies[i];
            double cam_mx = (double)g_cam.pos[0] * AU;
            double cam_my = (double)g_cam.pos[1] * AU;
            double cam_mz = (double)g_cam.pos[2] * AU;
            float oc_x = (float)((cam_mx - b->pos[0]) * RS);
            float oc_y = (float)((cam_my - b->pos[1]) * RS);
            float oc_z = (float)((cam_mz - b->pos[2]) * RS);
            /* Lighting: root star of this body, same logic as sphere pass */
            float sr_x, sr_y, sr_z;
            {
                int ls = i;
                while (g_bodies[ls].parent >= 0) ls = g_bodies[ls].parent;
                sr_x = (float)((g_bodies[ls].pos[0] - b->pos[0]) * RS);
                sr_y = (float)((g_bodies[ls].pos[1] - b->pos[1]) * RS);
                sr_z = (float)((g_bodies[ls].pos[2] - b->pos[2]) * RS);
            }

            float planet_r = info[i].dr;
            float atm_r    = planet_r * scale;

            glUniform3f(s_at_center,   -oc_x, -oc_y, -oc_z);
            glUniform1f(s_at_radius,    atm_r);
            glUniform1f(s_at_planet_r,  planet_r);
            glUniform3f(s_at_oc,        oc_x, oc_y, oc_z);
            glUniform3f(s_at_sun_rel,   sr_x, sr_y, sr_z);
            glUniform3f(s_at_color,     g_bodies[i].atm_color[0], g_bodies[i].atm_color[1], g_bodies[i].atm_color[2]);
            glUniform1f(s_at_intensity, intensity);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }

        glBindVertexArray(0);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    /* ------------------------------------------------------------------ 3. Center dots
     * Priority: Sun(0) > planets > moons. Within each tier: closer first.
     * Overlap check in screen space: two dots clash if their centres are
     * closer than DOT_EXCL_PX pixels.                                      */
#define DOT_EXCL_PX 6.0f   /* exclusion radius in pixels */

    /* Build priority order: stars (by dcam) > planets (by dcam) > moons (by dcam).
     * Stars must come first so they always win the screen-space overlap slot over
     * co-located planets (e.g. Proxima b and Proxima Cen share almost the same
     * screen position when seen from Sol — without this, the planet can grab the
     * slot and the star dot never renders because dot_fade kills the planet dot). */
    int dot_order[MAX_BODIES];
    int dot_ns = 0, dot_np = 0, dot_nm = 0;
    int dot_stars[MAX_BODIES], dot_planets[MAX_BODIES], dot_moons[MAX_BODIES];
    for (int i = 0; i < g_nbodies; i++) {
        if      (g_bodies[i].is_star)       dot_stars  [dot_ns++] = i;
        /* planet: no parent, or parent is a star (not a moon of a planet) */
        else if (g_bodies[i].parent < 0 ||
                 g_bodies[g_bodies[i].parent].is_star) dot_planets[dot_np++] = i;
        else                                           dot_moons  [dot_nm++] = i;
    }
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
    for (int i = 0; i < dot_ns; i++) dot_order[i]                  = dot_stars[i];
    for (int i = 0; i < dot_np; i++) dot_order[dot_ns + i]          = dot_planets[i];
    for (int i = 0; i < dot_nm; i++) dot_order[dot_ns + dot_np + i] = dot_moons[i];

    /* Project to screen, greedy overlap removal, collect surviving dots.
     *
     * IMPORTANT: use vp_camrel (rotation-only VP) with camera-relative
     * positions computed in double precision, NOT vp (full VP with float
     * translation) with absolute float world positions.  At 4+ ly the
     * camera offset is ~250,000 AU; a float32 absolute position and the
     * float translation term in vp nearly cancel, losing ~4 digits of
     * precision → star screen positions jump randomly on each frame →
     * visible as jerky "camera stutter" when moving at interstellar distances. */
    float dot_sx[MAX_BODIES], dot_sy[MAX_BODIES];
    int   dot_vis[MAX_BODIES];
    memset(dot_vis, 0, sizeof(dot_vis));

    {
        const float DOT_CLAMP_DIST_OV = 1500.0f;
        double cx2 = g_cam.pos[0];
        double cy2 = g_cam.pos[1];
        double cz2 = g_cam.pos[2];

        for (int oi = 0; oi < g_nbodies; oi++) {
            int i = dot_order[oi];
            if (!info[i].show) continue;   /* rendered as full sphere — no dot */
            Body *bi = &g_bodies[i];

            /* Camera-relative position in double → float (no float cancellation) */
            float rx = (float)(bi->pos[0] * RS - cx2);
            float ry = (float)(bi->pos[1] * RS - cy2);
            float rz = (float)(bi->pos[2] * RS - cz2);
            /* Clamp distant stars (same rule as the dot render below) */
            if (bi->is_star) {
                float d = sqrtf(rx*rx + ry*ry + rz*rz);
                if (d > DOT_CLAMP_DIST_OV && d > 1e-6f) {
                    float s = DOT_CLAMP_DIST_OV / d;
                    rx *= s; ry *= s; rz *= s;
                }
            }

            float sx, sy;
            if (!mat4_project(vp_camrel, rx, ry, rz, WIN_W, WIN_H, &sx, &sy)) continue;

            /* Check against all already-accepted dots */
            int blocked = 0;
            for (int oj = 0; oj < oi; oj++) {
                int j = dot_order[oj];
                if (!dot_vis[j]) continue;
                float dx = sx - dot_sx[j], dy = sy - dot_sy[j];
                if (dx*dx + dy*dy < DOT_EXCL_PX * DOT_EXCL_PX)
                    { blocked = 1; break; }
            }
            if (!blocked) {
                dot_sx[i]  = sx;
                dot_sy[i]  = sy;
                dot_vis[i] = 1;
            }
        }
    }

    /* Distance from camera to nearest star — used for LOD dot fade.
     * Non-star dots fade from full brightness at SYS_DOT_FADE_START to black
     * (invisible against the near-black background) at SYS_DOT_FADE_END.
     * The star dot is always kept at full brightness.
     * Subtraction is done in double BEFORE casting to float to avoid the
     * catastrophic float cancellation that occurs at 4+ ly (camera and star
     * both at ~250,000 AU → float32 difference is meaningless).               */
    float dot_fade = 1.0f;
    {
        int ref_star = nearest_star_idx();
        float sdx = (float)(g_cam.pos[0] - g_bodies[ref_star].pos[0] * RS);
        float sdy = (float)(g_cam.pos[1] - g_bodies[ref_star].pos[1] * RS);
        float sdz = (float)(g_cam.pos[2] - g_bodies[ref_star].pos[2] * RS);
        float dist_star = sqrtf(sdx*sdx + sdy*sdy + sdz*sdz);
        dot_fade = 1.0f - (dist_star - SYS_DOT_FADE_START)
                        / (SYS_DOT_FADE_END - SYS_DOT_FADE_START);
        if (dot_fade > 1.0f) dot_fade = 1.0f;
        if (dot_fade < 0.0f) dot_fade = 0.0f;
    }

    /* Upload and draw surviving dots.
     * Positions are camera-relative (world_AU - cam_AU), matching the
     * convention used for trails and labels.  The dot shader is given
     * vp_camrel (proj × view_rot, no translation) so the GL depth values
     * are produced by the same formula as the sphere shader's gl_FragDepth,
     * avoiding any world-space float cancellation at large distances.
     * Non-star dots have their RGB multiplied by dot_fade (fade to black).
     *
     * Sphere-approach fade: when a body's projected radius px approaches the
     * dot→sphere threshold (1.25 px), the dot's brightness fades from 1.0 at
     * px=0.75 to 0.2 at px=1.25.  This prevents the jarring jump from a
     * bright dot to a just-appearing sphere at the transition point.          */
    float dot_data[MAX_BODIES * 6];
    int   dot_count = 0;
    {
        double cx = g_cam.pos[0];
        double cy = g_cam.pos[1];
        double cz = g_cam.pos[2];
        /* Stars are clamped to within the view frustum so they remain visible
         * even at warp distances (far beyond the 2000 AU far plane).          */
        const float DOT_CLAMP_DIST = 1500.0f;

        /* Sphere-approach fade constants */
        const float SPHERE_FADE_START = 0.75f;   /* px: fade begins       */
        const float SPHERE_FADE_END   = 1.25f;   /* px: sphere appears    */
        const float SPHERE_FADE_MIN   = 0.2f;    /* alpha floor at end    */

        for (int oi = 0; oi < g_nbodies; oi++) {
            int i = dot_order[oi];
            if (!dot_vis[i]) continue;
            Body *b = &g_bodies[i];

            /* LOD fade: star always full; planets/moons fade in interstellar space */
            float f = b->is_star ? 1.0f : dot_fade;
            if (f <= 0.0f) continue;   /* skip invisible non-star dots */

            /* Sphere-approach fade: dot dims as body grows toward sphere threshold */
            float px_i = body_px[i];
            if (px_i > SPHERE_FADE_START) {
                float t = (px_i - SPHERE_FADE_START) / (SPHERE_FADE_END - SPHERE_FADE_START);
                if (t > 1.0f) t = 1.0f;
                float sfade = 1.0f - (1.0f - SPHERE_FADE_MIN) * t;
                f *= sfade;
            }
            if (f <= 0.0f) continue;

            float bx = (float)(b->pos[0] * RS - cx);
            float by = (float)(b->pos[1] * RS - cy);
            float bz = (float)(b->pos[2] * RS - cz);
            if (b->is_star) {
                float d = sqrtf(bx*bx + by*by + bz*bz);
                if (d > DOT_CLAMP_DIST && d > 1e-6f) {
                    float s = DOT_CLAMP_DIST / d;
                    bx *= s; by *= s; bz *= s;
                }
            }
            dot_data[dot_count*6+0] = bx;
            dot_data[dot_count*6+1] = by;
            dot_data[dot_count*6+2] = bz;
            dot_data[dot_count*6+3] = b->col[0] * f;
            dot_data[dot_count*6+4] = b->col[1] * f;
            dot_data[dot_count*6+5] = b->col[2] * f;
            dot_count++;
        }
    }

    if (dot_count > 0) {
        glUseProgram(s_dot_shader);
        glUniformMatrix4fv(s_dot_vp, 1, GL_FALSE, vp_camrel);
        glBindVertexArray(s_dot_vao);
        glBindBuffer(GL_ARRAY_BUFFER, s_dot_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        dot_count * 6 * sizeof(float), dot_data);
        glEnable(GL_DEPTH_TEST);
        glPointSize(2.5f);
        glDrawArrays(GL_POINTS, 0, dot_count);
        glPointSize(1.0f);
        glBindVertexArray(0);
    }

    /* ------------------------------------------------------------------ 4. Rings + Asteroid belts */
    rings_render(vp);
    asteroids_render(vp_camrel);

    /* ------------------------------------------------------------------ 5. Trails */
    trails_render(vp_camrel);

    /* ------------------------------------------------------------------ 6. Star glare
     * Additive (GL_ONE / GL_ONE) pass over all opaque geometry.
     * Depth test and depth writes are both disabled — the glare is a camera
     * lens effect and should always be visible regardless of scene depth.
     * Only drawn when the star is roughly in front of the camera.           */
    if (s_glare_shader) {
        /* Star glare uses vp_camrel (rotation-only) so the centre is passed
         * as a camera-relative offset.  This lets us clamp distant stars to
         * within the near/far range (the star at 63 000 AU in warp mode would
         * otherwise be clipped by the far plane).  The radius is scaled by the
         * same factor to maintain the star's angular size on screen.          */
        const float GLARE_MAX_DIST = 1500.0f;

        glUseProgram(s_glare_shader);
        glUniformMatrix4fv(s_gl_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_gl_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_gl_up,    cam_up[0],    cam_up[1],    cam_up[2]);

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);   /* additive */
        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);

        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].is_star) continue;

            /* Camera-relative star position (double precision → float) */
            float rx = (float)(g_bodies[i].pos[0] * RS - g_cam.pos[0]);
            float ry = (float)(g_bodies[i].pos[1] * RS - g_cam.pos[1]);
            float rz = (float)(g_bodies[i].pos[2] * RS - g_cam.pos[2]);

            /* Skip if star is behind the camera */
            if (rx*cam_fwd[0] + ry*cam_fwd[1] + rz*cam_fwd[2] < 0.0f) continue;

            /* Clamp distance to keep the billboard within the view frustum.
             * Radius is scaled proportionally to preserve angular size.       */
            float dist   = sqrtf(rx*rx + ry*ry + rz*rz);
            float radius = info[i].dr;
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
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    /* ------------------------------------------------------------------ 7. Labels */
    labels_render(view_rot, proj, vp_camrel, info, dt);
}

/* ------------------------------------------------------------------ shutdown */
void render_shutdown(void) {
    glDeleteProgram(s_sphere_shader);
    glDeleteProgram(s_atm_shader);
    glDeleteProgram(s_dot_shader);
    glDeleteProgram(s_glare_shader);
    glDeleteBuffers(1, &s_sphere_vbo);
    glDeleteBuffers(1, &s_sphere_ebo);
    glDeleteVertexArrays(1, &s_sphere_vao);
    glDeleteBuffers(1, &s_dot_vbo);
    glDeleteVertexArrays(1, &s_dot_vao);
    s_sphere_shader = s_dot_shader = 0;
}
