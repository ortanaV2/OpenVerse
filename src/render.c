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
static GLint  s_sp_ptype       = -1;

/* Planet-type mapping by body index (matches solar_system_init order).
 * 0=rocky  1=Earth  2=Mars  3=Venus  4=Jupiter  5=Saturn
 * 6=ice-giant  7=Io  8=Titan  9=Europa                      */
static const int s_planet_types[] = {
    0,          /* 0  Sun   (star — emission path, type unused) */
    0,          /* 1  Mercury  */
    3,          /* 2  Venus    */
    1,          /* 3  Earth    */
    2,          /* 4  Mars     */
    4,          /* 5  Jupiter  */
    5,          /* 6  Saturn   */
    6,          /* 7  Uranus   */
    6,          /* 8  Neptune  */
    0,          /* 9  Ceres    */
    0,          /* 10 Pluto    */
    0,          /* 11 Eris     */
    0,          /* 12 Makemake */
    0,          /* 13 Haumea   */
    0,          /* 14 Moon     */
    0,          /* 15 Phobos   */
    0,          /* 16 Deimos   */
    7,          /* 17 Io       */
    9,          /* 18 Europa   */
    0,          /* 19 Ganymede */
    0,          /* 20 Callisto */
    0,          /* 21 Mimas    */
    0,          /* 22 Enceladus*/
    0,          /* 23 Tethys   */
    0,          /* 24 Dione    */
    0,          /* 25 Rhea     */
    8,          /* 26 Titan    */
    0,          /* 27 Miranda  */
    0,          /* 28 Ariel    */
    0,          /* 29 Umbriel  */
    0,          /* 30 Titania  */
    0,          /* 31 Oberon   */
    0,          /* 32 Triton   */
};

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

    /* Camera basis vectors in world space (extracted from view matrix) */
    Vec3 cam_right, cam_up, cam_fwd;
    mat4_get_right(view, cam_right);
    mat4_get_up   (view, cam_up);
    mat4_get_fwd  (view, cam_fwd);

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

    for (int i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];

        float wx = (float)(b->pos[0] * RS);
        float wy = (float)(b->pos[1] * RS);
        float wz = (float)(b->pos[2] * RS);

        /* Camera distance (AU) */
        Vec4 wv4 = { wx, wy, wz, 1.0f };
        Vec4 cv4;
        mat4_mul_vec4(cv4, view, wv4);
        float dcam = sqrtf(cv4[0]*cv4[0] + cv4[1]*cv4[1] + cv4[2]*cv4[2]);

        float dr = visual_radius(i, dcam);

        /* Fill BodyRenderInfo */
        info[i].pos[0] = wx;
        info[i].pos[1] = wy;
        info[i].pos[2] = wz;
        info[i].dr     = dr;
        info[i].dcam   = dcam;
        /* show=1 when the body renders smaller than ~2px (label replaces disc) */
        float px = (WIN_H / 2.0f) * dr / (dcam * half_fov_tan() + 1e-9f);
        info[i].show   = (px < 2.5f) ? 1 : 0;

        /* Sub-pixel bodies are shown as dots only — skip billboard render.
         * Rendering a sub-pixel sphere quad produces 1-2 unstable fragments
         * at semi-random pixels that flicker as the camera moves/rotates. */
        if (info[i].show) continue;

        /* Compute oc = cam - center and sun_rel = sun - center in double
         * to avoid float cancellation for small/distant bodies.           */
        double cam_mx = (double)g_cam.pos[0] * AU;
        double cam_my = (double)g_cam.pos[1] * AU;
        double cam_mz = (double)g_cam.pos[2] * AU;
        float oc_x = (float)((cam_mx - b->pos[0]) * RS);
        float oc_y = (float)((cam_my - b->pos[1]) * RS);
        float oc_z = (float)((cam_mz - b->pos[2]) * RS);
        float sr_x = (float)((g_bodies[0].pos[0] - b->pos[0]) * RS);
        float sr_y = (float)((g_bodies[0].pos[1] - b->pos[1]) * RS);
        float sr_z = (float)((g_bodies[0].pos[2] - b->pos[2]) * RS);

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

        /* Procedural surface texture uniforms */
        glUniform1f(s_sp_rotation, (float)b->rotation_angle);
        {
            int ptype = (i < (int)(sizeof(s_planet_types)/sizeof(s_planet_types[0])))
                        ? s_planet_types[i] : 0;
            glUniform1i(s_sp_ptype, ptype);
        }

        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }
    glBindVertexArray(0);

    /* ------------------------------------------------------------------ 3. Center dots
     * Priority: Sun(0) > planets > moons. Within each tier: closer first.
     * Overlap check in screen space: two dots clash if their centres are
     * closer than DOT_EXCL_PX pixels.                                      */
#define DOT_EXCL_PX 6.0f   /* exclusion radius in pixels */

    /* Build priority order: Sun, then planets by dcam, then moons by dcam */
    int dot_order[MAX_BODIES];
    int dot_np = 0, dot_nm = 0;
    int dot_planets[MAX_BODIES], dot_moons[MAX_BODIES];
    for (int i = 1; i < g_nbodies; i++) {
        if (g_bodies[i].parent < 0) dot_planets[dot_np++] = i;
        else                        dot_moons  [dot_nm++] = i;
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
    dot_order[0] = 0;
    for (int i = 0; i < dot_np; i++) dot_order[1 + i]        = dot_planets[i];
    for (int i = 0; i < dot_nm; i++) dot_order[1 + dot_np + i] = dot_moons[i];

    /* Project to screen, greedy overlap removal, collect surviving dots */
    float dot_sx[MAX_BODIES], dot_sy[MAX_BODIES];
    int   dot_vis[MAX_BODIES];
    memset(dot_vis, 0, sizeof(dot_vis));

    for (int oi = 0; oi < g_nbodies; oi++) {
        int i = dot_order[oi];
        if (!info[i].show) continue;   /* rendered as full sphere — no dot */
        float sx, sy;
        if (!mat4_project(vp, info[i].pos[0], info[i].pos[1], info[i].pos[2],
                          WIN_W, WIN_H, &sx, &sy)) continue;
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

    /* Upload and draw surviving dots.
     * Positions are camera-relative (world_AU - cam_AU), matching the
     * convention used for trails and labels.  The dot shader is given
     * vp_camrel (proj × view_rot, no translation) so the GL depth values
     * are produced by the same formula as the sphere shader's gl_FragDepth,
     * avoiding any world-space float cancellation at large distances.       */
    float dot_data[MAX_BODIES * 6];
    int   dot_count = 0;
    {
        double cx = g_cam.pos[0];
        double cy = g_cam.pos[1];
        double cz = g_cam.pos[2];
        for (int oi = 0; oi < g_nbodies; oi++) {
            int i = dot_order[oi];
            if (!dot_vis[i]) continue;
            Body *b = &g_bodies[i];
            dot_data[dot_count*6+0] = (float)(b->pos[0] * RS - cx);
            dot_data[dot_count*6+1] = (float)(b->pos[1] * RS - cy);
            dot_data[dot_count*6+2] = (float)(b->pos[2] * RS - cz);
            dot_data[dot_count*6+3] = b->col[0];
            dot_data[dot_count*6+4] = b->col[1];
            dot_data[dot_count*6+5] = b->col[2];
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

    /* ------------------------------------------------------------------ 4. Rings */
    rings_render(vp);

    /* ------------------------------------------------------------------ 5. Trails */
    trails_render(vp_camrel);

    /* ------------------------------------------------------------------ 6. Star glare
     * Additive (GL_ONE / GL_ONE) pass over all opaque geometry.
     * Depth test and depth writes are both disabled — the glare is a camera
     * lens effect and should always be visible regardless of scene depth.
     * Only drawn when the star is roughly in front of the camera.           */
    if (s_glare_shader) {
        glUseProgram(s_glare_shader);
        glUniformMatrix4fv(s_gl_vp, 1, GL_FALSE, vp);
        glUniform3f(s_gl_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_gl_up,    cam_up[0],    cam_up[1],    cam_up[2]);

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);   /* additive */
        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);

        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].is_star) continue;

            /* Skip if star is behind the camera */
            float dx = info[i].pos[0] - g_cam.pos[0];
            float dy = info[i].pos[1] - g_cam.pos[1];
            float dz = info[i].pos[2] - g_cam.pos[2];
            if (dx*cam_fwd[0] + dy*cam_fwd[1] + dz*cam_fwd[2] < 0.0f) continue;

            glUniform3f(s_gl_center, info[i].pos[0], info[i].pos[1], info[i].pos[2]);
            glUniform1f(s_gl_radius, info[i].dr);
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
    labels_render(view, proj, vp_camrel, info, dt);
}

/* ------------------------------------------------------------------ shutdown */
void render_shutdown(void) {
    glDeleteProgram(s_sphere_shader);
    glDeleteProgram(s_dot_shader);
    glDeleteProgram(s_glare_shader);
    glDeleteBuffers(1, &s_sphere_vbo);
    glDeleteBuffers(1, &s_sphere_ebo);
    glDeleteVertexArrays(1, &s_sphere_vao);
    glDeleteBuffers(1, &s_dot_vbo);
    glDeleteVertexArrays(1, &s_dot_vao);
    s_sphere_shader = s_dot_shader = 0;
}
