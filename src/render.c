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
static GLint  s_sp_cam_pos     = -1;
static GLint  s_sp_cam_fwd     = -1;
static GLint  s_sp_fov_tan     = -1;
static GLint  s_sp_aspect      = -1;
static GLint  s_sp_color       = -1;
static GLint  s_sp_emission    = -1;
static GLint  s_sp_ambient     = -1;
static GLint  s_sp_sun_world   = -1;

/* ------------------------------------------------------------------ dots */
static GLuint s_dot_shader  = 0;
static GLuint s_dot_vao     = 0;
static GLuint s_dot_vbo     = 0;
static GLint  s_dot_vp      = -1;

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
    s_sp_cam_pos   = glGetUniformLocation(s_sphere_shader, "u_cam_pos");
    s_sp_cam_fwd   = glGetUniformLocation(s_sphere_shader, "u_cam_fwd");
    s_sp_fov_tan   = glGetUniformLocation(s_sphere_shader, "u_fov_tan");
    s_sp_aspect    = glGetUniformLocation(s_sphere_shader, "u_aspect");

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
    glUniformMatrix4fv(s_sp_vp,       1, GL_FALSE, vp);
    glUniform3f(s_sp_sun_world,  sun_wx, sun_wy, sun_wz);
    glUniform3f(s_sp_cam_right,  cam_right[0], cam_right[1], cam_right[2]);
    glUniform3f(s_sp_cam_up,     cam_up[0],    cam_up[1],    cam_up[2]);
    glUniform3f(s_sp_cam_pos,    g_cam.pos[0], g_cam.pos[1], g_cam.pos[2]);
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

        glUniform3f(s_sp_center,   wx, wy, wz);
        glUniform1f(s_sp_radius,   dr);
        glUniform3f(s_sp_color,    b->col[0], b->col[1], b->col[2]);
        glUniform1f(s_sp_emission, b->is_star ? 1.0f : 0.0f);
        glUniform1f(s_sp_ambient,  b->is_star ? 1.0f : 0.05f);

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

    /* Upload and draw surviving dots */
    float dot_data[MAX_BODIES * 6];
    int   dot_count = 0;
    for (int oi = 0; oi < g_nbodies; oi++) {
        int i = dot_order[oi];
        if (!dot_vis[i]) continue;
        Body *b = &g_bodies[i];
        dot_data[dot_count*6+0] = (float)(b->pos[0] * RS);
        dot_data[dot_count*6+1] = (float)(b->pos[1] * RS);
        dot_data[dot_count*6+2] = (float)(b->pos[2] * RS);
        dot_data[dot_count*6+3] = b->col[0];
        dot_data[dot_count*6+4] = b->col[1];
        dot_data[dot_count*6+5] = b->col[2];
        dot_count++;
    }

    if (dot_count > 0) {
        glUseProgram(s_dot_shader);
        glUniformMatrix4fv(s_dot_vp, 1, GL_FALSE, vp);
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

    /* ------------------------------------------------------------------ 4. Trails */
    trails_render(vp);

    /* ------------------------------------------------------------------ 5. Labels */
    labels_render(view, proj, vp, info, dt);
}

/* ------------------------------------------------------------------ shutdown */
void render_shutdown(void) {
    glDeleteProgram(s_sphere_shader);
    glDeleteProgram(s_dot_shader);
    glDeleteBuffers(1, &s_sphere_vbo);
    glDeleteBuffers(1, &s_sphere_ebo);
    glDeleteVertexArrays(1, &s_sphere_vao);
    glDeleteBuffers(1, &s_dot_vbo);
    glDeleteVertexArrays(1, &s_dot_vao);
    s_sphere_shader = s_dot_shader = 0;
}
