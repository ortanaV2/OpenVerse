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
#include "build.h"
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
static const float STAR_GLARE_BILL_SCALE = 15.0f;
static const float BODY_SPHERE_APPEAR_PX = 1.25f;
static const float BODY_DOT_FADE_START_PX = 0.75f;
static const float BODY_DOT_FADE_END_PX = 1.75f;
static const float STAR_DOT_FULL_GLARE_PX = 1.25f;
static const float STAR_DOT_FADE_START_GLARE_PX = 5.00f;

/* ------------------------------------------------------------------ build preview */
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

typedef struct {
    GLuint tex;
    int w, h;
    char str[96];
} BuildTextCache;

static BuildTextCache s_build_dist_text[3];

/* ------------------------------------------------------------------ helpers */
static float half_fov_tan(void) {
    return tanf(FOV * 0.5f * (float)(PI / 180.0));
}

static TTF_Font *build_find_font(int size) {
    static const char *paths[] = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        NULL
    };
    for (int i = 0; paths[i]; i++) {
        TTF_Font *f = TTF_OpenFont(paths[i], size);
        if (f) return f;
    }
    return NULL;
}

static GLuint build_surface_to_tex(SDL_Surface *surf, int *w, int *h) {
    SDL_Surface *c = SDL_ConvertSurfaceFormat(surf, SDL_PIXELFORMAT_ABGR8888, 0);
    SDL_FreeSurface(surf);
    if (!c) return 0;
    *w = c->w; *h = c->h;
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, c->w, c->h,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, c->pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    SDL_FreeSurface(c);
    return tex;
}

static void build_update_text(BuildTextCache *tc, const char *str) {
    if (!s_build_font || strcmp(tc->str, str) == 0) return;
    snprintf(tc->str, sizeof(tc->str), "%s", str);
    if (tc->tex) {
        glDeleteTextures(1, &tc->tex);
        tc->tex = 0;
    }
    SDL_Color col = {235, 245, 255, 235};
    SDL_Surface *surf = TTF_RenderText_Blended(s_build_font, str, col);
    if (surf) tc->tex = build_surface_to_tex(surf, &tc->w, &tc->h);
}

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

static void build_draw_text(BuildTextCache *tc, float x, float y, float h) {
    if (!tc || !tc->tex || !s_build_ui_shader) return;
    float w = h * (float)tc->w / (float)tc->h;
    glUniform1i(s_build_ui_use_tex, 1);
    glUniform4f(s_build_ui_color, 1, 1, 1, 1);
    glBindTexture(GL_TEXTURE_2D, tc->tex);
    build_draw_ui_quad(x, y, w, h);
}

static void format_dist_au(double au, char *buf, size_t n) {
    if (au < 0.001)
        snprintf(buf, n, "%.0f km", au * AU / 1000.0);
    else if (au < 1.0)
        snprintf(buf, n, "%.4f AU", au);
    else if (au < 1000.0)
        snprintf(buf, n, "%.2f AU", au);
    else
        snprintf(buf, n, "%.3f ly", au / 63241.0);
}

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Natural visual radius for a body (AU) — no boost.
 * Bodies too small to see as a disc are rendered as dots instead. */
static float visual_radius(int i, float dcam) {
    (void)dcam;
    return (float)(g_bodies[i].radius * RS);
}

static double smoothstepd(double edge0, double edge1, double x) {
    double t = (x - edge0) / (edge1 - edge0);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t * t * (3.0 - 2.0 * t);
}

static float body_point_star_glare_visibility(int body_idx) {
    Body *b = &g_bodies[body_idx];

    double bx = b->pos[0] * RS - g_cam.pos[0];
    double by = b->pos[1] * RS - g_cam.pos[1];
    double bz = b->pos[2] * RS - g_cam.pos[2];
    double bd2 = bx*bx + by*by + bz*bz;
    if (bd2 <= 1e-18) return 0;

    double bd = sqrt(bd2);
    double ux = bx / bd;
    double uy = by / bd;
    double uz = bz / bd;
    double visibility = 1.0;

    for (int i = 0; i < g_nbodies; i++) {
        if (i == body_idx) continue;
        Body *s = &g_bodies[i];
        if (!s->is_star) continue;

        double sx = s->pos[0] * RS - g_cam.pos[0];
        double sy = s->pos[1] * RS - g_cam.pos[1];
        double sz = s->pos[2] * RS - g_cam.pos[2];
        double along = sx*ux + sy*uy + sz*uz;
        if (along <= 0.0 || along >= bd) continue;

        double sd2 = sx*sx + sy*sy + sz*sz;
        double perp2 = sd2 - along*along;
        double sr = s->radius * RS;
        double r_units = sqrt(perp2 < 0.0 ? 0.0 : perp2) / sr;
        if (r_units <= 1.0) return 0.0f;

        /* Match star_glare.frag's falloff so dots fade like trails under the
         * additive glow instead of vanishing at an arbitrary outer radius. */
        double r_safe = r_units > 0.05 ? r_units : 0.05;
        double shine = 2.1 * exp(-r_safe * 0.48);
        double outer_fade = 1.0 - smoothstepd(6.0, 15.0, r_units);
        double glare = shine * outer_fade;
        double star_vis = 1.0 - smoothstepd(0.04, 0.38, glare);
        if (star_vis < visibility) visibility = star_vis;
    }

    return (float)visibility;
}

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
        if (g_bodies[i].is_star || info[i].show) continue;

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
    s_dot_vbo = gl_vbo_create(MAX_BODIES * 7 * sizeof(float),
                               NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7*sizeof(float),
                          (void*)(3*sizeof(float)));
    glBindVertexArray(0);

    /* --- Build-mode preview lines --- */
    s_build_line_shader = gl_shader_load("assets/shaders/build_line.vert",
                                         "assets/shaders/build_line.frag");
    if (s_build_line_shader) {
        s_build_line_vp = glGetUniformLocation(s_build_line_shader, "u_vp");
        s_build_line_vao = gl_vao_create();
        s_build_line_vbo = gl_vbo_create(6 * 7 * sizeof(float),
                                         NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7*sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7*sizeof(float),
                              (void*)(3*sizeof(float)));
        glBindVertexArray(0);
    }

    /* --- Build-mode distance labels (screen-space text) --- */
    s_build_ui_shader = gl_shader_load("assets/shaders/ui.vert",
                                       "assets/shaders/ui.frag");
    if (s_build_ui_shader) {
        s_build_ui_screen  = glGetUniformLocation(s_build_ui_shader, "u_screen");
        s_build_ui_color   = glGetUniformLocation(s_build_ui_shader, "u_color");
        s_build_ui_use_tex = glGetUniformLocation(s_build_ui_shader, "u_use_tex");
        s_build_ui_tex     = glGetUniformLocation(s_build_ui_shader, "u_tex");
        s_build_ui_vao = gl_vao_create();
        s_build_ui_vbo = gl_vbo_create(24 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
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
    s_build_font = build_find_font(12);
}

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

    float px = (float)(preview_m[0] * RS - g_cam.pos[0]);
    float py = (float)(preview_m[1] * RS - g_cam.pos[1]);
    float pz = (float)(preview_m[2] * RS - g_cam.pos[2]);

    /* Ghost body: screen-readable marker whose distance is radius-aware. */
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

    /* Distance guide lines to nearest bodies. */
    if (s_build_line_shader) {
        float line_data[6 * 7];
        int v = 0;
        for (int k = 0; k < 3; k++) {
            if (idx[k] < 0) continue;
            Body *b = &g_bodies[idx[k]];
            float bx = (float)(b->pos[0] * RS - g_cam.pos[0]);
            float by = (float)(b->pos[1] * RS - g_cam.pos[1]);
            float bz = (float)(b->pos[2] * RS - g_cam.pos[2]);
            float a = 0.62f - 0.12f * (float)k;

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

    /* Screen-space distance list beside the preview.
     * The list is placed on the side opposite the strongest guide-line bundle,
     * which keeps it readable without a twitchy per-label solver. */
    if (s_build_ui_shader && s_build_font) {
        glUseProgram(s_build_ui_shader);
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
            format_dist_au(dist_au[k], dist_buf, sizeof(dist_buf));
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
                if (dl < 1.0f) {
                    dir_x[k] = 1.0f;
                    dir_y[k] = 0.0f;
                    dl = 1.0f;
                }
                dir_x[k] /= dl;
                dir_y[k] /= dl;
            }

            text_w[k] = 12.0f * (float)s_build_dist_text[k].w
                      / (float)s_build_dist_text[k].h;
            if (text_w[k] > max_w) max_w = text_w[k];
            active[k] = 1;
            n_active++;
        }

        if (n_active > 0) {
            float row_h = 15.0f;
            float list_h = row_h * (float)n_active;
            float margin_x = 42.0f;
            float margin_y = 28.0f;
            float best_score = 1e30f;
            float list_x = psx + margin_x;
            float list_y = preview_y - margin_y - list_h;
            int side = 1;

            for (int q = 0; q < 4; q++) {
                int sx = (q == 0 || q == 3) ? 1 : -1;
                int sy = (q == 0 || q == 1) ? -1 : 1;
                float cx = (sx > 0) ? psx + margin_x : psx - margin_x - max_w;
                float cy = (sy < 0) ? preview_y - margin_y - list_h
                                     : preview_y + margin_y;
                float clamped_x = clampf_local(cx, 8.0f, (float)WIN_W - max_w - 8.0f);
                float clamped_y = clampf_local(cy, 8.0f, (float)WIN_H - list_h - 8.0f);
                float center_x = clamped_x + max_w * 0.5f;
                float center_y = clamped_y + list_h * 0.5f;
                float vx = center_x - psx;
                float vy = center_y - preview_y;
                float vl = sqrtf(vx*vx + vy*vy);
                float score = fabsf(clamped_x - cx) * 8.0f
                            + fabsf(clamped_y - cy) * 8.0f;
                if (vl < 1.0f) vl = 1.0f;
                vx /= vl;
                vy /= vl;

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
                build_draw_text(&s_build_dist_text[k], x, list_y + row_h * (float)row, 12.0f);
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
        info[i].show = (px < BODY_SPHERE_APPEAR_PX) ? 1 : 0;

        /* Sub-pixel bodies are shown as dots only — skip billboard render. */
        if (info[i].show) continue;
        if (b->is_star) continue;

        /* Compute oc = cam - center in double to avoid float cancellation. */
        double cam_mx = (double)g_cam.pos[0] * AU;
        double cam_my = (double)g_cam.pos[1] * AU;
        double cam_mz = (double)g_cam.pos[2] * AU;
        float oc_x = (float)((cam_mx - b->pos[0]) * RS);
        float oc_y = (float)((cam_my - b->pos[1]) * RS);
        float oc_z = (float)((cam_mz - b->pos[2]) * RS);

        /* Lighting direction: sun_rel = (root star) - body.
         * Walk parent chain to find the star this body orbits so Proxima b
         * is lit by Proxima Centauri, not Sol. */
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
     * co-located planets (e.g. Proxima b and Proxima Centauri share almost the same
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
            if (body_point_star_glare_visibility(i) <= 0.02f) continue;
            if (g_bodies[i].is_star &&
                body_px[i] * STAR_GLARE_BILL_SCALE >= STAR_DOT_FADE_START_GLARE_PX) continue;
            if (!g_bodies[i].is_star && body_px[i] >= BODY_DOT_FADE_END_PX)
                continue;
            if (body_point_occluded_by_body(i, info)) continue;
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
     * Non-star dots fade from opaque at SYS_DOT_FADE_START to transparent
     * at SYS_DOT_FADE_END.  The star dot is handled by its own glare fade.
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
     * Dot fades are carried in alpha, not by darkening RGB.  This keeps
     * transitions clean over star glare and other bright layers.
     *
     * Sphere-approach fade: planet/moon dots fade later and overlap slightly
     * with the first visible sphere pixels, so the transition is transparent
     * instead of a hard handoff.                                             */
    float dot_data[MAX_BODIES * 7];
    int   dot_count = 0;
    {
        double cx = g_cam.pos[0];
        double cy = g_cam.pos[1];
        double cz = g_cam.pos[2];
        /* Stars are clamped to within the view frustum so they remain visible
         * even at warp distances (far beyond the 2000 AU far plane).          */
        const float DOT_CLAMP_DIST = 1500.0f;

        for (int oi = 0; oi < g_nbodies; oi++) {
            int i = dot_order[oi];
            if (!dot_vis[i]) continue;
            Body *b = &g_bodies[i];

            /* LOD fade: star always full; planets/moons fade in interstellar space */
            float f = b->is_star ? 1.0f : dot_fade;
            f *= body_point_star_glare_visibility(i);

            if (b->is_star) {
                float glare_px = body_px[i] * STAR_GLARE_BILL_SCALE;
                if (glare_px > STAR_DOT_FULL_GLARE_PX) {
                    float t = (glare_px - STAR_DOT_FULL_GLARE_PX)
                            / (STAR_DOT_FADE_START_GLARE_PX - STAR_DOT_FULL_GLARE_PX);
                    if (t > 1.0f) t = 1.0f;
                    f *= 1.0f - t;
                }
            }
            if (f <= 0.0f) continue;   /* skip invisible non-star dots */

            /* Sphere-approach fade: dot remains briefly while the sphere appears. */
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
    rings_render(vp);
    asteroids_render(vp_camrel);

    /* ------------------------------------------------------------------ 5. Trails */
    trails_render(vp_camrel);

    /* ------------------------------------------------------------------ 6. Star glare
     * Draw after trails so stellar glare hides orbit lines inside the glow.
     * Depth test stays enabled, so foreground opaque bodies still eclipse it. */
    if (s_glare_shader) {
        const float GLARE_MAX_DIST = 1500.0f;

        glUseProgram(s_glare_shader);
        glUniformMatrix4fv(s_gl_vp, 1, GL_FALSE, vp_camrel);
        glUniform3f(s_gl_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_gl_up,    cam_up[0],    cam_up[1],    cam_up[2]);

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDepthMask(GL_FALSE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glBindVertexArray(s_sphere_vao);

        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].is_star) continue;

            float rx = (float)(g_bodies[i].pos[0] * RS - g_cam.pos[0]);
            float ry = (float)(g_bodies[i].pos[1] * RS - g_cam.pos[1]);
            float rz = (float)(g_bodies[i].pos[2] * RS - g_cam.pos[2]);

            if (rx*cam_fwd[0] + ry*cam_fwd[1] + rz*cam_fwd[2] < 0.0f) continue;

            float dist   = sqrtf(rx*rx + ry*ry + rz*rz);
            float radius = (float)(g_bodies[i].radius * RS);
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

    /* ------------------------------------------------------------------ 7. Labels */
    labels_render(view_rot, proj, vp_camrel, info, dt);
}

/* ------------------------------------------------------------------ shutdown */
void render_shutdown(void) {
    for (int i = 0; i < 3; i++)
        if (s_build_dist_text[i].tex) glDeleteTextures(1, &s_build_dist_text[i].tex);
    if (s_build_font) TTF_CloseFont(s_build_font);
    glDeleteProgram(s_sphere_shader);
    glDeleteProgram(s_atm_shader);
    glDeleteProgram(s_dot_shader);
    glDeleteProgram(s_glare_shader);
    glDeleteProgram(s_build_line_shader);
    glDeleteProgram(s_build_ui_shader);
    glDeleteBuffers(1, &s_sphere_vbo);
    glDeleteBuffers(1, &s_sphere_ebo);
    glDeleteVertexArrays(1, &s_sphere_vao);
    glDeleteBuffers(1, &s_dot_vbo);
    glDeleteVertexArrays(1, &s_dot_vao);
    glDeleteBuffers(1, &s_build_line_vbo);
    glDeleteVertexArrays(1, &s_build_line_vao);
    glDeleteBuffers(1, &s_build_ui_vbo);
    glDeleteVertexArrays(1, &s_build_ui_vao);
    s_sphere_shader = s_dot_shader = 0;
    s_build_line_shader = s_build_ui_shader = 0;
    s_build_font = NULL;
}
