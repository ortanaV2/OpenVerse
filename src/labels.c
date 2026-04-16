/*
 * labels.c — body label rendering
 *
 * Pipeline:
 *   1. SDL_ttf renders each name to a surface → uploaded as GL texture
 *   2. Each frame: project label anchor to screen space, sort by distance,
 *      greedy AABB overlap removal, draw surviving labels as billboard quads
 *   3. The Sun always renders its label at highest priority
 *
 * Billboard shader receives the camera right/up vectors and anchor position;
 * the quad geometry is constructed entirely in the vertex shader.
 */
#include "labels.h"
#include "body.h"
#include "camera.h"
#include "gl_utils.h"
#include "math3d.h"

#define MAX_LABEL_DIST 55.0f   /* AU — labels disappear beyond this */
#define LBL_PAD        6.0f    /* extra px padding for overlap AABB  */
#define LABEL_PX_H    14.0f    /* desired label height in pixels     */

/* GL resources */
static GLuint s_shader   = 0;
static GLuint s_vao      = 0;
static GLuint s_vbo      = 0;
static GLuint s_ebo      = 0;

static GLint  s_loc_vp     = -1;
static GLint  s_loc_anchor = -1;
static GLint  s_loc_right  = -1;
static GLint  s_loc_up     = -1;
static GLint  s_loc_tex    = -1;

/* Per-body label textures */
static GLuint s_tex[MAX_BODIES];
static int    s_tex_w[MAX_BODIES];
static int    s_tex_h[MAX_BODIES];

/* Hysteresis: label only turns on after SHOW_DELAY seconds of eligibility,
 * and only turns off after HIDE_DELAY seconds of being blocked/absent.
 * This prevents labels from flickering when they sit on the overlap boundary. */
#define SHOW_DELAY 0.20f   /* s — must be eligible this long before appearing  */
#define HIDE_DELAY 0.06f   /* s — must be blocked this long before disappearing */
static float s_show_accum[MAX_BODIES];   /* time continuously eligible to show */
static float s_hide_accum[MAX_BODIES];   /* time continuously blocked/off      */
static int   s_active[MAX_BODIES];       /* current visible state (0/1)        */
/* anchor is recomputed from g_bodies[i].pos each frame — no float32 cache needed */

static TTF_Font *s_font = NULL;

/* ------------------------------------------------------------------ font */
static const char *FONT_PATHS[] = {
    "C:/Windows/Fonts/segoeui.ttf",
    "C:/Windows/Fonts/arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    NULL
};

static TTF_Font *find_font(int size) {
    for (int i = 0; FONT_PATHS[i]; i++) {
        TTF_Font *f = TTF_OpenFont(FONT_PATHS[i], size);
        if (f) return f;
    }
    return NULL;
}

/* ------------------------------------------------------------------ texture */
static GLuint surface_to_texture(SDL_Surface *surf, int *w, int *h) {
    SDL_Surface *conv = SDL_ConvertSurfaceFormat(surf, SDL_PIXELFORMAT_ABGR8888, 0);
    SDL_FreeSurface(surf);
    if (!conv) return 0;

    *w = conv->w; *h = conv->h;
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, conv->w, conv->h,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, conv->pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    SDL_FreeSurface(conv);
    return tex;
}

/* ------------------------------------------------------------------ public */
void labels_init(void) {
    /* Shader */
    s_shader = gl_shader_load("assets/shaders/label.vert",
                              "assets/shaders/label.frag");
    if (!s_shader) { fprintf(stderr,"[Labels] shader failed\n"); return; }

    s_loc_vp     = glGetUniformLocation(s_shader, "u_vp");
    s_loc_anchor = glGetUniformLocation(s_shader, "u_anchor");
    s_loc_right  = glGetUniformLocation(s_shader, "u_right");
    s_loc_up     = glGetUniformLocation(s_shader, "u_up");
    s_loc_tex    = glGetUniformLocation(s_shader, "u_tex");

    /* Unit quad: 4 vertices with UV = position in (0..1, 0..1) */
    static const float quad_verts[] = {
        0.0f, 0.0f,   /* bottom-left  */
        1.0f, 0.0f,   /* bottom-right */
        1.0f, 1.0f,   /* top-right    */
        0.0f, 1.0f,   /* top-left     */
    };
    static const unsigned int quad_idx[] = { 0,1,2, 0,2,3 };

    s_vao = gl_vao_create();
    s_vbo = gl_vbo_create(sizeof(quad_verts), quad_verts, GL_STATIC_DRAW);
    s_ebo = gl_ebo_create(sizeof(quad_idx), quad_idx);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glBindVertexArray(0);

    /* Font */
    if (TTF_Init() < 0) {
        fprintf(stderr, "[Labels] TTF_Init: %s\n", TTF_GetError());
        return;
    }
    s_font = find_font(16);
    if (!s_font) {
        fprintf(stderr, "[Labels] no usable font found — labels disabled\n");
        return;
    }

    /* Pre-render one texture per body */
    memset(s_tex,        0, sizeof(s_tex));
    memset(s_tex_w,      0, sizeof(s_tex_w));
    memset(s_tex_h,      0, sizeof(s_tex_h));
    memset(s_show_accum, 0, sizeof(s_show_accum));
    memset(s_hide_accum, 0, sizeof(s_hide_accum));
    memset(s_active,     0, sizeof(s_active));
    for (int i = 0; i < g_nbodies; i++) {
        SDL_Color col;
        col.r = (Uint8)(fminf(g_bodies[i].col[0]*1.4f+0.15f, 1.0f)*255);
        col.g = (Uint8)(fminf(g_bodies[i].col[1]*1.4f+0.15f, 1.0f)*255);
        col.b = (Uint8)(fminf(g_bodies[i].col[2]*1.4f+0.15f, 1.0f)*255);
        col.a = 255;
        int is_moon = (g_bodies[i].parent >= 0);
        TTF_SetFontStyle(s_font, is_moon ? TTF_STYLE_ITALIC : TTF_STYLE_NORMAL);
        SDL_Surface *surf = TTF_RenderText_Blended(s_font, g_bodies[i].name, col);
        if (!surf) continue;
        s_tex[i] = surface_to_texture(surf, &s_tex_w[i], &s_tex_h[i]);
    }
    TTF_SetFontStyle(s_font, TTF_STYLE_NORMAL);
}

void labels_render(const float view[16], const float proj[16],
                   const float vp[16], const BodyRenderInfo *info,
                   float dt) {
    (void)proj;   /* reserved for future use (e.g. depth-sort) */
    if (!s_shader || !s_font) return;

    /* Camera right and up in world space (extracted from view matrix) */
    Vec3 cam_right, cam_up;
    mat4_get_right(view, cam_right);
    mat4_get_up   (view, cam_up);

    /* FOV helper for computing world-space label size */
    float half_fov_tan = tanf(FOV * 0.5f * (float)(PI / 180.0));

    /* ---- Step 1: project label anchor to screen, compute rect ---- */
    float lsx[MAX_BODIES], lsy[MAX_BODIES];
    float lsw[MAX_BODIES], lsh[MAX_BODIES];
    int   lvis[MAX_BODIES];
    int   order[MAX_BODIES];

    for (int i = 0; i < g_nbodies; i++) {
        int is_sun = (i == 0);
        order[i]   = i;
        lvis[i]    = 0;
        if (!s_tex[i])                                continue;
        if (!is_sun && info[i].dcam > MAX_LABEL_DIST) continue;

        /* Anchor: just above the body centre.
         * Subtract camera position in double to eliminate float32 cancellation
         * when the camera is within a few thousand km of the body.
         * vp here is proj×view_rot (camera-relative VP, no translation). */
        double cx = (double)g_cam.pos[0];
        double cy = (double)g_cam.pos[1];
        double cz = (double)g_cam.pos[2];
        float ax = (float)(g_bodies[i].pos[0] * RS - cx);
        float ay = (float)(g_bodies[i].pos[1] * RS - cy) + info[i].dr * 1.4f;
        float az = (float)(g_bodies[i].pos[2] * RS - cz);

        float sx, sy;
        if (!mat4_project(vp, ax, ay, az, WIN_W, WIN_H, &sx, &sy)) continue;

        /* Convert from GL (y=0 bottom) to SDL (y=0 top) screen coords */
        float screen_y = (float)WIN_H - sy;

        float ph = LABEL_PX_H;
        float pw = ph * (float)s_tex_w[i] / (float)s_tex_h[i];

        lsx[i] = sx;
        lsy[i] = screen_y - ph;
        lsw[i] = pw + LBL_PAD;
        lsh[i] = ph + LBL_PAD;
        lvis[i] = 1;
    }

    /* ---- Step 2: priority order — Sun, then planets, then moons.
     * Within planets and within moons: sorted by camera distance (closest first).
     * This guarantees planet labels always win over moon labels in overlap removal. */
    {
        int np = 0, nm = 0;
        int planets[MAX_BODIES], moons[MAX_BODIES];
        for (int i = 1; i < g_nbodies; i++) {
            if (g_bodies[i].parent < 0) planets[np++] = i;
            else                        moons  [nm++] = i;
        }
        /* insertion-sort planets by dcam */
        for (int i = 1; i < np; i++) {
            int tmp = planets[i], k = i;
            while (k > 0 && info[planets[k-1]].dcam > info[tmp].dcam)
                { planets[k] = planets[k-1]; k--; }
            planets[k] = tmp;
        }
        /* insertion-sort moons by dcam */
        for (int i = 1; i < nm; i++) {
            int tmp = moons[i], k = i;
            while (k > 0 && info[moons[k-1]].dcam > info[tmp].dcam)
                { moons[k] = moons[k-1]; k--; }
            moons[k] = tmp;
        }
        order[0] = 0;   /* Sun always first */
        for (int i = 0; i < np; i++) order[1 + i]        = planets[i];
        for (int i = 0; i < nm; i++) order[1 + np + i]   = moons[i];
    }

    /* ---- Step 3: greedy AABB overlap removal ---- */
    for (int i = 0; i < g_nbodies; i++) {
        int idx = order[i];
        if (!lvis[idx]) continue;
        for (int j = 0; j < i; j++) {
            int jdx = order[j];
            if (!lvis[jdx]) continue;
            if (lsx[idx]          < lsx[jdx]+lsw[jdx] &&
                lsx[idx]+lsw[idx] > lsx[jdx]           &&
                lsy[idx]          < lsy[jdx]+lsh[jdx] &&
                lsy[idx]+lsh[idx] > lsy[jdx]) {
                lvis[idx] = 0; break;
            }
        }
    }

    /* ---- Step 4: hysteresis — debounce lvis into s_active ---- */
    for (int i = 0; i < g_nbodies; i++) {
        if (lvis[i]) {
            s_show_accum[i] += dt;
            s_hide_accum[i]  = 0.0f;
            if (s_show_accum[i] >= SHOW_DELAY)
                s_active[i] = 1;
        } else {
            s_hide_accum[i] += dt;
            s_show_accum[i]  = 0.0f;
            if (s_hide_accum[i] >= HIDE_DELAY)
                s_active[i] = 0;
        }
    }

    /* ---- Step 5: draw surviving labels ---- */
    glUseProgram(s_shader);
    glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);
    glUniform1i(s_loc_tex, 0);
    glActiveTexture(GL_TEXTURE0);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);

    glBindVertexArray(s_vao);

    for (int i = 0; i < g_nbodies; i++) {
        if (!s_active[i]) continue;
        if (!s_tex[i])    continue;

        /* World-space label dimensions */
        float fh = (info[i].dcam * 2.0f * LABEL_PX_H * half_fov_tan) / (float)WIN_H;
        float fw = fh * (float)s_tex_w[i] / (float)s_tex_h[i];

        /* Camera-relative anchor computed fresh from the double-precision body
         * position — eliminates float32 jitter when close to small bodies.
         * vp is proj×view_rot so camera-relative coords map correctly. */
        double cx2 = (double)g_cam.pos[0];
        double cy2 = (double)g_cam.pos[1];
        double cz2 = (double)g_cam.pos[2];
        float bax = (float)(g_bodies[i].pos[0] * RS - cx2);
        float bay = (float)(g_bodies[i].pos[1] * RS - cy2) + info[i].dr * 1.4f;
        float baz = (float)(g_bodies[i].pos[2] * RS - cz2);

        float ax = bax + cam_right[0]*(fw*0.1f);
        float ay = bay              + cam_up[1]*(fh*0.1f);
        float az = baz + cam_right[2]*(fw*0.1f);

        Vec3 right_scaled, up_scaled;
        vec3_scale(right_scaled, cam_right, fw);
        vec3_scale(up_scaled,   cam_up,    fh);

        glUniform3f(s_loc_anchor, ax, ay, az);
        glUniform3f(s_loc_right,  right_scaled[0], right_scaled[1], right_scaled[2]);
        glUniform3f(s_loc_up,     up_scaled[0],    up_scaled[1],    up_scaled[2]);

        glBindTexture(GL_TEXTURE_2D, s_tex[i]);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }

    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void labels_shutdown(void) {
    glDeleteTextures(MAX_BODIES, s_tex);
    glDeleteBuffers(1, &s_vbo);
    glDeleteBuffers(1, &s_ebo);
    glDeleteVertexArrays(1, &s_vao);
    glDeleteProgram(s_shader);
    if (s_font) TTF_CloseFont(s_font);
    TTF_Quit();
    s_shader = s_vao = s_vbo = s_ebo = 0;
    s_font = NULL;
}
