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
    memset(s_tex,   0, sizeof(s_tex));
    memset(s_tex_w, 0, sizeof(s_tex_w));
    memset(s_tex_h, 0, sizeof(s_tex_h));

    for (int i = 0; i < g_nbodies; i++) {
        SDL_Color col;
        col.r = (Uint8)(fminf(g_bodies[i].col[0]*1.4f+0.15f, 1.0f)*255);
        col.g = (Uint8)(fminf(g_bodies[i].col[1]*1.4f+0.15f, 1.0f)*255);
        col.b = (Uint8)(fminf(g_bodies[i].col[2]*1.4f+0.15f, 1.0f)*255);
        col.a = 255;
        SDL_Surface *surf = TTF_RenderText_Blended(s_font, g_bodies[i].name, col);
        if (!surf) continue;
        s_tex[i] = surface_to_texture(surf, &s_tex_w[i], &s_tex_h[i]);
    }
}

void labels_render(const float view[16], const float proj[16],
                   const float vp[16], const BodyRenderInfo *info) {
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
        if (!s_tex[i])                                    continue;
        if (!is_sun && !info[i].show)                     continue;
        if (!is_sun && info[i].dcam > MAX_LABEL_DIST)     continue;

        /* Anchor: just above the body centre */
        float ax = info[i].pos[0];
        float ay = info[i].pos[1] + info[i].dr * 1.4f;
        float az = info[i].pos[2];

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

    /* ---- Step 2: sort by distance (closest = highest priority, Sun pinned 0) ---- */
    order[0] = 0;
    for (int i = 1; i < g_nbodies; i++) order[i] = i;
    for (int i = 2; i < g_nbodies; i++) {
        int tmp = order[i], k = i;
        while (k > 1 && info[order[k-1]].dcam > info[tmp].dcam) {
            order[k] = order[k-1]; k--;
        }
        order[k] = tmp;
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

    /* ---- Step 4: draw surviving labels ---- */
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
        int is_sun = (i == 0);
        if (!lvis[i] && !is_sun) continue;
        if (!s_tex[i])           continue;

        /* World-space label dimensions */
        float fh = (info[i].dcam * 2.0f * LABEL_PX_H * half_fov_tan) / (float)WIN_H;
        float fw = fh * (float)s_tex_w[i] / (float)s_tex_h[i];

        /* Anchor = just above body, shifted slightly right */
        float ax = info[i].pos[0] + cam_right[0]*(fw*0.1f);
        float ay = info[i].pos[1] + info[i].dr*1.4f + cam_up[1]*(fh*0.1f);
        float az = info[i].pos[2] + cam_right[2]*(fw*0.1f);

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
