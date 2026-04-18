/*
 * ui.c — 2D HUD overlay
 *
 * Layout (top bar, full width, BAR_H pixels tall):
 *   - Background: semi-transparent dark strip
 *   - Blue fill:  log-normalised camera movement speed (left → right)
 *   - Centre text: physical movement speed  ("0.50 AU/s")
 *   - Right text:  simulation speed         ("365 days/s")
 */
#include "ui.h"
#include "camera.h"
#include "physics.h"
#include "gl_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ layout */
#define BAR_H         4       /* bar height, pixels                 */
#define BAR_W_FRAC    0.5f    /* fraction of screen width           */
#define BAR_TOP       12.0f   /* distance from top of screen        */
#define TEXT_GAP      6.0f    /* gap between bar bottom and text    */
#define FONT_SIZE     12

/* Camera speed range (AU / real-second) */
#define CAM_MIN  0.00001f
#define CAM_MAX  200.0f

/* ------------------------------------------------------------------ GL */
static GLuint s_shader     = 0;
static GLuint s_vao        = 0;
static GLuint s_vbo        = 0;    /* 6 vertices × 4 floats (x,y,u,v)  */
static GLint  s_loc_screen  = -1;
static GLint  s_loc_color   = -1;
static GLint  s_loc_use_tex = -1;
static GLint  s_loc_tex     = -1;

static TTF_Font *s_font = NULL;

/* ------------------------------------------------------------------ text cache */
typedef struct { GLuint tex; int w, h; char str[64]; } TextCache;
static TextCache s_tc_move = {0};
static TextCache s_tc_sim  = {0};
static TextCache s_tc_fps  = {0};

/* ------------------------------------------------------------------ FPS smoothing */
/* Exponential moving average over ~30 frames, updated every UI frame.
 * We measure time internally so the ui_render() signature stays unchanged. */
static Uint64 s_fps_prev  = 0;
static float  s_fps_smooth = 0.0f;

/* ------------------------------------------------------------------ helpers */
static const char *s_font_paths[] = {
    "C:/Windows/Fonts/segoeui.ttf",
    "C:/Windows/Fonts/arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    NULL
};

static TTF_Font *find_font(int size) {
    for (int i = 0; s_font_paths[i]; i++) {
        TTF_Font *f = TTF_OpenFont(s_font_paths[i], size);
        if (f) return f;
    }
    return NULL;
}

static GLuint surf_to_tex(SDL_Surface *surf, int *w, int *h) {
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

/* Re-render text texture only when the string changes */
static void update_text(TextCache *tc, const char *str, SDL_Color col) {
    if (strcmp(tc->str, str) == 0) return;
    strncpy(tc->str, str, 63);
    tc->str[63] = '\0';
    if (tc->tex) { glDeleteTextures(1, &tc->tex); tc->tex = 0; }
    SDL_Surface *surf = TTF_RenderText_Blended(s_font, str, col);
    if (surf) tc->tex = surf_to_tex(surf, &tc->w, &tc->h);
}

/* Upload 2 triangles and draw */
static void draw_quad(float x, float y, float w, float h) {
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

static void draw_rect(float x, float y, float w, float h,
                      float r, float g, float b, float a) {
    glUniform1i(s_loc_use_tex, 0);
    glUniform4f(s_loc_color, r, g, b, a);
    draw_quad(x, y, w, h);
}

static void draw_tex(TextCache *tc, float x, float y, float h) {
    if (!tc || !tc->tex) return;
    float w = h * (float)tc->w / (float)tc->h;
    glUniform1i(s_loc_use_tex, 1);
    glUniform4f(s_loc_color, 1, 1, 1, 1);
    glBindTexture(GL_TEXTURE_2D, tc->tex);
    draw_quad(x, y, w, h);
}

/* ------------------------------------------------------------------ public */
void ui_init(void) {
    s_shader = gl_shader_load("assets/shaders/ui.vert",
                              "assets/shaders/ui.frag");
    if (!s_shader) { fprintf(stderr, "[UI] shader failed\n"); return; }

    s_loc_screen  = glGetUniformLocation(s_shader, "u_screen");
    s_loc_color   = glGetUniformLocation(s_shader, "u_color");
    s_loc_use_tex = glGetUniformLocation(s_shader, "u_use_tex");
    s_loc_tex     = glGetUniformLocation(s_shader, "u_tex");

    s_vao = gl_vao_create();
    s_vbo = gl_vbo_create(24 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float),
                          (void*)(2*sizeof(float)));
    glBindVertexArray(0);

    /* u_screen and u_tex are frame-constant */
    glUseProgram(s_shader);
    glUniform2f(s_loc_screen, (float)WIN_W, (float)WIN_H);
    glUniform1i(s_loc_tex, 0);
    glUseProgram(0);

    TTF_Init();
    s_font = find_font(FONT_SIZE);
    if (!s_font) fprintf(stderr, "[UI] no font found\n");
}

void ui_render(void) {
    if (!s_shader || !s_font) return;

    const float W  = (float)WIN_W;
    const float TH = (float)FONT_SIZE;

    /* Log-normalised camera speed → fill fraction */
    float spd = g_cam.speed;
    if (spd < CAM_MIN) spd = CAM_MIN;
    if (spd > CAM_MAX) spd = CAM_MAX;
    float t = logf(spd / CAM_MIN) / logf(CAM_MAX / CAM_MIN);

    /* Format movement speed string — show WARP indicator when T is held */
    char mv_str[64];
    if (g_warp) {
        snprintf(mv_str, sizeof(mv_str), "WARP  1.00 ly/s");
    } else if (spd < 0.001f) {
        snprintf(mv_str, sizeof(mv_str), "%.5f AU/s", (double)spd);
    } else if (spd < 1.0f) {
        snprintf(mv_str, sizeof(mv_str), "%.3f AU/s", (double)spd);
    } else {
        snprintf(mv_str, sizeof(mv_str), "%.2f AU/s", (double)spd);
    }

    /* Format sim speed string */
    char ss_str[64];
    if (g_paused)
        snprintf(ss_str, sizeof(ss_str), "Paused");
    else {
        double days = g_sim_speed / DAY;
        if (days < 1.0)
            snprintf(ss_str, sizeof(ss_str), "%.2g days/s", days);
        else if (days < 10.0)
            snprintf(ss_str, sizeof(ss_str), "%.1f days/s", days);
        else
            snprintf(ss_str, sizeof(ss_str), "%.0f days/s", days);
    }

    /* FPS — exponential moving average, alpha=0.1 (≈ 10-frame window) */
    Uint64 now  = SDL_GetPerformanceCounter();
    Uint64 freq = SDL_GetPerformanceFrequency();
    if (s_fps_prev != 0 && freq > 0) {
        float inst = (float)freq / (float)(now - s_fps_prev);
        s_fps_smooth = (s_fps_smooth == 0.0f) ? inst
                     : s_fps_smooth + 0.1f * (inst - s_fps_smooth);
    }
    s_fps_prev = now;

    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "%.0f fps", (double)s_fps_smooth);

    SDL_Color white = {255, 255, 255, 220};
    update_text(&s_tc_move, mv_str, white);
    update_text(&s_tc_sim,  ss_str, white);
    update_text(&s_tc_fps,  fps_str, white);

    /* ---- layout ---- */
    const float BH   = (float)BAR_H;
    const float BW   = W * BAR_W_FRAC;
    const float BX   = (W - BW) * 0.5f;     /* centered */
    const float BY   = BAR_TOP;
    const float TY   = BY + BH + TEXT_GAP;  /* text baseline below bar */

    /* ---- GL state ---- */
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(s_shader);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(s_vao);
    glBindBuffer(GL_ARRAY_BUFFER, s_vbo);

    /* Bar background (dark, full width of bar) */
    draw_rect(BX, BY, BW, BH, 1.0f, 1.0f, 1.0f, 0.15f);

    /* Bar fill */
    draw_rect(BX, BY, BW * t, BH, 1.0f, 1.0f, 1.0f, 0.85f);

    /* Movement speed — centred below bar */
    if (s_tc_move.tex) {
        float tw = TH * (float)s_tc_move.w / (float)s_tc_move.h;
        draw_tex(&s_tc_move, (W - tw) * 0.5f, TY, TH);
    }

    /* Sim speed — right-aligned below bar */
    if (s_tc_sim.tex) {
        float tw = TH * (float)s_tc_sim.w / (float)s_tc_sim.h;
        draw_tex(&s_tc_sim, BX + BW - tw, TY, TH);
    }

    /* FPS — top-right corner, right-aligned to screen edge */
    if (s_tc_fps.tex) {
        const float MARGIN = 12.0f;
        float tw = TH * (float)s_tc_fps.w / (float)s_tc_fps.h;
        draw_tex(&s_tc_fps, W - tw - MARGIN, TY, TH);
    }

    /* ---- restore ---- */
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void ui_shutdown(void) {
    if (s_tc_move.tex) glDeleteTextures(1, &s_tc_move.tex);
    if (s_tc_sim.tex)  glDeleteTextures(1, &s_tc_sim.tex);
    if (s_tc_fps.tex)  glDeleteTextures(1, &s_tc_fps.tex);
    if (s_vbo)  glDeleteBuffers(1, &s_vbo);
    if (s_vao)  glDeleteVertexArrays(1, &s_vao);
    if (s_shader) glDeleteProgram(s_shader);
    if (s_font)   TTF_CloseFont(s_font);
    TTF_Quit();
    s_shader = s_vao = s_vbo = 0;
    s_font = NULL;
}
