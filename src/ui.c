/*
 * ui.c — 2D HUD overlay
 *
 * Layout (top bar, full width, BAR_H pixels tall):
 *   - Background: semi-transparent dark strip
 *   - White fill:  log-normalised camera movement speed (left → right)
 *   - Centre text: physical movement speed  ("0.50 AU/s")
 *   - Right text:  simulation speed         ("365 days/s")
 *   - Left text:   nearest body and distance
 *   - Top-right:   FPS counter
 *
 * Speed bar normalisation:
 *   Camera speed spans ~7 orders of magnitude (0.00001 → 200 AU/s in normal
 *   mode, 200 → 63241 AU/s in warp). A linear bar would be useless — the fill
 *   fraction is computed as t = log(speed/min) / log(max/min), compressing
 *   the full range into the bar width.
 *
 * Text caching (TextCache):
 *   SDL_TTF re-rendering is expensive. Each text element stores the last
 *   rendered string and colour; a strcmp check skips re-rendering when the
 *   content hasn't changed. Only a new string triggers a GL texture upload.
 *
 * FPS smoothing:
 *   An exponential moving average with α=0.1 (~10-frame effective window)
 *   damps per-frame jitter. The EMA is updated once per ui_render() call
 *   using SDL_GetPerformanceCounter for sub-millisecond resolution.
 *
 * Build bar slide-in animation:
 *   When build mode is entered, s_build_anim_t ramps from 0 to 1 over 0.22 s.
 *   An ease-out cubic (1 - (1-t)³) starts fast and settles smoothly so the
 *   panel doesn't abruptly appear. The x-offset is (ease - 1) × panel_width,
 *   which moves the panel from off-screen left to its final position.
 *
 * Pause menu:
 *   White buttons float over a 40%-alpha full-screen dark overlay.
 *   Hit-testing uses the same PauseMenuLayout struct as drawing, so mouse
 *   hover and click coordinates map to the same item rectangles.
 */
#include "ui.h"
#include "camera.h"
#include "physics.h"
#include "build.h"
#include "inspect.h"
#include "body.h"
#include "gl_utils.h"
#include "ui_theme.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ── layout constants ─────────────────────────────────────────────────────── */
#define BAR_H         4       /* bar height, pixels                 */
#define BAR_W_FRAC    0.5f    /* fraction of screen width           */
#define BAR_TOP       12.0f   /* distance from top of screen        */
#define TEXT_GAP      6.0f    /* gap between bar bottom and text    */
#define FONT_SIZE     16
#define BUILD_ITEM_FONT_SIZE 18
#define MENU_TITLE_SIZE 24
#define MENU_TEXT_SIZE  18
#define PAUSE_MENU_PANEL_W 324.0f
#define SLIDER_BTN_W       38.0f
#define PAUSE_MENU_PANEL_H   354.0f   /* 7 items: 7×42 + 6×10 */

/* ── controls page layout ──────────────────────────────────────────────────── */
#define CONTROLS_PANEL_W    PAUSE_MENU_PANEL_W   /* same as main-menu buttons */
#define CONTROLS_ITEM_H      18.0f
#define CONTROLS_ITEM_GAP     3.0f
#define CONTROLS_TITLE_AREA  52.0f
#define CONTROLS_FONT_H      16.0f   /* native s_font size — renders sharply */
#define CONTROLS_PAD         14.0f   /* horizontal padding from panel edges */
#define CONTROLS_PAD_BOTTOM  10.0f   /* extra space below last row */
#define CONTROLS_RETURN_H    42.0f
#define CONTROLS_COUNT       14
/* content height = title area + rows + bottom padding */
#define CONTROLS_CONTENT_H  (CONTROLS_TITLE_AREA \
                             + (float)CONTROLS_COUNT * CONTROLS_ITEM_H \
                             + (float)(CONTROLS_COUNT - 1) * CONTROLS_ITEM_GAP \
                             + CONTROLS_PAD_BOTTOM)
#define CONTROLS_PANEL_H    (CONTROLS_CONTENT_H + 12.0f + CONTROLS_RETURN_H)
#define PAUSE_MENU_ITEM_W PAUSE_MENU_PANEL_W
#define PAUSE_MENU_ITEM_H 42.0f
#define PAUSE_MENU_ITEM_ACTIVE_H 42.0f
#define PAUSE_MENU_ITEM_GAP 10.0f

/* Camera speed range (AU / real-second) for log-normalised bar */
#define CAM_MIN       0.00001f
#define CAM_MAX       200.0f       /* normal max = warp min */
#define WARP_MAX  63241.0f         /* 1 ly/s */

/* ── GL objects ───────────────────────────────────────────────────────────── */
static GLuint s_shader     = 0;
static GLuint s_vao        = 0;
static GLuint s_vbo        = 0;    /* 6 vertices × 4 floats (x,y,u,v)  */
static GLuint s_pause_blur_tex = 0;
static int    s_pause_blur_w = 0;
static int    s_pause_blur_h = 0;
static GLint  s_loc_screen  = -1;
static GLint  s_loc_color   = -1;
static GLint  s_loc_use_tex = -1;
static GLint  s_loc_tex     = -1;
static GLint  s_loc_texel_size = -1;

static TTF_Font *s_font = NULL;
static TTF_Font *s_build_item_font = NULL;
static TTF_Font *s_menu_font = NULL;
static TTF_Font *s_menu_title_font = NULL;

/* ── text cache ───────────────────────────────────────────────────────────── */
/* Each TextCache entry holds a GL texture, its pixel dimensions, the last
 * rendered string, and the last colour. update_text_with_font() compares the
 * incoming string and colour against the cached values; only on a mismatch
 * does it call TTF_RenderText_Blended() and upload a new texture. */
typedef struct {
    GLuint tex;
    int w, h;
    char str[64];
    SDL_Color col;
} TextCache;
static TextCache s_tc_move = {0};
static TextCache s_tc_sim  = {0};
static TextCache s_tc_fps  = {0};
static TextCache s_tc_nearest = {0};
static TextCache s_tc_build_title = {0};
static TextCache s_tc_build_hint  = {0};
static TextCache s_tc_build_items[8];
static TextCache s_tc_pause_items[7];
static TextCache s_tc_arrow_l   = {0};  /* black < */
static TextCache s_tc_arrow_r   = {0};  /* black > */
static TextCache s_tc_arrow_l_w = {0};  /* white < (for hover) */
static TextCache s_tc_arrow_r_w = {0};  /* white > (for hover) */

/* ── controls page text caches ─────────────────────────────────────────────── */
static TextCache s_tc_ctrl_title  = {0};
static TextCache s_tc_ctrl_return = {0};
static TextCache s_tc_ctrl_key [CONTROLS_COUNT];
static TextCache s_tc_ctrl_desc[CONTROLS_COUNT];

/* Pause menu item indices for slider rows — must match the enum in main.c. */
#define PAUSE_SLIDER_MUSIC  4
#define PAUSE_SLIDER_SENS   5

/* Accent colour #0064DE as float components. */
#define ACCENT_R 0.000f
#define ACCENT_G 0.392f
#define ACCENT_B 0.871f

static int   s_pause_menu_visible = 0;
static int   s_pause_menu_selected = 0;
static int   s_pause_menu_vsync = 0;
static float s_pause_menu_music_vol = 0.6f;
static float s_pause_menu_mouse_sens = 0.25f;
static int   s_pause_page = 0;   /* 0 = main menu, 1 = controls */

/* Flash timer: show sim speed briefly after +/- is pressed even while paused */
#define SPEED_FLASH_MS 1500
static Uint64 s_speed_flash_at = 0;  /* SDL tick (ms) when last speed change occurred; 0 = no flash */

void ui_notify_speed_change(void) {
    s_speed_flash_at = SDL_GetTicks64();
}

/* Layout structs computed fresh each frame so resizing is free. */
typedef struct {
    int n;
    float item_w;
    float item_h;
    float item_active_h;
    float gap;
    float first_item_y;
    float item_x;
    float panel_x;
    float panel_y;
    float panel_w;
    float panel_h;
} PauseMenuLayout;

typedef struct {
    int n;
    float item_x;         /* x of item body (fixed) */
    float item_w;
    float item_active_w;
    float item_h;
    float strip_w;        /* strip right-edge = item_x */
    float gap;
    float items_y;        /* y of first item */
    float title_y;        /* y of "BUILD" title text */
    float hint_y;         /* y of hint text below panel */
    float panel_x;
    float panel_y;
    float panel_w;
    float panel_h;
} BuildBarLayout;

/* ── build bar slide-in animation ─────────────────────────────────────────── */
/* s_build_anim_t: normalised progress [0, 1] of the slide-in animation.
 * Advances at rate 1/0.22 s (reaches 1.0 after 220 ms).
 * Resets to 0.0 each time build mode is entered (s_build_prev_mode flips). */
static float  s_build_anim_t    = 0.0f;
static Uint64 s_build_anim_ts   = 0;
static int    s_build_prev_mode = 0;

/* ── FPS smoothing ────────────────────────────────────────────────────────── */
/* Exponential moving average over ~10 frames (α=0.1).
 * Updated every ui_render() call; timer state managed internally. */
static Uint64 s_fps_prev  = 0;
static float  s_fps_smooth = 0.0f;

/* ── layout helpers ───────────────────────────────────────────────────────── */
static PauseMenuLayout pause_menu_layout(float W, float H)
{
    PauseMenuLayout layout;
    layout.n = 7;
    layout.item_w = PAUSE_MENU_ITEM_W;
    layout.item_h = PAUSE_MENU_ITEM_H;
    layout.item_active_h = PAUSE_MENU_ITEM_ACTIVE_H;
    layout.gap = PAUSE_MENU_ITEM_GAP;
    layout.panel_x = (W - PAUSE_MENU_PANEL_W) * 0.5f;
    layout.panel_y = (H - PAUSE_MENU_PANEL_H) * 0.5f;
    layout.panel_w = PAUSE_MENU_PANEL_W;
    layout.panel_h = PAUSE_MENU_PANEL_H;
    layout.item_x = layout.panel_x;
    layout.first_item_y = layout.panel_y;
    return layout;
}

static BuildBarLayout build_bar_layout(float W)
{
    (void)W;
    BuildBarLayout layout;
    layout.n = build_preset_count();
    if (layout.n > 8) layout.n = 8;

    const float PANEL_PAD    = 12.0f;
    const float LEFT_MARGIN  = PANEL_PAD;
    const float TITLE_AREA_H = 72.0f;
    const float BOTTOM_AREA_H = 42.0f;
    layout.strip_w        = 8.0f;
    layout.item_h         = PAUSE_MENU_ITEM_H;
    layout.item_w         = 112.0f;
    layout.item_active_w  = 132.0f;
    layout.gap            = PAUSE_MENU_ITEM_GAP;
    layout.panel_x        = LEFT_MARGIN;
    layout.item_x         = layout.panel_x + PANEL_PAD + layout.strip_w;
    layout.panel_w        = PANEL_PAD + layout.strip_w + layout.item_w + PANEL_PAD;

    float total_h    = layout.n * layout.item_h + (layout.n - 1) * layout.gap;
    layout.panel_h   = TITLE_AREA_H + total_h + BOTTOM_AREA_H;
    layout.panel_y   = ((float)WIN_H - layout.panel_h) * 0.5f;
    layout.items_y   = layout.panel_y + TITLE_AREA_H;
    layout.title_y   = layout.panel_y + 24.0f;
    layout.hint_y    = layout.panel_y + layout.panel_h - 28.0f;

    return layout;
}


/* Re-render text texture only when the string or colour changes. */
static void update_text_with_font(TextCache *tc, TTF_Font *font,
                                  const char *str, SDL_Color col) {
    if (!font) return;
    if (strcmp(tc->str, str) == 0 &&
        tc->col.r == col.r && tc->col.g == col.g &&
        tc->col.b == col.b && tc->col.a == col.a) return;
    strncpy(tc->str, str, 63);
    tc->str[63] = '\0';
    tc->col = col;
    if (tc->tex) { glDeleteTextures(1, &tc->tex); tc->tex = 0; }
    SDL_Surface *surf = TTF_RenderText_Blended(font, str, col);
    if (surf) tc->tex = gl_surf_to_tex(surf, &tc->w, &tc->h);
}

static void update_text(TextCache *tc, const char *str, SDL_Color col) {
    update_text_with_font(tc, s_font, str, col);
}

/* ── pause menu hit testing ───────────────────────────────────────────────── */
/* Returns the item index under (mx, my), or -1 if outside all items.
 * Uses the same PauseMenuLayout as drawing for consistent coordinates. */
static int pause_menu_item_at(float mx, float my)
{
    PauseMenuLayout layout = pause_menu_layout((float)WIN_W, (float)WIN_H);

    if (mx < layout.item_x || mx > layout.item_x + layout.item_w) return -1;

    for (int i = 0; i < layout.n; i++) {
        int active = (i == s_pause_menu_selected);
        float item_h = active ? layout.item_active_h : layout.item_h;
        float item_y = layout.first_item_y
                     + (float)i * (layout.item_h + layout.gap)
                     + (layout.item_h - item_h);
        if (my >= item_y && my <= item_y + item_h)
            return i;
    }
    return -1;
}


/*
 * nearest_body_distance_string — find the nearest body to the camera and
 * format a "name  distance" label.
 *
 * Two-pass logic: the first pass searches all bodies. If the nearest is more
 * than 1000 AU away (i.e. interstellar distances), a second pass finds only
 * the nearest star so the label always shows the most relevant body, not a
 * dead planet orbiting a distant star.
 */
static void nearest_body_distance_string(char *buf, size_t n)
{
    int best = -1;
    double best_d = 1e300;

    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        double dx = g_bodies[i].pos[0] * RS - g_cam.pos[0];
        double dy = g_bodies[i].pos[1] * RS - g_cam.pos[1];
        double dz = g_bodies[i].pos[2] * RS - g_cam.pos[2];
        double d = sqrt(dx*dx + dy*dy + dz*dz);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }

    /* Fall back to nearest star if everything is > 1000 AU away */
    if (best >= 0 && best_d > 1000.0) {
        best = -1;
        best_d = 1e300;
        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive) continue;
            if (!g_bodies[i].is_star) continue;
            double dx = g_bodies[i].pos[0] * RS - g_cam.pos[0];
            double dy = g_bodies[i].pos[1] * RS - g_cam.pos[1];
            double dz = g_bodies[i].pos[2] * RS - g_cam.pos[2];
            double d = sqrt(dx*dx + dy*dy + dz*dz);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
    }

    if (best < 0) {
        snprintf(buf, n, "nearest --");
    } else {
        char dist[32];
        body_format_dist_au(best_d, dist, sizeof(dist));
        snprintf(buf, n, "%.31s  %.28s", g_bodies[best].name, dist);
    }
}

/* ── quad drawing ─────────────────────────────────────────────────────────── */
/* Upload 2 triangles and draw. UV [0,1]² for textured quads. */
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

static void draw_tex(TextCache *tc, float x, float y, float h);

static void draw_bottom_mode_label(float W, float H, const char *title, const char *hint)
{
    SDL_Color title_col = {255, 255, 255, 235};
    SDL_Color hint_col  = {255, 255, 255, 170};
    float title_y = hint ? H - 48.0f : H - 34.0f;
    float title_w = 0.0f;
    float hint_w = 0.0f;

    update_text_with_font(&s_tc_build_title, s_menu_title_font, title, title_col);
    if (hint)
        update_text_with_font(&s_tc_build_hint, s_font, hint, hint_col);

    if (s_tc_build_title.tex) {
        title_w = (float)MENU_TITLE_SIZE * (float)s_tc_build_title.w / (float)s_tc_build_title.h;
        draw_tex(&s_tc_build_title, (W - title_w) * 0.5f, title_y, (float)MENU_TITLE_SIZE);
    }

    if (hint && s_tc_build_hint.tex) {
        hint_w = (float)FONT_SIZE * (float)s_tc_build_hint.w / (float)s_tc_build_hint.h;
        draw_tex(&s_tc_build_hint, (W - hint_w) * 0.5f, H - 22.0f, (float)FONT_SIZE);
    }

    {
        float bar_w = fmaxf(title_w, hint_w) + 28.0f;
        if (bar_w > W * 0.55f) bar_w = W * 0.55f;
        draw_rect((W - bar_w) * 0.5f, H - (float)BAR_H, bar_w, (float)BAR_H,
                  1.0f, 1.0f, 1.0f, 1.0f);
    }
}

static void draw_pause_blur(float W, float H)
{
    int win_w = WIN_W;
    int win_h = WIN_H;
    if (win_w <= 0 || win_h <= 0) return;

    if (!s_pause_blur_tex || s_pause_blur_w != win_w || s_pause_blur_h != win_h) {
        if (s_pause_blur_tex)
            glDeleteTextures(1, &s_pause_blur_tex);
        glGenTextures(1, &s_pause_blur_tex);
        glBindTexture(GL_TEXTURE_2D, s_pause_blur_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, win_w, win_h,
                     0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
        s_pause_blur_w = win_w;
        s_pause_blur_h = win_h;
    } else {
        glBindTexture(GL_TEXTURE_2D, s_pause_blur_tex);
    }

    glReadBuffer(GL_BACK);
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, win_w, win_h);

    glUniform1i(s_loc_use_tex, 2);
    glUniform2f(s_loc_texel_size, 1.0f / (float)win_w, 1.0f / (float)win_h);
    glUniform4f(s_loc_color, 1.0f, 1.0f, 1.0f, 1.0f);
    draw_quad(0.0f, 0.0f, W, H);
}

/* draw_tex — render a TextCache entry at pixel position (x, y) with a given
 * height in pixels. Width is computed from the texture's aspect ratio so
 * glyphs are never stretched. */
static void draw_tex(TextCache *tc, float x, float y, float h) {
    if (!tc || !tc->tex) return;
    float w = h * (float)tc->w / (float)tc->h;
    glUniform1i(s_loc_use_tex, 1);
    glUniform4f(s_loc_color, 1, 1, 1, 1);
    glBindTexture(GL_TEXTURE_2D, tc->tex);
    draw_quad(x, y, w, h);
}

/* ── build bar ────────────────────────────────────────────────────────────── */
/*
 * draw_build_bar — render the preset selector panel on the left side of screen.
 *
 * Slide-in animation:
 *   s_build_anim_t is advanced each call by dt/0.22. Ease-out cubic:
 *     ease = 1 - (1 - t)³
 *   The panel offset ox = (ease - 1) × (panel_x + panel_w) translates from
 *   fully off-screen left (ease=0 → ox = -panel_right) to final position
 *   (ease=1 → ox = 0).
 *
 * Active item indication:
 *   The colour strip stays fixed-width. The selected white item body expands
 *   to the right and uses full opacity.
 */
static void draw_build_bar(float W)
{
    if (!g_build_mode) {
        s_build_anim_t    = 0.0f;
        s_build_prev_mode = 0;
        return;
    }

    /* Slide-in animation: start timer on mode entry */
    if (!s_build_prev_mode) {
        s_build_anim_t  = 0.0f;
        s_build_anim_ts = SDL_GetPerformanceCounter();
    }
    s_build_prev_mode = 1;

    if (s_build_anim_t < 1.0f) {
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = (float)(now - s_build_anim_ts) / (float)SDL_GetPerformanceFrequency();
        s_build_anim_ts = now;
        s_build_anim_t += dt / 0.22f;
        if (s_build_anim_t > 1.0f) s_build_anim_t = 1.0f;
    }

    /* Ease-out cubic: starts fast, settles smoothly */
    float inv  = 1.0f - s_build_anim_t;
    float ease = 1.0f - inv * inv * inv;

    BuildBarLayout layout = build_bar_layout(W);

    /* Shift everything left by (1-ease) × full panel width+margin */
    float ox = (ease - 1.0f) * (layout.panel_x + layout.panel_w);

    SDL_Color item_col  = {0, 0, 0, 255};
    const char *hint = g_build_tab_held ? "scroll to select" : "hold TAB and scroll";

    draw_bottom_mode_label(W, (float)WIN_H, "BUILD", hint);

    int selected = build_selected_index();

    for (int i = 0; i < layout.n; i++) {
        const BuildPreset *p = build_preset_at(i);
        if (!p) continue;
        int active = (i == selected);
        float item_y  = layout.items_y + (layout.item_h + layout.gap) * (float)i;
        float item_w  = active ? layout.item_active_w : layout.item_w;
        float strip_x = layout.item_x - layout.strip_w + ox;

        draw_rect(layout.item_x + ox, item_y, item_w, layout.item_h,
                  1.0f, 1.0f, 1.0f, active ? 1.0f : 0.72f);
        draw_rect(strip_x, item_y, layout.strip_w, layout.item_h,
                  p->col[0], p->col[1], p->col[2], 1.0f);

        update_text_with_font(&s_tc_build_items[i], s_build_item_font ? s_build_item_font : s_font,
                              p->name, item_col);
        if (s_tc_build_items[i].tex) {
            float tw     = (float)BUILD_ITEM_FONT_SIZE * (float)s_tc_build_items[i].w / (float)s_tc_build_items[i].h;
            float text_y = item_y + (layout.item_h - (float)BUILD_ITEM_FONT_SIZE) * 0.5f;
            draw_tex(&s_tc_build_items[i],
                     layout.item_x + ox + (item_w - tw) * 0.5f,
                     text_y,
                     (float)BUILD_ITEM_FONT_SIZE);
        }
    }

}

/* ── pause menu ───────────────────────────────────────────────────────────── */
static void draw_pause_menu(float W, float H)
{
    if (!s_pause_menu_visible) return;

    PauseMenuLayout layout = pause_menu_layout(W, H);
    char vol_label[32], sens_label[32];
    snprintf(vol_label, sizeof(vol_label), "Music Volume: %d%%",
             (int)(s_pause_menu_music_vol * 100.0f + 0.5f));
    snprintf(sens_label, sizeof(sens_label), "Mouse Sensitivity: %.2f",
             s_pause_menu_mouse_sens);
    const char *labels[7] = {
        "Continue",
        "Reset Universe",
        s_pause_menu_vsync ? "Deactivate V-Sync" : "Activate V-Sync",
        "Controls",
        vol_label,
        sens_label,
        "Leave"
    };

    SDL_Color black = {0,   0,   0,   255};
    SDL_Color white = {255, 255, 255, 255};
    update_text_with_font(&s_tc_arrow_l,   s_menu_font, "<", black);
    update_text_with_font(&s_tc_arrow_r,   s_menu_font, ">", black);
    update_text_with_font(&s_tc_arrow_l_w, s_menu_font, "<", white);
    update_text_with_font(&s_tc_arrow_r_w, s_menu_font, ">", white);
    for (int i = 0; i < 7; i++)
        update_text_with_font(&s_tc_pause_items[i], s_menu_font, labels[i], black);

    /* Get current mouse position for button hover highlight. */
    int mx_i, my_i;
    SDL_GetMouseState(&mx_i, &my_i);
    float fmx = (float)mx_i;
    float fmy = (float)my_i;

    draw_pause_blur(W, H);
    draw_rect(0.0f, 0.0f, W, H, 0.0f, 0.0f, 0.0f, 0.40f);

    for (int i = 0; i < layout.n; i++) {
        int active = (i == s_pause_menu_selected);
        float base_alpha = active ? 1.0f : 0.72f;
        float item_h = active ? layout.item_active_h : layout.item_h;
        float item_y = layout.first_item_y + (float)i * (layout.item_h + layout.gap)
                     + (layout.item_h - item_h);
        int is_slider = (i == PAUSE_SLIDER_MUSIC || i == PAUSE_SLIDER_SENS);

        if (!is_slider) {
            draw_rect(layout.item_x, item_y, layout.item_w, item_h,
                      1.0f, 1.0f, 1.0f, base_alpha);
            if (s_tc_pause_items[i].tex) {
                float tw = (float)MENU_TEXT_SIZE * (float)s_tc_pause_items[i].w
                         / (float)s_tc_pause_items[i].h;
                draw_tex(&s_tc_pause_items[i],
                         layout.item_x + (layout.item_w - tw) * 0.5f,
                         item_y + (item_h - (float)MENU_TEXT_SIZE) * 0.5f,
                         (float)MENU_TEXT_SIZE);
            }
        } else {
            /* Slider item — three zones: [<] [body] [>] */
            float body_x = layout.item_x + SLIDER_BTN_W;
            float body_w = layout.item_w - 2.0f * SLIDER_BTN_W;
            float btn_r_x = layout.item_x + layout.item_w - SLIDER_BTN_W;

            /* Detect which zone the mouse is over (for highlight). */
            int mouse_in_item = (fmy >= item_y && fmy <= item_y + item_h);
            int hover_l = mouse_in_item && (fmx >= layout.item_x)  && (fmx < body_x);
            int hover_r = mouse_in_item && (fmx >= btn_r_x)        && (fmx <= layout.item_x + layout.item_w);

            /* Button backgrounds: grey normally, #0064DE on hover. */
            draw_rect(layout.item_x, item_y, SLIDER_BTN_W, item_h,
                      hover_l ? ACCENT_R : 0.72f,
                      hover_l ? ACCENT_G : 0.72f,
                      hover_l ? ACCENT_B : 0.72f,
                      base_alpha);
            draw_rect(btn_r_x, item_y, SLIDER_BTN_W, item_h,
                      hover_r ? ACCENT_R : 0.72f,
                      hover_r ? ACCENT_G : 0.72f,
                      hover_r ? ACCENT_B : 0.72f,
                      base_alpha);

            /* Slider body — white. */
            draw_rect(body_x, item_y, body_w, item_h, 1.0f, 1.0f, 1.0f, base_alpha);

            /* Fill bar at bottom of body area. */
            float val;
            if (i == PAUSE_SLIDER_MUSIC) {
                val = s_pause_menu_music_vol;
            } else {
                val = (s_pause_menu_mouse_sens - 0.05f) / (1.0f - 0.05f);
                if (val < 0.0f) val = 0.0f;
                if (val > 1.0f) val = 1.0f;
            }
            float bar_h = 5.0f;
            draw_rect(body_x, item_y + item_h - bar_h,
                      val * body_w, bar_h,
                      ACCENT_R, ACCENT_G, ACCENT_B, active ? 0.9f : 0.65f);

            /* Arrow glyphs: white on hovered (#0064DE) button, black otherwise. */
            {
                TextCache *tcl = hover_l ? &s_tc_arrow_l_w : &s_tc_arrow_l;
                if (tcl->tex) {
                    float tw = (float)MENU_TEXT_SIZE * (float)tcl->w / (float)tcl->h;
                    draw_tex(tcl,
                             layout.item_x + (SLIDER_BTN_W - tw) * 0.5f,
                             item_y + (item_h - (float)MENU_TEXT_SIZE) * 0.5f,
                             (float)MENU_TEXT_SIZE);
                }
            }
            {
                TextCache *tcr = hover_r ? &s_tc_arrow_r_w : &s_tc_arrow_r;
                if (tcr->tex) {
                    float tw = (float)MENU_TEXT_SIZE * (float)tcr->w / (float)tcr->h;
                    draw_tex(tcr,
                             btn_r_x + (SLIDER_BTN_W - tw) * 0.5f,
                             item_y + (item_h - (float)MENU_TEXT_SIZE) * 0.5f,
                             (float)MENU_TEXT_SIZE);
                }
            }

            /* Label + value centred on body. */
            if (s_tc_pause_items[i].tex) {
                float tw = (float)MENU_TEXT_SIZE * (float)s_tc_pause_items[i].w
                         / (float)s_tc_pause_items[i].h;
                draw_tex(&s_tc_pause_items[i],
                         body_x + (body_w - tw) * 0.5f,
                         item_y + (item_h - (float)MENU_TEXT_SIZE) * 0.5f,
                         (float)MENU_TEXT_SIZE);
            }
        }
    }
}

/* Returns -1 (left arrow hit), +1 (right arrow hit), or 0 (body / not a slider). */
int ui_pause_menu_slider_click_delta(int mouse_x, int mouse_y)
{
    PauseMenuLayout layout = pause_menu_layout((float)WIN_W, (float)WIN_H);
    float mx = (float)mouse_x;
    float my = (float)mouse_y;

    for (int i = PAUSE_SLIDER_MUSIC; i <= PAUSE_SLIDER_SENS; i++) {
        int active = (i == s_pause_menu_selected);
        float item_h = active ? layout.item_active_h : layout.item_h;
        float item_y = layout.first_item_y + (float)i * (layout.item_h + layout.gap)
                     + (layout.item_h - item_h);
        if (my < item_y || my > item_y + item_h) continue;
        if (mx < layout.item_x || mx > layout.item_x + layout.item_w) continue;
        if (mx < layout.item_x + SLIDER_BTN_W) return -1;
        if (mx > layout.item_x + layout.item_w - SLIDER_BTN_W) return 1;
        return 0;
    }
    return 0;
}

/* ── public API ───────────────────────────────────────────────────────────── */
void ui_init(void) {
    s_shader = gl_shader_load("assets/shaders/ui.vert",
                              "assets/shaders/ui.frag");
    if (!s_shader) { fprintf(stderr, "[UI] shader failed\n"); return; }

    s_loc_screen  = glGetUniformLocation(s_shader, "u_screen");
    s_loc_color   = glGetUniformLocation(s_shader, "u_color");
    s_loc_use_tex = glGetUniformLocation(s_shader, "u_use_tex");
    s_loc_tex     = glGetUniformLocation(s_shader, "u_tex");
    s_loc_texel_size = glGetUniformLocation(s_shader, "u_texel_size");

    s_vao = gl_vao_create();
    /* Single VBO for 6 vertices (2 triangles); re-uploaded per draw_quad call. */
    s_vbo = gl_vbo_create(24 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float),
                          (void*)(2*sizeof(float)));
    glBindVertexArray(0);

    /* u_screen and u_tex are frame-constant; set once */
    glUseProgram(s_shader);
    glUniform2f(s_loc_screen, (float)WIN_W, (float)WIN_H);
    glUniform1i(s_loc_tex, 0);
    glUseProgram(0);

    TTF_Init();
    s_font = ui_theme_open_font(FONT_SIZE);
    s_build_item_font = ui_theme_open_font(BUILD_ITEM_FONT_SIZE);
    s_menu_font = ui_theme_open_font(MENU_TEXT_SIZE);
    s_menu_title_font = ui_theme_open_font(MENU_TITLE_SIZE);
    if (!s_font) fprintf(stderr, "[UI] no font found\n");
    if (!s_build_item_font) s_build_item_font = s_font;
    if (!s_menu_font) s_menu_font = s_font;
    if (!s_menu_title_font) s_menu_title_font = s_menu_font;
    /* Bold title only when it's a distinct font object (avoid double-bolding) */
    if (s_menu_title_font && s_menu_title_font != s_menu_font && s_menu_title_font != s_font)
        TTF_SetFontStyle(s_menu_title_font, TTF_STYLE_BOLD);
}

void ui_set_pause_menu(int visible, int selected, int vsync_enabled,
                       float music_vol, float mouse_sens, int page)
{
    s_pause_menu_visible  = visible ? 1 : 0;
    s_pause_menu_selected = selected;
    s_pause_menu_vsync    = vsync_enabled ? 1 : 0;
    s_pause_menu_music_vol  = music_vol;
    s_pause_menu_mouse_sens = mouse_sens;
    s_pause_page = page;
}

/* ── controls page ───────────────────────────────────────────────────────────
 * Read-only list of all controls. Only the Return button is interactive. */
static void draw_controls_page(float W, float H)
{
    static const char *KEYS[CONTROLS_COUNT] = {
        "W / S", "A / D", "Q / E", "Mouse", "Scroll",
        "T", "+ / -", "Space",
        "B", "Tab + Scroll", "I",
        "R", "Escape", "F11 / Alt+Enter"
    };
    static const char *DESCS[CONTROLS_COUNT] = {
        "Move forward / backward", "Strafe left / right", "Move down / up",
        "Look around", "Adjust camera speed",
        "Toggle warp mode", "Sim speed up / down", "Pause / resume",
        "Toggle build mode", "Cycle build presets", "Toggle inspection mode",
        "Reset camera", "Open menu / exit modes", "Fullscreen"
    };

    float panel_x    = (W - CONTROLS_PANEL_W) * 0.5f;
    float panel_y    = (H - CONTROLS_PANEL_H) * 0.5f;
    float content_h  = CONTROLS_CONTENT_H;
    float btn_y      = panel_y + content_h + 12.0f;

    SDL_Color black    = {0,   0,   0,   255};
    SDL_Color darkgrey = {90,  90,  90,  255};

    update_text_with_font(&s_tc_ctrl_title,  s_menu_title_font, "CONTROLS", black);
    update_text_with_font(&s_tc_ctrl_return, s_menu_font,       "Return",   black);
    for (int i = 0; i < CONTROLS_COUNT; i++) {
        update_text_with_font(&s_tc_ctrl_key[i],  s_font, KEYS[i],  black);
        update_text_with_font(&s_tc_ctrl_desc[i], s_font, DESCS[i], darkgrey);
    }

    draw_pause_blur(W, H);
    draw_rect(0.0f, 0.0f, W, H, 0.0f, 0.0f, 0.0f, 0.40f);

    /* Content background — same white as inactive menu buttons */
    draw_rect(panel_x, panel_y, CONTROLS_PANEL_W, content_h,
              1.0f, 1.0f, 1.0f, 0.72f);

    /* Title centred in title area */
    if (s_tc_ctrl_title.tex) {
        float tw = (float)MENU_TITLE_SIZE * (float)s_tc_ctrl_title.w / (float)s_tc_ctrl_title.h;
        draw_tex(&s_tc_ctrl_title,
                 panel_x + (CONTROLS_PANEL_W - tw) * 0.5f,
                 panel_y + (CONTROLS_TITLE_AREA - (float)MENU_TITLE_SIZE) * 0.5f,
                 (float)MENU_TITLE_SIZE);
    }

    /* Column divider at 42% of the padded interior — dark subtle line */
    float list_y    = panel_y + CONTROLS_TITLE_AREA;
    float inner_w   = CONTROLS_PANEL_W - 2.0f * CONTROLS_PAD;
    float div_x     = panel_x + CONTROLS_PAD + inner_w * 0.42f;
    float key_right = div_x - 10.0f;
    float desc_left = div_x + 10.0f;
    float rows_h    = (float)CONTROLS_COUNT * CONTROLS_ITEM_H
                    + (float)(CONTROLS_COUNT - 1) * CONTROLS_ITEM_GAP;
    draw_rect(div_x - 0.5f, list_y, 1.0f, rows_h, 0.0f, 0.0f, 0.0f, 0.18f);

    for (int i = 0; i < CONTROLS_COUNT; i++) {
        float row_y  = list_y + (float)i * (CONTROLS_ITEM_H + CONTROLS_ITEM_GAP);
        float text_y = row_y  + (CONTROLS_ITEM_H - CONTROLS_FONT_H) * 0.5f;

        /* Alternating row tint for readability */
        if (i % 2 == 0)
            draw_rect(panel_x, row_y, CONTROLS_PANEL_W, CONTROLS_ITEM_H,
                      0.0f, 0.0f, 0.0f, 0.05f);

        if (s_tc_ctrl_key[i].tex) {
            float tw = CONTROLS_FONT_H * (float)s_tc_ctrl_key[i].w / (float)s_tc_ctrl_key[i].h;
            draw_tex(&s_tc_ctrl_key[i], key_right - tw, text_y, CONTROLS_FONT_H);
        }
        if (s_tc_ctrl_desc[i].tex)
            draw_tex(&s_tc_ctrl_desc[i], desc_left, text_y, CONTROLS_FONT_H);
    }

    /* Return button — separate from content, 12 px gap below */
    int active = (s_pause_menu_selected == 0);
    draw_rect(panel_x, btn_y, CONTROLS_PANEL_W, CONTROLS_RETURN_H,
              1.0f, 1.0f, 1.0f, active ? 1.0f : 0.72f);
    if (s_tc_ctrl_return.tex) {
        float tw = (float)MENU_TEXT_SIZE * (float)s_tc_ctrl_return.w / (float)s_tc_ctrl_return.h;
        draw_tex(&s_tc_ctrl_return,
                 panel_x + (CONTROLS_PANEL_W - tw) * 0.5f,
                 btn_y + (CONTROLS_RETURN_H - (float)MENU_TEXT_SIZE) * 0.5f,
                 (float)MENU_TEXT_SIZE);
    }
}

int ui_controls_return_hit_test(int mouse_x, int mouse_y)
{
    float W = (float)WIN_W, H = (float)WIN_H;
    float panel_x = (W - CONTROLS_PANEL_W) * 0.5f;
    float panel_y = (H - CONTROLS_PANEL_H) * 0.5f;
    float btn_y   = panel_y + CONTROLS_CONTENT_H + 12.0f;
    float mx = (float)mouse_x, my = (float)mouse_y;
    return (mx >= panel_x && mx <= panel_x + CONTROLS_PANEL_W &&
            my >= btn_y   && my <= btn_y + CONTROLS_RETURN_H) ? 1 : 0;
}

int ui_pause_menu_hit_test(int mouse_x, int mouse_y)
{
    return pause_menu_item_at((float)mouse_x, (float)mouse_y);
}

/*
 * ui_render — draw the full HUD for the current frame.
 *
 * Draw order:
 *   1. Speed bar background + fill
 *   2. Movement speed text (centred below bar)
 *   3. Sim speed text (right-aligned below bar)
 *   4. FPS counter (top-right corner)
 *   5. Nearest body label (left-aligned below bar)
 *   6. Build bar (left panel, if build mode active)
 *   7. Pause menu (centre modal, if open)
 *
 * Speed bar fill fraction:
 *   In normal mode:  t = log(speed / CAM_MIN) / log(CAM_MAX / CAM_MIN)
 *   In warp mode:    t = log(speed / CAM_MAX) / log(WARP_MAX / CAM_MAX)
 *   Both map [min, max] → [0, 1] logarithmically.
 *
 * GL state: depth test disabled, depth writes off, alpha blending on for all
 * 2D drawing; restored to 3D defaults (depth test on, writes on) at end.
 */
void ui_render(void) {
    if (!s_shader || !s_font) return;

    const float W  = (float)WIN_W;
    const float TH = (float)FONT_SIZE;

    /* Log-normalised camera speed → fill fraction */
    float spd = g_cam.speed;
    float t;
    if (g_warp) {
        float ws = spd;
        if (ws < CAM_MAX)  ws = CAM_MAX;
        if (ws > WARP_MAX) ws = WARP_MAX;
        t = logf(ws / CAM_MAX) / logf(WARP_MAX / CAM_MAX);
    } else {
        if (spd < CAM_MIN) spd = CAM_MIN;
        if (spd > CAM_MAX) spd = CAM_MAX;
        t = logf(spd / CAM_MIN) / logf(CAM_MAX / CAM_MIN);
    }

    /* Format movement speed string — show WARP indicator when T is held */
    char mv_str[64];
    if (g_warp) {
        double ly_s = (double)g_cam.speed / (double)WARP_MAX;
        snprintf(mv_str, sizeof(mv_str), "WARP  %.4f ly/s", ly_s);
    } else if (spd < 0.001f) {
        snprintf(mv_str, sizeof(mv_str), "%.5f AU/s", (double)spd);
    } else if (spd < 1.0f) {
        snprintf(mv_str, sizeof(mv_str), "%.3f AU/s", (double)spd);
    } else {
        snprintf(mv_str, sizeof(mv_str), "%.2f AU/s", (double)spd);
    }

    /* Format sim speed string */
    char ss_str[64];
    int flashing = s_speed_flash_at != 0 &&
                   (SDL_GetTicks64() - s_speed_flash_at) < SPEED_FLASH_MS;
    if (g_paused && !flashing)
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

    /* FPS — exponential moving average, α=0.1 (≈ 10-frame window).
     * First frame bootstraps the EMA directly to the instantaneous value. */
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

    char nearest_str[64];
    nearest_body_distance_string(nearest_str, sizeof(nearest_str));

    SDL_Color white = {255, 255, 255, 220};
    update_text(&s_tc_move, mv_str, white);
    update_text(&s_tc_sim,  ss_str, white);
    update_text(&s_tc_fps,  fps_str, white);
    update_text(&s_tc_nearest, nearest_str, white);

    /* Layout */
    const float BH   = (float)BAR_H;
    const float BW   = W * BAR_W_FRAC;
    const float BX   = (W - BW) * 0.5f;     /* centred horizontally */
    const float BY   = BAR_TOP;
    const float TY   = BY + BH + TEXT_GAP;  /* text baseline below bar */

    /* GL state for 2D overlay */
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(s_shader);
    glUniform2f(s_loc_screen, (float)WIN_W, (float)WIN_H);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(s_vao);
    glBindBuffer(GL_ARRAY_BUFFER, s_vbo);

    if (s_pause_menu_visible) {
        if (s_pause_page == 1)
            draw_controls_page(W, (float)WIN_H);
        else
            draw_pause_menu(W, (float)WIN_H);
        glBindVertexArray(0);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        return;
    }

    /* Bar background (dim white), then fill */
    draw_rect(BX, BY, BW, BH, 1.0f, 1.0f, 1.0f, 0.15f);
    draw_rect(BX, BY, BW * t, BH, 1.0f, 1.0f, 1.0f, 0.85f);

    /* Movement speed — centred below bar */
    if (s_tc_move.tex) {
        float tw = TH * (float)s_tc_move.w / (float)s_tc_move.h;
        draw_tex(&s_tc_move, (W - tw) * 0.5f, TY, TH);
    }

    /* Sim speed — right-aligned to bar right edge */
    if (s_tc_sim.tex) {
        float tw = TH * (float)s_tc_sim.w / (float)s_tc_sim.h;
        draw_tex(&s_tc_sim, BX + BW - tw, TY, TH);
    }

    /* FPS — top-right corner, right-aligned to screen edge */
    if (s_tc_fps.tex) {
        const float MARGIN = 12.0f;
        float tw = TH * (float)s_tc_fps.w / (float)s_tc_fps.h;
        draw_tex(&s_tc_fps, W - tw - MARGIN, 8.0f, TH);
    }

    if (s_tc_nearest.tex) {
        draw_tex(&s_tc_nearest, BX, TY, TH);
    }

    draw_build_bar(W);
    if (!g_build_mode && g_inspect_mode)
        draw_bottom_mode_label(W, (float)WIN_H, "INSPECT", "click to inspect planet");

    /* Restore 3D state */
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void ui_shutdown(void) {
    if (s_tc_move.tex) glDeleteTextures(1, &s_tc_move.tex);
    if (s_tc_sim.tex)  glDeleteTextures(1, &s_tc_sim.tex);
    if (s_tc_fps.tex)  glDeleteTextures(1, &s_tc_fps.tex);
    if (s_tc_nearest.tex) glDeleteTextures(1, &s_tc_nearest.tex);
    if (s_tc_build_title.tex) glDeleteTextures(1, &s_tc_build_title.tex);
    if (s_tc_build_hint.tex)  glDeleteTextures(1, &s_tc_build_hint.tex);
    for (int i = 0; i < 8; i++)
        if (s_tc_build_items[i].tex) glDeleteTextures(1, &s_tc_build_items[i].tex);
    for (int i = 0; i < 7; i++)
        if (s_tc_pause_items[i].tex) glDeleteTextures(1, &s_tc_pause_items[i].tex);
    if (s_tc_arrow_l.tex)   glDeleteTextures(1, &s_tc_arrow_l.tex);
    if (s_tc_arrow_r.tex)   glDeleteTextures(1, &s_tc_arrow_r.tex);
    if (s_tc_arrow_l_w.tex) glDeleteTextures(1, &s_tc_arrow_l_w.tex);
    if (s_tc_arrow_r_w.tex) glDeleteTextures(1, &s_tc_arrow_r_w.tex);
    if (s_tc_ctrl_title.tex)  glDeleteTextures(1, &s_tc_ctrl_title.tex);
    if (s_tc_ctrl_return.tex) glDeleteTextures(1, &s_tc_ctrl_return.tex);
    for (int i = 0; i < CONTROLS_COUNT; i++) {
        if (s_tc_ctrl_key[i].tex)  glDeleteTextures(1, &s_tc_ctrl_key[i].tex);
        if (s_tc_ctrl_desc[i].tex) glDeleteTextures(1, &s_tc_ctrl_desc[i].tex);
    }
    if (s_vbo)  glDeleteBuffers(1, &s_vbo);
    if (s_vao)  glDeleteVertexArrays(1, &s_vao);
    if (s_shader) glDeleteProgram(s_shader);
    if (s_pause_blur_tex) glDeleteTextures(1, &s_pause_blur_tex);
    /* Close distinct font objects; shared fallback pointers must not be closed twice */
    if (s_menu_title_font && s_menu_title_font != s_menu_font && s_menu_title_font != s_font)
        TTF_CloseFont(s_menu_title_font);
    if (s_build_item_font && s_build_item_font != s_font)
        TTF_CloseFont(s_build_item_font);
    if (s_menu_font && s_menu_font != s_font)
        TTF_CloseFont(s_menu_font);
    if (s_font)   TTF_CloseFont(s_font);
    TTF_Quit();
    s_shader = s_vao = s_vbo = s_pause_blur_tex = 0;
    s_pause_blur_w = s_pause_blur_h = 0;
    s_font = NULL;
    s_build_item_font = NULL;
    s_menu_font = NULL;
    s_menu_title_font = NULL;
}
