/*
 * main.c — application entry point
 *
 * Responsibilities:
 *   - SDL2 window + OpenGL 3.3 Core context
 *   - GLEW initialisation
 *   - Module init / shutdown sequence
 *   - Main loop: event handling, physics step, camera update, render
 *
 * Camera controls:
 *   W/S        - move forward / backward
 *   A/D        - strafe left / right
 *   Q/E        - move down / up
 *   Mouse drag - look (yaw/pitch)
 *   Scroll     - speed multiplier (×1.3 per notch)
 *   Space      - pause / resume simulation
 *   R          - reset camera
 *   +/-        - simulation speed up / down
 *   T          - toggle warp mode (interstellar camera speed)
 *   B          - toggle build mode
 *   F11/Alt+↵  - toggle fullscreen
 *
 * Physics integration overview (see physics.c for full detail):
 *   The simulation uses a two-rate RESPA integrator with per-system timesteps.
 *   Each planetary system runs independently with its own dt_outer / dt_inner.
 *   The main loop caps the per-frame sim time to MAX_OUTER_STEPS × dt_outer to
 *   prevent spiral-of-death when the frame rate drops.
 *
 * Warp mode:
 *   Camera speed range normally is [0.00001, 200] AU/s. Pressing T switches to
 *   the warp range [200, 63241] AU/s (≈ [0.003, 1] ly/s). Warp does not affect
 *   the simulation clock; only camera movement speed changes.
 */
#include "common.h"
#include "math3d.h"
#include "body.h"
#include "universe.h"
#include "physics.h"
#include "camera.h"
#include "starfield.h"
#include "trails.h"
#include "labels.h"
#include "render.h"
#include "rings.h"
#include "asteroids.h"
#include "ui.h"
#include "build.h"
#include "inspect.h"
#include "collision.h"
#include "supernova.h"
#include "blackhole.h"
#include "audio.h"
#ifdef _OPENMP
#include <omp.h>
#endif

/* ── window / context ─────────────────────────────────────────────────────── */
static SDL_Window   *s_win = NULL;
static SDL_GLContext s_ctx = NULL;
int g_win_w = DEFAULT_WIN_W;
int g_win_h = DEFAULT_WIN_H;
static int s_fullscreen = 0;

/* ── input state ──────────────────────────────────────────────────────────── */
static int   s_freelook   = 0;       /* 1 = free-look active, mouse captured */
static float s_mouse_sens = 0.25f;   /* degrees per pixel */
static int   s_vsync_enabled = 1;
static float s_music_vol  = 0.6f;

/* ── pause menu ───────────────────────────────────────────────────────────── */
#define SLIDER_STEP    0.05f
#define MOUSE_SENS_MIN 0.05f
#define MOUSE_SENS_MAX 1.0f

static int s_pause_menu_open = 0;
static int s_pause_menu_selected = 0;
static int s_pause_menu_prev_paused = 0;
static int s_pause_page = 0;   /* 0 = main menu, 1 = controls */

enum {
    PAUSE_MENU_CONTINUE = 0,
    PAUSE_MENU_RESET_UNIVERSE,
    PAUSE_MENU_TOGGLE_VSYNC,
    PAUSE_MENU_CONTROLS,
    PAUSE_MENU_MUSIC_VOL,
    PAUSE_MENU_MOUSE_SENS,
    PAUSE_MENU_LEAVE,
    PAUSE_MENU_COUNT
};

/* Movement keys held */
static int s_key_w, s_key_s, s_key_a, s_key_d, s_key_q, s_key_e;

/* ── simulation speed table ───────────────────────────────────────────────── */
/* Clean display values in days/s. Index 4 (1.0 days/s) is the default.
 * The lowest non-zero entry (0.1 days/s) still lets the user watch fast orbiters.
 * The highest (365 days/s) runs one Earth year per real second. */
static const double SPEED_TABLE[] = {
    0.0,
    0.1, 0.25, 0.5,
    1.0, 2.0, 5.0, 10.0, 30.0, 60.0, 100.0,
    365.0
};
#define SPEED_TABLE_LEN (int)(sizeof(SPEED_TABLE)/sizeof(SPEED_TABLE[0]))
static int s_speed_idx = 4;   /* start at 1.0 days/s */

/* ── warp mode ────────────────────────────────────────────────────────────── */
/* Warp mode (T key) — variable interstellar camera speed.
 * Normal range : [0.00001, 200] AU/s  (0.00001 AU/s ≈ walking pace near Earth)
 * Warp range   : [200, 63241] AU/s    (63241 AU/s = 1 ly/s, i.e. full warp)
 * Pressing T clamps the current speed to the warp range and shows "WARP" in HUD. */
#define WARP_SPEED_MIN_AU    200.0f
#define WARP_SPEED_MAX_AU  63241.0f
int s_warp = 0;
int g_warp = 0;

/* ── logging helpers ──────────────────────────────────────────────────────── */
static void boot_log(const char *msg) {
    fprintf(stdout, "[Boot] %s\n", msg);
    fflush(stdout);
}

static void clear_movement_keys(void) {
    s_key_w = s_key_s = s_key_a = s_key_d = s_key_q = s_key_e = 0;
}

static void leave_inspect_keep_mouse(void) {
    inspect_cancel();
    s_freelook = 1;
    SDL_SetRelativeMouseMode(SDL_TRUE);
}

static int set_vsync(int enabled) {
    int interval = enabled ? 1 : 0;
    if (SDL_GL_SetSwapInterval(interval) != 0) {
        fprintf(stderr, "[Main] vsync toggle: %s\n", SDL_GetError());
        return 0;
    }
    s_vsync_enabled = enabled ? 1 : 0;
    fprintf(stdout, "[Main] V-Sync %s\n", s_vsync_enabled ? "ON" : "OFF");
    return 1;
}

static void set_music_vol(float vol) {
    if (vol < 0.0f) vol = 0.0f;
    if (vol > 1.0f) vol = 1.0f;
    s_music_vol = vol;
    audio_set_music_volume(vol);
}

static void adjust_mouse_sensitivity(float delta) {
    s_mouse_sens += delta;
    if (s_mouse_sens < MOUSE_SENS_MIN) s_mouse_sens = MOUSE_SENS_MIN;
    if (s_mouse_sens > MOUSE_SENS_MAX) s_mouse_sens = MOUSE_SENS_MAX;
}

static void sync_pause_menu_ui(void) {
    ui_set_pause_menu(s_pause_menu_open, s_pause_menu_selected, s_vsync_enabled,
                      s_music_vol, s_mouse_sens, s_pause_page);
}

static void open_controls_page(void) {
    s_pause_page = 1;
    s_pause_menu_selected = -1;
    sync_pause_menu_ui();
}

static void close_controls_page(void) {
    s_pause_page = 0;
    s_pause_menu_selected = PAUSE_MENU_CONTROLS;
    sync_pause_menu_ui();
}

static void close_pause_menu(int resume_freelook) {
    s_pause_menu_open = 0;
    g_paused = s_pause_menu_prev_paused;
    if (resume_freelook) {
        s_freelook = 1;
        SDL_SetRelativeMouseMode(SDL_TRUE);
    } else {
        s_freelook = 0;
        SDL_SetRelativeMouseMode(SDL_FALSE);
    }
    sync_pause_menu_ui();
}

static void open_pause_menu(void) {
    s_pause_menu_prev_paused = g_paused;
    s_pause_menu_open = 1;
    s_pause_menu_selected = PAUSE_MENU_CONTINUE;
    g_paused = 1;
    s_freelook = 0;
    SDL_SetRelativeMouseMode(SDL_FALSE);
    clear_movement_keys();
    sync_pause_menu_ui();
}

static void move_pause_menu_selection(int delta) {
    s_pause_menu_selected += delta;
    while (s_pause_menu_selected < 0) s_pause_menu_selected += PAUSE_MENU_COUNT;
    while (s_pause_menu_selected >= PAUSE_MENU_COUNT) s_pause_menu_selected -= PAUSE_MENU_COUNT;
    sync_pause_menu_ui();
}

/* ── pre-simulation (warm-up) ─────────────────────────────────────────────── */
/*
 * warmup_universe — pre-simulate 2 years of physics before the first frame.
 *
 * Without a warm-up, freshly loaded elliptical orbits start at t=0 (periapsis)
 * and moons/rings appear bunched up at the same orbital phase. Running 2 years
 * of sim time spreads bodies to realistic positions.
 *
 * Per-system parallelism (OpenMP):
 *   Each planetary system runs independently; systems share no state during
 *   integration. OpenMP distributes systems across threads. Progress is reported
 *   from a critical section so console output isn't interleaved.
 *
 * Timestep model:
 *   Each system uses its own outer and inner timestep limits, queried from the
 *   physics module. n_inner = ceil(dt_outer / dt_inner) ensures the inner loop
 *   always covers the full outer step. outer_total = floor(WARMUP_DT / dt_outer).
 *   Trails are ticked alongside the integrator so the ring buffer is populated;
 *   otherwise the first frame would show empty trails.
 *
 * physics_advance_time() updates the global simulation clock after all systems
 * are done. It must be called once, not once per system, to keep g_sim_time
 * consistent with all bodies' state.
 */
static void warmup_universe(void) {
    const double WARMUP_DT = 365.0 * 2.0 * DAY;
    int sys_n = physics_system_count();
    int completed = 0;
    fprintf(stdout, "[Boot] Warm-up: pre-simulating %.0f days across %d system%s\n",
            WARMUP_DT / DAY, sys_n, sys_n == 1 ? "" : "s");
    fflush(stdout);
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int s = 0; s < sys_n; s++) {
        double step_outer = physics_system_outer_dt_limit(s);
        double step_inner = physics_system_inner_dt_limit(s);
        int n_inner = (int)(step_outer / step_inner) + 1;
        int outer_total = (int)(WARMUP_DT / step_outer);
        int root = physics_system_root(s);
        for (int o = 0; o < outer_total; o++) {
            double dt_outer = WARMUP_DT / outer_total;
            double dt_inner = dt_outer / n_inner;
            physics_respa_begin_system(root, dt_outer);
            for (int i = 0; i < n_inner; i++) {
                physics_respa_inner_system(root, dt_inner);
            }
            physics_respa_end_system(root, dt_outer);
            trails_tick_system(root, dt_outer);
        }
#ifdef _OPENMP
#pragma omp critical
#endif
        {
            completed++;
            fprintf(stdout, "[Boot] Warm-up progress: %d/%d systems (%s)\n",
                    completed, sys_n, g_bodies[root].name);
            fflush(stdout);
        }
    }
    physics_advance_time(WARMUP_DT);
    boot_log("Warm-up complete");
}

/* ── world init / shutdown ────────────────────────────────────────────────── */
static void init_runtime_world(void) {
    boot_log("Preparing runtime world");
    universe_load("assets/universe.json");
    boot_log("Resetting camera");
    cam_reset();
    boot_log("Initializing starfield");
    starfield_init();
    boot_log("Initializing trails");
    trails_gl_init();
    boot_log("Initializing renderer");
    render_init();
    boot_log("Initializing rings");
    rings_init("assets/universe.json");
    boot_log("Initializing asteroid belts");
    asteroids_init("assets/universe.json");
    boot_log("Initializing labels");
    labels_init();
    boot_log("Initializing build mode");
    build_init();
    boot_log("Initializing inspect mode");
    inspect_init();
    boot_log("Refreshing physics timestep model");
    physics_refresh_timestep_model();
    warmup_universe();
    boot_log("Runtime world ready");
}

static void shutdown_runtime_world(void) {
    asteroids_shutdown();
    rings_shutdown();
    render_shutdown();
    labels_shutdown();
    trails_gl_shutdown();
    starfield_shutdown();
    universe_shutdown();
}

/* Tear down and rebuild everything (triggered by "Reset Universe" menu item). */
static void reset_universe_state(void) {
    shutdown_runtime_world();
    collision_reset();
    supernova_reset();
    clear_movement_keys();
    s_freelook = 0;
    s_warp = 0;
    g_warp = 0;
    s_speed_idx = 4;
    g_sim_time = 0.0;
    g_sim_speed = SPEED_TABLE[s_speed_idx] * DAY;
    g_paused = 0;
    s_pause_menu_open = 0;
    s_pause_menu_selected = PAUSE_MENU_CONTINUE;
    s_pause_menu_prev_paused = 0;
    SDL_SetRelativeMouseMode(SDL_FALSE);
    sync_pause_menu_ui();
    init_runtime_world();
}

/* ── init / quit ──────────────────────────────────────────────────────────── */
static void update_viewport_size(void) {
    int w = DEFAULT_WIN_W;
    int h = DEFAULT_WIN_H;
    if (s_win) SDL_GL_GetDrawableSize(s_win, &w, &h);
    if (w < 1) w = 1;
    if (h < 1) h = 1;
    g_win_w = w;
    g_win_h = h;
    glViewport(0, 0, g_win_w, g_win_h);
}

static void toggle_fullscreen(void) {
    Uint32 flags = s_fullscreen ? 0u : SDL_WINDOW_FULLSCREEN_DESKTOP;
    if (SDL_SetWindowFullscreen(s_win, flags) != 0) {
        fprintf(stderr, "[Main] fullscreen toggle: %s\n", SDL_GetError());
        return;
    }
    s_fullscreen = !s_fullscreen;
    update_viewport_size();
}

static int app_init(void) {
    boot_log("Initializing SDL");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        fprintf(stderr, "[Main] SDL_Init: %s\n", SDL_GetError());
        return 0;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,  24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    s_win = SDL_CreateWindow("OpenVerse Simulator",
                             SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             DEFAULT_WIN_W, DEFAULT_WIN_H,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN |
                             SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!s_win) {
        fprintf(stderr, "[Main] SDL_CreateWindow: %s\n", SDL_GetError());
        return 0;
    }

    {
        SDL_Surface *icon = SDL_LoadBMP("assets/window_icon.bmp");
        if (!icon) {
            fprintf(stderr, "[Main] window icon: %s\n", SDL_GetError());
        } else {
            SDL_SetWindowIcon(s_win, icon);
            SDL_FreeSurface(icon);
        }
    }

    s_ctx = SDL_GL_CreateContext(s_win);
    if (!s_ctx) {
        fprintf(stderr, "[Main] GL context: %s\n", SDL_GetError());
        return 0;
    }

    boot_log("Configuring swap interval");
    set_vsync(1);

    /* GLEW */
    boot_log("Initializing GLEW");
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        fprintf(stderr, "[Main] GLEW: %s\n", glewGetErrorString(err));
        return 0;
    }
    /* glewInit() spuriously sets GL_INVALID_ENUM on some drivers; flush it */
    glGetError();

    fprintf(stdout, "[Main] OpenGL %s | GLSL %s\n",
            glGetString(GL_VERSION),
            glGetString(GL_SHADING_LANGUAGE_VERSION));

    glEnable(GL_MULTISAMPLE);
    glClearColor(0.0f, 0.0f, 0.02f, 1.0f);
    update_viewport_size();
    boot_log("OpenGL context ready");

    boot_log("Initializing audio");
    audio_init();

    return 1;
}

static void app_quit(void) {
    audio_shutdown();
    ui_shutdown();
    shutdown_runtime_world();
    SDL_GL_DeleteContext(s_ctx);
    SDL_DestroyWindow(s_win);
    SDL_Quit();
}

/* ── event handling ───────────────────────────────────────────────────────── */
static void activate_pause_menu_action(int *running) {
    switch (s_pause_menu_selected) {
    case PAUSE_MENU_CONTINUE:
        close_pause_menu(1);
        break;
    case PAUSE_MENU_RESET_UNIVERSE:
        reset_universe_state();
        break;
    case PAUSE_MENU_TOGGLE_VSYNC:
        set_vsync(!s_vsync_enabled);
        sync_pause_menu_ui();
        break;
    case PAUSE_MENU_CONTROLS:
        open_controls_page();
        break;
    case PAUSE_MENU_MUSIC_VOL:
    case PAUSE_MENU_MOUSE_SENS:
        /* adjusted via left/right arrows or scroll, not Enter */
        break;
    case PAUSE_MENU_LEAVE:
        *running = 0;
        break;
    }
}

static void handle_event(const SDL_Event *e, float dt, int *running) {
    if (s_pause_menu_open && s_pause_page == 1) {
        /* ── controls page ── */
        switch (e->type) {
        case SDL_KEYDOWN:
            switch (e->key.keysym.sym) {
            case SDLK_ESCAPE:
            case SDLK_RETURN:
            case SDLK_KP_ENTER:
            case SDLK_SPACE:
                close_controls_page();
                break;
            default: break;
            }
            break;
        case SDL_MOUSEMOTION: {
            int hit = ui_controls_return_hit_test(e->motion.x, e->motion.y);
            int sel = hit ? 0 : -1;
            if (sel != s_pause_menu_selected) {
                s_pause_menu_selected = sel;
                sync_pause_menu_ui();
            }
        }   break;
        case SDL_MOUSEBUTTONDOWN:
            if (e->button.button == SDL_BUTTON_LEFT &&
                ui_controls_return_hit_test(e->button.x, e->button.y))
                close_controls_page();
            break;
        case SDL_WINDOWEVENT:
            if (e->window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
                e->window.event == SDL_WINDOWEVENT_RESIZED)
                update_viewport_size();
            break;
        default: break;
        }
        (void)dt; (void)running;
        return;
    }

    if (s_pause_menu_open) {
        switch (e->type) {
        case SDL_QUIT:
            break;

        case SDL_KEYDOWN:
            switch (e->key.keysym.sym) {
            case SDLK_ESCAPE:
                close_pause_menu(1);
                break;
            case SDLK_UP:
            case SDLK_w:
                if (!e->key.repeat) move_pause_menu_selection(-1);
                break;
            case SDLK_DOWN:
            case SDLK_s:
                if (!e->key.repeat) move_pause_menu_selection(1);
                break;
            case SDLK_LEFT:
                if (s_pause_menu_selected == PAUSE_MENU_MUSIC_VOL)
                    set_music_vol(s_music_vol - SLIDER_STEP);
                else if (s_pause_menu_selected == PAUSE_MENU_MOUSE_SENS)
                    adjust_mouse_sensitivity(-SLIDER_STEP);
                sync_pause_menu_ui();
                break;
            case SDLK_RIGHT:
                if (s_pause_menu_selected == PAUSE_MENU_MUSIC_VOL)
                    set_music_vol(s_music_vol + SLIDER_STEP);
                else if (s_pause_menu_selected == PAUSE_MENU_MOUSE_SENS)
                    adjust_mouse_sensitivity(SLIDER_STEP);
                sync_pause_menu_ui();
                break;
            case SDLK_RETURN:
            case SDLK_KP_ENTER:
            case SDLK_SPACE:
                if (!e->key.repeat) activate_pause_menu_action(running);
                break;
            default:
                break;
            }
            break;

        case SDL_MOUSEWHEEL:
            if (s_pause_menu_selected == PAUSE_MENU_MUSIC_VOL)
                set_music_vol(s_music_vol + e->wheel.y * SLIDER_STEP);
            else if (s_pause_menu_selected == PAUSE_MENU_MOUSE_SENS)
                adjust_mouse_sensitivity(e->wheel.y * SLIDER_STEP);
            sync_pause_menu_ui();
            break;

        case SDL_MOUSEMOTION:
        {
            int hover = ui_pause_menu_hit_test(e->motion.x, e->motion.y);
            if (hover != s_pause_menu_selected) {
                s_pause_menu_selected = hover;
                sync_pause_menu_ui();
            }
        }   break;

        case SDL_MOUSEBUTTONDOWN:
            if (e->button.button == SDL_BUTTON_LEFT) {
                int hover = ui_pause_menu_hit_test(e->button.x, e->button.y);
                if (hover >= 0) {
                    s_pause_menu_selected = hover;
                    sync_pause_menu_ui();
                    if (hover == PAUSE_MENU_MUSIC_VOL || hover == PAUSE_MENU_MOUSE_SENS) {
                        int delta = ui_pause_menu_slider_click_delta(e->button.x, e->button.y);
                        if (delta == -1) {
                            if (hover == PAUSE_MENU_MUSIC_VOL)
                                set_music_vol(s_music_vol - SLIDER_STEP);
                            else
                                adjust_mouse_sensitivity(-SLIDER_STEP);
                            sync_pause_menu_ui();
                        } else if (delta == 1) {
                            if (hover == PAUSE_MENU_MUSIC_VOL)
                                set_music_vol(s_music_vol + SLIDER_STEP);
                            else
                                adjust_mouse_sensitivity(SLIDER_STEP);
                            sync_pause_menu_ui();
                        }
                    } else {
                        activate_pause_menu_action(running);
                    }
                }
            }
            break;

        case SDL_WINDOWEVENT:
            if (e->window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
                e->window.event == SDL_WINDOWEVENT_RESIZED) {
                update_viewport_size();
            }
            break;

        default:
            break;
        }
        (void)dt;
        return;
    }

    switch (e->type) {
    case SDL_QUIT:
        /* handled in main loop */
        break;

    case SDL_KEYDOWN:
        switch (e->key.keysym.sym) {
        case SDLK_w:
            s_key_w = 1;
            if (g_inspect_orbit_mode) leave_inspect_keep_mouse();
            break;
        case SDLK_s:
            s_key_s = 1;
            if (g_inspect_orbit_mode) leave_inspect_keep_mouse();
            break;
        case SDLK_a:
            s_key_a = 1;
            if (g_inspect_orbit_mode) leave_inspect_keep_mouse();
            break;
        case SDLK_d:
            s_key_d = 1;
            if (g_inspect_orbit_mode) leave_inspect_keep_mouse();
            break;
        case SDLK_q:
            s_key_q = 1;
            if (g_inspect_orbit_mode) leave_inspect_keep_mouse();
            break;
        case SDLK_e:
            s_key_e = 1;
            if (g_inspect_orbit_mode) leave_inspect_keep_mouse();
            break;
        case SDLK_r: cam_reset(); break;
        case SDLK_F11:
            if (!e->key.repeat) toggle_fullscreen();
            break;
        case SDLK_RETURN:
            if (!e->key.repeat && (e->key.keysym.mod & KMOD_ALT))
                toggle_fullscreen();
            break;
        case SDLK_b:
            if (!e->key.repeat) {
                if (g_inspect_mode)
                    leave_inspect_keep_mouse();
                build_toggle();
            }
            break;
        case SDLK_i:
            if (!e->key.repeat) {
                if (g_build_mode) build_toggle();
                inspect_toggle();
                s_freelook = 1;
                SDL_SetRelativeMouseMode(SDL_TRUE);
            }
            break;
        case SDLK_TAB:
            build_set_tab_held(1);
            break;
        case SDLK_t:
            /* Toggle warp mode, clamping speed into the appropriate range. */
            s_warp = !s_warp;
            g_warp = s_warp;
            if (s_warp) {
                if (g_cam.speed < WARP_SPEED_MIN_AU) g_cam.speed = WARP_SPEED_MIN_AU;
                if (g_cam.speed > WARP_SPEED_MAX_AU) g_cam.speed = WARP_SPEED_MAX_AU;
            } else {
                if (g_cam.speed > WARP_SPEED_MIN_AU) g_cam.speed = WARP_SPEED_MIN_AU;
            }
            fprintf(stdout, "[Cam] warp %s (%.0f AU/s = %.4f ly/s)\n",
                    s_warp ? "ON" : "OFF",
                    (double)g_cam.speed,
                    (double)(g_cam.speed / WARP_SPEED_MAX_AU));
            break;
        case SDLK_SPACE:
            g_paused = !g_paused;
            break;
        case SDLK_ESCAPE:
            if (g_build_mode) {
                build_toggle();
                break;
            }
            if (g_inspect_mode) {
                leave_inspect_keep_mouse();
                clear_movement_keys();
                break;
            }
            if (s_freelook) {
                open_pause_menu();
            }
            break;
        case SDLK_EQUALS:
        case SDLK_PLUS:
            if (s_speed_idx < SPEED_TABLE_LEN - 1) s_speed_idx++;
            g_sim_speed = SPEED_TABLE[s_speed_idx] * DAY;
            ui_notify_speed_change();
            fprintf(stdout, "[Sim] speed = %g days/s\n",
                    SPEED_TABLE[s_speed_idx]);
            break;
        case SDLK_MINUS:
            if (s_speed_idx > 0) s_speed_idx--;
            g_sim_speed = SPEED_TABLE[s_speed_idx] * DAY;
            ui_notify_speed_change();
            fprintf(stdout, "[Sim] speed = %g days/s\n",
                    SPEED_TABLE[s_speed_idx]);
            break;
        }
        break;

    case SDL_KEYUP:
        switch (e->key.keysym.sym) {
        case SDLK_w: s_key_w = 0; break;
        case SDLK_s: s_key_s = 0; break;
        case SDLK_a: s_key_a = 0; break;
        case SDLK_d: s_key_d = 0; break;
        case SDLK_q: s_key_q = 0; break;
        case SDLK_e: s_key_e = 0; break;
        case SDLK_TAB: build_set_tab_held(0); break;
        }
        break;

    case SDL_MOUSEBUTTONDOWN:
        if (g_build_mode && e->button.button == SDL_BUTTON_LEFT) {
            build_place_current();
            break;
        }
        if (g_inspect_mode && e->button.button == SDL_BUTTON_LEFT) {
            inspect_begin_orbit();
            break;
        }
        if (e->button.button == SDL_BUTTON_LEFT && !s_freelook) {
            s_freelook = 1;
            SDL_SetRelativeMouseMode(SDL_TRUE);
        }
        break;

    case SDL_MOUSEMOTION:
        if (g_inspect_orbit_mode) {
            inspect_orbit_mouse(e->motion.xrel, e->motion.yrel, s_mouse_sens);
        } else if (s_freelook) {
            g_cam.yaw   += e->motion.xrel * s_mouse_sens;
            g_cam.pitch -= e->motion.yrel * s_mouse_sens;
            if (g_cam.pitch >  89.0f) g_cam.pitch =  89.0f;
            if (g_cam.pitch < -89.0f) g_cam.pitch = -89.0f;
        }
        break;

    case SDL_MOUSEWHEEL:
        if (g_inspect_orbit_mode) {
            inspect_orbit_zoom(e->wheel.y);
            break;
        }
        if (g_build_mode && g_build_tab_held) {
            build_scroll(e->wheel.y);
            break;
        }
        /* Speed steps by ×1.3 per notch; clamped to the active range. */
        g_cam.speed *= (e->wheel.y > 0) ? 1.3f : (1.0f / 1.3f);
        if (s_warp) {
            if (g_cam.speed < WARP_SPEED_MIN_AU) g_cam.speed = WARP_SPEED_MIN_AU;
            if (g_cam.speed > WARP_SPEED_MAX_AU) g_cam.speed = WARP_SPEED_MAX_AU;
        } else {
            if (g_cam.speed < 0.00001f)          g_cam.speed = 0.00001f;
            if (g_cam.speed > WARP_SPEED_MIN_AU) g_cam.speed = WARP_SPEED_MIN_AU;
        }
        break;

    case SDL_WINDOWEVENT:
        if (e->window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
            e->window.event == SDL_WINDOWEVENT_RESIZED) {
            update_viewport_size();
        }
        break;

    default:
        break;
    }
    (void)dt;
}

/* ── camera movement ──────────────────────────────────────────────────────── */
/*
 * camera_move — apply WASDQE movement each frame.
 *
 * Position is stored as double-precision AU. Speed is in AU/s. Delta is
 * computed as double so that small increments (slow camera near planets) are
 * not lost to float32 ULP at large absolute coordinates. The right vector is
 * derived from (forward × Y_up) projected onto the XZ plane — this keeps
 * Q/E purely vertical regardless of pitch.
 */
static void camera_move(float dt) {
    float fdx, fdy, fdz;
    cam_get_dir(&fdx, &fdy, &fdz);

    /* Right vector: cross(forward, world_up) projected to XZ */
    float rx = -fdz, rz = fdx;
    float rlen = sqrtf(rx*rx + rz*rz);
    if (rlen > 1e-6f) { rx /= rlen; rz /= rlen; }

    double dspd = (double)g_cam.speed * (double)dt;

    if (s_key_w) { g_cam.pos[0] += (double)fdx*dspd; g_cam.pos[1] += (double)fdy*dspd; g_cam.pos[2] += (double)fdz*dspd; }
    if (s_key_s) { g_cam.pos[0] -= (double)fdx*dspd; g_cam.pos[1] -= (double)fdy*dspd; g_cam.pos[2] -= (double)fdz*dspd; }
    if (s_key_d) { g_cam.pos[0] += (double)rx*dspd;                                      g_cam.pos[2] += (double)rz*dspd;  }
    if (s_key_a) { g_cam.pos[0] -= (double)rx*dspd;                                      g_cam.pos[2] -= (double)rz*dspd;  }
    if (s_key_e) { g_cam.pos[1] += dspd; }
    if (s_key_q) { g_cam.pos[1] -= dspd; }
}

/* ── main loop ────────────────────────────────────────────────────────────── */
/*
 * Physics integration loop (inside main):
 *
 * Per-frame sim time (sim_dt = g_sim_speed × dt_real) is capped to
 * MAX_OUTER_STEPS × dt_outer_max per system. This prevents the simulation
 * from freezing under low frame rates or high sim speeds.
 *
 * effective_sim_dt: the minimum of all systems' caps. All systems advance by
 * the same wall-clock step so g_sim_time stays consistent.
 *
 * Close-approach subdivision:
 *   collision_system_close_approach_subdivide() returns a factor > 1 when
 *   bodies in a system are near a collision threshold. outer_steps and dt_outer
 *   are refined by this factor, ensuring the collision detector sees smaller
 *   steps and can pinpoint the impact time accurately.
 *
 * Local encounter re-snapshot:
 *   collision_system_maybe_has_encounter() checks if any pair in the system
 *   is within encounter range for this outer step. If so, a new frame snapshot
 *   is taken (trails + positions) at the start of that step, so rollback on
 *   collision is accurate to within one dt_outer rather than one full frame.
 *
 * View matrices:
 *   'view'     — full lookAt including translation (float eye). Used only for
 *                ring rendering which is already in float world coordinates.
 *   'view_rot' — lookAt from the origin with direction only (no translation).
 *                Combined with proj to form vp_camrel = proj × view_rot, which
 *                is what render_frame() uses for all distant geometry. Avoids
 *                float32 cancellation at interstellar distances.
 */
int main(int argc, char **argv) {
    (void)argc; (void)argv;

    if (!app_init()) return 1;

    boot_log("Resetting collision state");
    collision_reset();
    boot_log("Resetting supernova state");
    supernova_reset();
    boot_log("Initializing UI");
    ui_init();
    sync_pause_menu_ui();
    init_runtime_world();

    /* Timing */
    Uint64 freq    = SDL_GetPerformanceFrequency();
    Uint64 prev    = SDL_GetPerformanceCounter();
    int    running = 1;

    while (running) {
        Uint64 now = SDL_GetPerformanceCounter();
        float  dt  = (float)((double)(now - prev) / (double)freq);
        if (dt > 0.1f) dt = 0.1f;  /* clamp: don't spiral if a frame takes > 100 ms */
        prev = now;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            handle_event(&e, dt, &running);
        }

        if (!s_pause_menu_open && !g_inspect_orbit_mode)
            camera_move(dt);

        /* Physics — RESPA hierarchical integrator */
        if (!g_paused && g_sim_speed > 0.0) {
            const int MAX_OUTER_STEPS = 120;
            physics_refresh_timestep_model();
            {
                int sys_n = physics_system_count();
                double sim_dt = g_sim_speed * dt;
                double effective_sim_dt = sim_dt;

                /* Find the most constrained system and cap all systems to it. */
                for (int s = 0; s < sys_n; s++) {
                    double dt_outer_max = physics_system_outer_dt_limit(s);
                    double sys_cap = dt_outer_max * MAX_OUTER_STEPS;
                    if (effective_sim_dt > sys_cap)
                        effective_sim_dt = sys_cap;
                }

                trails_begin_frame_snapshot();
                collision_snapshot_positions();
                for (int s = 0; s < sys_n; s++) {
                    double dt_outer_max = physics_system_outer_dt_limit(s);
                    double dt_inner_max = physics_system_inner_dt_limit(s);
                    int root = physics_system_root(s);
                    double sys_dt = effective_sim_dt;

                    int outer_steps = (int)(sys_dt / dt_outer_max) + 1;
                    double dt_outer = sys_dt / outer_steps;

                    /* Subdivide further if a close approach is detected */
                    int ca_factor = collision_system_close_approach_subdivide(root, dt_outer);
                    outer_steps *= ca_factor;
                    dt_outer    /= ca_factor;

                    int n_inner = (int)(dt_outer / dt_inner_max) + 1;
                    double dt_inner = dt_outer / n_inner;

                    for (int o = 0; o < outer_steps; o++) {
                        /* Re-snapshot at encounter onset for sub-frame rollback */
                        int local_encounter = collision_system_maybe_has_encounter(root, dt_outer);
                        if (local_encounter) {
                            trails_begin_frame_snapshot();
                            collision_snapshot_positions();
                        }
                        physics_respa_begin_system(root, dt_outer);
                        for (int i = 0; i < n_inner; i++) {
                            physics_respa_inner_system(root, dt_inner);
                        }
                        physics_respa_end_system(root, dt_outer);
                        rings_step_system(root, dt_outer);
                        trails_tick_system(root, dt_outer);
                        if (local_encounter) {
                            collision_step_system(root, dt_outer);
                        }
                    }
                }
                physics_advance_time(effective_sim_dt);
                supernova_step(effective_sim_dt);
                collision_step(effective_sim_dt);
                blackhole_step(effective_sim_dt);
                asteroids_step(effective_sim_dt);
                rings_tick(effective_sim_dt);
            }
        }

        if (!s_pause_menu_open && g_inspect_orbit_mode)
            inspect_orbit_update(dt);

        /* Build matrices.
         * view_rot: rotation-only lookAt (origin as eye). Used for all distant
         *           geometry via vp_camrel = proj × view_rot.
         * view:     full lookAt with float eye. Only used for ring rendering
         *           (rings.c operates in float world space).                  */
        Mat4 proj, view, view_rot;

        float aspect = (float)WIN_W / (float)WIN_H;
        mat4_perspective(proj, FOV, aspect, 0.0001f, 2000.0f);

        float fdx, fdy, fdz;
        cam_get_dir(&fdx, &fdy, &fdz);

        float up[3] = { 0.0f, 1.0f, 0.0f };
        float dir[3]  = { fdx, fdy, fdz };
        float zero3[3] = { 0.0f, 0.0f, 0.0f };
        mat4_lookAt(view_rot, zero3, dir, up);

        {
            float eye[3] = { (float)g_cam.pos[0], (float)g_cam.pos[1], (float)g_cam.pos[2] };
            float ctr[3] = { eye[0]+fdx,          eye[1]+fdy,          eye[2]+fdz          };
            mat4_lookAt(view, eye, ctr, up);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        render_frame(view, proj, view_rot, dt);
        ui_render();
        SDL_GL_SwapWindow(s_win);
    }

    app_quit();
    return 0;
}
