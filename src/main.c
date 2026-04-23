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
 *   W/S        — move forward / backward
 *   A/D        — strafe left / right
 *   Q/E        — move down / up
 *   Mouse drag — look (yaw/pitch)
 *   Scroll     — speed multiplier
 *   Space      — pause / resume simulation
 *   R          — reset camera
 *   +/-        — simulation speed × 2 / ÷ 2
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
#include "collision.h"
#ifdef _OPENMP
#include <omp.h>
#endif

/* ------------------------------------------------------------------ globals */
static SDL_Window   *s_win = NULL;
static SDL_GLContext s_ctx = NULL;
int g_win_w = DEFAULT_WIN_W;
int g_win_h = DEFAULT_WIN_H;
static int s_fullscreen = 0;

/* Mouse state */
static int   s_freelook   = 0;       /* 1 = Tab toggled free-look, mouse captured */
static float s_mouse_sens = 0.25f;   /* degrees per pixel */

/* Movement keys held */
static int s_key_w, s_key_s, s_key_a, s_key_d, s_key_q, s_key_e;

/* Sim-speed table — clean display values, 0.1 days/s as first non-zero step */
static const double SPEED_TABLE[] = {
    0.0,
    0.1, 0.25, 0.5,
    1.0, 2.0, 5.0, 10.0, 30.0, 60.0, 100.0,
    365.0
};
#define SPEED_TABLE_LEN (int)(sizeof(SPEED_TABLE)/sizeof(SPEED_TABLE[0]))
static int s_speed_idx = 4;   /* start at 1.0 days/s */

/* Warp mode (T key) — variable interstellar speed, adjustable via scroll wheel.
 * The speed range shifts from the normal [0.00001, 200] AU/s to the warp
 * range [200, 63241] AU/s (= [0.0032 ly/s, 1 ly/s]).                        */
#define WARP_SPEED_MIN_AU    200.0f   /* AU/s — lowest warp speed  (normal max) */
#define WARP_SPEED_MAX_AU  63241.0f   /* AU/s — highest warp speed (1 ly/s)     */
int s_warp = 0;            /* 0 = normal, 1 = warp engaged */
int g_warp = 0;            /* public mirror of s_warp for UI/other modules */


/* ------------------------------------------------------------------ init / quit */
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
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
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

    s_win = SDL_CreateWindow("verse",
                             SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             DEFAULT_WIN_W, DEFAULT_WIN_H,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN |
                             SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!s_win) {
        fprintf(stderr, "[Main] SDL_CreateWindow: %s\n", SDL_GetError());
        return 0;
    }

    s_ctx = SDL_GL_CreateContext(s_win);
    if (!s_ctx) {
        fprintf(stderr, "[Main] GL context: %s\n", SDL_GetError());
        return 0;
    }

    SDL_GL_SetSwapInterval(0);   /* vsync deactivated for debugging purpose - activate for production*/

    /* GLEW */
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

    return 1;
}

static void app_quit(void) {
    ui_shutdown();
    asteroids_shutdown();
    rings_shutdown();
    render_shutdown();
    labels_shutdown();
    trails_gl_shutdown();
    starfield_shutdown();
    universe_shutdown();
    SDL_GL_DeleteContext(s_ctx);
    SDL_DestroyWindow(s_win);
    SDL_Quit();
}

/* ------------------------------------------------------------------ event handling */
static void handle_event(const SDL_Event *e, float dt) {
    switch (e->type) {
    case SDL_QUIT:
        /* handled in main loop */
        break;

    case SDL_KEYDOWN:
        switch (e->key.keysym.sym) {
        case SDLK_w: s_key_w = 1; break;
        case SDLK_s: s_key_s = 1; break;
        case SDLK_a: s_key_a = 1; break;
        case SDLK_d: s_key_d = 1; break;
        case SDLK_q: s_key_q = 1; break;
        case SDLK_e: s_key_e = 1; break;
        case SDLK_r: cam_reset(); break;
        case SDLK_F11:
            if (!e->key.repeat) toggle_fullscreen();
            break;
        case SDLK_RETURN:
            if (!e->key.repeat && (e->key.keysym.mod & KMOD_ALT))
                toggle_fullscreen();
            break;
        case SDLK_b:
            if (!e->key.repeat) build_toggle();
            break;
        case SDLK_TAB:
            build_set_tab_held(1);
            break;
        case SDLK_t:
            s_warp = !s_warp;
            g_warp = s_warp;
            if (s_warp) {
                /* Entering warp: clamp speed into the warp range */
                if (g_cam.speed < WARP_SPEED_MIN_AU) g_cam.speed = WARP_SPEED_MIN_AU;
                if (g_cam.speed > WARP_SPEED_MAX_AU) g_cam.speed = WARP_SPEED_MAX_AU;
            } else {
                /* Leaving warp: clamp back to normal range */
                if (g_cam.speed > WARP_SPEED_MIN_AU) g_cam.speed = WARP_SPEED_MIN_AU;
            }
            fprintf(stdout, "[Cam] warp %s (%.0f AU/s = %.4f ly/s)\n",
                    s_warp ? "ON" : "OFF",
                    (double)g_cam.speed,
                    (double)(g_cam.speed / WARP_SPEED_MAX_AU));
            break;
        case SDLK_SPACE: g_paused = !g_paused; break;
        case SDLK_ESCAPE:
            if (g_build_mode) {
                build_toggle();
                break;
            }
            if (s_freelook) {
                s_freelook = 0;
                SDL_SetRelativeMouseMode(SDL_FALSE);
            }
            break;
        case SDLK_EQUALS: /* + key */
        case SDLK_PLUS:
            if (s_speed_idx < SPEED_TABLE_LEN - 1) s_speed_idx++;
            g_sim_speed = SPEED_TABLE[s_speed_idx] * DAY;
            fprintf(stdout, "[Sim] speed = %g days/s\n",
                    SPEED_TABLE[s_speed_idx]);
            break;
        case SDLK_MINUS:
            if (s_speed_idx > 0) s_speed_idx--;
            g_sim_speed = SPEED_TABLE[s_speed_idx] * DAY;
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
        if (e->button.button == SDL_BUTTON_LEFT && !s_freelook) {
            s_freelook = 1;
            SDL_SetRelativeMouseMode(SDL_TRUE);
        }
        break;

    case SDL_MOUSEMOTION:
        if (s_freelook) {
            /* xrel>0 = Maus nach rechts  → Kamera nach rechts → yaw steigt  */
            /* yrel>0 = Maus nach unten   → Kamera nach unten  → pitch sinkt  */
            g_cam.yaw   += e->motion.xrel * s_mouse_sens;
            g_cam.pitch -= e->motion.yrel * s_mouse_sens;
            if (g_cam.pitch >  89.0f) g_cam.pitch =  89.0f;
            if (g_cam.pitch < -89.0f) g_cam.pitch = -89.0f;
        }
        break;

    case SDL_MOUSEWHEEL:
        if (g_build_mode && g_build_tab_held) {
            build_scroll(e->wheel.y);
            break;
        }
        g_cam.speed *= (e->wheel.y > 0) ? 1.3f : (1.0f / 1.3f);
        if (s_warp) {
            /* Warp range: 200 AU/s (0.003 ly/s) … 63 241 AU/s (1 ly/s) */
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

/* ------------------------------------------------------------------ camera movement */
static void camera_move(float dt) {
    float fdx, fdy, fdz;
    cam_get_dir(&fdx, &fdy, &fdz);

    /* Right vector: cross(forward, world_up) */
    float rx = -fdz, rz = fdx;
    float rlen = sqrtf(rx*rx + rz*rz);
    if (rlen > 1e-6f) { rx /= rlen; rz /= rlen; }

    /* Compute delta in double so that pos (double) += tiny_delta never loses
     * the increment due to float32 ULP at large orbital distances.
     * Both normal and warp modes use g_cam.speed directly; the scroll wheel
     * and T-key clamp it to the appropriate range. */
    double dspd = (double)g_cam.speed * (double)dt;

    if (s_key_w) { g_cam.pos[0] += (double)fdx*dspd; g_cam.pos[1] += (double)fdy*dspd; g_cam.pos[2] += (double)fdz*dspd; }
    if (s_key_s) { g_cam.pos[0] -= (double)fdx*dspd; g_cam.pos[1] -= (double)fdy*dspd; g_cam.pos[2] -= (double)fdz*dspd; }
    if (s_key_d) { g_cam.pos[0] += (double)rx*dspd;                                      g_cam.pos[2] += (double)rz*dspd;  }
    if (s_key_a) { g_cam.pos[0] -= (double)rx*dspd;                                      g_cam.pos[2] -= (double)rz*dspd;  }
    if (s_key_e) { g_cam.pos[1] += dspd; }
    if (s_key_q) { g_cam.pos[1] -= dspd; }
}

/* ------------------------------------------------------------------ main */
int main(int argc, char **argv) {
    (void)argc; (void)argv;

    if (!app_init()) return 1;

    /* Module initialisation */
    universe_load("assets/universe.json");
    cam_reset();
    starfield_init();
    trails_gl_init();
    render_init();
    rings_init("assets/universe.json");
    asteroids_init("assets/universe.json");
    labels_init();
    ui_init();
    build_init();
    physics_refresh_timestep_model();

    /* Trail warm-up: pre-simulate 2 years using RESPA.
     * 730 outer steps × 50 inner steps = 36 500 inner steps total —
     * identical resolution to the old 0.02-day loop but ~20× faster
     * because slow forces are only evaluated at the outer (1-day) rate. */
    {
        const double WARMUP_DT = 365.0 * 2.0 * DAY;
        int sys_n = physics_system_count();
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
                    trails_tick_system(root, dt_inner);
                }
                physics_respa_end_system(root, dt_outer);
            }
        }
        physics_advance_time(WARMUP_DT);
    }

    /* Timing */
    Uint64 freq    = SDL_GetPerformanceFrequency();
    Uint64 prev    = SDL_GetPerformanceCounter();
    int    running = 1;

    while (running) {
        /* Delta time */
        Uint64 now = SDL_GetPerformanceCounter();
        float  dt  = (float)((double)(now - prev) / (double)freq);
        if (dt > 0.1f) dt = 0.1f;   /* clamp huge frames on breakpoint */
        prev = now;

        /* Events */
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            handle_event(&e, dt);
        }

        /* Camera */
        camera_move(dt);

        /* Physics — RESPA hierarchical integrator
         * Outer step ~1 day: slow forces (primary-primary + tidal).
         * Inner step 0.02 days: fast forces (parent-satellite dominant).
         * ~32× fewer pair evaluations than uniform 0.02-day stepping.
         *
         * sim_dt is capped to MAX_OUTER_STEPS outer steps so that a slow
         * frame cannot snowball into even-slower subsequent frames (the
         * lag-spiral).  At extreme sim speeds the simulation runs at a
         * reduced rate rather than making the UI unresponsive.
         *
         * trails_tick is called once per outer step (not every inner step).
         * The distance-based accumulator in trails_tick sums |v|*dt, so the
         * total distance is identical whether we call it N×(dt/N) or 1×dt —
         * this gives ~50× fewer sqrt calls with no change in sample spacing. */
        if (!g_paused && g_sim_speed > 0.0) {
            const int    MAX_OUTER_STEPS = 120;   /* per system cap */
            physics_refresh_timestep_model();
            {
                int sys_n = physics_system_count();
                double sim_dt = g_sim_speed * dt;
                double effective_sim_dt = sim_dt;

                for (int s = 0; s < sys_n; s++) {
                    double dt_outer_max = physics_system_outer_dt_limit(s);
                    double sys_cap = dt_outer_max * MAX_OUTER_STEPS;
                    if (effective_sim_dt > sys_cap)
                        effective_sim_dt = sys_cap;
                }

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
                for (int s = 0; s < sys_n; s++) {
                    double dt_outer_max = physics_system_outer_dt_limit(s);
                    double dt_inner_max = physics_system_inner_dt_limit(s);
                    int root = physics_system_root(s);
                    double sys_dt = effective_sim_dt;

                    int outer_steps = (int)(sys_dt / dt_outer_max) + 1;
                    double dt_outer = sys_dt / outer_steps;
                    int n_inner = (int)(dt_outer / dt_inner_max) + 1;
                    double dt_inner = dt_outer / n_inner;

                    for (int o = 0; o < outer_steps; o++) {
                        physics_respa_begin_system(root, dt_outer);
                        for (int i = 0; i < n_inner; i++) {
                            physics_respa_inner_system(root, dt_inner);
                            trails_tick_system(root, dt_inner);
                        }
                        physics_respa_end_system(root, dt_outer);
                    }
                }
                physics_advance_time(effective_sim_dt);
                collision_step(effective_sim_dt);
                asteroids_step(effective_sim_dt);
                rings_tick(effective_sim_dt);
            }
        }

        /* Build matrices */
        Mat4 proj, view, view_rot;

        float aspect = (float)WIN_W / (float)WIN_H;
        mat4_perspective(proj, FOV, aspect, 0.0001f, 2000.0f);

        float fdx, fdy, fdz;
        cam_get_dir(&fdx, &fdy, &fdz);

        float up[3] = { 0.0f, 1.0f, 0.0f };

        /* Rotation-only view matrix built with a ZERO origin.
         *
         * The naive approach (eye = (float)g_cam.pos, ctr = eye + dir) suffers
         * catastrophic cancellation at interstellar distances: at Barnard's Star
         * eye[1] ≈ −365 000 AU and the float32 ULP is 0.031 AU, larger than the
         * small direction component fdy = sin(1°) = 0.017.  The addition
         * fl32(−365 000 + 0.017) rounds back to −365 000, so (ctr − eye).y = 0
         * for several mouse pixels, then jumps a full ULP → stepwise rotation
         * jitter in ALL axes wherever the camera offset is large.
         *
         * Fix: use eye = [0,0,0] so the forward vector is computed directly from
         * [fdx, fdy, fdz] without any large-offset subtraction.  All body
         * rendering is already camera-relative (body_pos − g_cam.pos in double),
         * so no translation term is needed in this VP matrix anyway. */
        float dir[3]  = { fdx, fdy, fdz };
        float zero3[3] = { 0.0f, 0.0f, 0.0f };
        mat4_lookAt(view_rot, zero3, dir, up);

        /* Full view matrix with float eye — kept for ring rendering.
         * Rings are always near Sol so float32 is accurate enough there. */
        float eye[3] = { (float)g_cam.pos[0], (float)g_cam.pos[1], (float)g_cam.pos[2] };
        float ctr[3] = { eye[0]+fdx,          eye[1]+fdy,          eye[2]+fdz          };
        mat4_lookAt(view, eye, ctr, up);

        /* Render */
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        render_frame(view, proj, view_rot, dt);
        ui_render();
        SDL_GL_SwapWindow(s_win);
    }

    app_quit();
    return 0;
}
