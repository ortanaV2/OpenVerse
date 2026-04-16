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
#include "physics.h"
#include "camera.h"
#include "starfield.h"
#include "trails.h"
#include "labels.h"
#include "render.h"
#include "ui.h"

/* ------------------------------------------------------------------ globals */
static SDL_Window   *s_win = NULL;
static SDL_GLContext s_ctx = NULL;

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
    365.0, 730.0, 1825.0, 3650.0
};
#define SPEED_TABLE_LEN (int)(sizeof(SPEED_TABLE)/sizeof(SPEED_TABLE[0]))
static int s_speed_idx = 4;   /* start at 1.0 days/s */


/* ------------------------------------------------------------------ init / quit */
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
                             WIN_W, WIN_H,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (!s_win) {
        fprintf(stderr, "[Main] SDL_CreateWindow: %s\n", SDL_GetError());
        return 0;
    }

    s_ctx = SDL_GL_CreateContext(s_win);
    if (!s_ctx) {
        fprintf(stderr, "[Main] GL context: %s\n", SDL_GetError());
        return 0;
    }

    SDL_GL_SetSwapInterval(1);   /* vsync */

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

    return 1;
}

static void app_quit(void) {
    ui_shutdown();
    render_shutdown();
    labels_shutdown();
    trails_gl_shutdown();
    starfield_shutdown();
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
        case SDLK_SPACE: g_paused = !g_paused; break;
        case SDLK_ESCAPE:
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
        }
        break;

    case SDL_MOUSEBUTTONDOWN:
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
        g_cam.speed *= (e->wheel.y > 0) ? 1.3f : (1.0f / 1.3f);
        if (g_cam.speed < 0.00001f) g_cam.speed = 0.00001f;
        if (g_cam.speed > 200.0f)   g_cam.speed = 200.0f;
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

    float spd = g_cam.speed * dt;

    if (s_key_w) { g_cam.pos[0] += fdx*spd; g_cam.pos[1] += fdy*spd; g_cam.pos[2] += fdz*spd; }
    if (s_key_s) { g_cam.pos[0] -= fdx*spd; g_cam.pos[1] -= fdy*spd; g_cam.pos[2] -= fdz*spd; }
    if (s_key_d) { g_cam.pos[0] += rx*spd;                            g_cam.pos[2] += rz*spd;  }
    if (s_key_a) { g_cam.pos[0] -= rx*spd;                            g_cam.pos[2] -= rz*spd;  }
    if (s_key_e) { g_cam.pos[1] += spd; }
    if (s_key_q) { g_cam.pos[1] -= spd; }
}

/* ------------------------------------------------------------------ main */
int main(int argc, char **argv) {
    (void)argc; (void)argv;

    if (!app_init()) return 1;

    /* Module initialisation */
    solar_system_init();
    cam_reset();
    starfield_init();
    trails_gl_init();
    render_init();
    labels_init();
    ui_init();

    /* Trail warm-up: pre-simulate 2 years in 0.02-day steps.
     * trails_tick() uses per-body intervals so moons fill their buffers
     * with many orbits while planets accumulate ~730 daily samples. */
    {
        const double STEP        = DAY * 0.02;
        const int    TOTAL_STEPS = (int)(365.0 * 2.0 / 0.02);
        for (int i = 0; i < TOTAL_STEPS; i++) {
            physics_step(STEP);
            trails_tick(STEP);
        }
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

        /* Physics */
        if (!g_paused && g_sim_speed > 0.0) {
            double sim_dt = g_sim_speed * dt;
            /* Sub-step for stability: max 0.02 sim-days per step (~16
             * steps per Phobos orbit, the fastest body in the system). */
            int    steps  = 1;
            double step   = sim_dt;
            if (sim_dt > DAY * 0.02) {
                steps = (int)(sim_dt / (DAY * 0.02)) + 1;
                step  = sim_dt / steps;
            }
            for (int i = 0; i < steps; i++) {
                physics_step(step);
                trails_tick(step);
            }
        }

        /* Build matrices */
        Mat4 proj, view, view_rot;

        float aspect = (float)WIN_W / (float)WIN_H;
        mat4_perspective(proj, FOV, aspect, 0.0001f, 2000.0f);

        float fdx, fdy, fdz;
        cam_get_dir(&fdx, &fdy, &fdz);

        float eye[3] = { g_cam.pos[0], g_cam.pos[1], g_cam.pos[2] };
        float ctr[3] = { eye[0]+fdx,   eye[1]+fdy,   eye[2]+fdz   };
        float up[3]  = { 0.0f, 1.0f, 0.0f };
        mat4_lookAt(view, eye, ctr, up);
        mat4_strip_translation(view_rot, view);

        /* Render */
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        render_frame(view, proj, view_rot, dt);
        ui_render();
        SDL_GL_SwapWindow(s_win);
    }

    app_quit();
    return 0;
}
