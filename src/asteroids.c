/*
 * asteroids.c — Main Belt and Kuiper Belt
 *
 * Two rendering layers:
 *
 * 1. HAZE (GL_POINTS, additive, color.frag)
 *    Dense point cloud (8 000 / 4 000 points per belt).  Very dim, fades
 *    to zero within 0.05 AU of the camera so it doesn't fight the spheres.
 *    Shows belt structure from any distance.
 *
 * 2. SPHERES (instanced billboard ray-sphere, asteroid_ball.vert/frag)
 *    2 500 / 1 000 real objects with visually inflated radii so they are
 *    discoverable when flying through the belt.  Each sphere still uses
 *    the correct physical Phong lighting — just oversized on screen.
 *    CPU culls objects beyond cull_dist before upload.
 *
 * Orbital mechanics: first-order Kepler, mean anomaly advanced on CPU
 * each physics sub-step (ΔM = n·dt), position computed on GPU.
 */
#include "asteroids.h"
#include "body.h"
#include "camera.h"
#include "gl_utils.h"
#include "common.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define GM_SUN_SI  1.327124e20   /* m³/s² */

/* ── Per-asteroid storage ───────────────────────────────────────── */
typedef struct {
    float M;        /* mean anomaly (rad) — advanced each tick */
    float a;        /* semi-major axis (AU)                    */
    float e;
    float inc;      /* (rad) */
    float Omega;    /* (rad) */
    float omega;    /* (rad) */
    float radius;   /* physical radius (AU) — inflated for visuals */
    float bright;
    float n_rate;   /* mean motion (rad/s) */
} Asteroid;

/* ── Instance upload layout ─────────────────────────────────────── */
/* 2 × vec4 = 8 floats per sphere instance */
#define SINST  8
/* 2 × vec4 = 8 floats per haze point      */
#define HINST  8

/* ── XorShift32 ─────────────────────────────────────────────────── */
static uint32_t s_rng = 1;
static void  rng_seed(uint32_t s) { s_rng = s ? s : 1; }
static float rng_f(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng >> 8) * (1.0f / (float)(1 << 24));
}

/* ── Belt descriptor ────────────────────────────────────────────── */
typedef struct {
    /* Sphere layer */
    Asteroid *ast;
    int       n_ast;
    GLuint    sph_shader;
    GLint     sl_vp, sl_sun, sl_right, sl_up, sl_fwd, sl_color;
    GLuint    sph_quad_vao, sph_quad_vbo, sph_quad_ebo, sph_inst_vbo;
    float     cull_dist;   /* AU: sphere cull radius */

    /* Haze layer */
    float    *haze_data;   /* HINST floats × n_haze */
    float    *haze_n;      /* mean motion per haze point */
    int       n_haze;
    GLuint    haze_shader;
    GLint     hl_vp, hl_sun, hl_color, hl_fade_start, hl_fade_end, hl_cam_dist;
    float     haze_fade_start;  /* AU: begin fading           */
    float     haze_fade_end;    /* AU: fully invisible        */
    GLuint    haze_vao, haze_vbo;

    float     color[3];
    int       initialized;
} Belt;

static Belt s_main;
static Belt s_kuiper;

/* Shared sphere instance upload buffer */
static float *s_inst_buf;
static int    s_inst_cap;

/* ── helpers ────────────────────────────────────────────────────── */

static Asteroid *bake_spheres(int n,
                               float a_min, float a_max,
                               float e_max, float i_max_deg,
                               float r_min_km, float r_max_km,
                               float inflate_km,
                               uint32_t seed)
{
    Asteroid *ast = (Asteroid*)malloc(n * sizeof(Asteroid));
    if (!ast) return NULL;
    rng_seed(seed);
    float a2min = a_min*a_min, a2max = a_max*a_max;
    for (int i = 0; i < n; i++) {
        float a      = sqrtf(a2min + rng_f()*(a2max-a2min));
        float e      = rng_f()*e_max;
        float inc    = rng_f()*i_max_deg*(float)(PI/180.0);
        float Omega  = rng_f()*2.0f*(float)PI;
        float omega  = rng_f()*2.0f*(float)PI;
        float M0     = rng_f()*2.0f*(float)PI;
        float bright = rng_f();
        /* power-law radius: t² biases toward small objects */
        float t      = rng_f();
        float r_km   = r_min_km * powf(r_max_km/r_min_km, t*t);
        float r_au   = (r_km + inflate_km) / 149600000.0f;

        ast[i].M      = M0;
        ast[i].a      = a;
        ast[i].e      = e;
        ast[i].inc    = inc;
        ast[i].Omega  = Omega;
        ast[i].omega  = omega;
        ast[i].radius = r_au;
        ast[i].bright = bright;
        double am     = (double)a * AU;
        ast[i].n_rate = (float)sqrt(GM_SUN_SI/(am*am*am));
    }
    return ast;
}

static void bake_haze(float *data, float *narr, int n,
                      float a_min, float a_max,
                      float e_max, float i_max_deg,
                      uint32_t seed)
{
    rng_seed(seed);
    float a2min = a_min*a_min, a2max = a_max*a_max;
    for (int i = 0; i < n; i++) {
        float a      = sqrtf(a2min + rng_f()*(a2max-a2min));
        float e      = rng_f()*e_max;
        float inc    = rng_f()*i_max_deg*(float)(PI/180.0);
        float Omega  = rng_f()*2.0f*(float)PI;
        float omega  = rng_f()*2.0f*(float)PI;
        float M0     = rng_f()*2.0f*(float)PI;
        float bright = rng_f();
        data[i*HINST+0] = M0;
        data[i*HINST+1] = a;
        data[i*HINST+2] = e;
        data[i*HINST+3] = inc;
        data[i*HINST+4] = Omega;
        data[i*HINST+5] = omega;
        data[i*HINST+6] = 0.0f;
        data[i*HINST+7] = bright;
        double am     = (double)a * AU;
        narr[i] = (float)sqrt(GM_SUN_SI/(am*am*am));
    }
}

static void sph_setup_attribs(void) {
    /* loc 1: (M,a,e,inc)         instanced */
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE,
                          SINST*(int)sizeof(float), (void*)(0));
    glVertexAttribDivisor(1, 1);
    /* loc 2: (Omega,omega,R,bright) instanced */
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE,
                          SINST*(int)sizeof(float), (void*)(4*sizeof(float)));
    glVertexAttribDivisor(2, 1);
}

static void haze_setup_attribs(void) {
    int stride = HINST*(int)sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, stride, (void*)(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride, (void*)(4*sizeof(float)));
}

static int init_belt(Belt *b,
                     /* sphere params */
                     int n_sph,
                     float a_min, float a_max, float e_max, float i_max,
                     float r_min_km, float r_max_km, float inflate_km,
                     float cull_dist,
                     /* haze params */
                     int n_haze,
                     /* shared */
                     float cr, float cg, float cb,
                     uint32_t seed_sph, uint32_t seed_haze)
{
    b->color[0]=cr; b->color[1]=cg; b->color[2]=cb;
    b->cull_dist = cull_dist;

    /* ── Spheres ── */
    b->n_ast = n_sph;
    b->ast   = bake_spheres(n_sph, a_min,a_max, e_max,i_max,
                             r_min_km,r_max_km, inflate_km, seed_sph);
    if (!b->ast) return 0;

    b->sph_shader = gl_shader_load("assets/shaders/asteroid_ball.vert",
                                   "assets/shaders/asteroid_ball.frag");
    if (!b->sph_shader) { fprintf(stderr,"[Asteroids] sph shader failed\n"); return 0; }
    b->sl_vp    = glGetUniformLocation(b->sph_shader, "u_vp");
    b->sl_sun   = glGetUniformLocation(b->sph_shader, "u_sun");
    b->sl_right = glGetUniformLocation(b->sph_shader, "u_cam_right");
    b->sl_up    = glGetUniformLocation(b->sph_shader, "u_cam_up");
    b->sl_fwd   = glGetUniformLocation(b->sph_shader, "u_cam_fwd");
    b->sl_color = glGetUniformLocation(b->sph_shader, "u_color");
    glUseProgram(b->sph_shader);
    glUniform1f(glGetUniformLocation(b->sph_shader,"u_fov_tan"),
                tanf(FOV*0.5f*(float)(PI/180.0)));
    glUniform1f(glGetUniformLocation(b->sph_shader,"u_aspect"),
                (float)WIN_W/(float)WIN_H);
    glUseProgram(0);

    static const float qv[] = {-1,-1, 1,-1, 1,1, -1,1};
    static const unsigned qi[] = {0,1,2,0,2,3};

    b->sph_quad_vao = gl_vao_create();
    b->sph_quad_vbo = gl_vbo_create(sizeof(qv), qv, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
    b->sph_quad_ebo = gl_ebo_create(sizeof(qi), qi);
    b->sph_inst_vbo = gl_vbo_create(n_sph*SINST*sizeof(float), NULL, GL_DYNAMIC_DRAW);
    sph_setup_attribs();
    glBindVertexArray(0);

    /* ── Haze ── */
    b->n_haze    = n_haze;
    b->haze_data = (float*)malloc(n_haze*HINST*sizeof(float));
    b->haze_n    = (float*)malloc(n_haze*sizeof(float));
    if (!b->haze_data || !b->haze_n) return 0;
    bake_haze(b->haze_data, b->haze_n, n_haze,
              a_min,a_max, e_max,i_max, seed_haze);

    b->haze_shader = gl_shader_load("assets/shaders/asteroid_haze.vert",
                                    "assets/shaders/color.frag");
    if (!b->haze_shader) { fprintf(stderr,"[Asteroids] haze shader failed\n"); return 0; }
    b->hl_vp         = glGetUniformLocation(b->haze_shader, "u_vp");
    b->hl_sun        = glGetUniformLocation(b->haze_shader, "u_sun");
    b->hl_color      = glGetUniformLocation(b->haze_shader, "u_color");
    b->hl_fade_start = glGetUniformLocation(b->haze_shader, "u_fade_start");
    b->hl_fade_end   = glGetUniformLocation(b->haze_shader, "u_fade_end");
    b->hl_cam_dist   = glGetUniformLocation(b->haze_shader, "u_cam_dist_sun");

    /* fade distances are set by caller after init_belt returns */
    b->haze_vao = gl_vao_create();
    b->haze_vbo = gl_vbo_create(n_haze*HINST*sizeof(float),
                                b->haze_data, GL_DYNAMIC_DRAW);
    haze_setup_attribs();
    glBindVertexArray(0);

    b->initialized = 1;
    return 1;
}

/* ── public API ─────────────────────────────────────────────────── */

void asteroids_init(void) {
    memset(&s_main,   0, sizeof(s_main));
    memset(&s_kuiper, 0, sizeof(s_kuiper));

    /*
     * Main Belt  2.2–3.2 AU
     * Spheres : 2 500 objects, 3–250 km physical + 180 km inflate
     *           → minimum visible radius ~180 km
     *           cull beyond 2.0 AU from camera
     * Haze    : 8 000 points
     */
    init_belt(&s_main,
              2500, 2.2f,3.2f, 0.25f,20.0f, 3.0f,250.0f, 180.0f, 2.0f,
              8000,
              0.74f,0.70f,0.64f,
              0xA1B2C3D4u, 0xF0E1D2C3u);
    /* Fade: start at 7 AU from sun, gone at 9.5 AU (Saturn distance) */
    s_main.haze_fade_start = 7.0f;
    s_main.haze_fade_end   = 9.5f;

    /*
     * Kuiper Belt  39–48 AU
     * Spheres : 1 000 objects, 50–800 km physical + 2 000 km inflate
     *           cull beyond 8.0 AU from camera
     * Haze    : 4 000 points
     */
    init_belt(&s_kuiper,
              1000, 39.0f,48.0f, 0.15f,20.0f, 50.0f,800.0f, 2000.0f, 8.0f,
              4000,
              0.78f,0.78f,0.82f,
              0x11223344u, 0x55667788u);
    /* Fade: start at 50 AU from sun, gone at 65 AU */
    s_kuiper.haze_fade_start = 50.0f;
    s_kuiper.haze_fade_end   = 65.0f;

    int cap = s_main.n_ast > s_kuiper.n_ast ? s_main.n_ast : s_kuiper.n_ast;
    s_inst_buf = (float*)malloc(cap*SINST*sizeof(float));
    s_inst_cap = cap;
}

void asteroids_tick(double dt) {
    const float TWO_PI = 6.28318530718f;

    Belt *belts[2] = {&s_main, &s_kuiper};
    for (int b = 0; b < 2; b++) {
        Belt *belt = belts[b];
        if (!belt->initialized) continue;

        /* Advance spheres */
        for (int i = 0; i < belt->n_ast; i++) {
            float M = belt->ast[i].M + (float)((double)belt->ast[i].n_rate*dt);
            if (M >= TWO_PI) M -= TWO_PI*(float)(int)(M/TWO_PI);
            belt->ast[i].M = M;
        }
        /* Advance haze */
        for (int i = 0; i < belt->n_haze; i++) {
            float M = belt->haze_data[i*HINST]
                    + (float)((double)belt->haze_n[i]*dt);
            if (M >= TWO_PI) M -= TWO_PI*(float)(int)(M/TWO_PI);
            belt->haze_data[i*HINST] = M;
        }
    }
}

static void render_belt(Belt *b, const float vp[16],
                        float sx, float sy, float sz,
                        const float right[3], const float up[3],
                        const float fwd[3])
{
    if (!b->initialized) return;

    /* ── Haze pass (GL_POINTS, additive) ── */
    if (b->haze_shader) {
        /* Camera distance from sun: sun is camera-relative at (sx,sy,sz),
         * so |sun_vec| = distance from camera to sun = cam dist from sun. */
        float cam_dist_sun = sqrtf(sx*sx + sy*sy + sz*sz);

        glUseProgram(b->haze_shader);
        glUniformMatrix4fv(b->hl_vp,         1, GL_FALSE, vp);
        glUniform3f       (b->hl_sun,        sx, sy, sz);
        glUniform3f       (b->hl_color,      b->color[0], b->color[1], b->color[2]);
        glUniform1f       (b->hl_fade_start, b->haze_fade_start);
        glUniform1f       (b->hl_fade_end,   b->haze_fade_end);
        glUniform1f       (b->hl_cam_dist,   cam_dist_sun);

        glBindVertexArray(b->haze_vao);
        glBindBuffer(GL_ARRAY_BUFFER, b->haze_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        b->n_haze*HINST*sizeof(float), b->haze_data);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);   /* additive */
        glPointSize(1.0f);
        glDrawArrays(GL_POINTS, 0, b->n_haze);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glBindVertexArray(0);
    }

    /* ── Sphere pass (instanced, depth-writing) ── */
    if (!b->sph_shader || !s_inst_buf) return;

    float cd2 = b->cull_dist * b->cull_dist;
    int   inst_n = 0;

    for (int i = 0; i < b->n_ast; i++) {
        Asteroid *a = &b->ast[i];
        /* Fast circular cull (ignores e/inc/Ω/ω) */
        float ax = sx + a->a * cosf(a->M);
        float az = sz + a->a * sinf(a->M);
        float dy = sy;
        if (ax*ax + dy*dy + az*az > cd2) continue;

        float *d = s_inst_buf + inst_n*SINST;
        d[0]=a->M; d[1]=a->a; d[2]=a->e;    d[3]=a->inc;
        d[4]=a->Omega; d[5]=a->omega; d[6]=a->radius; d[7]=a->bright;
        inst_n++;
    }
    if (inst_n == 0) return;

    glBindVertexArray(b->sph_quad_vao);
    glBindBuffer(GL_ARRAY_BUFFER, b->sph_inst_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, inst_n*SINST*sizeof(float), s_inst_buf);

    glUseProgram(b->sph_shader);
    glUniformMatrix4fv(b->sl_vp,    1, GL_FALSE, vp);
    glUniform3f       (b->sl_sun,   sx, sy, sz);
    glUniform3f       (b->sl_right, right[0], right[1], right[2]);
    glUniform3f       (b->sl_up,    up[0],    up[1],    up[2]);
    glUniform3f       (b->sl_fwd,   fwd[0],   fwd[1],   fwd[2]);
    glUniform3f       (b->sl_color, b->color[0], b->color[1], b->color[2]);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, inst_n);
    glBindVertexArray(0);
}

void asteroids_render(const float vp_camrel[16],
                      float cam_x, float cam_y, float cam_z,
                      const float cam_right[3], const float cam_up[3],
                      const float cam_fwd[3])
{
    float sx = (float)(g_bodies[0].pos[0]*RS) - cam_x;
    float sy = (float)(g_bodies[0].pos[1]*RS) - cam_y;
    float sz = (float)(g_bodies[0].pos[2]*RS) - cam_z;

    render_belt(&s_main,   vp_camrel, sx,sy,sz, cam_right,cam_up,cam_fwd);
    render_belt(&s_kuiper, vp_camrel, sx,sy,sz, cam_right,cam_up,cam_fwd);
}

void asteroids_shutdown(void) {
    free(s_inst_buf); s_inst_buf=NULL;

    Belt *belts[2] = {&s_main, &s_kuiper};
    for (int b = 0; b < 2; b++) {
        Belt *belt = belts[b];
        free(belt->ast);
        free(belt->haze_data);
        free(belt->haze_n);
        if (belt->sph_inst_vbo)  glDeleteBuffers(1,      &belt->sph_inst_vbo);
        if (belt->sph_quad_ebo)  glDeleteBuffers(1,      &belt->sph_quad_ebo);
        if (belt->sph_quad_vbo)  glDeleteBuffers(1,      &belt->sph_quad_vbo);
        if (belt->sph_quad_vao)  glDeleteVertexArrays(1, &belt->sph_quad_vao);
        if (belt->sph_shader)    glDeleteProgram(belt->sph_shader);
        if (belt->haze_vbo)      glDeleteBuffers(1,      &belt->haze_vbo);
        if (belt->haze_vao)      glDeleteVertexArrays(1, &belt->haze_vao);
        if (belt->haze_shader)   glDeleteProgram(belt->haze_shader);
    }
    memset(&s_main,   0, sizeof(s_main));
    memset(&s_kuiper, 0, sizeof(s_kuiper));
}
