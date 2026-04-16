/*
 * rings.c — Saturn particle ring system
 *
 * Physics
 * ───────
 * Each particle follows a Keplerian elliptical orbit around Saturn.
 * The CPU advances the mean anomaly  M += n·dt  each physics sub-step
 * (one float addition per particle, no trig).  The GPU then solves the
 * first-order Kepler equation every frame:
 *
 *   ν ≈ M + 2e·sin M          (true anomaly)
 *   r ≈ a·(1 − e·cos M)       (orbital radius)
 *
 * where n = √(GM_Saturn / a³) is pre-computed at init.  Inner particles
 * automatically orbit faster than outer ones (Kepler's 3rd law).
 *
 * Distribution fix
 * ────────────────
 * Uses a 32-bit XorShift PRNG (replaces rand() whose RAND_MAX=32767 on
 * MINGW causes visible banding).  Radius is sampled as
 *   a = √(a_min² + u·(a_max² – a_min²))   (u uniform in [0,1))
 * so particles are uniformly distributed by ring AREA, not by radius.
 *
 * LOD
 * ───
 *   dist > SPRITE_DIST  →  single flat sprite quad (ring_sprite.vert/frag)
 *   dist > LOD_DIST     →  2 000 particles
 *   dist ≤ LOD_DIST     →  25 000 particles
 *
 * Ring zones (km from Saturn centre):
 *   C ring:           74 658 –  92 000   dim grey
 *   B ring:           92 000 – 117 580   bright tan
 *   Cassini Division: 117 580 – 122 170  sparse gap
 *   A ring:           122 170 – 136 775  medium
 */
#include "rings.h"
#include "body.h"
#include "physics.h"
#include "gl_utils.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define SATURN_IDX       6
#define GM_SATURN_M3S2   3.7931e16   /* m³/s² */

#define SPRITE_DIST      0.2f        /* AU: beyond this use flat sprite         */
#define LOD_DIST         0.05f       /* AU: beyond this use reduced count       */

/* Outer edge of A ring in AU — sprite quad half-extent (× 1.05 margin) */
#define RING_SPRITE_R    (136775.0f / 149600000.0f * 1.05f)

/* Ring zone table */
typedef struct { float r_min; float r_max; float density; float r,g,b; } Zone;
static const Zone ZONES[] = {
    {  74658,  92000, 0.10f, 0.60f, 0.56f, 0.50f },  /* C */
    {  92000, 117580, 0.60f, 0.88f, 0.82f, 0.68f },  /* B */
    { 117580, 122170, 0.02f, 0.40f, 0.38f, 0.35f },  /* Cassini */
    { 122170, 136775, 0.28f, 0.78f, 0.73f, 0.60f },  /* A */
};
#define N_ZONES 4

/* ── XorShift32 PRNG ────────────────────────────────────────────────────── */
static uint32_t s_rng = 1;
static void  s_seed(uint32_t seed) { s_rng = seed ? seed : 1; }
static float s_randf(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    /* 24 high bits → uniform float in [0, 1) */
    return (float)(s_rng >> 8) * (1.0f / (float)(1 << 24));
}

/* ─────────────────────────────────────────────────────────────────────── */
static ParticleDisc s_ring;

/* Ring plane basis vectors (Saturn obliquity 26.73°) */
static float s_b1[3];    /* (1, 0, 0)                   */
static float s_b2[3];    /* (0,  sin(obl), −cos(obl))   */
static float s_pole[3];  /* (0,  cos(obl),  sin(obl))   b1 × b2 */

/* ─────────────────────────────────────────────────────────────────────── */
/*
 * bake_particles — fill data[8×n] and n_arr[n].
 *
 * data layout per particle: [M, a_au, e, omega, height, cr, cg, cb]
 */
static void bake_particles(float *data, float *n_arr, int n) {
    int idx = 0;
    for (int z = 0; z < N_ZONES; z++) {
        int cnt = (z == N_ZONES - 1) ? (n - idx)
                                      : (int)(ZONES[z].density * n);
        float r_min_au = ZONES[z].r_min / 1.496e8f;   /* km → AU */
        float r_max_au = ZONES[z].r_max / 1.496e8f;
        /* Area-correct bounds */
        float r2_min = r_min_au * r_min_au;
        float r2_max = r_max_au * r_max_au;

        for (int k = 0; k < cnt && idx < n; k++, idx++) {
            /* Area-uniform semi-major axis: sample r² uniformly */
            float a_au = sqrtf(r2_min + s_randf() * (r2_max - r2_min));
            /* Random initial mean anomaly */
            float M0    = s_randf() * 2.0f * (float)PI;
            /* Small eccentricity (Saturn ring particles: typically < 0.01) */
            float e     = s_randf() * 0.008f;
            /* Random argument of periapsis */
            float omega = s_randf() * 2.0f * (float)PI;
            /* Tiny vertical offset (±37 km expressed in AU) */
            float h     = (s_randf() - 0.5f) * 5e-7f;

            data[idx*8+0] = M0;
            data[idx*8+1] = a_au;
            data[idx*8+2] = e;
            data[idx*8+3] = omega;
            data[idx*8+4] = h;

            float j = 0.85f + 0.15f * s_randf();
            data[idx*8+5] = ZONES[z].r * j;
            data[idx*8+6] = ZONES[z].g * j;
            data[idx*8+7] = ZONES[z].b * j;

            /* Keplerian mean motion  n = √(GM / a³)  [rad/s] */
            double a_m = (double)a_au * 1.496e11;
            n_arr[idx] = (float)sqrt(GM_SATURN_M3S2 / (a_m * a_m * a_m));
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────── */
/*
 * VBO layout per particle (stride = 8 floats = 32 bytes):
 *   loc 0 — float  a_M      offset  0
 *   loc 1 — float  a_a      offset  4
 *   loc 2 — float  a_e      offset  8
 *   loc 3 — float  a_omega  offset 12
 *   loc 4 — float  a_height offset 16
 *   loc 5 — vec3   a_color  offset 20
 */
static void setup_particle_attribs(void) {
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(0*sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(1*sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(4*sizeof(float)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void*)(5*sizeof(float)));
}

static void init_disc_gl(ParticleDisc *d) {
    /* Particle shader */
    d->shader = gl_shader_load("assets/shaders/ring.vert",
                               "assets/shaders/color.frag");
    if (!d->shader) return;
    d->loc_vp     = glGetUniformLocation(d->shader, "u_vp");
    d->loc_center = glGetUniformLocation(d->shader, "u_center");
    d->loc_b1     = glGetUniformLocation(d->shader, "u_b1");
    d->loc_b2     = glGetUniformLocation(d->shader, "u_b2");
    d->loc_pole   = glGetUniformLocation(d->shader, "u_pole");

    /* Full LOD VBO */
    d->vao_full = gl_vao_create();
    d->vbo_full = gl_vbo_create(d->n_full * 8 * sizeof(float),
                                d->data_full, GL_DYNAMIC_DRAW);
    setup_particle_attribs();
    glBindVertexArray(0);

    /* Reduced LOD VBO */
    d->vao_lod = gl_vao_create();
    d->vbo_lod = gl_vbo_create(d->n_lod * 8 * sizeof(float),
                               d->data_lod, GL_DYNAMIC_DRAW);
    setup_particle_attribs();
    glBindVertexArray(0);

    /* Sprite shader */
    d->sprite_shader = gl_shader_load("assets/shaders/ring_sprite.vert",
                                      "assets/shaders/ring_sprite.frag");
    if (!d->sprite_shader) return;
    d->sp_loc_vp     = glGetUniformLocation(d->sprite_shader, "u_vp");
    d->sp_loc_center = glGetUniformLocation(d->sprite_shader, "u_center");
    d->sp_loc_b1     = glGetUniformLocation(d->sprite_shader, "u_b1");
    d->sp_loc_b2     = glGetUniformLocation(d->sprite_shader, "u_b2");

    /* Sprite quad — 6 vertices (2 triangles), positions updated each frame */
    d->sprite_vao = gl_vao_create();
    d->sprite_vbo = gl_vbo_create(6 * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glBindVertexArray(0);
}

/* ─────────────────────────────────────────────────────── public ─────── */
void rings_init(void) {
    float obl = 26.73f * (float)(PI / 180.0);
    s_b1[0]=1.0f; s_b1[1]=0.0f;          s_b1[2]=0.0f;
    s_b2[0]=0.0f; s_b2[1]= sinf(obl);    s_b2[2]=-cosf(obl);
    /* pole = b1 × b2 = (0, cos(obl), sin(obl)) */
    s_pole[0]=0.0f; s_pole[1]=cosf(obl); s_pole[2]=sinf(obl);

    s_ring.parent_idx = SATURN_IDX;
    s_ring.n_full     = 25000;
    s_ring.n_lod      = 2000;

    s_ring.data_full  = (float*)malloc(s_ring.n_full * 8 * sizeof(float));
    s_ring.data_lod   = (float*)malloc(s_ring.n_lod  * 8 * sizeof(float));
    s_ring.n_arr_full = (float*)malloc(s_ring.n_full * sizeof(float));
    s_ring.n_arr_lod  = (float*)malloc(s_ring.n_lod  * sizeof(float));

    if (!s_ring.data_full || !s_ring.data_lod ||
        !s_ring.n_arr_full || !s_ring.n_arr_lod) return;

    s_seed(1234);
    bake_particles(s_ring.data_full, s_ring.n_arr_full, s_ring.n_full);

    s_seed(5678);
    bake_particles(s_ring.data_lod,  s_ring.n_arr_lod,  s_ring.n_lod);

    init_disc_gl(&s_ring);
    s_ring.initialized = 1;
}

void rings_tick(double dt) {
    if (!s_ring.initialized) return;
    /* Use double precision for the increment to avoid cancellation at high
     * sim speeds, then wrap M into [0, 2π] so float32 never grows large
     * enough to lose the ~7 significant digits needed for accurate sin/cos. */
    const float TWO_PI = 6.28318530718f;
    for (int i = 0; i < s_ring.n_full; i++) {
        float M = s_ring.data_full[i*8+0]
                + (float)((double)s_ring.n_arr_full[i] * dt);
        if (M >= TWO_PI) M -= TWO_PI * (float)(int)(M / TWO_PI);
        s_ring.data_full[i*8+0] = M;
    }
    for (int i = 0; i < s_ring.n_lod; i++) {
        float M = s_ring.data_lod[i*8+0]
                + (float)((double)s_ring.n_arr_lod[i] * dt);
        if (M >= TWO_PI) M -= TWO_PI * (float)(int)(M / TWO_PI);
        s_ring.data_lod[i*8+0] = M;
    }
}

/* Build the flat sprite quad in the ring plane around Saturn */
static void build_sprite_quad(float *verts, float sx, float sy, float sz) {
    float R = RING_SPRITE_R;
    /* 4 corners: ±R along b1 and ±R along b2 */
    float c[4][3];
    for (int i = 0; i < 4; i++) {
        float sb1 = (i == 1 || i == 2) ?  R : -R;
        float sb2 = (i == 2 || i == 3) ?  R : -R;
        c[i][0] = sx + sb1*s_b1[0] + sb2*s_b2[0];
        c[i][1] = sy + sb1*s_b1[1] + sb2*s_b2[1];
        c[i][2] = sz + sb1*s_b1[2] + sb2*s_b2[2];
    }
    /* Triangle 0: C0, C1, C2 */
    memcpy(verts+0,  c[0], 12);
    memcpy(verts+3,  c[1], 12);
    memcpy(verts+6,  c[2], 12);
    /* Triangle 1: C0, C2, C3 */
    memcpy(verts+9,  c[0], 12);
    memcpy(verts+12, c[2], 12);
    memcpy(verts+15, c[3], 12);
}

void rings_render(const float vp[16]) {
    if (!s_ring.initialized) return;

    Body *saturn = &g_bodies[s_ring.parent_idx];
    float sx = (float)(saturn->pos[0] * RS);
    float sy = (float)(saturn->pos[1] * RS);
    float sz = (float)(saturn->pos[2] * RS);

    /* Camera distance to Saturn (AU) */
    float dx   = sx - (float)g_cam.pos[0];
    float dy   = sy - (float)g_cam.pos[1];
    float dz   = sz - (float)g_cam.pos[2];
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (dist > SPRITE_DIST) {
        /* ── Far LOD: flat sprite disc ─────────────────────────────── */
        if (!s_ring.sprite_shader) return;

        float quad[18];
        build_sprite_quad(quad, sx, sy, sz);

        glUseProgram(s_ring.sprite_shader);
        glUniformMatrix4fv(s_ring.sp_loc_vp,     1, GL_FALSE, vp);
        glUniform3f       (s_ring.sp_loc_center,  sx, sy, sz);
        glUniform3fv      (s_ring.sp_loc_b1,    1, s_b1);
        glUniform3fv      (s_ring.sp_loc_b2,    1, s_b2);

        glBindVertexArray(s_ring.sprite_vao);
        glBindBuffer(GL_ARRAY_BUFFER, s_ring.sprite_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quad), quad);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glDisable(GL_BLEND);
        glBindVertexArray(0);

    } else {
        /* ── Near LOD: Keplerian particle ring ─────────────────────── */
        if (!s_ring.shader) return;

        int    n   = (dist > LOD_DIST) ? s_ring.n_lod   : s_ring.n_full;
        float *data= (dist > LOD_DIST) ? s_ring.data_lod : s_ring.data_full;
        GLuint vao = (dist > LOD_DIST) ? s_ring.vao_lod  : s_ring.vao_full;
        GLuint vbo = (dist > LOD_DIST) ? s_ring.vbo_lod  : s_ring.vbo_full;

        glUseProgram(s_ring.shader);
        glUniformMatrix4fv(s_ring.loc_vp,     1, GL_FALSE, vp);
        glUniform3f       (s_ring.loc_center,  sx, sy, sz);
        glUniform3fv      (s_ring.loc_b1,    1, s_b1);
        glUniform3fv      (s_ring.loc_b2,    1, s_b2);
        glUniform3fv      (s_ring.loc_pole,  1, s_pole);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, n * 8 * sizeof(float), data);

        glEnable(GL_DEPTH_TEST);
        glPointSize(1.0f);
        glDrawArrays(GL_POINTS, 0, n);
        glBindVertexArray(0);
    }
}

void rings_shutdown(void) {
    free(s_ring.data_full);  free(s_ring.data_lod);
    free(s_ring.n_arr_full); free(s_ring.n_arr_lod);
    if (s_ring.vbo_full)     glDeleteBuffers(1, &s_ring.vbo_full);
    if (s_ring.vao_full)     glDeleteVertexArrays(1, &s_ring.vao_full);
    if (s_ring.vbo_lod)      glDeleteBuffers(1, &s_ring.vbo_lod);
    if (s_ring.vao_lod)      glDeleteVertexArrays(1, &s_ring.vao_lod);
    if (s_ring.shader)       glDeleteProgram(s_ring.shader);
    if (s_ring.sprite_vbo)   glDeleteBuffers(1, &s_ring.sprite_vbo);
    if (s_ring.sprite_vao)   glDeleteVertexArrays(1, &s_ring.sprite_vao);
    if (s_ring.sprite_shader)glDeleteProgram(s_ring.sprite_shader);
    memset(&s_ring, 0, sizeof(s_ring));
}
