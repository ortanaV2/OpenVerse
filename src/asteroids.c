/*
 * asteroids.c — Main Belt and Kuiper Belt, gravity-integrated GL_POINTS
 *
 * Each particle is a true test particle:
 *   - Feels gravitational acceleration from all non-moon bodies in g_bodies[]
 *     (Sun + planets + dwarf planets, ~14 bodies — moons are negligible).
 *   - Exerts no force back (mass << planet mass).
 *   - Integrated with symplectic Euler at the outer RESPA step rate (~1 day).
 *
 * Rendering:
 *   GL_POINTS via asteroid_particle.vert + color.frag.
 *   Positions uploaded camera-relative each frame.
 *   Point size grows 1→3 px as camera approaches; smooth near/far fades.
 *   Additive blending so overlapping points don't saturate to solid white.
 *
 * Gravity optimisations (coarser but fast):
 *   1. Only loop over bodies with parent < 0 (no moons) — ~14 vs 33 bodies.
 *      Saves ~2.5× evaluations; moons contribute < 0.001% of asteroid accel.
 *   2. Major-body positions are copied into a small stack cache before the
 *      inner loop so every access hits L1 cache instead of g_bodies[].
 *   3. Force accumulation in float (positions/velocities stay double).
 *   4. Softening omitted — asteroid–planet closest approaches are >> 1e5 m.
 */
#include "asteroids.h"
#include "body.h"
#include "camera.h"
#include "physics.h"
#include "gl_utils.h"
#include "json.h"
#include "common.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define GM_SUN_SI  1.327124e20   /* m³/s² */
#define MAX_MAJOR  32            /* upper bound for major-body cache */

/* ── Particle ───────────────────────────────────────────────────── */
typedef struct {
    double pos[3];   /* metres, same frame as g_bodies  */
    double vel[3];   /* m/s                             */
    float  bright;   /* per-particle brightness [0, 1]  */
} Particle;

/* ── Upload buffer: 4 floats per particle ───────────────────────── */
/* (cam_rel_x, cam_rel_y, cam_rel_z, bright)                        */
#define PFP 4

/* ── XorShift32 ─────────────────────────────────────────────────── */
static uint32_t s_rng = 1;
static void  rng_seed(uint32_t s) { s_rng = s ? s : 1; }
static float rng_f(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng >> 8) * (1.0f / (float)(1u << 24));
}

/* ── Kepler solver ──────────────────────────────────────────────── */
static double kepler_E(double M, double e) {
    double E = M;
    for (int k = 0; k < 50; k++) {
        double dE = (M - E + e*sin(E)) / (1.0 - e*cos(E));
        E += dE;
        if (fabs(dE) < 1e-12) break;
    }
    return E;
}

/* Keplerian elements → SI state, heliocentric ecliptic → GL frame,
 * then shifted to g_bodies[] origin by adding Sun pos/vel.          */
static void kepler_to_state(double a_au, double e,
                             double i, double Om, double om, double M0,
                             double pos[3], double vel[3])
{
    double a  = a_au * AU;
    double E  = kepler_E(M0, e);
    double nu = 2.0*atan2(sqrt(1.0+e)*sin(E*0.5), sqrt(1.0-e)*cos(E*0.5));
    double r  = a*(1.0-e*cos(E));
    double h  = sqrt(GM_SUN_SI*a*(1.0-e*e));

    double xp = r*cos(nu),  yp = r*sin(nu);
    double vxp= -(GM_SUN_SI/h)*sin(nu);
    double vyp=  (GM_SUN_SI/h)*(e+cos(nu));

    double cO=cos(Om),sO=sin(Om), co=cos(om),so=sin(om);
    double ci=cos(i), si=sin(i);
    double Px=cO*co-sO*so*ci, Qx=-cO*so-sO*co*ci;
    double Py=sO*co+cO*so*ci, Qy=-sO*so+cO*co*ci;
    double Pz=so*si,           Qz= co*si;

    double ex=Px*xp+Qx*yp, ey=Py*xp+Qy*yp, ez=Pz*xp+Qz*yp;
    double evx=Px*vxp+Qx*vyp, evy=Py*vxp+Qy*vyp, evz=Pz*vxp+Qz*vyp;

    /* ecliptic → GL (Y=north=eclZ, Z=east=eclY) + Sun offset */
    pos[0] = ex  + g_bodies[0].pos[0];
    pos[1] = ez  + g_bodies[0].pos[1];
    pos[2] = ey  + g_bodies[0].pos[2];
    vel[0] = evx + g_bodies[0].vel[0];
    vel[1] = evz + g_bodies[0].vel[1];
    vel[2] = evy + g_bodies[0].vel[2];
}

/* ── Belt ───────────────────────────────────────────────────────── */
typedef struct {
    Particle *p;
    int       n;

    GLuint shader;
    GLint  loc_vp, loc_color, loc_fade_start, loc_fade_end;
    GLuint vao, vbo;   /* dynamic upload */

    float  color[3];
    float  fade_start;   /* AU: begin fading (camera distance) */
    float  fade_end;     /* AU: fully gone  (camera distance)  */

    int    initialized;
} Belt;

static Belt  *s_belts      = NULL;
static int    s_n_belts    = 0;
static float *s_upload_buf = NULL;   /* PFP × max(n) across all belts */

/* ── bake ───────────────────────────────────────────────────────── */
static Particle *bake(int n,
                      float a_min, float a_max,
                      float e_max, float i_max_deg,
                      uint32_t seed)
{
    Particle *p = (Particle*)malloc(n * sizeof(Particle));
    if (!p) return NULL;
    rng_seed(seed);
    float a2min=a_min*a_min, a2max=a_max*a_max;
    for (int i = 0; i < n; i++) {
        double a  = sqrt(a2min + rng_f()*(a2max-a2min));
        double e  = rng_f()*e_max;
        double ii = rng_f()*i_max_deg*(PI/180.0);
        double Om = rng_f()*2.0*PI;
        double om = rng_f()*2.0*PI;
        double M0 = rng_f()*2.0*PI;
        kepler_to_state(a, e, ii, Om, om, M0, p[i].pos, p[i].vel);
        p[i].bright = rng_f();
    }
    return p;
}

/* ── GL init ────────────────────────────────────────────────────── */
static int init_belt_gl(Belt *b)
{
    b->shader = gl_shader_load("assets/shaders/asteroid_particle.vert",
                               "assets/shaders/color.frag");
    if (!b->shader) { fprintf(stderr,"[Asteroids] shader failed\n"); return 0; }

    b->loc_vp         = glGetUniformLocation(b->shader, "u_vp");
    b->loc_color      = glGetUniformLocation(b->shader, "u_color");
    b->loc_fade_start = glGetUniformLocation(b->shader, "u_fade_start");
    b->loc_fade_end   = glGetUniformLocation(b->shader, "u_fade_end");

    b->vao = gl_vao_create();
    b->vbo = gl_vbo_create(b->n * PFP * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    /* layout: loc0 = vec3 pos, loc1 = float bright */
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          PFP*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE,
                          PFP*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);

    b->initialized = 1;
    return 1;
}

/* ── public ─────────────────────────────────────────────────────── */

void asteroids_init(const char *path)
{
    JsonNode *root = json_parse_file(path);
    if (!root) {
        fprintf(stderr, "[Asteroids] cannot parse '%s'\n", path);
        return;
    }

    JsonNode *belts_arr = json_get(root, "asteroid_belts");
    if (!belts_arr || belts_arr->type != JSON_ARRAY) {
        fprintf(stderr, "[Asteroids] no 'asteroid_belts' in '%s' — skipping\n", path);
        json_free(root);
        return;
    }

    /* Count */
    int count = 0;
    { JsonNode *n = belts_arr->first_child; while (n) { count++; n = n->next; } }
    if (count == 0) { json_free(root); return; }

    s_belts   = (Belt*)calloc(count, sizeof(Belt));
    s_n_belts = 0;
    if (!s_belts) { json_free(root); return; }

    int max_n = 0;

    JsonNode *bnode = belts_arr->first_child;
    while (bnode) {
        int   n           = (int)json_num(json_get(bnode, "n_particles"),  1000);
        float a_min       = (float)json_num(json_get(bnode, "a_min_au"),    2.0);
        float a_max       = (float)json_num(json_get(bnode, "a_max_au"),    3.0);
        float e_max       = (float)json_num(json_get(bnode, "e_max"),       0.2);
        float i_max       = (float)json_num(json_get(bnode, "i_max_deg"),  20.0);
        uint32_t seed     = (uint32_t)json_num(json_get(bnode, "seed"),    1234);
        float fade_start  = (float)json_num(json_get(bnode, "fade_start_au"), 5.0);
        float fade_end    = (float)json_num(json_get(bnode, "fade_end_au"),  10.0);
        JsonNode *col     = json_get(bnode, "color");

        Belt *b = &s_belts[s_n_belts++];
        b->n          = n;
        b->color[0]   = (float)json_num(json_idx(col, 0), 0.6);
        b->color[1]   = (float)json_num(json_idx(col, 1), 0.6);
        b->color[2]   = (float)json_num(json_idx(col, 2), 0.6);
        b->fade_start = fade_start;
        b->fade_end   = fade_end;
        b->p = bake(n, a_min, a_max, e_max, i_max, seed);
        if (b->p) init_belt_gl(b);
        if (n > max_n) max_n = n;

        bnode = bnode->next;
    }

    if (max_n > 0)
        s_upload_buf = (float*)malloc(max_n * PFP * sizeof(float));

    json_free(root);
}

/*
 * asteroids_step — symplectic Euler, major bodies only.
 *
 * Optimisations:
 *   - Only bodies with parent<0 (no moons): ~14 bodies
 *   - Positions cached in a small stack array before inner loop
 *   - Force accumulation in float (pos/vel update in double)
 *   - No softening term (nearest approach >> softening radius)
 */
void asteroids_step(double dt) {
    /* Cache major-body positions + GM into stack arrays */
    double bx[MAX_MAJOR], by[MAX_MAJOR], bz[MAX_MAJOR], bgm[MAX_MAJOR];
    int nb = 0;
    for (int j = 0; j < g_nbodies && nb < MAX_MAJOR; j++) {
        /* skip moons (parent >= 0 and parent is not a star) */
        if (g_bodies[j].parent >= 0 &&
            !g_bodies[g_bodies[j].parent].is_star) continue;
        bx[nb]  = g_bodies[j].pos[0];
        by[nb]  = g_bodies[j].pos[1];
        bz[nb]  = g_bodies[j].pos[2];
        bgm[nb] = G_CONST * g_bodies[j].mass;
        nb++;
    }

    for (int b = 0; b < s_n_belts; b++) {
        Belt *belt = &s_belts[b];
        if (!belt->initialized) continue;

        for (int i = 0; i < belt->n; i++) {
            Particle *p = &belt->p[i];

            /* Accumulate acceleration from major bodies */
            double ax=0, ay=0, az=0;
            for (int j = 0; j < nb; j++) {
                double dx = bx[j] - p->pos[0];
                double dy = by[j] - p->pos[1];
                double dz = bz[j] - p->pos[2];
                double r2 = dx*dx + dy*dy + dz*dz;
                double r  = sqrt(r2);
                double f  = bgm[j] / (r2 * r);
                ax += f * dx;
                ay += f * dy;
                az += f * dz;
            }

            /* Symplectic Euler: kick then drift */
            p->vel[0] += ax * dt;
            p->vel[1] += ay * dt;
            p->vel[2] += az * dt;
            p->pos[0] += p->vel[0] * dt;
            p->pos[1] += p->vel[1] * dt;
            p->pos[2] += p->vel[2] * dt;
        }
    }
}

static void render_belt(Belt *b, const float vp[16],
                        double cam_x, double cam_y, double cam_z)
{
    if (!b->initialized || !s_upload_buf) return;

    /* Build upload buffer: camera-relative positions */
    for (int i = 0; i < b->n; i++) {
        float *d = s_upload_buf + i*PFP;
        d[0] = (float)(b->p[i].pos[0] * RS - cam_x);
        d[1] = (float)(b->p[i].pos[1] * RS - cam_y);
        d[2] = (float)(b->p[i].pos[2] * RS - cam_z);
        d[3] = b->p[i].bright;
    }

    glBindVertexArray(b->vao);
    glBindBuffer(GL_ARRAY_BUFFER, b->vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, b->n*PFP*sizeof(float), s_upload_buf);

    glUseProgram(b->shader);
    glUniformMatrix4fv(b->loc_vp,         1, GL_FALSE, vp);
    glUniform3f       (b->loc_color,      b->color[0], b->color[1], b->color[2]);
    glUniform1f       (b->loc_fade_start, b->fade_start);
    glUniform1f       (b->loc_fade_end,   b->fade_end);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);   /* points don't write depth — no z-fighting */
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);   /* additive */
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDrawArrays(GL_POINTS, 0, b->n);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

void asteroids_render(const float vp_camrel[16])
{
    double cam_x = g_cam.pos[0];
    double cam_y = g_cam.pos[1];
    double cam_z = g_cam.pos[2];

    for (int b = 0; b < s_n_belts; b++)
        render_belt(&s_belts[b], vp_camrel, cam_x, cam_y, cam_z);
}

void asteroids_shutdown(void) {
    free(s_upload_buf); s_upload_buf = NULL;

    for (int b = 0; b < s_n_belts; b++) {
        Belt *belt = &s_belts[b];
        free(belt->p);
        if (belt->vbo)    glDeleteBuffers(1,      &belt->vbo);
        if (belt->vao)    glDeleteVertexArrays(1, &belt->vao);
        if (belt->shader) glDeleteProgram(belt->shader);
    }
    free(s_belts);
    s_belts   = NULL;
    s_n_belts = 0;
}
