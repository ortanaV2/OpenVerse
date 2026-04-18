/*
 * rings.c — Keplerian ring particle system, data-driven from universe.json
 *
 * Physics
 * ───────
 * Each particle follows a Keplerian elliptical orbit around its parent body.
 * The CPU advances the mean anomaly  M += n·dt  each physics sub-step.
 * The GPU solves the first-order Kepler equation every frame.
 *
 * Ring-plane orientation
 * ──────────────────────
 * Each planet's ring plane is perpendicular to its rotation pole.
 * We build an orthonormal basis {b1, b2, pole} by rotating the ecliptic
 * frame about the X-axis by the planet's obliquity.
 *
 * LOD
 * ───
 *   dist > SPRITE_DIST  →  flat sprite quad
 *   dist > LOD_DIST     →  reduced particle count
 *   dist ≤ LOD_DIST     →  full particle count
 */
#include "rings.h"
#include "body.h"
#include "physics.h"
#include "gl_utils.h"
#include "json.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define SPRITE_DIST   0.2f
#define LOD_DIST      0.05f
#define MAX_ZONES     16

/* ── Zone ───────────────────────────────────────────────────────────── */
typedef struct {
    float r_min, r_max;
    float density;
    float r, g, b;
} Zone;

/* ── Active disc instances ──────────────────────────────────────────── */
static ParticleDisc *s_discs  = NULL;
static int           s_n_discs = 0;

/* ── XorShift32 PRNG ────────────────────────────────────────────────── */
static uint32_t s_rng = 1;
static void  s_seed(uint32_t seed) { s_rng = seed ? seed : 1; }
static float s_randf(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng >> 8) * (1.0f / (float)(1 << 24));
}

/* ── build_basis — ring-plane orthonormal basis from obliquity ───────  */
static void build_basis(ParticleDisc *d, float obl_deg)
{
    float obl = obl_deg * (float)(PI / 180.0);
    d->b1[0] = 1.0f; d->b1[1] = 0.0f;       d->b1[2] = 0.0f;
    d->b2[0] = 0.0f; d->b2[1] = sinf(obl);  d->b2[2] = -cosf(obl);
    d->pole[0] = 0.0f; d->pole[1] = cosf(obl); d->pole[2] = sinf(obl);
}

/* ── bake_particles ─────────────────────────────────────────────────── */
static void bake_particles(float *data, float *n_arr, int n,
                            const Zone *zones, int n_zones,
                            double gm, float e_max, float h_scale)
{
    int idx = 0;
    for (int z = 0; z < n_zones; z++) {
        int cnt = (z == n_zones - 1)
                  ? (n - idx)
                  : (int)(zones[z].density * n);

        float r_min_au = zones[z].r_min / 1.496e8f;
        float r_max_au = zones[z].r_max / 1.496e8f;
        float r2_min = r_min_au * r_min_au;
        float r2_max = r_max_au * r_max_au;

        for (int k = 0; k < cnt && idx < n; k++, idx++) {
            float a_au = sqrtf(r2_min + s_randf() * (r2_max - r2_min));
            float M0   = s_randf() * 2.0f * (float)PI;
            float e    = s_randf() * e_max;
            float omega= s_randf() * 2.0f * (float)PI;
            float h    = (s_randf() - 0.5f) * h_scale;

            data[idx*8+0] = M0;
            data[idx*8+1] = a_au;
            data[idx*8+2] = e;
            data[idx*8+3] = omega;
            data[idx*8+4] = h;

            float j = 0.85f + 0.15f * s_randf();
            data[idx*8+5] = zones[z].r * j;
            data[idx*8+6] = zones[z].g * j;
            data[idx*8+7] = zones[z].b * j;

            double a_m = (double)a_au * 1.496e11;
            n_arr[idx] = (float)sqrt(gm / (a_m * a_m * a_m));
        }
    }
}

/* ── VBO attribute layout ───────────────────────────────────────────── */
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

/* ── init_disc_gl ───────────────────────────────────────────────────── */
static void init_disc_gl(ParticleDisc *d)
{
    d->shader = gl_shader_load("assets/shaders/ring.vert",
                               "assets/shaders/color.frag");
    if (!d->shader) return;
    d->loc_vp     = glGetUniformLocation(d->shader, "u_vp");
    d->loc_center = glGetUniformLocation(d->shader, "u_center");
    d->loc_b1     = glGetUniformLocation(d->shader, "u_b1");
    d->loc_b2     = glGetUniformLocation(d->shader, "u_b2");
    d->loc_pole   = glGetUniformLocation(d->shader, "u_pole");

    d->vao_full = gl_vao_create();
    d->vbo_full = gl_vbo_create(d->n_full * 8 * sizeof(float),
                                d->data_full, GL_DYNAMIC_DRAW);
    setup_particle_attribs();
    glBindVertexArray(0);

    d->vao_lod = gl_vao_create();
    d->vbo_lod = gl_vbo_create(d->n_lod * 8 * sizeof(float),
                               d->data_lod, GL_DYNAMIC_DRAW);
    setup_particle_attribs();
    glBindVertexArray(0);

    const char *frag = d->use_generic_sprite
                       ? "assets/shaders/ring_sprite_generic.frag"
                       : "assets/shaders/ring_sprite.frag";
    d->sprite_shader = gl_shader_load("assets/shaders/ring_sprite.vert", frag);
    if (!d->sprite_shader) return;

    d->sp_loc_vp     = glGetUniformLocation(d->sprite_shader, "u_vp");
    d->sp_loc_center = glGetUniformLocation(d->sprite_shader, "u_center");
    d->sp_loc_b1     = glGetUniformLocation(d->sprite_shader, "u_b1");
    d->sp_loc_b2     = glGetUniformLocation(d->sprite_shader, "u_b2");

    if (d->use_generic_sprite) {
        d->sp_loc_r_inner    = glGetUniformLocation(d->sprite_shader, "u_r_inner_km");
        d->sp_loc_r_outer    = glGetUniformLocation(d->sprite_shader, "u_r_outer_km");
        d->sp_loc_ring_color = glGetUniformLocation(d->sprite_shader, "u_ring_color");
        d->sp_loc_alpha_max  = glGetUniformLocation(d->sprite_shader, "u_alpha_max");
    }

    d->sprite_vao = gl_vao_create();
    d->sprite_vbo = gl_vbo_create(6 * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glBindVertexArray(0);
}

/* ── build_sprite_quad ──────────────────────────────────────────────── */
static void build_sprite_quad(float *verts,
                               float cx, float cy, float cz,
                               float R,
                               const float b1[3], const float b2[3])
{
    float c[4][3];
    for (int i = 0; i < 4; i++) {
        float sb1 = (i == 1 || i == 2) ?  R : -R;
        float sb2 = (i == 2 || i == 3) ?  R : -R;
        c[i][0] = cx + sb1*b1[0] + sb2*b2[0];
        c[i][1] = cy + sb1*b1[1] + sb2*b2[1];
        c[i][2] = cz + sb1*b1[2] + sb2*b2[2];
    }
    memcpy(verts+0,  c[0], 12);
    memcpy(verts+3,  c[1], 12);
    memcpy(verts+6,  c[2], 12);
    memcpy(verts+9,  c[0], 12);
    memcpy(verts+12, c[2], 12);
    memcpy(verts+15, c[3], 12);
}

/* ── render_disc ────────────────────────────────────────────────────── */
static void render_disc(const ParticleDisc *d, const float vp[16])
{
    Body *par = &g_bodies[d->parent_idx];
    float px = (float)(par->pos[0] * RS);
    float py = (float)(par->pos[1] * RS);
    float pz = (float)(par->pos[2] * RS);

    float dx   = px - (float)g_cam.pos[0];
    float dy   = py - (float)g_cam.pos[1];
    float dz   = pz - (float)g_cam.pos[2];
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (dist > SPRITE_DIST) {
        if (!d->sprite_shader) return;

        float quad[18];
        build_sprite_quad(quad, px, py, pz, d->sprite_r, d->b1, d->b2);

        glUseProgram(d->sprite_shader);
        glUniformMatrix4fv(d->sp_loc_vp,     1, GL_FALSE, vp);
        glUniform3f       (d->sp_loc_center,  px, py, pz);
        glUniform3fv      (d->sp_loc_b1,    1, d->b1);
        glUniform3fv      (d->sp_loc_b2,    1, d->b2);

        if (d->use_generic_sprite) {
            glUniform1f (d->sp_loc_r_inner,    d->sp_r_inner_km);
            glUniform1f (d->sp_loc_r_outer,    d->sp_r_outer_km);
            glUniform3fv(d->sp_loc_ring_color, 1, d->sp_color);
            glUniform1f (d->sp_loc_alpha_max,  d->sp_alpha_max);
        }

        glBindVertexArray(d->sprite_vao);
        glBindBuffer(GL_ARRAY_BUFFER, d->sprite_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quad), quad);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glDisable(GL_BLEND);
        glBindVertexArray(0);

    } else {
        if (!d->shader) return;

        int    n   = (dist > LOD_DIST) ? d->n_lod   : d->n_full;
        float *data= (dist > LOD_DIST) ? d->data_lod : d->data_full;
        GLuint vao = (dist > LOD_DIST) ? d->vao_lod  : d->vao_full;
        GLuint vbo = (dist > LOD_DIST) ? d->vbo_lod  : d->vbo_full;

        glUseProgram(d->shader);
        glUniformMatrix4fv(d->loc_vp,     1, GL_FALSE, vp);
        glUniform3f       (d->loc_center,  px, py, pz);
        glUniform3fv      (d->loc_b1,    1, d->b1);
        glUniform3fv      (d->loc_b2,    1, d->b2);
        glUniform3fv      (d->loc_pole,  1, d->pole);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, n * 8 * sizeof(float), data);

        glEnable(GL_DEPTH_TEST);
        glPointSize(1.0f);
        glDrawArrays(GL_POINTS, 0, n);
        glBindVertexArray(0);
    }
}

/* ── public ─────────────────────────────────────────────────────────── */

void rings_init(const char *path)
{
    /* Parse rings section from universe.json */
    JsonNode *root = json_parse_file(path);
    if (!root) {
        fprintf(stderr, "[Rings] cannot parse '%s'\n", path);
        return;
    }

    JsonNode *rings_arr = json_get(root, "rings");
    if (!rings_arr || rings_arr->type != JSON_ARRAY) {
        fprintf(stderr, "[Rings] no 'rings' array in '%s' — skipping\n", path);
        json_free(root);
        return;
    }

    /* Count entries */
    int count = 0;
    { JsonNode *n = rings_arr->first_child; while (n) { count++; n = n->next; } }
    if (count == 0) { json_free(root); return; }

    s_discs   = (ParticleDisc*)calloc(count, sizeof(ParticleDisc));
    s_n_discs = 0;
    if (!s_discs) { json_free(root); return; }

    JsonNode *rnode = rings_arr->first_child;
    while (rnode) {
        const char *body_name  = json_str(json_get(rnode, "body"), "");
        const char *shader_type = json_str(json_get(rnode, "shader_type"), "generic");
        int    n_full      = (int)json_num(json_get(rnode, "n_full"),       1000);
        int    n_lod       = (int)json_num(json_get(rnode, "n_lod"),         200);
        uint32_t seed_full = (uint32_t)json_num(json_get(rnode, "seed_full"), 1234);
        uint32_t seed_lod  = (uint32_t)json_num(json_get(rnode, "seed_lod"),  5678);
        float  e_max       = (float)json_num(json_get(rnode, "e_max"),       0.01);
        float  h_scale     = (float)json_num(json_get(rnode, "h_scale"),     1e-6);
        float  sprite_r    = (float)json_num(json_get(rnode, "sprite_r_au"), 0.001);

        /* Find parent body */
        int par_idx = -1;
        for (int i = 0; i < g_nbodies; i++) {
            if (strcmp(g_bodies[i].name, body_name) == 0) { par_idx = i; break; }
        }
        if (par_idx < 0) {
            fprintf(stderr, "[Rings] body '%s' not found — ring skipped\n", body_name);
            rnode = rnode->next;
            continue;
        }

        /* Parse zones */
        Zone zones[MAX_ZONES];
        int  n_zones = 0;
        JsonNode *zones_arr = json_get(rnode, "zones");
        if (zones_arr) {
            JsonNode *zn = zones_arr->first_child;
            while (zn && n_zones < MAX_ZONES) {
                zones[n_zones].r_min    = (float)json_num(json_get(zn, "r_min_km"), 0.0);
                zones[n_zones].r_max    = (float)json_num(json_get(zn, "r_max_km"), 1.0);
                zones[n_zones].density  = (float)json_num(json_get(zn, "density"),  1.0);
                JsonNode *col = json_get(zn, "color");
                zones[n_zones].r = (float)json_num(json_idx(col, 0), 0.7);
                zones[n_zones].g = (float)json_num(json_idx(col, 1), 0.7);
                zones[n_zones].b = (float)json_num(json_idx(col, 2), 0.7);
                n_zones++;
                zn = zn->next;
            }
        }
        if (n_zones == 0) { rnode = rnode->next; continue; }

        ParticleDisc *disc = &s_discs[s_n_discs++];
        disc->parent_idx     = par_idx;
        disc->n_full         = n_full;
        disc->n_lod          = n_lod;
        disc->sprite_r       = sprite_r;
        disc->use_generic_sprite = (strcmp(shader_type, "saturn") != 0);

        if (disc->use_generic_sprite) {
            disc->sp_r_inner_km = (float)json_num(json_get(rnode, "sprite_r_inner_km"), 0.0);
            disc->sp_r_outer_km = (float)json_num(json_get(rnode, "sprite_r_outer_km"), 1.0);
            JsonNode *sc = json_get(rnode, "sprite_color");
            disc->sp_color[0]   = (float)json_num(json_idx(sc, 0), 0.7);
            disc->sp_color[1]   = (float)json_num(json_idx(sc, 1), 0.7);
            disc->sp_color[2]   = (float)json_num(json_idx(sc, 2), 0.7);
            disc->sp_alpha_max  = (float)json_num(json_get(rnode, "sprite_alpha_max"), 0.3);
        }

        disc->data_full  = (float*)malloc(n_full * 8 * sizeof(float));
        disc->data_lod   = (float*)malloc(n_lod  * 8 * sizeof(float));
        disc->n_arr_full = (float*)malloc(n_full * sizeof(float));
        disc->n_arr_lod  = (float*)malloc(n_lod  * sizeof(float));
        if (!disc->data_full || !disc->data_lod ||
            !disc->n_arr_full || !disc->n_arr_lod) {
            rnode = rnode->next; continue;
        }

        build_basis(disc, (float)g_bodies[par_idx].obliquity);

        double gm = G_CONST * g_bodies[par_idx].mass;
        s_seed(seed_full);
        bake_particles(disc->data_full, disc->n_arr_full, n_full,
                       zones, n_zones, gm, e_max, h_scale);

        s_seed(seed_lod);
        bake_particles(disc->data_lod, disc->n_arr_lod, n_lod,
                       zones, n_zones, gm, e_max, h_scale);

        init_disc_gl(disc);
        disc->initialized = 1;

        rnode = rnode->next;
    }

    json_free(root);
}

void rings_tick(double dt)
{
    const float TWO_PI = 6.28318530718f;

    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        if (!disc->initialized) continue;

        for (int i = 0; i < disc->n_full; i++) {
            float M = disc->data_full[i*8+0]
                    + (float)((double)disc->n_arr_full[i] * dt);
            if (M >= TWO_PI) M -= TWO_PI * (float)(int)(M / TWO_PI);
            disc->data_full[i*8+0] = M;
        }
        for (int i = 0; i < disc->n_lod; i++) {
            float M = disc->data_lod[i*8+0]
                    + (float)((double)disc->n_arr_lod[i] * dt);
            if (M >= TWO_PI) M -= TWO_PI * (float)(int)(M / TWO_PI);
            disc->data_lod[i*8+0] = M;
        }
    }
}

void rings_render(const float vp[16])
{
    for (int d = 0; d < s_n_discs; d++) {
        if (s_discs[d].initialized)
            render_disc(&s_discs[d], vp);
    }
}

void rings_shutdown(void)
{
    for (int d = 0; d < s_n_discs; d++) {
        ParticleDisc *disc = &s_discs[d];
        free(disc->data_full);   free(disc->data_lod);
        free(disc->n_arr_full);  free(disc->n_arr_lod);
        if (disc->vbo_full)      glDeleteBuffers(1,      &disc->vbo_full);
        if (disc->vao_full)      glDeleteVertexArrays(1, &disc->vao_full);
        if (disc->vbo_lod)       glDeleteBuffers(1,      &disc->vbo_lod);
        if (disc->vao_lod)       glDeleteVertexArrays(1, &disc->vao_lod);
        if (disc->shader)        glDeleteProgram(disc->shader);
        if (disc->sprite_vbo)    glDeleteBuffers(1,      &disc->sprite_vbo);
        if (disc->sprite_vao)    glDeleteVertexArrays(1, &disc->sprite_vao);
        if (disc->sprite_shader) glDeleteProgram(disc->sprite_shader);
    }
    free(s_discs);
    s_discs   = NULL;
    s_n_discs = 0;
}
