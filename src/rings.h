/*
 * rings.h — Keplerian ring particle system (Saturn, Uranus, Neptune)
 *
 * ParticleDisc is a self-contained ring system for one planet.
 * Three instances are maintained internally (rings.c); the public API
 * treats them as a unit.
 *
 * LOD strategy (per disc, based on camera distance to parent body):
 *   dist > SPRITE_DIST AU  →  flat procedural sprite quad
 *   dist > LOD_DIST    AU  →  n_lod  Keplerian particles
 *   dist ≤ LOD_DIST    AU  →  n_full Keplerian particles
 *
 * Per-particle data layout in data_full / data_lod  (8 floats):
 *   [0] M      — mean anomaly (rad), advanced by rings_tick each physics step
 *   [1] a      — semi-major axis (AU), static
 *   [2] e      — eccentricity, static
 *   [3] omega  — argument of periapsis (rad), static
 *   [4] height — vertical offset (AU), static
 *   [5..7] cr,cg,cb — zone colour, static
 */
#pragma once
#include "common.h"
#include "camera.h"

typedef struct {
    int    parent_idx;    /* g_bodies index of central body               */
    int    n_full;        /* full LOD particle count                      */
    int    n_lod;         /* reduced LOD particle count                   */
    float *data_full;     /* 8 floats × n_full: M,a,e,ω,h,cr,cg,cb       */
    float *data_lod;      /* 8 floats × n_lod                             */
    float *n_arr_full;    /* Keplerian mean motion (rad/s) — CPU only     */
    float *n_arr_lod;

    /* Ring-plane basis — computed from parent obliquity at init */
    float  b1[3], b2[3], pole[3];
    float  sprite_r;      /* sprite quad half-extent (AU)                 */

    /* Particle shader (ring.vert + color.frag) */
    GLuint shader;
    GLuint loc_vp, loc_center, loc_b1, loc_b2, loc_pole;
    GLuint vao_full, vbo_full;
    GLuint vao_lod,  vbo_lod;

    /* Sprite shader — Saturn uses ring_sprite.frag (hardcoded zones);
     * other planets use ring_sprite_generic.frag (uniform-driven).     */
    GLuint sprite_shader;
    GLuint sp_loc_vp, sp_loc_center, sp_loc_b1, sp_loc_b2;
    int    use_generic_sprite;
    /* Generic sprite extra uniforms */
    GLuint sp_loc_r_inner, sp_loc_r_outer, sp_loc_ring_color, sp_loc_alpha_max;
    float  sp_r_inner_km, sp_r_outer_km;
    float  sp_color[3];
    float  sp_alpha_max;

    GLuint sprite_vao, sprite_vbo;
    int    initialized;
} ParticleDisc;

/* rings_init — parse ring configs from the given universe.json path. */
void rings_init(const char *path);
void rings_tick(double dt);         /* advance mean anomalies — call each physics sub-step */
void rings_render(const float vp[16]);
void rings_on_body_absorbed(int target_idx, int impactor_idx);
void rings_shutdown(void);
