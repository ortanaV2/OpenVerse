/*
 * rings.h — Saturn particle ring system
 *
 * ParticleDisc is a reusable struct for any ring/belt system.
 *
 * LOD strategy
 * ─────────────
 *   dist > SPRITE_DIST AU  →  flat textured disc (ring_sprite shader)
 *   dist > LOD_DIST    AU  →  2 000 Keplerian particles  (ring shader)
 *   dist ≤ LOD_DIST    AU  →  25 000 Keplerian particles (ring shader)
 *
 * Per-particle layout in data_full / data_lod  (8 floats interleaved):
 *   [0] M      — mean anomaly (rad), advanced by rings_tick each physics step
 *   [1] a      — semi-major axis (AU), static
 *   [2] e      — eccentricity, static (small, ~0–0.008)
 *   [3] omega  — argument of periapsis (rad), static
 *   [4] height — vertical offset (AU), static
 *   [5] cr, [6] cg, [7] cb — colour
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
    /* Particle shader (ring.vert + color.frag) */
    GLuint shader;
    GLuint loc_vp, loc_center, loc_b1, loc_b2, loc_pole;
    GLuint vao_full, vbo_full;
    GLuint vao_lod,  vbo_lod;
    /* Sprite shader (ring_sprite.vert + ring_sprite.frag) */
    GLuint sprite_shader;
    GLuint sp_loc_vp, sp_loc_center, sp_loc_b1, sp_loc_b2;
    GLuint sprite_vao, sprite_vbo;
    int    initialized;
} ParticleDisc;

void rings_init(void);
void rings_tick(double dt);         /* advance mean anomalies — call each physics sub-step */
void rings_render(const float vp[16]);
void rings_shutdown(void);
