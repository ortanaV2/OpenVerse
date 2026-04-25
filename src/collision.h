/*
 * collision.h - broadphase collision checks and simple impact heat effects.
 */
#pragma once
#include "common.h"

#define COLLISION_MAX_SPOTS 16

typedef enum {
    COLLISION_VIS_CRATER    = 1,
    COLLISION_VIS_MAJOR     = 2,
    COLLISION_VIS_MERGE     = 3,
    COLLISION_VIS_INTERSECT = 4  /* live sphere-sphere boundary during merge */
} CollisionVisualKind;

typedef struct {
    float dir[3];          /* body-local unit direction */
    float tangent1[3];     /* stable body-local tangent basis for scar noise */
    float angular_radius;  /* radians on sphere */
    float heat;            /* 0..1 cooled intensity */
    float progress;        /* 0..1 event lifetime progress */
    int   kind;            /* CollisionVisualKind */
} CollisionSpot;

typedef struct {
    float pos[3];          /* camera-relative render-space position */
    float color[4];        /* rgba */
    float size;            /* per-particle size multiplier */
} CollisionParticle;

void collision_snapshot_positions(void);
void collision_step(double dt);
int  collision_spots_for_body(int body_idx, CollisionSpot spots[COLLISION_MAX_SPOTS]);
void collision_on_body_added(int body_idx);
double collision_visual_radius(int body_idx, double physical_radius);
void collision_body_heat_glow(int body_idx, float out_color[3],
                              float *out_intensity, float *out_scale);
int collision_body_has_active_merge(int body_idx);
int collision_particles(CollisionParticle *out, int max_particles,
                        const double cam_pos[3]);
