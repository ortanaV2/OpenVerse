/*
 * collision.h - broadphase collision checks and simple impact heat effects.
 */
#pragma once
#include "common.h"

#define COLLISION_MAX_SPOTS 4

typedef enum {
    COLLISION_VIS_CRATER = 1,
    COLLISION_VIS_MAJOR  = 2,
    COLLISION_VIS_MERGE  = 3
} CollisionVisualKind;

typedef struct {
    float dir[3];          /* body-local unit direction */
    float angular_radius;  /* radians on sphere */
    float heat;            /* 0..1 cooled intensity */
    float progress;        /* 0..1 event lifetime progress */
    int   kind;            /* CollisionVisualKind */
} CollisionSpot;

void collision_step(double dt);
int  collision_spots_for_body(int body_idx, CollisionSpot spots[COLLISION_MAX_SPOTS]);
void collision_on_body_added(int body_idx);
double collision_visual_radius(int body_idx, double physical_radius);
