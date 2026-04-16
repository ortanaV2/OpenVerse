/*
 * camera.h — free-look camera (yaw/pitch, position in GL/AU units)
 */
#pragma once
#include "common.h"

typedef struct {
    double pos[3];  /* AU — double precision to avoid precision loss at large distances */
    float yaw;      /* degrees, horizontal */
    float pitch;    /* degrees, vertical   */
    float speed;    /* AU/second           */
} Camera;

extern Camera g_cam;

/* Reset to default top-down solar-system view */
void cam_reset(void);

/* Compute forward direction vector from current yaw/pitch */
void cam_get_dir(float *dx, float *dy, float *dz);
