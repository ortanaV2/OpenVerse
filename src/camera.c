/*
 * camera.c — camera state and movement helpers
 */
#include "camera.h"
#include <math.h>

Camera g_cam;

void cam_reset(void) {
    /* Ecliptic plane = GL XZ.  Camera sits above (Y+) looking down. */
    g_cam.pos[0] =   0.0f;
    g_cam.pos[1] =  20.0f;   /* 20 AU above ecliptic */
    g_cam.pos[2] =  25.0f;   /* 25 AU "south"        */
    g_cam.yaw    = 180.0f;
    g_cam.pitch  = -38.0f;
    g_cam.speed  =   0.5f;
}

void cam_get_dir(float *dx, float *dy, float *dz) {
    float cy = cosf(g_cam.yaw   * (float)(PI / 180.0));
    float sy = sinf(g_cam.yaw   * (float)(PI / 180.0));
    float cp = cosf(g_cam.pitch * (float)(PI / 180.0));
    float sp = sinf(g_cam.pitch * (float)(PI / 180.0));
    *dx = cy * cp;
    *dy = sp;
    *dz = sy * cp;
}
