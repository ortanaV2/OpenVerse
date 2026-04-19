/*
 * camera.c — camera state and movement helpers
 */
#include "camera.h"
#include <math.h>

Camera g_cam;

void cam_reset(void) {
    /* Ecliptic plane = GL XZ.  Camera starts near Sol and looks at the Sun. */
    g_cam.pos[0] =   0.0;
    g_cam.pos[1] =   3.0;   /* 3 AU above ecliptic */
    g_cam.pos[2] =   6.0;   /* 6 AU out along +Z   */
    g_cam.yaw    = -90.0f;
    g_cam.pitch  = -26.565f;
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
