#pragma once

/* asteroids_init — parse belt configs from the given universe.json path. */
void asteroids_init(const char *path);
void asteroids_step(double dt);   /* gravity integration — call once per outer step */
void asteroids_render(const float vp_camrel[16],
                      float cam_x, float cam_y, float cam_z);
void asteroids_shutdown(void);
