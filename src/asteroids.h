#pragma once

void asteroids_init(void);
void asteroids_tick(double dt);
void asteroids_render(const float vp_camrel[16],
                      float cam_x, float cam_y, float cam_z,
                      const float cam_right[3], const float cam_up[3],
                      const float cam_fwd[3]);
void asteroids_shutdown(void);
