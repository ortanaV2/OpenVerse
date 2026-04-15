/*
 * starfield.h — procedural starfield rendered as a skybox (rotation only)
 */
#pragma once
#include "common.h"

void starfield_init(void);
/* view_rot: view matrix with translation stripped (rotation only)
 * proj:     projection matrix */
void starfield_render(const float view_rot[16], const float proj[16]);
void starfield_shutdown(void);
