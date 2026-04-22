/*
 * trails.h — orbital trail rendering
 */
#pragma once
#include "common.h"

/* Allocate per-body trail VAOs/VBOs.  Call after GL context is ready. */
void trails_gl_init(void);

/* Allocate GL trail buffers for one runtime-added body. */
void trails_add_body(int body_idx);

/* Upload and draw all trails.  vp: view-projection matrix. */
void trails_render(const float vp[16]);

void trails_gl_shutdown(void);
