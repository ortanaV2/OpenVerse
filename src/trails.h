/*
 * trails.h — orbital trail rendering
 */
#pragma once
#include "common.h"

/* Allocate per-body trail VAOs/VBOs.  Call after GL context is ready. */
void trails_gl_init(void);

/* Allocate GL trail buffers for one runtime-added body. */
void trails_add_body(int body_idx);

/* Clear cached trail state for a removed/absorbed body. */
void trails_remove_body(int body_idx);

/* Re-seed a body's trail after a discontinuous state change like a merge. */
void trails_reset_body(int body_idx);

/* Upload and draw all trails.  vp: view-projection matrix. */
void trails_render(const float vp[16]);

void trails_gl_shutdown(void);
