/*
 * labels.h — SDL_ttf body label system (billboard quads, overlap avoidance)
 */
#pragma once
#include "common.h"

/* Per-body data needed by the label renderer (computed each frame) */
typedef struct {
    float pos[3];   /* body centre in GL/AU world space      */
    float dr;       /* visual radius (AU)                    */
    float dcam;     /* camera distance (AU)                  */
    int   show;     /* 1 when body is too small to see as disc */
} BodyRenderInfo;

/* Initialise SDL_ttf, load font, pre-render name textures */
void labels_init(void);

/* Create/update the cached text texture for a runtime-added body. */
void labels_add_body(int body_idx);

/* Disable and release the cached label for a removed/absorbed body. */
void labels_remove_body(int body_idx);

/*
 * Render labels for all bodies.
 *   view  — view matrix (for extracting camera right/up)
 *   proj  — projection matrix
 *   vp    — pre-multiplied proj*view (for screen-space projection)
 *   info  — per-body render data (array of g_nbodies entries)
 */
void labels_render(const float view[16], const float proj[16],
                   const float vp[16], const BodyRenderInfo *info,
                   float dt);

void labels_shutdown(void);
