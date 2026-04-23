/*
 * universe.h — data-driven universe loader
 *
 * Reads assets/universe.json and populates g_bodies[] / g_nbodies.
 * Call universe_load() from main.c instead of solar_system_init().
 */
#pragma once

#include "body.h"

typedef struct {
    const char *name;
    double mass;              /* kg */
    double radius;            /* m */
    double pos[3];            /* m */
    double vel[3];            /* m/s */
    float  col[3];
    int    is_star;
    int    parent;
    double obliquity;         /* degrees */
    double rotation_rate;     /* rad/s */
    float  atm_color[3];
    float  atm_intensity;
    float  atm_scale;
} BodyCreateSpec;

/*
 * universe_load — parse the given JSON file and populate the g_bodies
 * array with all bodies in the first stellar system found.
 *
 * On any error (file not found, parse failure, etc.) this function
 * prints an error to stderr and calls exit(1).
 */
void universe_load(const char *path);

/* Add a fully specified runtime body. Returns the new body index, or -1. */
int universe_add_body(const BodyCreateSpec *spec);

/* Reassign planets/dwarf bodies to the nearest star after sandbox edits. */
void universe_rebind_to_nearest_stars(void);

/* universe_shutdown — free all body trail buffers and the g_bodies array. */
void universe_shutdown(void);
