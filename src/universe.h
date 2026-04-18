/*
 * universe.h — data-driven universe loader
 *
 * Reads assets/universe.json and populates g_bodies[] / g_nbodies.
 * Call universe_load() from main.c instead of solar_system_init().
 */
#pragma once

/*
 * universe_load — parse the given JSON file and populate the g_bodies
 * array with all bodies in the first stellar system found.
 *
 * On any error (file not found, parse failure, etc.) this function
 * prints an error to stderr and calls exit(1).
 */
void universe_load(const char *path);

/* universe_shutdown — free all body trail buffers and the g_bodies array. */
void universe_shutdown(void);
