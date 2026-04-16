/*
 * common.h — shared constants, macros, and includes for all modules
 */
#pragma once

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

/* GLEW must come before any other GL/GLU headers */
#include <GL/glew.h>
#include <GL/glu.h>

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ window */
#define WIN_W  1280
#define WIN_H   720
#define FOV    60.0f

/* ------------------------------------------------------------------ math */
#define PI  3.14159265358979323846

/* ------------------------------------------------------------------ physics */
#define AU         1.496e11          /* m per AU                         */
#define DAY        86400.0           /* s per day                        */
#define G_CONST    6.674e-11         /* m^3 kg^-1 s^-2                   */
#define GM_SUN     2.9591220828559093e-4  /* AU^3/day^2                 */
#define SOFTENING  1e5               /* collision softening radius (m)   */

/* ------------------------------------------------------------------ sim */
#define MAX_BODIES         128
#define TRAIL_LEN         2048
#define NUM_STARS          4000

/* 1 AU → 1.0 GL unit */
#define RS  (1.0 / AU)
