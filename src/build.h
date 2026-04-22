/*
 * build.h - runtime body placement mode
 */
#pragma once
#include "common.h"

typedef struct {
    const char *name;
    double mass;
    double radius;
    float  col[3];
    int    is_star;
    int    wants_planet_parent;
    int    wants_nonstar_parent;
    float  atm_color[3];
    float  atm_intensity;
    float  atm_scale;
} BuildPreset;

extern int g_build_mode;
extern int g_build_tab_held;

void build_init(void);
void build_toggle(void);
void build_set_tab_held(int held);
void build_scroll(int wheel_y);
int  build_place_current(void);

int build_preset_count(void);
int build_selected_index(void);
const BuildPreset *build_current_preset(void);
const BuildPreset *build_preset_at(int idx);

void build_preview_pos_au(double out[3]);
void build_preview_pos_m(double out[3]);
void build_nearest3(const double pos_m[3], int out_idx[3], double out_dist_au[3]);
