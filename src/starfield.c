/*
 * starfield.c - catalog-backed background star skybox
 *
 * The runtime path loads a compact Yale Bright Star Catalog subset from
 * assets/bright_star_catalog.csv. Coordinates are J2000.0 equatorial
 * RA/Dec and are rotated into the simulation's ecliptic GL frame:
 *
 *   GL X = ecliptic X (vernal equinox)
 *   GL Y = ecliptic Z (north ecliptic pole)
 *   GL Z = ecliptic Y
 *
 * If the catalog asset is missing, a deterministic procedural fallback is
 * generated so the renderer still starts in development builds.
 */
#include "starfield.h"
#include "gl_utils.h"
#include "math3d.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define STAR_CATALOG_PATH "assets/bright_star_catalog.csv"
#define STAR_MAG_BRIGHT   1.5f
#define STAR_MAG_MID      4.5f

typedef struct {
    float pos[3];
    float col[3];
    float mag;
} StarVertex;

/* ---------------------------------------------------------------- private */

static GLuint s_shader       = 0;
static GLuint s_vao          = 0;
static GLuint s_vbo          = 0;
static GLint  s_loc_vp       = -1;
static int    s_count        = 0;
static int    s_faint_count  = 0;
static int    s_mid_count    = 0;
static int    s_bright_count = 0;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }


static int star_cmp_faint_to_bright(const void *a, const void *b)
{
    const StarVertex *sa = (const StarVertex *)a;
    const StarVertex *sb = (const StarVertex *)b;
    if (sa->mag < sb->mag) return 1;
    if (sa->mag > sb->mag) return -1;
    return 0;
}

static void temperature_to_rgb(float kelvin, float *r, float *g, float *b)
{
    float t = clampf(kelvin, 1000.0f, 40000.0f) / 100.0f;

    if (t <= 66.0f) {
        *r = 1.0f;
        *g = clampf(0.39008158f * logf(t) - 0.63184144f, 0.0f, 1.0f);
        if (t <= 19.0f)
            *b = 0.0f;
        else
            *b = clampf(0.54320679f * logf(t - 10.0f) - 1.19625409f, 0.0f, 1.0f);
    } else {
        *r = clampf(1.29293619f * powf(t - 60.0f, -0.13320476f), 0.0f, 1.0f);
        *g = clampf(1.12989086f * powf(t - 60.0f, -0.07551485f), 0.0f, 1.0f);
        *b = 1.0f;
    }
}

static float display_brightness_from_mag(float mag)
{
    float t = (6.70f - mag) / (6.70f - (-1.50f));
    t = clampf(t, 0.0f, 1.0f);
    return 0.18f + 0.82f * powf(t, 0.65f);
}

static void equatorial_to_gl(double ra_deg, double dec_deg, float out[3])
{
    const double deg = PI / 180.0;
    const double eps = 23.4392911 * deg; /* J2000 mean obliquity */

    double ra  = ra_deg  * deg;
    double dec = dec_deg * deg;
    double ce  = cos(eps);
    double se  = sin(eps);

    double x_eq = cos(dec) * cos(ra);
    double y_eq = cos(dec) * sin(ra);
    double z_eq = sin(dec);

    double x_ecl = x_eq;
    double y_ecl = y_eq * ce + z_eq * se;
    double z_ecl = -y_eq * se + z_eq * ce;

    out[0] = (float)x_ecl;
    out[1] = (float)z_ecl;
    out[2] = (float)y_ecl;
}

static int load_catalog(StarVertex **out)
{
    FILE *f = fopen(STAR_CATALOG_PATH, "rb");
    if (!f) return 0;

    int cap = 8192;
    int n = 0;
    StarVertex *stars = (StarVertex *)malloc((size_t)cap * sizeof(StarVertex));
    if (!stars) {
        fclose(f);
        return 0;
    }

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        double ra_deg, dec_deg, mag, temp_k;
        float bright;

        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;
        if (strncmp(line, "ra_deg", 6) == 0)
            continue;
        if (sscanf(line, "%lf,%lf,%lf,%lf", &ra_deg, &dec_deg, &mag, &temp_k) != 4)
            continue;

        if (n >= cap) {
            int new_cap = cap * 2;
            StarVertex *grown = (StarVertex *)realloc(stars, (size_t)new_cap * sizeof(StarVertex));
            if (!grown) break;
            stars = grown;
            cap = new_cap;
        }

        equatorial_to_gl(ra_deg, dec_deg, stars[n].pos);
        temperature_to_rgb((float)temp_k, &stars[n].col[0], &stars[n].col[1], &stars[n].col[2]);
        bright = display_brightness_from_mag((float)mag);
        stars[n].col[0] *= bright;
        stars[n].col[1] *= bright;
        stars[n].col[2] *= bright;
        stars[n].mag = (float)mag;
        n++;
    }

    fclose(f);

    if (n <= 0) {
        free(stars);
        return 0;
    }

    *out = stars;
    return n;
}

static void procedural_color(float *r, float *g, float *b)
{
    float t = randf();
    if      (t < 0.03f) { *r=0.70f; *g=0.77f; *b=1.00f; }
    else if (t < 0.13f) { *r=0.90f; *g=0.92f; *b=1.00f; }
    else if (t < 0.30f) { *r=1.00f; *g=0.98f; *b=0.85f; }
    else if (t < 0.55f) { *r=1.00f; *g=0.95f; *b=0.70f; }
    else if (t < 0.78f) { *r=1.00f; *g=0.80f; *b=0.50f; }
    else                { *r=1.00f; *g=0.55f; *b=0.35f; }
}

static int build_procedural(StarVertex **out)
{
    int n = NUM_STARS;
    StarVertex *stars = (StarVertex *)malloc((size_t)n * sizeof(StarVertex));
    if (!stars) return 0;

    srand(42);
    for (int i = 0; i < n; i++) {
        float theta = acosf(1.0f - 2.0f * randf());
        float phi   = 2.0f * (float)PI * randf();
        float bright;

        stars[i].pos[0] = sinf(theta) * cosf(phi);
        stars[i].pos[1] = cosf(theta);
        stars[i].pos[2] = sinf(theta) * sinf(phi);

        procedural_color(&stars[i].col[0], &stars[i].col[1], &stars[i].col[2]);
        stars[i].mag = -1.0f + 7.7f * randf();
        bright = display_brightness_from_mag(stars[i].mag);
        stars[i].col[0] *= bright;
        stars[i].col[1] *= bright;
        stars[i].col[2] *= bright;
    }

    *out = stars;
    return n;
}

static float *pack_vertices(StarVertex *stars, int n)
{
    float *verts = (float *)malloc((size_t)n * 6 * sizeof(float));
    if (!verts) return NULL;

    qsort(stars, (size_t)n, sizeof(StarVertex), star_cmp_faint_to_bright);

    s_faint_count = s_mid_count = s_bright_count = 0;
    for (int i = 0; i < n; i++) {
        verts[i*6+0] = stars[i].pos[0];
        verts[i*6+1] = stars[i].pos[1];
        verts[i*6+2] = stars[i].pos[2];
        verts[i*6+3] = stars[i].col[0];
        verts[i*6+4] = stars[i].col[1];
        verts[i*6+5] = stars[i].col[2];

        if (stars[i].mag > STAR_MAG_MID)
            s_faint_count++;
        else if (stars[i].mag > STAR_MAG_BRIGHT)
            s_mid_count++;
        else
            s_bright_count++;
    }

    return verts;
}

/* ---------------------------------------------------------------- public */

void starfield_init(void) {
    StarVertex *stars = NULL;
    float *verts;

    s_shader = gl_shader_load("assets/shaders/color.vert",
                              "assets/shaders/color.frag");
    if (!s_shader) return;

    s_loc_vp = glGetUniformLocation(s_shader, "u_vp");

    s_count = load_catalog(&stars);
    if (s_count > 0) {
        fprintf(stdout, "[Starfield] loaded %d BSC5 catalog stars\n", s_count);
    } else {
        s_count = build_procedural(&stars);
        fprintf(stdout, "[Starfield] catalog missing; generated %d fallback stars\n", s_count);
    }
    if (s_count <= 0 || !stars) return;

    verts = pack_vertices(stars, s_count);
    free(stars);
    if (!verts) return;

    s_vao = gl_vao_create();
    s_vbo = gl_vbo_create((size_t)s_count * 6 * sizeof(float), verts, GL_STATIC_DRAW);
    free(verts);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                          (void*)(3*sizeof(float)));

    glBindVertexArray(0);
}

void starfield_render(const float view_rot[16], const float proj[16]) {
    if (!s_shader || !s_vao || s_count <= 0) return;

    Mat4 vp;
    mat4_mul(vp, proj, view_rot);

    glUseProgram(s_shader);
    glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);

    glBindVertexArray(s_vao);

    if (s_faint_count > 0) {
        glPointSize(1.0f);
        glDrawArrays(GL_POINTS, 0, s_faint_count);
    }
    if (s_mid_count > 0) {
        glPointSize(2.0f);
        glDrawArrays(GL_POINTS, s_faint_count, s_mid_count);
    }
    if (s_bright_count > 0) {
        glPointSize(3.0f);
        glDrawArrays(GL_POINTS, s_faint_count + s_mid_count, s_bright_count);
    }

    glBindVertexArray(0);
    glPointSize(1.0f);
}

void starfield_shutdown(void) {
    glDeleteBuffers(1, &s_vbo);
    glDeleteVertexArrays(1, &s_vao);
    glDeleteProgram(s_shader);
    s_vao = s_vbo = s_shader = 0;
    s_count = s_faint_count = s_mid_count = s_bright_count = 0;
}
