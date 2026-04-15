/*
 * starfield.c — procedural background stars (skybox, rotation-only rendering)
 *
 * Stars are placed on a unit sphere and rendered with the translation
 * component stripped from the view matrix, so they appear infinitely distant.
 * Colour distribution matches real stellar spectral types.
 */
#include "starfield.h"
#include "gl_utils.h"
#include "math3d.h"
#include <stdlib.h>
#include <math.h>

static GLuint s_shader   = 0;
static GLuint s_vao      = 0;
static GLuint s_vbo      = 0;
static GLint  s_loc_vp   = -1;
static int    s_count    = 0;

/* ------------------------------------------------------------------ internal */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static void star_color(float *r, float *g, float *b) {
    /* Rough stellar colour distribution by spectral type frequency */
    float t = randf();
    if      (t < 0.03f) { *r=0.70f; *g=0.77f; *b=1.00f; } /* O/B blue-white */
    else if (t < 0.13f) { *r=0.90f; *g=0.92f; *b=1.00f; } /* A white        */
    else if (t < 0.30f) { *r=1.00f; *g=0.98f; *b=0.85f; } /* F yellow-white */
    else if (t < 0.55f) { *r=1.00f; *g=0.95f; *b=0.70f; } /* G yellow (Sun) */
    else if (t < 0.78f) { *r=1.00f; *g=0.80f; *b=0.50f; } /* K orange       */
    else                { *r=1.00f; *g=0.55f; *b=0.35f; } /* M red          */
}

/* ------------------------------------------------------------------ public */
void starfield_init(void) {
    s_shader = gl_shader_load("assets/shaders/color.vert",
                              "assets/shaders/color.frag");
    if (!s_shader) return;

    s_loc_vp = glGetUniformLocation(s_shader, "u_vp");

    /* Build star vertex buffer: position (3) + color (3) = 6 floats per star */
    srand(42);
    s_count = NUM_STARS;
    float *verts = (float *)malloc(s_count * 6 * sizeof(float));

    for (int i = 0; i < s_count; i++) {
        /* Uniform distribution on sphere */
        float theta = acosf(1.0f - 2.0f * randf());
        float phi   = 2.0f * (float)PI * randf();
        float sx    = sinf(theta) * cosf(phi);
        float sy    = cosf(theta);
        float sz    = sinf(theta) * sinf(phi);

        float r, g, b;
        star_color(&r, &g, &b);
        /* Dim stars slightly for realism */
        float bright = 0.4f + 0.6f * randf();
        r *= bright; g *= bright; b *= bright;

        verts[i*6+0]=sx; verts[i*6+1]=sy; verts[i*6+2]=sz;
        verts[i*6+3]=r;  verts[i*6+4]=g;  verts[i*6+5]=b;
    }

    s_vao = gl_vao_create();
    s_vbo = gl_vbo_create(s_count * 6 * sizeof(float), verts, GL_STATIC_DRAW);
    free(verts);

    /* position: location 0 */
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    /* color: location 1 */
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                          (void*)(3*sizeof(float)));

    glBindVertexArray(0);
}

void starfield_render(const float view_rot[16], const float proj[16]) {
    if (!s_shader) return;

    Mat4 vp;
    mat4_mul(vp, proj, view_rot);

    glUseProgram(s_shader);
    glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);

    glBindVertexArray(s_vao);

    /* Draw small stars (1px) */
    glPointSize(1.0f);
    glDrawArrays(GL_POINTS, 0, s_count * 3 / 4);

    /* Draw brighter stars (2px) — last quarter of buffer */
    glPointSize(2.0f);
    glDrawArrays(GL_POINTS, s_count * 3 / 4, s_count / 4);

    glBindVertexArray(0);
    glPointSize(1.0f);
}

void starfield_shutdown(void) {
    glDeleteBuffers(1, &s_vbo);
    glDeleteVertexArrays(1, &s_vao);
    glDeleteProgram(s_shader);
    s_vao = s_vbo = s_shader = 0;
}
