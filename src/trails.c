/*
 * trails.c — per-body orbital trail rendering
 *
 * Each body has a fixed-size circular buffer (TRAIL_LEN samples).
 * trail_interval ≈ T/200 gives ~200 samples per orbit; at high sim speeds
 * the buffer fills faster, naturally showing more orbital history.
 *
 * Render strategy:
 *   - Trail positions are stored camera-relative in the VBO (world − cam),
 *     computed in double precision on the CPU before converting to float.
 *     This eliminates float cancellation jitter that arises when subtracting
 *     two large absolute AU-scale positions of similar magnitude.
 *   - Dirty check: re-upload when new samples were recorded OR the camera moved.
 *   - The scratch buffer linearises the circular buffer (oldest→newest)
 *     and appends the live planet position as the final vertex.
 */
#include "trails.h"
#include "body.h"
#include "camera.h"
#include "gl_utils.h"
#include "math3d.h"
#include <stdlib.h>
#include <string.h>

static GLuint *s_vao = NULL;
static GLuint *s_vbo = NULL;
static int     s_n   = 0;

/* Dirty tracking — skip upload when no new samples */
static int *s_last_head  = NULL;
static int *s_last_count = NULL;

static GLuint s_shader    = 0;
static GLint  s_loc_vp    = -1;
static GLint  s_loc_color = -1;
static GLint  s_loc_count = -1;

/* Last camera position — used to detect when a re-upload is needed even if no
 * new trail samples were recorded (camera moved → camera-relative VBO is stale) */
static double s_last_cam[3] = {0.0, 0.0, 0.0};

/* Scratch: linearised trail + live position, (TRAIL_LEN+1) × 3 floats */
static float s_scratch[(TRAIL_LEN + 1) * 3];

/* ---------------------------------------------------------------- public */

void trails_gl_init(void)
{
    s_shader = gl_shader_load("assets/shaders/solid.vert",
                              "assets/shaders/solid.frag");
    if (!s_shader) return;

    s_loc_vp    = glGetUniformLocation(s_shader, "u_vp");
    s_loc_color = glGetUniformLocation(s_shader, "u_color");
    s_loc_count = glGetUniformLocation(s_shader, "u_count");

    s_n          = g_nbodies;
    s_vao        = (GLuint*)malloc(s_n * sizeof(GLuint));
    s_vbo        = (GLuint*)malloc(s_n * sizeof(GLuint));
    s_last_head  = (int*)   malloc(s_n * sizeof(int));
    s_last_count = (int*)   malloc(s_n * sizeof(int));
    if (!s_vao || !s_vbo || !s_last_head || !s_last_count) return;

    for (int i = 0; i < s_n; i++) {
        s_last_head[i]  = 0;
        s_last_count[i] = 0;

        s_vao[i] = gl_vao_create();
        s_vbo[i] = gl_vbo_create((TRAIL_LEN + 1) * 3 * sizeof(float),
                                 NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              3 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }
}

void trails_render(const float vp[16])
{
    if (!s_shader) return;

    /* Distance from camera to star (body 0) — controls LOD fade.
     * Computed in render units (AU).  Returns early if trails are fully faded. */
    float trail_fade = 1.0f;
    if (g_nbodies > 0) {
        float sdx = (float)(g_cam.pos[0] - g_bodies[0].pos[0] * RS);
        float sdy = (float)(g_cam.pos[1] - g_bodies[0].pos[1] * RS);
        float sdz = (float)(g_cam.pos[2] - g_bodies[0].pos[2] * RS);
        float dist = sqrtf(sdx*sdx + sdy*sdy + sdz*sdz);
        trail_fade = 1.0f - (dist - SYS_TRAIL_FADE_START)
                          / (SYS_TRAIL_FADE_END - SYS_TRAIL_FADE_START);
        if (trail_fade > 1.0f) trail_fade = 1.0f;
        if (trail_fade <= 0.0f) return;   /* fully faded — skip all work */
    }

    glUseProgram(s_shader);
    glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);

    /* Camera moved since last frame? If so every trail's VBO must be re-uploaded
     * because the positions are stored camera-relative (double-precision CPU
     * subtraction to avoid float cancellation jitter on WASD movement). */
    const int cam_moved = (g_cam.pos[0] != s_last_cam[0] ||
                           g_cam.pos[1] != s_last_cam[1] ||
                           g_cam.pos[2] != s_last_cam[2]);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    for (int i = 0; i < g_nbodies && i < s_n; i++) {
        Body *b = &g_bodies[i];
        if (b->is_star || b->trail_count < 2 || !b->trail) continue;

        const int head  = b->trail_head;
        const int count = b->trail_count;

        glBindVertexArray(s_vao[i]);
        glBindBuffer(GL_ARRAY_BUFFER, s_vbo[i]);

        /* Re-upload when new samples were recorded OR the camera moved.
         * Positions are stored camera-relative: the double-precision subtraction
         * (b->trail[idx][x] - g_cam.pos[x]) is done here on the CPU so the
         * shader only ever receives small residual floats, eliminating the
         * cancellation jitter that appears when WASD moves the camera. */
        if (cam_moved || head != s_last_head[i] || count != s_last_count[i]) {
            /* Linearise circular buffer: oldest → newest, camera-relative */
            for (int k = 0; k < count; k++) {
                int idx = (head - count + k + TRAIL_LEN) % TRAIL_LEN;
                s_scratch[k*3+0] = (float)(b->trail[idx][0] - g_cam.pos[0]);
                s_scratch[k*3+1] = (float)(b->trail[idx][1] - g_cam.pos[1]);
                s_scratch[k*3+2] = (float)(b->trail[idx][2] - g_cam.pos[2]);
            }
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            count * 3 * sizeof(float), s_scratch);
            s_last_head[i]  = head;
            s_last_count[i] = count;
        }

        /* Always write the current live position as the final vertex so the
         * trail tip follows the planet every frame even without a new sample.
         * Camera-relative, same double-precision subtraction as above. */
        float live[3] = {
            (float)(b->pos[0] * RS - g_cam.pos[0]),
            (float)(b->pos[1] * RS - g_cam.pos[1]),
            (float)(b->pos[2] * RS - g_cam.pos[2])
        };
        glBufferSubData(GL_ARRAY_BUFFER,
                        count * 3 * sizeof(float), sizeof(live), live);

        int draw_count = count + 1;
        glUniform4f(s_loc_color, b->col[0], b->col[1], b->col[2], 0.6f * trail_fade);
        glUniform1i(s_loc_count, draw_count);
        glDrawArrays(GL_LINE_STRIP, 0, draw_count);
    }

    /* Record camera position so we can detect movement next frame */
    s_last_cam[0] = g_cam.pos[0];
    s_last_cam[1] = g_cam.pos[1];
    s_last_cam[2] = g_cam.pos[2];

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

void trails_gl_shutdown(void)
{
    if (s_vbo) { glDeleteBuffers(s_n, s_vbo);       free(s_vbo);      s_vbo = NULL; }
    if (s_vao) { glDeleteVertexArrays(s_n, s_vao);  free(s_vao);      s_vao = NULL; }
    if (s_last_head)  { free(s_last_head);  s_last_head  = NULL; }
    if (s_last_count) { free(s_last_count); s_last_count = NULL; }
    glDeleteProgram(s_shader);
    s_shader = 0;
    s_n = 0;
}
