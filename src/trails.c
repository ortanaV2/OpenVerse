/*
 * trails.c — per-body orbital trail rendering
 *
 * Each body has a dynamic VBO (GL_DYNAMIC_DRAW) holding its trail positions.
 * Every frame the circular buffer is unrolled into a linear scratch array
 * and uploaded, then drawn as a GL_LINE_STRIP with alpha fade via gl_VertexID.
 */
#include "trails.h"
#include "body.h"
#include "camera.h"
#include "gl_utils.h"
#include "math3d.h"

static GLuint s_shader    = 0;
static GLuint s_vao[MAX_BODIES];
static GLuint s_vbo[MAX_BODIES];

static GLint  s_loc_vp    = -1;
static GLint  s_loc_color = -1;
static GLint  s_loc_count = -1;

static float  s_scratch[(TRAIL_LEN + 1) * 3];  /* +1 for live planet position */

void trails_gl_init(void) {
    s_shader = gl_shader_load("assets/shaders/solid.vert",
                              "assets/shaders/solid.frag");
    if (!s_shader) return;

    s_loc_vp    = glGetUniformLocation(s_shader, "u_vp");
    s_loc_color = glGetUniformLocation(s_shader, "u_color");
    s_loc_count = glGetUniformLocation(s_shader, "u_count");

    for (int i = 0; i < MAX_BODIES; i++) {
        s_vao[i] = gl_vao_create();
        /* Allocate maximum capacity; actual data is partial each frame */
        s_vbo[i] = gl_vbo_create((TRAIL_LEN + 1) * 3 * sizeof(float),
                                  NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              3*sizeof(float), (void*)0);
        glBindVertexArray(0);
    }
}

void trails_render(const float vp[16]) {
    if (!s_shader) return;

    glUseProgram(s_shader);
    glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);  /* don't write trail to depth buffer */

    for (int i = 0; i < g_nbodies; i++) {
        Body *b = &g_bodies[i];
        if (b->is_star || b->trail_count < 2) continue;

        /* Unroll the circular buffer: oldest → newest.
         * Subtract camera position in double to avoid float32 catastrophic
         * cancellation when the camera is close to a small/distant body. */
        int count = b->trail_count;
        int head  = b->trail_head;
        double cx = (double)g_cam.pos[0];
        double cy = (double)g_cam.pos[1];
        double cz = (double)g_cam.pos[2];
        for (int k = 0; k < count; k++) {
            int idx = (head - count + k + TRAIL_LEN) % TRAIL_LEN;
            s_scratch[k*3+0] = (float)(b->trail[idx][0] - cx);
            s_scratch[k*3+1] = (float)(b->trail[idx][1] - cy);
            s_scratch[k*3+2] = (float)(b->trail[idx][2] - cz);
        }

        /* Append the live planet position as the final point so the trail
         * always connects to the planet with zero lag, regardless of how
         * frequently trails_sample() is called. */
        s_scratch[count*3+0] = (float)(b->pos[0] * RS - cx);
        s_scratch[count*3+1] = (float)(b->pos[1] * RS - cy);
        s_scratch[count*3+2] = (float)(b->pos[2] * RS - cz);
        int draw_count = count + 1;

        glBindVertexArray(s_vao[i]);
        glBindBuffer(GL_ARRAY_BUFFER, s_vbo[i]);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        draw_count * 3 * sizeof(float), s_scratch);

        glUniform4f(s_loc_color,
                    b->col[0], b->col[1], b->col[2], 0.6f);
        glUniform1i(s_loc_count, draw_count);
        glDrawArrays(GL_LINE_STRIP, 0, draw_count);
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

void trails_gl_shutdown(void) {
    glDeleteBuffers(MAX_BODIES, s_vbo);
    glDeleteVertexArrays(MAX_BODIES, s_vao);
    glDeleteProgram(s_shader);
    s_shader = 0;
}
