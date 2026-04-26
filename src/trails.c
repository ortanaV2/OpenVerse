/*
 * trails.c — per-body orbital trail rendering
 *
 * Trail samples are still stored as a fixed-size circular buffer, but the
 * visible trail length is derived from physical distance instead of raw
 * vertex count. A short front fade-in softens the segment directly behind the
 * body, while the old tail fades smoothly to zero alpha before it disappears.
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

static int   *s_last_head   = NULL;
static int   *s_last_count  = NULL;
static float *s_total_len   = NULL;
static unsigned char *s_spawn_fade_active = NULL;

static GLuint s_shader           = 0;
static GLint  s_loc_vp           = -1;
static GLint  s_loc_color        = -1;
static GLint  s_loc_visible_len  = -1;
static GLint  s_loc_front_fade   = -1;
static GLint  s_loc_tail_fade    = -1;


/* Scratch: (TRAIL_LEN samples + live tip) × [x, y, z, arc_from_tip] */
static float s_scratch[(TRAIL_LEN + 1) * 4];

/* TRAIL_PLANET_AU / TRAIL_MOON_AU live in common.h so physics.c can derive
 * the per-sample minimum-arc gate from the same values.
 *
 * Tail fade covers the full visible length (old system faded t² over all
 * vertices; smoothstep over the full arc gives the same visual result). */
#define TRAIL_TAIL_FADE_FRAC  1.0f

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float trail_target_length_au(int body_idx)
{
    if (body_idx < 0 || body_idx >= g_nbodies) return 0.0f;
    return g_bodies[body_idx].is_moon ? TRAIL_MOON_AU : TRAIL_PLANET_AU;
}

static void trail_fade_lengths(int body_idx,
                               float visible_len,
                               float *front_fade,
                               float *tail_fade)
{
    (void)body_idx;
    *front_fade = 0.0f;
    *tail_fade  = visible_len * TRAIL_TAIL_FADE_FRAC;
}

static void trails_alloc_state(int new_n)
{
    GLuint *new_vao;
    GLuint *new_vbo;
    int *new_head;
    int *new_count;
    float *new_total_len;
    unsigned char *new_spawn_fade_active;

    new_vao = (GLuint*)malloc(new_n * sizeof(GLuint));
    new_vbo = (GLuint*)malloc(new_n * sizeof(GLuint));
    new_head = (int*)malloc(new_n * sizeof(int));
    new_count = (int*)malloc(new_n * sizeof(int));
    new_total_len = (float*)malloc(new_n * sizeof(float));
    new_spawn_fade_active = (unsigned char*)malloc(new_n * sizeof(unsigned char));
    if (!new_vao || !new_vbo || !new_head || !new_count ||
        !new_total_len || !new_spawn_fade_active) {
        free(new_vao);
        free(new_vbo);
        free(new_head);
        free(new_count);
        free(new_total_len);
        free(new_spawn_fade_active);
        return;
    }

    if (s_n > 0) {
        memcpy(new_vao, s_vao, s_n * sizeof(GLuint));
        memcpy(new_vbo, s_vbo, s_n * sizeof(GLuint));
        memcpy(new_head, s_last_head, s_n * sizeof(int));
        memcpy(new_count, s_last_count, s_n * sizeof(int));
        memcpy(new_total_len, s_total_len, s_n * sizeof(float));
        memcpy(new_spawn_fade_active, s_spawn_fade_active,
               s_n * sizeof(unsigned char));
    }

    free(s_vao);
    free(s_vbo);
    free(s_last_head);
    free(s_last_count);
    free(s_total_len);
    free(s_spawn_fade_active);

    s_vao = new_vao;
    s_vbo = new_vbo;
    s_last_head = new_head;
    s_last_count = new_count;
    s_total_len = new_total_len;
    s_spawn_fade_active = new_spawn_fade_active;

    for (int i = s_n; i < new_n; i++) {
        s_last_head[i] = 0;
        s_last_count[i] = 0;
        s_total_len[i] = 0.0f;
        s_spawn_fade_active[i] = 0;
        s_vao[i] = gl_vao_create();
        s_vbo[i] = gl_vbo_create((TRAIL_LEN + 1) * 4 * sizeof(float),
                                 NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE,
                              4 * sizeof(float), (void*)(3 * sizeof(float)));
        glBindVertexArray(0);
    }

    s_n = new_n;
}

static float trail_upload_body(int body_idx)
{
    Body *b = &g_bodies[body_idx];
    int head = b->trail_head;
    int count = b->trail_count;
    float total_len = 0.0f;

    for (int k = 0; k < count; k++) {
        int idx = (head - count + k + TRAIL_LEN) % TRAIL_LEN;
        s_scratch[k*4+0] = (float)(b->trail[idx][0] - g_cam.pos[0]);
        s_scratch[k*4+1] = (float)(b->trail[idx][1] - g_cam.pos[1]);
        s_scratch[k*4+2] = (float)(b->trail[idx][2] - g_cam.pos[2]);
        s_scratch[k*4+3] = 0.0f;
    }

    if (count > 0) {
        float live_x = (float)(b->pos[0] * RS);
        float live_y = (float)(b->pos[1] * RS);
        float live_z = (float)(b->pos[2] * RS);
        float prev_x = live_x;
        float prev_y = live_y;
        float prev_z = live_z;

        for (int k = count - 1; k >= 0; k--) {
            float px = s_scratch[k*4+0] + (float)g_cam.pos[0];
            float py = s_scratch[k*4+1] + (float)g_cam.pos[1];
            float pz = s_scratch[k*4+2] + (float)g_cam.pos[2];
            float dx = prev_x - px;
            float dy = prev_y - py;
            float dz = prev_z - pz;
            total_len += sqrtf(dx*dx + dy*dy + dz*dz);
            s_scratch[k*4+3] = total_len;
            prev_x = px;
            prev_y = py;
            prev_z = pz;
        }
    }

    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    count * 4 * sizeof(float), s_scratch);
    s_last_head[body_idx] = head;
    s_last_count[body_idx] = count;
    s_total_len[body_idx] = total_len;
    return total_len;
}

static int trail_prune_hidden_tail(int body_idx, float invisible_cutoff)
{
    Body *b;
    int count;
    int drop = 0;

    if (body_idx < 0 || body_idx >= g_nbodies) return 0;
    b = &g_bodies[body_idx];
    count = b->trail_count;
    if (!b->trail || count < 3) return 0;

    /* Keep one fully invisible anchor vertex just beyond the cutoff so the
     * fade-out can still taper smoothly instead of ending on a hard segment. */
    while (drop + 1 < count &&
           s_scratch[(drop + 1) * 4 + 3] > invisible_cutoff) {
        drop++;
    }

    if (drop <= 0) return 0;
    if (count - drop < 2) drop = count - 2;
    if (drop <= 0) return 0;

    b->trail_count -= drop;
    s_last_head[body_idx] = -1;
    s_last_count[body_idx] = -1;
    return 1;
}

void trails_gl_init(void)
{
    s_shader = gl_shader_load("assets/shaders/solid.vert",
                              "assets/shaders/solid.frag");
    if (!s_shader) return;

    s_loc_vp          = glGetUniformLocation(s_shader, "u_vp");
    s_loc_color       = glGetUniformLocation(s_shader, "u_color");
    s_loc_visible_len = glGetUniformLocation(s_shader, "u_visible_len");
    s_loc_front_fade  = glGetUniformLocation(s_shader, "u_front_fade");
    s_loc_tail_fade   = glGetUniformLocation(s_shader, "u_tail_fade");

    trails_alloc_state(g_nbodies);
}

void trails_add_body(int body_idx)
{
    if (!s_shader || body_idx < 0 || body_idx >= MAX_BODIES) return;
    if (body_idx < s_n) return;
    trails_alloc_state(body_idx + 1);
}

void trails_remove_body(int body_idx)
{
    if (body_idx < 0 || body_idx >= s_n) return;
    s_last_head[body_idx] = 0;
    s_last_count[body_idx] = 0;
    s_total_len[body_idx] = 0.0f;
    s_spawn_fade_active[body_idx] = 0;
    if (body_idx < g_nbodies) {
        g_bodies[body_idx].trail_head = 0;
        g_bodies[body_idx].trail_count = 0;
        g_bodies[body_idx].trail_accum = 0.0;
    }
}

void trails_reset_body(int body_idx)
{
    Body *b;
    double x, y, z;

    if (body_idx < 0 || body_idx >= g_nbodies) return;
    b = &g_bodies[body_idx];
    if (!b->trail) return;

    x = b->pos[0] * RS;
    y = b->pos[1] * RS;
    z = b->pos[2] * RS;
    for (int i = 0; i < TRAIL_LEN; i++) {
        b->trail[i][0] = x;
        b->trail[i][1] = y;
        b->trail[i][2] = z;
    }
    b->trail_head = 2 % TRAIL_LEN;
    b->trail_count = 2;
    b->trail_accum = 0.0;

    if (body_idx < s_n) {
        s_last_head[body_idx] = -1;
        s_last_count[body_idx] = -1;
        s_total_len[body_idx] = 0.0f;
        s_spawn_fade_active[body_idx] = 1;
    }
}

void trails_render(const float vp[16])
{
    if (!s_shader) return;

    {
        float trail_fade = 1.0f;

        if (g_nbodies > 0) {
            int star = nearest_star_idx();
            float sdx = (float)(g_cam.pos[0] - g_bodies[star].pos[0] * RS);
            float sdy = (float)(g_cam.pos[1] - g_bodies[star].pos[1] * RS);
            float sdz = (float)(g_cam.pos[2] - g_bodies[star].pos[2] * RS);
            float dist = sqrtf(sdx*sdx + sdy*sdy + sdz*sdz);
            trail_fade = 1.0f - (dist - SYS_TRAIL_FADE_START)
                              / (SYS_TRAIL_FADE_END - SYS_TRAIL_FADE_START);
            if (trail_fade > 1.0f) trail_fade = 1.0f;
            if (trail_fade <= 0.0f) return;
        }

        glUseProgram(s_shader);
        glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);

        {
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);

            for (int i = 0; i < g_nbodies && i < s_n; i++) {
                Body *b = &g_bodies[i];
                int head, count, draw_count;
                float visible_len, front_fade, tail_fade;

                if (b->is_star || b->trail_count < 2 || !b->trail) continue;
                if (!b->alive && b->trail_fade <= 0.0) continue;

                head = b->trail_head;
                count = b->trail_count;

                glBindVertexArray(s_vao[i]);
                glBindBuffer(GL_ARRAY_BUFFER, s_vbo[i]);

                /* Always re-upload so arc values and s_total_len stay current
                 * every frame. Skipping uploads when head/count/cam unchanged
                 * caused discrete jumps in visible_len each time a new sample
                 * fired, producing jerky transparency changes on short trails. */
                trail_upload_body(i);

                draw_count = count;
                if (b->alive && b->trail_interval > 0.0) {
                    float live[4] = {
                        (float)(b->pos[0] * RS - g_cam.pos[0]),
                        (float)(b->pos[1] * RS - g_cam.pos[1]),
                        (float)(b->pos[2] * RS - g_cam.pos[2]),
                        0.0f
                    };
                    glBufferSubData(GL_ARRAY_BUFFER,
                                    count * 4 * sizeof(float), sizeof(live), live);
                    draw_count = count + 1;
                }

                visible_len = trail_target_length_au(i);
                if (visible_len <= 0.0f || visible_len > s_total_len[i])
                    visible_len = s_total_len[i];
                if (visible_len <= 0.0f) continue;

                trail_fade_lengths(i, visible_len, &front_fade, &tail_fade);
                if (trail_prune_hidden_tail(i, visible_len)) {
                    count = b->trail_count;
                    head = b->trail_head;
                    trail_upload_body(i);
                    draw_count = count;
                    if (b->alive && b->trail_interval > 0.0) {
                        float live[4] = {
                            (float)(b->pos[0] * RS - g_cam.pos[0]),
                            (float)(b->pos[1] * RS - g_cam.pos[1]),
                            (float)(b->pos[2] * RS - g_cam.pos[2]),
                            0.0f
                        };
                        glBufferSubData(GL_ARRAY_BUFFER,
                                        count * 4 * sizeof(float), sizeof(live), live);
                        draw_count = count + 1;
                    }
                    if (visible_len > s_total_len[i])
                        visible_len = s_total_len[i];
                }
                if (!s_spawn_fade_active[i]) front_fade = 0.0f;
                front_fade = clampf_local(front_fade, 0.0f, visible_len);
                tail_fade = clampf_local(tail_fade, 0.0f, visible_len);

                glUniform4f(s_loc_color,
                            b->col[0], b->col[1], b->col[2],
                            0.6f * (float)b->trail_fade * trail_fade);
                glUniform1f(s_loc_visible_len, visible_len);
                glUniform1f(s_loc_front_fade, front_fade);
                glUniform1f(s_loc_tail_fade, tail_fade);
                glDrawArrays(GL_LINE_STRIP, 0, draw_count);

                if (s_spawn_fade_active[i] && s_total_len[i] >= front_fade)
                    s_spawn_fade_active[i] = 0;
            }

            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glBindVertexArray(0);
        }
    }
}

void trails_gl_shutdown(void)
{
    if (s_vbo) { glDeleteBuffers(s_n, s_vbo); free(s_vbo); s_vbo = NULL; }
    if (s_vao) { glDeleteVertexArrays(s_n, s_vao); free(s_vao); s_vao = NULL; }
    if (s_last_head) { free(s_last_head); s_last_head = NULL; }
    if (s_last_count) { free(s_last_count); s_last_count = NULL; }
    if (s_total_len) { free(s_total_len); s_total_len = NULL; }
    if (s_spawn_fade_active) { free(s_spawn_fade_active); s_spawn_fade_active = NULL; }
    s_n = 0;
}
