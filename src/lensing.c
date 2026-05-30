/*
 * lensing.c - gravitational black-hole compositor (geodesic ray bending)
 *
 * One offscreen FBO (RGBA8 colour texture + depth renderbuffer) sized to the
 * window. The scene renders into it normally; lensing_resolve() then draws a
 * fullscreen quad that, per pixel, integrates the bent photon path around each
 * black hole (see lensing.frag): horizon, accretion disk and photon ring are
 * computed from geometry and the star background is re-sampled in the deflected
 * direction. The FBO is lazily (re)created whenever the window size changes.
 */
#include "lensing.h"
#include "gl_utils.h"

static GLuint s_fbo = 0;
static GLuint s_color_tex = 0;
static GLuint s_star_tex = 0;     /* sky-only snapshot for bent-ray sampling */
static GLuint s_depth_rbo = 0;
static int    s_fbo_w = 0, s_fbo_h = 0;

static GLuint s_shader = 0;
static GLuint s_vao = 0, s_vbo = 0;

static GLint s_u_scene = -1, s_u_stars = -1, s_u_screen = -1, s_u_count = -1, s_u_fov_tan = -1;
static GLint s_u_cam_right = -1, s_u_cam_up = -1, s_u_cam_fwd = -1;
static GLint s_u_center = -1, s_u_rh = -1, s_u_ri = -1, s_u_ro = -1;
static GLint s_u_du = -1, s_u_dv = -1, s_u_dn = -1, s_u_color = -1, s_u_time = -1;

void lensing_init(void)
{
    /* Fullscreen triangle pair in clip space; v_uv derived in the shader. */
    static const float quad[] = {
        -1.0f, -1.0f,   1.0f, -1.0f,   1.0f,  1.0f,
        -1.0f, -1.0f,   1.0f,  1.0f,  -1.0f,  1.0f,
    };
    s_vao = gl_vao_create();
    s_vbo = gl_vbo_create(sizeof(quad), quad, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    s_shader = gl_shader_load("assets/shaders/lensing.vert",
                              "assets/shaders/lensing.frag");
    if (!s_shader) {
        fprintf(stderr, "[Lensing] shader failed; lensing disabled\n");
        return;
    }
    s_u_scene     = glGetUniformLocation(s_shader, "u_scene");
    s_u_stars     = glGetUniformLocation(s_shader, "u_stars");
    s_u_screen    = glGetUniformLocation(s_shader, "u_screen");
    s_u_count     = glGetUniformLocation(s_shader, "u_count");
    s_u_fov_tan   = glGetUniformLocation(s_shader, "u_fov_tan");
    s_u_cam_right = glGetUniformLocation(s_shader, "u_cam_right");
    s_u_cam_up    = glGetUniformLocation(s_shader, "u_cam_up");
    s_u_cam_fwd   = glGetUniformLocation(s_shader, "u_cam_fwd");
    s_u_center    = glGetUniformLocation(s_shader, "u_center");
    s_u_rh        = glGetUniformLocation(s_shader, "u_rh");
    s_u_ri        = glGetUniformLocation(s_shader, "u_ri");
    s_u_ro        = glGetUniformLocation(s_shader, "u_ro");
    s_u_du        = glGetUniformLocation(s_shader, "u_du");
    s_u_dv        = glGetUniformLocation(s_shader, "u_dv");
    s_u_dn        = glGetUniformLocation(s_shader, "u_dn");
    s_u_color     = glGetUniformLocation(s_shader, "u_color");
    s_u_time      = glGetUniformLocation(s_shader, "u_time");
}

static int ensure_target(int w, int h)
{
    if (w <= 0 || h <= 0) return 0;
    if (s_fbo && s_fbo_w == w && s_fbo_h == h) return 1;

    if (!s_fbo)        glGenFramebuffers(1, &s_fbo);
    if (!s_color_tex)  glGenTextures(1, &s_color_tex);
    if (!s_star_tex)   glGenTextures(1, &s_star_tex);
    if (!s_depth_rbo)  glGenRenderbuffers(1, &s_depth_rbo);

    glBindTexture(GL_TEXTURE_2D, s_color_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, s_star_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindRenderbuffer(GL_RENDERBUFFER, s_depth_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);

    glBindFramebuffer(GL_FRAMEBUFFER, s_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, s_color_tex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, s_depth_rbo);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "[Lensing] incomplete FBO (0x%x)\n", status);
        return 0;
    }
    s_fbo_w = w;
    s_fbo_h = h;
    return 1;
}

int lensing_begin_scene(void)
{
    if (!s_shader) return 0;
    if (!ensure_target(WIN_W, WIN_H)) return 0;
    glBindFramebuffer(GL_FRAMEBUFFER, s_fbo);
    return 1;
}

void lensing_capture_background(void)
{
    if (!s_shader || !s_star_tex) return;
    /* s_fbo is the bound read framebuffer here; copy its colour (the sky drawn
     * so far) into the sky-only texture sampled by bent rays in the resolve. */
    glBindTexture(GL_TEXTURE_2D, s_star_tex);
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, s_fbo_w, s_fbo_h);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void lensing_resolve(const LensSource *sources, int count,
                     const float cam_right[3], const float cam_up[3],
                     const float cam_fwd[3], float fov_tan)
{
    if (!s_shader) return;
    if (count > LENS_MAX_HOLES) count = LENS_MAX_HOLES;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    /* Pure copy + composite: no blending, no depth. */
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    float center[3 * LENS_MAX_HOLES], du[3 * LENS_MAX_HOLES];
    float dv[3 * LENS_MAX_HOLES], dn[3 * LENS_MAX_HOLES];
    float color[3 * LENS_MAX_HOLES];
    float rh[LENS_MAX_HOLES], ri[LENS_MAX_HOLES], ro[LENS_MAX_HOLES];
    float tm[LENS_MAX_HOLES];
    for (int i = 0; i < count; i++) {
        const LensSource *s = &sources[i];
        for (int k = 0; k < 3; k++) {
            center[3*i+k] = s->center[k];
            du[3*i+k]     = s->disk_u[k];
            dv[3*i+k]     = s->disk_v[k];
            dn[3*i+k]     = s->disk_n[k];
            color[3*i+k]  = s->color[k];
        }
        rh[i] = s->r_horizon;
        ri[i] = s->r_inner;
        ro[i] = s->r_outer;
        tm[i] = s->disk_time;
    }

    glUseProgram(s_shader);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, s_color_tex);
    glUniform1i(s_u_scene, 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, s_star_tex);
    glUniform1i(s_u_stars, 1);
    glActiveTexture(GL_TEXTURE0);
    glUniform2f(s_u_screen, (float)s_fbo_w, (float)s_fbo_h);
    glUniform1f(s_u_fov_tan, fov_tan);
    glUniform3fv(s_u_cam_right, 1, cam_right);
    glUniform3fv(s_u_cam_up,    1, cam_up);
    glUniform3fv(s_u_cam_fwd,   1, cam_fwd);
    glUniform1i(s_u_count, count);
    if (count > 0) {
        glUniform3fv(s_u_center, count, center);
        glUniform1fv(s_u_rh,     count, rh);
        glUniform1fv(s_u_ri,     count, ri);
        glUniform1fv(s_u_ro,     count, ro);
        glUniform3fv(s_u_du,     count, du);
        glUniform3fv(s_u_dv,     count, dv);
        glUniform3fv(s_u_dn,     count, dn);
        glUniform3fv(s_u_color,  count, color);
        glUniform1fv(s_u_time,   count, tm);
    }

    glBindVertexArray(s_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void lensing_shutdown(void)
{
    if (s_shader)     { glDeleteProgram(s_shader); s_shader = 0; }
    if (s_vbo)        { glDeleteBuffers(1, &s_vbo); s_vbo = 0; }
    if (s_vao)        { glDeleteVertexArrays(1, &s_vao); s_vao = 0; }
    if (s_color_tex)  { glDeleteTextures(1, &s_color_tex); s_color_tex = 0; }
    if (s_star_tex)   { glDeleteTextures(1, &s_star_tex); s_star_tex = 0; }
    if (s_depth_rbo)  { glDeleteRenderbuffers(1, &s_depth_rbo); s_depth_rbo = 0; }
    if (s_fbo)        { glDeleteFramebuffers(1, &s_fbo); s_fbo = 0; }
    s_fbo_w = s_fbo_h = 0;
}
