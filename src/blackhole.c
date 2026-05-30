/*
 * blackhole.c — accretion disk, photon ring, and event-horizon capture
 *
 * Geometry
 * ────────
 * Disk  : a static annulus mesh in disk-local polar form (cos θ, sin θ, t),
 *         oriented per body by its obliquity and scaled by the event-horizon
 *         radius. blackhole_disk.frag shades it as an emissive swirling disk.
 * Ring  : the shared star-glare billboard quad, shaded by blackhole_ring.frag
 *         into a bright photon ring with a soft outward glow.
 *
 * Capture
 * ───────
 * blackhole_step() absorbs any body whose centre crosses a hole's event
 * horizon: momentum and mass transfer to the hole, the horizon grows mildly,
 * orphaned children reparent to the hole, and the swallowed slot is retired
 * with no supernova or merge animation.
 */
#include "blackhole.h"
#include "body.h"
#include "camera.h"
#include "common.h"
#include "gl_utils.h"
#include "physics.h"
#include "labels.h"
#include "rings.h"
#include "trails.h"
#include <math.h>

#define BH_DISK_SEGMENTS 128

/* ── disk geometry / shader ──────────────────────────────────────────────── */
static GLuint s_disk_shader = 0;
static GLuint s_disk_vao = 0;
static GLuint s_disk_vbo = 0;
static int    s_disk_vert_count = 0;
static GLint  s_disk_vp = -1, s_disk_center = -1, s_disk_axis_u = -1,
              s_disk_axis_v = -1, s_disk_r_inner = -1, s_disk_r_outer = -1,
              s_disk_color = -1, s_disk_time = -1;

/* ── photon ring shader (reuses star_glare.vert billboard) ───────────────── */
static GLuint s_ring_shader = 0;
static GLuint s_ring_vao = 0;
static GLuint s_ring_vbo = 0;
static GLuint s_ring_ebo = 0;
static GLint  s_ring_vp = -1, s_ring_center = -1, s_ring_radius = -1,
              s_ring_right = -1, s_ring_up = -1, s_ring_color = -1,
              s_ring_fwd = -1, s_ring_oc = -1, s_ring_fov_tan = -1,
              s_ring_aspect = -1, s_ring_screen = -1;

void blackhole_init(void)
{
    /* Annulus as a triangle strip: pairs of (inner, outer) vertices around
     * the ring. Each vertex = (cos θ, sin θ, t) with t=0 inner, t=1 outer. */
    float verts[(BH_DISK_SEGMENTS + 1) * 2 * 3];
    int n = 0;
    for (int s = 0; s <= BH_DISK_SEGMENTS; s++) {
        double a = (2.0 * PI * s) / BH_DISK_SEGMENTS;
        float cx = (float)cos(a), cy = (float)sin(a);
        verts[n++] = cx; verts[n++] = cy; verts[n++] = 0.0f; /* inner */
        verts[n++] = cx; verts[n++] = cy; verts[n++] = 1.0f; /* outer */
    }
    s_disk_vert_count = (BH_DISK_SEGMENTS + 1) * 2;

    s_disk_vao = gl_vao_create();
    s_disk_vbo = gl_vbo_create(sizeof(verts), verts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    s_disk_shader = gl_shader_load("assets/shaders/blackhole_disk.vert",
                                   "assets/shaders/blackhole_disk.frag");
    if (!s_disk_shader) {
        fprintf(stderr, "[BlackHole] disk shader failed\n");
    } else {
        s_disk_vp      = glGetUniformLocation(s_disk_shader, "u_vp");
        s_disk_center  = glGetUniformLocation(s_disk_shader, "u_center");
        s_disk_axis_u  = glGetUniformLocation(s_disk_shader, "u_axis_u");
        s_disk_axis_v  = glGetUniformLocation(s_disk_shader, "u_axis_v");
        s_disk_r_inner = glGetUniformLocation(s_disk_shader, "u_r_inner");
        s_disk_r_outer = glGetUniformLocation(s_disk_shader, "u_r_outer");
        s_disk_color   = glGetUniformLocation(s_disk_shader, "u_color");
        s_disk_time    = glGetUniformLocation(s_disk_shader, "u_time");
    }

    /* Billboard quad for the photon ring (same layout as star glare). */
    static const float quad_v[] = { 0,0, 1,0, 1,1, 0,1 };
    static const unsigned int quad_i[] = { 0,1,2, 0,2,3 };
    s_ring_vao = gl_vao_create();
    s_ring_vbo = gl_vbo_create(sizeof(quad_v), quad_v, GL_STATIC_DRAW);
    s_ring_ebo = gl_ebo_create(sizeof(quad_i), quad_i);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    s_ring_shader = gl_shader_load("assets/shaders/star_glare.vert",
                                   "assets/shaders/blackhole_ring.frag");
    if (!s_ring_shader) {
        fprintf(stderr, "[BlackHole] ring shader failed\n");
    } else {
        s_ring_vp      = glGetUniformLocation(s_ring_shader, "u_vp");
        s_ring_center  = glGetUniformLocation(s_ring_shader, "u_center");
        s_ring_radius  = glGetUniformLocation(s_ring_shader, "u_radius");
        s_ring_right   = glGetUniformLocation(s_ring_shader, "u_cam_right");
        s_ring_up      = glGetUniformLocation(s_ring_shader, "u_cam_up");
        s_ring_color   = glGetUniformLocation(s_ring_shader, "u_color");
        s_ring_fwd     = glGetUniformLocation(s_ring_shader, "u_cam_fwd");
        s_ring_oc      = glGetUniformLocation(s_ring_shader, "u_oc");
        s_ring_fov_tan = glGetUniformLocation(s_ring_shader, "u_fov_tan");
        s_ring_aspect  = glGetUniformLocation(s_ring_shader, "u_aspect");
        s_ring_screen  = glGetUniformLocation(s_ring_shader, "u_screen");
    }
}

/* ── horizon capture ─────────────────────────────────────────────────────── */
/*
 * swallow_body — transfer a body's mass and momentum into the hole, reparent
 * any of its children to the hole, and retire its slot. No merge animation:
 * a black hole consumes silently.
 */
static void swallow_body(int bh, int j)
{
    Body *h = &g_bodies[bh];
    Body *b = &g_bodies[j];

    double m_total = h->mass + b->mass;
    if (m_total > 0.0) {
        for (int k = 0; k < 3; k++)
            h->vel[k] = (h->mass * h->vel[k] + b->mass * b->vel[k]) / m_total;
    }
    /* Event horizon grows with the cube root of the mass ratio (gentle). */
    if (h->mass > 0.0 && m_total > h->mass)
        h->radius *= pow(m_total / h->mass, 1.0 / 3.0);
    h->mass = m_total;

    /* Reparent the swallowed body's children onto the hole (a system root). */
    for (int k = 0; k < g_nbodies; k++) {
        if (!g_bodies[k].alive || g_bodies[k].parent != j) continue;
        g_bodies[k].parent = bh;
        labels_add_body(k);
    }

    rings_on_body_absorbed(bh, j);
    b->alive = 0;
    b->mass = 0.0;
    b->trail_emitting = 0;
    labels_remove_body(j);

    fprintf(stderr, "[BlackHole] %s swallowed %s\n", h->name, b->name);
}

void blackhole_step(double dt)
{
    int swallowed_any = 0;

    for (int i = 0; i < g_nbodies; i++) {
        Body *h = &g_bodies[i];
        if (!h->alive || !h->is_black_hole) continue;

        /* Advance the disk swirl; rotation_rate carries sign/speed. */
        h->disk_angle += h->rotation_rate * dt;

        double horizon = h->radius;
        for (int j = 0; j < g_nbodies; j++) {
            if (j == i) continue;
            Body *b = &g_bodies[j];
            if (!b->alive || b->is_black_hole) continue;

            double dx = b->pos[0] - h->pos[0];
            double dy = b->pos[1] - h->pos[1];
            double dz = b->pos[2] - h->pos[2];
            if (dx*dx + dy*dy + dz*dz <= horizon * horizon) {
                swallow_body(i, j);
                swallowed_any = 1;
            }
        }
    }

    if (swallowed_any)
        physics_refresh_timestep_model();
}

/* ── render ──────────────────────────────────────────────────────────────── */
/* Build an orthonormal in-plane basis for the disk from the body's obliquity
 * (tilt about the world X axis). axis_u stays on X; axis_v completes the plane. */
static void disk_basis(double obliquity_deg, float axis_u[3], float axis_v[3])
{
    double o = obliquity_deg * (PI / 180.0);
    double c = cos(o), s = sin(o);
    axis_u[0] = 1.0f; axis_u[1] = 0.0f; axis_u[2] = 0.0f;
    axis_v[0] = 0.0f; axis_v[1] = (float)s; axis_v[2] = (float)-c;
}

void blackhole_render(const float vp[16], const float cam_right[3],
                      const float cam_up[3], const float cam_fwd[3])
{
    int any = 0;
    for (int i = 0; i < g_nbodies; i++)
        if (g_bodies[i].alive && g_bodies[i].is_black_hole) { any = 1; break; }
    if (!any) return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);   /* additive */
    glDepthMask(GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    /* ---- accretion disks ---- */
    if (s_disk_shader) {
        glDisable(GL_CULL_FACE);   /* disk is two-sided */
        glUseProgram(s_disk_shader);
        glUniformMatrix4fv(s_disk_vp, 1, GL_FALSE, vp);
        glBindVertexArray(s_disk_vao);

        for (int i = 0; i < g_nbodies; i++) {
            Body *h = &g_bodies[i];
            if (!h->alive || !h->is_black_hole || h->disk_outer <= 0.0f) continue;

            float rx = (float)(h->pos[0] * RS - g_cam.pos[0]);
            float ry = (float)(h->pos[1] * RS - g_cam.pos[1]);
            float rz = (float)(h->pos[2] * RS - g_cam.pos[2]);

            float horizon_au = (float)(h->radius * RS);
            float axis_u[3], axis_v[3];
            disk_basis(h->obliquity, axis_u, axis_v);

            glUniform3f(s_disk_center, rx, ry, rz);
            glUniform3f(s_disk_axis_u, axis_u[0], axis_u[1], axis_u[2]);
            glUniform3f(s_disk_axis_v, axis_v[0], axis_v[1], axis_v[2]);
            glUniform1f(s_disk_r_inner, horizon_au * h->disk_inner);
            glUniform1f(s_disk_r_outer, horizon_au * h->disk_outer);
            glUniform3f(s_disk_color, h->disk_color[0], h->disk_color[1],
                        h->disk_color[2]);
            glUniform1f(s_disk_time, (float)fmod(h->disk_angle, 2.0 * PI));
            glDrawArrays(GL_TRIANGLE_STRIP, 0, s_disk_vert_count);
        }
        glBindVertexArray(0);
    }

    /* ---- photon rings ---- */
    /* The ring is shaded by a per-pixel ray-sphere intersection identical to
     * phong.frag's event-horizon raycast, so the glow stays locked to the dark
     * sphere's silhouette from any view angle (a flat billboard drifts off the
     * silhouette near the frame edge because the raycast is perspective-exact
     * and aspect-corrected, whereas a billboard is not). The quad just supplies
     * raster coverage; the silhouette geometry comes entirely from the ray. */
    if (s_ring_shader) {
        const float RING_MAX_DIST = 1500.0f;
        float fov_tan = tanf(FOV * 0.5f * (float)(PI / 180.0));
        float aspect  = (float)WIN_W / (float)WIN_H;

        glUseProgram(s_ring_shader);
        glUniformMatrix4fv(s_ring_vp, 1, GL_FALSE, vp);
        glUniform3f(s_ring_right, cam_right[0], cam_right[1], cam_right[2]);
        glUniform3f(s_ring_up,    cam_up[0],    cam_up[1],    cam_up[2]);
        glUniform3f(s_ring_fwd,   cam_fwd[0],   cam_fwd[1],   cam_fwd[2]);
        glUniform1f(s_ring_fov_tan, fov_tan);
        glUniform1f(s_ring_aspect,  aspect);
        glUniform2f(s_ring_screen,  (float)WIN_W, (float)WIN_H);
        glBindVertexArray(s_ring_vao);

        for (int i = 0; i < g_nbodies; i++) {
            Body *h = &g_bodies[i];
            if (!h->alive || !h->is_black_hole) continue;

            float rx = (float)(h->pos[0] * RS - g_cam.pos[0]);
            float ry = (float)(h->pos[1] * RS - g_cam.pos[1]);
            float rz = (float)(h->pos[2] * RS - g_cam.pos[2]);
            if (rx*cam_fwd[0] + ry*cam_fwd[1] + rz*cam_fwd[2] < 0.0f) continue;

            float dist = sqrtf(rx*rx + ry*ry + rz*rz);
            float radius = (float)(h->radius * RS);
            /* Pull a far hole into depth range; uniform scaling about the camera
             * preserves the on-screen silhouette angle, so the raycast still
             * matches the dark sphere (which, that far out, renders as a dot). */
            if (dist > RING_MAX_DIST && dist > 1e-6f) {
                float sc = RING_MAX_DIST / dist;
                rx *= sc; ry *= sc; rz *= sc; radius *= sc;
            }

            glUniform3f(s_ring_center, rx, ry, rz);
            glUniform1f(s_ring_radius, radius);
            glUniform3f(s_ring_oc, -rx, -ry, -rz);
            glUniform3f(s_ring_color, h->disk_color[0], h->disk_color[1],
                        h->disk_color[2]);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }
        glBindVertexArray(0);
    }

    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void blackhole_shutdown(void)
{
    if (s_disk_shader) { glDeleteProgram(s_disk_shader); s_disk_shader = 0; }
    if (s_ring_shader) { glDeleteProgram(s_ring_shader); s_ring_shader = 0; }
    if (s_disk_vbo) { glDeleteBuffers(1, &s_disk_vbo); s_disk_vbo = 0; }
    if (s_ring_vbo) { glDeleteBuffers(1, &s_ring_vbo); s_ring_vbo = 0; }
    if (s_ring_ebo) { glDeleteBuffers(1, &s_ring_ebo); s_ring_ebo = 0; }
    if (s_disk_vao) { glDeleteVertexArrays(1, &s_disk_vao); s_disk_vao = 0; }
    if (s_ring_vao) { glDeleteVertexArrays(1, &s_ring_vao); s_ring_vao = 0; }
}
