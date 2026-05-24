/*
 * labels.c — body label rendering
 *
 * Pipeline each frame:
 *   1. SDL_TTF renders each body name to an RGBA surface → GL texture (once,
 *      on labels_init or labels_add_body).
 *   2. Project label anchor to screen; compute axis-aligned bounding rects.
 *   3. Sort candidates by priority (stars > planets > moons, within each
 *      tier by camera distance).
 *   4. Greedy AABB overlap removal: iterate in priority order; a label is
 *      rejected if its rect intersects any already-confirmed label's rect.
 *   5. Hysteresis debounce: labels must be continuously eligible for
 *      SHOW_DELAY seconds before appearing, and continuously blocked for
 *      HIDE_DELAY seconds before disappearing.  Prevents flickering when a
 *      label sits on the overlap boundary.
 *   6. Draw surviving labels as billboards using camera right/up vectors.
 *
 * ── Label sizing ─────────────────────────────────────────────────────────
 *
 * Labels maintain a constant pixel height (LABEL_PX_H) on screen, regardless
 * of body distance.  The shader computes world-space width/height from eye_z:
 *   fh = eye_z × 2 × LABEL_PX_H × tan(FOV/2) / WIN_H
 *
 * eye_z is used instead of Euclidean dcam because eye_z is stable under
 * pure camera rotation: rotating does not change the forward-axis depth of a
 * stationary body.  dcam = eye_z / cos(θ) — rotating the camera changes θ
 * and therefore dcam, causing the label size to change when only the camera
 * rotates, not when the body actually moves.
 *
 * ── Camera-relative positioning ──────────────────────────────────────────
 *
 * Anchor positions are computed as (body_pos × RS − cam_pos) in double
 * precision before casting to float.  The vp passed in is proj × view_rot
 * (no translation), matching the convention used by the sphere and dot passes.
 *
 * ── Label appearance ─────────────────────────────────────────────────────
 *
 * Moon labels are rendered in italic to visually distinguish them from planets.
 * Label color is body_col × 1.4 + 0.15, clamped to 1.0 — brightened above
 * the body's diffuse color so labels remain legible against dark backgrounds.
 */
#include "labels.h"
#include "body.h"
#include "camera.h"
#include "gl_utils.h"
#include "math3d.h"
#include "ui_theme.h"

/* Hard far cutoff: non-star labels are suppressed beyond this distance (AU).
 * Stars are always shown regardless of distance (they are visible at any range). */
#define MAX_LABEL_DIST   55.0f

/* Close cutoff in body radii: suppresses the label when the camera is too close
 * to the body disc.  Stars use a larger threshold because glare dominates earlier. */
#define MIN_LABEL_RADII       10
#define STAR_MIN_LABEL_RADII  80

#define LBL_PAD      6.0f   /* extra pixels added to rect for overlap comparison */
#define LABEL_PX_H  14.0f   /* desired label height in pixels                    */

/* ── GL resources ───────────────────────────────────────────────────────── */

static GLuint s_shader   = 0;
static GLuint s_vao      = 0;
static GLuint s_vbo      = 0;
static GLuint s_ebo      = 0;

static GLint  s_loc_vp     = -1;
static GLint  s_loc_anchor = -1;
static GLint  s_loc_right  = -1;
static GLint  s_loc_up     = -1;
static GLint  s_loc_tex    = -1;

/* Per-body SDL_TTF-rendered name textures */
static GLuint s_tex[MAX_BODIES];
static int    s_tex_w[MAX_BODIES];
static int    s_tex_h[MAX_BODIES];

/* ── Hysteresis state ───────────────────────────────────────────────────── */
/*
 * Labels use a two-threshold hysteresis to prevent rapid show/hide cycling
 * when a label sits on the edge of the overlap-rejection zone:
 *
 *   s_show_accum[i] — seconds the label has continuously been eligible.
 *                     Once this reaches SHOW_DELAY, s_active[i] is set to 1.
 *   s_hide_accum[i] — seconds the label has continuously been blocked/absent.
 *                     Once this reaches HIDE_DELAY, s_active[i] is set to 0.
 *
 * The show threshold is longer (0.20 s) to absorb momentary occlusion from
 * passing bodies without causing a flash.  The hide threshold is shorter
 * (0.06 s) so labels disappear promptly when the camera approaches a body.
 */
#define SHOW_DELAY 0.20f
#define HIDE_DELAY 0.06f

static float s_show_accum[MAX_BODIES];
static float s_hide_accum[MAX_BODIES];
static int   s_active[MAX_BODIES];

static TTF_Font *s_font = NULL;

/* ── helpers ─────────────────────────────────────────────────────────────── */


/*
 * build_label_texture — render the body name into s_tex[i] via SDL_TTF.
 *
 * Color: body_col × 1.4 + 0.15 clamped to [0,1], converted to uint8.
 * Brightens the body color so the label is legible on dark backgrounds.
 *
 * Style: moons (parent exists and parent is not a star) are rendered italic
 * to visually distinguish them from planets and stars.
 *
 * Any previous texture for this index is deleted before the new one is created.
 */
static void build_label_texture(int i)
{
    if (i < 0 || i >= MAX_BODIES || i >= g_nbodies || !s_font) return;

    if (s_tex[i]) {
        glDeleteTextures(1, &s_tex[i]);
        s_tex[i] = 0;
    }

    SDL_Color col;
    col.r = (Uint8)(fminf(g_bodies[i].col[0]*1.4f+0.15f, 1.0f)*255);
    col.g = (Uint8)(fminf(g_bodies[i].col[1]*1.4f+0.15f, 1.0f)*255);
    col.b = (Uint8)(fminf(g_bodies[i].col[2]*1.4f+0.15f, 1.0f)*255);
    col.a = 255;
    int is_moon = (g_bodies[i].parent >= 0 &&
                   !g_bodies[g_bodies[i].parent].is_star);
    TTF_SetFontStyle(s_font, is_moon ? TTF_STYLE_ITALIC : TTF_STYLE_NORMAL);
    SDL_Surface *surf = TTF_RenderText_Blended(s_font, g_bodies[i].name, col);
    if (!surf) { TTF_SetFontStyle(s_font, TTF_STYLE_NORMAL); return; }
    s_tex[i] = gl_surf_to_tex(surf, &s_tex_w[i], &s_tex_h[i]);
    TTF_SetFontStyle(s_font, TTF_STYLE_NORMAL);
}

/* ── public API ─────────────────────────────────────────────────────────── */

void labels_init(void) {
    s_shader = gl_shader_load("assets/shaders/label.vert",
                              "assets/shaders/label.frag");
    if (!s_shader) { fprintf(stderr,"[Labels] shader failed\n"); return; }

    s_loc_vp     = glGetUniformLocation(s_shader, "u_vp");
    s_loc_anchor = glGetUniformLocation(s_shader, "u_anchor");
    s_loc_right  = glGetUniformLocation(s_shader, "u_right");
    s_loc_up     = glGetUniformLocation(s_shader, "u_up");
    s_loc_tex    = glGetUniformLocation(s_shader, "u_tex");

    /* Unit quad: each vertex carries its UV position (0..1 range).
     * label.vert expands this to world-space using the cam right/up vectors
     * and world-space label dimensions computed from eye_z. */
    static const float quad_verts[] = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
    };
    static const unsigned int quad_idx[] = { 0,1,2, 0,2,3 };

    s_vao = gl_vao_create();
    s_vbo = gl_vbo_create(sizeof(quad_verts), quad_verts, GL_STATIC_DRAW);
    s_ebo = gl_ebo_create(sizeof(quad_idx), quad_idx);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glBindVertexArray(0);

    if (TTF_Init() < 0) {
        fprintf(stderr, "[Labels] TTF_Init: %s\n", TTF_GetError());
        return;
    }
    s_font = ui_theme_open_font(16);
    if (!s_font) {
        fprintf(stderr, "[Labels] no usable font found — labels disabled\n");
        return;
    }

    memset(s_tex,        0, sizeof(s_tex));
    memset(s_tex_w,      0, sizeof(s_tex_w));
    memset(s_tex_h,      0, sizeof(s_tex_h));
    memset(s_show_accum, 0, sizeof(s_show_accum));
    memset(s_hide_accum, 0, sizeof(s_hide_accum));
    memset(s_active,     0, sizeof(s_active));
    for (int i = 0; i < g_nbodies; i++)
        if (g_bodies[i].alive) build_label_texture(i);
}

/* Register a newly added body and generate its label texture. */
void labels_add_body(int body_idx)
{
    if (body_idx < 0 || body_idx >= MAX_BODIES) return;
    s_show_accum[body_idx] = 0.0f;
    s_hide_accum[body_idx] = 0.0f;
    s_active[body_idx] = 0;
    build_label_texture(body_idx);
}

/* Deregister a body (e.g. after collision merge) and free its texture. */
void labels_remove_body(int body_idx)
{
    if (body_idx < 0 || body_idx >= MAX_BODIES) return;
    s_show_accum[body_idx] = 0.0f;
    s_hide_accum[body_idx] = 0.0f;
    s_active[body_idx] = 0;
    if (s_tex[body_idx]) {
        glDeleteTextures(1, &s_tex[body_idx]);
        s_tex[body_idx] = 0;
    }
    s_tex_w[body_idx] = 0;
    s_tex_h[body_idx] = 0;
}

void labels_render(const float view[16], const float proj[16],
                   const float vp[16], const BodyRenderInfo *info,
                   float dt) {
    (void)proj;
    if (!s_shader || !s_font) return;

    Vec3 cam_right, cam_up, cam_fwd;
    mat4_get_right(view, cam_right);
    mat4_get_up   (view, cam_up);
    mat4_get_fwd  (view, cam_fwd);

    float half_fov_tan = tanf(FOV * 0.5f * (float)(PI / 180.0));

    /* ---- Step 1: project anchor to screen and build label AABB ---- */
    float lsx[MAX_BODIES], lsy[MAX_BODIES];
    float lsw[MAX_BODIES], lsh[MAX_BODIES];
    int   lvis[MAX_BODIES];
    int   order[MAX_BODIES];

    for (int i = 0; i < g_nbodies; i++) {
        order[i]   = i;
        lvis[i]    = 0;
        if (!g_bodies[i].alive) continue;
        if (!s_tex[i]) continue;
        /* Far cutoff: planet/moon labels are only useful in their local system */
        if (!g_bodies[i].is_star && info[i].dcam > MAX_LABEL_DIST) continue;

        /* Close cutoff: hide label when camera is inside the body's glow region */
        {
            float min_radii = g_bodies[i].is_star
                            ? (float)STAR_MIN_LABEL_RADII
                            : (float)MIN_LABEL_RADII;
            if (info[i].dcam < min_radii * info[i].dr) continue;
        }

        /* Camera-relative anchor in double → float.  vp is proj×view_rot (no translation). */
        double cx = (double)g_cam.pos[0];
        double cy = (double)g_cam.pos[1];
        double cz = (double)g_cam.pos[2];
        float ax = (float)(g_bodies[i].pos[0] * RS - cx);
        float ay = (float)(g_bodies[i].pos[1] * RS - cy) + info[i].dr * 1.4f;
        float az = (float)(g_bodies[i].pos[2] * RS - cz);

        float sx, sy;
        if (!mat4_project(vp, ax, ay, az, WIN_W, WIN_H, &sx, &sy)) continue;

        /* Convert from GL (y=0 bottom) to SDL (y=0 top) for screen-space comparison */
        float screen_y = (float)WIN_H - sy;

        float ph = LABEL_PX_H;
        float pw = ph * (float)s_tex_w[i] / (float)s_tex_h[i];

        lsx[i] = sx;
        lsy[i] = screen_y - ph;
        lsw[i] = pw + LBL_PAD;
        lsh[i] = ph + LBL_PAD;
        lvis[i] = 1;
    }

    /* ---- Step 2: priority order (stars > planets > moons, nearest first) ---- */
    {
        int ns = 0, np = 0, nm = 0;
        int stars[MAX_BODIES], planets[MAX_BODIES], moons[MAX_BODIES];
        for (int i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].alive) continue;
            if      (g_bodies[i].is_star) stars[ns++] = i;
            else if (g_bodies[i].parent < 0 ||
                     g_bodies[g_bodies[i].parent].is_star) planets[np++] = i;
            else                                           moons[nm++] = i;
        }
        /* Insertion sort each tier by dcam ascending */
        for (int i = 1; i < ns; i++) {
            int tmp = stars[i], k = i;
            while (k > 0 && info[stars[k-1]].dcam > info[tmp].dcam)
                { stars[k] = stars[k-1]; k--; }
            stars[k] = tmp;
        }
        for (int i = 1; i < np; i++) {
            int tmp = planets[i], k = i;
            while (k > 0 && info[planets[k-1]].dcam > info[tmp].dcam)
                { planets[k] = planets[k-1]; k--; }
            planets[k] = tmp;
        }
        for (int i = 1; i < nm; i++) {
            int tmp = moons[i], k = i;
            while (k > 0 && info[moons[k-1]].dcam > info[tmp].dcam)
                { moons[k] = moons[k-1]; k--; }
            moons[k] = tmp;
        }
        for (int i = 0; i < ns; i++) order[i]          = stars[i];
        for (int i = 0; i < np; i++) order[ns + i]      = planets[i];
        for (int i = 0; i < nm; i++) order[ns + np + i] = moons[i];
        for (int i = ns + np + nm; i < g_nbodies; i++) order[i] = -1;
    }

    /* ---- Step 3: greedy AABB overlap removal ---- */
    for (int i = 0; i < g_nbodies; i++) {
        int idx = order[i];
        if (idx < 0) continue;
        if (!lvis[idx]) continue;
        /* Reject if rect overlaps any previously accepted label */
        for (int j = 0; j < i; j++) {
            int jdx = order[j];
            if (jdx < 0) continue;
            if (!lvis[jdx]) continue;
            if (lsx[idx]          < lsx[jdx]+lsw[jdx] &&
                lsx[idx]+lsw[idx] > lsx[jdx]           &&
                lsy[idx]          < lsy[jdx]+lsh[jdx] &&
                lsy[idx]+lsh[idx] > lsy[jdx]) {
                lvis[idx] = 0; break;
            }
        }
    }

    /* ---- Step 4: hysteresis debounce ---- */
    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        if (lvis[i]) {
            s_show_accum[i] += dt;
            s_hide_accum[i]  = 0.0f;
            if (s_show_accum[i] >= SHOW_DELAY)
                s_active[i] = 1;
        } else {
            s_hide_accum[i] += dt;
            s_show_accum[i]  = 0.0f;
            if (s_hide_accum[i] >= HIDE_DELAY)
                s_active[i] = 0;
        }
    }

    /* ---- Step 5: draw surviving labels as camera-aligned billboards ---- */
    glUseProgram(s_shader);
    glUniformMatrix4fv(s_loc_vp, 1, GL_FALSE, vp);
    glUniform1i(s_loc_tex, 0);
    glActiveTexture(GL_TEXTURE0);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);

    glBindVertexArray(s_vao);

    for (int i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive) continue;
        if (!s_active[i]) continue;
        if (!s_tex[i])    continue;

        /* Camera-relative body anchor in double → float */
        double cx2 = (double)g_cam.pos[0];
        double cy2 = (double)g_cam.pos[1];
        double cz2 = (double)g_cam.pos[2];
        float bx = (float)(g_bodies[i].pos[0] * RS - cx2);
        float by = (float)(g_bodies[i].pos[1] * RS - cy2);
        float bz = (float)(g_bodies[i].pos[2] * RS - cz2);

        /* eye_z: forward-axis depth.  Stable under rotation (dcam is not). */
        float eye_z = bx*cam_fwd[0] + by*cam_fwd[1] + bz*cam_fwd[2];
        if (eye_z <= 0.0f) continue;

        /* World-space dimensions: constant pixel height regardless of distance */
        float fh = (eye_z * 2.0f * LABEL_PX_H * half_fov_tan) / (float)WIN_H;
        float fw = fh * (float)s_tex_w[i] / (float)s_tex_h[i];

        /* Anchor: body centre + up×dr_off, then nudge by right+up so the quad
         * sits clear of the body disc.  Vector math avoids per-component jumps
         * when the camera is tilted (a tilt changes all three components together). */
        float dr_off = info[i].dr * 1.4f;
        float ax = bx + cam_up[0]*dr_off + cam_right[0]*(fw*0.1f) + cam_up[0]*(fh*0.1f);
        float ay = by + cam_up[1]*dr_off + cam_right[1]*(fw*0.1f) + cam_up[1]*(fh*0.1f);
        float az = bz + cam_up[2]*dr_off + cam_right[2]*(fw*0.1f) + cam_up[2]*(fh*0.1f);

        Vec3 right_scaled, up_scaled;
        vec3_scale(right_scaled, cam_right, fw);
        vec3_scale(up_scaled,   cam_up,    fh);

        glUniform3f(s_loc_anchor, ax, ay, az);
        glUniform3f(s_loc_right,  right_scaled[0], right_scaled[1], right_scaled[2]);
        glUniform3f(s_loc_up,     up_scaled[0],    up_scaled[1],    up_scaled[2]);

        glBindTexture(GL_TEXTURE_2D, s_tex[i]);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }

    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void labels_shutdown(void) {
    glDeleteTextures(MAX_BODIES, s_tex);
    glDeleteBuffers(1, &s_vbo);
    glDeleteBuffers(1, &s_ebo);
    glDeleteVertexArrays(1, &s_vao);
    glDeleteProgram(s_shader);
    if (s_font) TTF_CloseFont(s_font);
    TTF_Quit();
    s_shader = s_vao = s_vbo = s_ebo = 0;
    s_font = NULL;
}
