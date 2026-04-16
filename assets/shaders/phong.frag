#version 330 core
/*
 * phong.frag — per-pixel ray-sphere intersection using gl_FragCoord
 *
 * Instead of deriving the ray from the interpolated billboard UV (which
 * produces perspective errors for large/close spheres), we compute the
 * exact ray for each screen pixel from gl_FragCoord + camera parameters.
 * The billboard geometry still provides efficient culling; the fragment
 * shader does the correct per-pixel intersection.
 */

/* v_uv is kept for the emissive limb-darkening fallback only */
in vec2 v_uv;

uniform vec3  u_color;
uniform float u_emission;
uniform float u_ambient;
uniform vec3  u_sun_pos_world;
uniform vec3  u_center;
uniform float u_radius;
uniform vec3  u_cam_pos;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform vec3  u_cam_fwd;     /* unit vector: direction camera looks (world) */
uniform float u_fov_tan;     /* tan(FOV/2) — set once from render_init      */
uniform float u_aspect;      /* WIN_W / WIN_H                               */

out vec4 frag_color;

void main() {
    /* ---- per-pixel ray from gl_FragCoord (perspective-correct) --------- */
    /* gl_FragCoord: (0,0) bottom-left → (WIN_W, WIN_H) top-right
     * u_aspect*360 = WIN_W/2,  360 = WIN_H/2
     * ndc = fragCoord / halfRes - 1  (NOT * 2 - 1, that would double-scale) */
    vec2 ndc = gl_FragCoord.xy / vec2(u_aspect * 360.0, 360.0) - 1.0;
    vec3 ray_dir = normalize(u_cam_fwd
                           + u_cam_right * (ndc.x * u_aspect * u_fov_tan)
                           + u_cam_up    * (ndc.y * u_fov_tan));

    /* ---- ray-sphere intersection ----------------------------------------
     * |cam_pos + t*D - center|^2 = radius^2
     * -------------------------------------------------------------------- */
    vec3  oc   = u_cam_pos - u_center;
    float b_   = dot(oc, ray_dir);
    float c_   = dot(oc, oc) - u_radius * u_radius;
    float disc = b_ * b_ - c_;

    if (disc < 0.0) discard;

    float t = -b_ - sqrt(disc);
    if (t < 0.0) t = -b_ + sqrt(disc);
    if (t < 0.0) discard;

    vec3 hit = u_cam_pos + t * ray_dir;
    vec3 N   = normalize(hit - u_center);

    /* ---- emissive (sun / stars) ---------------------------------------- */
    if (u_emission > 0.5) {
        float cosA = max(dot(N, -ray_dir), 0.0);  /* 1=centre, 0=limb */
        float limb = 0.75 + 0.25 * cosA;
        frag_color = vec4(u_color * limb, 1.0);
        return;
    }

    /* ---- Phong diffuse -------------------------------------------------- */
    vec3  L     = normalize(u_sun_pos_world - hit);
    float diff  = max(dot(N, L), 0.0);
    float light = u_ambient + (1.0 - u_ambient) * diff;

    frag_color = vec4(u_color * light, 1.0);
}
