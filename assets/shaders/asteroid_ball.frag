#version 330 core
/*
 * asteroid_ball.frag — rocky sphere for asteroid belt particles
 *
 * Per-pixel ray–sphere intersection using v_oc (cam−centre) and v_radius
 * passed from the vertex shader.  Simple Phong diffuse + ambient; no
 * procedural texture (asteroids are too small for detail to matter).
 *
 * Fragments outside the sphere silhouette are discarded — there is no
 * dot/pixel fallback.  The CPU culls distant asteroids before draw,
 * so very small screen-footprint asteroids are simply never submitted.
 */

in vec3  v_oc;
in float v_radius;
in float v_bright;

uniform vec3  u_cam_fwd;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform float u_fov_tan;
uniform float u_aspect;
uniform vec3  u_sun;       /* sun, camera-relative (AU) — for lighting */
uniform vec3  u_color;     /* base rocky tint                          */

out vec4 frag_color;

void main() {
    /* Per-pixel ray direction */
    vec2 ndc     = gl_FragCoord.xy / vec2(u_aspect * 360.0, 360.0) - 1.0;
    vec3 ray_dir = normalize(u_cam_fwd
                           + u_cam_right * (ndc.x * u_aspect * u_fov_tan)
                           + u_cam_up    * (ndc.y * u_fov_tan));

    /* Ray–sphere intersection */
    float b    = dot(v_oc, ray_dir);
    float c    = dot(v_oc, v_oc) - v_radius * v_radius;
    float disc = b * b - c;
    if (disc < 0.0) discard;

    float t = -b - sqrt(disc);
    if (t < 0.0) t = -b + sqrt(disc);
    if (t < 0.0) discard;

    vec3  hit  = v_oc + t * ray_dir;
    vec3  N    = normalize(hit);

    /* Lighting: sun relative to asteroid centre */
    vec3  sun_rel = u_sun - (-v_oc);   /* u_sun is cam-rel, −v_oc = centre */
    vec3  L       = normalize(sun_rel - hit);
    float diff    = max(dot(N, L), 0.0);
    float light   = 0.08 + 0.92 * diff;

    /* Subtle brightness variation per-asteroid */
    vec3 col = u_color * (0.7 + 0.3 * v_bright) * light;

    /* Logarithmic depth — consistent with rest of scene */
    const float FAR = 2000.0;
    gl_FragDepth = log2(t + 1.0) / log2(FAR + 1.0);

    frag_color = vec4(col, 1.0);
}
