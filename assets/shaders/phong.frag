#version 330 core
/*
 * phong.frag — billboard sphere with per-pixel Phong shading
 *
 * The vertex shader placed us on a camera-aligned billboard quad.
 * v_uv is in -1..+1 (billboard local).  We discard outside the unit
 * circle and compute a sphere-surface normal in view space.
 *
 * In view space:
 *   - camera right  = (1, 0, 0)
 *   - camera up     = (0, 1, 0)
 *   - toward viewer = (0, 0, 1)
 *
 * So the sphere normal at a billboard fragment is simply:
 *   N_view = normalize(v_uv.x, v_uv.y, sqrt(1 - |v_uv|^2))
 */

in vec2 v_uv;

uniform vec3  u_color;
uniform float u_emission;      /* 1.0 for Sun/stars, 0.0 for planets  */
uniform float u_ambient;       /* 0.05 for planets                    */
uniform vec3  u_sun_pos_view;  /* sun centre in view space            */
uniform mat4  u_view;
uniform vec3  u_center;        /* sphere centre in world space        */
uniform float u_radius;

out vec4 frag_color;

void main() {
    float r2 = dot(v_uv, v_uv);
    if (r2 > 1.0) discard;

    /* --- stars / sun: emissive with slight limb darkening --- */
    if (u_emission > 0.5) {
        float limb = 1.0 - 0.25 * r2;
        frag_color = vec4(u_color * limb, 1.0);
        return;
    }

    /* --- sphere surface normal in view space --- */
    vec3 N = normalize(vec3(v_uv, sqrt(max(0.0, 1.0 - r2))));

    /* Fragment position on sphere in view space */
    vec3 center_view = (u_view * vec4(u_center, 1.0)).xyz;
    vec3 frag_view   = center_view + vec3(v_uv * u_radius,
                                          sqrt(max(0.0, 1.0 - r2)) * u_radius);

    /* Light direction (from fragment toward sun) */
    vec3 L = normalize(u_sun_pos_view - frag_view);

    float diff  = max(dot(N, L), 0.0);
    float light = u_ambient + (1.0 - u_ambient) * diff;

    frag_color = vec4(u_color * light, 1.0);
}
