#version 330 core
/*
 * ring.vert — Keplerian ring particle animation on the GPU
 *
 * Each particle stores its mean anomaly (a_M), semi-major axis (a_a),
 * eccentricity (a_e), argument of periapsis (a_omega), and height (a_height).
 * The CPU advances a_M each physics step by  ΔM = n·dt  (pure addition).
 * The GPU computes the first-order Keplerian solution each frame:
 *
 *   ν ≈ M + 2e·sin(M)          true anomaly  (O(e²) error)
 *   r ≈ a·(1 − e·cos(M))       orbital radius (O(e²) error)
 *
 * This makes inner particles orbit faster than outer ones (Kepler's 3rd law)
 * and gives each particle a slightly elliptical path around Saturn.
 */
layout(location = 0) in float a_M;      /* mean anomaly (rad)              */
layout(location = 1) in float a_a;      /* semi-major axis (AU)            */
layout(location = 2) in float a_e;      /* eccentricity                    */
layout(location = 3) in float a_omega;  /* argument of periapsis (rad)     */
layout(location = 4) in float a_height; /* vertical offset (AU)            */
layout(location = 5) in vec3  a_color;

uniform mat4 u_vp;
uniform vec3 u_center;   /* Saturn world pos (AU) */
uniform vec3 u_b1;       /* ring-plane basis 1 */
uniform vec3 u_b2;       /* ring-plane basis 2 */
uniform vec3 u_pole;     /* ring-plane normal (Saturn pole) */

out vec4 v_color;

void main() {
    /* First-order Keplerian approximation — valid for e < 0.05 */
    float sinM = sin(a_M);
    float cosM = cos(a_M);
    float nu   = a_M + 2.0 * a_e * sinM;    /* true anomaly  */
    float r    = a_a * (1.0 - a_e * cosM);  /* orbital radius */

    float c = cos(nu + a_omega);
    float s = sin(nu + a_omega);

    vec3 pos = u_center
             + r * (c * u_b1 + s * u_b2)
             + a_height * u_pole;

    v_color     = vec4(a_color, 1.0);
    gl_Position = u_vp * vec4(pos, 1.0);
}
