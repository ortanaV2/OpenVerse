#version 330 core
/*
 * asteroid_ball.vert — instanced Keplerian billboard for asteroid belts
 *
 * Base geometry  (divisor 0): vec2 quad corner in [−1, +1]
 * Per-instance   (divisor 1):
 *   a_orb0 = (M, a_AU, e, inc_rad)
 *   a_orb1 = (Omega_rad, omega_rad, radius_AU, bright)
 */

layout(location = 0) in vec2 a_quad;
layout(location = 1) in vec4 a_orb0;   /* (M, a, e, inc)                 */
layout(location = 2) in vec4 a_orb1;   /* (Omega, omega, radius, bright)  */

uniform mat4  u_vp;
uniform vec3  u_sun;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

out vec3  v_oc;
out float v_radius;
out float v_bright;

void main() {
    float M     = a_orb0.x;
    float a_au  = a_orb0.y;
    float e     = a_orb0.z;
    float inc   = a_orb0.w;
    float Omega = a_orb1.x;
    float omega = a_orb1.y;
    float R     = a_orb1.z;
    float bright= a_orb1.w;

    float nu = M + 2.0 * e * sin(M);
    float r  = a_au * (1.0 - e * cos(M));
    float f  = omega + nu;

    float cf = cos(f), sf = sin(f);
    float cO = cos(Omega), sO = sin(Omega);
    float ci = cos(inc),   si = sin(inc);

    float ecl_x = (cO * cf - sO * sf * ci) * r;
    float ecl_y = (sO * cf + cO * sf * ci) * r;
    float ecl_z =  si * sf * r;

    vec3 centre = u_sun + vec3(ecl_x, ecl_z, ecl_y);

    vec3 pos = centre
             + u_cam_right * (a_quad.x * R)
             + u_cam_up    * (a_quad.y * R);

    v_oc        = -centre;
    v_radius    = R;
    v_bright    = bright;
    gl_Position = u_vp * vec4(pos, 1.0);
}
