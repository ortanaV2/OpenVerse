#version 330 core
/*
 * atm.vert — atmospheric glow billboard
 *
 * Sizes the quad to the outer atmosphere radius (planet_r × scale).
 * v_uv ∈ [−1, +1] maps to the outer atmosphere edge, used in atm.frag
 * only for the fast corner-discard (billboard is square, glow is circular).
 */

layout(location = 0) in vec2 a_uv;

uniform mat4  u_vp;
uniform vec3  u_center;      /* camera-relative planet centre */
uniform float u_radius;      /* outer atmosphere radius (AU)  */
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

out vec2 v_uv;

void main() {
    vec2 off    = a_uv * 2.0 - 1.0;          /* −1 .. +1 */
    vec3 pos    = u_center
                + u_cam_right * (off.x * u_radius)
                + u_cam_up    * (off.y * u_radius);
    v_uv        = off;
    gl_Position = u_vp * vec4(pos, 1.0);
}
