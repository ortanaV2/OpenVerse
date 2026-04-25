#version 330 core
/*
 * atm.vert — atmospheric glow billboard
 *
 * Sizes the quad to an oversized outer-atmosphere billboard so off-axis
 * planets do not have their atmosphere clipped by the quad bounds.
 */

layout(location = 0) in vec2 a_uv;

uniform mat4  u_vp;
uniform vec3  u_center;      /* camera-relative planet centre */
uniform float u_radius;      /* outer atmosphere radius (AU)  */
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

const float BILL_SCALE = 2.0;

void main() {
    vec2 off    = a_uv * 2.0 - 1.0;          /* −1 .. +1 */
    vec3 pos    = u_center
                + u_cam_right * (off.x * u_radius * BILL_SCALE)
                + u_cam_up    * (off.y * u_radius * BILL_SCALE);
    gl_Position = u_vp * vec4(pos, 1.0);
}
