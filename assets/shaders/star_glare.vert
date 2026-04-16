#version 330 core
/*
 * star_glare.vert — oversized billboard for star glare/bloom
 *
 * Identical layout to phong.vert but BILL_SCALE is 12× instead of 2×
 * so the quad covers the full glare halo and diffraction-spike region.
 *
 * v_uv is in solar-radius units:
 *   length(v_uv) == 1.0  at the disc silhouette edge
 *   length(v_uv) == 12.0 at the billboard corners (clipped to a circle
 *                         by the fragment shader)
 */

layout(location = 0) in vec2 a_uv;

uniform mat4  u_vp;
uniform vec3  u_center;   /* star centre, world AU */
uniform float u_radius;   /* physical radius, AU   */
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

out vec2 v_uv;

const float BILL_SCALE = 15.0;

void main() {
    vec2 off   = a_uv * 2.0 - 1.0;          /* -1 .. +1 */
    vec3 world = u_center
               + u_cam_right * (off.x * u_radius * BILL_SCALE)
               + u_cam_up    * (off.y * u_radius * BILL_SCALE);
    v_uv        = off * BILL_SCALE;          /* solar-radius units */
    gl_Position = u_vp * vec4(world, 1.0);
}
