#version 330 core
/*
 * phong.vert — sphere billboard (oversized for off-axis coverage)
 *
 * The billboard is scaled by BILL_SCALE (2×) so that the sphere silhouette
 * is never clipped when the planet is near the viewport edge. The fragment
 * shader (ray-sphere intersection) handles the actual discard boundary.
 *
 * v_uv is passed in [-BILL_SCALE, +BILL_SCALE] so the fragment shader can
 * reconstruct the exact same world-space position as this vertex.
 */

layout(location = 0) in vec2 a_uv;   /* (0,0)..(1,1) unit quad */

uniform mat4  u_vp;
uniform vec3  u_center;
uniform float u_radius;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

out vec2 v_uv;

const float BILL_SCALE = 2.0;

void main() {
    vec2 off = a_uv * 2.0 - 1.0;           /* -1..+1                     */

    /* Oversized billboard: BILL_SCALE * radius in each camera axis */
    vec3 world = u_center
               + u_cam_right * (off.x * u_radius * BILL_SCALE)
               + u_cam_up    * (off.y * u_radius * BILL_SCALE);

    /* v_uv scaled to match: frag_bill = center + right*v_uv.x*radius + ... */
    v_uv        = off * BILL_SCALE;         /* -BILL_SCALE..+BILL_SCALE   */
    gl_Position = u_vp * vec4(world, 1.0);
}
