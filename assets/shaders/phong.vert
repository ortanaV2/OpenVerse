#version 330 core
/*
 * phong.vert — sphere billboard via instanced procedural geometry
 *
 * Each sphere is drawn as a unit quad (positions 0..1 in xy, passed as
 * attribute 0).  The vertex shader expands it into a world-space billboard
 * facing the camera, then the fragment shader ray-marches against a perfect
 * sphere for correct silhouette and depth.
 *
 * Uniforms set per body:
 *   u_vp      — view-projection matrix
 *   u_center  — sphere centre in world (AU)
 *   u_radius  — sphere visual radius (AU)
 *   u_cam_right / u_cam_up — camera billboard axes (world space, unit)
 */

layout(location = 0) in vec2 a_uv;   /* (0,0)..(1,1) unit quad */

uniform mat4  u_vp;
uniform vec3  u_center;
uniform float u_radius;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

out vec2 v_uv;    /* passes through for fragment ray-march */

void main() {
    /* Map UV (0,0)..(1,1) → (-1,-1)..(+1,+1) */
    vec2 off = a_uv * 2.0 - 1.0;

    /* Billboard vertex in world space */
    vec3 world = u_center
                + u_cam_right * (off.x * u_radius)
                + u_cam_up    * (off.y * u_radius);

    v_uv        = off;   /* -1..+1 for unit-disk test in fragment */
    gl_Position = u_vp * vec4(world, 1.0);
}
