#version 330 core
/*
 * blackhole_disk.vert — flat annulus for a black-hole accretion disk
 *
 * The annulus is generated CPU-side in disk-local polar form:
 *   a_ring = (cos θ, sin θ)   unit direction in the disk plane
 *   a_t    = 0 at the inner edge, 1 at the outer edge
 *
 * The two orthonormal in-plane axes (u_axis_u, u_axis_v) come from the body's
 * obliquity, so the disk lies in the hole's equatorial plane. Radii are in AU.
 */

layout(location = 0) in vec2  a_ring;
layout(location = 1) in float a_t;

uniform mat4  u_vp;
uniform vec3  u_center;    /* camera-relative AU */
uniform vec3  u_axis_u;    /* unit in-plane axis */
uniform vec3  u_axis_v;    /* unit in-plane axis */
uniform float u_r_inner;   /* AU */
uniform float u_r_outer;   /* AU */

out float v_t;
out vec2  v_dir;   /* raw in-plane direction; angle is taken per-pixel */

void main() {
    float r = mix(u_r_inner, u_r_outer, a_t);
    vec3 world = u_center + (u_axis_u * a_ring.x + u_axis_v * a_ring.y) * r;
    v_t   = a_t;
    v_dir = a_ring;
    gl_Position = u_vp * vec4(world, 1.0);
}
