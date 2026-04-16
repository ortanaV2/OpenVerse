#version 330 core
/*
 * ring_sprite.vert — pass-through for the far-LOD ring disc
 *
 * Vertices are computed CPU-side in Saturn-relative world space.
 * The fragment shader does all the ring-zone math.
 */
layout(location = 0) in vec3 a_pos;

uniform mat4 u_vp;

out vec3 v_pos;   /* world-space position for fragment shader */

void main() {
    v_pos       = a_pos;
    gl_Position = u_vp * vec4(a_pos, 1.0);
}
