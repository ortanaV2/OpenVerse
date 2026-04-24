#version 330 core
/*
 * impact_particle.vert — glowing impact debris point sprites
 *
 * Positions are uploaded camera-relative each frame.
 * Point size scales with camera distance so particles appear larger when
 * close and smaller when far away.
 */

layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec4 a_color;
layout(location = 2) in float a_size;

uniform mat4 u_vp;

out vec4 v_color;
out float v_size;

void main() {
    float dist = max(length(a_pos), 0.001);
    gl_PointSize = clamp((0.22 * a_size) / dist, 4.0 * a_size, 16.0 * a_size);
    v_color = a_color;
    v_size = a_size;
    gl_Position = u_vp * vec4(a_pos, 1.0);
}
