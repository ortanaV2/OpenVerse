#version 330 core
/*
 * color.vert — pass-through vertex colour shader
 * Used for: background starfield (GL_POINTS), body centre dots (GL_POINTS)
 */

layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec4 a_color;

uniform mat4 u_vp;

out vec4 v_color;

void main() {
    v_color     = a_color;
    gl_Position = u_vp * vec4(a_pos, 1.0);
}
