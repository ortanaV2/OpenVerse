#version 330 core
/*
 * lensing.vert - fullscreen pass for the gravitational-lensing resolve.
 * The quad is given directly in clip space; v_uv is the [0,1] scene texcoord.
 */
layout(location = 0) in vec2 a_pos;   /* -1..+1 clip-space quad */

out vec2 v_uv;

void main() {
    v_uv = a_pos * 0.5 + 0.5;
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
