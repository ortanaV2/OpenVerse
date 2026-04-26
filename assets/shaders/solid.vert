#version 330 core
/*
 * solid.vert — uniform-colour geometry with arc-length trail fades
 * Used for: orbital trails (GL_LINE_STRIP)
 *
 * Positions are camera-relative (pre-subtracted on CPU in double precision
 * to avoid float cancellation jitter when the camera is near the trail).
 * a_arc stores distance from the live tip along the trail in AU.
 */

layout(location = 0) in vec3 a_pos;
layout(location = 1) in float a_arc;

uniform mat4 u_vp;
uniform float u_visible_len;
uniform float u_front_fade;
uniform float u_tail_fade;

out float v_alpha;

void main() {
    float front = (u_front_fade > 0.0) ? smoothstep(0.0, u_front_fade, a_arc) : 1.0;
    float tail_start = max(0.0, u_visible_len - u_tail_fade);
    float tail = 1.0 - smoothstep(tail_start, u_visible_len, a_arc);
    v_alpha = front * tail;
    gl_Position = u_vp * vec4(a_pos, 1.0);
}
