#version 330 core
/*
 * solid.vert — uniform-colour geometry with gl_VertexID alpha fade
 * Used for: orbital trails (GL_LINE_STRIP)
 *
 * Positions are camera-relative (pre-subtracted on CPU in double precision
 * to avoid float cancellation jitter when the camera is near the trail).
 * Alpha fades quadratically from 0 (oldest vertex) to 1 (newest).
 */

layout(location = 0) in vec3 a_pos;

uniform mat4 u_vp;
uniform int  u_count;   /* total vertices in this draw call */

out float v_alpha;

void main() {
    float t   = (u_count > 1) ? float(gl_VertexID) / float(u_count - 1) : 1.0;
    v_alpha   = t * t;
    gl_Position = u_vp * vec4(a_pos, 1.0);
}
