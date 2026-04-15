#version 330 core
/*
 * solid.vert — uniform-colour geometry with gl_VertexID alpha fade
 * Used for: orbital trails (GL_LINE_STRIP)
 *
 * Alpha fades linearly from 0 (oldest point, vertex 0) to 1 (newest,
 * vertex u_count-1).  The trail circular buffer is unrolled oldest→newest
 * before upload, so vertex 0 is always the tail.
 */

layout(location = 0) in vec3 a_pos;

uniform mat4 u_vp;
uniform int  u_count;   /* total number of vertices in this draw call */

out float v_alpha;

void main() {
    /* Smoothstep for a nicer fade than pure linear */
    float t   = (u_count > 1) ? float(gl_VertexID) / float(u_count - 1) : 1.0;
    v_alpha   = t * t;   /* quadratic: very faint tail, bright head */

    gl_Position = u_vp * vec4(a_pos, 1.0);
}
