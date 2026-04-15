#version 330 core
/*
 * label.vert — camera-aligned billboard for text labels
 *
 * The CPU sends a unit quad (a_uv in 0..1) and per-label uniforms:
 *   u_anchor — world-space attachment point (just above body)
 *   u_right  — camera right vector  * label width  (world space)
 *   u_up     — camera up    vector  * label height (world space)
 *
 * The quad is placed so that its bottom-left corner is at u_anchor:
 *   world = anchor + right * a_uv.x + up * a_uv.y
 *
 * UV is passed to the fragment shader for texture lookup (y-flipped so
 * SDL_ttf surfaces render upright).
 */

layout(location = 0) in vec2 a_uv;

uniform mat4 u_vp;
uniform vec3 u_anchor;
uniform vec3 u_right;
uniform vec3 u_up;

out vec2 v_uv;

void main() {
    vec3 world  = u_anchor + u_right * a_uv.x + u_up * a_uv.y;
    /* Flip V: SDL surfaces have y=0 at top, GL textures at bottom */
    v_uv        = vec2(a_uv.x, 1.0 - a_uv.y);
    gl_Position = u_vp * vec4(world, 1.0);
}
