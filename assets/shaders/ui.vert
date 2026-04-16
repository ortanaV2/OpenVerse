#version 330 core
/*
 * ui.vert — 2D screen-space UI shader
 * Input positions are in pixel coordinates (origin = top-left).
 */
layout(location = 0) in vec2 a_pos;
layout(location = 1) in vec2 a_uv;

uniform vec2 u_screen;   /* WIN_W, WIN_H */

out vec2 v_uv;

void main() {
    vec2 ndc = vec2(
         a_pos.x / u_screen.x * 2.0 - 1.0,
        -a_pos.y / u_screen.y * 2.0 + 1.0   /* flip Y: pixel 0 = top */
    );
    gl_Position = vec4(ndc, 0.0, 1.0);
    v_uv = a_uv;
}
