#version 330 core
/*
 * ui.frag — flat colour or textured quad
 */
in vec2 v_uv;

uniform vec4      u_color;
uniform sampler2D u_tex;
uniform int       u_use_tex;

out vec4 frag_color;

void main() {
    if (u_use_tex == 1)
        frag_color = texture(u_tex, v_uv) * u_color;
    else
        frag_color = u_color;
}
