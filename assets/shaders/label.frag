#version 330 core
/*
 * label.frag — sample SDL_ttf RGBA texture for text labels
 */

in  vec2 v_uv;

uniform sampler2D u_tex;

out vec4 frag_color;

void main() {
    frag_color = texture(u_tex, v_uv);
    /* Discard nearly-transparent pixels so depth buffer is clean */
    if (frag_color.a < 0.04) discard;
}
