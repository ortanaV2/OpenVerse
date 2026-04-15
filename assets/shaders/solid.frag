#version 330 core
/*
 * solid.frag — uniform colour with per-vertex alpha (from solid.vert)
 */

in  float v_alpha;

uniform vec4 u_color;   /* rgb from body colour, a = base opacity (0.6) */

out vec4 frag_color;

void main() {
    frag_color = vec4(u_color.rgb, u_color.a * v_alpha);
}
