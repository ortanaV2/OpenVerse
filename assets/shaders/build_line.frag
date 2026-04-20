#version 330 core
in vec4 v_color;
out vec4 frag_color;

void main() {
    const float FAR = 2000.0;
    gl_FragDepth = log2(1.0 / gl_FragCoord.w + 1.0) / log2(FAR + 1.0);
    frag_color = v_color;
}
