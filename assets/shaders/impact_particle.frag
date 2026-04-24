#version 330 core
/*
 * impact_particle.frag — soft glowing point sprite for impact debris
 */

in vec4 v_color;
in float v_size;
out vec4 frag_color;

void main() {
    const float FAR = 2000.0;
    vec2 uv = gl_PointCoord * 2.0 - 1.0;
    float r = length(uv);
    if (r > 1.0) discard;

    gl_FragDepth = log2(1.0 / gl_FragCoord.w + 1.0) / log2(FAR + 1.0);

    float compact_r = r * max(v_size, 1.0);
    float glow = exp(-compact_r * 4.6);
    float core = 1.0 - smoothstep(0.0, 0.34, compact_r);
    float alpha = max(glow * 0.85, core);

    vec3 hot = vec3(1.15, 1.0, 0.82);
    vec3 col = mix(v_color.rgb, hot, 0.35);
    frag_color = vec4(col * alpha, v_color.a * alpha);
}
