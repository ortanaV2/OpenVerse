#version 330 core
/*
 * color.frag — outputs interpolated vertex colour as-is.
 * When rendering GL_POINTS, pixels outside the inscribed circle are
 * discarded so points appear round instead of square.
 */

in  vec3 v_color;
out vec4 frag_color;

void main() {
    /* Logarithmic depth — consistent with phong.frag.
     * gl_FragCoord.w = 1/t for perspective, so t = 1/gl_FragCoord.w.   */
    const float FAR = 2000.0;
    gl_FragDepth = log2(1.0 / gl_FragCoord.w + 1.0) / log2(FAR + 1.0);

    /* gl_PointCoord is (0,0)..(1,1) across the point sprite.
     * Discard anything further than 0.5 from the centre. */
    float d = length(gl_PointCoord - vec2(0.5));
    if (d > 0.5) discard;

    frag_color = vec4(v_color, 1.0);
}
