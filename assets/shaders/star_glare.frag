#version 330 core
/*
 * star_glare.frag — soft additive shine for star bodies
 *
 * Rendered with GL_ONE / GL_ONE (additive) over the disc pass.
 * v_uv is in solar-radius units; length(v_uv) == 1.0 at the disc edge.
 *
 * Single smooth exponential bloom — no spikes, no hard edges.
 *
 * Two fade multipliers guarantee the glow is exactly zero at the billboard
 * boundary so no sprite edge ring is ever visible:
 *   inner_fade  — ramps in over [0.5, 1.3] so the disc pass is not doubled
 *   outer_fade  — ramps out over [BILL_SCALE-4, BILL_SCALE] → smooth cutoff
 *
 * Colour: white-hot just outside the disc, transitions to star colour
 *         (typically solar yellow) as distance increases.
 */

in vec2 v_uv;

uniform vec3 u_color;

out vec4 frag_color;

const float BILL_SCALE = 15.0;

void main() {
    const float FAR = 2000.0;
    gl_FragDepth = log2(1.0 / gl_FragCoord.w + 1.0) / log2(FAR + 1.0);

    float r = length(v_uv);
    if (r >= BILL_SCALE) discard;

    float r_safe = max(r, 0.05);

    /* Smooth exponential bloom — covers disc + surround.
     * No inner_fade: the glow also brightens the disc area additively,
     * which eliminates the dark ring that appeared at the disc silhouette. */
    float shine = 2.1 * exp(-r_safe * 0.48);

    /* Very wide fade zone (9 solar radii) so the glow melts into black
     * gradually — no hard outer ring visible.                              */
    float outer_fade = 1.0 - smoothstep(BILL_SCALE - 9.0, BILL_SCALE, r);

    float total = shine * outer_fade;

    if (total < 0.001) discard;

    /* Colour: warm white near disc edge, pure star colour at distance */
    vec3 hot = vec3(1.3, 1.2, 0.85);          /* white-yellow hot core      */
    vec3 col = mix(hot, u_color, smoothstep(1.0, 4.0, r));

    frag_color = vec4(col * total, 1.0);
}
