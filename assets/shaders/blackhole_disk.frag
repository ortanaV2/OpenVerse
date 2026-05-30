#version 330 core
/*
 * blackhole_disk.frag — emissive, swirling accretion disk
 *
 * Rendered additively (GL_ONE / GL_ONE). Inner edge is white-hot and fades to
 * the disk's base colour outward. Spiral bands rotate via u_time, and a simple
 * relativistic-Doppler asymmetry brightens the approaching side.
 *
 * Depth is written with the same logarithmic metric as phong.frag so the
 * opaque event-horizon sphere correctly occludes the far half of the disk.
 */

in float v_t;     /* 0 inner .. 1 outer */
in vec2  v_dir;   /* raw in-plane direction (interpolated) */

uniform vec3  u_color;
uniform float u_time;   /* swirl phase, rad */

out vec4 frag_color;

void main() {
    const float FAR = 2000.0;
    gl_FragDepth = log2(1.0 / gl_FragCoord.w + 1.0) / log2(FAR + 1.0);

    /* Radial brightness: hot inner edge, smooth fade to the outer rim. */
    float inner_fade = smoothstep(0.0, 0.12, v_t);
    float outer_fade = 1.0 - smoothstep(0.55, 1.0, v_t);
    float radial = inner_fade * outer_fade;

    /* Azimuth taken per-pixel so the atan branch cut at +/-pi stays invisible:
     * every term below is 2*pi-periodic in v_ang, so the sign flip cancels. */
    float v_ang = atan(v_dir.y, v_dir.x);

    /* Spiral bands: arms wind inward and rotate over time. The u_time
     * coefficient must be an integer so wrapping u_time by 2*pi (see
     * blackhole.c) leaves the phase continuous -- otherwise the loop stutters. */
    float arms = sin(v_ang * 2.0 - u_time * 2.0 + v_t * 9.0);
    float band = 0.65 + 0.35 * arms * arms;

    /* Doppler-like asymmetry: brighten the side moving toward the viewer. */
    float doppler = 1.0 + 0.45 * cos(v_ang);

    float intensity = radial * band * doppler * 1.6;
    if (intensity < 0.002) discard;

    /* White-hot at the inner edge, base disk colour outward. */
    vec3 hot = vec3(1.4, 1.2, 0.95);
    vec3 col = mix(hot, u_color, smoothstep(0.05, 0.6, v_t));

    frag_color = vec4(col * intensity, 1.0);
}
