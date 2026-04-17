#version 330 core
/*
 * asteroid_particle.vert — gravity-integrated asteroid belt particles
 *
 * Positions are computed on the CPU (N-body, major bodies only) and
 * uploaded camera-relative each frame.
 *
 * Point size grows as the camera approaches (clamped to 1–3 px).
 * Two smooth fades:
 *   near — fades IN from 0 to 0.08 AU so very-close particles don't pop
 *   far  — fades OUT driven by u_fade_start / u_fade_end (AU)
 */

layout(location = 0) in vec3  a_pos;    /* camera-relative position (AU) */
layout(location = 1) in float a_bright; /* per-particle brightness [0,1] */

uniform mat4  u_vp;
uniform vec3  u_color;
uniform float u_fade_start;   /* AU from camera: begin fading out */
uniform float u_fade_end;     /* AU from camera: fully invisible  */

out vec3 v_color;

void main() {
    float dist = length(a_pos);

    /* Fade near: particles invisible at < 0.02 AU (you're inside the belt) */
    float fade_near = smoothstep(0.02, 0.08, dist);

    /* Fade far: whole belt fades as camera recedes */
    float fade_far  = 1.0 - smoothstep(u_fade_start, u_fade_end, dist);

    float alpha = (0.28 + 0.28 * a_bright) * fade_near * fade_far;
    v_color     = u_color * alpha;

    /* Larger point when closer — stays crisp at distance */
    gl_PointSize = clamp(0.6 / (dist + 0.001), 1.0, 3.0);

    gl_Position = u_vp * vec4(a_pos, 1.0);
}
