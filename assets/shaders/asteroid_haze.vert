#version 330 core
/*
 * asteroid_haze.vert — sparse dim GL_POINTS haze layer for asteroid belts
 *
 * Shows belt structure from a distance.  Points fade to zero when the
 * camera is very close (< 0.05 AU) so they don't fight the sphere layer.
 *
 * Data layout: 2 × vec4 per point
 *   loc 0: (M, a, e, inc)
 *   loc 1: (Omega, omega, _unused, bright)
 */

layout(location = 0) in vec4 a_orb0;   /* (M, a, e, inc)          */
layout(location = 1) in vec4 a_orb1;   /* (Omega, omega, _, bright) */

uniform mat4  u_vp;
uniform vec3  u_sun;           /* sun position, camera-relative (AU)      */
uniform vec3  u_color;         /* base belt tint                          */
uniform float u_fade_start;    /* AU from sun: begin fading               */
uniform float u_fade_end;      /* AU from sun: fully invisible            */
uniform float u_cam_dist_sun;  /* camera distance from sun (AU)           */

out vec3 v_color;

void main() {
    float M     = a_orb0.x;
    float a_au  = a_orb0.y;
    float e     = a_orb0.z;
    float inc   = a_orb0.w;
    float Omega = a_orb1.x;
    float omega = a_orb1.y;
    float bright= a_orb1.w;

    float nu = M + 2.0 * e * sin(M);
    float r  = a_au * (1.0 - e * cos(M));
    float f  = omega + nu;

    float cf = cos(f), sf = sin(f);
    float cO = cos(Omega), sO = sin(Omega);
    float ci = cos(inc),   si = sin(inc);

    float ecl_x = (cO*cf - sO*sf*ci) * r;
    float ecl_y = (sO*cf + cO*sf*ci) * r;
    float ecl_z =  si * sf * r;

    /* ecliptic → GL (Y=north, Z=east) */
    vec3 pos = u_sun + vec3(ecl_x, ecl_z, ecl_y);

    /* Distance from camera (cam = origin in camera-relative space) */
    float dist = length(pos);

    /* Fade near: invisible within 0.05 AU (sphere layer takes over),
     * full brightness beyond 0.3 AU.                                */
    float fade_near = smoothstep(0.05, 0.30, dist);

    /* Fade far: driven by camera distance from sun — whole belt fades
     * as the camera leaves the solar system.                         */
    float fade_far = 1.0 - smoothstep(u_fade_start, u_fade_end, u_cam_dist_sun);

    float b   = (0.06 + 0.06 * bright) * fade_near * fade_far;
    v_color   = u_color * b;

    gl_Position = u_vp * vec4(pos, 1.0);
}
