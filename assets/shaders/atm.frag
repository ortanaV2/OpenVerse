#version 330 core
/*
 * atm.frag — atmospheric limb glow
 *
 * Per-pixel ray–atmosphere shell intersection.
 *
 * Glow model:
 *   p    = closest-approach distance of the ray to the planet centre
 *   norm = (p − R) / (R_atm − R)   maps [R, R_atm] → [0, 1]
 *   glow = (1 − norm)^3            smooth power-law falloff
 *
 * Using a power-law (rather than the chord-length model) gives a
 * continuously smooth gradient that fades to zero at R_atm with no
 * visible hard edge.  The cubic exponent concentrates the bulk of the
 * glow near the planet limb (norm ≈ 0) and tapers gently outward.
 *
 * Lit-side boost: dot(outward_normal, sun_dir) dims the night-side
 * glow to ~15 % of its day-side value.
 *
 * Rendered additively (GL_SRC_ALPHA / GL_ONE); logarithmic
 * gl_FragDepth keeps it correctly depth-sorted against opaque geometry.
 */

uniform vec3  u_oc;              /* cam − centre  (camera-relative, AU) */
uniform float u_planet_radius;   /* inner sphere radius  (AU)           */
uniform float u_radius;          /* outer atmosphere radius (AU)        */
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform vec3  u_cam_fwd;
uniform float u_fov_tan;
uniform float u_aspect;
uniform vec2  u_screen;

uniform vec3  u_sun_rel;         /* sun − centre (AU)                   */
uniform vec3  u_atm_color;
uniform float u_atm_intensity;

out vec4 frag_color;

void main() {
    /* Per-pixel ray direction — identical formula to phong.frag */
    vec2 ndc     = (gl_FragCoord.xy / (u_screen * 0.5)) - 1.0;
    vec3 ray_dir = normalize(u_cam_fwd
                           + u_cam_right * (ndc.x * u_aspect * u_fov_tan)
                           + u_cam_up    * (ndc.y * u_fov_tan));

    float b   = dot(u_oc, ray_dir);
    float oc2 = dot(u_oc, u_oc);
    float p2  = max(0.0, oc2 - b * b);  /* squared closest-approach dist */

    float R     = u_planet_radius;
    float R_atm = u_radius;

    /* Discard outside outer atmosphere shell */
    if (p2 > R_atm * R_atm) discard;

    /* Discard where ray hits the planet's front face (sphere renders that) */
    float d_inner = R * R - p2;
    if (d_inner >= 0.0) {
        float t = -b - sqrt(d_inner);
        if (t > 0.0) discard;
    }

    /* Front face of the outer atmosphere sphere */
    float t_atm = -b - sqrt(max(0.0, R_atm * R_atm - p2));
    if (t_atm <= 0.0) discard;   /* camera inside atmosphere */

    /* Smooth power-law falloff.
     * norm = 0 at the planet surface (limb), 1 at the outer atmosphere edge.
     * (1−norm)^3 concentrates the glow at the limb and tapers continuously
     * to zero — no visible hard cutoff at the boundary.                    */
    float p    = sqrt(p2);
    float norm = clamp((p - R) / (R_atm - R), 0.0, 1.0);
    float glow = pow(1.0 - norm, 3.0);

    /* Lit-side boost: outward normal at atmosphere hit point vs. sun dir */
    vec3  hit_n   = normalize(u_oc + t_atm * ray_dir);
    float sun_dot = dot(hit_n, normalize(u_sun_rel));
    float lit     = 0.15 + 0.85 * clamp(sun_dot, 0.0, 1.0);

    float alpha = glow * u_atm_intensity * lit;
    if (alpha < 0.003) discard;

    /* Logarithmic depth — use the BACK face of the atmosphere sphere.
     *
     * Using the front face (t_atm) causes the glow of body A to render
     * additively over body B whenever A's atmosphere front face sits in
     * front of B's solid surface — which happens during merges because the
     * target's atmosphere shell extends past the impactor's sphere.  The
     * back face (t_atm_back) is always behind any solid object that sits
     * inside the atmosphere shell, so the depth test correctly rejects the
     * glow at pixels occupied by a closer solid body.                    */
    const float FAR = 2000.0;
    float t_atm_back = -b + sqrt(max(0.0, R_atm * R_atm - p2));
    float eye_depth  = t_atm_back * dot(ray_dir, u_cam_fwd);
    gl_FragDepth = log2(eye_depth + 1.0) / log2(FAR + 1.0);

    frag_color = vec4(u_atm_color * alpha, alpha);
}
