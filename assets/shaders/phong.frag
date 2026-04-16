#version 330 core
/*
 * phong.frag — per-pixel ray-sphere intersection using gl_FragCoord
 *
 * Key detail: after intersecting the true sphere surface we write the
 * correct clip-space depth to gl_FragDepth.  This means the depth buffer
 * holds the *sphere surface* depth, not the flat billboard quad depth.
 * Trails and other geometry drawn later are then correctly occluded by
 * the planet — they disappear where they would pass through the sphere.
 *
 * u_vp is declared in phong.vert; GLSL uniforms are program-wide so the
 * same value is visible here without an extra upload.
 */

in vec2 v_uv;   /* unused for planets, kept for linker compatibility */

uniform mat4  u_vp;
uniform vec3  u_color;
uniform float u_emission;
uniform float u_ambient;
uniform vec3  u_sun_pos_world;   /* kept for depth-write only          */
uniform vec3  u_center;          /* body centre, world AU              */
uniform float u_radius;
uniform vec3  u_oc;              /* cam_pos - center, computed CPU-side in double */
uniform vec3  u_sun_rel;         /* sun_pos - center, computed CPU-side in double */
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform vec3  u_cam_fwd;
uniform float u_fov_tan;
uniform float u_aspect;

out vec4 frag_color;

void main() {
    /* ---- per-pixel ray (perspective-correct via gl_FragCoord) ----------- */
    vec2 ndc = gl_FragCoord.xy / vec2(u_aspect * 360.0, 360.0) - 1.0;
    vec3 ray_dir = normalize(u_cam_fwd
                           + u_cam_right * (ndc.x * u_aspect * u_fov_tan)
                           + u_cam_up    * (ndc.y * u_fov_tan));

    /* ---- ray-sphere intersection (all in center-relative space) ----------
     * u_oc = cam_pos - center, computed CPU-side with double precision.
     * This avoids catastrophic cancellation when center >> radius (e.g.
     * Haumea at 43 AU with radius 0.000004 AU).                           */
    float b_   = dot(u_oc, ray_dir);
    float c_   = dot(u_oc, u_oc) - u_radius * u_radius;
    float disc = b_ * b_ - c_;

    if (disc < 0.0) discard;

    float t = -b_ - sqrt(disc);
    if (t < 0.0) t = -b_ + sqrt(disc);
    if (t < 0.0) discard;

    vec3 hit_rel = u_oc + t * ray_dir;      /* hit relative to centre     */
    vec3 N       = normalize(hit_rel);
    vec3 hit     = u_center + hit_rel;      /* world space (for depth)    */

    /* ---- write true sphere-surface depth -------------------------------- */
    vec4 hit_clip  = u_vp * vec4(hit, 1.0);
    gl_FragDepth   = (hit_clip.z / hit_clip.w) * 0.5 + 0.5;

    /* ---- emissive (sun / stars) ---------------------------------------- */
    if (u_emission > 0.5) {
        float mu = max(dot(N, -ray_dir), 0.0);   /* 1 at disc centre, 0 at limb */

        /* Quadratic limb darkening (Claret 2000, visual band)
         * I(µ) = 1 − 0.38·(1−µ) − 0.31·(1−µ)²                           */
        float one_mu = 1.0 - mu;
        float limb   = 1.0 - 0.38 * one_mu - 0.31 * one_mu * one_mu;

        /* Disc is white — the additive glow pass provides the star's colour.
         * A very faint warmth at the limb (one_mu term) keeps it from looking
         * sterile; the limb-darkening already makes the edge dim anyway.     */
        vec3 white = vec3(1.0, 0.98, 0.95);
        vec3 disc  = mix(white, u_color, 0.15 * one_mu);

        frag_color = vec4(disc * limb, 1.0);
        return;
    }

    /* ---- Phong diffuse -------------------------------------------------- */
    vec3  L     = normalize(u_sun_rel - hit_rel); /* sun relative to centre */
    float diff  = max(dot(N, L), 0.0);
    float light = u_ambient + (1.0 - u_ambient) * diff;

    frag_color = vec4(u_color * light, 1.0);
}
