#version 330 core
/*
 * phong.frag — per-pixel ray-sphere intersection + procedural surface texturing
 *
 * Texturing uses 3D value-noise FBM keyed on the surface normal rotated into
 * the body's local frame.  3D noise is seam-free by construction — no UV
 * atlas, no texture files required.
 *
 * u_planet_type selects the colour recipe:
 *   0  generic rocky   (modulates u_color with noise)
 *   1  Earth           (ocean / land / desert / polar ice)
 *   2  Mars            (red-orange with polar frost)
 *   3  Venus           (swirly yellow-white clouds)
 *   4  Jupiter         (orange-brown horizontal bands)
 *   5  Saturn          (tan-cream horizontal bands)
 *   6  ice giant       (smooth pale tint with faint bands)
 *   7  Io              (yellow-orange with dark volcanic spots)
 *   8  Titan           (orange haze)
 *   9  Europa          (icy white with rust cracks)
 */

in vec2 v_uv;   /* unused for planets, kept for linker compatibility */

uniform mat4  u_vp;
uniform vec3  u_color;
uniform float u_emission;
uniform float u_ambient;
uniform vec3  u_sun_pos_world;
uniform vec3  u_center;
uniform float u_radius;
uniform vec3  u_oc;
uniform vec3  u_sun_rel;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform vec3  u_cam_fwd;
uniform float u_fov_tan;
uniform float u_aspect;

uniform float u_rotation;     /* body rotation angle, radians           */
uniform float u_obliquity;    /* axial tilt, radians (0 = upright)      */
uniform int   u_planet_type;  /* 0-9, selects colour recipe             */

out vec4 frag_color;

/* ======================================================================
 * 3-D value noise — no seams, rotation-aware
 * ====================================================================== */

float hash3(vec3 p) {
    p  = fract(p * vec3(127.1, 311.7, 74.7));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float vnoise(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);          /* smooth-step */
    return mix(
        mix(mix(hash3(i),               hash3(i + vec3(1,0,0)), f.x),
            mix(hash3(i + vec3(0,1,0)), hash3(i + vec3(1,1,0)), f.x), f.y),
        mix(mix(hash3(i + vec3(0,0,1)), hash3(i + vec3(1,0,1)), f.x),
            mix(hash3(i + vec3(0,1,1)), hash3(i + vec3(1,1,1)), f.x), f.y),
        f.z);
}

/* 2-octave FBM — deliberately blurry (rough and washy, as requested) */
float fbm(vec3 p) {
    return vnoise(p)               * 0.65
         + vnoise(p * 2.1 + vec3(7.3, 2.1, 5.8)) * 0.35;
}

/* ======================================================================
 * Per-planet surface colour  (NL = normal in body-local frame)
 * ====================================================================== */

vec3 surface_color(vec3 NL) {
    float n = fbm(NL * 3.5);

    /* ---- Earth -------------------------------------------------------- */
    if (u_planet_type == 1) {
        float n2  = fbm(NL * 3.5 + vec3(5.3, 2.7, 8.1));
        float lat = abs(NL.y);
        vec3 col;
        if (n < 0.44)
            col = mix(vec3(0.04, 0.13, 0.52), vec3(0.08, 0.28, 0.68),
                      n / 0.44);
        else if (n < 0.68)
            col = mix(vec3(0.22, 0.46, 0.14), vec3(0.58, 0.46, 0.22),
                      (n - 0.44) / 0.24);
        else
            col = mix(vec3(0.58, 0.46, 0.22), vec3(0.70, 0.65, 0.50),
                      (n - 0.68) / 0.32);
        /* polar ice caps */
        col = mix(col, vec3(0.88, 0.91, 0.96),
                  smoothstep(0.82, 0.97, lat));
        return col;
    }

    /* ---- Mars --------------------------------------------------------- */
    if (u_planet_type == 2) {
        vec3 col = mix(vec3(0.38, 0.16, 0.06), vec3(0.86, 0.34, 0.13), n);
        col = mix(col, vec3(0.84, 0.81, 0.76),
                  smoothstep(0.88, 0.97, abs(NL.y)));
        return col;
    }

    /* ---- Venus -------------------------------------------------------- */
    if (u_planet_type == 3) {
        float n2 = fbm(NL * 4.5 + vec3(3.7, 1.2, 8.4));
        return mix(vec3(0.74, 0.66, 0.36), vec3(0.96, 0.90, 0.60),
                   n * 0.5 + n2 * 0.5);
    }

    /* ---- Jupiter ------------------------------------------------------ */
    if (u_planet_type == 4) {
        float band = sin(NL.y * 18.0 + (n - 0.5) * 3.2) * 0.5 + 0.5;
        return mix(vec3(0.48, 0.32, 0.14), vec3(0.94, 0.80, 0.54),
                   band * 0.65 + n * 0.35);
    }

    /* ---- Saturn ------------------------------------------------------- */
    if (u_planet_type == 5) {
        float band = sin(NL.y * 11.0 + (n - 0.5) * 2.2) * 0.5 + 0.5;
        return mix(vec3(0.58, 0.48, 0.24), vec3(0.98, 0.92, 0.64),
                   band * 0.65 + n * 0.35);
    }

    /* ---- Ice giant (Uranus / Neptune) --------------------------------- */
    if (u_planet_type == 6) {
        float band = sin(NL.y * 5.0 + (n - 0.5)) * 0.07 + 0.93;
        return u_color * (0.72 + 0.56 * n) * band;
    }

    /* ---- Io ----------------------------------------------------------- */
    if (u_planet_type == 7) {
        float n2 = fbm(NL * 8.5 + vec3(5.1, 3.2, 7.4));
        vec3 col;
        if (n < 0.5)
            col = mix(vec3(0.88, 0.80, 0.22), vec3(0.84, 0.38, 0.06), n * 2.0);
        else
            col = mix(vec3(0.88, 0.80, 0.22), vec3(0.93, 0.90, 0.84),
                      (n - 0.5) * 2.0);
        /* dark volcanic patches */
        col = mix(col, vec3(0.10, 0.07, 0.03),
                  smoothstep(0.72, 0.84, n2) * 0.75);
        return col;
    }

    /* ---- Titan -------------------------------------------------------- */
    if (u_planet_type == 8) {
        float n2 = fbm(NL * 2.2 + vec3(4.2, 1.8, 3.3));
        return mix(vec3(0.50, 0.28, 0.06), vec3(0.84, 0.62, 0.26),
                   n2 * 0.6 + 0.2);
    }

    /* ---- Europa ------------------------------------------------------- */
    if (u_planet_type == 9) {
        float n2 = fbm(NL * 9.0 + vec3(2.2, 6.6, 1.1));
        vec3 col = mix(vec3(0.84, 0.82, 0.79), vec3(0.65, 0.56, 0.40),
                       n * 0.25);
        /* rust-brown cracks */
        col = mix(col, vec3(0.44, 0.28, 0.16),
                  smoothstep(0.60, 0.72, n2) * 0.55);
        return col;
    }

    /* ---- type 0: generic rocky --------------------------------------- */
    return mix(u_color * 0.55, u_color * 1.28, n);
}

/* ====================================================================== */

void main() {
    /* ---- per-pixel ray ------------------------------------------------ */
    vec2 ndc    = gl_FragCoord.xy / vec2(u_aspect * 360.0, 360.0) - 1.0;
    vec3 ray_dir = normalize(u_cam_fwd
                           + u_cam_right * (ndc.x * u_aspect * u_fov_tan)
                           + u_cam_up    * (ndc.y * u_fov_tan));

    /* ---- ray-sphere intersection --------------------------------------- */
    float b_   = dot(u_oc, ray_dir);
    float c_   = dot(u_oc, u_oc) - u_radius * u_radius;
    float disc = b_ * b_ - c_;
    if (disc < 0.0) discard;

    float t = -b_ - sqrt(disc);
    if (t < 0.0) t = -b_ + sqrt(disc);
    if (t < 0.0) discard;

    vec3 hit_rel = u_oc + t * ray_dir;
    vec3 N       = normalize(hit_rel);

    /* ---- logarithmic depth — consistent with solid.frag / color.frag
     * All shaders must use the same depth metric: view-direction depth
     * (eye_depth = distance along the view axis, NOT Euclidean distance).
     * Using raw t (Euclidean) causes t > eye_depth off-centre, making the
     * planet appear deeper in the log buffer than rings/trails, which use
     * 1/gl_FragCoord.w = eye_depth.  The mismatch only shows when the body
     * is not centred on screen (rings render in front of the sphere limb).
     *
     * eye_depth = t * dot(ray_dir, u_cam_fwd)  [= t * cos(θ) off-axis]
     * This equals 1/gl_FragCoord.w for all non-raycast geometry.         */
    const float FAR  = 2000.0;
    float eye_depth  = t * dot(ray_dir, u_cam_fwd);
    gl_FragDepth = log2(eye_depth + 1.0) / log2(FAR + 1.0);

    /* ---- emissive (stars) --------------------------------------------- */
    if (u_emission > 0.5) {
        float mu     = max(dot(N, -ray_dir), 0.0);
        float one_mu = 1.0 - mu;
        float limb   = 1.0 - 0.38 * one_mu - 0.31 * one_mu * one_mu;
        vec3 white   = vec3(1.0, 0.98, 0.95);
        vec3 disc    = mix(white, u_color, 0.15 * one_mu);
        frag_color   = vec4(disc * limb, 1.0);
        return;
    }

    /* ---- procedural surface texture ----------------------------------- */
    /* Rotate N into body-local frame.
     * The spin axis is tilted from ecliptic-north (Y) toward +X by the
     * body's obliquity: axis = (sin(obl), cos(obl), 0).
     * Apply the inverse body rotation (Rodrigues, angle = -u_rotation).
     * When u_obliquity == 0 this reduces to a plain Y-axis rotation.    */
    vec3 NL;
    {
        /* Step 1 — undo axial tilt: rotate N around Z by +obliquity.
         * This maps the tilted spin axis back onto the Y axis so the
         * subsequent Y-rotation is a simple spin around the new Y.     */
        float co = cos(u_obliquity);
        float so = sin(u_obliquity);
        float tx =  co * N.x - so * N.y;
        float ty =  so * N.x + co * N.y;
        float tz =  N.z;

        /* Step 2 — undo body spin: plain Y-axis rotation (same convention
         * as the original no-tilt code).                                */
        float cr = cos(-u_rotation);
        float sr = sin(-u_rotation);
        NL = vec3(tx * cr - tz * sr,
                  ty,
                  tx * sr + tz * cr);
    }

    vec3 surface = surface_color(NL);

    /* ---- Phong diffuse ------------------------------------------------ */
    vec3  L     = normalize(u_sun_rel - hit_rel);
    float diff  = max(dot(N, L), 0.0);
    float light = u_ambient + (1.0 - u_ambient) * diff;

    frag_color = vec4(surface * light, 1.0);
}
