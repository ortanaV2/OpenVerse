#version 330 core
/*
 * lensing.frag - gravitational black-hole compositor via geodesic ray bending.
 *
 * For every pixel whose view ray passes near a black hole we integrate the bent
 * photon path through a small window around the hole (step count and window
 * width are independent of how far the hole is, so the look is scale-stable):
 *
 *   - gravity from each hole pulls the ray inward (Newtonian 1/r^2 marcher);
 *   - if the path falls inside the event horizon the ray is captured -> black;
 *   - every time the path crosses a disk plane inside [r_inner, r_outer] the
 *     analytic disk emission is added, so the FAR side of the disk that wraps
 *     up and over the hole appears naturally as a second crossing;
 *   - the bent path is intersected against every scene body as an analytic
 *     SPHERE; the nearest hit along the path wins, so a body in front genuinely
 *     occludes the hole and a body behind is genuinely lensed (its image bends,
 *     wraps, or splits). A hit takes its colour from the body's real rasterised
 *     pixel (reprojected to screen, depth-checked) for full texture + lighting,
 *     falling back to analytic shading when that pixel isn't on screen;
 *   - a photon-ring glow is added for rays that graze the photon sphere;
 *   - rays that escape everything sample the sky-only texture in the FINAL
 *     deflected direction.
 *
 * Pixels outside every hole's influence pass the captured scene straight
 * through. This is real per-pixel ray-tracing of the bent geodesic against the
 * scene's sphere geometry, so occlusion and parallax are correct -- no warped
 * flat image, no ghost copies. UI/text is drawn after the resolve and stays crisp.
 */

in vec2 v_uv;

uniform sampler2D u_scene;   /* full scene (sky + near geometry) for passthrough */
uniform sampler2D u_stars;   /* sky only — safe to sample by bent ray direction  */
uniform sampler2D u_depth;   /* scene depth (shared log metric) for SSR hit tests */
uniform vec2  u_screen;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform vec3  u_cam_fwd;
uniform float u_fov_tan;
uniform int   u_count;

uniform vec3  u_center[4];
uniform float u_rh[4];
uniform float u_ri[4];
uniform float u_ro[4];
uniform vec3  u_du[4];
uniform vec3  u_dv[4];
uniform vec3  u_dn[4];
uniform vec3  u_color[4];
uniform float u_time[4];

/* Scene bodies, ray-traced as analytic spheres along the bent path. */
uniform int   u_nbody;
uniform vec3  u_body_c[32];
uniform float u_body_r[32];
uniform vec3  u_body_col[32];
uniform int   u_body_emis[32];
uniform vec3  u_sun;       /* camera-relative sun position (lambert light) */
uniform vec3  u_sun_col;   /* sun colour for diffuse shading              */

out vec4 frag_color;

float g_aspect;            /* x/y, set once in main, read by projection helpers */

/* ---- tunables ------------------------------------------------------------ */
const int   STEPS         = 96;     /* path integration steps per pixel        */
const float LENS_STRENGTH = 1.0;    /* GM = LENS_STRENGTH * r_horizon          */
const float SPAN_RH       = 14.0;   /* half march window, in horizon radii     */
const float SPAN_RO       = 1.4;    /* ...or in disk-outer radii, whichever wins*/
const float RING_GAIN     = 0.9;    /* photon-ring brightness                  */
const float RING_R        = 1.55;   /* ring peak, in horizon radii             */
const float RING_W        = 0.30;   /* ring width, in horizon radii            */
const float FAR           = 2000.0; /* shared far plane for the log-depth metric*/
const float DEPTH_BIAS    = 0.002;  /* tolerance so grazing rays don't acne     */

/* Encode/decode a camera-forward distance into the shared log metric every other
 * shader writes to gl_FragDepth, so a ray-traced point can be matched to u_depth. */
float encode_depth(float fwd) { return log2(fwd + 1.0) / log2(FAR + 1.0); }
float decode_depth(float e)   { return exp2(e * log2(FAR + 1.0)) - 1.0;     }

/* Project a camera-relative point to a screen UV; returns false if behind the
 * camera or off-screen. Forward distance returned in out_fwd. */
bool project_uv(vec3 hp, out vec2 uv, out float out_fwd) {
    out_fwd = dot(hp, u_cam_fwd);
    if (out_fwd <= 1e-4) return false;
    float sx = dot(hp, u_cam_right) / out_fwd;
    float sy = dot(hp, u_cam_up)    / out_fwd;
    uv = vec2(sx / (g_aspect * u_fov_tan), sy / u_fov_tan) * 0.5 + 0.5;
    return all(greaterThanEqual(uv, vec2(0.0))) &&
           all(lessThanEqual(uv, vec2(1.0)));
}

/* Simple fallback shading when a ray-traced body is NOT visible on screen at its
 * reprojected pixel (off-screen, or occluded there by nearer geometry). */
vec3 shade_body(int i, vec3 hp) {
    vec3 base = u_body_col[i];
    if (u_body_emis[i] != 0) return base;          /* star: self-luminous */
    vec3  nrm = normalize(hp - u_body_c[i]);
    vec3  l   = u_sun - hp;
    vec3  ld  = l / max(length(l), 1e-6);
    float diff = max(dot(nrm, ld), 0.0);
    return base * (0.04 + 0.96 * diff) * u_sun_col;
}

/* Colour of a body hit at hp: prefer its real rasterised pixel (full texture +
 * lighting) by reprojecting to screen and confirming the depth there matches the
 * hit distance; otherwise fall back to analytic shading. */
vec3 body_color(int i, vec3 hp) {
    vec2  uv; float hf;
    if (project_uv(hp, uv, hf)) {
        float scene_fwd = decode_depth(texture(u_depth, uv).r);
        if (abs(scene_fwd - hf) <= 0.06 * hf + 1e-3)
            return texture(u_scene, uv).rgb;
    }
    return shade_body(i, hp);
}

/* Analytic accretion-disk emission, identical to blackhole_disk.frag. */
vec3 disk_emit(int i, vec3 hr, float rr) {
    float v_t = clamp((rr - u_ri[i]) / max(u_ro[i] - u_ri[i], 1e-6), 0.0, 1.0);
    float ang = atan(dot(hr, u_dv[i]), dot(hr, u_du[i]));

    float radial  = smoothstep(0.0, 0.12, v_t) * (1.0 - smoothstep(0.55, 1.0, v_t));
    float arms    = sin(ang * 2.0 - u_time[i] * 2.0 + v_t * 9.0);
    float band    = 0.65 + 0.35 * arms * arms;
    float doppler = 1.0 + 0.45 * cos(ang);
    float intensity = radial * band * doppler * 1.6;

    vec3 hot = vec3(1.4, 1.2, 0.95);
    vec3 col = mix(hot, u_color[i], smoothstep(0.05, 0.6, v_t));
    return col * intensity;
}

void main() {
    float aspect = u_screen.x / u_screen.y;
    g_aspect     = aspect;
    vec2  ndc    = v_uv * 2.0 - 1.0;
    vec3  u = normalize(u_cam_fwd
                      + u_cam_right * (ndc.x * aspect * u_fov_tan)
                      + u_cam_up    * (ndc.y * u_fov_tan));

    /* Pick the dominant hole (smallest impact parameter relative to its reach)
     * to size the integration window; all holes still act inside that window. */
    int   di   = -1;
    float best = 1e30;
    float tc_d = 0.0;
    for (int i = 0; i < u_count; i++) {
        float tc = dot(u_center[i], u);
        if (tc <= 0.0) continue;                  /* hole behind closest approach */
        vec3  perp = u * tc - u_center[i];
        float b    = length(perp);
        float infl = max(8.0 * u_rh[i], u_ro[i] + 4.0 * u_rh[i]);
        float ratio = b / infl;
        if (ratio < best) { best = ratio; di = i; tc_d = tc; }
    }

    if (di < 0 || best >= 1.0) {                   /* no hole reaches this pixel */
        frag_color = texture(u_scene, v_uv);
        return;
    }

    float span = max(u_ro[di] * SPAN_RO, SPAN_RH * u_rh[di]);
    float t0   = max(tc_d - span, 0.0);
    float t1   = tc_d + span;
    float dt   = (t1 - t0) / float(STEPS);

    vec3  p = u * t0;
    vec3  v = u;

    vec3  accum    = vec3(0.0);
    bool  captured = false;
    float rmin     = 1e30;        /* closest approach to dominant hole, in r_h  */

    bool  body_hit = false;       /* bent path struck a scene body (sphere)      */
    vec3  body_rgb = vec3(0.0);   /* shaded colour of that body                  */

    for (int s = 0; s < STEPS; s++) {
        /* gravity + capture + closest-approach, summed over holes */
        vec3 a = vec3(0.0);
        for (int i = 0; i < u_count; i++) {
            vec3  d  = u_center[i] - p;
            float r2 = dot(d, d) + 1e-9;
            float r  = sqrt(r2);
            a += (d / r) * (LENS_STRENGTH * u_rh[i] / r2);
            if (r < u_rh[i]) captured = true;
            if (i == di) rmin = min(rmin, r / u_rh[i]);
        }
        if (captured) break;

        v += a * dt;
        vec3 pn = p + v * dt;

        /* disk-plane crossings (front image, then the wrapped far image) */
        for (int i = 0; i < u_count; i++) {
            if (u_ri[i] <= 0.0 || u_ro[i] <= u_ri[i]) continue;
            float s0 = dot(p  - u_center[i], u_dn[i]);
            float s1 = dot(pn - u_center[i], u_dn[i]);
            if (s0 * s1 < 0.0) {
                float frac = s0 / (s0 - s1);
                vec3  hit  = mix(p, pn, frac);
                vec3  hr   = hit - u_center[i];
                float rr   = length(hr);
                if (rr >= u_ri[i] && rr <= u_ro[i])
                    accum += disk_emit(i, hr, rr);
            }
        }

        /* analytic ray-tracing of scene bodies along this path segment. Each body
         * is a sphere; we solve the segment/sphere quadratic and keep the nearest
         * entry. Because we march outward, the first segment that hits any sphere
         * yields the nearest surface along the bent path -> first hit wins, so a
         * body in front naturally occludes the hole and one behind is lensed. */
        vec3  seg = pn - p;
        float seg2 = dot(seg, seg);
        float bt   = 2.0;
        int   bi   = -1;
        for (int i = 0; i < u_nbody; i++) {
            vec3  oc = p - u_body_c[i];
            float bq = dot(oc, seg);
            float cq = dot(oc, oc) - u_body_r[i] * u_body_r[i];
            float disc = bq * bq - seg2 * cq;
            if (disc < 0.0) continue;
            float sd = sqrt(disc);
            float t  = (-bq - sd) / seg2;          /* entry point */
            if (t < 0.0) t = (-bq + sd) / seg2;    /* segment started inside */
            if (t >= 0.0 && t <= 1.0 && t < bt) { bt = t; bi = i; }
        }
        if (bi >= 0) {
            vec3 hp = p + seg * bt;
            body_hit = true;
            body_rgb = body_color(bi, hp);
            p = hp;
            break;
        }
        p = pn;
    }

    /* Compose along the ray. accum already holds the disk emission crossed in
     * front of whatever the ray finally meets (disk gas is additive). */
    vec3 col = accum;
    if (captured) {
        /* fell through the horizon: nothing behind shows, only the disk in front */
    } else if (body_hit) {
        /* the bent ray struck a real body: that body's surface is the background */
        col += body_rgb;
    } else {
        /* the ray escaped past everything solid: add the photon-ring glow, then
         * the lensed background in the final deflected direction. */
        float ring = exp(-pow((rmin - RING_R) / RING_W, 2.0));
        col += u_color[di] * ring * RING_GAIN;

        vec3  dir = normalize(v);
        float fz  = dot(dir, u_cam_fwd);
        if (fz > 1e-3) {
            vec2  buv = vec2(dot(dir, u_cam_right) / (fz * aspect * u_fov_tan),
                             dot(dir, u_cam_up)    /  fz)                * 0.5 + 0.5;
            buv = clamp(buv, 0.0, 1.0);
            /* Bright EXTENDED structures behind the hole (star glare, atmospheres,
             * nebulae, supernova clouds) are near-geometry billboards, so they are
             * absent from the sky-only snapshot -- sampling u_stars there yields
             * black, which reads as a flat disc covering them instead of bending
             * them. So we sample the FULL scene by direction wherever the content
             * at that pixel sits BEHIND the hole (parallax-safe background); only
             * truly near content falls back to the sky-only texture to avoid the
             * ghost copies that direction-sampling near geometry would produce. */
            float bg_fwd = decode_depth(texture(u_depth, buv).r);
            if (bg_fwd > tc_d)  col += texture(u_scene, buv).rgb;
            else                col += texture(u_stars, buv).rgb;
        }
    }

    /* Feather the whole composite back into the untouched scene at the influence
     * boundary so there is no hard circular edge. Deep inside (best small) the
     * composite is used in full; near best=1 it fades to the plain passthrough,
     * where the deflection is already negligible. */
    vec3  through = texture(u_scene, v_uv).rgb;
    float edge    = smoothstep(1.0, 0.75, best);

    /* Occlusion: if real geometry rasterised at THIS pixel sits in front of the
     * hole's closest approach, that body hides the hole and all its lensing, so
     * the pixel must show the untouched scene. Without this the composite draws
     * over foreground bodies and the hole appears to shine through them. */
    float here_d = texture(u_depth, v_uv).r;
    float hole_d = encode_depth(tc_d * dot(u, u_cam_fwd));
    if (here_d < hole_d - DEPTH_BIAS) edge = 0.0;

    frag_color = vec4(mix(through, col, edge), 1.0);
}
