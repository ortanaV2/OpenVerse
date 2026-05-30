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
 *   - a photon-ring glow is added from the path's closest approach;
 *   - the star background is sampled in the FINAL deflected direction by
 *     projecting it back to a scene texcoord.
 *
 * Pixels outside every hole's influence pass the captured scene straight
 * through. This is a screen-space pass but it never warps a flat image: the
 * disk and horizon are computed from geometry, so there is no tearing or
 * ghosting -- only the background, which has no parallax, is re-sampled.
 */

in vec2 v_uv;

uniform sampler2D u_scene;   /* full scene (sky + near geometry) for passthrough */
uniform sampler2D u_stars;   /* sky only — safe to sample by bent ray direction  */
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

out vec4 frag_color;

/* ---- tunables ------------------------------------------------------------ */
const int   STEPS         = 96;     /* path integration steps per pixel        */
const float LENS_STRENGTH = 1.0;    /* GM = LENS_STRENGTH * r_horizon          */
const float SPAN_RH       = 14.0;   /* half march window, in horizon radii     */
const float SPAN_RO       = 1.4;    /* ...or in disk-outer radii, whichever wins*/
const float RING_GAIN     = 0.9;    /* photon-ring brightness                  */
const float RING_R        = 1.55;   /* ring peak, in horizon radii             */
const float RING_W        = 0.30;   /* ring width, in horizon radii            */

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
        p = pn;
    }

    /* photon-ring glow from rays that graze the photon sphere and escape */
    if (!captured) {
        float ring = exp(-pow((rmin - RING_R) / RING_W, 2.0));
        accum += u_color[di] * ring * RING_GAIN;
    }

    /* lensed background: sample the scene in the final deflected direction */
    vec3 col = accum;
    if (!captured) {
        vec3  dir = normalize(v);
        float fz  = dot(dir, u_cam_fwd);
        if (fz > 1e-3) {
            float nx = dot(dir, u_cam_right) / fz;
            float ny = dot(dir, u_cam_up)    / fz;
            vec2  buv = vec2(nx / (aspect * u_fov_tan), ny / u_fov_tan) * 0.5 + 0.5;
            /* Sample the SKY-ONLY texture: only the sky is at infinity, so only
             * it may be sampled by direction. Sampling the full scene here would
             * duplicate near geometry (trails, planets) as lensed ghost copies.
             * CLAMP_TO_EDGE keeps off-screen samples at the (dark) frame edge. */
            col += texture(u_stars, clamp(buv, 0.0, 1.0)).rgb;
        }
    }

    /* Feather the whole composite back into the untouched scene at the influence
     * boundary so there is no hard circular edge. Deep inside (best small) the
     * composite is used in full; near best=1 it fades to the plain passthrough,
     * where the deflection is already negligible. */
    vec3  through = texture(u_scene, v_uv).rgb;
    float edge    = smoothstep(1.0, 0.75, best);
    frag_color = vec4(mix(through, col, edge), 1.0);
}
