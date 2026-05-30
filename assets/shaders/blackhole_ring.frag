#version 330 core
/*
 * blackhole_ring.frag — photon ring + thin outer glow for a black hole
 *
 * The ring is anchored to the event horizon by re-running the SAME per-pixel
 * ray-sphere intersection that phong.frag uses for the dark sphere, instead of
 * reading a flat billboard UV. A billboard's centre is the projection of the
 * 3-D centre, but a sphere seen off-axis projects to a silhouette whose centre
 * drifts toward the frame edge (and the raycast also applies the horizontal
 * aspect factor) — so a billboard ring slides off the sphere during horizontal
 * pans. Deriving the radial coordinate from the ray keeps the two locked.
 *
 * Radial coordinate:
 *   r = (perpendicular distance from the hole centre to the view ray) / radius
 *   r == 1.0 exactly on the silhouette;  r < 1.0 is the event-horizon interior.
 *
 * The quad supplied by star_glare.vert only rasterises the screen region; all
 * geometry comes from the ray. Additive (GL_ONE / GL_ONE).
 */

in vec2 v_uv;   /* unused: raster coverage only */

uniform vec3  u_color;
uniform vec3  u_oc;        /* camera − centre, AU (same as phong's u_oc) */
uniform float u_radius;    /* event-horizon radius, AU */
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;
uniform vec3  u_cam_fwd;
uniform float u_fov_tan;
uniform float u_aspect;
uniform vec2  u_screen;

out vec4 frag_color;

const float BILL_SCALE = 15.0;

void main() {
    /* Reconstruct the per-pixel view ray exactly as phong.frag does. */
    vec2 ndc = (gl_FragCoord.xy / (u_screen * 0.5)) - 1.0;
    vec3 ray_dir = normalize(u_cam_fwd
                           + u_cam_right * (ndc.x * u_aspect * u_fov_tan)
                           + u_cam_up    * (ndc.y * u_fov_tan));

    /* Ray-sphere discriminant (oc = cam − centre, ray_dir unit). */
    float b_   = dot(u_oc, ray_dir);
    float c_   = dot(u_oc, u_oc) - u_radius * u_radius;
    float disc = b_ * b_ - c_;

    /* Silhouette-relative radius. perp^2 = radius^2 - disc, so
     * r^2 = perp^2 / radius^2 = 1 - disc/radius^2. */
    float r = sqrt(max(1.0 - disc / (u_radius * u_radius), 0.0));
    if (r >= BILL_SCALE) discard;
    if (r < 1.0)         discard;   /* interior is the opaque event horizon */

    /* Depth: forward distance to the centre (centre_rel = −u_oc), same log
     * metric as phong.frag so neighbouring geometry occludes consistently. */
    const float FAR = 2000.0;
    float eye_depth = max(dot(-u_oc, u_cam_fwd), 1e-4);
    gl_FragDepth = log2(eye_depth + 1.0) / log2(FAR + 1.0);

    /* Bright thin photon ring just outside the horizon edge. */
    float ring = exp(-pow((r - 1.18) / 0.16, 2.0));

    /* Soft outward glow that melts into black. */
    float glow = 0.30 * exp(-(r - 1.0) * 0.9);
    float outer_fade = 1.0 - smoothstep(BILL_SCALE - 8.0, BILL_SCALE, r);

    float total = (ring * 1.8 + glow) * outer_fade;
    if (total < 0.002) discard;

    vec3 hot = vec3(1.4, 1.15, 0.9);
    vec3 col = mix(hot, u_color, smoothstep(1.0, 3.0, r));

    frag_color = vec4(col * total, 1.0);
}
