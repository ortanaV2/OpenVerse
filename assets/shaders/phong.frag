#version 330 core
/*
 * phong.frag — ray-sphere intersection, world-space Phong shading
 *
 * v_uv arrives in [-BILL_SCALE, +BILL_SCALE] (see phong.vert).
 * Reconstructing frag_bill = center + right*v_uv.x*radius + up*v_uv.y*radius
 * gives the exact world-space position of this fragment on the billboard,
 * consistent with the vertex expansion — so the ray direction is correct
 * even for off-axis spheres.
 *
 * The surface normal is derived from the true sphere intersection, not the
 * billboard UV, so shadows are fixed in world space and never rotate with
 * the camera.
 */

in vec2 v_uv;

uniform vec3  u_color;
uniform float u_emission;
uniform float u_ambient;
uniform vec3  u_sun_pos_world;
uniform vec3  u_center;
uniform float u_radius;
uniform vec3  u_cam_pos;
uniform vec3  u_cam_right;
uniform vec3  u_cam_up;

out vec4 frag_color;

const float BILL_SCALE = 2.0;

void main() {
    /* ---- emissive (sun / stars) ---------------------------------------- */
    if (u_emission > 0.5) {
        /* v_uv is in [-BILL_SCALE, +BILL_SCALE]; normalise to unit circle  */
        float r2 = dot(v_uv, v_uv) / (BILL_SCALE * BILL_SCALE);
        if (r2 > 1.0) discard;
        float limb = 1.0 - 0.25 * r2;
        frag_color = vec4(u_color * limb, 1.0);
        return;
    }

    /* ---- reconstruct world-space billboard fragment position ------------ */
    /* Consistent with vertex shader: off * BILL_SCALE → v_uv               */
    vec3 frag_bill = u_center
                   + u_cam_right * (v_uv.x * u_radius)
                   + u_cam_up    * (v_uv.y * u_radius);

    /* ---- ray from camera through this fragment -------------------------- */
    vec3 ray_dir = normalize(frag_bill - u_cam_pos);

    /* ---- ray-sphere intersection ----------------------------------------
     * |cam_pos + t*D - center|^2 = radius^2
     * oc = cam_pos - center
     * t^2 + 2*dot(oc,D)*t + dot(oc,oc) - R^2 = 0
     * -------------------------------------------------------------------- */
    vec3  oc   = u_cam_pos - u_center;
    float b_   = dot(oc, ray_dir);
    float c_   = dot(oc, oc) - u_radius * u_radius;
    float disc = b_ * b_ - c_;

    if (disc < 0.0) discard;

    float t = -b_ - sqrt(disc);         /* front face                       */
    if (t < 0.0) t = -b_ + sqrt(disc); /* camera inside sphere — back face */
    if (t < 0.0) discard;

    vec3 hit = u_cam_pos + t * ray_dir;

    /* ---- world-space normal (fixed, never rotates with the camera) ------ */
    vec3 N = normalize(hit - u_center);

    /* ---- Phong diffuse -------------------------------------------------- */
    vec3  L     = normalize(u_sun_pos_world - hit);
    float diff  = max(dot(N, L), 0.0);
    float light = u_ambient + (1.0 - u_ambient) * diff;

    frag_color = vec4(u_color * light, 1.0);
}
