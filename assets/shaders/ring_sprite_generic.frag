#version 330 core
/*
 * ring_sprite_generic.frag — simple procedural ring disc for far-LOD
 *
 * Generic version for faint/narrow rings (Uranus, Neptune).
 * Discards pixels outside the ring annulus; remaining pixels get a
 * smooth edge fade.  Zone boundaries and colour are passed as uniforms
 * instead of being hardcoded like the Saturn-specific shader.
 */

in vec3 v_pos;

uniform vec3  u_center;         /* planet world pos (AU)               */
uniform vec3  u_b1;             /* ring-plane basis 1                  */
uniform vec3  u_b2;             /* ring-plane basis 2                  */
uniform float u_r_inner_km;     /* inner edge (km from planet centre)  */
uniform float u_r_outer_km;     /* outer edge (km)                     */
uniform vec3  u_ring_color;     /* base ring colour                    */
uniform float u_alpha_max;      /* peak opacity                        */

out vec4 frag_color;

void main() {
    /* Project onto ring plane to get radial distance */
    vec3  rel  = v_pos - u_center;
    float pu   = dot(rel, u_b1);
    float pv   = dot(rel, u_b2);
    float r_km = sqrt(pu*pu + pv*pv) * 149600000.0;

    if (r_km < u_r_inner_km || r_km > u_r_outer_km) discard;

    /* Smooth fade at inner and outer edges (8% of total width each) */
    float width  = u_r_outer_km - u_r_inner_km;
    float fade_w = width * 0.08;
    float alpha  = smoothstep(u_r_inner_km,           u_r_inner_km + fade_w, r_km)
                 * smoothstep(u_r_outer_km,           u_r_outer_km - fade_w, r_km);

    frag_color = vec4(u_ring_color, u_alpha_max * alpha);
}
