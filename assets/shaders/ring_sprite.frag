#version 330 core
/*
 * ring_sprite.frag — procedural ring disc for far-LOD
 *
 * Discards pixels outside the ring annulus; colours remaining pixels
 * according to the four ring zones (C, B, Cassini gap, A).
 *
 * The ring-plane projection uses the same b1/b2 basis as ring.vert so the
 * sprite aligns exactly with the particle ring at the LOD crossover.
 */
in vec3 v_pos;

uniform vec3 u_center;   /* Saturn world pos (AU) */
uniform vec3 u_b1;
uniform vec3 u_b2;

out vec4 frag_color;

void main() {
    /* Project fragment onto ring plane to get radial distance from Saturn */
    vec3  rel  = v_pos - u_center;
    float pu   = dot(rel, u_b1);
    float pv   = dot(rel, u_b2);
    float r_km = sqrt(pu*pu + pv*pv) * 149600000.0;   /* AU → km */

    /* Ring zone boundaries (km from Saturn centre) */
    const float C_IN   =  74658.0;
    const float C_OUT  =  92000.0;
    const float B_OUT  = 117580.0;
    const float CASS   = 122170.0;
    const float A_OUT  = 136775.0;

    if (r_km < C_IN || r_km > A_OUT) discard;

    vec3  col;
    float alpha;

    if (r_km < C_OUT) {
        /* C ring — faint grey, fades toward inner edge */
        float t = (r_km - C_IN) / (C_OUT - C_IN);
        col   = vec3(0.60, 0.56, 0.50);
        alpha = 0.28 * t;
    } else if (r_km < B_OUT) {
        /* B ring — bright tan, peaks in the middle */
        float t = (r_km - C_OUT) / (B_OUT - C_OUT);
        float env = smoothstep(0.0, 0.12, t) * smoothstep(1.0, 0.88, t);
        col   = vec3(0.88, 0.82, 0.68);
        alpha = 0.55 + 0.30 * env;
    } else if (r_km < CASS) {
        /* Cassini division — nearly empty gap */
        col   = vec3(0.35, 0.33, 0.30);
        alpha = 0.04;
    } else {
        /* A ring — medium brightness, fades at outer edge */
        float t = 1.0 - (r_km - CASS) / (A_OUT - CASS);
        col   = vec3(0.78, 0.73, 0.60);
        alpha = 0.48 * t;
    }

    frag_color = vec4(col, alpha);
}
