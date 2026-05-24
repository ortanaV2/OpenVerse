/*
 * body.c — Keplerian orbital mechanics and helper functions
 *
 * Planets: JPL "Keplerian Elements for Approximate Positions of the Major
 *   Planets", Table 1 (J2000.0 epoch, valid ~1800–2050 AD).
 *   https://ssd.jpl.nasa.gov/txt/p_elem_t1.txt
 *
 * Moons: mean orbital elements from JPL/NASA fact sheets.
 *   Moon inclinations are referenced to the ecliptic plane (approx. =
 *   planet obliquity + equatorial inclination for inner moons).
 *
 * GL coordinate frame (ecliptic → GL):
 *   GL X  =  ecliptic X   (toward vernal equinox)
 *   GL Y  =  ecliptic Z   (north ecliptic pole → "up")
 *   GL Z  =  ecliptic Y   (90° east → "depth")
 *
 * The Y↔Z swap is applied at the very end of each state conversion so all
 * intermediate math stays in the standard ecliptic frame used by JPL tables.
 */
#include "body.h"
#include "camera.h"
#include <math.h>

Body *g_bodies     = NULL;
int   g_nbodies    = 0;
int   g_bodies_cap = 0;

/* ---------------------------------------------------------------- Kepler */

/* Newton-Raphson solution of Kepler's equation  M = E − e·sin(E).
 * Starting from E = M converges in < 10 iterations for e < 0.9 (all planets).
 * Tolerance 1e-12 rad ≈ 1 mm at 1 AU — well below any other error source. */
static double solve_kepler(double M, double e)
{
    double E = M;
    int k;
    for (k = 0; k < 50; k++) {
        double dE = (M - E + e * sin(E)) / (1.0 - e * cos(E));
        E += dE;
        if (fabs(dE) < 1e-12) break;
    }
    return E;
}

/*
 * keplerian_to_state — convert JPL planet elements to Cartesian state.
 *
 * Input angles are in degrees; a in AU; gm_au_day2 in AU³/day².
 * Output pos_m is in metres and vel_ms in m/s, in the GL coordinate frame.
 *
 * Algorithm:
 *   1. Convert angles to radians; compute argument of periapsis ω = ω̃ − Ω.
 *   2. Compute mean anomaly M = L − ω̃, normalise to (−π, π].
 *   3. Solve Kepler's equation for eccentric anomaly E.
 *   4. Convert E → true anomaly ν and radius r.
 *   5. Express position/velocity in the perifocal frame (x_p, y_p, vx_p, vy_p).
 *   6. Rotate perifocal → ecliptic via the (P, Q) direction-cosine matrix built
 *      from Ω, ω, i.  P points toward periapsis, Q is 90° ahead in the orbit.
 *   7. Apply the ecliptic → GL frame swap (y↔z) and scale to SI units.
 */
void keplerian_to_state(
        double a, double e, double i_deg,
        double Omega_deg, double omega_tilde_deg, double L_deg,
        double gm_au_day2,
        double pos_m[3], double vel_ms[3])
{
    const double deg = PI / 180.0;
    double i           = i_deg           * deg;
    double Omega       = Omega_deg       * deg;
    double omega_tilde = omega_tilde_deg * deg;
    double L           = L_deg           * deg;
    double omega       = omega_tilde - Omega;   /* argument of periapsis */

    double M = fmod(L - omega_tilde, 2.0 * PI);
    if (M >  PI) M -= 2.0 * PI;
    if (M < -PI) M += 2.0 * PI;

    double E  = solve_kepler(M, e);
    double nu = 2.0 * atan2(sqrt(1.0 + e) * sin(E * 0.5),
                             sqrt(1.0 - e) * cos(E * 0.5));
    double r  = a * (1.0 - e * cos(E));

    /* Specific angular momentum h = sqrt(GM·a·(1−e²)) in AU²/day */
    double h    = sqrt(gm_au_day2 * a * (1.0 - e * e));
    double x_p  = r * cos(nu);
    double y_p  = r * sin(nu);
    double vx_p = -(gm_au_day2 / h) * sin(nu);
    double vy_p =  (gm_au_day2 / h) * (e + cos(nu));

    /* Direction cosines for the perifocal → ecliptic rotation:
     *   P = (cos Ω cos ω − sin Ω sin ω cos i,
     *        sin Ω cos ω + cos Ω sin ω cos i,
     *        sin ω sin i)
     *   Q = (−cos Ω sin ω − sin Ω cos ω cos i, ...)
     * This is the standard Bate–Mueller–White formulation. */
    double cO=cos(Omega), sO=sin(Omega);
    double co=cos(omega), so=sin(omega);
    double ci=cos(i),     si=sin(i);

    double Px= cO*co - sO*so*ci,  Qx= -cO*so - sO*co*ci;
    double Py= sO*co + cO*so*ci,  Qy= -sO*so + cO*co*ci;
    double Pz= so*si,              Qz=  co*si;

    /* Ecliptic state vectors */
    double ex = Px*x_p + Qx*y_p,  evx = Px*vx_p + Qx*vy_p;
    double ey = Py*x_p + Qy*y_p,  evy = Py*vx_p + Qy*vy_p;
    double ez = Pz*x_p + Qz*y_p,  evz = Pz*vx_p + Qz*vy_p;

    double au2m   = AU;
    double aud2ms = AU / DAY;

    /* Ecliptic → GL: swap Y (ecliptic north→depth) and Z (ecliptic Y→up).
     * pos_m[0]=ex, pos_m[1]=ez (eclZ→GL Y "up"), pos_m[2]=ey (eclY→GL Z "depth") */
    pos_m[0] = ex * au2m;     vel_ms[0] = evx * aud2ms;
    pos_m[1] = ez * au2m;     vel_ms[1] = evz * aud2ms;
    pos_m[2] = ey * au2m;     vel_ms[2] = evy * aud2ms;
}

/*
 * moon_to_state — convert simple moon elements to parent-relative state.
 *
 * Differences from keplerian_to_state:
 *   - a is in km (not AU), GM is the parent body's GM in m³/s².
 *   - No longitude-based mean anomaly: M0 is given directly in degrees.
 *   - Output is parent-relative (not heliocentric); caller adds parent state.
 *   - Same ecliptic → GL frame swap applied at the end.
 *
 *   a_km    — semi-major axis (km)
 *   e       — eccentricity
 *   i_deg   — inclination to ecliptic (degrees)
 *   Omega_deg, omega_deg — longitude of ascending node and argument of periapsis
 *   M0_deg  — mean anomaly at epoch (degrees)
 *   gm      — GM of parent body (m³/s²)
 */
void moon_to_state(
        double a_km, double e, double i_deg,
        double Omega_deg, double omega_deg, double M0_deg,
        double gm,
        double pos_m[3], double vel_ms[3])
{
    const double deg = PI / 180.0;
    double a     = a_km * 1000.0;
    double i     = i_deg     * deg;
    double Omega = Omega_deg * deg;
    double omega = omega_deg * deg;
    double M     = M0_deg    * deg;

    M = fmod(M, 2.0 * PI);
    if (M < 0.0) M += 2.0 * PI;

    double E  = solve_kepler(M, e);
    double nu = 2.0 * atan2(sqrt(1.0 + e) * sin(E * 0.5),
                             sqrt(1.0 - e) * cos(E * 0.5));
    double r  = a * (1.0 - e * cos(E));

    double h    = sqrt(gm * a * (1.0 - e * e));
    double x_p  = r * cos(nu);
    double y_p  = r * sin(nu);
    double vx_p = -(gm / h) * sin(nu);
    double vy_p =  (gm / h) * (e + cos(nu));

    double cO=cos(Omega), sO=sin(Omega);
    double co=cos(omega), so=sin(omega);
    double ci=cos(i),     si_=sin(i);

    double Px= cO*co - sO*so*ci,  Qx= -cO*so - sO*co*ci;
    double Py= sO*co + cO*so*ci,  Qy= -sO*so + cO*co*ci;
    double Pz= so*si_,             Qz=  co*si_;

    double ex = Px*x_p + Qx*y_p,  evx = Px*vx_p + Qx*vy_p;
    double ey = Py*x_p + Qy*y_p,  evy = Py*vx_p + Qy*vy_p;
    double ez = Pz*x_p + Qz*y_p,  evz = Pz*vx_p + Qz*vy_p;

    /* Ecliptic → GL frame (Y↔Z swap) — same as keplerian_to_state */
    pos_m[0] = ex;   vel_ms[0] = evx;
    pos_m[1] = ez;   vel_ms[1] = evz;
    pos_m[2] = ey;   vel_ms[2] = evy;
}

/* ---------------------------------------------------------------- helpers */

/*
 * nearest_star_idx — index of the living star body closest to the camera.
 *
 * Used by trails_render to compute the system-level trail fade distance.
 * Camera position is in AU (render units); body positions are in metres,
 * so body pos is multiplied by RS (= 1/AU) to convert to AU before the
 * distance comparison.
 */
int nearest_star_idx(void)
{
    int best = 0;
    double best_d2 = 1e300;
    int i;
    for (i = 0; i < g_nbodies; i++) {
        if (!g_bodies[i].alive || !g_bodies[i].is_star) continue;
        double dx = g_cam.pos[0] - g_bodies[i].pos[0] * RS;
        double dy = g_cam.pos[1] - g_bodies[i].pos[1] * RS;
        double dz = g_cam.pos[2] - g_bodies[i].pos[2] * RS;
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
}

/* Walk the parent chain from body i until a body with parent == -1 is found
 * (that body is a star, by the parent-chain convention in universe.c).
 * Returns -1 if the chain is broken or i is invalid. */
int body_root_star(int i)
{
    if (i < 0 || i >= g_nbodies) return -1;
    while (g_bodies[i].parent >= 0) {
        i = g_bodies[i].parent;
        if (i < 0 || i >= g_nbodies) return -1;
    }
    return i;
}

/*
 * body_world_to_local_surface_dir — transform a world-space direction into the
 * body's surface (body-fixed) frame, accounting for axial tilt (obliquity) and
 * current rotation angle.
 *
 * This is used by the collision system to record crater positions in the
 * body's own frame so they remain fixed on the surface as the planet spins.
 *
 * The transform is:
 *   1. Apply obliquity rotation around X (tilt the pole away from ecliptic Y).
 *   2. Apply the inverse of the current rotation angle around the tilted pole
 *      (un-rotate back to the body's epoch orientation).
 *
 * The result is a unit vector pointing to the impact site on the unit sphere
 * in body-fixed coordinates.
 */
void body_world_to_local_surface_dir(int body_idx, const double world_dir[3],
                                     float out[3])
{
    Body *b;
    double n[3];
    double len, co, so, tx, ty, tz, cr, sr;
    float flen;

    if (!out || body_idx < 0 || body_idx >= g_nbodies) return;

    b = &g_bodies[body_idx];
    n[0] = world_dir[0];
    n[1] = world_dir[1];
    n[2] = world_dir[2];
    len = sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    if (len <= 1e-12) {
        out[0] = 1.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        return;
    }

    n[0] /= len;
    n[1] /= len;
    n[2] /= len;

    /* Step 1: rotate by −obliquity around the X axis (undo axial tilt) */
    co = cos(b->obliquity * PI / 180.0);
    so = sin(b->obliquity * PI / 180.0);
    tx =  co * n[0] - so * n[1];
    ty =  so * n[0] + co * n[1];
    tz =  n[2];

    /* Step 2: rotate by −rotation_angle around the (tilted) pole axis.
     * Negative angle: undo the current spin to reach the epoch frame. */
    cr = cos(-b->rotation_angle);
    sr = sin(-b->rotation_angle);
    out[0] = (float)(tx * cr - tz * sr);
    out[1] = (float)ty;
    out[2] = (float)(tx * sr + tz * cr);

    flen = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
    if (flen <= 1e-8f) {
        out[0] = 1.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        return;
    }
    out[0] /= flen;
    out[1] /= flen;
    out[2] /= flen;
}

void body_format_dist_au(double au, char *buf, size_t n) {
    if (au < 0.001)
        snprintf(buf, n, "%.0f km", au * AU / 1000.0);
    else if (au < 1.0)
        snprintf(buf, n, "%.4f AU", au);
    else if (au < 1000.0)
        snprintf(buf, n, "%.2f AU", au);
    else
        snprintf(buf, n, "%.3f ly", au / 63241.0);
}
