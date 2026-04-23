/*
 * body.c — Keplerian orbital mechanics and solar-system initialisation
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
 *   GL Y  =  ecliptic Z   (north pole → "up")
 *   GL Z  =  ecliptic Y   (90° east → "depth")
 */
#include "body.h"
#include "camera.h"
#include <math.h>

Body *g_bodies     = NULL;
int   g_nbodies    = 0;
int   g_bodies_cap = 0;

/* ------------------------------------------------------------------ Kepler */

/* Newton-Raphson solver for Kepler's equation: M = E - e*sin(E) */
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
 * keplerian_to_state — heliocentric state from JPL planet elements.
 *
 * Angles in degrees, a in AU.  Uses GM_SUN (AU³/day²).
 * Output: pos_m (metres), vel_ms (m/s), ecliptic → GL frame.
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
    double omega       = omega_tilde - Omega;

    double M = fmod(L - omega_tilde, 2.0 * PI);
    if (M >  PI) M -= 2.0 * PI;
    if (M < -PI) M += 2.0 * PI;

    double E  = solve_kepler(M, e);
    double nu = 2.0 * atan2(sqrt(1.0 + e) * sin(E * 0.5),
                             sqrt(1.0 - e) * cos(E * 0.5));
    double r  = a * (1.0 - e * cos(E));

    double h    = sqrt(gm_au_day2 * a * (1.0 - e * e));
    double x_p  = r * cos(nu);
    double y_p  = r * sin(nu);
    double vx_p = -(gm_au_day2 / h) * sin(nu);
    double vy_p =  (gm_au_day2 / h) * (e + cos(nu));

    double cO=cos(Omega), sO=sin(Omega);
    double co=cos(omega), so=sin(omega);
    double ci=cos(i),     si=sin(i);

    double Px= cO*co - sO*so*ci,  Qx= -cO*so - sO*co*ci;
    double Py= sO*co + cO*so*ci,  Qy= -sO*so + cO*co*ci;
    double Pz= so*si,              Qz=  co*si;

    double ex = Px*x_p + Qx*y_p,  evx = Px*vx_p + Qx*vy_p;
    double ey = Py*x_p + Qy*y_p,  evy = Py*vx_p + Qy*vy_p;
    double ez = Pz*x_p + Qz*y_p,  evz = Pz*vx_p + Qz*vy_p;

    double au2m   = AU;
    double aud2ms = AU / DAY;

    pos_m[0] = ex * au2m;     vel_ms[0] = evx * aud2ms;
    pos_m[1] = ez * au2m;     vel_ms[1] = evz * aud2ms;
    pos_m[2] = ey * au2m;     vel_ms[2] = evy * aud2ms;
}

/*
 * moon_to_state — planetocentric state from simple moon elements.
 *
 *   a_km    — semi-major axis (km)
 *   e       — eccentricity
 *   i_deg   — inclination to ecliptic (degrees)
 *   Omega_deg, omega_deg — node and argument of periapsis (degrees)
 *   M0_deg  — mean anomaly at epoch (degrees)
 *   gm      — GM of parent body (m³/s²)
 *
 * Output: position (m) and velocity (m/s) relative to parent, GL frame.
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

    /* ecliptic → GL frame */
    pos_m[0] = ex;   vel_ms[0] = evx;
    pos_m[1] = ez;   vel_ms[1] = evz;
    pos_m[2] = ey;   vel_ms[2] = evy;
}

/* ------------------------------------------------------------------ helpers */

/*
 * nearest_star_idx — index of the star body closest to the camera.
 * Iterates only over is_star==1 bodies (currently 3).
 * Camera position is in AU; body positions are in metres → multiply by RS.
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

int body_root_star(int i)
{
    if (i < 0 || i >= g_nbodies) return -1;
    while (g_bodies[i].parent >= 0) {
        i = g_bodies[i].parent;
        if (i < 0 || i >= g_nbodies) return -1;
    }
    return i;
}

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

    co = cos(b->obliquity * PI / 180.0);
    so = sin(b->obliquity * PI / 180.0);
    tx =  co * n[0] - so * n[1];
    ty =  so * n[0] + co * n[1];
    tz =  n[2];

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
