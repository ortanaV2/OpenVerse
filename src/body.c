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

    double h    = sqrt(GM_SUN * a * (1.0 - e * e));
    double x_p  = r * cos(nu);
    double y_p  = r * sin(nu);
    double vx_p = -(GM_SUN / h) * sin(nu);
    double vy_p =  (GM_SUN / h) * (e + cos(nu));

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


