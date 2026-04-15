/*
 * body.c — Keplerian orbital mechanics and solar-system initialisation
 *
 * Orbital elements from JPL "Keplerian Elements for Approximate Positions
 * of the Major Planets", Table 1 (J2000.0 epoch, valid ~1800–2050 AD).
 * https://ssd.jpl.nasa.gov/txt/p_elem_t1.txt
 *
 * GL coordinate frame (ecliptic → GL):
 *   GL X  =  ecliptic X   (toward vernal equinox)
 *   GL Y  =  ecliptic Z   (north pole → "up")
 *   GL Z  =  ecliptic Y   (90° east → "depth")
 */
#include "body.h"
#include <math.h>

Body g_bodies[MAX_BODIES];
int  g_nbodies = 0;

/* ------------------------------------------------------------------ Kepler */

/* Newton-Raphson solver for Kepler's equation: M = E - e*sin(E) */
static double solve_kepler(double M, double e) {
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
 * Convert JPL Keplerian elements (J2000.0) to heliocentric Cartesian state
 * in the simulation's GL frame.
 *
 * Parameters (angles in degrees, a in AU):
 *   a            semi-major axis
 *   e            eccentricity
 *   i_deg        inclination
 *   Omega_deg    longitude of ascending node
 *   omega_tilde  longitude of perihelion  (= Omega + omega)
 *   L_deg        mean longitude at epoch  (= Omega + omega + M)
 *
 * Output: pos_m in metres, vel_ms in m/s
 */
static void keplerian_to_state(
        double a, double e, double i_deg,
        double Omega_deg, double omega_tilde_deg, double L_deg,
        double pos_m[3], double vel_ms[3])
{
    const double deg = PI / 180.0;
    double i           = i_deg           * deg;
    double Omega       = Omega_deg       * deg;
    double omega_tilde = omega_tilde_deg * deg;
    double L           = L_deg           * deg;
    double omega       = omega_tilde - Omega;   /* argument of perihelion */

    /* Mean anomaly, normalised to (-π, π] */
    double M = fmod(L - omega_tilde, 2.0 * PI);
    if (M >  PI) M -= 2.0 * PI;
    if (M < -PI) M += 2.0 * PI;

    double E  = solve_kepler(M, e);
    double nu = 2.0 * atan2(sqrt(1.0 + e) * sin(E * 0.5),
                             sqrt(1.0 - e) * cos(E * 0.5));
    double r  = a * (1.0 - e * cos(E));

    /* Specific angular momentum (AU²/day) and perifocal state */
    double h    = sqrt(GM_SUN * a * (1.0 - e * e));
    double x_p  = r * cos(nu);
    double y_p  = r * sin(nu);
    double vx_p = -(GM_SUN / h) * sin(nu);
    double vy_p =  (GM_SUN / h) * (e + cos(nu));

    /* Rotation matrices: perifocal (PQW) → ecliptic */
    double cO=cos(Omega), sO=sin(Omega);
    double co=cos(omega), so=sin(omega);
    double ci=cos(i),     si=sin(i);

    double Px= cO*co - sO*so*ci,  Qx= -cO*so - sO*co*ci;
    double Py= sO*co + cO*so*ci,  Qy= -sO*so + cO*co*ci;
    double Pz= so*si,              Qz=  co*si;

    /* Ecliptic Cartesian (AU, AU/day) */
    double ex = Px*x_p + Qx*y_p,  evx = Px*vx_p + Qx*vy_p;
    double ey = Py*x_p + Qy*y_p,  evy = Py*vx_p + Qy*vy_p;
    double ez = Pz*x_p + Qz*y_p,  evz = Pz*vx_p + Qz*vy_p;

    /* Map ecliptic → GL and convert to SI */
    double au2m   = AU;
    double aud2ms = AU / DAY;

    pos_m[0] = ex * au2m;     vel_ms[0] = evx * aud2ms;
    pos_m[1] = ez * au2m;     vel_ms[1] = evz * aud2ms;  /* ecl Z → GL Y */
    pos_m[2] = ey * au2m;     vel_ms[2] = evy * aud2ms;  /* ecl Y → GL Z */
}

/* ------------------------------------------------------------------ helpers */
static void add_body_si(const char *name, double mass, double radius_km,
                        const double p[3], const double v[3],
                        float r, float g, float b, int is_star) {
    Body *bo = &g_bodies[g_nbodies++];
    strncpy(bo->name, name, 31);
    bo->mass   = mass;
    bo->radius = radius_km * 1000.0;
    bo->pos[0]=p[0]; bo->pos[1]=p[1]; bo->pos[2]=p[2];
    bo->vel[0]=v[0]; bo->vel[1]=v[1]; bo->vel[2]=v[2];
    bo->acc[0]=bo->acc[1]=bo->acc[2]=0.0;
    bo->col[0]=r; bo->col[1]=g; bo->col[2]=b;
    bo->is_star     = is_star;
    bo->trail_head  = 0;
    bo->trail_count = 0;
    memset(bo->trail, 0, sizeof(bo->trail));
}

/* ------------------------------------------------------------------ public */
void solar_system_init(void) {
    double p[3], v[3];
    int i;
    g_nbodies = 0;

    /* Sun — at origin, velocity corrected below for CoM frame */
    {
        double pz[3]={0,0,0}, vz[3]={0,0,0};
        add_body_si("Sun", 1.989e30, 696000.0, pz, vz,
                    1.0f, 0.92f, 0.23f, 1);
    }

    /* Mercury */
    keplerian_to_state(0.38709927, 0.20563593,  7.00497902,
                       48.33076593, 77.45779628, 252.25032350, p, v);
    add_body_si("Mercury", 3.301e23, 2439.7, p, v, 0.75f,0.72f,0.68f, 0);

    /* Venus */
    keplerian_to_state(0.72333566, 0.00677672,  3.39467605,
                       76.67984255, 131.60246718, 181.97909950, p, v);
    add_body_si("Venus",   4.867e24, 6051.8, p, v, 0.95f,0.78f,0.35f, 0);

    /* Earth (uses EM-Barycenter elements — close enough for single-body) */
    keplerian_to_state(1.00000261, 0.01671123, -0.00001531,
                       0.0, 102.93768193, 100.46457166, p, v);
    add_body_si("Earth",   5.972e24, 6371.0, p, v, 0.27f,0.55f,0.95f, 0);

    /* Mars */
    keplerian_to_state(1.52371034, 0.09339410,  1.84969142,
                       49.55953891, -23.94362959, -4.55343205, p, v);
    add_body_si("Mars",    6.417e23, 3389.5, p, v, 0.90f,0.32f,0.14f, 0);

    /* Jupiter */
    keplerian_to_state(5.20288700, 0.04838624,  1.30439695,
                       100.47390909, 14.72847983, 34.39644051, p, v);
    add_body_si("Jupiter", 1.898e27, 71492.0, p, v, 0.82f,0.65f,0.45f, 0);

    /* Saturn */
    keplerian_to_state(9.53667594, 0.05386179,  2.48599187,
                       113.66242448, 92.59887831, 49.95424423, p, v);
    add_body_si("Saturn",  5.683e26, 60268.0, p, v, 1.00f,0.90f,0.60f, 0);

    /* Uranus */
    keplerian_to_state(19.18916464, 0.04725744,  0.77263783,
                       74.01692503, 170.95427630, 313.23810451, p, v);
    add_body_si("Uranus",  8.681e25, 25559.0, p, v, 0.55f,0.82f,0.96f, 0);

    /* Neptune */
    keplerian_to_state(30.06992276, 0.00859048,  1.77004347,
                       131.78422574, 44.96476227, -55.12002969, p, v);
    add_body_si("Neptune", 1.024e26, 24764.0, p, v, 0.25f,0.38f,0.95f, 0);

    /* Centre-of-mass correction: give the Sun the negative total momentum */
    for (i = 1; i < g_nbodies; i++) {
        g_bodies[0].vel[0] -= g_bodies[i].mass*g_bodies[i].vel[0]/g_bodies[0].mass;
        g_bodies[0].vel[1] -= g_bodies[i].mass*g_bodies[i].vel[1]/g_bodies[0].mass;
        g_bodies[0].vel[2] -= g_bodies[i].mass*g_bodies[i].vel[2]/g_bodies[0].mass;
    }
}
