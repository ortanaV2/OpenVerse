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

Body g_bodies[MAX_BODIES];
int  g_nbodies = 0;

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
static void moon_to_state(
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

static void add_body_si(const char *name, double mass, double radius_km,
                        const double p[3], const double v[3],
                        float r, float g, float b, int is_star)
{
    Body *bo = &g_bodies[g_nbodies++];
    strncpy(bo->name, name, 31);
    bo->mass   = mass;
    bo->radius = radius_km * 1000.0;
    bo->pos[0]=p[0]; bo->pos[1]=p[1]; bo->pos[2]=p[2];
    bo->vel[0]=v[0]; bo->vel[1]=v[1]; bo->vel[2]=v[2];
    bo->acc[0]=bo->acc[1]=bo->acc[2]=0.0;
    bo->col[0]=r; bo->col[1]=g; bo->col[2]=b;
    bo->is_star        = is_star;
    bo->parent         = -1;
    bo->obliquity      = 0.0;
    bo->rotation_rate  = 0.0;
    bo->rotation_angle = 0.0;
    bo->trail_interval = DAY;    /* overridden per-body after init */
    bo->trail_accum    = 0.0;
    bo->trail_head  = 0;
    bo->trail_count = 0;
    memset(bo->trail, 0, sizeof(bo->trail));
}

/*
 * add_moon — add a moon body orbiting g_bodies[parent_idx].
 * Elements: a_km, e, i_deg (ecliptic), Omega_deg, omega_deg, M0_deg, gm_parent.
 */
static void add_moon(const char *name, double mass, double radius_km,
                     int parent_idx,
                     double a_km, double e, double i_deg,
                     double Omega_deg, double omega_deg, double M0_deg,
                     double gm_parent,
                     float r, float g, float b)
{
    double rel_p[3], rel_v[3];
    moon_to_state(a_km, e, i_deg, Omega_deg, omega_deg, M0_deg,
                  gm_parent, rel_p, rel_v);

    Body *par = &g_bodies[parent_idx];
    double p[3] = { par->pos[0] + rel_p[0],
                    par->pos[1] + rel_p[1],
                    par->pos[2] + rel_p[2] };
    double v[3] = { par->vel[0] + rel_v[0],
                    par->vel[1] + rel_v[1],
                    par->vel[2] + rel_v[2] };

    add_body_si(name, mass, radius_km, p, v, r, g, b, 0);
    {
        Body  *bo = &g_bodies[g_nbodies - 1];
        double a_m = a_km * 1000.0;
        double T   = 2.0 * PI * sqrt(a_m * a_m * a_m / gm_parent); /* seconds */
        bo->trail_interval = T / 200.0;   /* 200 samples per orbit */
        if (bo->trail_interval < 60.0) bo->trail_interval = 60.0;
        bo->parent = parent_idx;
    }
}

static void set_rotation(int idx, double obliquity_deg, double period_days)
{
    g_bodies[idx].obliquity     = obliquity_deg;
    g_bodies[idx].rotation_rate = (2.0 * PI) / (period_days * DAY);
}

/* ------------------------------------------------------------------ public */
void solar_system_init(void)
{
    double p[3], v[3];
    int i;
    g_nbodies = 0;

    /* ---- Sun (index 0) ---- */
    {
        double pz[3]={0,0,0}, vz[3]={0,0,0};
        add_body_si("Sun", 1.989e30, 696000.0, pz, vz,
                    1.0f, 0.92f, 0.23f, 1);
    }

    /* ---- Planets (indices 1–8) ---- */

    /* Mercury */
    keplerian_to_state(0.38709927, 0.20563593,  7.00497902,
                       48.33076593, 77.45779628, 252.25032350, p, v);
    add_body_si("Mercury", 3.301e23, 2439.7, p, v, 0.75f,0.72f,0.68f, 0);

    /* Venus */
    keplerian_to_state(0.72333566, 0.00677672,  3.39467605,
                       76.67984255, 131.60246718, 181.97909950, p, v);
    add_body_si("Venus",   4.867e24, 6051.8, p, v, 0.95f,0.78f,0.35f, 0);

    /* Earth  (index 3) */
    keplerian_to_state(1.00000261, 0.01671123, -0.00001531,
                       0.0, 102.93768193, 100.46457166, p, v);
    add_body_si("Earth",   5.972e24, 6371.0, p, v, 0.27f,0.55f,0.95f, 0);

    /* Mars   (index 4) */
    keplerian_to_state(1.52371034, 0.09339410,  1.84969142,
                       49.55953891, -23.94362959, -4.55343205, p, v);
    add_body_si("Mars",    6.417e23, 3389.5, p, v, 0.90f,0.32f,0.14f, 0);

    /* Jupiter (index 5) */
    keplerian_to_state(5.20288700, 0.04838624,  1.30439695,
                       100.47390909, 14.72847983, 34.39644051, p, v);
    add_body_si("Jupiter", 1.898e27, 71492.0, p, v, 0.82f,0.65f,0.45f, 0);

    /* Saturn  (index 6) */
    keplerian_to_state(9.53667594, 0.05386179,  2.48599187,
                       113.66242448, 92.59887831, 49.95424423, p, v);
    add_body_si("Saturn",  5.683e26, 60268.0, p, v, 1.00f,0.90f,0.60f, 0);

    /* Uranus  (index 7) */
    keplerian_to_state(19.18916464, 0.04725744,  0.77263783,
                       74.01692503, 170.95427630, 313.23810451, p, v);
    add_body_si("Uranus",  8.681e25, 25559.0, p, v, 0.55f,0.82f,0.96f, 0);

    /* Neptune (index 8) */
    keplerian_to_state(30.06992276, 0.00859048,  1.77004347,
                       131.78422574, 44.96476227, -55.12002969, p, v);
    add_body_si("Neptune", 1.024e26, 24764.0, p, v, 0.25f,0.38f,0.95f, 0);

    /* Ceres  (index 9) */
    keplerian_to_state(2.76969657, 0.07600902, 10.59407162,
                       80.30992285, 73.11734054, 77.37209589, p, v);
    add_body_si("Ceres",     9.393e20,  476.2, p, v, 0.72f,0.68f,0.64f, 0);

    /* Pluto  (index 10) */
    keplerian_to_state(39.48211675, 0.24882730, 17.14001206,
                       110.30393684, 224.06891629, 238.92903833, p, v);
    add_body_si("Pluto",     1.303e22, 1188.3, p, v, 0.85f,0.78f,0.68f, 0);

    /* Eris   (index 11) */
    keplerian_to_state(67.6681, 0.44177, 44.0445,
                       35.9531, 151.4305, 204.1574, p, v);
    add_body_si("Eris",      1.660e22, 1163.0, p, v, 0.92f,0.92f,0.90f, 0);

    /* Makemake (index 12) */
    keplerian_to_state(45.4302, 0.15586, 28.9633,
                       79.3812, 296.0481, 41.8866, p, v);
    add_body_si("Makemake",  3.100e21,  715.0, p, v, 0.88f,0.72f,0.62f, 0);

    /* Haumea (index 13) */
    keplerian_to_state(43.1355, 0.19510, 28.2137,
                       122.1030, 240.1972, 202.6739, p, v);
    add_body_si("Haumea",    4.006e21,  620.0, p, v, 0.90f,0.88f,0.88f, 0);

    /* ---- Rotation ---- */
    set_rotation(0,   7.25,    25.38);    /* Sun     */
    set_rotation(1,   0.034,   58.646);   /* Mercury */
    set_rotation(2, 177.4,    243.025);   /* Venus   */
    set_rotation(3,  23.44,    0.99727);  /* Earth   */
    set_rotation(4,  25.19,    1.02596);  /* Mars    */
    set_rotation(5,   3.13,    0.41354);  /* Jupiter */
    set_rotation(6,  26.73,    0.44401);  /* Saturn  */
    set_rotation(7,  97.77,    0.71833);  /* Uranus  */
    set_rotation(8,  28.32,    0.67125);  /* Neptune */

    /* ---------------------------------------------------------------- Moons
     * GM values (m³/s²) = G * planet_mass
     * Earth:   3.9860e14   Mars:    4.2828e13
     * Jupiter: 1.2669e17   Saturn:  3.7931e16
     * Uranus:  5.7940e15   Neptune: 6.8351e15
     *
     * Inclinations are referenced to the ecliptic plane.
     * Inner moons: i ≈ planet_obliquity + equatorial_inclination.
     * M0 is spread across the system so moons start at different phases.
     * ---------------------------------------------------------------- */

    /* --- Earth (index 3) --- */
    /* Moon */
    add_moon("Moon",    7.342e22, 1737.4, 3,
             384400.0, 0.0549, 5.14, 125.1, 318.1,   0.0,
             3.9860e14, 0.85f, 0.85f, 0.85f);

    /* --- Mars (index 4) --- */
    /* Phobos */
    add_moon("Phobos",  1.066e16,   11.3, 4,
               9376.0, 0.0151, 1.08,  0.0,   0.0,   0.0,
             4.2828e13, 0.65f, 0.60f, 0.58f);
    /* Deimos */
    add_moon("Deimos",  1.476e15,    6.2, 4,
              23458.0, 0.0002, 1.79, 54.5,   0.0, 120.0,
             4.2828e13, 0.62f, 0.58f, 0.55f);

    /* --- Jupiter (index 5) — Galilean moons --- */
    /* Io */
    add_moon("Io",      8.932e22, 1821.6, 5,
             421800.0, 0.0041, 3.13,  0.0,  84.1,   0.0,
             1.2669e17, 0.95f, 0.85f, 0.30f);
    /* Europa */
    add_moon("Europa",  4.800e22, 1560.8, 5,
             671100.0, 0.0094, 3.60, 90.0,   0.0, 100.0,
             1.2669e17, 0.80f, 0.75f, 0.70f);
    /* Ganymede */
    add_moon("Ganymede",1.482e23, 2634.1, 5,
            1070400.0, 0.0013, 3.30, 45.0,   0.0, 200.0,
             1.2669e17, 0.75f, 0.72f, 0.65f);
    /* Callisto */
    add_moon("Callisto",1.076e23, 2410.3, 5,
            1882700.0, 0.0074, 3.30, 20.0,   0.0, 300.0,
             1.2669e17, 0.55f, 0.50f, 0.48f);

    /* --- Saturn (index 6) --- */
    /* Mimas */
    add_moon("Mimas",   3.750e19,  198.2, 6,
             185539.0, 0.0196, 28.3, 0.0,  0.0,   0.0,
             3.7931e16, 0.80f, 0.78f, 0.78f);
    /* Enceladus */
    add_moon("Enceladus",1.080e20,  252.1, 6,
             238020.0, 0.0047, 26.7, 60.0,  0.0,  60.0,
             3.7931e16, 0.95f, 0.95f, 0.95f);
    /* Tethys */
    add_moon("Tethys",  6.174e20,  531.1, 6,
             294619.0, 0.0001, 27.8, 120.0, 0.0, 120.0,
             3.7931e16, 0.82f, 0.80f, 0.78f);
    /* Dione */
    add_moon("Dione",   1.096e21,  561.4, 6,
             377396.0, 0.0022, 26.8, 180.0, 0.0, 180.0,
             3.7931e16, 0.80f, 0.78f, 0.75f);
    /* Rhea */
    add_moon("Rhea",    2.307e21,  763.8, 6,
             527108.0, 0.0013, 27.1, 240.0, 0.0, 240.0,
             3.7931e16, 0.78f, 0.75f, 0.72f);
    /* Titan */
    add_moon("Titan",   1.345e23, 2574.7, 6,
            1221870.0, 0.0288, 27.1, 300.0, 0.0, 300.0,
             3.7931e16, 0.85f, 0.70f, 0.45f);

    /* --- Uranus (index 7) — moons orbit in Uranus's equatorial plane
     *  i ≈ 97.77° (Uranus obliquity) for near-equatorial moons --- */
    /* Miranda */
    add_moon("Miranda", 6.590e19,  235.8, 7,
             129390.0, 0.0013, 102.1,  0.0, 0.0,   0.0,
             5.7940e15, 0.70f, 0.68f, 0.65f);
    /* Ariel */
    add_moon("Ariel",   1.353e21,  578.9, 7,
             191020.0, 0.0012,  97.8, 90.0, 0.0,  72.0,
             5.7940e15, 0.72f, 0.70f, 0.68f);
    /* Umbriel */
    add_moon("Umbriel", 1.172e21,  584.7, 7,
             266300.0, 0.0039,  97.9, 180.0,0.0, 144.0,
             5.7940e15, 0.50f, 0.48f, 0.48f);
    /* Titania */
    add_moon("Titania", 3.527e21,  788.4, 7,
             435910.0, 0.0011,  97.8, 270.0,0.0, 216.0,
             5.7940e15, 0.68f, 0.65f, 0.62f);
    /* Oberon */
    add_moon("Oberon",  3.014e21,  761.4, 7,
             583520.0, 0.0014,  97.9,  45.0,0.0, 288.0,
             5.7940e15, 0.62f, 0.58f, 0.55f);

    /* --- Neptune (index 8) --- */
    /* Triton (retrograde: i > 90°) */
    add_moon("Triton",  2.139e22, 1353.4, 8,
             354759.0, 0.000016, 156.885, 177.0, 0.0, 0.0,
             6.8351e15, 0.78f, 0.75f, 0.82f);

    /* ---- Planet trail intervals: ~200 samples per orbit ----
     * T = 2π * sqrt(a³ / GM_SUN)  in days                    */
    {
        static const double a_AU[] =
            { 0, 0.387, 0.723, 1.000, 1.524, 5.203, 9.537, 19.19, 30.07,
              2.769, 39.48, 67.67, 45.43, 43.14 };
        for (i = 1; i <= 13; i++) {
            double T_days = 2.0 * PI *
                sqrt(a_AU[i]*a_AU[i]*a_AU[i] / GM_SUN);   /* days */
            g_bodies[i].trail_interval = (T_days / 200.0) * DAY;
        }
        g_bodies[0].trail_interval = DAY * 25.0;  /* Sun barely moves */
    }

    /* ---- Centre-of-mass correction ---- */
    for (i = 1; i < g_nbodies; i++) {
        g_bodies[0].vel[0] -= g_bodies[i].mass * g_bodies[i].vel[0] / g_bodies[0].mass;
        g_bodies[0].vel[1] -= g_bodies[i].mass * g_bodies[i].vel[1] / g_bodies[0].mass;
        g_bodies[0].vel[2] -= g_bodies[i].mass * g_bodies[i].vel[2] / g_bodies[0].mass;
    }
}
