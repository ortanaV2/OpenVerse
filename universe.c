/*
 * universe.c -- 3D Universe / Solar-System Simulator
 *
 * Build (Linux):   gcc -O2 -o universe universe.c -lSDL2 -lGL -lGLU -lm
 * Build (Windows): gcc -O2 -o universe universe.c -lSDL2 -lSDL2main -lopengl32 -lglu32 -lm
 *
 * Controls:
 *   Mouse           - look around
 *   W/S / Up/Dn     - move forward / back
 *   A/D / Lt/Rt     - strafe left / right
 *   Q / E           - move up / down
 *   Scroll wheel    - change movement speed
 *   +/-             - simulation speed x2 / /2
 *   Space           - pause / resume
 *   T               - toggle orbital trails
 *   R               - reset camera to default
 *   ESC (grabbed)   - release mouse
 *   ESC (free)      - quit
 */

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef GL_CLAMP_TO_EDGE
#  define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

/* ------------------------------------------------------------------ constants */
#define MAX_BODIES   64
#define TRAIL_LEN  2048          /* circular buffer length per body            */

#define G          6.674e-11     /* m^3 kg^-1 s^-2                             */
#define PI         3.14159265358979323846
#define AU         1.496e11      /* meters per astronomical unit               */
#define DAY        86400.0       /* seconds per day                            */
#define SOFTENING  1e9           /* collision softening radius (m)             */
/* GM_sun in AU^3/day^2  (= 6.674e-11 * 1.989e30 / AU^3 * DAY^2) */
#define GM_SUN     2.9591220828559093e-4

#define WIN_W      1280
#define WIN_H       720
#define FOV         60.0f

/* 1 AU  -->  1.0 GL unit */
#define RS  (1.0 / AU)

/* ------------------------------------------------------------------ types */
typedef struct {
    char   name[32];
    double mass;           /* kg                         */
    double radius;         /* m                          */
    double pos[3];         /* m, heliocentric            */
    double vel[3];         /* m/s                        */
    double acc[3];         /* m/s^2 (recomputed each step) */
    float  col[3];         /* RGB display colour         */
    int    is_star;

    /* trail */
    int    thead;                      /* write head index   */
    int    tcount;                     /* filled entries     */
    float  trail[TRAIL_LEN][3];        /* GL units (AU)      */
} Body;

typedef struct {
    float pos[3];
    float yaw;             /* degrees, horizontal angle  */
    float pitch;           /* degrees, vertical angle    */
    float speed;           /* GL units / second          */
} Camera;

/* ------------------------------------------------------------------ globals */
static Body    g_bodies[MAX_BODIES];
static int     g_nbodies = 0;

static Camera  g_cam;
static double  g_sim_speed   = DAY;    /* simulated seconds per real second   */
static int     g_paused      = 0;
static int     g_running     = 1;
static int     g_grabbed     = 1;
static int     g_show_trails = 1;
static double  g_sim_time    = 0.0;    /* total simulated seconds             */

/* how many sim-steps between trail samples */
#define TRAIL_STEP_INTERVAL 10
static int     g_trail_counter = 0;

/* ------------------------------------------------------------------ helpers */
static void cam_dir(const Camera *c, float *dx, float *dy, float *dz)
{
    float cy = cosf(c->yaw   * (float)PI / 180.0f);
    float sy = sinf(c->yaw   * (float)PI / 180.0f);
    float cp = cosf(c->pitch * (float)PI / 180.0f);
    float sp = sinf(c->pitch * (float)PI / 180.0f);
    *dx = cy * cp;
    *dy = sp;
    *dz = sy * cp;
}

static void cam_reset(void)
{
    /* Ecliptic plane is now GL-XZ.  Camera sits above (Y) and slightly
     * to the "south" (positive Z) for a natural top-down perspective. */
    g_cam.pos[0] =  0.0f;
    g_cam.pos[1] = 20.0f;   /* 20 AU above ecliptic plane */
    g_cam.pos[2] = 25.0f;   /* 25 AU "south"              */
    g_cam.yaw    = 180.0f;
    g_cam.pitch  = -38.0f;
    g_cam.speed  =  0.5f;
}

/* ------------------------------------------------------------------ solar system */
static void add_body(
    const char *name,
    double mass,    double radius_km,
    double ax,  double ay,  double az,   /* AU   */
    double vx,  double vy,  double vz,   /* km/s */
    float r, float g, float b,
    int is_star)
{
    Body *bo = &g_bodies[g_nbodies++];
    strncpy(bo->name, name, 31);
    bo->mass     = mass;
    bo->radius   = radius_km * 1000.0;
    bo->pos[0]   = ax * AU;
    bo->pos[1]   = ay * AU;
    bo->pos[2]   = az * AU;
    bo->vel[0]   = vx * 1000.0;
    bo->vel[1]   = vy * 1000.0;
    bo->vel[2]   = vz * 1000.0;
    bo->acc[0]   = bo->acc[1] = bo->acc[2] = 0.0;
    bo->col[0]   = r;
    bo->col[1]   = g;
    bo->col[2]   = b;
    bo->is_star  = is_star;
    bo->thead    = 0;
    bo->tcount   = 0;
    memset(bo->trail, 0, sizeof(bo->trail));
}

/* ----------------------------------------------------------------
 * Keplerian orbital mechanics
 * ---------------------------------------------------------------- */

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

/* Convert Keplerian elements (J2000.0 epoch, JPL convention) to
 * heliocentric Cartesian state in the simulation's GL frame.
 *
 * GL coordinate mapping from ecliptic:
 *   GL X  =  ecl X   (toward vernal equinox)
 *   GL Y  =  ecl Z   (ecliptic north pole  → "up" in GL)
 *   GL Z  =  ecl Y   (90° east in ecliptic → "depth" in GL)
 *
 * Parameters (all angles in degrees, a in AU):
 *   a            semi-major axis
 *   e            eccentricity
 *   i            inclination
 *   Omega        longitude of ascending node
 *   omega_tilde  longitude of perihelion (= Omega + omega)
 *   L            mean longitude at epoch
 *
 * Output: pos_m in metres, vel_ms in m/s (both 3-element arrays)
 */
static void keplerian_to_state(
    double a, double e, double i_deg,
    double Omega_deg, double omega_tilde_deg, double L_deg,
    double pos_m[3], double vel_ms[3])
{
    double deg = PI / 180.0;
    double i     = i_deg           * deg;
    double Omega = Omega_deg       * deg;
    double omega_tilde = omega_tilde_deg * deg;
    double L     = L_deg           * deg;
    double omega = omega_tilde - Omega;   /* argument of perihelion */

    /* Mean anomaly, normalised to (-PI, PI] */
    double M = fmod(L - omega_tilde, 2.0*PI);
    if (M >  PI) M -= 2.0*PI;
    if (M < -PI) M += 2.0*PI;

    double E  = solve_kepler(M, e);
    double nu = 2.0 * atan2(sqrt(1.0+e)*sin(E*0.5),
                             sqrt(1.0-e)*cos(E*0.5));
    double r  = a * (1.0 - e*cos(E));

    /* Position and velocity in the perifocal (PQW) frame */
    double h    = sqrt(GM_SUN * a * (1.0 - e*e));   /* spec. ang. momentum AU²/day */
    double x_p  = r * cos(nu);
    double y_p  = r * sin(nu);
    double vx_p = -(GM_SUN/h) * sin(nu);
    double vy_p =  (GM_SUN/h) * (e + cos(nu));

    /* Rotation matrix (PQW → ecliptic):  P-hat and Q-hat columns */
    double cO = cos(Omega), sO = sin(Omega);
    double co = cos(omega), so = sin(omega);
    double ci = cos(i),     si = sin(i);

    double Px =  cO*co - sO*so*ci,   Qx = -cO*so - sO*co*ci;
    double Py =  sO*co + cO*so*ci,   Qy = -sO*so + cO*co*ci;
    double Pz =  so*si,              Qz =  co*si;

    /* Ecliptic Cartesian (AU and AU/day) */
    double ex = Px*x_p + Qx*y_p,   evx = Px*vx_p + Qx*vy_p;
    double ey = Py*x_p + Qy*y_p,   evy = Py*vx_p + Qy*vy_p;
    double ez = Pz*x_p + Qz*y_p,   evz = Pz*vx_p + Qz*vy_p;

    /* Map ecliptic → GL and convert units */
    double au2m  = AU;
    double aud2ms = AU / DAY;  /* AU/day → m/s */

    pos_m[0] = ex * au2m;    vel_ms[0] = evx * aud2ms;
    pos_m[1] = ez * au2m;    vel_ms[1] = evz * aud2ms;  /* ecl Z → GL Y */
    pos_m[2] = ey * au2m;    vel_ms[2] = evy * aud2ms;  /* ecl Y → GL Z */
}

/* Add a body with position (m) and velocity (m/s) given directly */
static void add_body_si(const char *name, double mass, double radius_km,
                        const double p[3], const double v[3],
                        float r, float gc, float b, int is_star)
{
    Body *bo = &g_bodies[g_nbodies++];
    strncpy(bo->name, name, 31);
    bo->mass     = mass;
    bo->radius   = radius_km * 1000.0;
    bo->pos[0]   = p[0]; bo->pos[1] = p[1]; bo->pos[2] = p[2];
    bo->vel[0]   = v[0]; bo->vel[1] = v[1]; bo->vel[2] = v[2];
    bo->acc[0]   = bo->acc[1] = bo->acc[2] = 0.0;
    bo->col[0]   = r;  bo->col[1] = gc; bo->col[2] = b;
    bo->is_star  = is_star;
    bo->thead    = 0;  bo->tcount = 0;
    memset(bo->trail, 0, sizeof(bo->trail));
}

/* ----------------------------------------------------------------
 * Solar system initialisation — real J2000.0 Keplerian elements
 * Source: JPL "Keplerian Elements for Approximate Positions of the
 *         Major Planets", Table 1 (valid ~1800–2050 AD)
 *         https://ssd.jpl.nasa.gov/txt/p_elem_t1.txt
 *
 * Columns: a(AU)  e  i(°)  Ω(°)  ω̃(°)  L(°)
 * ---------------------------------------------------------------- */
static void init_solar_system(void)
{
    double p[3], v[3];
    int i;

    /* Sun — positioned at origin; velocity corrected below */
    add_body("Sun", 1.989e30, 696000,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             1.0f, 0.92f, 0.23f, 1);

    /* Mercury */
    keplerian_to_state(0.38709927, 0.20563593,  7.00497902,
                       48.33076593, 77.45779628, 252.25032350, p, v);
    add_body_si("Mercury", 3.301e23,  2439.7, p, v, 0.75f,0.72f,0.68f, 0);

    /* Venus */
    keplerian_to_state(0.72333566, 0.00677672,  3.39467605,
                       76.67984255, 131.60246718, 181.97909950, p, v);
    add_body_si("Venus",   4.867e24,  6051.8, p, v, 0.95f,0.78f,0.35f, 0);

    /* Earth */
    keplerian_to_state(1.00000261, 0.01671123, -0.00001531,
                       0.0, 102.93768193, 100.46457166, p, v);
    add_body_si("Earth",   5.972e24,  6371.0, p, v, 0.27f,0.55f,0.95f, 0);

    /* Mars */
    keplerian_to_state(1.52371034, 0.09339410,  1.84969142,
                       49.55953891, -23.94362959, -4.55343205, p, v);
    add_body_si("Mars",    6.417e23,  3389.5, p, v, 0.90f,0.32f,0.14f, 0);

    /* Jupiter */
    keplerian_to_state(5.20288700, 0.04838624,  1.30439695,
                       100.47390909, 14.72847983, 34.39644051, p, v);
    add_body_si("Jupiter", 1.898e27, 71492.0, p, v, 0.82f,0.65f,0.45f, 0);

    /* Saturn */
    keplerian_to_state(9.53667594, 0.05386179,  2.48599187,
                       113.66242448, 92.59887831, 49.95424423, p, v);
    add_body_si("Saturn",  5.683e26, 60268.0, p, v, 1.00f,0.90f,0.60f, 0);

    /* Uranus */
    keplerian_to_state(19.18916464, 0.04725744, 0.77263783,
                       74.01692503, 170.95427630, 313.23810451, p, v);
    add_body_si("Uranus",  8.681e25, 25559.0, p, v, 0.55f,0.82f,0.96f, 0);

    /* Neptune */
    keplerian_to_state(30.06992276, 0.00859048, 1.77004347,
                       131.78422574, 44.96476227, -55.12002969, p, v);
    add_body_si("Neptune", 1.024e26, 24764.0, p, v, 0.25f,0.38f,0.95f, 0);

    /* Put everything in the centre-of-mass frame:
     * Sun gets the negative total momentum so Σ(m*v) = 0 */
    for (i = 1; i < g_nbodies; i++) {
        g_bodies[0].vel[0] -= g_bodies[i].mass * g_bodies[i].vel[0] / g_bodies[0].mass;
        g_bodies[0].vel[1] -= g_bodies[i].mass * g_bodies[i].vel[1] / g_bodies[0].mass;
        g_bodies[0].vel[2] -= g_bodies[i].mass * g_bodies[i].vel[2] / g_bodies[0].mass;
    }
}

/* ------------------------------------------------------------------ physics */
static void compute_acc(void)
{
    int i, j;
    for (i = 0; i < g_nbodies; i++)
        g_bodies[i].acc[0] = g_bodies[i].acc[1] = g_bodies[i].acc[2] = 0.0;

    for (i = 0; i < g_nbodies; i++) {
        for (j = i + 1; j < g_nbodies; j++) {
            double dx = g_bodies[j].pos[0] - g_bodies[i].pos[0];
            double dy = g_bodies[j].pos[1] - g_bodies[i].pos[1];
            double dz = g_bodies[j].pos[2] - g_bodies[i].pos[2];
            double r2 = dx*dx + dy*dy + dz*dz + SOFTENING*SOFTENING;
            double r  = sqrt(r2);
            double f  = G / (r2 * r);       /* G / r^3 */

            double ai = f * g_bodies[j].mass;
            double aj = f * g_bodies[i].mass;

            g_bodies[i].acc[0] += ai * dx;
            g_bodies[i].acc[1] += ai * dy;
            g_bodies[i].acc[2] += ai * dz;
            g_bodies[j].acc[0] -= aj * dx;
            g_bodies[j].acc[1] -= aj * dy;
            g_bodies[j].acc[2] -= aj * dz;
        }
    }
}

/* Leapfrog (KDK) integrator — excellent energy conservation */
static void physics_step(double dt)
{
    int i;
    compute_acc();

    /* half-kick velocities */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    /* drift positions */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].pos[0] += g_bodies[i].vel[0] * dt;
        g_bodies[i].pos[1] += g_bodies[i].vel[1] * dt;
        g_bodies[i].pos[2] += g_bodies[i].vel[2] * dt;
    }
    /* re-compute accelerations at new positions */
    compute_acc();
    /* half-kick velocities again */
    for (i = 0; i < g_nbodies; i++) {
        g_bodies[i].vel[0] += 0.5 * g_bodies[i].acc[0] * dt;
        g_bodies[i].vel[1] += 0.5 * g_bodies[i].acc[1] * dt;
        g_bodies[i].vel[2] += 0.5 * g_bodies[i].acc[2] * dt;
    }
    g_sim_time += dt;
}

static void sample_trail(Body *b)
{
    b->trail[b->thead][0] = (float)(b->pos[0] * RS);
    b->trail[b->thead][1] = (float)(b->pos[1] * RS);
    b->trail[b->thead][2] = (float)(b->pos[2] * RS);
    b->thead  = (b->thead + 1) % TRAIL_LEN;
    if (b->tcount < TRAIL_LEN) b->tcount++;
}

/* ------------------------------------------------------------------ rendering */

/* Draw a UV-sphere centred at (cx,cy,cz) */
static void draw_sphere(float cx, float cy, float cz, float radius,
                        int slices, int stacks)
{
    int i, j;
    for (i = 0; i < stacks; i++) {
        float phi0 = (float)PI * (-0.5f + (float) i      / stacks);
        float phi1 = (float)PI * (-0.5f + (float)(i + 1) / stacks);
        float cp0 = cosf(phi0), sp0 = sinf(phi0);
        float cp1 = cosf(phi1), sp1 = sinf(phi1);

        glBegin(GL_TRIANGLE_STRIP);
        for (j = 0; j <= slices; j++) {
            float theta = 2.0f * (float)PI * (float)j / slices;
            float ct = cosf(theta), st = sinf(theta);

            glNormal3f(ct * cp1, sp1, st * cp1);
            glVertex3f(cx + radius * ct * cp1,
                       cy + radius * sp1,
                       cz + radius * st * cp1);

            glNormal3f(ct * cp0, sp0, st * cp0);
            glVertex3f(cx + radius * ct * cp0,
                       cy + radius * sp0,
                       cz + radius * st * cp0);
        }
        glEnd();
    }
}

static void draw_trail(const Body *b)
{
    int k, count;
    if (b->tcount < 2) return;

    count = b->tcount;
    glBegin(GL_LINE_STRIP);
    for (k = 0; k < count; k++) {
        /* oldest sample first */
        int idx = (b->thead - count + k + TRAIL_LEN) % TRAIL_LEN;
        float alpha = (float)k / (float)(count - 1);
        alpha *= 0.7f;
        glColor4f(b->col[0], b->col[1], b->col[2], alpha);
        glVertex3fv(b->trail[idx]);
    }
    glEnd();
}

/* Static star-field — rendered as a pure skybox (camera-rotation only).
 * Stars have slight colour tint (blue giants, yellow dwarfs, red giants)
 * and two size classes so the field looks natural.                       */
#define NUM_STARS 4000
static void draw_starfield(void)
{
    static int   inited = 0;
    /* direction unit vectors + rgb colour */
    static float sx[NUM_STARS], sy[NUM_STARS], sz[NUM_STARS];
    static float sr[NUM_STARS], sg[NUM_STARS], sb_col[NUM_STARS];
    static int   big[NUM_STARS];   /* 1 = 2.5px bright star, 0 = 1.5px dim */
    int i;

    if (!inited) {
        /* Approximate stellar colour distribution:
         *  ~70% white/yellow (G/K dwarfs), ~20% blue-white (B/A),
         *  ~10% orange/red (K/M giants)                           */
        static const float tints[5][3] = {
            {1.00f, 1.00f, 1.00f},   /* pure white    */
            {0.95f, 0.97f, 1.00f},   /* blue-white    */
            {1.00f, 0.97f, 0.80f},   /* yellow        */
            {1.00f, 0.85f, 0.60f},   /* orange        */
            {1.00f, 0.65f, 0.50f},   /* red giant     */
        };
        static const int tint_weight[5] = {35, 25, 20, 12, 8}; /* roughly ~100 total */

        srand(12345u);
        for (i = 0; i < NUM_STARS; i++) {
            float theta = ((float)rand() / RAND_MAX) * 2.0f * (float)PI;
            float phi   = acosf(2.0f * ((float)rand() / RAND_MAX) - 1.0f);
            sx[i] = sinf(phi) * cosf(theta);
            sy[i] = cosf(phi);
            sz[i] = sinf(phi) * sinf(theta);

            /* pick colour tint by weighted random */
            float brightness = 0.35f + 0.65f * ((float)rand() / RAND_MAX);
            int   pick = rand() % 100;
            int   tidx = 0, accum = 0, t;
            for (t = 0; t < 5; t++) {
                accum += tint_weight[t];
                if (pick < accum) { tidx = t; break; }
            }
            sr[i]     = tints[tidx][0] * brightness;
            sg[i]     = tints[tidx][1] * brightness;
            sb_col[i] = tints[tidx][2] * brightness;
            big[i]    = (rand() % 100 < 8);   /* ~8% are "bright" stars */
        }
        inited = 1;
    }

    /* Draw dim stars */
    glPointSize(1.5f);
    glBegin(GL_POINTS);
    for (i = 0; i < NUM_STARS; i++) {
        if (big[i]) continue;
        glColor3f(sr[i], sg[i], sb_col[i]);
        glVertex3f(sx[i], sy[i], sz[i]);   /* unit sphere — scale doesn't matter
                                               since this is rotation-only space */
    }
    glEnd();

    /* Draw bright stars slightly larger */
    glPointSize(2.5f);
    glBegin(GL_POINTS);
    for (i = 0; i < NUM_STARS; i++) {
        if (!big[i]) continue;
        glColor3f(sr[i] * 1.3f > 1.0f ? 1.0f : sr[i] * 1.3f,
                  sg[i] * 1.3f > 1.0f ? 1.0f : sg[i] * 1.3f,
                  sb_col[i] * 1.3f > 1.0f ? 1.0f : sb_col[i] * 1.3f);
        glVertex3f(sx[i], sy[i], sz[i]);
    }
    glEnd();
    glPointSize(1.0f);
}


/* ================================================================
 * SDL2_ttf label system
 * Each body gets one pre-rendered texture (created at startup).
 * Labels render as camera-facing billboards with a fixed screen height.
 * ================================================================ */
static TTF_Font *g_font     = NULL;
static GLuint    g_label_tex[MAX_BODIES];
static int       g_label_w  [MAX_BODIES];
static int       g_label_h  [MAX_BODIES];

/* Try to open a font from a list of common paths */
static TTF_Font *open_any_font(int pt_size)
{
    static const char *paths[] = {
        /* Windows */
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibri.ttf",
        "C:/Windows/Fonts/verdana.ttf",
        "C:/Windows/Fonts/tahoma.ttf",
        /* Linux */
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        NULL
    };
    int i;
    for (i = 0; paths[i]; i++) {
        TTF_Font *f = TTF_OpenFont(paths[i], pt_size);
        if (f) { printf("Font: %s\n", paths[i]); return f; }
    }
    return NULL;
}

/* Upload one SDL surface as an OpenGL RGBA texture */
static GLuint surface_to_texture(SDL_Surface *surf)
{
    GLuint tex;
    SDL_Surface *conv;

    /* Convert to RGBA so OpenGL gets a predictable layout */
    conv = SDL_ConvertSurfaceFormat(surf, SDL_PIXELFORMAT_ABGR8888, 0);
    if (!conv) return 0;

    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 conv->w, conv->h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, conv->pixels);
    SDL_FreeSurface(conv);
    return tex;
}

static void init_labels(void)
{
    int i;
    memset(g_label_tex, 0, sizeof(g_label_tex));

    if (TTF_Init() < 0) {
        fprintf(stderr, "TTF_Init failed: %s\n", TTF_GetError());
        return;
    }

    g_font = open_any_font(15);
    if (!g_font) {
        fprintf(stderr, "No usable font found. Labels disabled.\n");
        return;
    }

    for (i = 0; i < g_nbodies; i++) {
        SDL_Surface *surf;
        /* Slightly brighten the body colour for readability on dark background */
        SDL_Color col;
        col.r = (Uint8)(fminf(g_bodies[i].col[0] * 1.4f + 0.15f, 1.0f) * 255);
        col.g = (Uint8)(fminf(g_bodies[i].col[1] * 1.4f + 0.15f, 1.0f) * 255);
        col.b = (Uint8)(fminf(g_bodies[i].col[2] * 1.4f + 0.15f, 1.0f) * 255);
        col.a = 255;

        surf = TTF_RenderText_Blended(g_font, g_bodies[i].name, col);
        if (!surf) continue;

        g_label_tex[i] = surface_to_texture(surf);
        g_label_w[i]   = surf->w;
        g_label_h[i]   = surf->h;
        SDL_FreeSurface(surf);
    }
}

/* Draw body label as a billboard that always faces the camera.
 * The label is anchored at (wx, wy, wz) and scales to a fixed screen height.
 * dcam = camera distance used to keep the label a constant pixel size.
 *
 * Camera right and up are extracted from the current modelview matrix so the
 * quad is always screen-parallel, regardless of camera pitch/roll. */
static void render_label(int idx, float wx, float wy, float wz, float dcam)
{
    float fw, fh, half_fov_tan;
    float rx, ry, rz;   /* camera right (world space) */
    float ux, uy, uz;   /* camera up    (world space) */
    GLdouble mv[16];
    const float LABEL_PX_H = 14.0f;

    if (!g_label_tex[idx]) return;

    /* World-space size that produces LABEL_PX_H pixels at this distance */
    half_fov_tan = tanf(FOV * 0.5f * (float)PI / 180.0f);
    fh = (dcam * 2.0f * LABEL_PX_H * half_fov_tan) / (float)WIN_H;
    fw = fh * (float)g_label_w[idx] / (float)g_label_h[idx];

    /* Extract camera right (row 0) and up (row 1) from the modelview matrix.
     * OpenGL stores matrices column-major, so row i = elements [i], [4+i],
     * [8+i] of the flat array. */
    glGetDoublev(GL_MODELVIEW_MATRIX, mv);
    rx = (float)mv[0]; ry = (float)mv[4]; rz = (float)mv[8];
    ux = (float)mv[1]; uy = (float)mv[5]; uz = (float)mv[9];

    /* Anchor: attach point offset slightly right so the label sits
     * just to the right of the body indicator. */
    float ax = wx + rx * (fw * 0.1f);
    float ay = wy + uy * (fh * 0.1f);
    float az = wz + rz * (fw * 0.1f);

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindTexture(GL_TEXTURE_2D, g_label_tex[idx]);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    /* SDL surface y=0 is top; GL texture t=0 is bottom-row of upload data,
     * which = top of the SDL surface.  So: quad-bottom → t=1, quad-top → t=0. */
    glBegin(GL_QUADS);
    /* bottom-left  */ glTexCoord2f(0.0f, 1.0f);
                       glVertex3f(ax,                   ay,                   az);
    /* bottom-right */ glTexCoord2f(1.0f, 1.0f);
                       glVertex3f(ax+fw*rx,              ay+fw*ry,             az+fw*rz);
    /* top-right    */ glTexCoord2f(1.0f, 0.0f);
                       glVertex3f(ax+fw*rx+fh*ux,        ay+fw*ry+fh*uy,       az+fw*rz+fh*uz);
    /* top-left     */ glTexCoord2f(0.0f, 0.0f);
                       glVertex3f(ax+fh*ux,              ay+fh*uy,             az+fh*uz);
    glEnd();

    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

static void render(void)
{
    int i;
    float dx, dy, dz;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* ---- Projection ---- */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(FOV, (float)WIN_W / WIN_H, 1e-4f, 20000.0f);

    /* ---- ModelView / Camera ---- */
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    cam_dir(&g_cam, &dx, &dy, &dz);
    gluLookAt((double)g_cam.pos[0], (double)g_cam.pos[1], (double)g_cam.pos[2],
              (double)(g_cam.pos[0] + dx),
              (double)(g_cam.pos[1] + dy),
              (double)(g_cam.pos[2] + dz),
              0.0, 1.0, 0.0);

    /* ---- Starfield: skybox-style (rotation only, no translation)
     *  We push a rotation-only view matrix so stars appear infinitely
     *  far away and never jitter or parallax as the camera moves.      ---- */
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glPushMatrix();
    {
        /* Replace current (translation+rotation) matrix with rotation-only */
        glLoadIdentity();
        gluLookAt(0.0, 0.0, 0.0,
                  (double)dx, (double)dy, (double)dz,
                  0.0, 1.0, 0.0);
        draw_starfield();
    }
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    /* ---- Orbital trails ---- */
    if (g_show_trails) {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (i = 0; i < g_nbodies; i++) {
            if (!g_bodies[i].is_star)
                draw_trail(&g_bodies[i]);
        }
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    /* ---- Set up a single point light at the Sun's position ---- */
    {
        Body *sun = &g_bodies[0];   /* Sun is always index 0 */
        float lpos[4] = {
            (float)(sun->pos[0] * RS),
            (float)(sun->pos[1] * RS),
            (float)(sun->pos[2] * RS),
            1.0f
        };
        float white[4]  = {1.0f, 0.95f, 0.80f, 1.0f};
        float dimamb[4] = {0.02f, 0.02f, 0.02f, 1.0f};
        glEnable(GL_LIGHT0);
        glLightfv(GL_LIGHT0, GL_POSITION, lpos);
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  white);
        glLightfv(GL_LIGHT0, GL_SPECULAR, white);
        glLightfv(GL_LIGHT0, GL_AMBIENT,  dimamb);
    }
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    /* ---- Bodies: pass 1 – draw geometry, cache per-body values ---- */
    {
        float bfx[MAX_BODIES], bfy[MAX_BODIES], bfz[MAX_BODIES];
        float bdr[MAX_BODIES], bdcam[MAX_BODIES];
        int   bshow[MAX_BODIES];
        float half_fov_tan = tanf(FOV * 0.5f * (float)PI / 180.0f);

        for (i = 0; i < g_nbodies; i++) {
            Body  *b   = &g_bodies[i];
            float  fx  = (float)(b->pos[0] * RS);
            float  fy  = (float)(b->pos[1] * RS);
            float  fz  = (float)(b->pos[2] * RS);
            float  nat_dr = (float)(b->radius * RS); /* physical radius in AU */
            float  dcam, dr, px_r;
            int    slices, stacks;

            /* Distance from camera — must be computed before dr */
            dcam = sqrtf(
                (fx - g_cam.pos[0]) * (fx - g_cam.pos[0]) +
                (fy - g_cam.pos[1]) * (fy - g_cam.pos[1]) +
                (fz - g_cam.pos[2]) * (fz - g_cam.pos[2]));

            /* Visual radius:
             *
             * Stars render at their true physical size.  From Earth the Sun
             * subtends ~0.53° — accurate.  A depth-tested dot handles the
             * case where it is too small to see as a disc.
             *
             * Planets use a distance-adaptive boost: just enough to keep the
             * body at MIN_VIS_PX apparent pixel radius, capped at MAX_BOOST.
             * When the camera is close the boost naturally falls toward 1×
             * (physical size), so proportions look correct up close.
             * No hard minimum-radius clamp is applied — the dot system
             * handles sub-pixel bodies. */
            if (b->is_star) {
                dr = nat_dr;
            } else {
                const float MIN_VIS_PX = 3.0f;
                const float MAX_BOOST  = 100.0f;
                float px_at_1x = (WIN_H / 2.0f) * nat_dr
                                 / (dcam * half_fov_tan + 1e-9f);
                float boost = MIN_VIS_PX / px_at_1x;
                if (boost < 1.0f) boost = 1.0f;
                if (boost > MAX_BOOST) boost = MAX_BOOST;
                dr = nat_dr * boost;
            }

            px_r = (WIN_H / 2.0f) * dr / (dcam * half_fov_tan + 1e-9f);

            bfx[i] = fx;  bfy[i] = fy;  bfz[i] = fz;
            bdr[i] = dr;  bdcam[i] = dcam;
            bshow[i] = (px_r < 8.0f);

            if (b->is_star) {
                float emit[4] = {b->col[0], b->col[1], b->col[2], 1.0f};
                glMaterialfv(GL_FRONT, GL_EMISSION, emit);
            } else {
                float zero[4] = {0.0f, 0.0f, 0.0f, 1.0f};
                glMaterialfv(GL_FRONT, GL_EMISSION, zero);
            }
            glColor3f(b->col[0], b->col[1], b->col[2]);
            slices = b->is_star ? 32 : 16;
            stacks = b->is_star ? 16 :  8;
            draw_sphere(fx, fy, fz, dr, slices, stacks);
        }

        glDisable(GL_LIGHTING);

        /* ---- Bodies: pass 2 – dots and labels with overlap avoidance ----
         *
         * Algorithm:
         *  1. Project each label anchor to screen space via gluProject.
         *  2. Sort indices by camera distance (closest = highest priority).
         *  3. Greedy scan: hide a label if its screen rect overlaps any
         *     already-accepted label rect.
         *  4. Draw dots always; labels only when not suppressed.
         *
         * Labels also disappear beyond MAX_LABEL_DIST regardless of overlap.
         */
#define MAX_LABEL_DIST 55.0f   /* GL units (= AU) beyond which labels vanish */
#define LBL_PAD        6.0f    /* extra px padding around each label rect     */
        {
            GLdouble mvmat[16], prmat[16];
            GLint    vp[4];
            float    lsx[MAX_BODIES], lsy[MAX_BODIES];
            float    lsw[MAX_BODIES], lsh[MAX_BODIES];
            int      lvis[MAX_BODIES];
            int      order[MAX_BODIES];
            int      j;

            glGetDoublev(GL_MODELVIEW_MATRIX,  mvmat);
            glGetDoublev(GL_PROJECTION_MATRIX, prmat);
            glGetIntegerv(GL_VIEWPORT, vp);

            /* Project label anchors → screen rects.
             * Body 0 (Sun) is always labelled; others obey show_dot + dist. */
            for (i = 0; i < g_nbodies; i++) {
                GLdouble sx, sy, sz;
                float ph, pw;
                int is_sun = (i == 0);
                order[i] = i;
                lvis[i]  = 0;

                if (!is_sun && !bshow[i])                continue;
                if (!is_sun && bdcam[i] > MAX_LABEL_DIST) continue;
                if (!g_label_tex[i])                      continue;

                if (gluProject(bfx[i], bfy[i] + bdr[i] * 1.4f, bfz[i],
                               mvmat, prmat, vp,
                               &sx, &sy, &sz) != GL_TRUE) continue;
                if (sz <= 0.0 || sz >= 1.0) continue;

                ph = 14.0f;
                pw = ph * (float)g_label_w[i] / (float)g_label_h[i];
                lsx[i] = (float)sx;
                lsy[i] = (float)(WIN_H - sy) - ph;
                lsw[i] = pw + LBL_PAD;
                lsh[i] = ph + LBL_PAD;
                lvis[i] = 1;
            }

            /* Insertion sort by distance – closest body wins overlaps.
             * Sun (index 0) is pinned to the front so it is processed
             * first and can never be displaced by another label.        */
            order[0] = 0;
            for (i = 1; i < g_nbodies; i++) order[i] = i;
            for (i = 2; i < g_nbodies; i++) {   /* start at 2 – skip Sun at 0 */
                int tmp = order[i], k = i;
                while (k > 1 && bdcam[order[k-1]] > bdcam[tmp]) {
                    order[k] = order[k-1];
                    k--;
                }
                order[k] = tmp;
            }

            /* Greedy overlap removal */
            for (i = 0; i < g_nbodies; i++) {
                int idx = order[i];
                if (!lvis[idx]) continue;
                for (j = 0; j < i; j++) {
                    int jdx = order[j];
                    if (!lvis[jdx]) continue;
                    if (lsx[idx]          < lsx[jdx] + lsw[jdx] &&
                        lsx[idx] + lsw[idx] > lsx[jdx]           &&
                        lsy[idx]          < lsy[jdx] + lsh[jdx] &&
                        lsy[idx] + lsh[idx] > lsy[jdx]) {
                        lvis[idx] = 0;
                        break;
                    }
                }
            }

            /* Pass A – dots.
             * Each dot is drawn at the body's geometric centre with the
             * depth test active.  The sphere rendered in pass 1 already
             * wrote depth values for its front face; those values are
             * closer to the camera than the centre point, so the depth
             * test automatically hides the dot whenever the sphere is
             * large enough to be seen.  No explicit pixel-radius threshold
             * is needed – the GPU handles it for free. */
            glEnable(GL_DEPTH_TEST);
            glPointSize(3.0f);
            glBegin(GL_POINTS);
            for (i = 0; i < g_nbodies; i++) {
                Body *b = &g_bodies[i];
                glColor3f(b->col[0], b->col[1], b->col[2]);
                glVertex3f(bfx[i], bfy[i], bfz[i]);
            }
            glEnd();
            glPointSize(1.0f);

            /* Pass B – labels.
             * Sun (index 0) is always labelled.  Other bodies only when
             * their label was projected and survived overlap removal. */
            for (i = 0; i < g_nbodies; i++) {
                int is_sun = (i == 0);
                if (lvis[i] || is_sun)
                    render_label(i, bfx[i], bfy[i] + bdr[i] * 1.4f, bfz[i],
                                 bdcam[i]);
            }
        }
    }
}

/* ---- HUD text (printed to stdout; a real bitmap-font pass can be added) ---- */
static void print_hud(void)
{
    double days = g_sim_time / DAY;
    double years = days / 365.25;

    /* Update title bar every ~60 frames */
    static int frame = 0;
    if ((frame++ % 60) == 0) {
        /* printf is a cheap stand-in; later replace with on-screen text */
        printf("\r  Sim time: %.1f days (%.2f yr)  |  Speed: %.0f d/s  |  "
               "Cam: (%.2f, %.2f, %.2f)  |  [Space]=pause [T]=trails [+/-]=speed",
               days, years, g_sim_speed / DAY,
               g_cam.pos[0], g_cam.pos[1], g_cam.pos[2]);
        fflush(stdout);
    }
}

/* ================================================================ main */
int main(int argc, char *argv[])
{
    SDL_Window   *win;
    SDL_GLContext  ctx;
    SDL_Event      ev;
    Uint32         t_prev, t_now;
    float          real_dt;
    int            sub_steps = 10;

    (void)argc; (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,  24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    win = SDL_CreateWindow(
        "Universe Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        SDL_Quit(); return 1;
    }

    ctx = SDL_GL_CreateContext(win);
    if (!ctx) {
        fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError());
        SDL_DestroyWindow(win); SDL_Quit(); return 1;
    }

    SDL_GL_SetSwapInterval(1);   /* vsync */

    /* OpenGL baseline */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.015f, 1.0f);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

    /* Specular highlights */
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    cam_reset();
    init_solar_system();
    init_labels();        /* must be called after GL context is created */

    SDL_SetRelativeMouseMode(SDL_TRUE);
    g_grabbed = 1;

    printf("=== Universe Simulator ===\n");
    printf("Controls: W/A/S/D/Q/E=move  Mouse=look  Scroll=speed\n");
    printf("          +/-=sim speed  Space=pause  T=trails  R=reset cam  ESC=quit\n\n");

    t_prev = SDL_GetTicks();

    while (g_running) {
        t_now   = SDL_GetTicks();
        real_dt = (float)(t_now - t_prev) / 1000.0f;
        t_prev  = t_now;
        if (real_dt > 0.1f) real_dt = 0.1f;

        /* ---- Events ---- */
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {

            case SDL_QUIT:
                g_running = 0;
                break;

            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE:
                    if (g_grabbed) {
                        SDL_SetRelativeMouseMode(SDL_FALSE);
                        g_grabbed = 0;
                    } else {
                        g_running = 0;
                    }
                    break;
                case SDLK_SPACE:
                    g_paused = !g_paused;
                    printf("\n%s\n", g_paused ? "[PAUSED]" : "[RUNNING]");
                    break;
                case SDLK_t:
                    g_show_trails = !g_show_trails;
                    break;
                case SDLK_r:
                    cam_reset();
                    break;
                case SDLK_EQUALS: case SDLK_KP_PLUS:  case SDLK_PLUS:
                    g_sim_speed *= 2.0;
                    printf("\nSim speed: %.1f days/sec\n", g_sim_speed / DAY);
                    break;
                case SDLK_MINUS:  case SDLK_KP_MINUS:
                    g_sim_speed /= 2.0;
                    if (g_sim_speed < 1.0) g_sim_speed = 1.0;
                    printf("\nSim speed: %.2f days/sec\n", g_sim_speed / DAY);
                    break;
                }
                break;

            case SDL_MOUSEBUTTONDOWN:
                if (!g_grabbed) {
                    SDL_SetRelativeMouseMode(SDL_TRUE);
                    g_grabbed = 1;
                }
                break;

            case SDL_MOUSEMOTION:
                if (g_grabbed) {
                    g_cam.yaw   += ev.motion.xrel * 0.18f;
                    g_cam.pitch -= ev.motion.yrel * 0.18f;
                    if (g_cam.pitch >  89.0f) g_cam.pitch =  89.0f;
                    if (g_cam.pitch < -89.0f) g_cam.pitch = -89.0f;
                }
                break;

            case SDL_MOUSEWHEEL:
                g_cam.speed *= (ev.wheel.y > 0) ? 1.25f : 0.80f;
                if (g_cam.speed < 0.001f) g_cam.speed = 0.001f;
                if (g_cam.speed > 200.0f) g_cam.speed = 200.0f;
                break;
            }
        }

        /* ---- Camera movement ---- */
        {
            const Uint8 *k = SDL_GetKeyboardState(NULL);
            float fdx, fdy, fdz;
            float right_x, right_z;
            float spd = g_cam.speed * real_dt;

            cam_dir(&g_cam, &fdx, &fdy, &fdz);
            right_x = -fdz;
            right_z =  fdx;

            if (k[SDL_SCANCODE_W] || k[SDL_SCANCODE_UP]) {
                g_cam.pos[0] += fdx * spd;
                g_cam.pos[1] += fdy * spd;
                g_cam.pos[2] += fdz * spd;
            }
            if (k[SDL_SCANCODE_S] || k[SDL_SCANCODE_DOWN]) {
                g_cam.pos[0] -= fdx * spd;
                g_cam.pos[1] -= fdy * spd;
                g_cam.pos[2] -= fdz * spd;
            }
            if (k[SDL_SCANCODE_A] || k[SDL_SCANCODE_LEFT]) {
                g_cam.pos[0] -= right_x * spd;
                g_cam.pos[2] -= right_z * spd;
            }
            if (k[SDL_SCANCODE_D] || k[SDL_SCANCODE_RIGHT]) {
                g_cam.pos[0] += right_x * spd;
                g_cam.pos[2] += right_z * spd;
            }
            if (k[SDL_SCANCODE_Q]) g_cam.pos[1] += spd;
            if (k[SDL_SCANCODE_E]) g_cam.pos[1] -= spd;
        }

        /* ---- Physics ---- */
        if (!g_paused) {
            double sim_dt_total = g_sim_speed * real_dt;
            double sub_dt = sim_dt_total / sub_steps;
            int s;
            for (s = 0; s < sub_steps; s++) {
                physics_step(sub_dt);
                g_trail_counter++;
                if (g_trail_counter >= TRAIL_STEP_INTERVAL) {
                    int bi;
                    g_trail_counter = 0;
                    for (bi = 0; bi < g_nbodies; bi++)
                        sample_trail(&g_bodies[bi]);
                }
            }
        }

        /* ---- Render ---- */
        render();
        SDL_GL_SwapWindow(win);
        print_hud();
    }

    printf("\nBye.\n");
    if (g_font) TTF_CloseFont(g_font);
    TTF_Quit();
    SDL_GL_DeleteContext(ctx);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
