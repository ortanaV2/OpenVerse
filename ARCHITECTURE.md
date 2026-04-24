# Contributing to OpenVerse — Codebase Reference

This document is the definitive architectural reference for the **OpenVerse** solar system simulator — a real-time N-body gravity simulator with OpenGL rendering, written in C99. It is intended for both human contributors and LLM assistants to quickly understand where everything lives and how the pieces fit together.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Repository Layout](#2-repository-layout)
3. [Build System](#3-build-system)
4. [Core Data Structures](#4-core-data-structures)
5. [Module Reference](#5-module-reference)
   - [main.c — Entry Point & Event Loop](#maincentry-point--event-loop)
   - [body.c / body.h — Body Data & Keplerian Mechanics](#bodycbodyh--body-data--keplerian-mechanics)
   - [physics.c / physics.h — N-Body RESPA Integrator](#physicscphysicsh--n-body-respa-integrator)
   - [universe.c / universe.h — JSON Universe Loader](#universecuniverseh--json-universe-loader)
   - [render.c / render.h — Scene Renderer](#rendercrenderh--scene-renderer)
   - [trails.c / trails.h — Orbital Trail Rendering](#trailsctrailsh--orbital-trail-rendering)
   - [labels.c / labels.h — Body Name Labels](#labelaclabelsh--body-name-labels)
   - [starfield.c / starfield.h — Background Stars](#starfieldcstarfieldh--background-stars)
   - [rings.c / rings.h — Planetary Ring System](#ringscringsh--planetary-ring-system)
   - [asteroids.c / asteroids.h — Asteroid Belts](#asteroidscasteroidsh--asteroid-belts)
   - [collision.c / collision.h — Impact & Merge System](#collisioncollisionh--impact--merge-system)
   - [build.c / build.h — Sandbox Body Placement](#buildcbuildh--sandbox-body-placement)
   - [ui.c / ui.h — 2D HUD Overlay](#uicuih--2d-hud-overlay)
   - [camera.c / camera.h — Free-Look Camera](#cameraccamerah--free-look-camera)
   - [json.c / json.h — JSON Parser](#jsoncjsonh--json-parser)
   - [gl_utils.c / gl_utils.h — OpenGL Helpers](#gl_utilscgl_utilsh--opengl-helpers)
   - [common.h — Shared Constants](#commonh--shared-constants)
   - [math3d.h — Header-Only 3D Math](#math3dh--header-only-3d-math)
6. [Shader Reference](#6-shader-reference)
7. [Universe Data Format](#7-universe-data-format)
8. [Physics Deep Dive](#8-physics-deep-dive)
9. [Rendering Pipeline](#9-rendering-pipeline)
10. [File Dependency Graph](#10-file-dependency-graph)
11. [Adding New Features — Where to Edit](#11-adding-new-features--where-to-edit)

---

## 1. Project Overview

OpenVerse is a physically accurate solar system simulator:

- **Language:** C99
- **Windowing & Input:** SDL2
- **Rendering:** OpenGL 3.3 Core Profile (GLSL 330)
- **Text Rendering:** SDL2_ttf
- **Physics:** 2R-RESPA hierarchical N-body integrator
- **Orbital Mechanics:** Keplerian elements (JPL J2000 tables)
- **Data Source:** `assets/universe.json` — single flat JSON file defining all bodies

**Coordinate System:**
- Simulation: SI units (metres, m/s)
- Rendering: AU (Astronomical Units), 1 AU = 1.496 × 10¹¹ m
- Conversion factor `RS = 1/AU` (metres → GL units) defined in `common.h`

**Scale range the renderer handles:** sub-kilometre (spacecraft, rings) to light-years (interstellar) via a logarithmic depth buffer.

---

## 2. Repository Layout

```
OpenVerse/
├── Makefile
├── src/
│   ├── main.c             Entry point, event loop, physics orchestration
│   ├── body.c / body.h    Body struct, Keplerian mechanics
│   ├── physics.c / .h     N-body RESPA gravity integrator
│   ├── universe.c / .h    JSON universe loader, runtime body creation
│   ├── render.c / .h      Scene renderer (delegates to sub-renderers)
│   ├── trails.c / .h      Orbital trail rendering
│   ├── labels.c / .h      Body name labels (SDL_ttf + OpenGL)
│   ├── starfield.c / .h   Procedural background starfield
│   ├── rings.c / .h       Keplerian ring particle system
│   ├── asteroids.c / .h   Gravity-integrated asteroid belts
│   ├── collision.c / .h   Solid-body impacts and merges
│   ├── build.c / .h       Runtime body placement (sandbox mode)
│   ├── ui.c / ui.h        2D HUD overlay
│   ├── camera.c / .h      Free-look camera state
│   ├── json.c / json.h    Minimal recursive-descent JSON parser
│   ├── gl_utils.c / .h    OpenGL shader/buffer utilities
│   ├── common.h           Shared constants and macros
│   └── math3d.h           Header-only 3D math (Mat4, Vec3)
└── assets/
    ├── universe.json          All bodies, rings, and asteroid belts
    └── shaders/
        ├── phong.vert/frag        Planet sphere (ray-sphere intersection)
        ├── atm.vert/frag          Atmospheric limb glow
        ├── solid.vert/frag        Orbital trail lines
        ├── color.vert/frag        Starfield GL_POINTS
        ├── ring.vert              Keplerian ring particles
        ├── ring_sprite.vert/frag  Saturn-specific sprite rings
        ├── ring_sprite_generic.frag  Generic sprite rings (Uranus/Neptune)
        ├── asteroid_particle.vert Asteroid belt GL_POINTS
        ├── impact_particle.vert/frag  Collision debris
        ├── label.vert/frag        Body name billboard quads
        ├── star_glare.vert/frag   Star halo glow
        ├── build_line.vert/frag   Build mode preview lines
        └── ui.vert/frag           2D screen-space HUD
```

---

## 3. Build System

**File:** `Makefile`

The Makefile auto-detects the platform (Linux / macOS / Windows MSYS2 MinGW64) and sets include paths and linker flags accordingly.

**Dependencies:**
- SDL2 (windowing, input, OpenGL context)
- SDL2_ttf (font rendering for labels and HUD)
- GLEW (OpenGL extension loading)
- OpenGL 3.3+
- OpenMP (parallelises multi-star-system warmup)

**Build on Windows (MSYS2 MinGW64 shell):**
```bash
cd OpenVerse
mingw32-make
```

**Build on Linux/macOS:**
```bash
cd OpenVerse
make
```

**Key compiler flags:** `-O2 -std=c99 -Wall -Wextra -fopenmp`

**Clean:**
```bash
make clean   # removes .o files and the executable
```

---

## 4. Core Data Structures

### `Body` — `src/body.h`

The fundamental simulation unit. Every star, planet, moon, and test particle is a `Body`.

```c
typedef struct {
    char   name[32];       // Display name ("Earth", "Luna", ...)
    double mass;           // kg
    double radius;         // metres (physical)

    // World-frame simulation state (SI units)
    double pos[3];         // metres, origin = solar system barycenter
    double vel[3];         // m/s
    double acc[3];         // m/s² — recomputed each physics step
    double fast_acc[3];    // dominant parent force, used by RESPA inner loop

    // Rendering
    float  col[3];         // RGB [0..1]
    int    is_star;        // 1 = star, 0 = planet/moon

    // Lifecycle
    int    alive;          // 0 = removed/absorbed; index is never reused
    int    parent;         // index into g_bodies; -1 for stars

    // Adaptive timestep hints (computed by physics_refresh_timestep_model)
    double dyn_period;     // estimated orbital period (seconds)
    double dyn_dt_outer;   // recommended slow-force timestep
    double dyn_dt_inner;   // recommended fast-force timestep
    int    dyn_bucket;     // 0 (slow) .. 3 (very fast)

    // Rotation
    double obliquity;      // axial tilt in degrees
    double rotation_rate;  // rad/s
    double rotation_angle; // current phase [0, 2π]

    // Atmosphere (zero values = no atmosphere)
    float  atm_color[3];
    float  atm_intensity;
    float  atm_scale;      // outer radius as multiple of body radius

    // Orbital trail (circular buffer, heap-allocated)
    double trail_interval; // seconds between trail samples
    double trail_accum;    // accumulator
    int    trail_head;     // next write index
    int    trail_count;    // valid samples [0, TRAIL_LEN]
    double (*trail)[3];    // TRAIL_LEN × 3 doubles (AU-scale positions)
} Body;
```

**Key invariants:**
- `parent = -1` for stars; `parent >= 0` for everything else
- `alive = 0` marks a dead body — its slot in `g_bodies[]` is never recycled
- Positions are in **metres**; the renderer divides by `AU` before upload

**Global state:**
```c
Body *g_bodies;      // dynamically allocated array (body.h)
int   g_nbodies;     // live count
int   g_bodies_cap;  // allocated capacity
```

---

### `Camera` — `src/camera.h`

```c
typedef struct {
    double pos[3];   // AU (double precision — needed at interstellar distances)
    float  yaw;      // degrees, horizontal
    float  pitch;    // degrees, vertical
    float  speed;    // AU/s movement speed
} Camera;

extern Camera g_cam;
extern int    g_warp;  // 1 if warp mode active
```

---

### `ParticleDisc` — `src/rings.h`

Represents one ring system. Particles are simulated Keplerianly: mean anomaly advances on the CPU each step; the GPU solves Kepler's equation per-vertex to get position.

Each particle is stored as 8 floats:
`[M, a, e, ω, h, r, g, b]`
— mean anomaly (rad), semi-major axis (AU), eccentricity, argument of periapsis (rad), vertical offset (AU), RGB color.

---

### `CollisionSpot` — `src/collision.h`

```c
typedef struct {
    float dir[3];          // body-local unit direction (impact location on surface)
    float angular_radius;  // radians (extent of visual effect)
    float heat;            // [0, 1] — cooled intensity (decreases over time)
    float progress;        // [0, 1] — event lifecycle
    int   kind;            // 1=crater, 2=major, 3=merge, 4=intersect
} CollisionSpot;
```

Up to 16 spots per body, passed as uniform arrays to `phong.frag`.

---

### `BuildPreset` — `src/build.h`

```c
typedef struct {
    const char *name;
    double mass;
    double radius;
    float  col[3];
    int    is_star;
    int    wants_planet_parent;
    int    wants_nonstar_parent;
    float  atm_color[3];
    float  atm_intensity;
    float  atm_scale;
} BuildPreset;
```

6 presets: Rocky, Gas Giant, Ice World, Moon, Dwarf Planet, Star.

---

### `JsonNode` — `src/json.h`

```c
typedef struct JsonNode {
    JsonType    type;         // NULL, BOOL, NUMBER, STRING, ARRAY, OBJECT
    char       *key;          // object member key (NULL for array elements)
    double      number;
    char       *string;
    int         boolean;
    JsonNode   *first_child;  // first child of OBJECT or ARRAY
    JsonNode   *next;         // next sibling
} JsonNode;
```

---

## 5. Module Reference

### `main.c` — Entry Point & Event Loop

**Location:** `src/main.c` (~495 lines)

The top-level orchestrator. Owns:
- SDL2 / OpenGL initialisation and teardown
- The main loop: event handling → physics → render → swap
- Per-frame camera movement
- Simulation speed control (`g_sim_speed`, `g_paused`)
- RESPA physics scheduling (outer/inner step counts per system)
- 2-year warmup pre-simulation before the loop starts

**Key sections:**
| Lines (approx.) | What happens |
|---|---|
| Startup | SDL/GL init, shader loading, `universe_load()`, `render_init()`, warmup |
| `handle_events()` | Keyboard (WASD, +/-, B, T, Space, Escape), mouse look |
| `camera_move()` | Apply camera velocity each frame |
| Physics block | `physics_refresh_timestep_model()`, RESPA loops per system, `collision_step()`, `asteroids_step()`, `rings_tick()` |
| Render block | Build matrices, call `render_frame()`, `ui_render()`, swap |

**Simulation speed table** (days/real-second):
```
0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0, 100.0, 365.0
```
Cycled with `+` / `-` keys.

**Warp mode** (T key): clamps camera speed to [200, 63241] AU/s (~0.003–1 ly/s).

---

### `body.c` / `body.h` — Body Data & Keplerian Mechanics

**Location:** `src/body.c` (~233 lines), `src/body.h` (~76 lines)

Owns the global body array and provides orbital mechanics utilities.

**Functions:**

| Function | Signature | Purpose |
|---|---|---|
| `keplerian_to_state` | `(a, e, i, Omega, omega_tilde, L, gm, pos[], vel[])` | Heliocentric Keplerian elements → Cartesian state. Uses JPL J2000 element convention (longitude of perihelion `ω̃ = Ω + ω`, mean longitude `L = ω̃ + M`). Rotates from ecliptic to GL frame (Y↔Z swap). |
| `moon_to_state` | `(a_km, e, i, Omega, omega, M0, gm, pos[], vel[])` | Planetocentric Keplerian elements → state (moons). Uses standard ω (argument of periapsis, not longitude). |
| `nearest_star_idx` | `(void) → int` | Returns index of the star closest to `g_cam.pos`. Used for lighting direction. |
| `body_root_star` | `(int i) → int` | Walks the parent chain upward, returns root star index. |
| `body_world_to_local_surface_dir` | `(int idx, double world_dir[3], float out[3])` | Converts a world-frame direction into body-local frame (accounting for obliquity and rotation angle). Used to place collision spots on the correct surface location. |

**Keplerian solver** (internal `solve_kepler`): Newton-Raphson, converges in 3–5 iterations for typical eccentricities, capped at 50 for robustness.

---

### `physics.c` / `physics.h` — N-Body RESPA Integrator

**Location:** `src/physics.c` (~400 lines), `src/physics.h` (~52 lines)

Implements a **2R-RESPA** (Reference System Propagator Algorithm) hierarchical integrator:

- **Slow forces** (outer, ~1 day timestep): planet-planet interactions + non-parent perturbations on moons
- **Fast forces** (inner, ~0.02 day timestep): dominant parent → satellite force

This gives ~32× efficiency over uniform short-timestep integration.

**Global state:**
```c
double g_sim_time;    // seconds elapsed since epoch
double g_sim_speed;   // seconds of sim per real second
int    g_paused;      // 1 = paused
```

**Public functions:**

| Function | Purpose |
|---|---|
| `physics_respa_begin(dt_outer)` | Slow half-kick: `v += 0.5 * a_slow * dt` for all bodies |
| `physics_respa_inner(dt_inner)` | Fast KDK step: compute parent force, half-kick, drift, half-kick |
| `physics_respa_end(dt_outer)` | Slow half-kick (second half) + rotation advance |
| `physics_respa_begin_system(root, dt)` | Per-star-system variant of begin |
| `physics_respa_inner_system(root, dt)` | Per-star-system inner step |
| `physics_respa_end_system(root, dt)` | Per-star-system end |
| `physics_refresh_timestep_model()` | Recompute `dyn_period`, `dyn_dt_outer`, `dyn_dt_inner` for all bodies |
| `physics_outer_dt_limit()` | Minimum outer dt across all systems |
| `physics_inner_dt_limit()` | Minimum inner dt across all systems |
| `physics_system_count()` | Number of independent star systems |
| `physics_system_root(idx)` | Root star index of system `idx` |
| `physics_system_outer_dt_limit(idx)` | Per-system outer dt limit |
| `physics_system_inner_dt_limit(idx)` | Per-system inner dt limit |
| `physics_advance_time(dt)` | Increment `g_sim_time` |
| `physics_step(dt)` | Legacy single-step KDK (for compatibility) |

**Internal helpers** (not exposed in header):
- `is_satellite(i)` — body has a non-star parent
- `has_fast_parent(i)` — body should use fast RESPA forces
- `timestep_anchor(i)` — find the dominant gravitational body for timestep estimation
- `estimate_period_about(i, anchor)` — estimate orbital period via Kepler's 3rd law
- `solve_kepler(M, e)` — Newton-Raphson Kepler solver

**Timestep model** (`physics_refresh_timestep_model`):
1. For each body: find dynamical anchor (explicit parent or nearest massive body)
2. Estimate period: `T = 2π√(r³/GM)`
3. `dt_outer = T / 24`, clamped to [0.05 day, 1 day]
4. `dt_inner = T / 96`, clamped to [60 s, 0.02 day]
5. Per-system limit = minimum across all bodies in that system

---

### `universe.c` / `universe.h` — JSON Universe Loader

**Location:** `src/universe.c` (~300 lines), `src/universe.h` (~44 lines)

Parses `assets/universe.json` and populates `g_bodies`. Also provides the runtime body creation interface used by build mode and collision merges.

**`BodyCreateSpec`** (universe.h): flat struct describing a body to create at runtime (name, mass, radius, pos, vel, col, is_star, parent, obliquity, rotation_rate, atmosphere params).

**Functions:**

| Function | Purpose |
|---|---|
| `universe_load(path)` | Parse JSON, create all bodies in 3 passes: 1) stars, 2) planets/dwarf planets, 3) moons. Applies centre-of-mass velocity correction per star system. |
| `universe_add_body(spec)` | Add a single body at runtime (build mode, collision merge). Returns new body index. Allocates trail buffer. |
| `universe_rebind_to_nearest_stars()` | Reassign orphaned planets to the nearest star (used after a star is added via build mode). |
| `universe_shutdown()` | Free all trail buffers and `g_bodies`. |

**Load order matters:** stars must exist before planets (parent lookup by name), planets before moons.

---

### `render.c` / `render.h` — Scene Renderer

**Location:** `src/render.c` (~600 lines), `src/render.h` (~24 lines)

The scene compositor. Owns sphere mesh geometry, all body-level GL programs, and delegates to sub-renderers (trails, labels, rings, asteroids, etc.).

**Functions:**

| Function | Purpose |
|---|---|
| `render_init()` | Compile all shaders (`phong`, `atm`, `solid`, `color`, `star_glare`, `impact_particle`, `label`, `build_line`). Generate sphere mesh VAO/VBO. |
| `render_frame(view, proj, view_rot, dt)` | Draw one complete frame (see [Rendering Pipeline](#9-rendering-pipeline)). |
| `render_shutdown()` | Release all GL resources. |

**Internal render passes** (called inside `render_frame`):
- `render_spheres()` — iterate `g_bodies`, draw one billboard quad per body via `phong.vert/frag`
- `render_atmospheres()` — additive glow pass for bodies with `atm_intensity > 0`
- `render_dots()` — GL_POINTS at body centres, plus star glare quads
- `render_collision_particles()` — impact debris (max 768 particles via `collision_particles()`)
- `render_build_preview()` — build mode ghost sphere and distance lines

**Logarithmic depth:** All shaders write `gl_FragDepth = log2(eye_depth + 1) / log2(FAR + 1)` (FAR = 2000 AU). This prevents z-fighting across the 14+ orders of magnitude from cm to light-years.

---

### `trails.c` / `trails.h` — Orbital Trail Rendering

**Location:** `src/trails.c` (~250 lines), `src/trails.h` (~23 lines)

Renders the coloured polylines showing orbital paths.

**Design decisions:**
- Positions stored camera-relative (double precision CPU → float VBO) to avoid float precision loss at large distances
- Sampling rate: `~T/400` where T is orbital period, so ~400 samples per orbit
- Dirty tracking: VBO re-uploaded only when new samples added OR camera moved (avoids unnecessary GPU traffic)
- Circular buffer of `TRAIL_LEN` (4096) samples per body

**Functions:**

| Function | Purpose |
|---|---|
| `trails_gl_init()` | Allocate VAOs/VBOs for all initial bodies |
| `trails_add_body(idx)` | Create GL buffers for a newly added body (build mode) |
| `trails_remove_body(idx)` | Delete GL resources for a removed body |
| `trails_reset_body(idx)` | Re-seed trail after discontinuous position change (e.g., capture) |
| `trails_render(vp[16])` | Upload dirty VBOs and draw all trails as `GL_LINE_STRIP` |

Trails fade when camera distance > `SYS_TRAIL_FADE_END` (1200 AU, defined in `common.h`).

---

### `labels.c` / `labels.h` — Body Name Labels

**Location:** `src/labels.c` (~300 lines), `src/labels.h` (~36 lines)

Renders body name labels using SDL_ttf pre-rendered textures on billboard quads.

**`BodyRenderInfo`** (labels.h):
```c
typedef struct {
    float pos[3];  // GL/AU world position
    float dr;      // visual radius (AU)
    float dcam;    // camera distance (AU)
    int   show;    // 1 if body is too small to see as disc
} BodyRenderInfo;
```

**Overlap avoidance:** Greedy AABB sweep — labels are sorted by camera distance, then accepted left-to-right only if their bounding box does not overlap any already-placed label.

**Hysteresis:** `SHOW_DELAY` / `HIDE_DELAY` timers prevent flickering when a body crosses the show/hide threshold.

**Functions:**

| Function | Purpose |
|---|---|
| `labels_init()` | Load font (SDL_ttf), pre-render name textures for all bodies |
| `labels_add_body(idx)` | Create cached text texture for a new body |
| `labels_remove_body(idx)` | Release label texture |
| `labels_render(view, proj, vp, info, dt)` | Project each body to screen, run overlap avoidance, draw billboard quads |
| `labels_shutdown()` | Cleanup |

The Sun always renders at highest priority. Non-star labels are hidden past `MAX_LABEL_DIST` (55 AU).

---

### `starfield.c` / `starfield.h` — Background Stars

**Location:** `src/starfield.c` (~105 lines), `src/starfield.h` (~12 lines)

4000 procedurally generated background stars distributed on the unit sphere.

**Spectral type color distribution:**
| Type | Fraction | Color |
|---|---|---|
| O/B | 3% | Blue-white |
| A | 10% | White |
| F | 17% | Yellow-white |
| G | 25% | Yellow |
| K | 23% | Orange |
| M | 22% | Red |

**Functions:**

| Function | Purpose |
|---|---|
| `starfield_init()` | Generate 4000 stars with realistic spectral colors (constant seed for reproducibility) |
| `starfield_render(view_rot, proj)` | Draw as `GL_POINTS` using rotation-only VP matrix (stars stay fixed) |
| `starfield_shutdown()` | Cleanup |

Two LOD tiers: 1px points (most stars) and 2px points (last quarter of buffer).

---

### `rings.c` / `rings.h` — Planetary Ring System

**Location:** `src/rings.c` (~400 lines), `src/rings.h` (~65 lines)

Keplerian ring particle systems for Saturn, Uranus, and Neptune.

**Physics model:** Each ring particle orbits independently. CPU advances mean anomaly `M += n·dt` each step; GPU solves Kepler's equation per-vertex to get 3D position.

**LOD strategy** (camera distance to ring parent):
| Distance | Mode |
|---|---|
| > 0.2 AU | Flat sprite quad (procedural texture, 1 draw call) |
| 0.05–0.2 AU | Reduced Keplerian particles (`n_lod`) |
| < 0.05 AU | Full Keplerian particles (`n_full`) |

**Particle data layout** (8 floats per particle):
`[M, a, e, ω, h, r, g, b]`

**Functions:**

| Function | Purpose |
|---|---|
| `rings_init(path)` | Parse ring zones from JSON, generate particle arrays, upload VBOs, compile shaders |
| `rings_tick(dt)` | Advance mean anomalies: `M[i] += n[i] * dt` |
| `rings_render(vp)` | LOD selection + draw (Keplerian particles or sprite quad) |
| `rings_on_body_absorbed(target, impactor)` | Update parent index after a collision merge |
| `rings_shutdown()` | Cleanup |

Saturn uses a hardcoded Cassini-division-aware sprite shader (`ring_sprite.frag`). Other planets use a generic uniform-driven sprite (`ring_sprite_generic.frag`).

---

### `asteroids.c` / `asteroids.h` — Asteroid Belts

**Location:** `src/asteroids.c` (~350 lines), `src/asteroids.h` (~7 lines)

Gravity-integrated test particles for the Main Belt and Kuiper Belt.

**Integration:** Symplectic Euler at the outer RESPA rate (~1 day). Particles are test particles (no back-reaction on planets).

**Optimisations:**
- Only loop over non-moon bodies (~14 vs 33 total) — saves ~2.5× pair evaluations
- Cache major-body positions per outer step for CPU L1 hit rate
- Upload positions camera-relative each frame
- Additive blending to avoid saturation on dense regions

**Functions:**

| Function | Purpose |
|---|---|
| `asteroids_init(path)` | Parse belt configs from JSON, generate particles with Keplerian initial conditions |
| `asteroids_step(dt)` | Gravity-integrate all belt particles one outer step |
| `asteroids_render(vp_camrel)` | Upload camera-relative positions, draw as `GL_POINTS` (1–3 px) |
| `asteroids_shutdown()` | Cleanup |

LOD fade parameters (`fade_start_au`, `fade_end_au`) per belt, defined in `universe.json`.

---

### `collision.c` / `collision.h` — Impact & Merge System

**Location:** `src/collision.c` (~700 lines), `src/collision.h` (~39 lines)

Handles solid-body collisions: detection, merge physics, and visual effects.

**Collision types** (`CollisionVisualKind`):
| Kind | Name | Description |
|---|---|---|
| 1 | `COLLISION_VIS_CRATER` | Small impactor — local hot spot, cools over ~45 sim-days |
| 2 | `COLLISION_VIS_MAJOR` | Large impactor — wide ash cloud + molten interior, cools over ~90 sim-days |
| 3 | `COLLISION_VIS_MERGE` | Bodies sinking into each other (MERGE_PHASE_SECONDS ~8 sim-days) |
| 4 | `COLLISION_VIS_INTERSECT` | Live sphere-sphere boundary — bright molten cap at overlap region |

**Functions:**

| Function | Purpose |
|---|---|
| `collision_step(dt)` | Broadphase sphere-sphere checks, initiate merges, update active collision spots |
| `collision_spots_for_body(idx, spots[16])` | Return array of active `CollisionSpot` for a body (passed to phong.frag as uniforms) |
| `collision_on_body_added(idx)` | Mark parent system for hot broadphase checks |
| `collision_visual_radius(idx, physical_radius)` | Returns inflated visual radius during merge (bodies look larger while sinking) |
| `collision_body_heat_glow(idx, col, intensity, scale)` | Returns atmospheric glow params driven by active impacts |
| `collision_body_has_active_merge(idx)` | True if body is currently in merge phase |
| `collision_particles(out, max, cam_pos)` | Fill array of up to 768 `CollisionParticle` for the impact debris renderer |

When a merge completes, the smaller body is removed (`alive = 0`), and the larger body receives the combined mass and momentum. `trails`, `labels`, and `rings` are notified to clean up.

---

### `build.c` / `build.h` — Sandbox Body Placement

**Location:** `src/build.c` (~400 lines), `src/build.h` (~37 lines)

Interactive mode allowing the user to place new bodies into the simulation at runtime.

**Global state:**
```c
int g_build_mode;      // 1 if build mode is active
int g_build_tab_held;  // 1 if Tab is held (scroll wheel changes preset)
```

**Functions:**

| Function | Purpose |
|---|---|
| `build_init()` | Initialise |
| `build_toggle()` | Enter/exit build mode (pauses simulation while active) |
| `build_set_tab_held(held)` | Track Tab key state |
| `build_scroll(wheel_y)` | Adjust placement distance (or cycle presets if Tab held) |
| `build_place_current()` | Place body at preview position; calls `universe_add_body()`, `trails_add_body()`, `labels_add_body()`, `collision_on_body_added()`; returns new body index |
| `build_preset_count()` | 6 |
| `build_selected_index()` | Current preset (0–5) |
| `build_current_preset()` | Pointer to current `BuildPreset` |
| `build_preset_at(idx)` | Preset at given index |
| `build_preview_pos_au(out[3])` | Preview position in AU |
| `build_preview_pos_m(out[3])` | Preview position in metres |
| `build_nearest3(pos_m, out_idx[3], out_dist_au[3])` | Find the 3 nearest bodies to a given position |

Placement velocity is computed to give the new body a circular orbit around the nearest parent (or zero if no suitable parent).

---

### `ui.c` / `ui.h` — 2D HUD Overlay

**Location:** `src/ui.c` (~300 lines), `src/ui.h` (~10 lines)

Renders a 2D screen-space HUD using `ui.vert` / `ui.frag` (orthographic projection).

**HUD elements:**
- **Speed bar** (top-left): log-normalised camera speed fill graphic
- **Speed text**: movement speed (AU/s) and simulation speed (days/s)
- **FPS counter**: exponential moving average
- **Build mode info**: selected preset name and instructions (when active)

**Functions:**

| Function | Purpose |
|---|---|
| `ui_init()` | Load font, prepare text caches, compile `ui.vert/frag` |
| `ui_render()` | Draw all HUD elements |
| `ui_shutdown()` | Cleanup |

---

### `camera.c` / `camera.h` — Free-Look Camera

**Location:** `src/camera.c` (~28 lines), `src/camera.h` (~22 lines)

Minimal camera state. Movement and mouse look are handled in `main.c`; this module only owns the state and provides direction computation.

**Functions:**

| Function | Purpose |
|---|---|
| `cam_reset()` | Reset to default view (pos=[0,3,6] AU, looking at origin) |
| `cam_get_dir(dx, dy, dz)` | Compute forward unit vector from `g_cam.yaw` and `g_cam.pitch` |

---

### `json.c` / `json.h` — JSON Parser

**Location:** `src/json.c` (~400 lines), `src/json.h` (~59 lines)

A minimal recursive-descent JSON parser with two extensions: `//` line comments and scientific notation (via `strtod`).

**Functions:**

| Function | Purpose |
|---|---|
| `json_parse_file(path)` | Read file and return root `JsonNode *` |
| `json_parse(text)` | Parse NUL-terminated string |
| `json_free(root)` | Recursively free tree |
| `json_get(obj, key)` | Find object member by key |
| `json_idx(arr, i)` | Get array element by index |
| `json_num(node, default)` | Safe number accessor |
| `json_str(node, default)` | Safe string accessor |
| `json_bool(node, default)` | Safe boolean accessor |

---

### `gl_utils.c` / `gl_utils.h` — OpenGL Helpers

**Location:** `src/gl_utils.c` (~90 lines), `src/gl_utils.h` (~31 lines)

Thin wrappers for boilerplate OpenGL operations used across multiple modules.

**Functions:**

| Function | Purpose |
|---|---|
| `gl_shader_load(vert_path, frag_path)` | Read, compile, and link a GLSL program; prints errors; returns `GLuint` program ID |
| `gl_vao_create()` | Create and bind a VAO |
| `gl_vbo_create(bytes, data, usage)` | Create VBO and optionally upload data |
| `gl_ebo_create(bytes, data)` | Create and upload an element index buffer |

Every module that uses shaders or GPU buffers calls these helpers instead of raw GL.

---

### `common.h` — Shared Constants

**Location:** `src/common.h` (~60 lines)

Included by almost every file. Defines all physical and rendering constants:

| Constant | Value | Meaning |
|---|---|---|
| `DEFAULT_WIN_W/H` | 1280 / 720 | Default window size |
| `FOV` | 60° | Vertical field of view |
| `PI` | 3.14159… | π |
| `AU` | 1.496e11 | Metres per AU |
| `DAY` | 86400 | Seconds per day |
| `G_CONST` | 6.674e-11 | Gravitational constant |
| `GM_SUN` | AU³/day² | Solar GM in simulation units |
| `SOFTENING` | 1e5 m | Gravitational softening radius |
| `GRAV_EPSILON` | 1e-14 m/s² | Minimum gravity threshold |
| `MAX_BODIES` | 128 | Maximum body array capacity |
| `TRAIL_LEN` | 4096 | Trail samples per body |
| `NUM_STARS` | 4000 | Background star count |
| `RS` | 1/AU | Metres → GL unit scale factor |
| `SYS_TRAIL_FADE_START/END` | AU | Trail LOD fade range |
| `SYS_DOT_FADE_START/END` | AU | Asteroid LOD fade range |

Also declares `int g_win_w, g_win_h` as externals.

---

### `math3d.h` — Header-Only 3D Math

**Location:** `src/math3d.h` (~125 lines)

All matrix and vector operations. Header-only (all functions are `static inline`).

**Types:**
```c
typedef float Mat4[16];   // column-major 4×4
typedef float Vec3[3];
typedef float Vec4[4];
```

**Functions:**
- `vec3_set`, `vec3_copy`, `vec3_add`, `vec3_sub`, `vec3_scale`
- `vec3_dot`, `vec3_len`, `vec3_normalize`, `vec3_cross`
- `mat4_mul_vec4`, `mat4_identity`, `mat4_copy`, `mat4_mul`
- `mat4_perspective(m, fov_y_deg, aspect, near, far)`
- `mat4_lookAt(m, eye, center, up)`
- `mat4_get_right`, `mat4_get_up`, `mat4_get_fwd`
- `mat4_strip_translation` — removes translation (used for skybox VP)
- `mat4_project` — world → screen NDC projection

---

## 6. Shader Reference

All shaders live in `assets/shaders/`. They are loaded at startup by `render_init()` (and by `starfield_init()`, `trails_gl_init()`, etc.) via `gl_shader_load()`.

### `phong.vert` / `phong.frag` — Planet Spheres

**Technique:** Per-pixel ray-sphere intersection. The vertex shader constructs an oversized billboard quad; the fragment shader traces a ray from the camera through each pixel and intersects it with the sphere.

**Key uniforms (frag):**
- `u_center`, `u_oc`, `u_radius` — sphere geometry
- `u_cam_right`, `u_cam_up`, `u_cam_fwd` — camera axes
- `u_fov_tan`, `u_aspect`, `u_screen` — FOV parameters
- `u_rotation`, `u_obliquity` — body rotation state
- `u_planet_type` (0–9) — surface appearance:
  - 0 = generic rocky, 1 = Earth, 2 = Mars, 3 = Venus, 4 = Jupiter, 5 = Saturn, 6 = ice giant, 7 = Io, 8 = Titan, 9 = Europa
- `u_impact_count`, `u_impact_dir[16]`, `u_impact_radius[16]`, `u_impact_heat[16]`, `u_impact_progress[16]`, `u_impact_kind[16]` — collision visuals (up to 16 spots)
- `u_sun_pos_world` — for Phong diffuse lighting
- `u_emission` — > 0.5 for stars (self-luminous)

**Surface texturing:** 3D FBM noise (`hash3` + `vnoise`) — no UV atlas, no texture files needed.

**Logarithmic depth:** `gl_FragDepth = log2(eye_depth + 1) / log2(2001)` where `eye_depth = t * dot(ray_dir, u_cam_fwd)`.

---

### `atm.vert` / `atm.frag` — Atmospheric Limb Glow

Renders additive atmospheric glow for bodies with `atm_intensity > 0`.

**Technique:** Ray-shell intersection. Glow intensity: `(1 - norm)^3` where `norm` is the normalised closest-approach distance inside the shell. Lit-side boost: `0.15 + 0.85 * clamp(sun_dot, 0, 1)`.

Uses the **back face** of the atmosphere sphere for depth so that glow renders correctly behind solid bodies during merges.

---

### `solid.vert` / `solid.frag` — Orbital Trail Lines

Simple `GL_LINE_STRIP` renderer. Takes camera-relative float positions. Uses `u_count` trick: if `u_count = 0`, renders fully opaque (used for pole axis lines in earlier versions). Logarithmic depth.

---

### `ring.vert` — Keplerian Ring Particles

Solves Kepler's equation on the GPU per vertex:
1. Newton-Raphson: `M = E - e·sin(E)` → eccentric anomaly E
2. True anomaly: `ν = 2·atan2(√(1+e)·sin(E/2), √(1-e)·cos(E/2))`
3. Orbital radius: `r = a(1 - e·cos(E))`
4. 3D position via ring-plane basis vectors `u_b1`, `u_b2`

Input attributes: `M`, `a`, `e`, `ω`, `h` (vertical offset), `r/g/b`.

---

### `ring_sprite.frag` / `ring_sprite_generic.frag` — Ring Sprites

Far-LOD ring rendering: a single quad textured procedurally.
- `ring_sprite.frag`: Saturn-specific hardcoded Cassini-division zone map
- `ring_sprite_generic.frag`: uniform-driven (`u_r_inner`, `u_r_outer`, `u_ring_color`, `u_alpha_max`) for Uranus/Neptune

---

### `asteroid_particle.vert` — Asteroid GL_POINTS

Camera-relative GL_POINTS with size ramp (1→3 px) and distance-based fade driven by `u_fade_start` / `u_fade_end`. Additive blending.

---

### `impact_particle.vert` / `impact_particle.frag` — Collision Debris

Per-particle size and RGBA. Fade over particle lifetime. Used for the debris cloud emitted on large impacts.

---

### `label.vert` / `label.frag` — Body Name Labels

Billboard quad constructed in the vertex shader from `u_right` and `u_up` (camera basis). Fragment shader: texture lookup, transparent regions discarded. Font pre-rendered with SDL_ttf.

---

### `star_glare.vert` / `star_glare.frag` — Star Halos

Extra-large quad centered on each star. Radial gradient fades to transparent. Fades in based on star's projected size on screen.

---

### `ui.vert` / `ui.frag` — 2D HUD

Screen-space orthographic projection. `u_use_tex` selects solid colour (speed bar fill) or textured (text quads).

---

### `build_line.vert` / `build_line.frag` — Build Mode Preview

Lines from placement preview position to the 3 nearest bodies. White = acceptable distance, red = too close.

---

## 7. Universe Data Format

**File:** `assets/universe.json`

The single source of truth for all simulation content. Supports `//` line comments.

### Stars

```json
{
  "name": "Sun",
  "type": "star",
  "pos_ly": [0.0, 0.0, 0.0],
  "mass": 1.989e30,
  "radius_km": 696000.0,
  "color": [1.0, 0.92, 0.23],
  "obliquity_deg": 7.25,
  "rotation_period_days": 25.38
}
```

### Planets (Keplerian elements, JPL J2000)

```json
{
  "name": "Earth",
  "type": "planet",
  "parent": "Sun",
  "mass": 5.972e24,
  "radius_km": 6371.0,
  "color": [0.29, 0.48, 0.78],
  "obliquity_deg": 23.44,
  "rotation_period_days": 0.9973,
  "atmosphere": { "color": [0.4, 0.6, 1.0], "intensity": 0.55, "scale": 1.03 },
  "keplerian": {
    "a_au": 1.00000261,
    "e": 0.01671123,
    "i_deg": -0.00001531,
    "Omega_deg": 0.0,
    "omega_tilde_deg": 102.93768193,
    "L_deg": 100.46457166,
    "epoch_jd": 2451545.0
  }
}
```

### Moons (planetocentric Keplerian)

```json
{
  "name": "Luna",
  "type": "moon",
  "parent": "Earth",
  "mass": 7.342e22,
  "radius_km": 1737.4,
  "color": [0.72, 0.72, 0.68],
  "moon_keplerian": {
    "a_km": 384400.0,
    "e": 0.0549,
    "i_deg": 5.16,
    "Omega_deg": 125.08,
    "omega_deg": 318.15,
    "M0_deg": 115.3654
  }
}
```

### Rings

```json
{
  "parent": "Saturn",
  "n_full": 4096,
  "n_lod": 512,
  "zones": [
    { "r_min_km": 66900.0, "r_max_km": 74500.0, "density": 0.15, "color": [0.85, 0.82, 0.70] }
  ]
}
```

### Asteroid Belts

```json
{
  "name": "Main Belt",
  "parent": "Sun",
  "a_au": 2.87,
  "e": 0.15,
  "i_deg": 1.86,
  "Omega_deg": 73.0,
  "omega_deg": 100.0,
  "count": 10000,
  "color": [0.6, 0.6, 0.55],
  "fade_start_au": 10.0,
  "fade_end_au": 50.0
}
```

---

## 8. Physics Deep Dive

### RESPA Force Split

Forces are split into:
- **Slow:** All body-body gravitational pairs *except* the dominant parent-satellite pair (~O(N²) for N primaries, updated every outer step `dt_outer ~1 day`)
- **Fast:** Dominant parent → satellite force only (updated every inner step `dt_inner ~0.02 day`)

For a moon, the dominant parent force is ~10³× larger than tidal perturbations — so using a 50× shorter inner timestep on only that force pair is ~32× more efficient than stepping everything at inner rate.

### One Outer Step

```
physics_respa_begin_system(root, dt_outer)
    v[all] += 0.5 * a_slow[all] * dt_outer

    for i in range(n_inner):
        physics_respa_inner_system(root, dt_inner)
            compute fast_acc[satellites] = gravity from parent only
            v[all]   += 0.5 * fast_acc * dt_inner
            pos[all] += v[all] * dt_inner
            v[all]   += 0.5 * fast_acc * dt_inner

physics_respa_end_system(root, dt_outer)
    v[all] += 0.5 * a_slow[all] * dt_outer   // second slow half-kick
    rotation_angle[i] += rotation_rate[i] * dt_outer
    g_sim_time += dt_outer
```

### Warmup (Pre-Simulation)

Before the render loop, the simulation runs forward 2 years (`WARMUP_DT = 2 * 365 * DAY`) to populate the orbital trail buffers. Multiple star systems are warmed up in parallel via `#pragma omp parallel for schedule(dynamic)`.

### Kepler Solver

```c
// Newton-Raphson: M = E - e*sin(E)
double E = M;
for (int k = 0; k < 50; k++) {
    double dE = (M - E + e*sin(E)) / (1.0 - e*cos(E));
    E += dE;
    if (fabs(dE) < 1e-12) break;
}
```

Converges in 3–5 iterations for `e < 0.3`; 50-iteration cap handles near-parabolic orbits.

---

## 9. Rendering Pipeline

Full sequence inside `render_frame()`:

| Step | Function / Shader | Notes |
|---|---|---|
| 1 | `starfield_render()` / `color.vert/frag` | Rotation-only VP, GL_POINTS |
| 2 | `render_spheres()` / `phong.vert/frag` | Billboard quads, ray-sphere intersection |
| 3 | `render_atmospheres()` / `atm.vert/frag` | Additive blend, GL_SRC_ALPHA / GL_ONE |
| 4 | `render_dots()` / `color.vert/frag` + `star_glare` | GL_POINTS + halos |
| 5 | `trails_render()` / `solid.vert/frag` | Camera-relative polylines |
| 6 | `rings_render()` / `ring.vert` or sprite shaders | LOD: particles ↔ sprite |
| 7 | `asteroids_render()` / `asteroid_particle.vert` | GL_POINTS, additive |
| 8 | `render_collision_particles()` / `impact_particle.vert/frag` | Additive debris |
| 9 | `labels_render()` / `label.vert/frag` | Billboard text quads |
| 10 | `render_build_preview()` / `build_line.vert/frag` | Build mode only |
| 11 | `ui_render()` / `ui.vert/frag` | Screen-space HUD |
| 12 | `SDL_GL_SwapWindow()` | Present |

**Blending modes:**
| Pass | `glBlendFunc` |
|---|---|
| Opaque geometry | `GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA` |
| Atmospheres, asteroids, collision particles, ring additive | `GL_SRC_ALPHA, GL_ONE` |

---

## 10. File Dependency Graph

```
main.c
├── body.h, physics.h, camera.h, universe.h
├── render.h, trails.h, labels.h, starfield.h
├── rings.h, asteroids.h, collision.h, build.h, ui.h
└── common.h

body.c       → body.h, camera.h, math3d.h
physics.c    → physics.h, body.h, common.h
universe.c   → universe.h, body.h, json.h, common.h
render.c     → render.h, body.h, camera.h, math3d.h, gl_utils.h
             → starfield.h, trails.h, labels.h, rings.h
             → asteroids.h, collision.h, build.h

trails.c     → trails.h, body.h, camera.h, gl_utils.h, math3d.h, common.h
labels.c     → labels.h, body.h, camera.h, gl_utils.h, math3d.h
starfield.c  → starfield.h, gl_utils.h, common.h
rings.c      → rings.h, body.h, json.h, gl_utils.h, math3d.h, common.h
asteroids.c  → asteroids.h, body.h, json.h, gl_utils.h, common.h
collision.c  → collision.h, body.h, labels.h, rings.h, trails.h, common.h
build.c      → build.h, body.h, camera.h, physics.h, universe.h
             → trails.h, labels.h, collision.h
ui.c         → ui.h, camera.h, physics.h, build.h, body.h, gl_utils.h

json.c       → json.h                    (standalone)
gl_utils.c   → gl_utils.h, common.h     (standalone)
camera.c     → camera.h                 (minimal, mostly state)
```

---

## 11. Adding New Features — Where to Edit

### Add a new planet or moon
→ Edit `assets/universe.json`. Add a JSON object under `"bodies"` with `"type": "planet"` or `"type": "moon"`, a `"parent"` name, and either `"keplerian"` (planet) or `"moon_keplerian"` (moon) elements. No code changes required.

### Add a planet surface appearance
→ Edit `assets/shaders/phong.frag`. Add a new `case` in the `u_planet_type` switch. Increment the type counter and assign the new index when constructing the body in `render.c` (search for `u_planet_type` uniform upload).

### Add a new ring system
→ Edit `assets/universe.json`. Add an entry to the `"rings"` array with `"parent"`, `"n_full"`, `"n_lod"`, and `"zones"`. If the planet needs a custom sprite shader, add it in `rings.c::rings_init()`.

### Add a new asteroid belt
→ Edit `assets/universe.json`. Add an entry to `"asteroid_belts"` with `"parent"`, `"a_au"`, `"e"`, `"i_deg"`, `"count"`, etc.

### Add a new body type / build preset
→ Edit `build.c`. Add a new `BuildPreset` to the `presets[]` array. No header change needed unless the preset count is hard-coded elsewhere.

### Add a new rendering effect per body
→ Add uniforms to `phong.frag`. Upload them in the sphere-draw loop in `render.c::render_spheres()`. Follow the pattern of how `u_impact_*` uniforms are set.

### Add a new HUD element
→ Edit `ui.c::ui_render()`. Use `gl_vbo_create()` for geometry and the `ui.vert/frag` shader. For text, pre-render with SDL_ttf in `ui_init()`.

### Add a new physics force
→ Edit `physics.c`. Add force computation to the appropriate section of `physics_respa_inner_system()` (fast forces, inner loop) or `physics_respa_begin/end_system()` (slow forces, outer loop). Update `physics_refresh_timestep_model()` if the new force affects timestep selection.

### Change the simulation initial state
→ Edit `assets/universe.json`. The Keplerian element format follows the JPL Horizons/Solar System Dynamics table 1 convention (a, e, i, Ω, ω̃, L at J2000 epoch).

### Add support for textures
→ The rotation angle is already tracked in `Body.rotation_angle` (updated in `physics_respa_end_system`). The obliquity is in `Body.obliquity`. Add a texture sampler to `phong.frag`, compute UV from the surface normal rotated by these values, and sample accordingly.

---

*This document reflects the codebase as of April 2025. When in doubt, the source files are the authoritative reference.*
