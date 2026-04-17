# verse — solar system simulator

> **WIP** — real-time N-body gravitational simulation rendered with OpenGL 3.3 Core.

---

## Building

**Windows (MSYS2 / MinGW-w64)**
```bash
pacman -S mingw-w64-x86_64-SDL2 mingw-w64-x86_64-SDL2_ttf mingw-w64-x86_64-glew
mingw32-make
./verse.exe
```

**Linux**
```bash
sudo apt install libsdl2-dev libsdl2-ttf-dev libglew-dev
make
./verse
```

---

## Controls

| Key / Input | Action |
|---|---|
| Left-click | Enter free-look (captures mouse) |
| Escape | Exit free-look |
| W / S | Move forward / backward |
| A / D | Strafe left / right |
| Q / E | Move down / up |
| Mouse | Look around |
| Scroll | Camera speed |
| `+` / `-` | Simulation speed up / down |
| Space | Pause / resume |
| R | Reset camera |

**Simulation speeds:** `0 → 0.1 → 0.25 → 0.5 → 1 → 2 → 5 → 10 → 30 → 60 → 100 → 365 → 730 → 1825 → 3650 → 36500` days/s

---

## Project structure

```
verse/
├── Makefile
├── assets/
│   └── shaders/
│       ├── color.vert / color.frag     # flat colour + round GL_POINTS
│       ├── phong.vert / phong.frag     # Phong-lit sphere billboards
│       ├── solid.vert / solid.frag     # uniform colour with alpha (trails)
│       └── label.vert / label.frag     # camera-aligned text billboard
└── src/
    ├── common.h        # shared constants (AU, DAY, window size, …)
    ├── math3d.h        # header-only Mat4/Vec3/Vec4 (column-major, OpenGL)
    ├── body.[ch]       # Body struct, g_bodies[], solar_system_init()
    ├── physics.[ch]    # Leapfrog KDK N-body integrator, trails_sample()
    ├── camera.[ch]     # free-look camera (yaw/pitch)
    ├── gl_utils.[ch]   # VAO/VBO/EBO/shader helpers
    ├── starfield.[ch]  # procedural star skybox (rotation-only VP)
    ├── trails.[ch]     # orbital trail circular buffers + GL upload
    ├── labels.[ch]     # SDL_ttf text → GL texture billboards
    ├── render.[ch]     # main render pipeline (spheres, dots, trails, labels)
    └── main.c          # SDL2 window, event loop, physics step, camera
```

### Module dependencies

```
main
 ├── physics   ← body
 ├── camera
 ├── render
 │    ├── starfield
 │    ├── trails    ← body
 │    └── labels    ← body
 └── gl_utils  (used by render, starfield, trails, labels)
```

---

## How to extend

### Add a new body
Add an entry to `solar_system_init()` in `src/body.c`. The physics, trails, labels and rendering all iterate `g_bodies[0..g_nbodies-1]` automatically.

### Add a new shader effect
1. Drop `name.vert` / `name.frag` into `assets/shaders/`
2. Load via `gl_shader_load()` in the relevant module
3. Call from `render_frame()` in `render.c` at the right pipeline stage

### Planned / easy to integrate

| Feature | Where |
|---|---|
| Moons / hierarchical orbits | `body.h` — add `parent_idx`, update `solar_system_init()` + camera follow |
| Click-to-select body | `math3d.h` has `mat4_project()`; ray-cast in `main.c` mouse handler |
| HUD / info panel | Extend `labels.c` with screen-space quads, or add an `imgui` layer |
| Atmospheric glow | Extra additive-blend pass in `render.c` after sphere draw |
| Time scrubbing | Record state snapshots in `physics.c`, add a rewind buffer |
| Export trajectory | `trails_sample()` already captures positions; write CSV from there |
| Relativistic effects | Swap integrator in `physics.c` — everything else stays the same |

### Big Features for the Future
- Replace the skybox with authentic stars and planets
- Add asteroid belts
- Implement collision physics (including supernovae, etc.)
- Refine the GUI / User Interface