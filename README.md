<img width="1920" height="710" alt="OpenVerse_Logo_Banner" src="https://github.com/user-attachments/assets/29ec1633-2c24-4738-bfa3-7923fc10b0d0" />

# OpenVerse

**OpenVerse** is an open-source, open-world universe simulator.  
The goal is to simulate as much of the universe as possible - multiple star systems, planets, moons, asteroid belts, rings, and more - driven by real gravitational physics.

Not a screensaver. Not a game. A sandbox for curiosity.

---

## Vision

Space is incomprehensibly large. Most simulations either abstract that away or confine you to a single solar system. OpenVerse doesn't.

- **Real scale.** Every distance, mass, and orbital period is physically accurate. The emptiness between planets is real. Flying from Earth to Neptune takes time.
- **Real dynamics.** Bodies move under genuine N-body gravity — no baked animations, no shortcuts. Disrupt the solar system and watch it react.
- **Open world.** Fly anywhere. Approach an asteroid from 10 km. Pull back until the entire solar system fits on screen. Eventually, jump to another star.
- **Sandbox.** Spawn scenarios, collide objects, break things. Curiosity should have no guardrails.

---

## Current State

| Feature | Status |
|---|---|
| N-body gravity (RESPA integrator) | ✓ |
| Solar system — Sun, 8 planets, dwarf planets | ✓ |
| Moons (Luna, Phobos, Deimos, Galilean moons, Titan, …) | ✓ |
| Axial tilt & planetary rotation | ✓ |
| Atmospheric glow (Venus, Earth, Mars, gas giants, Titan) | ✓ |
| Saturn rings — Keplerian particles + LOD sprite | ✓ |
| Uranus & Neptune rings | ✓ |
| Main Belt & Kuiper Belt — gravity-integrated particles | ✓ |
| Orbital trails | ✓ |
| Logarithmic depth buffer (cm → light-years in one scene) | ✓ |
| Multiple star systems | ✓ |
| Nearby exoplanet systems | ✓ |
| Data-driven universe config | ✓ |
| Variable warp travel | ✓ |
| Black holes | planned |

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
| Scroll | Camera speed / warp speed |
| T | Toggle warp mode |
| F11 / Alt+Enter | Toggle fullscreen |
| `+` / `-` | Simulation speed up / down |
| Space | Pause / resume |
| R | Reset camera near the Sun, looking at the Sun |

**Simulation speeds:** `0 → 0.1 → 0.25 → 0.5 → 1 → 2 → 5 → 10 → 30 → 60 → 100 → 365 → 730 → 1825 → 3650` days/s

---

## Showcase

<img width="1263" height="713" alt="image" src="https://github.com/user-attachments/assets/f50e9843-0965-46bb-957b-9d3794b07a5c" />
*Fly wherever you want and discover new perspectives.*

<img width="1273" height="712" alt="image" src="https://github.com/user-attachments/assets/47b08b19-8944-46ab-8b90-c19a2dfcb592" />
*Our solar system was precisely modeled with many small details.*

---

## Contributing

OpenVerse is open source and early in development. The physics engine, rendering pipeline, and coordinate system are all designed to scale beyond a single solar system. If you want to help push toward a truly open universe, contributions are welcome.
