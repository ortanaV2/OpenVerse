<img width="1920" height="710" alt="OpenVerse_Logo_Banner" src="https://github.com/user-attachments/assets/29ec1633-2c24-4738-bfa3-7923fc10b0d0" />

# OpenVerse - Universe Simulator
real-time N-body gravitational simulation rendered with OpenGL 3.3 Core.
A 3D simulation playground of the universe to illustrate its enormous dimensions.

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

## Showcase
<img width="1263" height="713" alt="image" src="https://github.com/user-attachments/assets/f50e9843-0965-46bb-957b-9d3794b07a5c" />
_Fly wherever you want and discover new perspectives._ 

<img width="1273" height="712" alt="image" src="https://github.com/user-attachments/assets/47b08b19-8944-46ab-8b90-c19a2dfcb592" />
_Our solar system was precisely modeled with many small details._ 
