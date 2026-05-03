# Raylib Boids

A [Boids](https://en.wikipedia.org/wiki/Boids) flocking simulation built with [raylib](https://www.raylib.com/).

## Features

- **Flocking Simulation** — Implements Craig Reynolds' Boids algorithm with separation, alignment, and cohesion rules.
- **Interactive Controls** — Click and drag to spawn boids, right-click to attract/repel them.
- **Audio Feedback** — Plays a sound effect on each click.
- **Custom Font** — Uses Press Start 2P for retro-style UI text.
- **Web Build** — Can be compiled to WebAssembly via Emscripten and run in a browser.

## Controls

| Input | Action |
|-------|--------|
| Left mouse click | Spawn a boid at cursor position |
| Left mouse drag | Continuously spawn boids along the drag path |
| Right mouse click | Attract/repel boids (toggle) |
| Escape | Exit simulation |

## Building

### Prerequisites

- [raylib](https://github.com/raysan5/raylib) installed on your system (the build expects it at `c:/raylib/raylib/` on Windows).
- [CMake](https://cmake.org/) 3.0 or later.
- A C++ compiler (MSVC, MinGW, GCC, Clang, etc.).

### Desktop (Windows / Linux / macOS)

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

On Windows with Visual Studio you can also open `raylib_boids.sln` and build from the IDE.

### Web (Emscripten)

1. Install [Emscripten](https://emscripten.org/) and ensure `emcc`/`em++` are in your `PATH`.
2. Run the build script:

```bash
./build_web.sh
```

3. Serve the output with any HTTP server:

```bash
python -m http.server 8080
```

Then open `http://localhost:8080` in your browser.

Alternatively, use `run_web.bat` on Windows to build and launch a local server automatically.

## License

This project is licensed under the MIT License. See [LICENSE.txt](LICENSE.txt) for details.