# GUI Build Instructions

The simulation uses **SFML 2.5 or later** for its graphical interface. You need to install SFML before building. Choose the instructions for your operating system.

---

## Linux (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install libsfml-dev build-essential
```

Then build:
```bash
cd sensor-fusion-system
make
./simulation
```

---

## macOS

Install [Homebrew](https://brew.sh) if you don't have it, then:

```bash
brew install sfml
```

Build:
```bash
cd sensor-fusion-system
make
./simulation
```

If `make` can't find SFML headers on newer Macs with Apple Silicon, compile manually with the Homebrew paths:
```bash
g++ -std=c++17 src/*.cpp -o simulation \
    -I/opt/homebrew/include -L/opt/homebrew/lib \
    -lsfml-graphics -lsfml-window -lsfml-system
./simulation
```

---

## Windows

### Option A — MinGW (simplest)

1. Download **GCC 13.1.0 MinGW (SEH)** SFML build from:
   https://www.sfml-dev.org/download/sfml/2.6.1/
2. Extract to `C:\SFML`
3. Install MinGW-w64 if you don't have `g++` already
4. Open a terminal in the `sensor-fusion-system` folder and run:

```bash
g++ -std=c++17 src/*.cpp -o simulation.exe ^
    -IC:\SFML\include -LC:\SFML\lib ^
    -lsfml-graphics -lsfml-window -lsfml-system
```

5. Copy all `.dll` files from `C:\SFML\bin\` into the same folder as `simulation.exe`
6. Run: `simulation.exe`

### Option B — Visual Studio

1. Download the **Visual C++ 17 (2022)** SFML build from the SFML website
2. Create a new "Empty C++ Project" in Visual Studio
3. Add all files from `src/` to the project
4. Right-click project → Properties:
   - **C/C++ → General → Additional Include Directories:** `C:\SFML\include`
   - **Linker → General → Additional Library Directories:** `C:\SFML\lib`
   - **Linker → Input → Additional Dependencies:** add `sfml-graphics.lib;sfml-window.lib;sfml-system.lib;`
5. Copy the SFML DLLs next to the compiled `.exe`
6. Build and run (F5)

---

## Controls Once It's Running

| Key | Action |
|---|---|
| **SPACE** | Pause / resume the simulation |
| **R** | Clear the trajectory trail |
| **ESC** | Quit |

## What You'll See

- **Left side (1000x800):** 2D map with grid, axes, and the vehicle rendered as a yellow triangle that rotates with its heading. A fading blue trail shows where the vehicle has been.
- **Right side (380px wide):** Five live data panels
  - **IMU Sensor** (orange header) — velocity and heading readings
  - **GPS Sensor** (blue header) — position readings
  - **Temperature Sensor** (red header) — ambient temperature
  - **Wheel Encoder** (green header) — velocity from wheel rotation
  - **Fused State Estimate** (highlighted blue) — the combined output

All panels update in real time as the simulation runs. Noise and sensor dropout are visible — you can watch the GPS reading jitter slightly while the fused estimate stays smooth.

---

## Troubleshooting

**"SFML/Graphics.hpp: No such file"**
SFML isn't installed. See the install steps above for your OS.

**"cannot find -lsfml-graphics"**
SFML libraries aren't in your linker path. On Linux, make sure you ran `sudo apt install libsfml-dev`. On Mac, verify `brew install sfml` completed. On Windows, double-check the `-L` path points to your SFML `lib` folder.

**Window opens but text is missing**
The visualizer couldn't find a font. On most Linux distros it picks up DejaVu Sans automatically. On Windows it uses Arial. If text is missing, drop any `.ttf` font file named `arial.ttf` or `DejaVuSans.ttf` into the folder where you run the executable.

**Window opens black / vehicle doesn't appear**
Press **R** to reset the trail, then give it a second — the vehicle should appear near the left-center of the map. If still nothing, check the console for error messages from the fusion system.
