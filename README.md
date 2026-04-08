# Sensor Fusion System for Autonomous Vehicle Simulation

**EECE 2140 — Professor Nafa | Spring 2026**

An interactive C++ simulation of an autonomous vehicle equipped with IMU, GPS,
Temperature, and Wheel Encoder sensors. Noisy sensor readings are fused using a
weighted-average algorithm and displayed on a real-time SFML graphics dashboard.

## Team Members

- **Jeongheon Han (David)** — Sensor class hierarchy (base + 4 subclasses)
- **Charles Wan** — FusionSystem class (weighted-average fusion engine)
- **Anish Machiraju** — Vehicle class (kinematics) + SFML Visualizer (GUI)

## Features

- **Live 2D map** with grid, axes, and the vehicle drawn as a heading arrow
- **Trajectory trail** that fades over time to show where the vehicle has been
- **Four sensor data panels** showing each sensor's latest noisy reading
- **Fused state estimate panel** showing the combined result from fusion
- **Interactive controls** — pause, clear trail, quit
- **Real-time simulation loop** synced to ~20 updates per second

## Screenshot

When you run the program, a 1400×800 window opens with:

```
+------------------------------------+------------------+
|                                    |  IMU Sensor      |
|        LIVE MAP VIEW               |  pos, vel, hdg…  |
|                                    +------------------+
|    • • •                           |  GPS Sensor      |
|       • •  <- vehicle              |  pos, vel, hdg…  |
|         • (heading arrow)          +------------------+
|          •                         |  Temperature     |
|           •                        |  temp reading…   |
|            •                       +------------------+
|             • trajectory trail     |  Wheel Encoder   |
|                                    |  velocity…       |
|                                    +------------------+
|  [grid, axes, (0,0) origin]        |  FUSED STATE     |
|                                    |  final estimate… |
+------------------------------------+------------------+
  SPACE: pause   R: clear trail   ESC: quit
```

## OOP Design Summary

The system consists of **9 components**:

- **`StateEstimate`** (struct) — Shared data type: position, velocity, heading, temperature, timestamp
- **`Sensor`** (base class) — Noise generation, calibration, update-rate tracking
  - **`IMUSensor`** — Velocity and heading noise
  - **`GPSSensor`** — Position noise with signal dropout
  - **`TemperatureSensor`** — Thermal noise with response delay
  - **`WheelEncoderSensor`** — Tick-based velocity estimation
- **`FusionSystem`** — Weighted-average sensor fusion
- **`Vehicle`** — Ground-truth kinematics and state history
- **`Visualizer`** — SFML-based real-time GUI dashboard

## Tools and Technologies

- **Language:** C++17
- **Graphics:** SFML 2.5+ (Simple and Fast Multimedia Library)
- **Build:** GNU Make + g++
- **Version Control:** Git / GitHub
- **IDE:** VS Code
- **Documentation:** LaTeX (Overleaf)

## Folder Structure

```
sensor-fusion-system/
├── README.md
├── Makefile
├── docs/
│   ├── System_Design_Overview.pdf
│   ├── Iteration_02.pdf
│   ├── Iteration05_Pseudocode.pdf
│   └── Iteration05_Pseudocode.tex
├── pseudocode/
│   └── pseudocode.txt
├── src/
│   ├── main.cpp
│   ├── StateEstimate.h
│   ├── Sensor.h / .cpp
│   ├── IMUSensor.h / .cpp
│   ├── GPSSensor.h / .cpp
│   ├── TemperatureSensor.h / .cpp
│   ├── WheelEncoderSensor.h / .cpp
│   ├── FusionSystem.h / .cpp
│   ├── Vehicle.h / .cpp
│   └── Visualizer.h / .cpp
└── images/
```

## Installing SFML

**Ubuntu / Debian / WSL:**
```bash
sudo apt-get install libsfml-dev
```

**macOS (Homebrew):**
```bash
brew install sfml
```

**Windows:**
Download the SFML 2.6 SDK from <https://www.sfml-dev.org/download.php> and
follow the Visual Studio or MinGW setup guide. For MinGW users, add SFML's
`bin/` to your PATH.

## Build and Run

With the provided Makefile:
```bash
make        # builds the executable
make run    # builds and runs
make clean  # removes the executable
```

Or manually:
```bash
g++ -std=c++17 -o simulation src/*.cpp \
    -lsfml-graphics -lsfml-window -lsfml-system
./simulation
```

## Controls

The GUI has three on-screen buttons at the bottom of the map:

| Button   | Action                                    |
|----------|-------------------------------------------|
| **START** | Begin simulation (becomes RESUME when paused) |
| **PAUSE** | Pause the running simulation              |
| **RESET** | Rebuild the simulation from t=0           |

Keyboard shortcuts still work:

| Key      | Action                     |
|----------|----------------------------|
| `SPACE`  | Pause / resume simulation  |
| `R`      | Reset simulation to t=0    |
| `ESC`    | Close window and quit      |

## How It Works

1. **Vehicle** advances its ground-truth state each timestep using a kinematic
   motion model (heading + velocity → position).
2. **Sensors** observe the true state and add type-specific noise via their
   overridden `generateReading()` method.
3. **FusionSystem** collects every sensor's latest reading and computes a
   weighted-average `StateEstimate`.
4. The fused estimate is fed back to the **Vehicle** via `updateState()`.
5. All four sensor readings and the fused estimate are pushed to the
   **Visualizer**, which redraws the full dashboard at 60 FPS.

## Project Goals

1. Demonstrate sensor fusion concepts through a practical simulation
2. Practice collaborative C++ development with Git and GitHub
3. Apply object-oriented design with inheritance and polymorphism
4. Deliver a well-documented, visually impressive, fully functional program
