# Sensor Fusion System for Autonomous Vehicle Simulation

**EECE 2140 — Professor Nafa | Spring 2026**

An interactive C++ simulation of an autonomous vehicle equipped with IMU, GPS,
Temperature, and Wheel Encoder sensors. Noisy sensor readings are fused using a
weighted-average algorithm. The program runs in two modes a terminal mode
with text prompts, and a GUI mode with a startup configuration dialog and a
live SFML graphics dashboard.

## Team Members

- **Jeongheon Han (David)** — Sensor class hierarchy (base + 4 subclasses)
- **Charles Wan** — FusionSystem class (weighted-average fusion engine)
- **Anish Machiraju** — Vehicle class (kinematics + mass physics) and SFML GUI

## Features

- **Two run modes** — terminal (CLI) and GUI (graphical dashboard)
- **User input in both modes** — five configurable parameters with full validation
- **Vehicle mass affects turning** — heavier vehicles turn more slowly (rotational inertia)
- **Live 2D map** with grid, axes, and the vehicle drawn as a heading arrow
- **Trajectory trail** that fades over time
- **Four sensor data panels** showing each sensor's latest noisy reading
- **Fused state estimate panel** showing the combined result
- **Interactive START / PAUSE / RESET buttons** plus keyboard shortcuts
- **Robust error handling** — out-of-range values are rejected with clear messages

## Configurable Parameters

Both run modes prompt for these five parameters and validate them:

| Parameter        | Range            | Default | Notes                                             |
|------------------|------------------|---------|---------------------------------------------------|
| Vehicle speed    | 0.1 – 50.0 m/s   | 5.0     | Scalar speed of the vehicle                       |
| Turn rate        | -90.0 – 90.0 deg/s | 10.0  | Heading change rate (positive = left)             |
| Duration         | 0.0 – 3600.0 s   | 30 / 0  | 0 = infinite in GUI mode; defaults to 30 in CLI   |
| GPS noise sigma  | 0.0 – 20.0 m     | 1.5     | Standard deviation of GPS position noise          |
| Vehicle mass     | 100 – 20000 kg   | 1000    | Affects turning agility (heavier = slower turns)  |

### How mass affects the simulation

Heavier vehicles have more rotational inertia, so they turn more slowly. The
effective turn rate is computed as:

```
effectiveTurnRate = commandedTurnRate × (1000 / mass)^0.3
```

| Mass     | Factor | Effective turn rate (when commanded = 10°/s) |
|----------|--------|------------------------------------------------|
| 100 kg   | 1.99   | 19.9 °/s                                       |
| 500 kg   | 1.23   | 12.3 °/s                                       |
| 1000 kg  | 1.00   | 10.0 °/s                                       |
| 5000 kg  | 0.62   |  6.2 °/s                                       |
| 20000 kg | 0.41   |  4.1 °/s                                       |

This means a 100 kg motorcycle traces a tight circle, while a 20000 kg semi
truck traces a wide arc — for the same commanded turn rate.

## OOP Design Summary

The system consists of **10 components**:

- **`StateEstimate`** (struct) — Shared data type holding position, velocity, heading, temperature, timestamp
- **`SimConfig`** (struct) — User-configurable parameters with validation
- **`Sensor`** (base class) — Noise generation, calibration, update-rate tracking
  - **`IMUSensor`** — Velocity and heading noise
  - **`GPSSensor`** — Position noise with signal dropout
  - **`TemperatureSensor`** — Thermal noise with response delay
  - **`WheelEncoderSensor`** — Tick-based velocity estimation
- **`FusionSystem`** — Weighted-average sensor fusion
- **`Vehicle`** — Ground-truth kinematics, mass physics, and state history
- **`Visualizer`** — SFML real-time GUI dashboard
- **`InputDialog`** — SFML startup dialog for parameter entry

## Tools and Technologies

- **Language:** C++17
- **Graphics:** SFML 3.x (Simple and Fast Multimedia Library)
- **Build:** GNU Make + g++
- **Version Control:** Git / GitHub
- **IDE:** VS Code
- **Documentation:** LaTeX (Overleaf)

## Folder Structure

```
sensor-fusion-system/
├── README.md
├── BUILD_GUI.md
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
│   ├── SimConfig.h
│   ├── StateEstimate.h
│   ├── Sensor.h / .cpp
│   ├── IMUSensor.h / .cpp
│   ├── GPSSensor.h / .cpp
│   ├── TemperatureSensor.h / .cpp
│   ├── WheelEncoderSensor.h / .cpp
│   ├── FusionSystem.h / .cpp
│   ├── Vehicle.h / .cpp
│   ├── Visualizer.h / .cpp
│   └── InputDialog.h / .cpp
└── images/
```

## Build and Run

### Install SFML 3 (one time)

**macOS (Homebrew):**
```bash
brew install sfml
```

**Linux (Ubuntu / Debian):**
See `BUILD_GUI.md` for detailed instructions.

### Build
```bash
make
```

### Run

Choose either mode:

```bash
make terminal       # Terminal mode (text prompts, console output, no graphics)
make gui            # GUI mode (input dialog + live dashboard) - default
make run            # Same as 'make gui'
```

You can also run the binary directly:

```bash
./simulation                # GUI mode
./simulation --gui          # GUI mode
./simulation --terminal     # Terminal mode
./simulation -t             # Short form
./simulation --help         # Show usage
```

## Run Modes

### Terminal Mode

Pure command-line. Prompts you for each parameter, validates it, then runs the
simulation and prints state to the console at each timestep:

```
================================================================
  Sensor Fusion System - Terminal Mode
================================================================
Enter simulation parameters (press Enter to keep default):

  Vehicle speed [5 m/s]: 8
  Turn rate [10 deg/s]: 15
  Duration (0 = infinite, will stop at 60s for terminal mode) [30 s]: 20
  GPS noise sigma [1.5 m]: 2.5
  Vehicle mass [1000 kg]: 5000

================================================================
  Running simulation for 20 seconds (dt=0.5)
  Speed=8 m/s   TurnRate=15 deg/s   Mass=5000 kg
  Effective turn rate after mass adjustment: 9.26 deg/s
  GPS noise=2.5 m
================================================================

t=  0.500s  pos=(    4.123,    -0.456)  vel=(  7.892,   0.541)  hdg=  4.628 deg
t=  1.000s  pos=(    8.234,     0.123)  ...
...
```

If you enter an invalid value (out of range, not a number, etc.), the prompt
re-asks until you give a valid one.

### GUI Mode

Opens a startup configuration dialog where you can type values into text fields.
Each field validates as you type — invalid fields turn red with an error message
next to them. After clicking **START**, the live dashboard opens.

The main GUI has three on-screen buttons:

| Button   | Action                                        |
|----------|-----------------------------------------------|
| **START** | Begin simulation (becomes RESUME when paused) |
| **PAUSE** | Pause the running simulation                  |
| **RESET** | Rebuild the simulation from t=0               |

Plus keyboard shortcuts in the main GUI:

| Key      | Action                     |
|----------|----------------------------|
| `SPACE`  | Pause / resume simulation  |
| `R`      | Reset simulation to t=0    |
| `ESC`    | Close window and quit      |

In the input dialog:

| Key      | Action                              |
|----------|--------------------------------------|
| `TAB`    | Move to the next field              |
| `↑` / `↓` | Move between fields                 |
| `ENTER`  | Confirm and start                   |
| `ESC`    | Cancel and quit                     |

## Error Handling

All five parameters are validated in three places:

1. **At input time** (terminal prompts re-ask, GUI fields turn red)
2. **At config validation** (`SimConfig::validate()`) — throws with a descriptive message
3. **At Vehicle construction** — throws if mass or speed is somehow still invalid

If any validation fails, the program prints a clear error message naming the
parameter and the allowed range.

## How It Works

1. **Vehicle** advances its ground-truth state each timestep using a kinematic
   motion model (heading + velocity → position). Mass adjusts the effective
   turn rate.
2. **Sensors** observe the true state and add type-specific noise via their
   overridden `generateReading()` method.
3. **FusionSystem** collects every sensor's latest reading and computes a
   weighted-average `StateEstimate`.
4. The fused estimate is fed back to the **Vehicle** via `updateState()`.
5. In GUI mode, all four sensor readings and the fused estimate are pushed to
   the **Visualizer** which redraws the dashboard at 60 FPS.
6. In terminal mode, the same data is printed to the console.

## Project Goals

1. Demonstrate sensor fusion concepts through a practical simulation
2. Practice collaborative C++ development with Git and GitHub
3. Apply object-oriented design with inheritance, polymorphism, and validation
4. Provide both terminal and GUI interfaces for flexibility
5. Deliver a well-documented, fully functional, presentation-ready program
