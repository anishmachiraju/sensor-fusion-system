# Sensor Fusion System for Autonomous Vehicle Simulation

**EECE 2140 — Professor Nafa | Spring 2026**

## Team Members

- **Jeongheon Han (David)** — Sensor class hierarchy (base class + all subclasses)
- **Charles Wan** — FusionSystem class (weighted-average fusion engine)
- **Anish Machiraju** — Vehicle class (kinematics, state history, display)

## Project Overview

This project simulates an autonomous vehicle equipped with four types of sensors — an Inertial Measurement Unit (IMU), a GPS receiver, a temperature sensor, and a wheel encoder. Each sensor generates noisy readings from the vehicle's true state. A weighted-average fusion algorithm combines these readings into a single, more accurate state estimate at every simulation timestep.

The program demonstrates core concepts in sensor fusion, object-oriented design, and collaborative C++ development.

## Main Functionalities

| Functionality | Description |
|---|---|
| **Sensor Simulation** | Generates realistic noisy readings with configurable noise sigma and bias for each sensor type |
| **Sensor Calibration** | Applies offset and scaling corrections to reduce systematic error |
| **Data Fusion** | Combines readings from all sensors using weighted averaging to produce an accurate fused estimate |
| **State Prediction** | Projects the current state forward in time using a kinematic motion model |
| **Vehicle State Update** | Overwrites the vehicle's internal state with the latest fused estimate |
| **Simulation Loop** | Orchestrates the full pipeline each timestep: step → sense → fuse → update → display |
| **Output Display** | Prints a formatted table of vehicle state at each timestep |
| **Exception Handling** | Validates inputs and guards against out-of-range values, null pointers, and missing data |

## OOP Design Summary

The system consists of **8 components** connected via polymorphism and a shared data contract:

- **`StateEstimate`** (struct) — Shared data type holding position, velocity, heading, temperature, and timestamp
- **`Sensor`** (base class) — Common interface for noise generation, calibration, and update-rate tracking
  - **`IMUSensor`** — Perturbs velocity and heading fields
  - **`GPSSensor`** — Perturbs position fields with signal dropout simulation
  - **`TemperatureSensor`** — Perturbs temperature with thermal response delay
  - **`WheelEncoderSensor`** — Converts wheel rotation to noisy velocity estimates
- **`FusionSystem`** — Registers sensors and computes weighted-average fusion
- **`Vehicle`** — Maintains ground-truth state, advances via kinematics, logs history

## Tools and Technologies

- **Language:** C++17
- **Build:** g++ (no external dependencies)
- **Version Control:** Git / GitHub
- **IDE:** VS Code
- **Documentation:** LaTeX (Overleaf), Google Workspace
- **AI Assistant:** Claude

## Folder Structure

```
sensor-fusion-system/
├── README.md
├── docs/
│   ├── System_Design_Overview.pdf
│   └── Iteration05_Pseudocode.pdf
├── pseudocode/
│   └── pseudocode.txt
├── src/
│   ├── main.cpp
│   ├── StateEstimate.h
│   ├── Sensor.h
│   ├── Sensor.cpp
│   ├── IMUSensor.h
│   ├── IMUSensor.cpp
│   ├── GPSSensor.h
│   ├── GPSSensor.cpp
│   ├── TemperatureSensor.h
│   ├── TemperatureSensor.cpp
│   ├── WheelEncoderSensor.h
│   ├── WheelEncoderSensor.cpp
│   ├── FusionSystem.h
│   ├── FusionSystem.cpp
│   ├── Vehicle.h
│   └── Vehicle.cpp
└── images/
    └── system_diagram.png
```

## How to Build and Run

```bash
# Clone the repository
git clone https://github.com/anishmachiraju/sensor-fusion-system.git
cd sensor-fusion-system

# Compile
g++ -std=c++17 -o simulation src/*.cpp

# Run
./simulation
```

## What Has Been Implemented

- [x] `StateEstimate` struct
- [x] `Sensor` base class with noise generation, calibration, and update-rate logic
- [x] `IMUSensor` subclass (acceleration + gyro noise)
- [x] `GPSSensor` subclass (position noise + signal dropout)
- [x] `TemperatureSensor` subclass (thermal noise + response delay)
- [x] `WheelEncoderSensor` subclass (tick-based velocity estimation)
- [x] `FusionSystem` class (weighted-average fusion)
- [x] `Vehicle` class (kinematics, history logging, display)
- [x] `main.cpp` simulation loop with exception handling
- [x] Full program compiles and runs end-to-end

## Project Goals

1. Demonstrate sensor fusion concepts in a practical C++ simulation
2. Practice collaborative software development using Git and GitHub
3. Apply object-oriented design with inheritance and polymorphism
4. Produce a well-documented, fully functional program
