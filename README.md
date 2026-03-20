# System Design Overview
### Sensor Fusion System for Autonomous Vehicle Simulation

**Jeongheon Han (David), Charles Wan, Anish Machiraju**
EECE 2140 — Professor Nafa | March 2026

---

## GitHub Repository

[https://github.com/anishmachiraju/sensor-fusion-system](https://github.com/anishmachiraju/sensor-fusion-system)

---

## 1. Basic Functionalities

The following table lists the fundamental functionalities of the sensor fusion system. Each functionality is described alongside its expected inputs, outputs, and contribution to the overall system.

| Functionality | Description | Input | Output |
|---|---|---|---|
| **Sensor Simulation** | Simulates IMU, GPS, and temperature sensors by generating realistic readings with configurable noise and bias. | Simulation timestep, noise parameters | Raw sensor reading (position, acceleration, orientation, or temperature) |
| **Sensor Calibration** | Applies calibration offsets and scaling factors to raw sensor readings to reduce systematic error. | Raw sensor reading, calibration parameters | Calibrated sensor reading |
| **Data Fusion** | Combines calibrated readings from multiple heterogeneous sensors into a single, more accurate state estimate using a weighted-average fusion algorithm. | Vector of calibrated sensor readings, sensor weights | Fused state estimate (position, velocity, heading, temperature) |
| **State Prediction** | Uses the fused estimate and a simple kinematic model to predict the vehicle's state at the next timestep. | Current fused state, time delta | Predicted state estimate |
| **Vehicle State Update** | Updates the vehicle's internal state (position, velocity, heading, temperature) using the latest fused or predicted estimate. | Fused / predicted state estimate | Updated vehicle state |
| **Simulation Loop** | Orchestrates the end-to-end pipeline: generates sensor data, fuses it, updates the vehicle, and logs each timestep. | Simulation duration, timestep size, initial conditions | Timestep-by-timestep log of vehicle states |
| **Output Display** | Prints or logs the simulation results in a readable, formatted table to the console. | Vector of logged vehicle states | Formatted console output |
| **Exception Handling** | Validates inputs and sensor readings; reports meaningful error messages for out-of-range values, missing data, or configuration errors. | Any invalid or missing data | Error message or graceful recovery |

---

## 2. Object-Oriented Design (OOP Structure)

The system is organized into three core classes — `Sensor`, `FusionSystem`, and `Vehicle` — plus a lightweight `StateEstimate` struct that serves as the common data type exchanged between components.

### 2.1 `StateEstimate` (struct)

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `positionX` | public | `double` | X-coordinate of the vehicle (meters) |
| `positionY` | public | `double` | Y-coordinate of the vehicle (meters) |
| `velocityX` | public | `double` | Velocity in the X direction (m/s) |
| `velocityY` | public | `double` | Velocity in the Y direction (m/s) |
| `heading` | public | `double` | Heading angle (degrees) |
| `temperature` | public | `double` | Ambient temperature (°C) |
| `timestamp` | public | `double` | Simulation time (seconds) |

### 2.2 `Sensor` class *(David)*

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `sensorType` | private | `double` | Type identifier |
| `noiseSigma` | private | `double` | Standard deviation of Gaussian noise |
| `biasOffset` | private | `double` | Systematic bias added to readings |
| `lastReading` | private | `StateEstimate` | Most recent reading produced |
| `isCalibrated` | private | `bool` | Whether calibration has been applied |
| `updateRate` | protected | `double` | Frequency (Hz) at which the sensor generates readings |
| `lastUpdateTime` | protected | `double` | Last simulation time the sensor produced a reading |
| `Sensor(type, sigma, bias, rate)` | public | constructor | Initializes sensor with type and noise parameters |
| `generateReading(trueState)` | public | `StateEstimate` | Produces a noisy reading from the true vehicle state |
| `shouldUpdate(currentTime)` | public | `bool` | Determines if enough time has passed to generate a new reading based on update rate |
| `calibrate(offset, scale)` | public | `void` | Sets calibration offset and scaling factor |
| `getLastReading()` | public | `StateEstimate` | Returns the most recent sensor reading |
| `getType()` | public | `string` | Returns the sensor type identifier |

### 2.3 `IMUSensor` subclass

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `accelNoiseSigma` | private | `double` | Standard deviation of noise applied to acceleration readings |
| `gyroNoiseSigma` | private | `double` | Standard deviation of noise applied to angular velocity / heading |
| `accelBias` | private | `double` | Systematic bias in acceleration measurements |
| `gyroBias` | private | `double` | Systematic bias in rotational measurements |
| `IMUSensor(sigma, bias, rate)` | public | constructor | Initializes IMU sensor with noise and bias parameters |
| `generateReading(trueState)` | public | `StateEstimate` | Produces noisy acceleration and orientation data from true state |
| `calibrate(offset, scale)` | public | `void` | Applies calibration to acceleration and gyro measurements |
| `getType()` | public | `string` | Returns the sensor type identifier ("IMU") |

### 2.4 `GPSSensor` subclass

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `positionNoiseSigma` | private | `double` | Standard deviation of noise applied to position readings |
| `positionBias` | private | `double` | Systematic offset in position measurements |
| `dropoutRate` | private | `double` | Probability of missing a reading to simulate signal loss |
| `GPSSensor(sigma, bias, rate)` | public | constructor | Initializes GPS sensor with noise and bias parameters |
| `generateReading(trueState)` | public | `StateEstimate` | Produces noisy position data from true vehicle state |
| `calibrate(offset, scale)` | public | `void` | Applies offset correction to position measurements |
| `getType()` | public | `string` | Returns the sensor type identifier ("GPS") |

### 2.5 `TemperatureSensor` subclass

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `tempNoiseSigma` | private | `double` | Standard deviation of noise applied to temperature readings |
| `tempBias` | private | `double` | Systematic offset in temperature measurement |
| `responseDelay` | private | `double` | Simulates slow response to temperature changes |
| `TemperatureSensor(sigma, bias, rate)` | public | constructor | Initializes temperature sensor with noise and bias parameters |
| `generateReading(trueState)` | public | `StateEstimate` | Produces noisy temperature reading from true state |
| `calibrate(offset, scale)` | public | `void` | Applies offset and scaling to temperature measurements |
| `getType()` | public | `string` | Returns the sensor type identifier ("Temperature") |

### 2.6 `WheelEncoderSensor` subclass

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `tickNoiseSigma` | private | `double` | Standard deviation of noise in wheel rotation measurements |
| `tickBias` | private | `double` | Systematic bias in encoder tick counts |
| `wheelRadius` | private | `double` | Radius of the wheel used to convert rotation to distance |
| `ticksPerRevolution` | private | `int` | Number of encoder ticks per full wheel rotation |
| `WheelEncoderSensor(sigma, bias, radius, ticks)` | public | constructor | Initializes encoder with noise, bias, and wheel parameters |
| `generateReading(trueState)` | public | `StateEstimate` | Produces noisy distance or velocity estimate based on wheel rotation |
| `calibrate(offset, scale)` | public | `void` | Applies calibration to encoder measurements |
| `getType()` | public | `string` | Returns the sensor type identifier ("WheelEncoder") |

### 2.7 `FusionSystem` class *(Charles)*

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `sensors` | private | `vector<Sensor*>` | Pointers to all registered sensors |
| `weights` | private | `vector<double>` | Weight assigned to each sensor |
| `fusedState` | private | `StateEstimate` | Latest fused state estimate |
| `predictionDt` | private | `double` | Default prediction timestep (seconds) |
| `FusionSystem()` | public | constructor | Initializes with empty sensor list |
| `addSensor(sensor, weight)` | public | `void` | Registers a sensor with a fusion weight |
| `fuseData()` | public | `StateEstimate` | Computes weighted-average fusion of all sensor readings |
| `getFusedState()` | public | `StateEstimate` | Returns the latest fused state |

### 2.8 `Vehicle` class *(Anish)*

| Attribute / Method | Access | Data Type | Description |
|---|---|---|---|
| `state` | private | `StateEstimate` | Current true state of the vehicle |
| `stateHistory` | private | `vector<StateEstimate>` | Log of all past states |
| `speed` | private | `double` | Scalar speed (m/s) |
| `turnRate` | private | `double` | Rate of heading change (deg/s) |
| `Vehicle(initState)` | public | constructor | Initializes vehicle at a given state |
| `updateState(fused)` | public | `void` | Overwrites internal state with fused estimate |
| `stepSimulation(dt)` | public | `void` | Advances true state by one timestep using kinematics |
| `getState()` | public | `StateEstimate` | Returns the current vehicle state |
| `getHistory()` | public | `vector<StateEstimate>` | Returns full state history |
| `displayState()` | public | `void` | Prints the current state to console |
| `predict(dt)` | public | `StateEstimate` | Predicts the next state using kinematics |

---

## 3. System Overview Diagram

The diagram below illustrates the high-level **Input → Process → Output** pipeline of the sensor fusion system.

```
┌─────────────────────┐     ┌──────────────────────────┐     ┌─────────────────────────┐
│       INPUT          │     │        PROCESS            │     │        OUTPUT            │
├─────────────────────┤     ├──────────────────────────┤     ├─────────────────────────┤
│                     │     │                          │     │                         │
│  IMU Sensor Data    │────▶│  Generate Noisy Sensor   │     │  Fused State Estimate   │
│  (accel, orient.)   │     │  Readings                │     │  (pos, vel, heading,    │
│                     │     │         │                │     │   temp)                 │
│  GPS Sensor Data    │────▶│         ▼                │     │                         │
│  (position)         │     │  Calibrate Readings      │     │  State History Log      │
│                     │     │         │                │     │                         │
│  Temperature Sensor │────▶│         ▼                │────▶│  Formatted Console      │
│  (ambient temp.)    │     │  Weighted-Average        │     │  Output                 │
│                     │     │  Data Fusion             │     │                         │
│  Configuration      │────▶│         │                │     │  Error Messages         │
│  (noise, timestep,  │     │         ▼                │     │  (if exceptions occur)  │
│   duration)         │     │  State Prediction        │     │                         │
│                     │     │  (kinematics)            │     │                         │
│                     │     │         │                │     │                         │
│                     │     │         ▼                │     │                         │
│                     │     │  Update Vehicle State    │     │                         │
└─────────────────────┘     └──────────────────────────┘     └─────────────────────────┘
```

**Summary.** The simulation begins by reading configuration parameters and generating noisy sensor readings from the IMU, GPS, and temperature sensors (Input). The Fusion System calibrates these readings, computes a weighted-average fused estimate, and predicts the next state (Process). Finally, the Vehicle updates its internal state and the system logs and displays results (Output). Exception handling guards every stage against invalid or out-of-range data.
