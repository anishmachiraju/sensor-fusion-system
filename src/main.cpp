/**
 * Sensor Fusion System for Autonomous Vehicle Simulation
 * EECE 2140 — Professor Nafa
 *
 * Team: Jeongheon Han (David), Charles Wan, Anish Machiraju
 *
 * This program simulates an autonomous vehicle equipped with
 * IMU, GPS, Temperature, and Wheel Encoder sensors. Noisy readings
 * from each sensor are fused via a weighted-average algorithm to
 * produce an accurate state estimate at every timestep.
 */

#include <iostream>
#include <iomanip>
#include <stdexcept>

#include "StateEstimate.h"
#include "IMUSensor.h"
#include "GPSSensor.h"
#include "TemperatureSensor.h"
#include "WheelEncoderSensor.h"
#include "FusionSystem.h"
#include "Vehicle.h"

int main() {
    // ---- Configuration ----
    double duration = 10.0;   // total simulation time (seconds)
    double dt       = 0.5;    // timestep size (seconds)

    // ---- Initial vehicle state ----
    StateEstimate initState;
    initState.positionX   = 0.0;
    initState.positionY   = 0.0;
    initState.velocityX   = 5.0;
    initState.velocityY   = 0.0;
    initState.heading     = 0.0;
    initState.temperature = 22.0;
    initState.timestamp   = 0.0;

    // ---- Create vehicle ----
    double vehicleSpeed    = 5.0;   // m/s
    double vehicleTurnRate = 10.0;  // deg/s
    Vehicle vehicle(initState, vehicleSpeed, vehicleTurnRate);

    // ---- Create sensors ----
    //                       accelSigma  gyroSigma  accelBias  gyroBias  rate(Hz)
    IMUSensor imu(             0.3,        0.5,       0.05,      0.02,    100.0);

    //                       posSigma  posBias  dropout  rate(Hz)
    GPSSensor gps(             1.5,      0.1,    0.05,    1.0);

    //                       sigma  bias   delay  rate(Hz)
    TemperatureSensor temp(    0.2,   0.1,   0.3,   1.0);

    //                            sigma  bias   radius  ticks  rate(Hz)
    WheelEncoderSensor encoder(    0.5,   0.05,  0.3,    360,   50.0);

    // ---- Calibrate sensors ----
    imu.calibrate(0.05, 1.0);
    gps.calibrate(0.1,  1.0);
    temp.calibrate(0.1, 1.0);
    encoder.calibrate(0.05, 1.0);

    // ---- Set up fusion system ----
    FusionSystem fusion;
    fusion.addSensor(&imu,     0.30);
    fusion.addSensor(&gps,     0.35);
    fusion.addSensor(&temp,    0.10);
    fusion.addSensor(&encoder, 0.25);

    // ---- Print header ----
    std::cout << "============================================================"
              << "============================\n";
    std::cout << "  Sensor Fusion System — Autonomous Vehicle Simulation\n";
    std::cout << "  Duration: " << duration << "s   Timestep: " << dt << "s\n";
    std::cout << "============================================================"
              << "============================\n\n";

    // ---- Simulation loop ----
    double currentTime = 0.0;
    while (currentTime < duration) {
        // Step 1: Advance ground-truth vehicle state
        vehicle.stepSimulation(dt);
        StateEstimate trueState = vehicle.getState();

        // Step 2: Generate sensor readings (if update rate allows)
        if (imu.shouldUpdate(currentTime)) {
            imu.generateReading(trueState);
        }
        if (gps.shouldUpdate(currentTime)) {
            gps.generateReading(trueState);
        }
        if (temp.shouldUpdate(currentTime)) {
            temp.generateReading(trueState);
        }
        if (encoder.shouldUpdate(currentTime)) {
            encoder.generateReading(trueState);
        }

        // Step 3: Fuse all latest readings
        try {
            StateEstimate fusedEstimate = fusion.fuseData();

            // Step 4: Update vehicle with fused estimate
            vehicle.updateState(fusedEstimate);

            // Step 5: Display
            vehicle.displayState();

        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Fusion failed at t=" << currentTime
                      << ": " << e.what() << std::endl;
        }

        currentTime += dt;
    }

    // ---- Summary ----
    std::cout << "\n============================================================"
              << "============================\n";
    std::cout << "  Simulation complete. Total timesteps: "
              << vehicle.getHistory().size() << std::endl;
    std::cout << "============================================================"
              << "============================\n";

    return 0;
}
