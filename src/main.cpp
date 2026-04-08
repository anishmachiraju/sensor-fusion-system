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
 *
 * Interactive GUI dashboard built with SFML shows:
 *   - A live 2D map with the vehicle, heading arrow, and trajectory
 *   - Real-time data panels for all four sensors
 *   - The fused state estimate panel
 *
 * Build:
 *   g++ -std=c++17 src/ALL.cpp -o simulation \
 *       -lsfml-graphics -lsfml-window -lsfml-system
 */

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <chrono>
#include <thread>

#include "StateEstimate.h"
#include "IMUSensor.h"
#include "GPSSensor.h"
#include "TemperatureSensor.h"
#include "WheelEncoderSensor.h"
#include "FusionSystem.h"
#include "Vehicle.h"
#include "Visualizer.h"

int main() {
    // ---- Configuration ----
    double dt = 0.1;   // timestep size (seconds) - smaller for smooth animation

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
    IMUSensor          imu(   0.3,  0.5,  0.05, 0.02, 100.0);
    GPSSensor          gps(   1.5,  0.1,  0.05, 1.0);
    TemperatureSensor  temp(  0.2,  0.1,  0.3,  1.0);
    WheelEncoderSensor encoder(0.5, 0.05, 0.3, 360, 50.0);

    // ---- Calibrate sensors ----
    imu.calibrate(0.05, 1.0);
    gps.calibrate(0.1,  1.0);
    temp.calibrate(0.1, 1.0);
    encoder.calibrate(0.05, 1.0);

    // ---- Fusion system ----
    FusionSystem fusion;
    fusion.addSensor(&imu,     0.30);
    fusion.addSensor(&gps,     0.35);
    fusion.addSensor(&temp,    0.10);
    fusion.addSensor(&encoder, 0.25);

    // ---- Launch GUI ----
    Visualizer viz;

    std::cout << "========================================================\n";
    std::cout << "  Sensor Fusion System — Autonomous Vehicle Simulation\n";
    std::cout << "  GUI launched. Controls:\n";
    std::cout << "    SPACE  pause / resume\n";
    std::cout << "    R      clear trajectory trail\n";
    std::cout << "    ESC    quit\n";
    std::cout << "========================================================\n";

    // ---- Main loop: runs until window closes ----
    double currentTime = 0.0;
    auto lastFrameTime = std::chrono::steady_clock::now();

    while (viz.isOpen()) {
        viz.handleEvents();

        // Throttle simulation to real-time (~100ms per step)
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFrameTime).count();

        if (!viz.isPaused() && elapsed >= 0.05) {
            lastFrameTime = now;

            // Step 1: Advance ground-truth vehicle state
            vehicle.stepSimulation(dt);
            StateEstimate trueState = vehicle.getState();

            // Step 2: Generate sensor readings (throttled by update rate)
            if (imu.shouldUpdate(currentTime))     imu.generateReading(trueState);
            if (gps.shouldUpdate(currentTime))     gps.generateReading(trueState);
            if (temp.shouldUpdate(currentTime))    temp.generateReading(trueState);
            if (encoder.shouldUpdate(currentTime)) encoder.generateReading(trueState);

            // Step 3: Fuse
            try {
                StateEstimate fusedEstimate = fusion.fuseData();
                vehicle.updateState(fusedEstimate);

                // Step 4: Push data into the visualizer
                viz.update(vehicle.getState(),
                           imu.getLastReading(),
                           gps.getLastReading(),
                           temp.getLastReading(),
                           encoder.getLastReading(),
                           fusedEstimate);

                // Also log to console
                std::cout << std::fixed << std::setprecision(2)
                          << "t=" << std::setw(6) << currentTime
                          << "s  pos=(" << std::setw(7) << fusedEstimate.positionX
                          << ", "       << std::setw(7) << fusedEstimate.positionY
                          << ")  hdg="  << std::setw(6) << fusedEstimate.heading
                          << "deg\n";

            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Fusion failed at t=" << currentTime
                          << ": " << e.what() << std::endl;
            }

            currentTime += dt;
        }

        // Always render (even when paused, so UI stays responsive)
        viz.render();

        // Sleep briefly so we don't peg the CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\nSimulation window closed. Final time: "
              << currentTime << "s\n";
    return 0;
}
