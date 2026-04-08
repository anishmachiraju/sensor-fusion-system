/**
 * Sensor Fusion System for Autonomous Vehicle Simulation
 * EECE 2140 - Professor Nafa
 *
 * Team: Jeongheon Han (David), Charles Wan, Anish Machiraju
 *
 * Build:
 *   make           (uses Makefile)
 *   ./simulation
 */

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <memory>

#include "StateEstimate.h"
#include "IMUSensor.h"
#include "GPSSensor.h"
#include "TemperatureSensor.h"
#include "WheelEncoderSensor.h"
#include "FusionSystem.h"
#include "Vehicle.h"
#include "Visualizer.h"

// ---------------------------------------------------------------------------
// Simulation bundle — everything that needs to be rebuilt on a reset
// ---------------------------------------------------------------------------
struct Simulation {
    std::unique_ptr<Vehicle>            vehicle;
    std::unique_ptr<IMUSensor>          imu;
    std::unique_ptr<GPSSensor>          gps;
    std::unique_ptr<TemperatureSensor>  temp;
    std::unique_ptr<WheelEncoderSensor> encoder;
    std::unique_ptr<FusionSystem>       fusion;
    double currentTime = 0.0;
};

// Build (or rebuild) a fresh Simulation instance from default parameters
static Simulation makeSimulation() {
    Simulation sim;

    // Initial vehicle state
    StateEstimate initState;
    initState.positionX   = 0.0;
    initState.positionY   = 0.0;
    initState.velocityX   = 5.0;
    initState.velocityY   = 0.0;
    initState.heading     = 0.0;
    initState.temperature = 22.0;
    initState.timestamp   = 0.0;

    // Vehicle parameters
    const double vehicleSpeed    = 5.0;   // m/s
    const double vehicleTurnRate = 10.0;  // deg/s

    sim.vehicle = std::make_unique<Vehicle>(initState, vehicleSpeed, vehicleTurnRate);

    // Sensors
    sim.imu     = std::make_unique<IMUSensor>(0.3, 0.5, 0.05, 0.02, 100.0);
    sim.gps     = std::make_unique<GPSSensor>(1.5, 0.1, 0.05, 1.0);
    sim.temp    = std::make_unique<TemperatureSensor>(0.2, 0.1, 0.3, 1.0);
    sim.encoder = std::make_unique<WheelEncoderSensor>(0.5, 0.05, 0.3, 360, 50.0);

    // Calibrate
    sim.imu->calibrate(0.05, 1.0);
    sim.gps->calibrate(0.1,  1.0);
    sim.temp->calibrate(0.1, 1.0);
    sim.encoder->calibrate(0.05, 1.0);

    // Fusion
    sim.fusion = std::make_unique<FusionSystem>();
    sim.fusion->addSensor(sim.imu.get(),     0.30);
    sim.fusion->addSensor(sim.gps.get(),     0.35);
    sim.fusion->addSensor(sim.temp.get(),    0.10);
    sim.fusion->addSensor(sim.encoder.get(), 0.25);

    sim.currentTime = 0.0;
    return sim;
}

int main() {
    const double dt = 0.1;  // simulation timestep (seconds)

    // Build initial simulation and launch GUI
    Simulation sim = makeSimulation();
    Visualizer viz;

    std::cout << "========================================================\n";
    std::cout << "  Sensor Fusion System - Autonomous Vehicle Simulation\n";
    std::cout << "  GUI launched. Click START to begin.\n";
    std::cout << "  Keyboard:  SPACE=pause/resume  R=reset  ESC=quit\n";
    std::cout << "========================================================\n";

    auto lastFrameTime = std::chrono::steady_clock::now();

    while (viz.isOpen()) {
        viz.handleEvents();

        // Handle reset request from the GUI (either button or keyboard)
        if (viz.consumeResetRequest()) {
            sim = makeSimulation();
            viz.clearForReset();
            std::cout << "[Reset] Simulation rebuilt from initial conditions.\n";
        }

        // Advance simulation only while Running
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFrameTime).count();

        if (viz.isRunning() && elapsed >= 0.05) {
            lastFrameTime = now;

            // 1. Advance ground truth
            sim.vehicle->stepSimulation(dt);
            StateEstimate trueState = sim.vehicle->getState();

            // 2. Generate sensor readings (throttled by update rate)
            if (sim.imu->shouldUpdate(sim.currentTime))
                sim.imu->generateReading(trueState);
            if (sim.gps->shouldUpdate(sim.currentTime))
                sim.gps->generateReading(trueState);
            if (sim.temp->shouldUpdate(sim.currentTime))
                sim.temp->generateReading(trueState);
            if (sim.encoder->shouldUpdate(sim.currentTime))
                sim.encoder->generateReading(trueState);

            // 3. Fuse, update, and push to GUI
            try {
                StateEstimate fusedEstimate = sim.fusion->fuseData();
                sim.vehicle->updateState(fusedEstimate);

                viz.update(sim.vehicle->getState(),
                           sim.imu->getLastReading(),
                           sim.gps->getLastReading(),
                           sim.temp->getLastReading(),
                           sim.encoder->getLastReading(),
                           fusedEstimate);

                std::cout << std::fixed << std::setprecision(2)
                          << "t=" << std::setw(6) << sim.currentTime
                          << "s  pos=(" << std::setw(7) << fusedEstimate.positionX
                          << ", "       << std::setw(7) << fusedEstimate.positionY
                          << ")  hdg="  << std::setw(6) << fusedEstimate.heading
                          << "deg\n";

            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Fusion failed at t=" << sim.currentTime
                          << ": " << e.what() << std::endl;
            }

            sim.currentTime += dt;
        }

        // Always render (so buttons stay responsive even when Stopped/Paused)
        viz.render();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\nSimulation window closed.\n";
    return 0;
}
