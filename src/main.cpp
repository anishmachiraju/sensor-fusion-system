/**
 * Sensor Fusion System for Autonomous Vehicle Simulation
 * EECE 2140 - Professor Nafa
 *
 * Team: Jeongheon Han (David), Charles Wan, Anish Machiraju
 *
 * RUN MODES
 *   ./simulation                  -> GUI mode (input dialog + live dashboard)
 *   ./simulation --gui            -> same as above
 *   ./simulation --terminal       -> Terminal mode (text prompts + console output)
 *   ./simulation -t               -> short form of --terminal
 *   ./simulation --help           -> usage info
 *
 * Build:
 *   make           (uses Makefile, links SFML)
 *   ./simulation
 */

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <sstream>

#include "StateEstimate.h"
#include "IMUSensor.h"
#include "GPSSensor.h"
#include "TemperatureSensor.h"
#include "WheelEncoderSensor.h"
#include "FusionSystem.h"
#include "Vehicle.h"
#include "Visualizer.h"
#include "InputDialog.h"
#include "SimConfig.h"

// ---------------------------------------------------------------------------
// Simulation bundle - everything that must be rebuilt on a reset
// ---------------------------------------------------------------------------
struct Simulation {
    std::unique_ptr<Vehicle>            vehicle;
    std::unique_ptr<IMUSensor>          imu;
    std::unique_ptr<GPSSensor>          gps;
    std::unique_ptr<TemperatureSensor>  temp;
    std::unique_ptr<WheelEncoderSensor> encoder;
    std::unique_ptr<FusionSystem>       fusion;
    SimConfig                           config;
    double                              currentTime = 0.0;
};

// Build a fresh Simulation instance from a SimConfig.
static Simulation makeSimulation(const SimConfig& cfg) {
    Simulation sim;
    sim.config = cfg;

    StateEstimate initState;
    initState.positionX   = 0.0;
    initState.positionY   = 0.0;
    initState.velocityX   = cfg.speed;
    initState.velocityY   = 0.0;
    initState.heading     = 0.0;
    initState.temperature = 22.0;
    initState.timestamp   = 0.0;

    sim.vehicle = std::make_unique<Vehicle>(initState, cfg.speed,
                                            cfg.turnRate, cfg.mass);

    // Sensors. Note that GPS noise is now driven by the user.
    sim.imu     = std::make_unique<IMUSensor>(0.3, 0.5, 0.05, 0.02, 100.0);
    sim.gps     = std::make_unique<GPSSensor>(cfg.gpsNoise, 0.1, 0.05, 1.0);
    sim.temp    = std::make_unique<TemperatureSensor>(0.2, 0.1, 0.3, 1.0);
    sim.encoder = std::make_unique<WheelEncoderSensor>(0.5, 0.05, 0.3, 360, 50.0);

    sim.imu->calibrate(0.05, 1.0);
    sim.gps->calibrate(0.1,  1.0);
    sim.temp->calibrate(0.1, 1.0);
    sim.encoder->calibrate(0.05, 1.0);

    sim.fusion = std::make_unique<FusionSystem>();
    sim.fusion->addSensor(sim.imu.get(),     0.30);
    sim.fusion->addSensor(sim.gps.get(),     0.35);
    sim.fusion->addSensor(sim.temp.get(),    0.10);
    sim.fusion->addSensor(sim.encoder.get(), 0.25);

    sim.currentTime = 0.0;
    return sim;
}

// ---------------------------------------------------------------------------
// TERMINAL MODE
// ---------------------------------------------------------------------------

// Prompt for a single double value with default + range validation.
// Re-prompts on invalid input. Returns the validated value.
static double promptDouble(const std::string& label, double defaultVal,
                           double minVal, double maxVal,
                           const std::string& units) {
    while (true) {
        std::cout << "  " << label << " [" << defaultVal << units << "]: ";
        std::cout.flush();
        std::string line;
        if (!std::getline(std::cin, line)) {
            // EOF / Ctrl-D - use default and bail out gracefully
            std::cout << "\n  (using default " << defaultVal << units << ")\n";
            return defaultVal;
        }
        // Trim whitespace
        size_t start = line.find_first_not_of(" \t");
        size_t end   = line.find_last_not_of(" \t");
        if (start == std::string::npos) {
            return defaultVal;
        }
        line = line.substr(start, end - start + 1);

        try {
            double v = std::stod(line);
            if (v < minVal || v > maxVal) {
                std::cout << "  ! value out of range (" << minVal
                          << " - " << maxVal << "), please try again\n";
                continue;
            }
            return v;
        } catch (const std::exception&) {
            std::cout << "  ! not a valid number, please try again\n";
            continue;
        }
    }
}

static SimConfig promptForConfig() {
    SimConfig cfg;

    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "  Sensor Fusion System - Terminal Mode\n";
    std::cout << "================================================================\n";
    std::cout << "Enter simulation parameters (press Enter to keep default):\n\n";

    cfg.speed = promptDouble("Vehicle speed",  cfg.speed,
                             SimConfig::MIN_SPEED, SimConfig::MAX_SPEED, " m/s");
    cfg.turnRate = promptDouble("Turn rate",   cfg.turnRate,
                                SimConfig::MIN_TURN, SimConfig::MAX_TURN, " deg/s");
    cfg.duration = promptDouble("Duration (0 = infinite, will stop at 60s for terminal mode)",
                                30.0,
                                SimConfig::MIN_DURATION, SimConfig::MAX_DURATION, " s");
    cfg.gpsNoise = promptDouble("GPS noise sigma", cfg.gpsNoise,
                                SimConfig::MIN_GPS_NOISE, SimConfig::MAX_GPS_NOISE, " m");
    cfg.mass     = promptDouble("Vehicle mass", cfg.mass,
                                SimConfig::MIN_MASS, SimConfig::MAX_MASS, " kg");

    cfg.validate();  // throws if anything is somehow still invalid
    return cfg;
}

static int runTerminal() {
    SimConfig cfg;
    try {
        cfg = promptForConfig();
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] Invalid configuration: " << e.what() << "\n";
        return 1;
    }

    Simulation sim = makeSimulation(cfg);

    // Default duration in terminal mode is 30 s (or whatever the user gave)
    double duration = (cfg.duration > 0.0) ? cfg.duration : 30.0;
    const double dt = 0.5;

    std::cout << "\n================================================================\n";
    std::cout << "  Running simulation for " << duration << " seconds (dt=" << dt << ")\n";
    std::cout << "  Speed=" << cfg.speed << " m/s   TurnRate=" << cfg.turnRate
              << " deg/s   Mass=" << cfg.mass << " kg\n";
    std::cout << "  Effective turn rate after mass adjustment: "
              << std::fixed << std::setprecision(2)
              << sim.vehicle->getEffectiveTurnRate() << " deg/s\n";
    std::cout << "  GPS noise=" << cfg.gpsNoise << " m\n";
    std::cout << "================================================================\n\n";

    while (sim.currentTime < duration) {
        try {
            sim.vehicle->stepSimulation(dt);
            StateEstimate trueState = sim.vehicle->getState();

            if (sim.imu->shouldUpdate(sim.currentTime))
                sim.imu->generateReading(trueState);
            if (sim.gps->shouldUpdate(sim.currentTime))
                sim.gps->generateReading(trueState);
            if (sim.temp->shouldUpdate(sim.currentTime))
                sim.temp->generateReading(trueState);
            if (sim.encoder->shouldUpdate(sim.currentTime))
                sim.encoder->generateReading(trueState);

            StateEstimate fused = sim.fusion->fuseData();
            sim.vehicle->updateState(fused);
            sim.vehicle->displayState();

        } catch (const std::exception& e) {
            std::cerr << "[ERROR] at t=" << sim.currentTime
                      << ": " << e.what() << std::endl;
        }
        sim.currentTime += dt;
    }

    std::cout << "\n================================================================\n";
    std::cout << "  Simulation complete. Total timesteps: "
              << sim.vehicle->getHistory().size() << "\n";
    std::cout << "================================================================\n";
    return 0;
}

// ---------------------------------------------------------------------------
// GUI MODE
// ---------------------------------------------------------------------------

static int runGui() {
    SimConfig cfg;

    // Step 1: show input dialog
    {
        InputDialog dialog;
        if (!dialog.show(cfg)) {
            std::cout << "Configuration cancelled. Exiting.\n";
            return 0;
        }
    }

    try {
        cfg.validate();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Invalid configuration: " << e.what() << "\n";
        return 1;
    }

    std::cout << "================================================================\n";
    std::cout << "  Launching GUI with:\n";
    std::cout << "    Speed=" << cfg.speed << " m/s\n";
    std::cout << "    Turn Rate=" << cfg.turnRate << " deg/s\n";
    std::cout << "    Duration=" << cfg.duration << " s\n";
    std::cout << "    GPS Noise=" << cfg.gpsNoise << " m\n";
    std::cout << "    Mass=" << cfg.mass << " kg\n";
    std::cout << "================================================================\n";

    Simulation sim = makeSimulation(cfg);
    Visualizer viz;

    const double dt = 0.1;
    auto lastFrameTime = std::chrono::steady_clock::now();

    while (viz.isOpen()) {
        viz.handleEvents();

        if (viz.consumeResetRequest()) {
            sim = makeSimulation(cfg);
            viz.clearForReset();
            std::cout << "[Reset] Simulation rebuilt.\n";
        }

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFrameTime).count();

        bool stopByDuration = (cfg.duration > 0.0 && sim.currentTime >= cfg.duration);

        if (viz.isRunning() && !stopByDuration && elapsed >= 0.05) {
            lastFrameTime = now;

            try {
                sim.vehicle->stepSimulation(dt);
                StateEstimate trueState = sim.vehicle->getState();

                if (sim.imu->shouldUpdate(sim.currentTime))
                    sim.imu->generateReading(trueState);
                if (sim.gps->shouldUpdate(sim.currentTime))
                    sim.gps->generateReading(trueState);
                if (sim.temp->shouldUpdate(sim.currentTime))
                    sim.temp->generateReading(trueState);
                if (sim.encoder->shouldUpdate(sim.currentTime))
                    sim.encoder->generateReading(trueState);

                StateEstimate fused = sim.fusion->fuseData();
                sim.vehicle->updateState(fused);

                viz.update(sim.vehicle->getState(),
                           sim.imu->getLastReading(),
                           sim.gps->getLastReading(),
                           sim.temp->getLastReading(),
                           sim.encoder->getLastReading(),
                           fused);

            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Fusion failed at t=" << sim.currentTime
                          << ": " << e.what() << std::endl;
            }

            sim.currentTime += dt;
        }

        viz.render();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\nGUI closed. Final simulation time: "
              << sim.currentTime << " s\n";
    return 0;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static void printHelp() {
    std::cout <<
        "Sensor Fusion System - Autonomous Vehicle Simulation\n"
        "\n"
        "Usage:\n"
        "  ./simulation              Launch GUI (input dialog + live dashboard)\n"
        "  ./simulation --gui        Same as above\n"
        "  ./simulation --terminal   Run in terminal mode (text prompts + console)\n"
        "  ./simulation -t           Short form of --terminal\n"
        "  ./simulation --help       Show this help message\n"
        "\n"
        "Configurable parameters (validated in both modes):\n"
        "  Vehicle speed       0.1 -    50.0 m/s     default 5.0\n"
        "  Turn rate         -90.0 -    90.0 deg/s   default 10.0\n"
        "  Duration            0.0 -  3600.0 s       default 30 (0 = infinite GUI)\n"
        "  GPS noise sigma     0.0 -    20.0 m       default 1.5\n"
        "  Vehicle mass      100.0 - 20000.0 kg      default 1000\n";
}

int main(int argc, char* argv[]) {
    bool terminalMode = false;
    bool guiMode      = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--terminal" || arg == "-t") {
            terminalMode = true;
        } else if (arg == "--gui" || arg == "-g") {
            guiMode = true;
        } else if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::cerr << "Use --help for usage.\n";
            return 1;
        }
    }

    if (terminalMode && guiMode) {
        std::cerr << "Cannot use both --terminal and --gui at the same time.\n";
        return 1;
    }

    try {
        if (terminalMode) {
            return runTerminal();
        } else {
            return runGui();
        }
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
