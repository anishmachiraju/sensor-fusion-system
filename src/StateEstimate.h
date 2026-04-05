#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H

/**
 * @brief Lightweight struct representing the vehicle's state at a point in time.
 *
 * Used as the common data type exchanged between Sensor, FusionSystem, and Vehicle.
 */
struct StateEstimate {
    double positionX   = 0.0;  // X-coordinate (meters)
    double positionY   = 0.0;  // Y-coordinate (meters)
    double velocityX   = 0.0;  // Velocity in X direction (m/s)
    double velocityY   = 0.0;  // Velocity in Y direction (m/s)
    double heading     = 0.0;  // Heading angle (degrees)
    double temperature = 20.0; // Ambient temperature (°C)
    double timestamp   = 0.0;  // Simulation time (seconds)
};

#endif // STATE_ESTIMATE_H
