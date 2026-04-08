#ifndef VEHICLE_H
#define VEHICLE_H

#include "StateEstimate.h"
#include <vector>

/**
 * @brief Represents the autonomous vehicle being simulated.
 *
 * Maintains the ground-truth state, advances it via kinematics,
 * accepts fused estimates, and logs/displays state history.
 *
 * The vehicle's mass affects its turning agility: heavier vehicles
 * turn more slowly (higher rotational inertia), lighter vehicles turn
 * more quickly. The effective turn rate is computed from the user's
 * commanded turn rate and the mass via:
 *   effectiveTurnRate = commandedTurnRate * (1000 / mass)^0.3
 *
 * Anish Machiraju — primary author.
 */
class Vehicle {
private:
    StateEstimate              state;
    std::vector<StateEstimate> stateHistory;
    double                     speed;             // scalar speed (m/s)
    double                     commandedTurnRate; // deg/s as set by the user
    double                     effectiveTurnRate; // deg/s after mass adjustment
    double                     mass;              // kg

    void recomputeEffectiveTurnRate();

public:
    Vehicle(const StateEstimate& initState,
            double speed = 5.0,
            double turnRate = 5.0,
            double mass = 1000.0);

    void          updateState(const StateEstimate& fused);
    void          stepSimulation(double dt);
    StateEstimate getState() const;
    std::vector<StateEstimate> getHistory() const;
    void          displayState() const;
    StateEstimate predict(double dt) const;

    // Accessors for the GUI / reporting
    double getSpeed()              const { return speed; }
    double getCommandedTurnRate()  const { return commandedTurnRate; }
    double getEffectiveTurnRate()  const { return effectiveTurnRate; }
    double getMass()               const { return mass; }
};

#endif // VEHICLE_H
