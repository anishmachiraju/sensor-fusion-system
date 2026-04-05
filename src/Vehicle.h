#ifndef VEHICLE_H
#define VEHICLE_H

#include "StateEstimate.h"
#include <vector>

/**
 * @brief Represents the autonomous vehicle being simulated.
 *
 * Maintains the ground-truth state, advances it via kinematics,
 * accepts fused estimates, and logs/displays state history.
 * Anish Machiraju — primary author.
 */
class Vehicle {
private:
    StateEstimate              state;
    std::vector<StateEstimate> stateHistory;
    double                     speed;     // scalar speed (m/s)
    double                     turnRate;  // heading change rate (deg/s)

public:
    Vehicle(const StateEstimate& initState, double speed = 5.0,
            double turnRate = 5.0);

    void          updateState(const StateEstimate& fused);
    void          stepSimulation(double dt);
    StateEstimate getState() const;
    std::vector<StateEstimate> getHistory() const;
    void          displayState() const;
    StateEstimate predict(double dt) const;
};

#endif // VEHICLE_H
