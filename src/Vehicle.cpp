#include "Vehicle.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>

Vehicle::Vehicle(const StateEstimate& initState, double spd, double tr)
    : state(initState), speed(spd), turnRate(tr)
{
    stateHistory.push_back(state);
}

void Vehicle::stepSimulation(double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("Timestep dt must be > 0");
    }

    // Update heading
    state.heading += turnRate * dt;

    // Wrap heading into [0, 360)
    while (state.heading >= 360.0) state.heading -= 360.0;
    while (state.heading <    0.0) state.heading += 360.0;

    // Convert heading to radians
    double headingRad = state.heading * M_PI / 180.0;

    // Update velocity components from speed and heading
    state.velocityX = speed * std::cos(headingRad);
    state.velocityY = speed * std::sin(headingRad);

    // Update position
    state.positionX += state.velocityX * dt;
    state.positionY += state.velocityY * dt;

    // Advance timestamp
    state.timestamp += dt;

    // Log to history
    stateHistory.push_back(state);
}

StateEstimate Vehicle::predict(double dt) const {
    if (dt <= 0.0) {
        throw std::invalid_argument("Prediction dt must be > 0");
    }

    StateEstimate predicted = state;

    predicted.heading += turnRate * dt;
    while (predicted.heading >= 360.0) predicted.heading -= 360.0;
    while (predicted.heading <    0.0) predicted.heading += 360.0;

    double headingRad = predicted.heading * M_PI / 180.0;
    predicted.velocityX = speed * std::cos(headingRad);
    predicted.velocityY = speed * std::sin(headingRad);
    predicted.positionX += predicted.velocityX * dt;
    predicted.positionY += predicted.velocityY * dt;
    predicted.timestamp += dt;

    return predicted;
}

void Vehicle::updateState(const StateEstimate& fused) {
    state = fused;
}

StateEstimate Vehicle::getState() const {
    return state;
}

std::vector<StateEstimate> Vehicle::getHistory() const {
    return stateHistory;
}

void Vehicle::displayState() const {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "t=" << std::setw(7) << state.timestamp << "s"
              << "  pos=(" << std::setw(9) << state.positionX
              << ", "      << std::setw(9) << state.positionY << ")"
              << "  vel=(" << std::setw(7) << state.velocityX
              << ", "      << std::setw(7) << state.velocityY << ")"
              << "  hdg="  << std::setw(7) << state.heading << "deg"
              << "  temp=" << std::setw(6) << state.temperature << "C"
              << std::endl;
}
