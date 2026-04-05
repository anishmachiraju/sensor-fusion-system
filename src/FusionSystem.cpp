#include "FusionSystem.h"
#include <stdexcept>

FusionSystem::FusionSystem()
    : predictionDt(0.1)
{
}

void FusionSystem::addSensor(Sensor* sensor, double weight) {
    if (sensor == nullptr) {
        throw std::invalid_argument("Cannot add a null sensor");
    }
    if (weight <= 0.0) {
        throw std::invalid_argument("Sensor weight must be > 0");
    }
    sensors.push_back(sensor);
    weights.push_back(weight);
}

StateEstimate FusionSystem::fuseData() {
    if (sensors.empty()) {
        throw std::runtime_error("No sensors registered — cannot fuse");
    }

    StateEstimate fused;
    fused.positionX   = 0.0;
    fused.positionY   = 0.0;
    fused.velocityX   = 0.0;
    fused.velocityY   = 0.0;
    fused.heading      = 0.0;
    fused.temperature  = 0.0;
    fused.timestamp    = 0.0;

    double totalWeight = 0.0;

    for (size_t i = 0; i < sensors.size(); ++i) {
        StateEstimate reading = sensors[i]->getLastReading();
        double w = weights[i];

        fused.positionX   += w * reading.positionX;
        fused.positionY   += w * reading.positionY;
        fused.velocityX   += w * reading.velocityX;
        fused.velocityY   += w * reading.velocityY;
        fused.heading      += w * reading.heading;
        fused.temperature  += w * reading.temperature;
        fused.timestamp    += w * reading.timestamp;

        totalWeight += w;
    }

    if (totalWeight <= 0.0) {
        throw std::runtime_error("Total weight is zero — cannot normalize");
    }

    // Normalize by total weight
    fused.positionX   /= totalWeight;
    fused.positionY   /= totalWeight;
    fused.velocityX   /= totalWeight;
    fused.velocityY   /= totalWeight;
    fused.heading      /= totalWeight;
    fused.temperature  /= totalWeight;
    fused.timestamp    /= totalWeight;

    fusedState = fused;
    return fusedState;
}

StateEstimate FusionSystem::getFusedState() const {
    return fusedState;
}
