#ifndef FUSION_SYSTEM_H
#define FUSION_SYSTEM_H

#include "Sensor.h"
#include "StateEstimate.h"
#include <vector>

/**
 * @brief Weighted-average sensor fusion engine.
 *
 * Registers multiple sensors, collects their latest readings, and computes
 * a single fused StateEstimate. Charles Wan — primary author.
 */
class FusionSystem {
private:
    std::vector<Sensor*> sensors;
    std::vector<double>  weights;
    StateEstimate        fusedState;
    double               predictionDt;

public:
    FusionSystem();

    void          addSensor(Sensor* sensor, double weight);
    StateEstimate fuseData();
    StateEstimate getFusedState() const;
};

#endif // FUSION_SYSTEM_H
