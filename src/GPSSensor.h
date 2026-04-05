#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include "Sensor.h"

/**
 * @brief GPS sensor subclass — generates noisy position data with signal dropout.
 *
 * Overrides generateReading() to perturb only positionX and positionY.
 * Simulates occasional signal loss via dropoutRate.
 * David (Jeongheon Han) — primary author.
 */
class GPSSensor : public Sensor {
private:
    double positionNoiseSigma;
    double positionBias;
    double dropoutRate;
    std::uniform_real_distribution<double> uniformDist;

public:
    GPSSensor(double posSigma, double posBias, double dropout, double rate);

    StateEstimate generateReading(const StateEstimate& trueState) override;
    void calibrate(double offset, double scale) override;
    std::string getType() const override;
};

#endif // GPS_SENSOR_H
