#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "Sensor.h"

/**
 * @brief IMU sensor subclass — generates noisy acceleration and heading data.
 *
 * Overrides generateReading() to perturb only velocity (as proxy for acceleration)
 * and heading fields. David (Jeongheon Han) — primary author.
 */
class IMUSensor : public Sensor {
private:
    double accelNoiseSigma;
    double gyroNoiseSigma;
    double accelBias;
    double gyroBias;

public:
    IMUSensor(double accelSigma, double gyroSigma,
              double accelBias, double gyroBias, double rate);

    StateEstimate generateReading(const StateEstimate& trueState) override;
    void calibrate(double offset, double scale) override;
    std::string getType() const override;
};

#endif // IMU_SENSOR_H
